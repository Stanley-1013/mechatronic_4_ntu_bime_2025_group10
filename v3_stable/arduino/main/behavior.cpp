// behavior.cpp - 行為控制模組 (距離+角度雙控制)
// 版本: 2.9
// 日期: 2025-12-07
//
// 核心邏輯：
// 1. 沿牆：距離誤差 + 角度誤差 → 雙 P 控制
// 2. 轉彎：前方暢通 + 與新牆平行 → 完成（無牆時 400ms 退出）
// 3. 找牆：使用連續函數 (軟閾值) 避免邊界抖動
//
// v2.8: 修正 ultrasonic.cpp 角度計算符號 (後-前 而非 前-後)
// v2.9: 根據實測調整無牆轉彎時間 600ms→400ms (實測角速度~225°/s)

#include "behavior.h"
#include <Arduino.h>

void BehaviorController::init() {
    _isTurning = false;
    _turnTimer = 0;
    _stableTimer = 0;
    _complete = false;
    _startTime = millis();
    _lastRightFront = 50.0f;  // 初始假設較遠

    wdt_enable(WDTO_2S);
}

MotorCommand BehaviorController::update(const SensorData& sensor) {
    wdt_reset();

    // 超時保護
    if (millis() - _startTime > RUN_TIMEOUT) {
        _complete = true;
        return {0, 0, true};
    }

    if (_complete) {
        return {0, 0, true};
    }

    // ===== 穩定期：轉彎後直走，不做大幅修正 =====
    if (_stableTimer > 0) {
        _stableTimer--;
        // 穩定期只做輕微角度修正
        float angular = 0;
        if (sensor.rightValid) {
            angular = KP_ANGLE * sensor.angle * 0.5f;  // 半強度角度修正 (與主邏輯一致)
        }
        if (angular > 0.15f) angular = 0.15f;
        if (angular < -0.15f) angular = -0.15f;

        int leftPWM = (int)(BASE_PWM * (1.0f - angular));
        int rightPWM = (int)(BASE_PWM * (1.0f + angular));
        return {leftPWM, rightPWM, false};
    }

    // ===== 優先級 1: 角落轉彎 =====
    if (_isTurning || sensor.front <= FRONT_STOP) {
        return _handleTurning(sensor);
    }

    // ===== 優先級 2: 沿牆 =====
    return _handleWallFollow(sensor);
}

// ===== 轉彎控制 =====
MotorCommand BehaviorController::_handleTurning(const SensorData& sensor) {
    if (!_isTurning) {
        _isTurning = true;
        _turnTimer = 0;
    }

    _turnTimer++;

    // 超時保護
    if (_turnTimer > TURN_TIMEOUT) {
        _isTurning = false;
        _stableTimer = TURN_STABLE;
        return _handleWallFollow(sensor);
    }

    // 最少轉彎時間
    if (_turnTimer < TURN_MIN_TIME) {
        return {-TURN_PWM, +TURN_PWM, false};
    }

    // 轉彎完成條件：
    // 1. 前方暢通
    // 2. 右側在合理範圍（或無效時前方暢通即可）
    // 3. 角度接近 0 (與新牆平行)
    bool frontClear = (sensor.front > TURN_FRONT_CLEAR);

    // 右側條件：有效時檢查範圍，無效時放寬（只要前方暢通）
    bool rightOK = false;
    if (sensor.rightValid) {
        rightOK = (sensor.rightAvg > TURN_RIGHT_MIN &&
                   sensor.rightAvg < TURN_RIGHT_MAX);
    } else {
        // 無有效右牆時，前方暢通 + 最少轉彎時間後可退出
        rightOK = (_turnTimer > 8);  // 400ms 後可考慮退出
    }

    // 平行檢查：
    // - 有效時：檢查角度
    // - 無效時：根據實測（1秒轉180-270°），400ms約轉90°
    bool aligned = false;
    if (sensor.rightValid) {
        aligned = (abs(sensor.angle) < TURN_ANGLE_TOL);
    } else {
        aligned = (_turnTimer > 8);  // 400ms 約轉 90° (實測角速度 ~225°/s)
    }

    if (frontClear && rightOK && aligned) {
        _isTurning = false;
        _stableTimer = TURN_STABLE;
        return _handleWallFollow(sensor);
    }

    // 繼續左轉
    return {-TURN_PWM, +TURN_PWM, false};
}

// ===== 沿牆控制 (距離+角度雙 P 控制) =====
MotorCommand BehaviorController::_handleWallFollow(const SensorData& sensor) {
    float angular = 0;

    if (sensor.rightValid) {
        // ===== 距離 + 角度雙控制 =====
        // 距離誤差：正=太近，需左轉遠離
        // rightAvg 越大 = 離牆越遠，所以用 TARGET - rightAvg
        float distError = TARGET_DIST - sensor.rightAvg;

        // 角度誤差：正=車頭朝牆，需左轉
        // (sensor.angle 正=車頭朝牆，TARGET_ANGLE=0)
        float angleError = sensor.angle - TARGET_ANGLE;

        // P 控制
        float distTerm = KP_DIST * distError;
        float angleTerm = KP_ANGLE * angleError;

        // 標準雙 P 控制
        // 角度定義已修正：正角度=車頭朝牆，負角度=車尾朝牆
        // angleTerm 正=左轉，負=右轉，與預期一致
        angular = distTerm + angleTerm;

    } else {
        // 右側無效 (rightValid=false，無法計算角度)
        // 使用連續函數避免邊界抖動

        // 計算各感測器的「近」程度 (0~1，越近越大)
        // 使用 sigmoid-like 軟閾值：30cm 以下為「近」
        float rfNear = _softThreshold(sensor.rightFront, 30, 10);  // 30±10cm 過渡
        float rrNear = _softThreshold(sensor.rightRear, 30, 10);
        float frontClear = 1.0f - _softThreshold(sensor.front, FRONT_SLOW, 10);

        // 右前遠 + 右後近 = 轉角/出口 → 右轉 (angular 負)
        float cornerWeight = (1.0f - rfNear) * rrNear;

        // 右前近 + 右後遠 = 發現牆前端 (正在接近新牆 or 入場)
        float wallFoundWeight = rfNear * (1.0f - rrNear);

        // 計算右前趨勢：正=接近牆，負=遠離牆
        float rfTrend = _lastRightFront - sensor.rightFront;  // 變小=接近

        // 發現牆時：依趨勢決定 (使用軟閾值避免抖動)
        // - 接近牆 (rfTrend > 0)：左修正避免撞牆
        // - 遠離牆 (rfTrend < 0)：右修正靠近牆
        // 閾值 2cm 考慮超音波雜訊，1-3cm 為過渡區
        float trendStrength = 0;
        if (rfTrend > 3.0f) {
            trendStrength = 1.0f;  // 明確接近
        } else if (rfTrend > 1.0f) {
            trendStrength = (rfTrend - 1.0f) / 2.0f;  // 0~1 線性過渡
        } else if (rfTrend < -3.0f) {
            trendStrength = -1.0f;  // 明確遠離
        } else if (rfTrend < -1.0f) {
            trendStrength = (rfTrend + 1.0f) / 2.0f;  // 0~-1 線性過渡
        }
        float trendAngular = trendStrength * 0.08f;

        // 加上基於距離的修正：右前太近也要左修正
        float distAngular = (sensor.rightFront < 20) ? 0.05f : 0;

        float wallFoundAngular = wallFoundWeight * (trendAngular + distAngular);

        // 完全無牆：輕微右弧線
        float noWallWeight = (1.0f - rfNear) * (1.0f - rrNear);
        float noWallAngular = noWallWeight * (-SEARCH_ANGULAR);

        // 混合計算
        angular = cornerWeight * (-0.12f) + wallFoundAngular + noWallAngular;
    }

    // 更新趨勢追蹤（每週期都更新，確保切換時有正確的歷史值）
    _lastRightFront = sensor.rightFront;

    // 前方減速
    float speedScale = 1.0f;
    if (sensor.front < FRONT_SLOW) {
        speedScale = 0.5f + 0.5f * (sensor.front / FRONT_SLOW);
        if (speedScale < 0.5f) speedScale = 0.5f;
    }

    // 限幅
    if (angular > MAX_ANGULAR) angular = MAX_ANGULAR;
    if (angular < -MAX_ANGULAR) angular = -MAX_ANGULAR;

    // 轉換 PWM
    int basePWM = (int)(BASE_PWM * speedScale);
    int leftPWM = (int)(basePWM * (1.0f - angular));
    int rightPWM = (int)(basePWM * (1.0f + angular));

    // 馬達校正
    leftPWM = (int)(leftPWM * LEFT_SCALE);
    rightPWM = (int)(rightPWM * RIGHT_SCALE);

    // 限幅
    if (leftPWM > 0 && leftPWM < MIN_PWM) leftPWM = MIN_PWM;
    if (rightPWM > 0 && rightPWM < MIN_PWM) rightPWM = MIN_PWM;
    if (leftPWM > 255) leftPWM = 255;
    if (rightPWM > 255) rightPWM = 255;

    return {leftPWM, rightPWM, false};
}

// ===== 軟閾值函數 =====
// 返回 0~1，value 越小返回值越接近 1
// threshold = 中心點，width = 過渡帶寬度
float BehaviorController::_softThreshold(float value, float threshold, float width) {
    if (value <= threshold - width) return 1.0f;
    if (value >= threshold + width) return 0.0f;
    // 線性過渡
    return 0.5f - 0.5f * (value - threshold) / width;
}
