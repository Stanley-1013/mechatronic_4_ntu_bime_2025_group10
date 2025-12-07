// behavior.cpp - 行為控制模組 (距離+角度 PD 控制)
// 版本: 3.0
// 日期: 2025-12-07
//
// 核心邏輯：
// 1. 沿牆：距離 P + 角度 PD 控制
// 2. 轉彎：最少 400ms + 前方暢通 → 退出
// 3. 找牆：使用連續函數 (軟閾值) 避免邊界抖動
//
// v2.8: 修正角度計算符號 (後-前)
// v2.9: 調整無牆轉彎時間 600ms→400ms
// v3.0: 三項修正
//   - 轉彎退出簡化為純時間制 + 前方暢通
//   - 前方觸發需連續 2 次確認
//   - 沿牆加入角度 D 項控制

#include "behavior.h"
#include <Arduino.h>

void BehaviorController::init() {
    _isTurning = false;
    _turnTimer = 0;
    _stableTimer = 0;
    _complete = false;
    _startTime = millis();
    _lastRightFront = 50.0f;
    _frontTriggerCount = 0;
    _lastAngle = 0.0f;

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
            angular = KP_ANGLE * sensor.angle * 0.5f;  // 半強度角度修正
        }
        if (angular > 0.15f) angular = 0.15f;
        if (angular < -0.15f) angular = -0.15f;

        int leftPWM = (int)(BASE_PWM * (1.0f - angular));
        int rightPWM = (int)(BASE_PWM * (1.0f + angular));
        return {leftPWM, rightPWM, false};
    }

    // ===== 優先級 1: 角落轉彎 =====
    // 前方觸發連續確認 (防止雜訊誤觸發)
    if (sensor.front <= FRONT_STOP) {
        _frontTriggerCount++;
    } else {
        _frontTriggerCount = 0;
    }

    if (_isTurning || _frontTriggerCount >= 2) {
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

    // 轉彎完成條件（簡化版）：
    // 1. 最少轉彎時間 400ms (8 個週期)
    // 2. 前方暢通
    if (_turnTimer >= 8 && sensor.front > TURN_FRONT_CLEAR) {
        _isTurning = false;
        _stableTimer = TURN_STABLE;
        return _handleWallFollow(sensor);
    }

    // 繼續左轉
    return {-TURN_PWM, +TURN_PWM, false};
}

// ===== 沿牆控制 (距離+角度 PD 控制) =====
MotorCommand BehaviorController::_handleWallFollow(const SensorData& sensor) {
    float angular = 0;

    if (sensor.rightValid) {
        // ===== 距離 + 角度 PD 控制 =====
        // 距離誤差：正=太近，需左轉遠離
        float distError = TARGET_DIST - sensor.rightAvg;

        // 角度誤差：正=車頭朝牆，需左轉
        float angleError = sensor.angle - TARGET_ANGLE;

        // 角度變化率 (D 項)：正=角度增加中(越來越朝牆)，需加強左轉
        float angleDerivative = sensor.angle - _lastAngle;
        _lastAngle = sensor.angle;

        // PD 控制
        float distTerm = KP_DIST * distError;
        float angleTerm = KP_ANGLE * angleError + KD_ANGLE * angleDerivative;

        angular = distTerm + angleTerm;

    } else {
        // 右側無效 (rightValid=false，無法計算角度)
        // 使用連續函數避免邊界抖動

        // 計算各感測器的「近」程度 (0~1，越近越大)
        // 使用 sigmoid-like 軟閾值：30cm 以下為「近」
        float rfNear = _softThreshold(sensor.rightFront, 30, 10);  // 30±10cm 過渡
        float rrNear = _softThreshold(sensor.rightRear, 30, 10);

        // 右前遠 + 右後近 = 轉角/出口 → 右轉進入 (angular 負)
        float cornerWeight = (1.0f - rfNear) * rrNear;

        // 右前近 + 右後遠 = 發現牆前端 (正在接近新牆 or 入場)
        float wallFoundWeight = rfNear * (1.0f - rrNear);

        // 計算右前趨勢：正=接近牆，負=遠離牆
        float rfTrend = _lastRightFront - sensor.rightFront;  // 變小=接近

        // 發現牆時：依趨勢決定 (使用軟閾值避免抖動)
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

        // 完全無牆（右前遠 + 右後遠）：輕微右轉找牆
        float noWallWeight = (1.0f - rfNear) * (1.0f - rrNear);
        float noWallAngular = noWallWeight * (-SEARCH_ANGULAR);  // 負=右轉

        // 混合計算
        angular = cornerWeight * (-0.12f) + wallFoundAngular + noWallAngular;

        // 重置 D 項追蹤（無有效角度時）
        _lastAngle = 0.0f;
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
