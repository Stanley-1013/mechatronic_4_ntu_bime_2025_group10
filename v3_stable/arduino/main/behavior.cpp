// behavior.cpp - 行為控制模組 (距離+角度 PD 控制)
// 版本: 3.5
// 日期: 2025-12-08
//
// 核心邏輯：
// 1. 沿牆：距離 P + 角度 PD 控制
// 2. 轉彎：IMU 判斷轉過 85~90 度 + 前方暢通 → 退出
// 3. 找牆：使用連續函數 (軟閾值) 避免邊界抖動
//
// v2.8: 修正角度計算符號 (後-前)
// v2.9: 調整無牆轉彎時間 600ms→400ms
// v3.0: 三項修正
//   - 轉彎退出簡化為純時間制 + 前方暢通
//   - 前方觸發需連續 2 次確認
//   - 沿牆加入角度 D 項控制
// v3.1: 三項修正
//   - 穩定期內重置 _frontTriggerCount（防止連續轉彎）
//   - 右前 <15cm 漸進式左修正（防止斜向撞牆）
//   - 新增 approachAngular 對稱左轉（右前近+右後遠）
// v3.2: 兩項修正
//   - KD_ANGLE 0.015→0.012 減少正常沿牆時的車頭晃動
//   - rightAvg <10cm 時漸進加強 distTerm（緊急距離保護）
// v3.3: 兩項修正
//   - rfDistAngular 線性→拋物線（0.0015*x²，更強的近距離修正）
//   - 角落轉彎補償暫時停用
// v3.4: IMU 轉彎判定
//   - 左轉時用 IMU 判斷角度，轉過 85~90 度 + 前方暢通即退出
// v3.5: IMU 修正
//   - 改為轉彎時才讀 IMU（resetYaw 歸零 + 轉彎中呼叫 update）
//   - 非轉彎時不更新 IMU，避免積分誤差累積
// v3.6: IMU 獨立更新
//   - main.ino 50Hz 更新 IMU，behavior 只讀值
//   - 轉彎開始時 resetYaw() 歸零

#include "behavior.h"
#include <Arduino.h>

void BehaviorController::init(MPU6050Sensor* imu) {
    _imu = imu;
    _isTurning = false;
    _turnTimer = 0;
    _stableTimer = 0;
    _complete = false;
    _startTime = millis();
    _lastRightFront = 50.0f;
    _frontTriggerCount = 0;
    _lastAngle = 0.0f;
    _turnFromCorner = false;
    _turnStartYaw = 0.0f;

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
        _frontTriggerCount = 0;  // v3.1: 防止穩定期結束後立刻再轉彎
        // 穩定期只做輕微角度修正
        float angular = 0;
        if (sensor.rightValid) {
            angular = KP_ANGLE * sensor.angle * 0.5f;  // 半強度角度修正
            // v3.3: 角落轉彎補償暫時停用測試
            // if (_turnFromCorner) {
            //     angular -= SEARCH_ANGULAR;  // 負 = 右轉靠近牆
            // }
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

// ===== 轉彎控制 (v3.5: 轉彎時才更新 IMU) =====
MotorCommand BehaviorController::_handleTurning(const SensorData& sensor) {
    if (!_isTurning) {
        _isTurning = true;
        _turnTimer = 0;
        _turnFromCorner = (sensor.rightFront < 20);  // 右前 <20cm = 角落
        // v3.5: 重置 yaw 為 0，之後 getYaw() 直接就是已轉角度
        if (_imu != nullptr) {
            _imu->resetYaw();  // 同時重置時間戳
        }
    }

    _turnTimer++;

    // v3.6: IMU 由 main.ino 獨立 50Hz 更新，這裡只讀值

    // 超時保護
    if (_turnTimer > TURN_TIMEOUT) {
        _isTurning = false;
        _stableTimer = TURN_STABLE;
        return _handleWallFollow(sensor);
    }

    // 最少轉彎時間 (300ms)
    if (_turnTimer < TURN_MIN_TIME) {
        return {-TURN_PWM, +TURN_PWM, false};
    }

    // v3.5: 直接取 yaw（已從 0 開始累積）
    // 左轉時 yaw 會變成負值，取絕對值
    float turnedAngle = 0;
    if (_imu != nullptr) {
        turnedAngle = -_imu->getYaw();  // 左轉 yaw 為負，取反得正值
    }

    // DEBUG: 每 10 個週期輸出一次
    if (_turnTimer % 10 == 0) {
        Serial.print("TURN: angle=");
        Serial.print(turnedAngle);
        Serial.print(" front=");
        Serial.print(sensor.front);
        Serial.print(" chk=");
        Serial.println((turnedAngle >= 85.0f && sensor.front > TURN_FRONT_CLEAR) ? "Y" : "N");
    }

    // 轉彎完成條件：
    // - 第一次檢查點在 85° (接近 90°)
    // - 之後每 45° 檢查一次 (135°, 180°, ...)
    // - 且前方暢通
    bool atCheckpoint = false;
    if (turnedAngle >= 85.0f) {
        // 第一個檢查點
        if (turnedAngle < 95.0f) {
            atCheckpoint = true;
        } else {
            // 之後的檢查點：130±5, 175±5, 220±5...
            float angleAfter90 = turnedAngle - 90.0f;
            int stepNum = (int)(angleAfter90 / 45.0f);
            float stepProgress = angleAfter90 - (stepNum * 45.0f);
            // 在每個 45° 步的 35~45 度區間檢查 (即 85°, 130°, 175°...)
            if (stepProgress >= 35.0f && stepProgress <= 50.0f) {
                atCheckpoint = true;
            }
        }
    }

    if (atCheckpoint && sensor.front > TURN_FRONT_CLEAR) {
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

        // v3.2: 緊急距離加強 - rightAvg < 10cm 時漸進加強 distTerm
        float urgencyScale = 1.0f;
        if (sensor.rightAvg < 10) {
            urgencyScale = 1.0f + (10.0f - sensor.rightAvg) * 0.15f;
            // 效果：10cm→1.0, 5cm→1.75, 2cm→2.2
        }

        // 角度誤差：正=車頭朝牆，需左轉
        float angleError = sensor.angle - TARGET_ANGLE;

        // 角度變化率 (D 項)：正=角度增加中(越來越朝牆)，需加強左轉
        float angleDerivative = sensor.angle - _lastAngle;
        _lastAngle = sensor.angle;

        // PD 控制
        float distTerm = KP_DIST * distError * urgencyScale;
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

        // 基於距離的修正：右前太近也要左修正
        float distAngular = (sensor.rightFront < 20) ? 0.05f : 0;

        float wallFoundAngular = wallFoundWeight * (trendAngular + distAngular);

        // 完全無牆（右前遠 + 右後遠）：輕微右轉找牆
        float noWallWeight = (1.0f - rfNear) * (1.0f - rrNear);
        float noWallAngular = noWallWeight * (-SEARCH_ANGULAR);  // 負=右轉

        // v3.3: 右前距離保護（15cm 內啟動，拋物線：越近修正越強）
        float rfDistAngular = 0;
        if (sensor.rightFront < 15) {
            float x = 15.0f - sensor.rightFront;
            rfDistAngular = 0.0015f * x * x;  // 二次函數
            if (rfDistAngular > 0.20f) rfDistAngular = 0.20f;  // 限幅
        }
        // 效果：15cm→0, 10cm→0.04, 7cm→0.10, 5cm→0.15, 2cm→0.20

        // 右前近 + 右後遠 = 斜向靠近牆 → 左轉 (與 cornerWeight 對稱)
        float approachAngular = wallFoundWeight * 0.12f;  // 正=左轉

        // 混合計算
        angular = cornerWeight * (-0.12f) + approachAngular + wallFoundAngular + noWallAngular + rfDistAngular;

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
