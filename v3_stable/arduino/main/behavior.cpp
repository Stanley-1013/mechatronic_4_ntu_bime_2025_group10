// behavior.cpp - 行為控制模組 (距離+角度 PD 控制)
// 版本: 3.14
// 日期: 2025-12-11
//
// 核心邏輯：
// 1. 沿牆：距離 P + 角度 PD 控制
// 2. 擺頭：角落轉彎前原地左右擺動清掃 (v3.14)
// 3. 轉彎：IMU 判斷轉過 85~90 度 → 退出
// 4. 找牆：使用連續函數 (軟閾值) 避免邊界抖動

#include "behavior.h"
#include <Arduino.h>

void BehaviorController::init(MPU6050Sensor* imu) {
    _imu = imu;
    _isTurning = false;
    _isSweeping = false;
    _sweepPhase = SWEEP_NONE;
    _sweepStartYaw = 0.0f;
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

    // ===== 優先級 0: 擺頭清掃 (v3.14) =====
    if (_isSweeping) {
        return _handleSweeping(sensor);
    }

    // ===== 優先級 1: 轉彎 =====
    if (_isTurning) {
        return _handleTurning(sensor);
    }

    // ===== 前方觸發連續確認 =====
    if (sensor.front <= FRONT_STOP) {
        _frontTriggerCount++;
    } else {
        _frontTriggerCount = 0;
    }

    // 前方觸發 → 判斷是否為角落轉彎
    if (_frontTriggerCount >= 2) {
        _turnFromCorner = (sensor.rightFront < 20);  // 右前 <20cm = 角落

        // v3.14: 角落轉彎前先擺頭清掃
        if (_turnFromCorner) {
            _isSweeping = true;
            _sweepPhase = SWEEP_LEFT;
            if (_imu != nullptr) {
                _sweepStartYaw = _imu->getYaw();
            }
            return _handleSweeping(sensor);
        } else {
            // 非角落，直接轉彎
            return _handleTurning(sensor);
        }
    }

    // ===== 優先級 2: 沿牆 =====
    return _handleWallFollow(sensor);
}

// ===== 擺頭清掃控制 (v3.14) =====
// 流程：左擺30° → 回中 → 右擺30° → 回起點 → 進入轉彎
MotorCommand BehaviorController::_handleSweeping(const SensorData& sensor) {
    if (_imu == nullptr) {
        // 無 IMU，跳過擺頭
        _isSweeping = false;
        return _handleTurning(sensor);
    }

    float currentYaw = _imu->getYaw();
    float diff = currentYaw - _sweepStartYaw;
    // 處理跨越 ±180
    if (diff < -180) diff += 360;
    if (diff > 180) diff -= 360;

    switch (_sweepPhase) {
        case SWEEP_LEFT:
            // 左轉直到 +30°
            if (diff >= SWEEP_ANGLE) {
                _sweepPhase = SWEEP_BACK_CENTER;
            }
            return {-SWEEP_PWM, SWEEP_PWM, false};  // 原地左轉

        case SWEEP_BACK_CENTER:
            // 右轉回中 (diff ≈ 0)
            if (diff <= 2.0f && diff >= -2.0f) {
                _sweepPhase = SWEEP_RIGHT;
            }
            return {SWEEP_PWM, -SWEEP_PWM, false};  // 原地右轉

        case SWEEP_RIGHT:
            // 右轉直到 -30°
            if (diff <= -SWEEP_ANGLE) {
                _sweepPhase = SWEEP_BACK_START;
            }
            return {SWEEP_PWM, -SWEEP_PWM, false};  // 原地右轉

        case SWEEP_BACK_START:
            // 左轉回起點 (diff ≈ 0)
            if (diff >= -2.0f && diff <= 2.0f) {
                _sweepPhase = SWEEP_DONE;
            }
            return {-SWEEP_PWM, SWEEP_PWM, false};  // 原地左轉

        case SWEEP_DONE:
        default:
            // 擺頭完成，進入轉彎
            _isSweeping = false;
            _sweepPhase = SWEEP_NONE;
            return _handleTurning(sensor);
    }
}

// ===== 轉彎控制 (v3.7: 只讀 IMU，計算前後差值) =====
MotorCommand BehaviorController::_handleTurning(const SensorData& sensor) {
    if (!_isTurning) {
        _isTurning = true;
        _turnTimer = 0;
        _turnFromCorner = (sensor.rightFront < 20);  // 右前 <20cm = 角落
        // v3.7: 記錄轉彎開始時的 yaw，不做 reset
        if (_imu != nullptr) {
            _turnStartYaw = _imu->getYaw();
        }
    }

    _turnTimer++;

    // 超時保護
    if (_turnTimer > TURN_TIMEOUT) {
        _isTurning = false;
        return _handleWallFollow(sensor);
    }

    // 最少轉彎時間 (300ms)
    if (_turnTimer < TURN_MIN_TIME) {
        // v3.12: 單輪轉彎（左輪停，右輪動），更穩定
        return {0, TURN_PWM, false};
    }

    // v3.7: 計算前後差值（左轉 yaw 減少，所以 start - current = 正值）
    float turnedAngle = 0;
    if (_imu != nullptr) {
        float currentYaw = _imu->getYaw();
        turnedAngle = _turnStartYaw - currentYaw;
        // 處理跨越 ±180 的情況
        if (turnedAngle < -180) turnedAngle += 360;
        if (turnedAngle > 180) turnedAngle -= 360;
        // 取絕對值（不管左轉右轉）
        if (turnedAngle < 0) turnedAngle = -turnedAngle;
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

    // v3.12: 只看角度，不看前方（前方由 update() 優先級處理）
    if (atCheckpoint) {
        _isTurning = false;
        return _handleWallFollow(sensor);
    }

    // 繼續左轉 (v3.12: 單輪轉彎)
    return {0, TURN_PWM, false};
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

        // 右前近 + 右後遠 = 斜向靠近牆 → 左轉 (與 cornerWeight 對稱)
        float approachAngular = wallFoundWeight * 0.12f;  // 正=左轉

        // 混合計算 (v3.13: 移除 rfDistAngular，避免轉彎後過度修正)
        angular = cornerWeight * (-0.12f) + approachAngular + wallFoundAngular + noWallAngular;

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

    // v3.8: 左右輪獨立 base，方便調參
    int leftBase = (int)(BASE_PWM_L * speedScale);
    int rightBase = (int)(BASE_PWM_R * speedScale);

    // v3.13: 不對稱修正 - 右修正(angular>0)時左輪加速 ×0.9，避免過衝
    float leftAngular = (angular > 0) ? angular * 0.9f : angular;
    int leftPWM = (int)(leftBase * (1.0f - leftAngular));
    int rightPWM = (int)(rightBase * (1.0f + angular));

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
