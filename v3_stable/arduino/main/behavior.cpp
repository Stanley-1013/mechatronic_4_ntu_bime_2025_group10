// behavior.cpp - 行為控制模組 (純角度 PD 控制)
// 版本: 3.15
// 日期: 2025-12-11
//
// 核心邏輯：
// 1. 沿牆：純角度 PD 控制，距離影響目標角度
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
    _sweepTimer = 0;
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
        _frontTriggerCount = 0;  // v3.15: 清零，避免重複觸發
        _turnFromCorner = (sensor.rightFront < 20);  // 右前 <20cm = 角落

        // v3.14: 角落轉彎前先右擺清掃
        if (_turnFromCorner) {
            _isSweeping = true;
            _sweepPhase = SWEEP_RIGHT;
            _sweepTimer = 0;
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
// 流程：右擺30° → 回正 → 進入左轉
MotorCommand BehaviorController::_handleSweeping(const SensorData& sensor) {
    if (_imu == nullptr) {
        // 無 IMU，跳過擺頭
        _isSweeping = false;
        return _handleTurning(sensor);
    }

    _sweepTimer++;

    // 超時保護 (5秒)
    if (_sweepTimer > SWEEP_TIMEOUT) {
        _isSweeping = false;
        _sweepPhase = SWEEP_NONE;
        return _handleTurning(sensor);
    }

    float currentYaw = _imu->getYaw();
    float diff = currentYaw - _sweepStartYaw;
    // 處理跨越 ±180
    if (diff < -180) diff += 360;
    if (diff > 180) diff -= 360;

    switch (_sweepPhase) {
        case SWEEP_RIGHT:
            // 右轉直到 -30°
            if (diff <= -SWEEP_ANGLE) {
                _sweepPhase = SWEEP_BACK;
            }
            return {SWEEP_PWM, -SWEEP_PWM, false};  // 原地右轉

        case SWEEP_BACK:
            // 左轉回正 (diff ≈ 0)
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

// ===== 沿牆控制 (v3.15: 純角度 PD，距離影響目標角度) =====
MotorCommand BehaviorController::_handleWallFollow(const SensorData& sensor) {
    float angular = 0;

    if (sensor.rightValid) {
        // ===== v3.15: 純角度 PD 控制 =====
        // 距離誤差：正=太近
        float distError = TARGET_DIST - sensor.rightAvg;

        // 目標角度 = 距離誤差 × 係數
        // 太近(distError負) → 目標角度負 → 車頭朝外 → 遠離牆
        // 太遠(distError正) → 目標角度正 → 車頭朝內 → 靠近牆
        // 注意：distError = TARGET - actual，太近時 actual > TARGET，所以 distError < 0
        float targetAngle = -distError * KP_DIST;  // 取負號修正方向

        // 限幅
        if (targetAngle > MAX_TARGET_ANGLE) targetAngle = MAX_TARGET_ANGLE;
        if (targetAngle < -MAX_TARGET_ANGLE) targetAngle = -MAX_TARGET_ANGLE;

        // 角度誤差：當前角度 - 目標角度
        // 正=車頭比目標更朝牆，需左轉
        float angleError = sensor.angle - targetAngle;

        // 角度變化率 (D 項)
        float angleDerivative = sensor.angle - _lastAngle;
        _lastAngle = sensor.angle;

        // 純角度 PD 控制
        angular = KP_ANGLE * angleError + KD_ANGLE * angleDerivative;

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

    // v3.13: 不對稱修正 - 右修正(angular>0)時左輪加速 ×0.95，避免過衝
    float leftAngular = (angular > 0) ? angular * 0.95f : angular;
    int leftPWM = (int)(leftBase * (1.0f - leftAngular));
    int rightPWM = (int)(rightBase * (1.0f + angular));

    // v3.14: 完全無牆時額外右偏（右輪減速）
    if (!sensor.rightValid && sensor.rightFront > 40 && sensor.rightRear > 40) {
        rightPWM -= 5;
    }

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
