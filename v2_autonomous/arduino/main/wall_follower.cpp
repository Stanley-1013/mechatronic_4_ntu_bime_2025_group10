#include "wall_follower.h"
#include "config.h"

/**
 * WallFollower - 連續差動控制沿牆
 * 版本: 2.0 (IMU 角度鎖定)
 *
 * 所有可調參數已移至 config.h
 */

// ==================== 可調參數全域變數 (可由 Pi 透過 CMD_SET_PARAMS 修改) ====================
// 距離閾值 (cm)
uint8_t g_targetRightDist = DEFAULT_TARGET_RIGHT_DIST;
uint8_t g_frontStopDist = DEFAULT_FRONT_STOP_DIST;
uint8_t g_frontSlowDist = DEFAULT_FRONT_SLOW_DIST;
uint8_t g_cornerRightDist = DEFAULT_CORNER_RIGHT_DIST;

// 速度參數 (-1.0 ~ +1.0)
float g_baseLinearSpeed = DEFAULT_BASE_LINEAR_SPEED;
float g_backupSpeed = DEFAULT_BACKUP_SPEED;
float g_turnAngularSpeed = DEFAULT_TURN_ANGULAR_SPEED;
float g_findWallLinear = DEFAULT_FIND_WALL_LINEAR;
float g_leftMotorScale = DEFAULT_LEFT_MOTOR_SCALE;
float g_rightMotorScale = DEFAULT_RIGHT_MOTOR_SCALE;

// PID 參數
static float g_wallFollowKp = DEFAULT_KP;
static float g_wallFollowKi = DEFAULT_KI;
static float g_wallFollowKd = DEFAULT_KD;

// PWM 參數
uint8_t g_minEffectivePWM = DEFAULT_MIN_EFFECTIVE_PWM;

// 角落轉彎角度 (度)
float g_cornerTurnAngle = DEFAULT_CORNER_TURN_ANGLE;
float g_redAvoidAngle = DEFAULT_RED_AVOID_ANGLE;

// Bang-Bang 沿牆控制
float g_gentleAngular = DEFAULT_GENTLE_ANGULAR;
float g_errorDeadzone = DEFAULT_ERROR_DEADZONE;

// 航向 PID 參數 (IMU 控制)
float g_yawKp = DEFAULT_YAW_KP;
float g_yawKi = DEFAULT_YAW_KI;
float g_yawKd = DEFAULT_YAW_KD;

// 時間參數 (ms)
uint16_t g_backupDurationMs = DEFAULT_BACKUP_DURATION_MS;
uint16_t g_turnTimeoutMs = DEFAULT_TURN_TIMEOUT_MS;

// ==================== 固定常數 ====================
static const int MAX_PWM = 255;
static const int SENSOR_INVALID = 0;
static const int SENSOR_MAX_RANGE = 500;

// ==================== 建構子 ====================

WallFollower::WallFollower()
    : _running(false),
      _cornerCount(0),
      _vacuumOn(true),
      _cornerLocked(false),
      _isRedAvoid(false),
      _backupStartTime(0),
      _backupYaw(0),
      _startupGracePeriod(0),
      _historyIndex(0),
      _leftPWM(0),
      _rightPWM(0),
      _lastError(0),
      _integral(0),
      _lastUpdateTime(0),
      _targetYaw(0),
      _yawLocked(false),
      _driftTimer(0),
      _lastTurnYaw(0) {
    // 初始化感測器歷史 (用 -1 表示無效)
    for (int i = 0; i < 3; i++) {
        _frontHistory[i] = -1;
        _rightHistory[i] = -1;
    }
}

// ==================== 公開介面 ====================

void WallFollower::start() {
    _running = true;
    _cornerCount = 0;
    _cornerLocked = false;
    _isRedAvoid = false;
    _backupStartTime = 0;
    _backupYaw = 0;
    _startupGracePeriod = millis();  // 啟動保護期

    // 重置 PID 狀態
    _lastError = 0;
    _integral = 0;
    _lastUpdateTime = millis();

    // IMU 航向鎖定 (會在第一次 update 時設定)
    _targetYaw = 0;
    _yawLocked = false;
    _driftTimer = 0;
    _lastTurnYaw = 0;

    // 重置感測器歷史
    for (int i = 0; i < 3; i++) {
        _frontHistory[i] = -1;
        _rightHistory[i] = -1;
    }
}

void WallFollower::stop() {
    _running = false;
    _leftPWM = 0;
    _rightPWM = 0;
}

void WallFollower::reset() {
    _running = false;
    _cornerCount = 0;
    _cornerLocked = false;
    _isRedAvoid = false;
    _backupStartTime = 0;
    _backupYaw = 0;
    _startupGracePeriod = 0;
    _leftPWM = 0;
    _rightPWM = 0;

    // 重置 PID 狀態
    _lastError = 0;
    _integral = 0;
    _lastUpdateTime = 0;

    // 重置 IMU 航向控制
    _targetYaw = 0;
    _yawLocked = false;
    _driftTimer = 0;
    _lastTurnYaw = 0;

    for (int i = 0; i < 3; i++) {
        _frontHistory[i] = -1;
        _rightHistory[i] = -1;
    }
}

void WallFollower::triggerAvoidRed() {
    /**
     * 觸發紅色迴避
     *
     * 策略: 小角度左轉閃避，繼續吸塵
     * - 不後退，直接左轉
     * - 只轉 45° (比角落的 85° 小)
     * - 轉完後會自動尋牆回到沿牆狀態
     */
    if (!_running || _cornerLocked) {
        return;  // 未執行或已在角落處理中，忽略
    }

    _cornerLocked = true;
    _isRedAvoid = true;  // 標記為紅色迴避，使用較小角度
    _backupStartTime = millis() - BACKUP_DURATION_MS;  // 跳過後退階段
    _backupYaw = 0;  // 會在下次 update 時用當前 yaw 更新
    // 注意: 紅色迴避不增加 cornerCount
}

void WallFollower::update(int frontDist, int rightDist, float yaw, bool imuValid) {
    // 未執行時，輸出為 0
    if (!_running) {
        _leftPWM = 0;
        _rightPWM = 0;
        return;
    }

    unsigned long currentTime = millis();

    // 0. 啟動保護期 (500ms) - 忽略可能的電壓突波干擾
    const unsigned long STARTUP_GRACE_MS = 500;
    if (currentTime - _startupGracePeriod < STARTUP_GRACE_MS) {
        _setMotorOutput(0.2, 0);  // 低速前進
        return;
    }

    // 1. 感測器濾波
    int front = _filterSensor(frontDist, _frontHistory);
    int right = _filterSensor(rightDist, _rightHistory);

    // 1.5 感測器有效性檢查
    if (front < 5 && right < 5) {
        _setMotorOutput(0, 0);
        Serial.println(F("[WF] Sensor anomaly detected, stopping"));
        return;
    }

    // 2. 計算時間差 (用於 PID)
    unsigned long now = millis();
    float dt = (now - _lastUpdateTime) / 1000.0;
    if (dt < 0.01 || dt > 0.2) dt = 0.02;  // 異常時使用預設值
    _lastUpdateTime = now;

    // ============================================================
    // 行為融合 (Behavior Blending) - 平滑連續控制
    // ============================================================
    //
    // 三個行為同時計算，用權重混合：
    // 1. 避障行為：前方有障礙時左轉
    // 2. 沿牆行為：PID 控制右側距離
    // 3. 尋牆行為：右側太遠時右轉尋牆
    //
    // linear 根據前方距離連續衰減
    // angular = w_obstacle * 避障 + w_wall * 沿牆 + w_seek * 尋牆
    // ============================================================

    // --- 行為 1: 前方障礙避讓 ---
    // 根據前方距離計算避障強度 (0~1)
    float obstacleWeight = 0.0;
    float obstacleAngular = 0.0;

    if (front < FRONT_SLOW_DIST) {
        // 前方有障礙，計算避障權重 (越近越強)
        obstacleWeight = _mapRange(front, FRONT_STOP_DIST, FRONT_SLOW_DIST, 1.0, 0.0);
        obstacleWeight = constrain(obstacleWeight, 0.0, 1.0);
        obstacleAngular = TURN_ANGULAR_SPEED;  // 左轉

        // 如果非常近，增加角落計數並記錄轉彎起始角度
        if (front < FRONT_STOP_DIST && !_cornerLocked) {
            _cornerLocked = true;  // 用作「已計數」標記
            _cornerCount++;
            _lastTurnYaw = yaw;    // 記錄轉彎起始航向
        }
    } else {
        // 前方安全，檢查轉彎是否完成
        if (_cornerLocked && imuValid) {
            float turnedAngle = _angleDiff(yaw, _lastTurnYaw);
            if (turnedAngle > 70.0) {
                // 轉彎完成，重置航向
                _targetYaw = yaw;
                _yawLocked = true;
                _driftTimer = 0;
            }
        }
        _cornerLocked = false;
    }

    // --- 行為 2: 沿牆控制 (緩慢修正，像正常開車) ---
    float wallWeight = 0.0;
    float wallAngular = 0.0;

    // 有效範圍：2cm ~ 100cm
    if (right >= 2 && right < 100) {
        wallWeight = 1.0;
        float error = right - TARGET_RIGHT_DIST;  // 正=太遠, 負=太近

        if (abs(error) < ERROR_DEADZONE) {
            // 死區內：直走
            wallAngular = 0;
        }
        else if (error > 0) {
            // 太遠：固定緩慢右轉靠近牆
            wallAngular = -GENTLE_ANGULAR;
        }
        else {
            // 太近：固定緩慢左轉遠離牆
            wallAngular = GENTLE_ANGULAR;
        }

        // 不用 PID，重置積分
        _integral = 0;
        _lastError = 0;
    } else {
        // 無效範圍，直走
        _integral = 0;
        _lastError = 0;
    }

    // --- 融合計算 ---
    // 線性速度：根據前方距離連續衰減
    float linear = BASE_LINEAR_SPEED;
    if (front < FRONT_SLOW_DIST) {
        linear = _mapRange(front, 5, FRONT_SLOW_DIST, 0.0, BASE_LINEAR_SPEED);
        linear = constrain(linear, 0.0, BASE_LINEAR_SPEED);
    }

    // 角速度：避障 + 沿牆修正（二選一：IMU 優先，無 IMU 時用超聲波）
    float angular = 0.0;
    angular += obstacleWeight * obstacleAngular;

    // ============================================================
    // IMU 航向控制 vs 超聲波沿牆控制（互斥，不疊加）
    // ============================================================

    // 1. 只有當右側距離接近目標時才鎖定航向（表示已經找到牆並靠近）
    //    避免在「尋牆」階段就鎖定，導致車子直走不靠近牆
    float distError = abs(right - TARGET_RIGHT_DIST);
    bool nearTargetDist = (right >= 2 && right < 100 && distError < ERROR_DEADZONE * 2);

    if (!_yawLocked && imuValid && nearTargetDist) {
        _targetYaw = yaw;
        _yawLocked = true;
    }

    // 2. IMU 有效時用 IMU 控制，否則用超聲波 Bang-Bang
    if (imuValid && _yawLocked && obstacleWeight < 0.5) {
        // === IMU 航向控制 ===
        float yawError = _normalizeAngle(yaw - _targetYaw);
        yawError = constrain(yawError, -30.0f, 30.0f);

        float yawCorrection = g_yawKp * yawError;
        yawCorrection = constrain(yawCorrection, -ANGULAR_MAX, ANGULAR_MAX);

        // IMU 修正取代超聲波（不疊加）
        angular += yawCorrection * (1.0 - obstacleWeight);
    } else {
        // === 超聲波 Bang-Bang 控制（備援）===
        angular += wallWeight * (1.0 - obstacleWeight) * wallAngular;
    }

    // ============================================================
    // 超聲波監督修正 (Phase 2)
    // ============================================================
    // 連續偏離超過 2 秒才修正，防止噪音干擾

    bool isTurning = (abs(angular) > 0.3) || _cornerLocked;

    if (!isTurning && imuValid && _yawLocked && right >= 2 && right < 100) {
        float error = right - TARGET_RIGHT_DIST;

        if (abs(error) > ERROR_DEADZONE) {
            _driftTimer += dt;

            if (_driftTimer > DRIFT_THRESHOLD_TIME) {
                // 連續偏離超過閾值，慢速修正目標航向
                if (error > 0) {
                    _targetYaw -= YAW_DRIFT_RATE * dt;  // 太遠，右轉
                } else {
                    _targetYaw += YAW_DRIFT_RATE * dt;  // 太近，左轉
                }
            }
        } else {
            _driftTimer = 0;  // 在死區內，重置計時器
        }
    } else if (!isTurning) {
        _driftTimer = 0;  // 非監督狀態，重置計時器
    }

    // 限制角速度：區分轉彎模式和直行模式
    if (_cornerLocked) {
        // 轉彎模式：允許大角速度（原地轉）
        angular = constrain(angular, -ANGULAR_MAX, ANGULAR_MAX);
    } else {
        // 直行模式：限制在 linear 的 30%，確保兩輪速度差異小
        float maxAngular = linear * 0.3f;
        angular = constrain(angular, -maxAngular, maxAngular);
    }

    _setMotorOutput(linear, angular);
}

int WallFollower::getLeftPWM() {
    return _leftPWM;
}

int WallFollower::getRightPWM() {
    return _rightPWM;
}

bool WallFollower::getVacuumState() {
    return _vacuumOn;
}

WallFollowerState WallFollower::getState() {
    if (_running) {
        return WF_RUNNING;
    }
    return WF_IDLE;
}

uint8_t WallFollower::getCornerCount() {
    return _cornerCount;
}

bool WallFollower::isCornerLocked() {
    return _cornerLocked;
}

// ==================== 輔助函數 ====================

int WallFollower::_filterSensor(int value, int* history) {
    /**
     * 中值濾波 - 取 3 個值的中位數
     *
     * 策略:
     * - 只接受有效值 (0 < value < 500)
     * - 歷史中用 -1 表示無效
     * - 回傳有效值的中位數，或原始值
     */

    // 接受有效讀數
    if (value > SENSOR_INVALID && value < SENSOR_MAX_RANGE) {
        // 移動歷史
        history[0] = history[1];
        history[1] = history[2];
        history[2] = value;
    }

    // 收集有效值
    int valid[3];
    int validCount = 0;
    for (int i = 0; i < 3; i++) {
        if (history[i] > 0) {
            valid[validCount++] = history[i];
        }
    }

    // 無有效歷史，傳回原值或最大範圍
    if (validCount == 0) {
        return (value > SENSOR_INVALID && value < SENSOR_MAX_RANGE) ? value : SENSOR_MAX_RANGE;
    }

    // 排序取中位數
    for (int i = 0; i < validCount - 1; i++) {
        for (int j = i + 1; j < validCount; j++) {
            if (valid[i] > valid[j]) {
                int temp = valid[i];
                valid[i] = valid[j];
                valid[j] = temp;
            }
        }
    }

    return valid[validCount / 2];
}

float WallFollower::_angleDiff(float currentYaw, float startYaw) {
    /**
     * 計算轉了多少度 (絕對值)
     *
     * 處理 360° 環繞
     */

    float diff = currentYaw - startYaw;

    // 正規化到 [-180, 180]
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    // 傳回絕對值
    return abs(diff);
}

void WallFollower::_setMotorOutput(float linear, float angular) {
    /**
     * 差速公式:
     * left  = (linear - angular) * MAX_PWM * leftScale
     * right = (linear + angular) * MAX_PWM * rightScale
     *
     * angular > 0 -> 左轉 (right 快)
     * angular < 0 -> 右轉 (left 快)
     *
     * leftScale/rightScale 用於補償左右輪馬達差異
     */

    float leftBase = linear - angular;
    float rightBase = linear + angular;

    int leftPWM = (int)(leftBase * MAX_PWM * LEFT_MOTOR_SCALE);
    int rightPWM = (int)(rightBase * MAX_PWM * RIGHT_MOTOR_SCALE);

    _leftPWM = _constrainPWM(leftPWM);
    _rightPWM = _constrainPWM(rightPWM);
}

int WallFollower::_constrainPWM(int pwm) {
    // 限制範圍，不設死區，讓控制更平滑
    // 低 PWM 時馬達可能不轉，但不會有硬切換
    return constrain(pwm, -MAX_PWM, MAX_PWM);
}

float WallFollower::_mapRange(float x, float inMin, float inMax, float outMin, float outMax) {
    if (inMax == inMin) return outMin;
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float WallFollower::_normalizeAngle(float angle) {
    /**
     * 正規化角度到 [-180, 180] 範圍
     * 用於計算航向誤差時正確處理 360° 環繞
     */
    angle = fmod(angle + 180.0f, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle - 180.0f;
}

// ==================== PID 參數設定 ====================

void WallFollower::setPID(float kp, float ki, float kd) {
    g_wallFollowKp = kp;
    g_wallFollowKi = ki;
    g_wallFollowKd = kd;
    // 重置積分避免參數切換時的異常
    _integral = 0;
    _lastError = 0;
}

void WallFollower::setParams(
    uint8_t targetRightDist,
    uint8_t frontStopDist,
    uint8_t frontSlowDist,
    uint8_t cornerRightDist,
    float baseLinearSpeed,
    float backupSpeed,
    float turnAngularSpeed,
    float findWallLinear,
    float leftMotorScale,
    float rightMotorScale,
    float kp,
    float ki,
    float kd,
    uint8_t minEffectivePWM,
    float cornerTurnAngle,
    float redAvoidAngle,
    uint16_t backupDurationMs,
    uint16_t turnTimeoutMs
) {
    // 更新距離閾值
    g_targetRightDist = targetRightDist;
    g_frontStopDist = frontStopDist;
    g_frontSlowDist = frontSlowDist;
    g_cornerRightDist = cornerRightDist;

    // 更新速度參數
    g_baseLinearSpeed = baseLinearSpeed;
    g_backupSpeed = backupSpeed;
    g_turnAngularSpeed = turnAngularSpeed;
    g_findWallLinear = findWallLinear;
    g_leftMotorScale = leftMotorScale;
    g_rightMotorScale = rightMotorScale;

    // 更新 PID 參數
    g_wallFollowKp = kp;
    g_wallFollowKi = ki;
    g_wallFollowKd = kd;

    // 更新 PWM 參數
    g_minEffectivePWM = minEffectivePWM;

    // 更新角度參數
    g_cornerTurnAngle = cornerTurnAngle;
    g_redAvoidAngle = redAvoidAngle;

    // 更新時間參數
    g_backupDurationMs = backupDurationMs;
    g_turnTimeoutMs = turnTimeoutMs;

    // 重置 PID 積分 (避免參數切換時的異常)
    _integral = 0;
    _lastError = 0;
}
