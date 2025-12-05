#include "wall_follower.h"

// ==================== 常數定義 ====================

// 沿牆參數 (參考 Python v3.0)
static const int TARGET_RIGHT_DISTANCE = 15;        // 目標右側距離 (cm)
static const int FRONT_STOP_DISTANCE = 15;          // 前方停止距離 (cm) - 到角落
static const int FRONT_SLOW_DISTANCE = 40;          // 前方減速距離 (cm)

// 速度參數 (normalized: -1.0 ~ +1.0)
static const float BASE_LINEAR_SPEED = 0.6;         // 基礎線性速度
static const float SLOW_LINEAR_SPEED = 0.35;        // 減速時線性速度
static const float BACKUP_SPEED = -0.4;             // 後退速度
static const float TURN_ANGULAR_SPEED = 0.7;        // 轉彎角速度 (原地左轉)
static const float WALL_FOLLOW_KP = 0.025;          // 沿牆 P 控制增益

// 時間常數 (ms -> s 轉換時使用)
static const unsigned long BACKUP_DURATION_MS = 300;     // 後退持續時間 (ms)
static const unsigned long TURN_DURATION_MS = 800;       // 轉彎持續時間 (ms)
static const unsigned long FIND_WALL_TIMEOUT_MS = 3000;  // 尋牆超時 (ms)

// PWM 常數
static const int MAX_PWM = 255;                     // 最大 PWM 值
static const int MIN_EFFECTIVE_PWM = 60;            // 最小有效 PWM (死區)

// 角度控制常數
static const float YAW_TOLERANCE = 5.0;             // 角度誤差容許 (度)
static const float TURN_TARGET_YAW_OFFSET = -90.0;  // 左轉目標偏移 (度)

// ==================== 建構子 ====================

WallFollower::WallFollower()
    : _state(WF_IDLE),
      _cornerCount(0),
      _vacuumOn(true),
      _stateStartTime(0),
      _currentTime(0),
      _targetYaw(0),
      _imuTurnActive(false),
      _leftPWM(0),
      _rightPWM(0) {
    // 初始化
}

// ==================== 公開介面 ====================

void WallFollower::start() {
    _state = WF_FIND_WALL;
    _stateStartTime = millis();
    _currentTime = millis();
    _cornerCount = 0;
}

void WallFollower::stop() {
    _state = WF_DONE;
    _leftPWM = 0;
    _rightPWM = 0;
}

void WallFollower::triggerAvoidRed() {
    // 預留介面，待紅色迴避邏輯實作
}

void WallFollower::update(int frontDist, int rightDist, float yaw, bool imuValid) {
    _currentTime = millis();

    // 處理停止狀態
    if (_state == WF_DONE || _state == WF_IDLE) {
        _leftPWM = 0;
        _rightPWM = 0;
        return;
    }

    // 狀態機主邏輯
    switch (_state) {
        case WF_FIND_WALL:
            _handleFindWall(rightDist);
            break;

        case WF_FORWARD:
            _handleForward(frontDist, rightDist, yaw, imuValid);
            break;

        case WF_BACKUP:
            _handleBackup();
            break;

        case WF_TURN_LEFT:
            _handleTurnLeft(yaw, imuValid);
            break;

        default:
            _leftPWM = 0;
            _rightPWM = 0;
            break;
    }
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
    return _state;
}

uint8_t WallFollower::getCornerCount() {
    return _cornerCount;
}

// ==================== 狀態處理函數 ====================

void WallFollower::_handleFindWall(int rightDist) {
    /**
     * 尋找右牆狀態 - 右前方移動
     *
     * 策略: 以右轉+前進的方式移動，直到感測到右牆
     * - angular 負值 -> 右轉 (left 快, right 慢)
     * - left PWM ~191, right PWM ~115
     */

    unsigned long elapsed = _getElapsed();

    // 超時保護 - 3 秒內沒找到牆
    if (elapsed > FIND_WALL_TIMEOUT_MS) {
        _state = WF_FORWARD;
        _setMotorOutput(BASE_LINEAR_SPEED, 0);
        return;
    }

    // 找到右牆 (10~50 cm)
    if (_isSensorValid(rightDist) && rightDist > 10 && rightDist < 50) {
        _state = WF_FORWARD;
        _setMotorOutput(BASE_LINEAR_SPEED, 0);
        return;
    }

    // 右前方移動: angular = -0.15 -> left 快, right 慢
    _setMotorOutput(0.6, -0.15);
}

void WallFollower::_handleForward(int frontDist, int rightDist, float yaw, bool imuValid) {
    /**
     * 直行沿牆狀態 - 核心邏輯
     *
     * 優先順序:
     * 1. 角落 (前+右有牆) -> 後退
     * 2. 只有前牆 -> 左轉
     * 3. 都沒牆 -> 尋牆
     * 4. 正常沿牆 -> P 控制
     */

    bool frontValid = _isSensorValid(frontDist) && frontDist < 80;
    bool rightValid = _isSensorValid(rightDist) && rightDist < 80;

    // 情況 1: 角落 - 前方有牆 + 右側有牆
    if (frontValid && frontDist < FRONT_STOP_DISTANCE &&
        rightValid && rightDist < 50) {
        // 進入角落 -> 後退
        _state = WF_BACKUP;
        _stateStartTime = _currentTime;
        _cornerCount++;
        _setMotorOutput(BACKUP_SPEED, 0);
        return;
    }

    // 情況 2: 只有前牆，沒有右牆 -> 原地左轉
    if (frontValid && frontDist < FRONT_STOP_DISTANCE &&
        !(rightValid && rightDist < 50)) {
        _state = WF_TURN_LEFT;
        _stateStartTime = _currentTime;
        _imuTurnActive = true;
        if (imuValid) {
            // 計算目標 yaw
            _targetYaw = yaw + TURN_TARGET_YAW_OFFSET;
            // 角度正規化 (-180 ~ +180)
            while (_targetYaw < -180.0) _targetYaw += 360.0;
            while (_targetYaw > 180.0) _targetYaw -= 360.0;
        }
        _setMotorOutput(0, -TURN_ANGULAR_SPEED);
        return;
    }

    // 情況 3: 前後左右都沒牆 -> 右前方尋牆
    if ((!frontValid || frontDist > 80) && (!rightValid || rightDist > 80)) {
        _state = WF_FIND_WALL;
        _stateStartTime = _currentTime;
        _setMotorOutput(0.6, -0.15);
        return;
    }

    // 情況 4: 正常沿牆 - P 控制
    float linear = BASE_LINEAR_SPEED;

    // 前方減速
    if (frontValid && frontDist < FRONT_SLOW_DISTANCE) {
        linear = SLOW_LINEAR_SPEED;
    }

    // 沿牆 P 控制器
    float angular = 0.0;
    if (rightValid && rightDist < 80) {
        // error > 0: 離牆太遠 -> 需要右轉 -> angular 負值
        // error < 0: 離牆太近 -> 需要左轉 -> angular 正值
        float error = rightDist - TARGET_RIGHT_DISTANCE;
        angular = -error * WALL_FOLLOW_KP;

        // 限制角速度
        angular = constrain(angular, -0.35, 0.35);
    }

    _setMotorOutput(linear, angular);
}

void WallFollower::_handleBackup() {
    /**
     * 後退狀態 - 角落時小幅後退 (0.3 秒)
     *
     * 完成後進入原地左轉狀態
     */

    unsigned long elapsed = _getElapsed();

    // 後退時間到 -> 進入左轉
    if (elapsed > BACKUP_DURATION_MS) {
        _state = WF_TURN_LEFT;
        _stateStartTime = _currentTime;
        _imuTurnActive = true;
        _setMotorOutput(0, -TURN_ANGULAR_SPEED);
        return;
    }

    // 繼續後退
    _setMotorOutput(BACKUP_SPEED, 0);
}

void WallFollower::_handleTurnLeft(float yaw, bool imuValid) {
    /**
     * 左轉狀態 - 原地左轉
     *
     * 優先順序:
     * 1. IMU 可用 -> 角度控制 (±5°)
     * 2. IMU 不可用 -> 時間控制 (0.8 秒)
     */

    unsigned long elapsed = _getElapsed();

    // === IMU 模式 ===
    if (_imuTurnActive && imuValid) {
        // 檢查是否達到目標角度
        if (_isYawReached(yaw, _targetYaw)) {
            _state = WF_FORWARD;
            _imuTurnActive = false;
            _setMotorOutput(BASE_LINEAR_SPEED, 0);
            return;
        }

        // IMU 有效，繼續轉
        _setMotorOutput(0, -TURN_ANGULAR_SPEED);
        return;
    }

    // IMU 轉彎中但資料無效，用時間做備援
    if (_imuTurnActive && !imuValid) {
        // 超時保護: 比正常時間長 50%
        if (elapsed > TURN_DURATION_MS * 1.5) {
            _state = WF_FORWARD;
            _imuTurnActive = false;
            _setMotorOutput(BASE_LINEAR_SPEED, 0);
            return;
        }

        // 繼續轉
        _setMotorOutput(0, -TURN_ANGULAR_SPEED);
        return;
    }

    // === 純時間模式 ===
    if (elapsed > TURN_DURATION_MS) {
        _state = WF_FORWARD;
        _imuTurnActive = false;
        _setMotorOutput(BASE_LINEAR_SPEED, 0);
        return;
    }

    // 繼續原地左轉
    _setMotorOutput(0, -TURN_ANGULAR_SPEED);
}

// ==================== 輔助函數 ====================

void WallFollower::_setMotorOutput(float linear, float angular) {
    /**
     * 根據線性和角速度計算馬達 PWM
     *
     * 差速公式:
     * left = linear * MAX_PWM - angular * MAX_PWM
     * right = linear * MAX_PWM + angular * MAX_PWM
     *
     * 注意:
     * - angular 負值 -> 右轉 (left 快, right 慢)
     * - angular 正值 -> 左轉 (left 慢, right 快)
     * - 死區保護: |PWM| < MIN_EFFECTIVE_PWM 時設為 0
     */

    // 計算原始 PWM
    int leftPWM = (int)(linear * MAX_PWM) - (int)(angular * MAX_PWM);
    int rightPWM = (int)(linear * MAX_PWM) + (int)(angular * MAX_PWM);

    // 限制 PWM 範圍
    _leftPWM = _constrainPWM(leftPWM);
    _rightPWM = _constrainPWM(rightPWM);
}

int WallFollower::_constrainPWM(int pwm) {
    /**
     * 將 PWM 值限制在有效範圍內
     *
     * - 範圍: -255 ~ +255
     * - 死區保護: |PWM| < MIN_EFFECTIVE_PWM 時設為 0
     */

    // 先限制到 [-255, +255]
    int constrained = constrain(pwm, -MAX_PWM, MAX_PWM);

    // 死區保護
    if (abs(constrained) < MIN_EFFECTIVE_PWM) {
        return 0;
    }

    return constrained;
}

bool WallFollower::_isSensorValid(int distance) {
    /**
     * 檢查感測器資料是否有效
     *
     * - 無效值: 0, 999
     * - 有效值: 1 ~ 998 cm
     */

    return distance != 0 && distance != 999 && distance > 0;
}

unsigned long WallFollower::_getElapsed() {
    /**
     * 計算從狀態開始到現在的經過時間 (ms)
     */

    return _currentTime - _stateStartTime;
}

bool WallFollower::_isYawReached(float currentYaw, float targetYaw) {
    /**
     * 檢查 yaw 角度是否達到目標 (誤差 ±5°)
     *
     * 考慮 360° 環繞，計算最短路徑
     */

    float diff = _getYawDifference(currentYaw, targetYaw);
    return abs(diff) <= YAW_TOLERANCE;
}

float WallFollower::_getYawDifference(float current, float target) {
    /**
     * 計算角度差 (考慮 360° 環繞)
     *
     * 傳回 (-180, +180] 之間的角度差
     */

    float diff = target - current;

    // 正規化到 (-180, +180]
    while (diff > 180.0) diff -= 360.0;
    while (diff <= -180.0) diff += 360.0;

    return diff;
}
