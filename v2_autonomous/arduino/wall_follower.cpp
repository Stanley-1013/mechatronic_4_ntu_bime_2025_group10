#include "wall_follower.h"

/**
 * WallFollower - 連續差動控制沿牆
 * 版本: 2.0 (IMU 角度鎖定)
 *
 * 控制策略:
 * 1. 角落鎖定 (corner_locked):
 *    - 偵測到角落 -> 鎖定控制
 *    - 後退 0.3 秒 (時間控制)
 *    - 左轉直到 IMU 偵測轉了 85° 或超時 3 秒
 *    - 解鎖，回到連續控制
 *
 * 2. 連續控制 (非鎖定時):
 *    - 前方障礙: 減速
 *    - 右側距離: P 控制保持 15cm
 *    - 無牆: 右轉尋牆
 */

// ==================== 常數定義 ====================

// 沿牆參數
static const int TARGET_RIGHT_DISTANCE = 15;        // 目標右側距離 (cm)
static const int FRONT_STOP_DISTANCE = 20;          // 角落偵測距離 (cm)
static const int FRONT_SLOW_DISTANCE = 40;          // 前方減速距離 (cm)
static const int CORNER_RIGHT_DISTANCE = 30;        // 角落右側閾值 (cm)

// 速度參數 (normalized: -1.0 ~ +1.0)
static const float BASE_LINEAR_SPEED = 0.6;         // 基礎線性速度
static const float BACKUP_SPEED = -0.4;             // 後退速度
static const float TURN_ANGULAR_SPEED = 0.7;        // 轉彎角速度
static const float WALL_FOLLOW_KP = 0.03;           // 沿牆 P 控制增益
static const float FIND_WALL_ANGULAR = -0.15;       // 尋牆角速度 (右轉)

// 時間常數 (ms)
static const unsigned long BACKUP_DURATION_MS = 300;    // 後退持續時間
static const unsigned long TURN_TIMEOUT_MS = 3000;      // 轉彎超時 (安全限制)

// PWM 常數
static const int MAX_PWM = 255;
static const int MIN_EFFECTIVE_PWM = 60;            // 死區

// 角度控制常數
static const float TURN_TARGET_ANGLE = 85.0;        // 目標轉彎角度 (度)

// 感測器常數
static const int SENSOR_INVALID = 0;
static const int SENSOR_MAX_RANGE = 500;

// ==================== 建構子 ====================

WallFollower::WallFollower()
    : _running(false),
      _cornerCount(0),
      _vacuumOn(true),
      _cornerLocked(false),
      _backupStartTime(0),
      _backupYaw(0),
      _historyIndex(0),
      _leftPWM(0),
      _rightPWM(0) {
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
    _backupStartTime = 0;
    _backupYaw = 0;

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
    _backupStartTime = 0;
    _backupYaw = 0;
    _leftPWM = 0;
    _rightPWM = 0;

    for (int i = 0; i < 3; i++) {
        _frontHistory[i] = -1;
        _rightHistory[i] = -1;
    }
}

void WallFollower::triggerAvoidRed() {
    // 預留介面
}

void WallFollower::update(int frontDist, int rightDist, float yaw, bool imuValid) {
    // 未執行時，輸出為 0
    if (!_running) {
        _leftPWM = 0;
        _rightPWM = 0;
        return;
    }

    unsigned long currentTime = millis();

    // 1. 感測器濾波
    int front = _filterSensor(frontDist, _frontHistory);
    int right = _filterSensor(rightDist, _rightHistory);

    // 2. 角落鎖定處理 (鎖定期間忽略感測器)
    if (_cornerLocked) {
        unsigned long elapsed = currentTime - _backupStartTime;

        // 2a. 後退階段 (時間控制)
        if (elapsed < BACKUP_DURATION_MS) {
            _setMotorOutput(BACKUP_SPEED, 0);
            return;
        }

        // 2b. 左轉階段 (IMU 角度控制 + 超時保護)
        unsigned long turnElapsed = elapsed - BACKUP_DURATION_MS;
        float turned = _angleDiff(yaw, _backupYaw);

        // 繼續轉彎條件: 還沒轉夠 AND 未超時
        if (turned < TURN_TARGET_ANGLE && turnElapsed < TURN_TIMEOUT_MS) {
            _setMotorOutput(0, TURN_ANGULAR_SPEED);  // 正值 = 左轉
            return;
        }

        // 2c. 角落處理完成，解鎖
        _cornerLocked = false;
        _backupStartTime = 0;
        _backupYaw = 0;
    }

    // 3. 角落偵測 (未鎖定時)
    if (front < FRONT_STOP_DISTANCE && right < CORNER_RIGHT_DISTANCE) {
        // 進入角落，鎖定並開始後退
        _cornerLocked = true;
        _backupStartTime = currentTime;
        _backupYaw = yaw;
        _cornerCount++;
        _setMotorOutput(BACKUP_SPEED, 0);
        return;
    }

    // 4. 前方障礙控制 (減速)
    float linear = BASE_LINEAR_SPEED;
    if (front <= 10) {
        // 太近，停止 (角落偵測應該已處理)
        linear = 0.0;
    } else if (front < FRONT_SLOW_DISTANCE) {
        // 線性插值: 40cm -> 0.6, 20cm -> 0.3
        linear = _mapRange(front, FRONT_STOP_DISTANCE, FRONT_SLOW_DISTANCE,
                          0.3, BASE_LINEAR_SPEED);
    }

    // 5. 右側距離控制 (P 控制器)
    float angular = 0.0;

    if (right > 5 && right < 80) {
        // 有效沿牆範圍
        // error > 0: 離牆太遠 -> 右轉 (angular 負)
        // error < 0: 離牆太近 -> 左轉 (angular 正)
        float error = right - TARGET_RIGHT_DISTANCE;
        angular = -error * WALL_FOLLOW_KP;
        angular = constrain(angular, -0.35, 0.35);
    } else if (right >= 80) {
        // 沒偵測到右牆，右轉尋牆
        angular = FIND_WALL_ANGULAR;
    } else {
        // 太近 (< 5cm)，左轉避開
        angular = 0.2;
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
     * left  = linear * MAX_PWM - angular * MAX_PWM
     * right = linear * MAX_PWM + angular * MAX_PWM
     *
     * angular > 0 -> 左轉 (right 快)
     * angular < 0 -> 右轉 (left 快)
     */

    int leftPWM = (int)(linear * MAX_PWM) - (int)(angular * MAX_PWM);
    int rightPWM = (int)(linear * MAX_PWM) + (int)(angular * MAX_PWM);

    _leftPWM = _constrainPWM(leftPWM);
    _rightPWM = _constrainPWM(rightPWM);
}

int WallFollower::_constrainPWM(int pwm) {
    // 限制範圍
    int constrained = constrain(pwm, -MAX_PWM, MAX_PWM);

    // 死區保護
    if (abs(constrained) < MIN_EFFECTIVE_PWM) {
        return 0;
    }

    return constrained;
}

float WallFollower::_mapRange(float x, float inMin, float inMax, float outMin, float outMax) {
    if (inMax == inMin) return outMin;
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
