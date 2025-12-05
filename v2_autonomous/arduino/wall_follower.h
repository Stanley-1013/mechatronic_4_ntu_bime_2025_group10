#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <Arduino.h>

/**
 * WallFollower - 沿牆狀態機 (Arduino C++ 實作)
 * 參考: Python wall_follower.py v3.0
 *
 * 狀態機流程:
 * IDLE -> FIND_WALL -> FORWARD -> (BACKUP -> TURN_LEFT) 或 TURN_LEFT -> FORWARD
 */

// 狀態定義
enum WallFollowerState {
    WF_IDLE = 0,        // 等待啟動
    WF_FIND_WALL = 1,   // 右前方尋找右牆
    WF_FORWARD = 2,     // 直行沿牆 (P 控制)
    WF_BACKUP = 3,      // 後退 (角落時)
    WF_TURN_LEFT = 4,   // 原地左轉 (角落後或只有前牆)
    WF_DONE = 5,        // 完成
    WF_ERROR = 0xFF     // 錯誤 (與 protocol.h 的 STATE_ERROR 一致)
};

class WallFollower {
public:
    /**
     * 建構子 - 初始化狀態機
     */
    WallFollower();

    /**
     * 啟動沿牆控制
     */
    void start();

    /**
     * 停止沿牆控制
     */
    void stop();

    /**
     * 觸發避紅色迴避 (預留介面)
     */
    void triggerAvoidRed();

    /**
     * 主更新函數 - 每個 Arduino loop() 呼叫一次
     *
     * @param frontDist   前方距離 (cm), 0 或 999 = 無效
     * @param rightDist   右側距離 (cm), 0 或 999 = 無效
     * @param yaw         當前 Yaw 角度 (度)
     * @param imuValid    IMU 資料是否有效
     */
    void update(int frontDist, int rightDist, float yaw, bool imuValid);

    /**
     * 取得左馬達 PWM 輸出 (-255 ~ +255)
     * 負值: 後退, 正值: 前進
     */
    int getLeftPWM();

    /**
     * 取得右馬達 PWM 輸出 (-255 ~ +255)
     */
    int getRightPWM();

    /**
     * 取得吸塵器狀態
     */
    bool getVacuumState();

    /**
     * 取得當前狀態
     */
    WallFollowerState getState();

    /**
     * 取得角落計數
     */
    uint8_t getCornerCount();

private:
    // ==================== 狀態變數 ====================
    WallFollowerState _state;      // 當前狀態
    uint8_t _cornerCount;          // 角落計數
    bool _vacuumOn;                // 吸塵器狀態

    // ==================== 計時變數 ====================
    unsigned long _stateStartTime; // 狀態開始時間 (ms)
    unsigned long _currentTime;    // 當前時間 (ms)

    // ==================== IMU 轉彎控制 ====================
    float _targetYaw;              // 目標 Yaw 角度
    bool _imuTurnActive;           // IMU 轉彎是否進行中

    // ==================== 馬達 PWM 輸出 ====================
    int _leftPWM;                  // 左馬達 PWM (-255 ~ +255)
    int _rightPWM;                 // 右馬達 PWM (-255 ~ +255)

    // ==================== 狀態處理函數 ====================

    /**
     * 尋找右牆狀態 - 右前方移動
     */
    void _handleFindWall(int rightDist);

    /**
     * 直行沿牆狀態 - 核心邏輯
     *
     * 優先順序:
     * 1. 角落 (前+右有牆) -> 後退
     * 2. 只有前牆 -> 左轉
     * 3. 都沒牆 -> 尋牆
     * 4. 正常沿牆 -> P 控制
     */
    void _handleForward(int frontDist, int rightDist, float yaw, bool imuValid);

    /**
     * 後退狀態 - 角落時小幅後退
     */
    void _handleBackup();

    /**
     * 左轉狀態 - 原地左轉
     *
     * 優先順序:
     * 1. IMU 可用 -> 角度控制
     * 2. IMU 不可用 -> 時間控制
     */
    void _handleTurnLeft(float yaw, bool imuValid);

    // ==================== 輔助函數 ====================

    /**
     * 根據線性和角速度計算馬達 PWM
     *
     * 差速公式 (重要!):
     * left = linear * MAX_PWM - angular * MAX_PWM
     * right = linear * MAX_PWM + angular * MAX_PWM
     *
     * 注意:
     * - angular 負值 -> 右轉 (left 快, right 慢)
     * - angular 正值 -> 左轉 (left 慢, right 快)
     * - 死區保護: |PWM| < MIN_EFFECTIVE_PWM 時設為 0
     *
     * @param linear   線性速度 (-1.0 ~ +1.0)
     * @param angular  角速度 (-1.0 ~ +1.0)
     */
    void _setMotorOutput(float linear, float angular);

    /**
     * 將 PWM 值限制在有效範圍內
     */
    int _constrainPWM(int pwm);

    /**
     * 檢查感測器資料是否有效
     */
    bool _isSensorValid(int distance);

    /**
     * 計算經過時間 (ms)
     */
    unsigned long _getElapsed();

    /**
     * 檢查 yaw 角度是否達到目標 (誤差 ±5°)
     */
    bool _isYawReached(float currentYaw, float targetYaw);

    /**
     * 計算角度差 (考慮 360° 環繞)
     */
    float _getYawDifference(float current, float target);
};

#endif // WALL_FOLLOWER_H
