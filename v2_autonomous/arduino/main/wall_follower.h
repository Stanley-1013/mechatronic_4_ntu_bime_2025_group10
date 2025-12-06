#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <Arduino.h>

/**
 * WallFollower - 連續差動控制沿牆 (Arduino C++ 實作)
 * 版本: 2.0 (IMU 角度鎖定)
 *
 * 架構變更:
 * - 移除複雜狀態機，改用 corner_locked flag
 * - 角落時鎖定控制，用 IMU 角度判斷轉彎完成
 * - 連續 P 控制沿牆
 *
 * 控制邏輯:
 * - 差速公式: left = linear - angular, right = linear + angular
 * - angular < 0: 右轉 (left 快)
 * - angular > 0: 左轉 (right 快)
 */

// 狀態定義 (簡化，主要用於外部查詢)
enum WallFollowerState {
    WF_IDLE = 0,        // 等待啟動
    WF_RUNNING = 2,     // 執行中 (對應 FORWARD，保持協定相容)
    WF_DONE = 5,        // 完成
    WF_ERROR = 0xFF     // 錯誤
};

class WallFollower {
public:
    /**
     * 建構子 - 初始化控制器
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
     * 重置控制器
     */
    void reset();

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

    /**
     * 檢查是否在角落鎖定中
     */
    bool isCornerLocked();

    /**
     * 設定 PID 參數 (可由 Pi 動態調整)
     */
    void setPID(float kp, float ki, float kd);

    /**
     * 設定所有控制參數 (由 Pi 透過 CMD_SET_PARAMS 呼叫)
     *
     * @param targetRightDist  目標右側距離 (cm)
     * @param frontStopDist    前方停止距離 (cm)
     * @param frontSlowDist    前方減速距離 (cm)
     * @param cornerRightDist  角落右側閾值 (cm)
     * @param baseLinearSpeed  基礎前進速度 (-1.0 ~ +1.0)
     * @param backupSpeed      後退速度 (-1.0 ~ +1.0)
     * @param turnAngularSpeed 角落轉彎角速度
     * @param findWallLinear   尋牆前進速度
     * @param leftMotorScale   左輪速度倍率 (補償馬達差異)
     * @param rightMotorScale  右輪速度倍率 (補償馬達差異)
     * @param kp               PID 比例增益
     * @param ki               PID 積分增益
     * @param kd               PID 微分增益
     * @param minEffectivePWM  最小有效 PWM (死區)
     * @param cornerTurnAngle  角落轉彎角度 (度)
     * @param redAvoidAngle    紅色迴避角度 (度)
     * @param backupDurationMs 後退持續時間 (ms)
     * @param turnTimeoutMs    轉彎超時 (ms)
     */
    void setParams(
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
    );

private:
    // ==================== 狀態變數 ====================
    bool _running;                 // 是否執行中
    uint8_t _cornerCount;          // 角落計數
    bool _vacuumOn;                // 吸塵器狀態

    // ==================== 角落鎖定 ====================
    bool _cornerLocked;            // 角落鎖定中
    bool _isRedAvoid;              // 是否為紅色迴避 (用較小角度)
    unsigned long _backupStartTime; // 後退開始時間 (ms)
    float _backupYaw;              // 後退開始時的 yaw 角度

    // ==================== 啟動保護 ====================
    unsigned long _startupGracePeriod;  // 啟動時間戳 (用於保護期)

    // ==================== 感測器濾波 ====================
    int _frontHistory[3];          // 前方距離歷史 (中值濾波)
    int _rightHistory[3];          // 右側距離歷史
    uint8_t _historyIndex;         // 歷史索引

    // ==================== 馬達 PWM 輸出 ====================
    int _leftPWM;                  // 左馬達 PWM (-255 ~ +255)
    int _rightPWM;                 // 右馬達 PWM (-255 ~ +255)

    // ==================== PID 控制變數 ====================
    float _lastError;              // 上一次的誤差 (用於 D 項)
    float _integral;               // 積分累積值 (用於 I 項)
    unsigned long _lastUpdateTime; // 上次更新時間 (用於計算 dt)

    // ==================== IMU 航向控制 ====================
    float _targetYaw;              // 目標航向（度）
    bool _yawLocked;               // 航向是否已鎖定
    float _driftTimer;             // 漂移計時器（秒）
    float _lastTurnYaw;            // 轉彎起始航向

    // ==================== 輔助函數 ====================

    /**
     * 中值濾波
     */
    int _filterSensor(int value, int* history);

    /**
     * 計算角度差絕對值 (考慮 360° 環繞)
     */
    float _angleDiff(float currentYaw, float startYaw);

    /**
     * 根據線性和角速度計算馬達 PWM
     */
    void _setMotorOutput(float linear, float angular);

    /**
     * 將 PWM 值限制在有效範圍內
     */
    int _constrainPWM(int pwm);

    /**
     * 線性映射
     */
    float _mapRange(float x, float inMin, float inMax, float outMin, float outMax);

    /**
     * 正規化角度到 [-180, 180] 範圍
     * 用於計算航向誤差
     */
    float _normalizeAngle(float angle);
};

#endif // WALL_FOLLOWER_H
