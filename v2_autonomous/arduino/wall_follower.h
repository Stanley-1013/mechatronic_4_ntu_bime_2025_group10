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

    // ==================== 感測器濾波 ====================
    int _frontHistory[3];          // 前方距離歷史 (中值濾波)
    int _rightHistory[3];          // 右側距離歷史
    uint8_t _historyIndex;         // 歷史索引

    // ==================== 馬達 PWM 輸出 ====================
    int _leftPWM;                  // 左馬達 PWM (-255 ~ +255)
    int _rightPWM;                 // 右馬達 PWM (-255 ~ +255)

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
};

#endif // WALL_FOLLOWER_H
