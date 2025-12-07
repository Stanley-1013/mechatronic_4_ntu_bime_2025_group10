// behavior.h - 行為控制模組 (距離+角度 PD 控制)
// 版本: 3.5
// 日期: 2025-12-08

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <avr/wdt.h>
#include "ultrasonic.h"  // for SensorData
#include "mpu6050_sensor.h"  // for IMU
#include "config.h"

struct MotorCommand {
    int leftPWM;        // -255 ~ +255
    int rightPWM;       // -255 ~ +255
    bool stop;          // 是否停止
};

class BehaviorController {
public:
    void init(MPU6050Sensor* imu);  // 傳入 IMU 指標
    MotorCommand update(const SensorData& sensor);

    bool isTurning() { return _isTurning; }
    bool isComplete() { return _complete; }

private:
    MotorCommand _handleTurning(const SensorData& sensor);
    MotorCommand _handleWallFollow(const SensorData& sensor);

    // 軟閾值函數：value < (threshold-width) → 1, value > (threshold+width) → 0
    // 中間平滑過渡，避免邊界抖動
    float _softThreshold(float value, float threshold, float width);

    // 狀態變數
    bool _isTurning;
    int _turnTimer;
    int _stableTimer;
    bool _complete;
    unsigned long _startTime;

    // 右前趨勢追蹤 (用於 rightValid=false 時判斷接近/遠離)
    float _lastRightFront;

    // 前方觸發連續確認 (防止雜訊誤觸發)
    int _frontTriggerCount;

    // 角度 D 項追蹤 (用於 PD 控制)
    float _lastAngle;

    // 角落轉彎標記 (用於穩定期補償)
    bool _turnFromCorner;

    // IMU 相關
    MPU6050Sensor* _imu;
    float _turnStartYaw;  // 轉彎開始時的 yaw
};

#endif
