// behavior.h - 行為控制模組 (距離+角度雙控制)
// 版本: 2.0
// 日期: 2025-12-07

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <avr/wdt.h>
#include "ultrasonic.h"  // for SensorData
#include "config.h"

struct MotorCommand {
    int leftPWM;        // -255 ~ +255
    int rightPWM;       // -255 ~ +255
    bool stop;          // 是否停止
};

class BehaviorController {
public:
    void init();
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
};

#endif
