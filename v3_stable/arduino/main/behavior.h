// behavior.h
#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <avr/wdt.h>  // Watchdog
#include "config.h"   // For EXIT_CONFIRM and other constants

struct MotorCommand {
    int leftPWM;        // -255 ~ +255
    int rightPWM;       // -255 ~ +255
    bool stop;          // 是否停止
};

class BehaviorController {
public:
    void init();
    MotorCommand update(int frontDist, int rightDist);

    int getCornerCount() { return _cornerCount; }
    bool isTurning() { return _isTurning; }
    bool isExiting() { return _exitCounter >= EXIT_CONFIRM; }
    bool isComplete() { return _complete; }

private:
    // 優先級控制
    MotorCommand _handleTurning(int frontDist, int rightDist);
    MotorCommand _handleExit(int frontDist, int rightDist);
    MotorCommand _handleWallFollow(int frontDist, int rightDist, bool stabilizing = false);
    void _updateExitCounter(int frontDist, int rightDist);

    // 狀態變數
    int _cornerCount;
    bool _isTurning;
    int _turnConfirm;
    int _turnTimer;         // 轉彎計時 (超時保護)
    int _stableTimer;       // 穩定期計時 (轉彎後直行)
    int _frontConfirm;      // 前方障礙確認計數
    float _exitCounter;     // float 支援緩慢衰減
    bool _complete;
    unsigned long _startTime;
    unsigned long _exitSearchStart;  // 出場搜尋開始時間 (第4角後)

    // PD 控制用
    float _lastRightDist;   // 上次右側距離 (用於計算變化率)
};

#endif
