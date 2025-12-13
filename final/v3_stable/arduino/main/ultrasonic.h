// ultrasonic.h - 三超音波管理 + 角度計算
// 版本: 2.0
// 日期: 2025-12-07

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// 感測器資料結構
struct SensorData {
    int front;          // 前方距離 (cm)
    int rightFront;     // 右前距離 (cm)
    int rightRear;      // 右後距離 (cm)
    float rightAvg;     // 右側平均距離 (cm)
    float angle;        // 車身與牆夾角 (度), 正=車頭朝牆
    bool rightValid;    // 右側資料是否有效 (可計算角度)
};

class UltrasonicManager {
public:
    void init();
    void update();                      // 輪流讀取 (每次呼叫讀一個)
    SensorData getData();               // 取得所有感測器資料

private:
    int _readOne(int trigPin, int echoPin);
    int _filter(int* history, int value);

    // 濾波後的讀值
    int _front, _rightF, _rightR;

    // 中值濾波歷史 (各 3 個)
    int _histFront[3], _histRightF[3], _histRightR[3];

    // 輪流讀取狀態 (0=前, 1=右前, 2=右後)
    int _readIndex;
};

#endif
