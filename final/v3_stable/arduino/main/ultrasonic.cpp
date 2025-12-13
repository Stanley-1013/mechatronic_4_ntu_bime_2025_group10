// ultrasonic.cpp - 三超音波管理 + 角度計算
// 版本: 2.0
// 日期: 2025-12-07

#include "ultrasonic.h"
#include "config.h"

void UltrasonicManager::init() {
    // 設定腳位
    pinMode(PIN_FRONT_TRIG, OUTPUT);
    pinMode(PIN_FRONT_ECHO, INPUT);
    pinMode(PIN_RIGHT_F_TRIG, OUTPUT);
    pinMode(PIN_RIGHT_F_ECHO, INPUT);
    pinMode(PIN_RIGHT_R_TRIG, OUTPUT);
    pinMode(PIN_RIGHT_R_ECHO, INPUT);

    // 初始化歷史值
    for (int i = 0; i < 3; i++) {
        _histFront[i] = 100;            // 前方預設較遠
        _histRightF[i] = TARGET_DIST;   // 右側預設目標距離
        _histRightR[i] = TARGET_DIST;
    }

    _front = 100;
    _rightF = TARGET_DIST;
    _rightR = TARGET_DIST;
    _readIndex = 0;
}

void UltrasonicManager::update() {
    // v3.12: 一次讀三個（順序讀取不會干擾）
    int val;

    // 前方
    val = _readOne(PIN_FRONT_TRIG, PIN_FRONT_ECHO);
    if (val > 0) _front = _filter(_histFront, val);

    // 右前
    val = _readOne(PIN_RIGHT_F_TRIG, PIN_RIGHT_F_ECHO);
    if (val > 0) _rightF = _filter(_histRightF, val);

    // 右後
    val = _readOne(PIN_RIGHT_R_TRIG, PIN_RIGHT_R_ECHO);
    if (val > 0) _rightR = _filter(_histRightR, val);
}

SensorData UltrasonicManager::getData() {
    SensorData data;
    data.front = _front;
    data.rightFront = _rightF;
    data.rightRear = _rightR;

    // 計算右側平均距離
    data.rightAvg = (_rightF + _rightR) / 2.0f;

    // 計算角度 (需兩個右側都在有效範圍內)
    // 有效範圍：2~50cm (太遠的讀數不可靠)
    bool fValid = (_rightF >= US_MIN_VALID && _rightF <= 50);
    bool rValid = (_rightR >= US_MIN_VALID && _rightR <= 50);

    if (fValid && rValid) {
        // angle = atan2(後-前, 間距) * 180 / PI
        // 正角度 = 車頭朝向牆 (rightFront < rightRear，需左轉修正)
        // 負角度 = 車尾朝向牆 (rightFront > rightRear，需右轉修正)
        // 修正：原本 (前-後) 符號相反，改為 (後-前)
        float diff = (float)_rightR - (float)_rightF;
        data.angle = atan2(diff, US_SPACING) * 57.2958f;  // 轉換為度
        data.rightValid = true;
    } else {
        data.angle = 0;
        data.rightValid = false;
    }

    return data;
}

int UltrasonicManager::_readOne(int trigPin, int echoPin) {
    // 發送 10μs 觸發脈衝
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 讀取回波時間
    long duration = pulseIn(echoPin, HIGH, US_TIMEOUT);

    if (duration == 0) {
        return -1;  // 超時
    }

    // 轉換為距離 (cm)
    int dist = duration / 58;

    // 有效性檢查
    if (dist < US_MIN_VALID || dist > US_MAX_VALID) {
        return -1;
    }

    return dist;
}

int UltrasonicManager::_filter(int* history, int value) {
    // 移入新值 (FIFO)
    history[2] = history[1];
    history[1] = history[0];
    history[0] = value;

    // 中值濾波
    int a = history[0];
    int b = history[1];
    int c = history[2];

    // 排序找中位數
    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) { int t = b; b = c; c = t; }
    if (a > b) { int t = a; a = b; b = t; }

    return b;  // 中位數
}
