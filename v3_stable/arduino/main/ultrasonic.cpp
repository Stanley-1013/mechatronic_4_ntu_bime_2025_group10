#include "ultrasonic.h"
#include "config.h"
#include <Arduino.h>

void Ultrasonic::init(int trigPin, int echoPin) {
    _trigPin = trigPin;
    _echoPin = echoPin;
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);

    // 初始化歷史為目標距離
    for (int i = 0; i < 3; i++) {
        _history[i] = TARGET_DIST;
    }
    _histIndex = 0;
}

int Ultrasonic::read() {
    // 發送觸發脈衝
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    // 讀取回波時間
    long duration = pulseIn(_echoPin, HIGH, US_TIMEOUT);

    // 轉換為距離 (cm)
    if (duration == 0) {
        return -1;  // 超時
    }
    int dist = duration / 58;

    // 有效性檢查
    if (dist < US_MIN_VALID || dist > US_MAX_VALID) {
        return -1;
    }

    return dist;
}

int Ultrasonic::getFiltered() {
    int raw = read();
    return _filter(raw);
}

int Ultrasonic::_filter(int value) {
    // 只接受有效值
    if (value > 0) {
        _history[_histIndex] = value;
        _histIndex = (_histIndex + 1) % 3;
    }

    // 中值濾波
    int a = _history[0];
    int b = _history[1];
    int c = _history[2];

    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) { int t = b; b = c; c = t; }
    if (a > b) { int t = a; a = b; b = t; }

    return b;  // 中位數
}
