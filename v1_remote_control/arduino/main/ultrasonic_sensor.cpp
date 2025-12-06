/*
 * ultrasonic_sensor.cpp - HC-SR04 超聲波感測器實作
 * 版本: 1.0
 * 日期: 2025-10-31
 *
 * 參考: 01_SRS_軟體需求規格書.md - FR6
 */

#include "ultrasonic_sensor.h"

// 常數定義
#define INVALID_DISTANCE 999
#define MIN_DISTANCE 2
#define MAX_DISTANCE 400
#define TIMEOUT_US 30000  // 30ms 逾時

UltrasonicSensor::UltrasonicSensor(uint8_t trig_pin, uint8_t echo_pin)
    : _trig_pin(trig_pin), _echo_pin(echo_pin) {
}

void UltrasonicSensor::begin() {
    pinMode(_trig_pin, OUTPUT);
    pinMode(_echo_pin, INPUT);
    digitalWrite(_trig_pin, LOW);
}

uint16_t UltrasonicSensor::getDistance() {
    // 步驟 1: 發送 10μs 觸發脈衝
    digitalWrite(_trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig_pin, LOW);

    // 步驟 2: 測量回波持續時間（最長等待 30ms）
    unsigned long duration = pulseIn(_echo_pin, HIGH, TIMEOUT_US);

    // 步驟 3: 計算距離
    // 公式: distance_cm = duration_us * 0.034 / 2
    // 聲速 340 m/s = 0.034 cm/μs
    // 除以 2 是因為來回
    uint16_t distance = duration * 0.034 / 2;

    // 步驟 4: 驗證距離有效性
    return _validateDistance(distance, duration);
}

uint16_t UltrasonicSensor::_validateDistance(uint16_t distance, unsigned long duration) {
    // 檢查 1: 逾時（無回波）
    if (duration == 0) {
        return INVALID_DISTANCE;
    }

    // 檢查 2: 距離太近
    if (distance < MIN_DISTANCE) {
        return INVALID_DISTANCE;
    }

    // 檢查 3: 距離太遠
    if (distance > MAX_DISTANCE) {
        return INVALID_DISTANCE;
    }

    // 有效距離
    return distance;
}
