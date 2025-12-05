/*
 * ultrasonic_sensor.h - HC-SR04 超聲波感測器模組
 * 版本: 1.0
 * 日期: 2025-10-31
 *
 * 讀取 HC-SR04 超聲波感測器距離。
 * 參考: 03_SD_系統設計.md - 4.2 Arduino 模組設計
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
    /**
     * @brief 建構子
     * @param trig_pin 觸發腳位
     * @param echo_pin 回波腳位
     */
    UltrasonicSensor(uint8_t trig_pin, uint8_t echo_pin);

    /**
     * @brief 初始化感測器（設定腳位模式）
     */
    void begin();

    /**
     * @brief 測量距離
     * @return 距離 (cm)，範圍 2-400cm，999 表示無效值
     *
     * 參考: 01_SRS_軟體需求規格書.md - FR6
     */
    uint16_t getDistance();

private:
    uint8_t _trig_pin;
    uint8_t _echo_pin;

    /**
     * @brief 驗證距離是否有效
     * @param distance 距離 (cm)
     * @param duration 回波持續時間 (μs)
     * @return 有效距離或 999
     */
    uint16_t _validateDistance(uint16_t distance, unsigned long duration);
};

#endif // ULTRASONIC_SENSOR_H
