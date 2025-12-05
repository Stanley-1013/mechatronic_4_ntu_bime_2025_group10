/*
 * motor_driver.h - L298N 馬達驅動模組
 * 版本: 1.0
 * 日期: 2025-10-31
 *
 * 控制 L298N H-Bridge 驅動雙馬達，支援 PWM 速度控制與方向控制。
 * 參考: 03_SD_系統設計.md - 4.2 Arduino 模組設計
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
public:
    /**
     * @brief 建構子
     * @param ena_pin 左輪 PWM 腳位 (Enable A)
     * @param in1_pin 左輪方向 A 腳位
     * @param in2_pin 左輪方向 B 腳位
     * @param enb_pin 右輪 PWM 腳位 (Enable B)
     * @param in3_pin 右輪方向 A 腳位
     * @param in4_pin 右輪方向 B 腳位
     */
    MotorDriver(uint8_t ena_pin, uint8_t in1_pin, uint8_t in2_pin,
                uint8_t enb_pin, uint8_t in3_pin, uint8_t in4_pin);

    /**
     * @brief 初始化馬達驅動（設定腳位模式）
     */
    void begin();

    /**
     * @brief 設定左輪馬達速度
     * @param pwm PWM 值 (-255 ~ +255)
     *            正值 = 前進，負值 = 後退，0 = 停止
     */
    void setLeftMotor(int16_t pwm);

    /**
     * @brief 設定右輪馬達速度
     * @param pwm PWM 值 (-255 ~ +255)
     */
    void setRightMotor(int16_t pwm);

    /**
     * @brief 停止所有馬達
     */
    void stop();

private:
    // 腳位定義
    uint8_t _ena_pin, _in1_pin, _in2_pin;
    uint8_t _enb_pin, _in3_pin, _in4_pin;

    /**
     * @brief 控制單一馬達
     * @param en_pin Enable 腳位
     * @param dir_a_pin 方向 A 腳位
     * @param dir_b_pin 方向 B 腳位
     * @param pwm PWM 值 (-255 ~ +255)
     */
    void _setMotor(uint8_t en_pin, uint8_t dir_a_pin, uint8_t dir_b_pin, int16_t pwm);
};

#endif // MOTOR_DRIVER_H
