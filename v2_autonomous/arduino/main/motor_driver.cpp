/*
 * motor_driver.cpp - L298N 馬達驅動模組實作
 * 版本: 1.0
 * 日期: 2025-10-31
 *
 * 參考: 01_SRS_軟體需求規格書.md - FR4
 */

#include "motor_driver.h"

MotorDriver::MotorDriver(uint8_t ena_pin, uint8_t in1_pin, uint8_t in2_pin,
                         uint8_t enb_pin, uint8_t in3_pin, uint8_t in4_pin)
    : _ena_pin(ena_pin), _in1_pin(in1_pin), _in2_pin(in2_pin),
      _enb_pin(enb_pin), _in3_pin(in3_pin), _in4_pin(in4_pin) {
}

void MotorDriver::begin() {
    // 設定腳位模式
    pinMode(_ena_pin, OUTPUT);
    pinMode(_in1_pin, OUTPUT);
    pinMode(_in2_pin, OUTPUT);
    pinMode(_enb_pin, OUTPUT);
    pinMode(_in3_pin, OUTPUT);
    pinMode(_in4_pin, OUTPUT);

    // 初始狀態：停止
    stop();
}

void MotorDriver::setLeftMotor(int16_t pwm) {
    _setMotor(_ena_pin, _in1_pin, _in2_pin, pwm);
}

void MotorDriver::setRightMotor(int16_t pwm) {
    _setMotor(_enb_pin, _in3_pin, _in4_pin, pwm);
}

void MotorDriver::stop() {
    setLeftMotor(0);
    setRightMotor(0);
}

void MotorDriver::_setMotor(uint8_t en_pin, uint8_t dir_a_pin, uint8_t dir_b_pin, int16_t pwm) {
    // 限制 PWM 範圍
    pwm = constrain(pwm, -255, 255);

    if (pwm > 0) {
        // 前進
        digitalWrite(dir_a_pin, HIGH);
        digitalWrite(dir_b_pin, LOW);
        analogWrite(en_pin, pwm);
    }
    else if (pwm < 0) {
        // 後退
        digitalWrite(dir_a_pin, LOW);
        digitalWrite(dir_b_pin, HIGH);
        analogWrite(en_pin, -pwm);  // PWM 必須是正值
    }
    else {
        // 停止（快速剎車）
        digitalWrite(dir_a_pin, LOW);
        digitalWrite(dir_b_pin, LOW);
        analogWrite(en_pin, 0);
    }
}
