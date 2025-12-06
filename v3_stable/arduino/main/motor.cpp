// motor.cpp
#include "motor.h"
#include "config.h"
#include <Arduino.h>

void Motor::init() {
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    stop();
}

void Motor::set(int leftPWM, int rightPWM) {
    _setLeft(leftPWM);
    _setRight(rightPWM);
}

void Motor::stop() {
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
}

void Motor::_setLeft(int pwm) {
    if (pwm > 0) {
        // 前進
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, pwm);
    } else if (pwm < 0) {
        // 後退
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        analogWrite(PIN_ENA, -pwm);
    } else {
        // 停止
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, 0);
    }
}

void Motor::_setRight(int pwm) {
    if (pwm > 0) {
        // 前進
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, pwm);
    } else if (pwm < 0) {
        // 後退
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
        analogWrite(PIN_ENB, -pwm);
    } else {
        // 停止
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, 0);
    }
}
