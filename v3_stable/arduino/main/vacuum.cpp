// vacuum.cpp
#include "vacuum.h"
#include "config.h"
#include <Arduino.h>

void Vacuum::init() {
    pinMode(PIN_VACUUM, OUTPUT);
    off();
}

void Vacuum::on() {
    digitalWrite(PIN_VACUUM, HIGH);
    _isOn = true;
}

void Vacuum::off() {
    digitalWrite(PIN_VACUUM, LOW);
    _isOn = false;
}
