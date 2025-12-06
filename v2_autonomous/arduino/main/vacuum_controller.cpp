/*
 * vacuum_controller.cpp - 吸塵器馬達控制實作
 */

#include "vacuum_controller.h"

VacuumController::VacuumController(uint8_t control_pin)
    : _control_pin(control_pin), _state(false) {
}

void VacuumController::begin() {
    pinMode(_control_pin, OUTPUT);
    digitalWrite(_control_pin, LOW);
}

void VacuumController::setState(bool state) {
    _state = state;
    digitalWrite(_control_pin, state ? HIGH : LOW);
}

bool VacuumController::getState() const {
    return _state;
}
