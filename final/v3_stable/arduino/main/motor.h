// motor.h
#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    void init();
    void set(int leftPWM, int rightPWM);
    void stop();

private:
    void _setLeft(int pwm);
    void _setRight(int pwm);
};

#endif
