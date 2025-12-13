// vacuum.h
#ifndef VACUUM_H
#define VACUUM_H

class Vacuum {
public:
    void init();
    void on();
    void off();
    bool isOn() { return _isOn; }

private:
    bool _isOn;
};

#endif
