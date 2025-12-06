#ifndef ULTRASONIC_H
#define ULTRASONIC_H

class Ultrasonic {
public:
    void init(int trigPin, int echoPin);
    int read();              // 讀取距離 (cm)
    int getFiltered();       // 取得濾波後距離

private:
    int _trigPin, _echoPin;
    int _history[3];         // 中值濾波歷史
    int _histIndex;
    int _filter(int value);
};

#endif
