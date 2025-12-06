/*
 * test_straight.ino - 最簡單的直走測試
 *
 * 上傳後車子會：
 * 1. 等待 3 秒 (讓你準備)
 * 2. 雙輪前進 5 秒
 * 3. 停止
 *
 * 腳位定義和 v2 主程式相同
 */

// L298N 馬達腳位
#define PIN_ENA 3   // 左輪 PWM
#define PIN_IN1 6   // 左輪方向 A
#define PIN_IN2 5   // 左輪方向 B
#define PIN_ENB 11  // 右輪 PWM
#define PIN_IN3 9   // 右輪方向 A
#define PIN_IN4 10  // 右輪方向 B

void setup() {
    Serial.begin(115200);

    // 設定馬達腳位
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    // 先停止
    stopMotors();

    Serial.println(F("=== 直走測試 ==="));
    Serial.println(F("3 秒後開始..."));
    delay(3000);

    // 前進 5 秒
    Serial.println(F("前進中 (PWM=80)..."));
    forward(80);
    delay(5000);

    // 停止
    stopMotors();
    Serial.println(F("測試完成"));
}

void loop() {
    // 什麼都不做
}

void forward(int pwm) {
    // 左輪前進
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, pwm);

    // 右輪前進
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, pwm);
}

void stopMotors() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);

    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, 0);
}
