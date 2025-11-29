/*
 * test_ultrasonic_a1a2.ino - 測試 A1/A2 超聲波
 *
 * 接線:
 *   右側超聲波 Trig -> A1
 *   右側超聲波 Echo -> A2
 *   VCC -> 5V
 *   GND -> GND
 *
 * 開啟 Serial Monitor (9600 baud) 查看結果
 */

#define TRIG_PIN A1
#define ECHO_PIN A2

void setup() {
    Serial.begin(9600);

    // 等待 Serial 連接
    while (!Serial) {
        ;
    }

    Serial.println("========================================");
    Serial.println(" A1/A2 Ultrasonic Test");
    Serial.println("========================================");
    Serial.print("TRIG_PIN (A1) = ");
    Serial.println(TRIG_PIN);  // 應該顯示 15
    Serial.print("ECHO_PIN (A2) = ");
    Serial.println(ECHO_PIN);  // 應該顯示 16
    Serial.println();

    // 設定腳位
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    Serial.println("Testing pin modes...");

    // 測試 Trig 腳位輸出
    Serial.print("  Trig LOW: ");
    digitalWrite(TRIG_PIN, LOW);
    delay(10);
    Serial.println("OK");

    Serial.print("  Trig HIGH: ");
    digitalWrite(TRIG_PIN, HIGH);
    delay(10);
    Serial.println("OK");

    digitalWrite(TRIG_PIN, LOW);

    // 測試 Echo 腳位輸入
    Serial.print("  Echo read: ");
    int echoState = digitalRead(ECHO_PIN);
    Serial.print(echoState);
    Serial.println(echoState == LOW ? " (LOW - normal)" : " (HIGH - check wiring!)");

    Serial.println("\nStarting distance measurements...\n");
    delay(1000);
}

void loop() {
    // 發送觸發脈衝
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // 測量回波時間
    unsigned long startTime = micros();
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms 超時
    unsigned long endTime = micros();

    // 計算距離
    float distance = duration * 0.034 / 2;

    // 顯示結果
    Serial.print("Duration: ");
    Serial.print(duration);
    Serial.print(" us");

    if (duration == 0) {
        Serial.println(" -> TIMEOUT (no echo received)");
        Serial.println("    Check: 1) Wiring  2) Sensor power  3) Object in range");
    } else {
        Serial.print(" -> Distance: ");
        Serial.print(distance, 1);
        Serial.print(" cm");

        if (distance < 2) {
            Serial.println(" (too close)");
        } else if (distance > 400) {
            Serial.println(" (too far / invalid)");
        } else {
            Serial.println(" (OK)");
        }
    }

    delay(200);  // 5Hz
}
