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

unsigned long count = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial) { ; }

    Serial.println(F("A1/A2 Ultrasonic Test"));
    Serial.println(F("Trig=A1, Echo=A2"));
    Serial.println();

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    // 檢查 Echo 初始狀態
    int echoState = digitalRead(ECHO_PIN);
    if (echoState == HIGH) {
        Serial.println(F("WARNING: Echo is HIGH - check wiring!"));
    }

    Serial.println(F("Count | Distance | Status"));
    Serial.println(F("------|----------|--------"));
    delay(500);
}

void loop() {
    // 觸發
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // 測量
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    int distance = duration * 0.034 / 2;

    // 覆蓋式單行顯示
    Serial.print(F("\rDistance: "));

    if (duration == 0) {
        Serial.print(F("--- cm (TIMEOUT)   "));
    } else {
        // 固定寬度輸出
        if (distance < 10) Serial.print(F("  "));
        else if (distance < 100) Serial.print(F(" "));
        Serial.print(distance);
        Serial.print(F(" cm            "));
    }

    delay(100);
}
