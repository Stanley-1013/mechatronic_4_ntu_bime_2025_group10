/*
 * main.ino - Arduino 主程式
 * 版本: 1.1 (使用 config.h 集中管理設定)
 * 日期: 2025-10-31
 *
 * 機電小車 Arduino 控制程式
 * 功能:
 * - 接收 Raspberry Pi 的馬達指令 (Serial)
 * - 控制 L298N 驅動雙馬達
 * - 讀取左右超聲波感測器
 * - 控制吸塵器馬達
 * - 回傳感測器資料給 Pi
 *
 * 參考: 03_SD_系統設計.md, 04_ICD_介面規格.md
 */

#include <SoftwareSerial.h>
#include "config.h"             // ← 使用 config.h 管理所有設定
#include "motor_driver.h"
#include "ultrasonic_sensor.h"
#include "vacuum_controller.h"
#include "serial_protocol.h"

// ==================== 物件初始化 ====================
SoftwareSerial piSerial(PIN_SERIAL_RX, PIN_SERIAL_TX);

MotorDriver motor(PIN_ENA, PIN_IN1, PIN_IN2, PIN_ENB, PIN_IN3, PIN_IN4);
UltrasonicSensor leftUltrasonic(PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO);
UltrasonicSensor rightUltrasonic(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);
VacuumController vacuum(PIN_VACUUM);

// ==================== 全域變數 ====================
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
unsigned long lastCommandTime = 0;
unsigned long lastSensorTime = 0;

uint16_t leftDistance = 999;
uint16_t rightDistance = 999;

// ==================== Setup ====================
void setup() {
    #ifdef DEBUG_SERIAL_ENABLED
    // 初始化 USB Serial (除錯用)
    Serial.begin(115200);
    DEBUG_PRINTLN(F("========================================="));
    DEBUG_PRINTLN(F(" Arduino Robot Controller v1.1"));
    DEBUG_PRINTLN(F("========================================="));
    DEBUG_PRINTLN(F("Initializing..."));
    #endif

    // 初始化 SoftwareSerial (與 Pi 通訊)
    piSerial.begin(SERIAL_BAUDRATE);
    DEBUG_PRINT(F("[OK] Serial @ "));
    DEBUG_PRINT(SERIAL_BAUDRATE);
    DEBUG_PRINTLN(F(" bps"));

    // 初始化馬達驅動
    motor.begin();
    DEBUG_PRINTLN(F("[OK] Motor driver"));

    // 初始化超聲波感測器
    leftUltrasonic.begin();
    rightUltrasonic.begin();
    DEBUG_PRINTLN(F("[OK] Ultrasonic sensors"));

    // 初始化吸塵器控制
    vacuum.begin();
    DEBUG_PRINTLN(F("[OK] Vacuum controller"));

    // 顯示設定資訊
    #ifdef DEBUG_SERIAL_ENABLED
    DEBUG_PRINTLN(F("\n[Config]"));
    DEBUG_PRINT(F("  Sensor interval: "));
    DEBUG_PRINT(SENSOR_UPDATE_INTERVAL);
    DEBUG_PRINTLN(F(" ms"));
    DEBUG_PRINT(F("  Command timeout: "));
    DEBUG_PRINT(COMMAND_TIMEOUT);
    DEBUG_PRINTLN(F(" ms"));
    DEBUG_PRINTLN(F("\n========================================="));
    DEBUG_PRINTLN(F(" System Ready - Waiting for commands"));
    DEBUG_PRINTLN(F("=========================================\n"));
    #endif

    lastCommandTime = millis();
    lastSensorTime = millis();
}

// ==================== Main Loop ====================
void loop() {
    unsigned long currentTime = millis();

    // ========== 1. 接收並處理馬達指令 ==========
    while (piSerial.available()) {
        uint8_t byte = piSerial.read();

        // 尋找 Header
        if (rxIndex == 0) {
            if (byte == MOTOR_HEADER) {
                rxBuffer[rxIndex++] = byte;
            }
        }
        else {
            rxBuffer[rxIndex++] = byte;

            // 收齊 8 bytes
            if (rxIndex >= PACKET_SIZE) {
                processMotorCommand();
                rxIndex = 0;
            }
        }
    }

    // ========== 2. 逾時保護 ==========
    if (currentTime - lastCommandTime > COMMAND_TIMEOUT) {
        // 超過 200ms 未收到指令 → 緊急停止
        motor.stop();
    }

    // ========== 3. 更新感測器資料 (10Hz) ==========
    if (currentTime - lastSensorTime >= SENSOR_UPDATE_INTERVAL) {
        updateSensors();
        sendSensorData();
        lastSensorTime = currentTime;
    }
}

// ==================== 處理馬達指令 ====================
void processMotorCommand() {
    int16_t leftPwm, rightPwm;
    bool vacuumState;

    // 解析封包
    if (parseMotorPacket(rxBuffer, leftPwm, rightPwm, vacuumState)) {
        // 封包有效
        motor.setLeftMotor(leftPwm);
        motor.setRightMotor(rightPwm);
        vacuum.setState(vacuumState);

        lastCommandTime = millis();  // 更新最後指令時間

        #ifdef DEBUG_SHOW_COMMANDS
        DEBUG_PRINT(F("[CMD] L:"));
        DEBUG_PRINT(leftPwm);
        DEBUG_PRINT(F(" R:"));
        DEBUG_PRINT(rightPwm);
        DEBUG_PRINT(F(" V:"));
        DEBUG_PRINTLN(vacuumState ? F("ON") : F("OFF"));
        #endif

        #ifdef DEBUG_VERBOSE
        // 顯示原始封包（詳細模式）
        DEBUG_PRINT(F("  Raw: "));
        for (int i = 0; i < 8; i++) {
            if (rxBuffer[i] < 0x10) DEBUG_PRINT(F("0"));
            DEBUG_PRINT(rxBuffer[i], HEX);
            DEBUG_PRINT(F(" "));
        }
        DEBUG_PRINTLN();
        #endif
    }
    else {
        // 封包無效 - 顯示錯誤封包內容
        DEBUG_PRINT(F("[ERR] Invalid packet: "));
        for (int i = 0; i < 8; i++) {
            if (rxBuffer[i] < 0x10) DEBUG_PRINT(F("0"));
            DEBUG_PRINT(rxBuffer[i], HEX);
            DEBUG_PRINT(F(" "));
        }
        DEBUG_PRINTLN();
    }
}

// ==================== 更新感測器資料 ====================
void updateSensors() {
    // 讀取左側超聲波
    leftDistance = leftUltrasonic.getDistance();

    // 等待一下再讀取右側（避免干擾）
    delay(10);

    // 讀取右側超聲波
    rightDistance = rightUltrasonic.getDistance();
}

// ==================== 發送感測器資料 ====================
void sendSensorData() {
    uint8_t packet[PACKET_SIZE];

    // 建構狀態旗標
    uint8_t status = 0;
    if (leftDistance != 999) status |= 0x01;   // bit0: 左側有效
    if (rightDistance != 999) status |= 0x02;  // bit1: 右側有效
    if (vacuum.getState()) status |= 0x08;     // bit3: 吸塵器狀態

    // 建構封包
    buildSensorPacket(packet, leftDistance, rightDistance, status);

    // 發送封包
    piSerial.write(packet, PACKET_SIZE);

    #ifdef DEBUG_SHOW_SENSORS
    // 除錯輸出 - 顯示感測器讀值
    DEBUG_PRINT(F("[SENSOR] L:"));
    DEBUG_PRINT(leftDistance);
    DEBUG_PRINT(F("cm "));
    DEBUG_PRINT((status & 0x01) ? F("✓") : F("✗"));
    DEBUG_PRINT(F(" R:"));
    DEBUG_PRINT(rightDistance);
    DEBUG_PRINT(F("cm "));
    DEBUG_PRINTLN((status & 0x02) ? F("✓") : F("✗"));
    #endif
}
