/*
 * main.ino - Arduino 主程式
 * 版本: 2.0 (新增 MPU6050 IMU 支援)
 * 日期: 2025-11-29
 *
 * 機電小車 Arduino 控制程式
 * 功能:
 * - 接收 Raspberry Pi 的馬達指令 (硬體 Serial via USB)
 * - 控制 L298N 驅動雙馬達
 * - 讀取前方與右側超聲波感測器
 * - 讀取 MPU6050 IMU (Yaw 角度、角速度)
 * - 控制吸塵器馬達
 * - 回傳感測器資料給 Pi (12-byte 封包含 IMU)
 *
 * 參考: 03_SD_系統設計.md, 04_ICD_介面規格.md
 */

// 不再使用 SoftwareSerial，改用硬體 Serial (USB)
// #include <SoftwareSerial.h>
#include "config.h"             // ← 使用 config.h 管理所有設定
#include "motor_driver.h"
#include "ultrasonic_sensor.h"
#include "vacuum_controller.h"
#include "serial_protocol.h"

#ifdef MPU6050_ENABLED
#include "mpu6050_sensor.h"
#endif

// ==================== 物件初始化 ====================
// 使用硬體 Serial (Serial) 而非 SoftwareSerial
// SoftwareSerial piSerial(PIN_SERIAL_RX, PIN_SERIAL_TX);

MotorDriver motor(PIN_ENA, PIN_IN1, PIN_IN2, PIN_ENB, PIN_IN3, PIN_IN4);
UltrasonicSensor frontUltrasonic(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
UltrasonicSensor rightUltrasonic(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);
VacuumController vacuum(PIN_VACUUM);

#ifdef MPU6050_ENABLED
MPU6050Sensor imu;
#endif

// ==================== 全域變數 ====================
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
unsigned long lastCommandTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastImuTime = 0;

uint16_t frontDistance = 999;
uint16_t rightDistance = 999;
bool ultrasonicEnabled = false;  // 超聲波啟用旗標（由 Pi 控制）
bool imuInitialized = false;     // IMU 初始化旗標

// ==================== Setup ====================
void setup() {
    // 初始化硬體 Serial (USB，與 Pi 通訊)
    Serial.begin(115200);  // 高速 baud rate，減少 buffer 塞車

    Serial.println(F("========================================="));
    Serial.println(F(" Arduino Robot Controller v2.0"));
    Serial.println(F(" (MPU6050 IMU Support)"));
    Serial.println(F("========================================="));
    Serial.println(F("Initializing..."));

    Serial.println(F("[OK] Serial @ 115200 bps"));

    // 初始化馬達驅動
    motor.begin();
    DEBUG_PRINTLN(F("[OK] Motor driver"));

    // 初始化超聲波感測器
    frontUltrasonic.begin();
    rightUltrasonic.begin();
    DEBUG_PRINTLN(F("[OK] Ultrasonic sensors"));

    // 初始化吸塵器控制
    vacuum.begin();
    DEBUG_PRINTLN(F("[OK] Vacuum controller"));

    // 初始化 MPU6050 IMU
    #ifdef MPU6050_ENABLED
    Serial.print(F("[IMU] Initializing MPU6050... "));
    if (imu.begin()) {
        imuInitialized = true;
        Serial.println(F("OK"));

        #ifdef MPU6050_CALIBRATE_ON_BOOT
        Serial.println(F("[IMU] Calibrating gyroscope (keep robot still)..."));
        imu.calibrate(IMU_CALIBRATION_SAMPLES);
        Serial.println(F("[IMU] Calibration complete"));
        #endif
    } else {
        imuInitialized = false;
        Serial.println(F("FAILED!"));
        Serial.println(F("[IMU] Will continue without IMU support"));
    }
    #else
    Serial.println(F("[IMU] MPU6050 disabled in config"));
    #endif

    // 顯示設定資訊
    #ifdef DEBUG_SERIAL_ENABLED
    DEBUG_PRINTLN(F("\n[Config]"));
    DEBUG_PRINT(F("  Sensor interval: "));
    DEBUG_PRINT(SENSOR_UPDATE_INTERVAL);
    DEBUG_PRINTLN(F(" ms"));
    DEBUG_PRINT(F("  Command timeout: "));
    DEBUG_PRINT(COMMAND_TIMEOUT);
    DEBUG_PRINTLN(F(" ms"));
    DEBUG_PRINT(F("  IMU interval: "));
    DEBUG_PRINT(IMU_UPDATE_INTERVAL);
    DEBUG_PRINTLN(F(" ms"));
    #endif

    Serial.println(F("\n========================================="));
    Serial.println(F(" System Ready - Waiting for commands"));
    Serial.println(F("=========================================\n"));

    lastCommandTime = millis();
    lastSensorTime = millis();
    lastImuTime = millis();
}

// ==================== Main Loop ====================
void loop() {
    unsigned long currentTime = millis();

    // ========== 1. 接收並處理馬達指令 ==========
    // 防止緩衝區溢出：只處理最多 3 個封包，避免長時間阻塞
    int packetsProcessed = 0;
    while (Serial.available() && packetsProcessed < 3) {
        uint8_t byte = Serial.read();

        // 尋找 Header
        if (rxIndex == 0) {
            if (byte == MOTOR_HEADER) {
                rxBuffer[rxIndex++] = byte;
            }
            // 若緩衝區太多資料，清空舊資料（避免處理過時的指令）
            else if (Serial.available() > 40) {
                while (Serial.available() > 8) {
                    Serial.read();  // 丟棄過時資料
                }
            }
        }
        else {
            rxBuffer[rxIndex++] = byte;

            // 收齊 8 bytes
            if (rxIndex >= PACKET_SIZE) {
                processMotorCommand();
                rxIndex = 0;
                packetsProcessed++;
            }
        }
    }

    // ========== 2. 逾時保護 ==========
    if (currentTime - lastCommandTime > COMMAND_TIMEOUT) {
        // 超過 200ms 未收到指令 → 緊急停止
        motor.stop();
    }

    // ========== 3. 更新 IMU 資料 (高頻率) ==========
    #ifdef MPU6050_ENABLED
    if (imuInitialized && currentTime - lastImuTime >= IMU_UPDATE_INTERVAL) {
        imu.update();
        lastImuTime = currentTime;
    }
    #endif

    // ========== 4. 更新感測器資料並發送 (僅在啟用時) ==========
    // 只有當 Pi 啟用超聲波時才讀取，避免遙控模式阻塞
    if (ultrasonicEnabled && currentTime - lastSensorTime >= 50) {
        // 使用交替讀取減少阻塞：每次只讀一個感測器，最多阻塞 15ms
        updateSensors();
        sendSensorData();
        lastSensorTime = currentTime;
    }
}

// ==================== 處理馬達指令 ====================
void processMotorCommand() {
    int16_t leftPwm, rightPwm;
    bool vacuumState;
    bool newUltrasonicEnabled;

    // 解析封包 (包含 ultrasonic_enable flag)
    if (parseMotorPacket(rxBuffer, leftPwm, rightPwm, vacuumState, newUltrasonicEnabled)) {
        // 封包有效
        motor.setLeftMotor(leftPwm);
        motor.setRightMotor(rightPwm);
        vacuum.setState(vacuumState);
        ultrasonicEnabled = newUltrasonicEnabled;  // 更新超聲波啟用狀態

        lastCommandTime = millis();  // 更新最後指令時間

        #ifdef DEBUG_SHOW_COMMANDS
        DEBUG_PRINT(F("[CMD] L:"));
        DEBUG_PRINT(leftPwm);
        DEBUG_PRINT(F(" R:"));
        DEBUG_PRINT(rightPwm);
        DEBUG_PRINT(F(" V:"));
        DEBUG_PRINT(vacuumState ? F("ON") : F("OFF"));
        DEBUG_PRINT(F(" US:"));
        DEBUG_PRINTLN(ultrasonicEnabled ? F("ON") : F("OFF"));
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
// 交替讀取：每次 loop 只讀一個感測器，減少阻塞
bool readFrontNext = true;  // 交替旗標

void updateSensors() {
    if (readFrontNext) {
        // 讀取前方超聲波
        frontDistance = frontUltrasonic.getDistance();
    } else {
        // 讀取右側超聲波
        rightDistance = rightUltrasonic.getDistance();
    }
    readFrontNext = !readFrontNext;  // 切換下次讀取目標
}

// ==================== 發送感測器資料 ====================
void sendSensorData() {
    uint8_t packet[PACKET_SIZE_SENSOR];  // 使用新的 12-byte 封包

    // 建構狀態旗標
    uint8_t status = 0;
    if (frontDistance != 999) status |= 0x01;  // bit0: 前方有效
    if (rightDistance != 999) status |= 0x02;  // bit1: 右側有效
    #ifdef MPU6050_ENABLED
    if (imuInitialized) status |= 0x04;        // bit2: IMU 有效
    #endif
    if (vacuum.getState()) status |= 0x08;     // bit3: 吸塵器狀態

    // 取得 IMU 資料
    float yaw = 0.0f;
    float gyroZ = 0.0f;
    #ifdef MPU6050_ENABLED
    if (imuInitialized) {
        yaw = imu.getYaw();
        gyroZ = imu.getGyroZ();
    }
    #endif

    // 建構封包 (v2.0 - 12 bytes，包含 IMU)
    buildSensorPacketV2(packet, frontDistance, rightDistance, yaw, gyroZ, status);

    // 發送封包
    Serial.write(packet, PACKET_SIZE_SENSOR);

    #ifdef DEBUG_SHOW_SENSORS
    // 除錯輸出 - 顯示感測器讀值
    DEBUG_PRINT(F("[SENSOR] F:"));
    DEBUG_PRINT(frontDistance);
    DEBUG_PRINT(F("cm "));
    DEBUG_PRINT((status & 0x01) ? F("✓") : F("✗"));
    DEBUG_PRINT(F(" R:"));
    DEBUG_PRINT(rightDistance);
    DEBUG_PRINT(F("cm "));
    DEBUG_PRINT((status & 0x02) ? F("✓") : F("✗"));
    DEBUG_PRINT(F(" Yaw:"));
    DEBUG_PRINT(yaw, 1);
    DEBUG_PRINT(F("° Gz:"));
    DEBUG_PRINT(gyroZ, 1);
    DEBUG_PRINTLN(F("°/s"));
    #endif
}
