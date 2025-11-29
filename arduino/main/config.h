/*
 * config.h - Arduino 系統設定檔
 * 版本: 1.0
 * 日期: 2025-10-31
 *
 * 集中管理所有腳位定義、參數設定與除錯選項
 * 修改此檔案後需重新上傳至 Arduino
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==================== 腳位定義 ====================
// L298N 馬達驅動
#define PIN_ENA 3   // 左輪 PWM (必須是 PWM 腳位)
#define PIN_IN1 6   // 左輪方向 A (匹配實際硬體)
#define PIN_IN2 5   // 左輪方向 B (匹配實際硬體)
#define PIN_ENB 11  // 右輪 PWM (必須是 PWM 腳位)
#define PIN_IN3 9   // 右輪方向 A
#define PIN_IN4 10  // 右輪方向 B

// 超聲波感測器 (前方 + 右側，用於沿右牆行駛)
#define PIN_US_FRONT_TRIG  7   // 前方 Trig (偵測前方牆壁，轉彎用)
#define PIN_US_FRONT_ECHO  8   // 前方 Echo
#define PIN_US_RIGHT_TRIG  A1  // 右側 Trig (偵測右側牆壁，沿牆用)
#define PIN_US_RIGHT_ECHO  A2  // 右側 Echo

// 吸塵器馬達 (Relay)
#define PIN_VACUUM A3  // 吸塵器繼電器控制腳位 (改為 A3)

// MPU6050 IMU (I2C)
// A4 = SDA, A5 = SCL (Arduino Uno 預設 I2C 腳位，無需額外定義)
#define MPU6050_ENABLED        // 啟用 MPU6050
#define MPU6050_CALIBRATE_ON_BOOT  // 開機時自動校準

// SoftwareSerial (與 Raspberry Pi 通訊)
#define PIN_SERIAL_RX 4  // 接收 (連接 Pi TXD GPIO14)
#define PIN_SERIAL_TX 2  // 發送 (連接 Pi RXD GPIO15)

// ==================== 通訊參數 ====================
#define SERIAL_BAUDRATE 57600  // 與 Pi 的鮑率（必須與 Python 一致）
#define SENSOR_UPDATE_INTERVAL 100  // ms (感測器更新頻率 = 10Hz)
#define COMMAND_TIMEOUT 200         // ms (指令逾時 - 超過此時間未收到指令則停止馬達)

// ==================== 除錯設定 ====================
// 註解以下行可關閉對應的除錯輸出

// #define DEBUG_SERIAL_ENABLED   // 啟用 USB Serial 除錯 - 關閉以避免阻塞
// #define DEBUG_SHOW_COMMANDS    // 顯示收到的馬達指令 - 關閉以避免阻塞
// #define DEBUG_SHOW_SENSORS     // 顯示感測器讀值 - 關閉以避免阻塞
// #define DEBUG_VERBOSE       // 詳細輸出（顯示原始封包），測試時可啟用

// ==================== 感測器參數 ====================
#define ULTRASONIC_MIN_DISTANCE 2    // cm (最小有效距離)
#define ULTRASONIC_MAX_DISTANCE 400  // cm (最大有效距離)
#define ULTRASONIC_TIMEOUT 15000     // μs (逾時時間 = 15ms，減少阻塞)

// MPU6050 參數
#define IMU_UPDATE_INTERVAL 20       // ms (IMU 更新間隔 = 50Hz)
#define IMU_CALIBRATION_SAMPLES 500  // 校準取樣次數

// ==================== 安全參數 ====================
// #define ENABLE_AUTO_RESET  // 啟用自動重置（10秒無指令時重置 Arduino）

// ==================== 輔助宏 ====================
// 條件編譯輔助宏
#ifdef DEBUG_SERIAL_ENABLED
    #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...)    // 接受任意參數但不做任何事
    #define DEBUG_PRINTLN(...)  // 接受任意參數但不做任何事
#endif

#endif // CONFIG_H
