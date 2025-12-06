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
#define PIN_IN3 10  // 右輪方向 A (對調以修正方向)
#define PIN_IN4 9   // 右輪方向 B (對調以修正方向)

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
#define SERIAL_BAUDRATE 115200  // 與 Pi 的鮑率（必須與 Python 一致）
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
#define ULTRASONIC_TIMEOUT 8000      // μs (逾時時間 = 8ms，進一步減少阻塞)

// MPU6050 參數
#define IMU_UPDATE_INTERVAL 20       // ms (IMU 更新間隔 = 50Hz)
#define IMU_CALIBRATION_SAMPLES 500  // 校準取樣次數

// ==================== 安全參數 ====================
// #define ENABLE_AUTO_RESET  // 啟用自動重置（10秒無指令時重置 Arduino）

// ==================== 沿牆控制參數 (調參區) ====================
// 預設值定義 (用於初始化)
#define DEFAULT_TARGET_RIGHT_DIST    15   // 目標右側距離
#define DEFAULT_FRONT_STOP_DIST      20   // 前方停止/角落偵測距離
#define DEFAULT_FRONT_SLOW_DIST      40   // 前方減速距離
#define DEFAULT_CORNER_RIGHT_DIST    30   // 角落右側閾值

#define DEFAULT_BASE_LINEAR_SPEED    0.20f // 基礎前進速度 (降低以避免急停急走)
#define DEFAULT_BACKUP_SPEED         0.0f  // 後退速度 (已停用)
#define DEFAULT_TURN_ANGULAR_SPEED   0.25f // 角落轉彎角速度 (降低以平滑轉彎)
#define DEFAULT_FIND_WALL_LINEAR     0.35f // 尋牆前進速度
#define DEFAULT_LEFT_MOTOR_SCALE     1.0f  // 左輪速度倍率 (補償馬達差異)
#define DEFAULT_RIGHT_MOTOR_SCALE    1.0f  // 右輪速度倍率

#define DEFAULT_KP           0.025f // 比例增益 (超聲波沿牆，已停用)
#define DEFAULT_KI           0.002f // 積分增益 (已停用)
#define DEFAULT_KD           0.010f // 微分增益 (已停用)
#define INTEGRAL_MAX         5.0f   // 積分上限 (防 windup) - 固定值
#define ANGULAR_MAX          0.35f  // 角速度輸出上限 - 固定值

#define DEFAULT_GENTLE_ANGULAR  0.12f  // 沿牆緩慢修正角速度
#define DEFAULT_ERROR_DEADZONE  3.0f   // 誤差死區 (cm)

// ==================== 航向 PID 參數 (IMU 控制) ====================
#define DEFAULT_YAW_KP           0.015f  // 航向比例增益 (偏1°→0.015, 偏10°→0.15)
#define DEFAULT_YAW_KI           0.0f    // 航向積分增益 (暫不使用)
#define DEFAULT_YAW_KD           0.08f   // 航向微分增益 (抑制震盪)
#define YAW_DRIFT_RATE           0.15f   // 超聲波監督漂移修正速度 (度/秒)
#define DRIFT_THRESHOLD_TIME     0.5f    // 連續偏離觸發時間 (秒) - 提早偵測但緩慢修正

#define DEFAULT_MIN_EFFECTIVE_PWM    45     // 死區 (低於此值馬達不轉)

#define DEFAULT_CORNER_TURN_ANGLE    85.0f  // 角落轉彎角度
#define DEFAULT_RED_AVOID_ANGLE      45.0f  // 紅色迴避角度

#define DEFAULT_BACKUP_DURATION_MS   300    // 後退持續時間
#define DEFAULT_TURN_TIMEOUT_MS      3000   // 轉彎超時

// ==================== 可調參數 (extern 宣告) ====================
// 這些變數可由 Pi 透過 CMD_SET_PARAMS 修改
// 實際定義在 wall_follower.cpp

// 距離閾值 (cm)
extern uint8_t g_targetRightDist;
extern uint8_t g_frontStopDist;
extern uint8_t g_frontSlowDist;
extern uint8_t g_cornerRightDist;

// 速度參數 (-1.0 ~ +1.0)
extern float g_baseLinearSpeed;
extern float g_backupSpeed;
extern float g_turnAngularSpeed;
extern float g_findWallLinear;
extern float g_leftMotorScale;
extern float g_rightMotorScale;

// PWM 參數
extern uint8_t g_minEffectivePWM;

// 角落轉彎角度 (度)
extern float g_cornerTurnAngle;
extern float g_redAvoidAngle;

// Bang-Bang 沿牆控制
extern float g_gentleAngular;
extern float g_errorDeadzone;

// 航向 PID 參數 (IMU 控制)
extern float g_yawKp;
extern float g_yawKi;
extern float g_yawKd;

// 時間參數 (ms)
extern uint16_t g_backupDurationMs;
extern uint16_t g_turnTimeoutMs;

// ==================== 相容性宏 (供舊程式碼使用) ====================
// 這些宏將舊的 #define 名稱映射到新的全域變數
#define TARGET_RIGHT_DIST    g_targetRightDist
#define FRONT_STOP_DIST      g_frontStopDist
#define FRONT_SLOW_DIST      g_frontSlowDist
#define CORNER_RIGHT_DIST    g_cornerRightDist

#define BASE_LINEAR_SPEED    g_baseLinearSpeed
#define BACKUP_SPEED         g_backupSpeed
#define TURN_ANGULAR_SPEED   g_turnAngularSpeed
#define FIND_WALL_LINEAR     g_findWallLinear
#define LEFT_MOTOR_SCALE     g_leftMotorScale
#define RIGHT_MOTOR_SCALE    g_rightMotorScale

#define MIN_EFFECTIVE_PWM    g_minEffectivePWM

#define CORNER_TURN_ANGLE    g_cornerTurnAngle
#define RED_AVOID_ANGLE      g_redAvoidAngle

#define GENTLE_ANGULAR       g_gentleAngular
#define ERROR_DEADZONE       g_errorDeadzone

#define BACKUP_DURATION_MS   g_backupDurationMs
#define TURN_TIMEOUT_MS      g_turnTimeoutMs

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
