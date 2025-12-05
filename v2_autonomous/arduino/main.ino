/*
 * v2_autonomous/arduino/main.ino - Arduino 自主沿牆控制主程式
 * 版本: 1.0
 * 日期: 2025-12-02
 *
 * 架構: Arduino 自主執行沿牆狀態機，Raspberry Pi 發送高階事件指令
 *
 * 功能:
 * - 接收 Pi 的通訊協定指令 (啟動/停止/避紅色/吸塵器控制)
 * - 讀取前方與右側超聲波感測器
 * - 讀取 MPU6050 IMU (Yaw 角度控制)
 * - 執行沿牆狀態機，控制左右馬達
 * - 控制吸塵器馬達
 * - 定期回報狀態給 Pi
 *
 * 參考: protocol.h (PI -> Arduino 指令格式)
 *      wall_follower.h (狀態機)
 *      motor_driver.h (馬達控制)
 */

// ==================== 包含標頭檔 ====================
#include "config.h"
#include "protocol.h"
#include "serial_handler.h"
#include "wall_follower.h"
#include "motor_driver.h"
#include "ultrasonic_sensor.h"
#include "vacuum_controller.h"

#ifdef MPU6050_ENABLED
#include "mpu6050_sensor.h"
#endif

// ==================== 模組實例 ====================
MotorDriver motor(PIN_ENA, PIN_IN1, PIN_IN2, PIN_ENB, PIN_IN3, PIN_IN4);
UltrasonicSensor usFront(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
UltrasonicSensor usRight(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);
VacuumController vacuum(PIN_VACUUM);
WallFollower wallFollower;
SerialHandler serialHandler;

#ifdef MPU6050_ENABLED
MPU6050Sensor imu;
#endif

// ==================== 全域狀態變數 ====================
bool systemRunning = false;        // 系統是否在執行沿牆
unsigned long lastStatusReport = 0;
const unsigned long STATUS_REPORT_INTERVAL = 100; // 100ms = 10Hz

// 感測器讀值
int frontDist = 999;
int rightDist = 999;
float yaw = 0.0;
bool imuValid = false;

// 交替讀取超聲波 (避免阻塞)
bool readFrontNext = true;

// ==================== 指令回調函數 ====================
/**
 * 處理來自 Pi 的指令回調
 */
void onCommandReceived(uint8_t cmd, uint8_t payloadLen, uint8_t* payload) {
    switch (cmd) {
        case CMD_START:
            // 啟動自主沿牆控制
            systemRunning = true;
            wallFollower.start();
            vacuum.setState(true);  // 自走模式吸塵器常開
            Serial.println(F("[CMD] START - Wall follower activated, vacuum ON"));
            break;

        case CMD_STOP:
            // 停止執行
            systemRunning = false;
            wallFollower.stop();
            motor.stop();
            vacuum.setState(false);
            Serial.println(F("[CMD] STOP - System halted"));
            break;

        case CMD_AVOID_RED:
            // 迴避紅色區域（預留介面）
            wallFollower.triggerAvoidRed();
            Serial.println(F("[CMD] AVOID_RED - Trigger red avoidance"));
            break;

        case CMD_SET_VACUUM:
            // 設定吸塵器狀態
            if (payloadLen >= 1) {
                bool vacuumState = (payload[0] != 0);
                vacuum.setState(vacuumState);
                Serial.print(F("[CMD] SET_VACUUM - "));
                Serial.println(vacuumState ? F("ON") : F("OFF"));
            }
            break;

        case CMD_QUERY_STATE:
            // 查詢當前狀態 - sendStatusReport() 會在 loop 中定期發送
            Serial.println(F("[CMD] QUERY_STATE"));
            break;

        default:
            Serial.print(F("[CMD] Unknown command: 0x"));
            Serial.println(cmd, HEX);
            break;
    }
}

// ==================== Setup 函數 ====================
void setup() {
    // 初始化硬體 Serial (USB，與 Pi 通訊)
    Serial.begin(115200);
    delay(100);

    // 啟動序列
    Serial.println();
    Serial.println(F("========================================="));
    Serial.println(F(" v2 Autonomous Wall Follower"));
    Serial.println(F(" Arduino Controller"));
    Serial.println(F("========================================="));
    Serial.println(F("Initializing modules..."));

    // 初始化馬達驅動
    motor.begin();
    Serial.println(F("[OK] Motor driver initialized"));

    // 初始化超聲波感測器
    usFront.begin();
    usRight.begin();
    Serial.println(F("[OK] Ultrasonic sensors initialized"));

    // 初始化吸塵器控制
    vacuum.begin();
    Serial.println(F("[OK] Vacuum controller initialized"));

    // 初始化 MPU6050 IMU
    #ifdef MPU6050_ENABLED
    Serial.print(F("[IMU] Initializing MPU6050... "));
    if (imu.begin()) {
        imuValid = true;
        Serial.println(F("OK"));

        #ifdef MPU6050_CALIBRATE_ON_BOOT
        Serial.println(F("[IMU] Calibrating gyroscope (keep robot still)..."));
        imu.calibrate(IMU_CALIBRATION_SAMPLES);
        Serial.println(F("[IMU] Calibration complete"));
        #endif
    } else {
        imuValid = false;
        Serial.println(F("FAILED - Continuing without IMU"));
    }
    #else
    Serial.println(F("[IMU] Disabled in config.h"));
    #endif

    // 初始化通訊協定
    serialHandler.begin(115200);
    serialHandler.setCommandCallback(onCommandReceived);
    Serial.println(F("[OK] Serial handler ready"));

    // 初始化狀態
    systemRunning = false;
    motor.stop();
    vacuum.setState(false);

    Serial.println(F("========================================="));
    Serial.println(F("Ready for commands from Raspberry Pi"));
    Serial.println(F("========================================="));
    Serial.println();
}

// ==================== Main Loop 函數 ====================
void loop() {
    // 1. 處理來自 Pi 的指令
    serialHandler.process();

    // 2. 更新感測器讀值 (交替讀取，避免阻塞)
    if (readFrontNext) {
        frontDist = usFront.getDistance();
    } else {
        rightDist = usRight.getDistance();
    }
    readFrontNext = !readFrontNext;

    yaw = 0;
    bool currentImuValid = false;
    #ifdef MPU6050_ENABLED
    if (imuValid) {
        imu.update();
        yaw = imu.getYaw();
        currentImuValid = true;
    }
    #endif

    // 3. 執行沿牆狀態機 (如果已啟動)
    if (systemRunning) {
        // 呼叫狀態機更新
        wallFollower.update(frontDist, rightDist, yaw, currentImuValid);

        // 套用馬達輸出
        int leftPWM = wallFollower.getLeftPWM();
        int rightPWM = wallFollower.getRightPWM();
        motor.setLeftMotor(leftPWM);
        motor.setRightMotor(rightPWM);

        // 吸塵器狀態由 wallFollower 控制，但可由 CMD_SET_VACUUM 覆蓋
        // 暫時讓 wallFollower 決定
    }

    // 4. 定期回報狀態給 Pi
    if (millis() - lastStatusReport >= STATUS_REPORT_INTERVAL) {
        sendStatusReport();
        lastStatusReport = millis();
    }
}

// ==================== 狀態回報函數 ====================
/**
 * 構建並發送狀態封包給 Pi
 */
void sendStatusReport() {
    // 取得狀態機當前狀態
    WallFollowerState wfState = wallFollower.getState();
    uint8_t state = (uint8_t)wfState;  // 枚舉值對應 protocol.h 的 STATE_* 常數
    uint8_t cornerCount = wallFollower.getCornerCount();
    bool vacuumEnabled = vacuum.getState();

    // 構建 FLAGS
    uint8_t flags = 0;
    if (vacuumEnabled) {
        flags |= FLAG_VACUUM_ENABLED;
    }
    // 如果後續實作紅色偵測，可在此設定 FLAG_RED_DETECTED

    // 轉換 yaw 為 int16_t (0.1 度精度: 乘以 10)
    int16_t yawInt = (int16_t)(yaw * 10);

    // 使用 SerialHandler 的 sendState 函數
    serialHandler.sendState(
        state,
        cornerCount,
        (uint16_t)frontDist,
        (uint16_t)rightDist,
        yawInt,
        flags
    );
}

// ==================== 應急停止 (可選) ====================
/**
 * 發生 error 時呼叫此函數
 */
void emergencyStop() {
    systemRunning = false;
    wallFollower.stop();
    motor.stop();
    vacuum.setState(false);
    Serial.println(F("[ERROR] Emergency stop triggered!"));
}
