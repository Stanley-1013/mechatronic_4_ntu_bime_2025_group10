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
#include <avr/wdt.h>  // Watchdog Timer

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
unsigned long lastImuUpdate = 0;   // IMU 獨立更新計時
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

        case CMD_SET_PID:
            // 設定 PID 參數 (payload: Kp*1000, Ki*1000, Kd*1000 各 2 bytes, 共 6 bytes)
            if (payloadLen >= 6) {
                uint16_t kpInt = (payload[0] << 8) | payload[1];
                uint16_t kiInt = (payload[2] << 8) | payload[3];
                uint16_t kdInt = (payload[4] << 8) | payload[5];
                float kp = kpInt / 1000.0;
                float ki = kiInt / 1000.0;
                float kd = kdInt / 1000.0;
                wallFollower.setPID(kp, ki, kd);
                Serial.print(F("[CMD] SET_PID - Kp="));
                Serial.print(kp, 3);
                Serial.print(F(" Ki="));
                Serial.print(ki, 3);
                Serial.print(F(" Kd="));
                Serial.println(kd, 3);
            }
            break;

        case CMD_SET_PARAMS:
            // 設定所有控制參數 (payload: 32 bytes, little-endian)
            // 格式: 4 dist + 12 speed + 6 pid + 4 misc + 4 time + 2 reserved = 32
            if (payloadLen >= 32) {
                // 解析距離閾值 (4 uint8, offset 0-3)
                uint8_t targetRightDist = payload[0];
                uint8_t frontStopDist = payload[1];
                uint8_t frontSlowDist = payload[2];
                uint8_t cornerRightDist = payload[3];

                // 解析速度參數 (6 int16, little-endian, * 100, offset 4-15)
                int16_t baseLinearRaw = (int16_t)(payload[4] | (payload[5] << 8));
                int16_t backupRaw = (int16_t)(payload[6] | (payload[7] << 8));
                int16_t turnAngularRaw = (int16_t)(payload[8] | (payload[9] << 8));
                int16_t findWallLinRaw = (int16_t)(payload[10] | (payload[11] << 8));
                int16_t leftMotorScaleRaw = (int16_t)(payload[12] | (payload[13] << 8));
                int16_t rightMotorScaleRaw = (int16_t)(payload[14] | (payload[15] << 8));

                float baseLinearSpeed = baseLinearRaw / 100.0;
                float backupSpeed = backupRaw / 100.0;
                float turnAngularSpeed = turnAngularRaw / 100.0;
                float findWallLinear = findWallLinRaw / 100.0;
                float leftMotorScale = leftMotorScaleRaw / 100.0;
                float rightMotorScale = rightMotorScaleRaw / 100.0;

                // 解析 PID (3 uint16, little-endian, * 1000, offset 16-21)
                uint16_t kpRaw = (uint16_t)(payload[16] | (payload[17] << 8));
                uint16_t kiRaw = (uint16_t)(payload[18] | (payload[19] << 8));
                uint16_t kdRaw = (uint16_t)(payload[20] | (payload[21] << 8));

                float kp = kpRaw / 1000.0;
                float ki = kiRaw / 1000.0;
                float kd = kdRaw / 1000.0;

                // 解析其他參數 (offset 22-25)
                uint8_t minEffectivePWM = payload[22];
                uint8_t cornerTurnAngleInt = payload[23];
                uint8_t redAvoidAngleInt = payload[24];
                // payload[25] = padding

                // 時間參數 (offset 26-29)
                uint16_t backupDurationMs = (uint16_t)(payload[26] | (payload[27] << 8));
                uint16_t turnTimeoutMs = (uint16_t)(payload[28] | (payload[29] << 8));
                // payload[30-31] = reserved

                // 呼叫 setParams
                wallFollower.setParams(
                    targetRightDist,
                    frontStopDist,
                    frontSlowDist,
                    cornerRightDist,
                    baseLinearSpeed,
                    backupSpeed,
                    turnAngularSpeed,
                    findWallLinear,
                    leftMotorScale,
                    rightMotorScale,
                    kp,
                    ki,
                    kd,
                    minEffectivePWM,
                    (float)cornerTurnAngleInt,
                    (float)redAvoidAngleInt,
                    backupDurationMs,
                    turnTimeoutMs
                );

                Serial.println(F("[CMD] SET_PARAMS - All parameters updated"));
                Serial.print(F("  Dist: R="));
                Serial.print(targetRightDist);
                Serial.print(F(" FStop="));
                Serial.print(frontStopDist);
                Serial.print(F(" FSlow="));
                Serial.print(frontSlowDist);
                Serial.print(F(" Corner="));
                Serial.println(cornerRightDist);
                Serial.print(F("  Speed: base="));
                Serial.print(baseLinearSpeed, 2);
                Serial.print(F(" turn="));
                Serial.print(turnAngularSpeed, 2);
                Serial.print(F(" L_scale="));
                Serial.print(leftMotorScale, 2);
                Serial.print(F(" R_scale="));
                Serial.println(rightMotorScale, 2);
                Serial.print(F("  PID: Kp="));
                Serial.print(kp, 3);
                Serial.print(F(" Ki="));
                Serial.print(ki, 3);
                Serial.print(F(" Kd="));
                Serial.println(kd, 3);
            } else {
                Serial.print(F("[CMD] SET_PARAMS - Invalid payload length: "));
                Serial.println(payloadLen);
            }
            break;

        default:
            Serial.print(F("[CMD] Unknown command: 0x"));
            Serial.println(cmd, HEX);
            break;
    }
}

// ==================== Setup 函數 ====================
void setup() {
    // !! 緊急: 先停用 Watchdog (如果是 watchdog 重置的話)
    MCUSR = 0;
    wdt_disable();

    // !! 緊急: 第一時間強制馬達停止 (避免 EMI 重置後繼續轉)
    // 直接操作 L298N 腳位，不等待任何初始化
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    // 強制停止: PWM = 0, 方向 = LOW
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);

    // 等待馬達完全停止，讓 EMI 消退
    delay(200);

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

    // 初始化馬達驅動 (會再次呼叫 stop())
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

    // 啟用 Watchdog Timer (2 秒超時)
    // 如果 loop() 卡住超過 2 秒，Arduino 會自動重置
    wdt_enable(WDTO_2S);
    Serial.println(F("[OK] Watchdog timer enabled (2s)"));
}

// ==================== Main Loop 函數 ====================
void loop() {
    // !! 餵狗: 每次 loop 都要餵，否則 2 秒後會重置
    wdt_reset();

    // 1. 處理來自 Pi 的指令
    serialHandler.process();

    // 2. 更新感測器讀值 (交替讀取，避免阻塞)
    if (readFrontNext) {
        frontDist = usFront.getDistance();
    } else {
        rightDist = usRight.getDistance();
    }
    readFrontNext = !readFrontNext;

    // IMU 獨立高頻更新 (50Hz = 20ms 間隔，確保陀螺儀積分精度)
    bool currentImuValid = false;
    #ifdef MPU6050_ENABLED
    if (imuValid) {
        unsigned long currentTime = millis();
        if (currentTime - lastImuUpdate >= IMU_UPDATE_INTERVAL) {
            imu.update();
            lastImuUpdate = currentTime;
        }

        // 檢查 IMU 健康狀態 (EMI 可能導致 I2C 錯誤)
        if (imu.isHealthy()) {
            yaw = imu.getYaw();
            currentImuValid = true;
        } else {
            // IMU 失效，讓 WallFollower 自動降級到 Bang-Bang 控制
            // 不停車，繼續用超聲波沿牆
            currentImuValid = false;
            // 僅首次失效時警告
            static bool imuWarnPrinted = false;
            if (!imuWarnPrinted) {
                Serial.println(F("[WARN] IMU failed, degraded to ultrasonic-only mode"));
                imuWarnPrinted = true;
            }
        }
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

    // 4. 定期回報狀態給 Pi (帶 TX 緩衝區檢查，避免阻塞)
    if (millis() - lastStatusReport >= STATUS_REPORT_INTERVAL) {
        // 檢查 TX 緩衝區是否有足夠空間 (PKT_STATE_LENGTH = 12 bytes)
        if (Serial.availableForWrite() >= PKT_STATE_LENGTH) {
            sendStatusReport();
            lastStatusReport = millis();
        }
        // 若緩衝區不足，跳過本次發送，下次再試
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
