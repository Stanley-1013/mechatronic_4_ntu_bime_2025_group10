// main.ino - V3 自走吸塵車 (雙右側超音波版)
// 版本: 3.6
// 日期: 2025-12-08
//
// 變更：
// - 使用 UltrasonicManager 統一管理 3 個超音波
// - 輪流讀取避免干擾 (每迴圈讀 1 個)
// - behavior 改用 SensorData 結構
// - v3.0: 加入 MPU6050 IMU 用於轉彎角度判定
// - v3.5: IMU 更新移到 behavior 內部，只在轉彎時更新
// - v3.6: IMU 獨立高頻更新 (不受主迴圈 20Hz 限制)

#include "config.h"
#include "ultrasonic.h"
#include "mpu6050_sensor.h"
#include "behavior.h"
#include "motor.h"
#include "vacuum.h"
#include "serial_cmd.h"

// 模組實例
UltrasonicManager ultrasonic;
MPU6050Sensor imu;
BehaviorController behavior;
Motor motor;
Vacuum vacuum;
SerialCommand serialCmd;

// 上次迴圈時間
unsigned long lastLoopTime = 0;
unsigned long lastImuTime = 0;  // IMU 獨立計時
const unsigned long IMU_INTERVAL = 20;  // 20ms = 50Hz

void setup() {
    // 延遲啟動 (避免 EMI)
    delay(3000);

    // 初始化模組
    ultrasonic.init();

    // 初始化 IMU
    if (imu.begin()) {
        imu.calibrate(500);  // 校準陀螺儀 (約 1 秒，需靜止)
    }

    behavior.init(&imu);  // 傳入 IMU 指標
    motor.init();
    vacuum.init();
    serialCmd.init(115200);

    // 開啟吸塵器
    vacuum.on();

    lastLoopTime = millis();
}

void loop() {
    unsigned long currentTime = millis();

    // ===== IMU 獨立高頻更新 (50Hz，不受主迴圈限制) =====
    if (currentTime - lastImuTime >= IMU_INTERVAL) {
        imu.update();
        lastImuTime = currentTime;
    }

    // 維持 20Hz 迴圈 (超音波、行為控制)
    if (currentTime - lastLoopTime < LOOP_INTERVAL) {
        return;
    }
    lastLoopTime = currentTime;

    // ===== 1. 讀取超音波感測器 (輪流) =====
    ultrasonic.update();
    SensorData sensor = ultrasonic.getData();

    // ===== 3. 檢查 Serial 指令 =====
    if (serialCmd.check()) {
        byte cmd = serialCmd.getCommand();
        if (cmd == CMD_VACUUM_ON) {
            vacuum.on();
        } else if (cmd == CMD_VACUUM_OFF) {
            vacuum.off();
        }
    }

    // ===== 3. 行為控制 =====
    MotorCommand cmd = behavior.update(sensor);

    // ===== 4. 輸出馬達 =====
    if (cmd.stop || behavior.isComplete()) {
        motor.stop();
        vacuum.off();
    } else {
        motor.set(cmd.leftPWM, cmd.rightPWM);
    }
}
