// main.ino - V3 自走吸塵車 (雙右側超音波版)
// 版本: 2.0
// 日期: 2025-12-07
//
// 變更：
// - 使用 UltrasonicManager 統一管理 3 個超音波
// - 輪流讀取避免干擾 (每迴圈讀 1 個)
// - behavior 改用 SensorData 結構

#include "config.h"
#include "ultrasonic.h"
#include "behavior.h"
#include "motor.h"
#include "vacuum.h"
#include "serial_cmd.h"

// 模組實例
UltrasonicManager ultrasonic;
BehaviorController behavior;
Motor motor;
Vacuum vacuum;
SerialCommand serialCmd;

// 上次迴圈時間
unsigned long lastLoopTime = 0;

void setup() {
    // 延遲啟動 (避免 EMI)
    delay(3000);

    // 初始化模組
    ultrasonic.init();
    behavior.init();
    motor.init();
    vacuum.init();
    serialCmd.init(115200);

    // 開啟吸塵器
    vacuum.on();

    lastLoopTime = millis();
}

void loop() {
    unsigned long currentTime = millis();

    // 維持 20Hz 迴圈
    if (currentTime - lastLoopTime < LOOP_INTERVAL) {
        return;
    }
    lastLoopTime = currentTime;

    // ===== 1. 讀取感測器 (輪流) =====
    ultrasonic.update();
    SensorData sensor = ultrasonic.getData();

    // ===== 2. 檢查 Serial 指令 =====
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
