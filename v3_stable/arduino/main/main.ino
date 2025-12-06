// main.ino - V3 自走吸塵車
// 版本: 1.0
// 日期: 2025-12-06

#include "config.h"
#include "ultrasonic.h"
#include "behavior.h"
#include "motor.h"
#include "vacuum.h"
#include "serial_cmd.h"

// 模組實例
Ultrasonic frontUS;
Ultrasonic rightUS;
BehaviorController behavior;
Motor motor;
Vacuum vacuum;
SerialCommand serialCmd;

// 交替讀取旗標
bool readFrontNext = true;

// 感測器值
int frontDist = 100;
int rightDist = 15;

// 上次迴圈時間
unsigned long lastLoopTime = 0;

void setup() {
    // 延遲啟動 (避免 EMI)
    delay(3000);

    // 初始化模組
    frontUS.init(PIN_FRONT_TRIG, PIN_FRONT_ECHO);
    rightUS.init(PIN_RIGHT_TRIG, PIN_RIGHT_ECHO);
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

    // ===== 1. 讀取感測器 (交替) =====
    if (readFrontNext) {
        int val = frontUS.getFiltered();
        if (val > 0) frontDist = val;
    } else {
        int val = rightUS.getFiltered();
        if (val > 0) rightDist = val;
    }
    readFrontNext = !readFrontNext;

    // ===== 2. 檢查 Serial 指令 =====
    if (serialCmd.check()) {
        byte cmd = serialCmd.getCommand();
        if (cmd == CMD_VACUUM_ON) {
            vacuum.on();
        } else if (cmd == CMD_VACUUM_OFF) {
            vacuum.off();
        }
    }

    // ===== 3. 行為融合控制 =====
    MotorCommand cmd = behavior.update(frontDist, rightDist);

    // ===== 4. 輸出馬達 =====
    if (cmd.stop || behavior.isComplete()) {
        motor.stop();
        vacuum.off();
    } else {
        motor.set(cmd.leftPWM, cmd.rightPWM);
    }

    // 出場時關閉吸塵器
    if (behavior.isExiting()) {
        vacuum.off();
    }
}
