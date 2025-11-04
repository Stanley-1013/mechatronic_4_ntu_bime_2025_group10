# 程式碼改善建議清單

> 專注於：穩定性、易除錯、易修改
> 日期：2025-10-31

---

## 🔧 Priority 1: 提升穩定性

### 1.1 Serial 通訊重連機制

**檔案**: `raspberry_pi/arduino_controller.py`

**問題**: 若 Serial 連線斷開（如拔除 USB、Arduino 重置），程式會持續報錯但無法自動恢復。

**改善方案**:
```python
# 在 ArduinoController 類別中添加
def _reconnect(self):
    """嘗試重新連接 Serial"""
    try:
        if self.serial and self.serial.is_open:
            self.serial.close()

        print("[INFO] 等待 2 秒後重新連接...")
        time.sleep(2)

        self._connect()
        self.reset_stats()  # 重置統計
        return True
    except Exception as e:
        print(f"❌ 重新連接失敗: {e}")
        return False

# 修改 send_command()
def send_command(self, motor_cmd: MotorCommand):
    try:
        packet = self._build_motor_packet(motor_cmd)
        self.serial.write(packet)
        self.stats['tx_packets'] += 1

    except serial.SerialException as e:
        self.stats['tx_errors'] += 1
        print(f"❌ Serial 發送錯誤 ({self.stats['tx_errors']}/{self.stats['tx_packets']}): {e}")

        # 錯誤率超過 50% 時嘗試重連
        if self.stats['tx_errors'] > 5:
            error_rate = self.stats['tx_errors'] / max(1, self.stats['tx_packets'])
            if error_rate > 0.5:
                print("⚠️  錯誤率過高，嘗試重新連接...")
                self._reconnect()
```

**優點**:
- ✅ 自動恢復連線，不需重啟程式
- ✅ 避免因暫時性問題導致整個系統停擺
- ✅ 適合長時間運行

---

### 1.2 看門狗 (Watchdog) 強化

**檔案**: `raspberry_pi/robot_controller.py`

**問題**: 現有看門狗只檢查逾時，但若 Serial 假死（連接存在但無回應），無法偵測。

**改善方案**:
```python
# 在 RobotController 中添加
def _check_watchdog(self):
    """增強版看門狗：檢查逾時 + Serial 健康度"""
    current_time = time.time()

    # 檢查 1: 指令逾時
    elapsed = current_time - self.last_command_time
    if elapsed > config.EMERGENCY_STOP_TIMEOUT:
        stop_cmd = MotorCommand(0, 0, False)
        self.arduino.send_command(stop_cmd)
        if config.DEBUG_MODE:
            print(f"\n⚠️  看門狗: {elapsed*1000:.0f}ms 未收到指令，已停止")

    # 檢查 2: Serial 健康度（每 5 秒檢查一次）
    if not hasattr(self, '_last_health_check'):
        self._last_health_check = current_time

    if current_time - self._last_health_check > 5.0:
        stats = self.arduino.get_stats()
        if stats['tx_packets'] > 50:  # 至少有一定數據量
            error_rate = stats['tx_errors'] / stats['tx_packets']
            if error_rate > 0.1:  # 錯誤率 > 10%
                print(f"\n⚠️  Serial 健康度異常: 錯誤率 {error_rate*100:.1f}%")

        self._last_health_check = current_time
```

---

### 1.3 Arduino 逾時保護完善

**檔案**: `arduino/main/main.ino`

**問題**: Arduino 逾時保護只停止馬達，但若 Pi 當機，Arduino 會持續嘗試讀取 Serial，浪費資源。

**改善方案**:
```cpp
// 在 loop() 中修改
void loop() {
    unsigned long currentTime = millis();

    // 逾時保護
    if (currentTime - lastCommandTime > COMMAND_TIMEOUT) {
        motor.stop();  // 停止馬達

        // 進入低功耗模式（降低 Serial 讀取頻率）
        static unsigned long lastLowPowerCheck = 0;
        if (currentTime - lastLowPowerCheck > 500) {  // 每 500ms 檢查一次
            // 顯示狀態
            if (Serial && (currentTime - lastLowPowerCheck > 5000)) {
                Serial.println(F("[WARN] Timeout - waiting for command..."));
                lastLowPowerCheck = currentTime;
            }
        }

        // 超過 10 秒無指令，重置 Arduino（可選）
        if (currentTime - lastCommandTime > 10000) {
            Serial.println(F("[ERROR] 10s timeout - system reset!"));
            asm volatile ("  jmp 0");  // 軟體重置
        }
    }

    // ... 其他邏輯
}
```

---

## 🐛 Priority 2: 增強除錯功能

### 2.1 即時狀態監控面板

**檔案**: `raspberry_pi/robot_controller.py`

**問題**: 當前只有一行輸出，難以快速診斷問題。

**改善方案**:
```python
def _display_status(self, vehicle_cmd, motor_cmd, sensor_data):
    """改良版狀態顯示 - 更清晰的格式"""
    elapsed = time.time() - self.start_time if self.start_time else 0

    # 清除螢幕並移動游標到頂部（可選）
    if config.DEBUG_MODE:
        print("\033[2J\033[H", end='')  # ANSI 清屏

    # 狀態面板
    print("=" * 80)
    print(f" 機電小車控制系統 | 運行時間: {elapsed:.1f}s | 迴圈: {self.loop_count}")
    print("=" * 80)

    # 遙控器輸入
    print(f"[遙控器] 線性: {vehicle_cmd.linear_velocity:+.2f} | "
          f"角速度: {vehicle_cmd.angular_velocity:+.2f} | "
          f"吸塵器: {'🟢 ON' if vehicle_cmd.vacuum_motor else '⚪ OFF'}")

    # 馬達輸出
    print(f"[馬達]   左輪: {motor_cmd.left_pwm:+4d} | "
          f"右輪: {motor_cmd.right_pwm:+4d}")

    # 感測器
    left_icon = "🟢" if sensor_data.left_valid else "🔴"
    right_icon = "🟢" if sensor_data.right_valid else "🔴"
    print(f"[感測器] 左側: {left_icon} {sensor_data.left_distance:3d}cm | "
          f"右側: {right_icon} {sensor_data.right_distance:3d}cm")

    # 通訊統計
    stats = self.arduino.get_stats()
    success_rate = 0
    if stats['tx_packets'] > 0:
        success_rate = (1 - stats['checksum_errors'] / max(1, stats['tx_packets'])) * 100

    print(f"[Serial] TX: {stats['tx_packets']} | RX: {stats['rx_packets']} | "
          f"錯誤: {stats['tx_errors']}+{stats['rx_errors']} | "
          f"成功率: {success_rate:.1f}%")
    print("=" * 80)

    # 保留原有的單行模式（非除錯模式）
    if not config.DEBUG_MODE:
        status = (
            f"[{elapsed:6.1f}s] "
            f"Joy(L:{vehicle_cmd.linear_velocity:+.2f} A:{vehicle_cmd.angular_velocity:+.2f}) "
            f"PWM(L:{motor_cmd.left_pwm:+4d} R:{motor_cmd.right_pwm:+4d}) "
            f"Dist(L:{sensor_data.left_distance:3d} R:{sensor_data.right_distance:3d})"
        )
        print(status, end='\r')
```

---

### 2.2 錯誤記錄檔案

**檔案**: `raspberry_pi/robot_controller.py`

**問題**: 錯誤訊息只顯示在終端機，關閉後無法追溯。

**改善方案**:
```python
import logging
from datetime import datetime

# 在 RobotController.__init__() 中添加
def __init__(self):
    # ... 原有初始化

    # 設定日誌
    self._setup_logging()

def _setup_logging(self):
    """設定日誌系統"""
    log_format = '%(asctime)s [%(levelname)s] %(message)s'
    handlers = [
        logging.StreamHandler(),  # 終端輸出
    ]

    if config.LOG_TO_FILE:
        # 添加檔案處理器
        log_filename = f"/tmp/robot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        handlers.append(logging.FileHandler(log_filename))
        print(f"[INFO] 日誌檔案: {log_filename}")

    logging.basicConfig(
        level=getattr(logging, config.LOG_LEVEL),
        format=log_format,
        handlers=handlers
    )

    self.logger = logging.getLogger(__name__)

# 使用範例
def _control_loop(self):
    try:
        vehicle_cmd = self.receiver.receive()
        # ...
    except Exception as e:
        self.logger.error(f"控制迴圈錯誤: {e}", exc_info=True)
```

---

### 2.3 Arduino Serial Monitor 輸出改善

**檔案**: `arduino/main/main.ino`

**問題**: Arduino 除錯訊息太簡單，難以診斷問題。

**改善方案**:
```cpp
// 添加除錯旗標
#define DEBUG_VERBOSE  // 註解此行可關閉詳細輸出

// 在 processMotorCommand() 中
void processMotorCommand() {
    int16_t leftPwm, rightPwm;
    bool vacuumState;

    if (parseMotorPacket(rxBuffer, leftPwm, rightPwm, vacuumState)) {
        motor.setLeftMotor(leftPwm);
        motor.setRightMotor(rightPwm);
        vacuum.setState(vacuumState);
        lastCommandTime = millis();

        #ifdef DEBUG_VERBOSE
        // 詳細輸出（包含封包原始資料）
        Serial.print(F("[CMD] Packet: "));
        for (int i = 0; i < 8; i++) {
            Serial.print(rxBuffer[i], HEX);
            Serial.print(F(" "));
        }
        Serial.print(F("| L:"));
        Serial.print(leftPwm);
        Serial.print(F(" R:"));
        Serial.print(rightPwm);
        Serial.print(F(" V:"));
        Serial.println(vacuumState ? F("ON") : F("OFF"));
        #else
        // 簡潔輸出
        Serial.print(F("[OK] L:"));
        Serial.print(leftPwm);
        Serial.print(F(" R:"));
        Serial.println(rightPwm);
        #endif
    }
    else {
        // 顯示錯誤封包內容（幫助診斷）
        Serial.print(F("[ERR] Invalid: "));
        for (int i = 0; i < 8; i++) {
            Serial.print(rxBuffer[i], HEX);
            Serial.print(F(" "));
        }
        Serial.println();
    }
}
```

---

## 🔄 Priority 3: 改善可修改性

### 3.1 參數調校介面

**檔案**: `raspberry_pi/config.py`

**問題**: 修改參數需要編輯 Python 檔案並重新啟動。

**改善方案**: 添加命令列參數覆寫

```python
# 在 main.py 中
def parse_args():
    parser = argparse.ArgumentParser(...)

    # 添加參數覆寫選項
    parser.add_argument('--deadzone', type=float, help='搖桿死區 (0.0-1.0)')
    parser.add_argument('--frequency', type=int, help='控制頻率 (Hz)')
    parser.add_argument('--left-scale', type=float, help='左輪速度倍率')
    parser.add_argument('--right-scale', type=float, help='右輪速度倍率')

    return parser.parse_args()

def main():
    args = parse_args()

    # 覆寫設定
    if args.deadzone:
        config.JOYSTICK_DEADZONE = args.deadzone
    if args.frequency:
        config.CONTROL_LOOP_FREQUENCY = args.frequency
        config.CONTROL_LOOP_PERIOD = 1.0 / args.frequency
    if args.left_scale:
        config.MOTOR_LEFT_SCALE = args.left_scale
    if args.right_scale:
        config.MOTOR_RIGHT_SCALE = args.right_scale

    # 顯示當前設定
    print(f"[設定] 死區: {config.JOYSTICK_DEADZONE}")
    print(f"[設定] 頻率: {config.CONTROL_LOOP_FREQUENCY} Hz")
    print(f"[設定] 馬達校準: L={config.MOTOR_LEFT_SCALE}, R={config.MOTOR_RIGHT_SCALE}\n")

    controller = RobotController()
    controller.start()
```

**使用範例**:
```bash
# 調整搖桿死區
python3 main.py --deadzone 0.15

# 調整馬達校準（左輪較慢）
python3 main.py --left-scale 1.1 --right-scale 1.0

# 降低頻率減輕 CPU 負擔
python3 main.py --frequency 30
```

---

### 3.2 Arduino 腳位設定集中管理

**檔案**: `arduino/main/config.h` (新建)

**問題**: 腳位定義散落在 main.ino，修改時容易遺漏。

**改善方案**: 建立統一設定檔

```cpp
/*
 * config.h - Arduino 設定檔（集中管理所有參數）
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==================== 腳位定義 ====================
// L298N 馬達驅動
#define PIN_ENA 3
#define PIN_IN1 5
#define PIN_IN2 6
#define PIN_ENB 11
#define PIN_IN3 9
#define PIN_IN4 10

// 超聲波感測器
#define PIN_US_LEFT_TRIG  7
#define PIN_US_LEFT_ECHO  8
#define PIN_US_RIGHT_TRIG A1
#define PIN_US_RIGHT_ECHO A2

// 吸塵器馬達
#define PIN_VACUUM 12

// SoftwareSerial
#define PIN_SERIAL_RX 4
#define PIN_SERIAL_TX 2

// ==================== 通訊參數 ====================
#define SERIAL_BAUDRATE 57600
#define SENSOR_UPDATE_INTERVAL 100  // ms
#define COMMAND_TIMEOUT 200         // ms

// ==================== 除錯設定 ====================
#define DEBUG_SERIAL_ENABLED   // 啟用 USB Serial 除錯
// #define DEBUG_VERBOSE       // 詳細輸出（註解以關閉）

#endif // CONFIG_H
```

然後在 `main.ino` 中：
```cpp
#include "config.h"  // 第一行引入設定

// 使用設定中的腳位
MotorDriver motor(PIN_ENA, PIN_IN1, PIN_IN2, PIN_ENB, PIN_IN3, PIN_IN4);
```

---

### 3.3 模組獨立測試腳本

**問題**: 難以單獨測試某個模組（如只測試馬達，不啟動整個系統）。

**改善方案**: 建立獨立測試腳本

**檔案**: `raspberry_pi/test_motor_only.py`
```python
#!/usr/bin/env python3
"""獨立測試馬達控制（不需遙控器）"""

from arduino_controller import ArduinoController, MotorCommand
import time

controller = ArduinoController()

test_sequence = [
    ("前進", MotorCommand(255, 255, False), 2),
    ("後退", MotorCommand(-255, -255, False), 2),
    ("左轉", MotorCommand(-200, 200, False), 1),
    ("右轉", MotorCommand(200, -200, False), 1),
    ("停止", MotorCommand(0, 0, False), 1),
]

for name, cmd, duration in test_sequence:
    print(f"[測試] {name}: {cmd}")
    controller.send_command(cmd)
    time.sleep(duration)

print("測試完成")
controller.close()
```

---

## 📊 總結

### 實作優先順序

| 優先級 | 項目 | 預估時間 | 影響 |
|-------|------|---------|------|
| 🔴 高 | 1.1 Serial 重連機制 | 30分鐘 | 大幅提升穩定性 |
| 🔴 高 | 2.1 狀態監控面板 | 20分鐘 | 顯著改善除錯效率 |
| 🟡 中 | 1.2 看門狗強化 | 15分鐘 | 提升可靠性 |
| 🟡 中 | 2.3 Arduino 除錯輸出 | 15分鐘 | 改善 Arduino 端除錯 |
| 🟡 中 | 3.1 參數調校介面 | 20分鐘 | 方便現場調整 |
| 🟢 低 | 2.2 錯誤記錄檔案 | 15分鐘 | 長期運行追溯 |
| 🟢 低 | 3.2 Arduino 設定檔 | 10分鐘 | 改善可維護性 |
| 🟢 低 | 3.3 獨立測試腳本 | 10分鐘 | 加速開發測試 |

### 建議實作順序
1. **先做 1.1 + 2.1**：立即提升穩定性與除錯能力
2. **再做 3.1**：方便後續測試時調整參數
3. **最後做其他項目**：視需求選擇性實作

---

**文件版本**: 1.0
**最後更新**: 2025-10-31
