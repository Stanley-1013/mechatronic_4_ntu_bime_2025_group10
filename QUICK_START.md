# 快速開始指南

## 🎯 最新更新：改用 USB 2.4G 遙控器

我們已將遙控方案簡化為 **USB 2.4G 遙控器**，無需焊接 NRF24L01 模組！

### 為什麼選擇 USB 遙控器？

- ✅ **即插即用**：USB 接收器插入 Pi，自動識別
- ✅ **節省時間**：不需要焊接、接線、除錯
- ✅ **降低成本**：商用遙控器約 $10-30，比自製便宜
- ✅ **更可靠**：商用產品品質測試充分
- ✅ **快速開發**：5 分鐘寫好接收程式
- ✅ **易於除錯**：用 `jstest` 即時查看輸入

## 📋 準備清單

### 硬體

**必備：**
- [ ] Raspberry Pi 4
- [ ] Arduino Uno
- [ ] L298N 馬達驅動
- [ ] TT 馬達 × 2
- [ ] HC-SR04 超聲波 × 2
- [ ] Pi Camera V2/V3
- [ ] 18650 電池 (3S = 11.1V)
- [ ] DC-DC 降壓模組 (5V/3A)
- [ ] **2.4G USB 遙控器組**（接收器 + 遙控器）
- [ ] 杜邦線、麵包板

**可選：**
- [ ] 車體底盤
- [ ] 電源開關
- [ ] LED 指示燈

### 軟體（Raspberry Pi）

```bash
# 更新系統
sudo apt update && sudo apt upgrade

# 啟用介面（Serial, SPI, Camera）
sudo raspi-config
# Interface Options → Serial, SPI, Camera (全部啟用)

# 安裝 Python 套件
sudo apt install python3-pygame python3-serial joystick

# 測試工具
sudo apt install htop

# 建立專案目錄
mkdir -p ~/robot_project
cd ~/robot_project
```

## 🚀 Step-by-Step 開發流程

### Step 1：測試 L298N 馬達（Arduino）

**目標：** 驗證馬達驅動接線與控制

**接線：**
```
L298N IN1 → Arduino D5
L298N IN2 → Arduino D6
L298N ENA → Arduino D3 (PWM)
L298N IN3 → Arduino D9
L298N IN4 → Arduino D10
L298N ENB → Arduino D11 (PWM)
L298N +12V → 電池 11.1V
L298N GND → Arduino GND
```

**移除 L298N 的 ENA/ENB 跳線！**

**Arduino 測試程式：**

```cpp
// test_motor.ino
const int IN1 = 5, IN2 = 6, ENA = 3;
const int IN3 = 9, IN4 = 10, ENB = 11;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(57600);
  Serial.println("Motor Test Ready");
}

void loop() {
  // 前進
  Serial.println("Forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 150);
  delay(2000);

  // 停止
  Serial.println("Stop");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);

  // 後退
  Serial.println("Backward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 150);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 150);
  delay(2000);

  // 停止
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);
}
```

**驗收：** 馬達應該前進 2 秒 → 停止 1 秒 → 後退 2 秒

---

### Step 2：測試超聲波感測器（Arduino）

**目標：** 讀取左右兩側距離

**接線：**
```
左側 HC-SR04:
  Trig → Arduino D7
  Echo → Arduino D8
  VCC → 5V, GND → GND

右側 HC-SR04:
  Trig → Arduino A1
  Echo → Arduino A2
  VCC → 5V, GND → GND
```

**Arduino 測試程式：**

```cpp
// test_ultrasonic.ino
const int TRIG_L = 7, ECHO_L = 8;
const int TRIG_R = A1, ECHO_R = A2;

void setup() {
  Serial.begin(57600);
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);
  Serial.println("Ultrasonic Test Ready");
}

float measure(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) return 999.0;  // 超時

  return duration * 0.034 / 2;
}

void loop() {
  float left = measure(TRIG_L, ECHO_L);
  delay(50);
  float right = measure(TRIG_R, ECHO_R);

  Serial.print("Left: ");
  Serial.print(left);
  Serial.print(" cm  Right: ");
  Serial.print(right);
  Serial.println(" cm");

  delay(200);
}
```

**驗收：** Serial Monitor 應顯示左右距離（用手靠近測試）

---

### Step 3：測試 2.4G USB 遙控器（Raspberry Pi）

**目標：** 確認遙控器可被 Pi 識別

**步驟：**

1. 將 USB 接收器插入 Raspberry Pi
2. 開啟遙控器電源

```bash
# 查看設備
ls /dev/input/js*
# 應該看到 /dev/input/js0

# 測試搖桿輸入
jstest /dev/input/js0
# 移動搖桿，應該看到軸值變化
```

**Python 測試程式：**

```bash
cd ~/robot_project
nano test_joystick.py
```

```python
#!/usr/bin/env python3
import pygame
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ 找不到遙控器！")
    exit(1)

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"✅ 遙控器: {joystick.get_name()}")
print(f"   軸數: {joystick.get_numaxes()}")
print(f"   按鈕數: {joystick.get_numbuttons()}")
print("\n移動搖桿測試...\n")

try:
    while True:
        pygame.event.pump()

        for i in range(joystick.get_numaxes()):
            value = joystick.get_axis(i)
            if abs(value) > 0.1:
                print(f"Axis {i}: {value:+.2f}  ", end='')

        print("\r", end='')
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n✅ 測試完成")
    pygame.quit()
```

```bash
chmod +x test_joystick.py
python3 test_joystick.py
```

**記錄你的遙控器軸編號**（稍後會用到）

---

### Step 4：建立 Arduino-Pi Serial 通訊

**目標：** Pi 發送指令，Arduino 控制馬達

**Arduino 接線（與 Pi）：**
```
Arduino D2 (TX) → Pi GPIO15 (RXD, Pin 10)
Arduino D4 (RX) → Pi GPIO14 (TXD, Pin 8)
Arduino GND → Pi GND
```

**Arduino 完整程式（motor_controller.ino）：**

```cpp
#include <SoftwareSerial.h>

SoftwareSerial piSerial(4, 2); // RX, TX

// L298N 接腳
const int IN1 = 5, IN2 = 6, ENA = 3;
const int IN3 = 9, IN4 = 10, ENB = 11;

// 超聲波接腳
const int TRIG_L = 7, ECHO_L = 8;
const int TRIG_R = A1, ECHO_R = A2;

// 封包緩衝區
uint8_t rxBuffer[8];
int rxIndex = 0;

void setup() {
  Serial.begin(57600);       // Debug
  piSerial.begin(57600);     // Pi 通訊

  // 馬達腳位
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // 超聲波腳位
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  Serial.println("Arduino Motor Controller Ready");
}

void loop() {
  // 接收 Pi 的馬達指令
  if (piSerial.available()) {
    uint8_t inByte = piSerial.read();

    if (rxIndex == 0 && inByte != 0xAA) return;

    rxBuffer[rxIndex++] = inByte;

    if (rxIndex == 8) {
      rxIndex = 0;

      // 驗證封包
      if (rxBuffer[7] != 0x55) return;

      // 解析馬達指令
      int16_t leftPWM = (int16_t)(rxBuffer[1] | (rxBuffer[2] << 8));
      int16_t rightPWM = (int16_t)(rxBuffer[3] | (rxBuffer[4] << 8));

      // 控制馬達
      setMotor(leftPWM, IN1, IN2, ENA);
      setMotor(rightPWM, IN3, IN4, ENB);

      Serial.print("L:"); Serial.print(leftPWM);
      Serial.print(" R:"); Serial.println(rightPWM);
    }
  }

  // 定期發送感測器資料給 Pi（每 100ms）
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 100) {
    lastSend = millis();
    sendSensorData();
  }
}

void setMotor(int16_t speed, int in1, int in2, int enable) {
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enable, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enable, -speed);
  }
}

float measure(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) return 999.0;

  return duration * 0.034 / 2;
}

void sendSensorData() {
  float left = measure(TRIG_L, ECHO_L);
  delay(10);
  float right = measure(TRIG_R, ECHO_R);

  // 封包格式：[0xBB] [L_cm] [R_cm] [0x66]
  uint16_t l_cm = (uint16_t)left;
  uint16_t r_cm = (uint16_t)right;

  uint8_t packet[8] = {
    0xBB,
    l_cm & 0xFF,
    (l_cm >> 8) & 0xFF,
    r_cm & 0xFF,
    (r_cm >> 8) & 0xFF,
    0,
    0,
    0x66
  };

  // 計算校驗和
  uint8_t checksum = 0;
  for (int i = 1; i < 6; i++) {
    checksum ^= packet[i];
  }
  packet[6] = checksum;

  piSerial.write(packet, 8);
}
```

上傳到 Arduino！

---

### Step 5：整合所有功能（Raspberry Pi）

**創建完整控制程式：**

```bash
cd ~/robot_project
nano robot_controller.py
```

```python
#!/usr/bin/env python3
"""機器人遙控主程式"""

import serial
import pygame
import time
import struct

class ArduinoController:
    def __init__(self, port='/dev/ttyAMA0', baudrate=57600):
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)
        print(f"✅ Arduino 已連接: {port}")

    def send_motor_command(self, left_speed, right_speed):
        """
        發送馬達指令
        left_speed, right_speed: -1.0 ~ 1.0
        """
        # 轉換為 PWM 值
        left_pwm = int(left_speed * 255)
        right_pwm = int(right_speed * 255)

        # 組裝封包
        packet = bytearray([
            0xAA,
            left_pwm & 0xFF,
            (left_pwm >> 8) & 0xFF,
            right_pwm & 0xFF,
            (right_pwm >> 8) & 0xFF,
            0,
            0,
            0x55
        ])

        # 校驗和
        checksum = 0
        for i in range(1, 6):
            checksum ^= packet[i]
        packet[6] = checksum

        self.serial.write(packet)

class USB24GReceiver:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        while pygame.joystick.get_count() == 0:
            print("等待遙控器...")
            time.sleep(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        print(f"✅ 遙控器已連接: {self.joystick.get_name()}")

    def receive(self):
        pygame.event.pump()

        # ⚠️ 根據你的遙控器調整軸編號
        left_y = -self.joystick.get_axis(1)   # 前進/後退
        left_x = self.joystick.get_axis(0)    # 左轉/右轉

        # 死區
        if abs(left_y) < 0.1:
            left_y = 0.0
        if abs(left_x) < 0.1:
            left_x = 0.0

        return left_y, left_x

class RobotController:
    def __init__(self):
        self.arduino = ArduinoController()
        self.receiver = USB24GReceiver()

    def run(self):
        print("🤖 機器人控制啟動！")
        print("   按 Ctrl+C 停止")

        try:
            while True:
                # 接收遙控指令
                linear, angular = self.receiver.receive()

                # 差動驅動轉換
                left_speed = linear - angular
                right_speed = linear + angular

                # 限制範圍
                left_speed = max(-1.0, min(1.0, left_speed))
                right_speed = max(-1.0, min(1.0, right_speed))

                # 發送到 Arduino
                self.arduino.send_motor_command(left_speed, right_speed)

                # 顯示狀態
                print(f"L: {left_speed:+.2f}  R: {right_speed:+.2f}", end='\r')

                time.sleep(0.02)  # 50Hz

        except KeyboardInterrupt:
            print("\n⏹️  停止")
            self.arduino.send_motor_command(0, 0)

if __name__ == "__main__":
    controller = RobotController()
    controller.run()
```

```bash
chmod +x robot_controller.py
python3 robot_controller.py
```

**測試遙控！** 🎮

---

## ✅ 驗收檢查

- [ ] 馬達可以前進/後退
- [ ] 馬達可以左轉/右轉
- [ ] 超聲波可以讀取距離
- [ ] 遙控器搖桿可以控制車輛
- [ ] Serial Monitor 顯示馬達 PWM 值
- [ ] 延遲 < 100ms（手感順暢）

## 🐛 常見問題

**Q: 馬達不轉**
- 檢查電源（11.1V 到 L298N）
- 檢查 ENA/ENB 跳線是否移除
- 檢查 GND 是否共地
- 用 Serial Monitor 查看是否收到指令

**Q: 遙控器沒反應**
- 執行 `jstest /dev/input/js0` 確認設備存在
- 檢查軸編號是否正確（用 test_joystick.py）
- 確認 pygame 已安裝

**Q: Serial 通訊失敗**
- 檢查 TX/RX 接線（注意交叉連接）
- 確認鮑率一致（57600）
- 檢查 GND 共地
- 用 `sudo chmod 666 /dev/ttyAMA0` 修改權限

**Q: 超聲波讀值不穩**
- 左右兩側不要同時觸發（程式中已加 delay）
- 避免對著光滑表面（會反射不回來）
- 調整安裝角度（45° 朝外）

## 📚 下一步

完成基礎遙控後：

1. **優化控制**：
   - 調整死區值
   - 加入速度限制（避免過快）
   - 實作緊急停止按鈕

2. **加入安全功能**：
   - 超聲波自動停止（距離 < 20cm）
   - 逾時自動停止（1 秒沒指令）
   - 低電壓警告

3. **準備視覺導航**：
   - 安裝 Pi Camera
   - 測試 Picamera2
   - 研究路徑偵測演算法

---

**文檔版本：** 1.0
**最後更新：** 2025-10-31
**適用平台：** Raspberry Pi 4 + Arduino Uno
