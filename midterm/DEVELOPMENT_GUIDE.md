# 開發指南

**專案名稱：** 機電小車自走清掃系統
**團隊：** NTU BIME 2025 機電整合四 Group 10
**文檔版本：** 2.0
**最後更新：** 2025-11-29

---

## 目錄

1. [環境建置](#1-環境建置)
2. [快速開始](#2-快速開始)
3. [測試流程](#3-測試流程)
4. [除錯指南](#4-除錯指南)
5. [維護與更新](#5-維護與更新)
6. [程式碼改進建議](#6-程式碼改進建議)

---

## 1. 環境建置

### 1.1 Raspberry Pi 環境

#### 系統需求
- Raspberry Pi 4 (推薦 4GB+ RAM)
- Raspberry Pi OS (Bookworm)
- Python 3.11+

#### 安裝步驟

```bash
# 1. 更新系統
sudo apt update && sudo apt upgrade -y

# 2. 啟用介面
sudo raspi-config
# → Interface Options → Serial Port
#   - Login shell over serial: No
#   - Serial port hardware: Yes
# → Interface Options → I2C → Yes
# → Interface Options → Camera → Yes (如需相機)
sudo reboot

# 3. 安裝系統套件
sudo apt install -y python3-pygame python3-serial joystick git

# 4. 複製專案
cd ~
git clone <your-repo-url> final
cd final/raspberry_pi

# 5. 安裝 Python 依賴
pip3 install -r requirements.txt

# 6. 驗證安裝
python3 -c "import pygame, serial, cv2; print('✅ 安裝成功')"
```

### 1.2 Arduino 環境

#### 使用 Arduino IDE

1. 下載安裝 [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. 開啟 `arduino/main/main.ino`
3. 選擇板子：**工具 → 板子 → Arduino Uno**
4. 選擇序列埠：**工具 → 序列埠 → /dev/ttyACM0**
5. 上傳程式

#### 使用 arduino-cli (命令列)

```bash
# 安裝
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# 初始化
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr

# 編譯
cd ~/final
arduino-cli compile --fqbn arduino:avr:uno arduino/main/

# 上傳
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/main/
```

### 1.3 遠端同步 (SSH)

```bash
# 從開發電腦同步到 Pi
scp -r raspberry_pi/ pi@<PI_IP>:~/final/

# 或使用 git
ssh pi@<PI_IP>
cd ~/final && git pull
```

---

## 2. 快速開始

### 2.1 硬體接線快速參考

```
L298N 馬達驅動:
  ENA → D3 (PWM)     左輪速度
  IN1 → D6           左輪方向
  IN2 → D5           左輪方向
  ENB → D11 (PWM)    右輪速度
  IN3 → D9           右輪方向
  IN4 → D10          右輪方向

超聲波:
  前方 Trig → D7     前方 Echo → D8
  右側 Trig → A1     右側 Echo → A2

MPU6050 IMU:
  SDA → A4           SCL → A5

吸塵器:
  Relay → A3

⚠️ 所有 GND 必須共地
```

### 2.2 啟動系統

```bash
cd ~/final/raspberry_pi

# 遙控模式
python3 main.py

# 自走模式（完整功能）
python3 autonomous_main.py

# 自走模式（不用相機）
python3 autonomous_main.py --no-camera

# 除錯模式
python3 autonomous_main.py --debug
```

### 2.3 操作說明

**遙控模式：**
| 操作 | 控制 |
|-----|------|
| 左搖桿 Y 軸 | 前進/後退 |
| 左搖桿 X 軸 | 左轉/右轉 |
| A 按鈕 | 吸塵器開關 (toggle) |
| Ctrl+C | 停止程式 |

**自走模式：**
| 按鍵 | 功能 |
|-----|------|
| Ctrl+C | 正常停止 |
| 緊急 | 超聲波偵測前方 <20cm 自動停止 |

---

## 3. 測試流程

### 3.1 測試順序

```
Step 1: 遙控器測試 ──→ 確認設備識別
    │
Step 2: Arduino 上傳 ──→ 確認韌體正常
    │
Step 3: Serial 通訊 ──→ 確認 Pi-Arduino 連接
    │
Step 4: 馬達測試 ──→ 確認方向與速度
    │
Step 5: 感測器測試 ──→ 確認超聲波/IMU
    │
Step 6: 完整系統測試 ──→ 整合驗證
```

### 3.2 Step 1: 遙控器測試

```bash
# 檢查設備
ls /dev/input/js*
# 應顯示: /dev/input/js0

# 系統工具測試
jstest /dev/input/js0

# Python 測試腳本
cd ~/final/raspberry_pi
python3 test_joystick.py
```

**記錄軸編號：**
- 左搖桿 Y (前後): Axis ___
- 左搖桿 X (左右): Axis ___
- 吸塵器按鈕: Button ___

### 3.3 Step 2: Arduino 上傳驗證

上傳後開啟 Serial Monitor (115200 baud)，應看到：

```
========================================
Arduino 機器人控制系統 v2.0
========================================
[初始化] 馬達驅動...完成
[初始化] 超聲波感測器...完成
[初始化] MPU6050 IMU...完成
[初始化] 吸塵器控制...完成
[系統] 準備就緒，等待指令...
========================================
```

### 3.4 Step 3: 超聲波測試

```bash
python3 test_ultrasonic.py
```

**預期輸出：**
```
[超聲波測試]
前方: 045cm ✓  右側: 120cm ✓
前方: 044cm ✓  右側: 119cm ✓
...
```

用手靠近感測器，數值應隨距離變化。

### 3.5 Step 4: IMU 測試

```bash
python3 test_mpu6050.py
```

**預期輸出：**
```
[MPU6050 測試]
Yaw: +0.0°  GyroZ: 0°/s  狀態: IMU OK
Yaw: +0.3°  GyroZ: 2°/s  狀態: IMU OK
...
```

旋轉車體，Yaw 角度應跟隨變化。

### 3.6 Step 5: 馬達測試

**⚠️ 安全提醒：將車輛架高，輪子離地！**

```bash
python3 test_motor_only.py
```

**測試項目：**
- ✅ 前進：兩輪同向正轉
- ✅ 後退：兩輪同向反轉
- ✅ 左轉：左輪反轉、右輪前進
- ✅ 右轉：左輪前進、右輪反轉

**手動測試指令（Serial Monitor）：**
```
M150,150    # 前進
M-150,-150  # 後退
M-100,100   # 左轉
M100,-100   # 右轉
M0,0        # 停止
V1          # 吸塵器開
V0          # 吸塵器關
```

### 3.7 測試記錄表

```
測試日期: __________
測試人員: __________

[ ] Step 1: 遙控器
    型號: __________
    軸: Linear=__, Angular=__

[ ] Step 2: Arduino 上傳成功

[ ] Step 3: 超聲波
    [ ] 前方讀值正常
    [ ] 右側讀值正常

[ ] Step 4: IMU
    [ ] Yaw 角度正常
    [ ] 漂移率可接受

[ ] Step 5: 馬達
    [ ] 前進 [ ] 後退
    [ ] 左轉 [ ] 右轉
    校準: L=____, R=____

[ ] Step 6: 完整系統正常

備註:
_________________________________
```

---

## 4. 除錯指南

### 4.1 分層除錯策略

```
第 1 層：硬體檢查（不需程式）
    ↓
第 2 層：單一模組測試
    ↓
第 3 層：模組整合測試
    ↓
第 4 層：完整系統測試
```

**原則：從下往上，哪層有問題就在哪層解決。**

### 4.2 常見問題診斷

#### 問題 1：Arduino 無法上傳

```bash
# 檢查連接
ls /dev/ttyACM* /dev/ttyUSB*

# 權限問題
sudo usermod -a -G dialout $USER
# 登出重新登入

# 重試上傳
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/main/
```

#### 問題 2：遙控器找不到

```bash
# 載入模組
sudo modprobe joydev

# 檢查 USB
lsusb

# 查看 kernel 訊息
dmesg | tail -20

# 權限問題
sudo usermod -a -G input $USER
```

#### 問題 3：Serial 通訊失敗

```bash
# 檢查裝置
ls -l /dev/ttyACM*

# 檢查佔用
sudo lsof /dev/ttyACM0

# 修改權限
sudo chmod 666 /dev/ttyACM0
```

#### 問題 4：馬達不轉

| 可能原因 | 解決方法 |
|---------|---------|
| 電源未接 | 檢查 L298N 12V 電源 |
| 跳線未拔 | 移除 ENA/ENB 跳線帽 |
| PWM 腳位錯 | 確認 ENA→D3, ENB→D11 |
| 接線鬆脫 | 重新檢查所有接線 |
| GND 未共地 | 連接所有 GND |

#### 問題 5：超聲波讀值異常

| 現象 | 可能原因 | 解決方法 |
|-----|---------|---------|
| 一直顯示 0 | Echo 未接 | 檢查 Echo 接線 |
| 一直顯示 999 | Trig 未接或電源不足 | 檢查 Trig 和 VCC |
| 數值跳動 | 干擾或角度問題 | 調整安裝角度 |

#### 問題 6：IMU 資料無效

```bash
# 檢查 I2C 設備
i2cdetect -y 1
# 應看到 0x68 位址

# 確認接線
# SDA → A4, SCL → A5
```

#### 問題 7：馬達轉向相反

```python
# 修改 config.py
MOTOR_LEFT_SCALE = -1.0   # 反轉左輪
# 或
MOTOR_RIGHT_SCALE = -1.0  # 反轉右輪
```

或在硬體端交換馬達線（OUT1 ↔ OUT2）。

### 4.3 Arduino 除錯輸出

編輯 `arduino/main/config.h`：

```cpp
// 基本除錯（預設）
#define DEBUG_SERIAL_ENABLED
#define DEBUG_SHOW_COMMANDS
#define DEBUG_SHOW_SENSORS

// 詳細除錯（問題排查時開啟）
#define DEBUG_VERBOSE  // 顯示封包 HEX
```

修改後重新上傳！

### 4.4 除錯流程圖

```
發現問題
    ↓
車子能動嗎？
├─ 能 → 方向正確嗎？
│        ├─ 正確 → 感測器正常嗎？ → 完成
│        └─ 錯誤 → 修改 MOTOR_*_SCALE
└─ 不能 → 從硬體層開始檢查
            ├─ 電源正常？
            ├─ Arduino 連接正常？
            └─ Serial 通訊正常？
```

---

## 5. 維護與更新

### 5.1 日常維護

```bash
# 同步最新程式
cd ~/final
git pull

# 重新安裝依賴（如有更新）
cd raspberry_pi
pip3 install -r requirements.txt --upgrade
```

### 5.2 Arduino 韌體更新

```bash
# 編譯
arduino-cli compile --fqbn arduino:avr:uno arduino/main/

# 上傳
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/main/
```

### 5.3 參數調校

**運行時調整（命令列參數）：**

```bash
# 調整搖桿死區
python3 main.py --deadzone 0.15

# 調整馬達校準
python3 main.py --left-scale 1.1 --right-scale 1.0

# 降低控制頻率
python3 main.py --frequency 30
```

**永久修改（config.py）：**

```python
# 搖桿設定
JOYSTICK_DEADZONE = 0.1
JOYSTICK_AXIS_LINEAR = 1
JOYSTICK_AXIS_ANGULAR = 0

# 馬達校準
MOTOR_LEFT_SCALE = 1.0
MOTOR_RIGHT_SCALE = 1.0

# 控制頻率
CONTROL_LOOP_FREQUENCY = 20
```

### 5.4 版本控制

```bash
# 查看修改
git status

# 提交變更
git add -A
git commit -m "描述修改內容"

# 推送
git push

# 回復到之前版本
git log --oneline  # 查看歷史
git checkout <commit-hash>
```

---

## 6. 程式碼改進建議

### 6.1 優先級 1：穩定性

#### Serial 重連機制

```python
# arduino_controller.py
def _reconnect(self):
    """嘗試重新連接 Serial"""
    try:
        if self.serial and self.serial.is_open:
            self.serial.close()
        time.sleep(2)
        self._connect()
        return True
    except Exception as e:
        print(f"❌ 重連失敗: {e}")
        return False

def send_command(self, motor_cmd):
    try:
        packet = self._build_packet(motor_cmd)
        self.serial.write(packet)
    except serial.SerialException:
        self.stats['tx_errors'] += 1
        if self.stats['tx_errors'] > 5:
            self._reconnect()
```

#### 看門狗強化

```python
# robot_controller.py
def _check_watchdog(self):
    elapsed = time.time() - self.last_command_time
    if elapsed > EMERGENCY_STOP_TIMEOUT:
        self.arduino.send_command(MotorCommand(0, 0, False))
        print("⚠️ 看門狗觸發：停止馬達")
```

### 6.2 優先級 2：除錯功能

#### 狀態監控面板

```python
def _display_status(self, cmd, sensor):
    print("=" * 60)
    print(f"[遙控器] L:{cmd.linear:+.2f} A:{cmd.angular:+.2f}")
    print(f"[馬達]   左:{cmd.left_pwm:+4d} 右:{cmd.right_pwm:+4d}")
    print(f"[感測器] 前:{sensor.front:3d}cm 右:{sensor.right:3d}cm")
    print(f"[IMU]    Yaw:{sensor.yaw:+.1f}°")
    print("=" * 60)
```

#### 錯誤記錄

```python
import logging

logging.basicConfig(
    filename=f'/tmp/robot_{datetime.now():%Y%m%d_%H%M%S}.log',
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s'
)
```

### 6.3 優先級 3：可維護性

#### Arduino 設定集中管理

所有硬體參數都在 `config.h`：

```cpp
// 腳位定義
#define PIN_ENA 3
#define PIN_IN1 6
// ...

// 時序參數
#define SENSOR_UPDATE_INTERVAL 100
#define COMMAND_TIMEOUT 200

// 除錯開關
#define DEBUG_SERIAL_ENABLED
```

#### 命令列參數覆寫

```python
parser.add_argument('--deadzone', type=float)
parser.add_argument('--frequency', type=int)
parser.add_argument('--left-scale', type=float)
parser.add_argument('--right-scale', type=float)
```

### 6.4 實作優先順序

| 優先級 | 項目 | 預估時間 | 影響 |
|-------|------|---------|------|
| 🔴 高 | Serial 重連 | 30 分鐘 | 穩定性提升 |
| 🔴 高 | 狀態監控 | 20 分鐘 | 除錯效率 |
| 🟡 中 | 看門狗強化 | 15 分鐘 | 可靠性 |
| 🟡 中 | 錯誤記錄 | 15 分鐘 | 問題追溯 |
| 🟢 低 | 參數介面 | 20 分鐘 | 便利性 |

---

## 附錄

### A. 快速指令參考

```bash
# 系統測試
python3 test_joystick.py      # 遙控器
python3 test_ultrasonic.py    # 超聲波
python3 test_mpu6050.py       # IMU
python3 test_motor_only.py    # 馬達

# 運行模式
python3 main.py               # 遙控
python3 autonomous_main.py    # 自走

# Arduino
arduino-cli compile --fqbn arduino:avr:uno arduino/main/
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/main/
```

### B. 設定檔位置

| 設定類型 | 檔案 |
|---------|------|
| Pi 參數 | `raspberry_pi/config.py` |
| Arduino 參數 | `arduino/main/config.h` |

### C. 修改後需重新上傳？

| 修改檔案 | Pi 重新執行 | Arduino 重新上傳 |
|---------|-------------|-----------------|
| `*.py` | ✅ 需要 | ❌ 不需要 |
| `config.h` | ❌ 不需要 | ✅ 需要 |
| `*.cpp` | ❌ 不需要 | ✅ 需要 |

---

**文檔維護者：** Mechatronics Team
**最後更新：** 2025-11-29
