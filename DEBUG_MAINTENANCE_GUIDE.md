# 除錯與維護指南 (Debug & Maintenance Guide)

本指南說明如何有效地除錯（debug）和更新程式，讓您能夠快速找到問題並修改程式。

---

## 🎯 核心理念：分層除錯

我們的程式設計採用**模組化分層架構**，每一層都可以獨立測試：

```
第 1 層：硬體測試 (最底層)
   ↓
第 2 層：單一模組測試
   ↓
第 3 層：模組整合測試
   ↓
第 4 層：完整系統測試 (最上層)
```

**除錯原則**：從下往上測試，哪一層出問題就在哪一層修改。

---

## 🔍 Part 1: 分層除錯策略

### 第 1 層：硬體測試（不需要程式）

**目的**：確認硬體本身沒問題

#### 1.1 測試遙控器

```bash
# 檢查 Linux 是否偵測到遙控器
ls /dev/input/js*
# 應該看到：/dev/input/js0

# 查看遙控器資訊
cat /dev/input/js0 | hexdump -C
# 移動搖桿應該看到數值變化

# 或使用 jstest（需安裝）
sudo apt-get install joystick
jstest /dev/input/js0
```

**如果失敗**：遙控器接收器有問題或未插好

#### 1.2 測試 Serial 連接

```bash
# 在 RPi 上檢查 Serial 埠
ls /dev/serial0 /dev/ttyAMA0
# 應該看到這兩個裝置

# 查看 Serial 設定
stty -F /dev/serial0
```

**如果失敗**：檢查 `/boot/config.txt` 的 Serial 設定

#### 1.3 測試 Arduino 硬體

```bash
# 檢查 Arduino 是否連接
ls /dev/ttyUSB* /dev/ttyACM*

# 使用 Arduino IDE 的 Serial Monitor
# Baud Rate: 9600
# 應該看到 Arduino 的除錯訊息
```

**如果失敗**：Arduino 未連接或程式未上傳

---

### 第 2 層：單一模組測試

**目的**：每個 Python 模組都能獨立執行和測試

#### 2.1 測試遙控器模組 (USB24GReceiver)

```bash
cd ~/robot/raspberry_pi

# 使用專用測試腳本
python3 test_joystick.py
```

**顯示內容**：
- 所有軸的即時數值
- 所有按鈕的狀態
- 幫助您找出正確的軸編號

**如果有問題**：
1. 檢查顯示的軸數量是否正確
2. 移動搖桿看哪個軸有反應
3. 修改 `config.py` 中的 `JOYSTICK_AXIS_LINEAR` 和 `JOYSTICK_AXIS_ANGULAR`

**修改示例**：
```python
# config.py
JOYSTICK_AXIS_LINEAR = 1   # 改成您測試出來的軸編號
JOYSTICK_AXIS_ANGULAR = 0  # 改成您測試出來的軸編號
JOYSTICK_BUTTON_VACUUM = 0 # 改成您測試出來的按鈕編號
```

#### 2.2 測試差速控制模組 (DifferentialDrive)

```bash
# 互動式測試
python3 -i differential_drive.py
```

```python
# 在 Python shell 中測試
>>> from differential_drive import *

# 測試 1: 前進
>>> cmd = VehicleCommand(linear_velocity=1.0, angular_velocity=0.0)
>>> drive = DifferentialDrive()
>>> motor = drive.convert(cmd)
>>> print(f"左輪: {motor.left_pwm}, 右輪: {motor.right_pwm}")
# 應該顯示：左輪: 255, 右輪: 255

# 測試 2: 右轉
>>> cmd = VehicleCommand(linear_velocity=0.0, angular_velocity=1.0)
>>> motor = drive.convert(cmd)
>>> print(f"左輪: {motor.left_pwm}, 右輪: {motor.right_pwm}")
# 應該顯示：左輪: 255, 右輪: -255

# 測試 3: 停止
>>> cmd = VehicleCommand(linear_velocity=0.0, angular_velocity=0.0)
>>> motor = drive.convert(cmd)
>>> print(f"左輪: {motor.left_pwm}, 右輪: {motor.right_pwm}")
# 應該顯示：左輪: 0, 右輪: 0
```

**如果計算錯誤**：檢查 `differential_drive.py` 中的公式

#### 2.3 測試 Serial 通訊模組 (ArduinoController)

```bash
# 使用專用測試腳本（不需遙控器）
python3 test_motor_only.py
```

**顯示內容**：
- 發送的指令封包（十六進位）
- Arduino 回傳的感測器資料
- 每個測試步驟的說明

**如果有問題**：
1. 檢查是否收到 Arduino 回傳的感測器資料
2. 檢查封包格式是否正確（Header 0xAA, Footer 0x55）
3. 開啟 Arduino Serial Monitor 看是否有錯誤訊息

**常見錯誤訊息**：
```
[ERROR] Invalid packet header: XX
```
→ Serial 線路雜訊或 baud rate 不符

```
[ERROR] Checksum mismatch
```
→ 封包傳輸錯誤，檢查接線

---

### 第 3 層：模組整合測試

**目的**：測試多個模組一起工作

```bash
# 完整系統測試（需要遙控器）
python3 main.py
```

**顯示內容**：
```
=== Robot Control System ===
[OK] Joystick connected: Logitech Gamepad
[OK] Serial connected: /dev/serial0

Linear: 0.85  Angular: -0.23  Vacuum: OFF
Left PWM: 217  Right PWM: 158
Left Dist: 045cm ✓  Right Dist: 132cm ✓

Press Ctrl+C to stop
```

**如果有問題看這裡**：

| 症狀 | 可能原因 | 解決方法 |
|------|---------|---------|
| `[ERROR] Joystick not found` | 遙控器未插好 | 回第 2.1 層測試 |
| `[ERROR] Serial connection failed` | Arduino 未連接 | 回第 2.3 層測試 |
| 車子不動 | 馬達接線或程式問題 | 看第 4 層硬體除錯 |
| 車子轉向相反 | PWM 正負號錯誤 | 修改 `config.py` 的 MOTOR_*_SCALE |
| 距離顯示 ✗ | 超音波感測器故障 | 檢查感測器接線 |

---

### 第 4 層：Arduino 程式除錯

#### 4.1 查看 Arduino 除錯訊息

```bash
# 方法 1: Arduino IDE
# Tools → Serial Monitor → 設定 9600 baud

# 方法 2: screen (命令列)
screen /dev/ttyUSB0 9600
# 離開：Ctrl+A, K, Y

# 方法 3: minicom
sudo apt-get install minicom
minicom -D /dev/ttyUSB0 -b 9600
```

**正常的除錯訊息**：
```
[INFO] System initialized
[INFO] Waiting for commands...
[CMD] L:127 R:127 V:OFF
[SENSOR] L:45cm ✓ R:132cm ✓
```

**異常的除錯訊息**：
```
[ERROR] Invalid packet header: A5
  Raw: A5 7F 00 7F 00 00 FE 55
```
→ 封包損壞，檢查 Serial 接線和 baud rate

#### 4.2 調整 Arduino 除錯等級

編輯 `arduino/main/config.h`：

```cpp
// 基本除錯（預設開啟）
#define DEBUG_SERIAL_ENABLED
#define DEBUG_SHOW_COMMANDS
#define DEBUG_SHOW_SENSORS

// 詳細除錯（找問題時開啟）
#define DEBUG_VERBOSE  // ← 取消註解這行
```

開啟 `DEBUG_VERBOSE` 後會顯示：
- 完整封包的十六進位內容
- Checksum 計算過程
- 感測器原始數值

**修改後記得重新上傳 Arduino 程式！**

---

## 🔧 Part 2: 程式更新流程

### 更新 Raspberry Pi 程式

#### 方法 A：直接在 RPi 上修改（推薦）

```bash
# 1. 編輯檔案
nano ~/robot/raspberry_pi/config.py
# 或使用您喜歡的編輯器（vim, VS Code 等）

# 2. 儲存後直接執行測試
python3 test_joystick.py

# 3. 沒問題就執行主程式
python3 main.py
```

**不需要重新安裝套件**，只要修改 .py 檔案就會立即生效。

#### 方法 B：在其他電腦修改後傳輸

```bash
# 1. 在開發電腦上修改程式

# 2. 傳輸單一檔案到 RPi
scp config.py pi@192.168.1.100:~/robot/raspberry_pi/

# 3. SSH 登入 RPi 測試
ssh pi@192.168.1.100
cd ~/robot/raspberry_pi
python3 main.py
```

#### 常見修改位置

| 需求 | 修改檔案 | 修改內容 |
|------|---------|---------|
| 調整搖桿軸編號 | `config.py` | `JOYSTICK_AXIS_LINEAR`, `JOYSTICK_AXIS_ANGULAR` |
| 調整搖桿靈敏度 | `config.py` | `JOYSTICK_DEADZONE`（預設 0.1） |
| 修正馬達轉向 | `config.py` | `MOTOR_LEFT_SCALE = -1.0`（改成負數可反轉） |
| 修改控制頻率 | `config.py` | `CONTROL_LOOP_FREQUENCY`（預設 50Hz） |
| 修改 Serial 埠 | `config.py` | `SERIAL_PORT`（預設 `/dev/serial0`） |
| 修改差速演算法 | `differential_drive.py` | `convert()` 方法內的公式 |

**修改範例**：

```python
# config.py

# 如果左輪轉向相反
MOTOR_LEFT_SCALE = -1.0  # 原本是 1.0

# 如果右輪轉向相反
MOTOR_RIGHT_SCALE = -1.0  # 原本是 1.0

# 如果搖桿太敏感
JOYSTICK_DEADZONE = 0.2  # 原本是 0.1（加大死區）

# 如果搖桿不夠靈敏
JOYSTICK_DEADZONE = 0.05  # 原本是 0.1（縮小死區）
```

---

### 更新 Arduino 程式

#### 修改 Pin 腳定義（最常見）

```bash
# 1. 編輯 config.h
nano ~/mechtronic_4/arduino/main/config.h
```

```cpp
// arduino/main/config.h

// 例如：改變馬達驅動 Pin 腳
#define PIN_ENA 3    // 左輪 PWM
#define PIN_IN1 6    // 左輪方向 A (實際硬體)
#define PIN_IN2 5    // 左輪方向 B (實際硬體)
#define PIN_ENB 11   // 右輪 PWM
#define PIN_IN3 9    // 右輪方向 A
#define PIN_IN4 10   // 右輪方向 B

// 例如：改變超音波感測器 Pin 腳
#define PIN_US_LEFT_TRIG  7
#define PIN_US_LEFT_ECHO  8
#define PIN_US_RIGHT_TRIG A1
#define PIN_US_RIGHT_ECHO A2

// 例如：改變吸塵器 Pin 腳
#define PIN_VACUUM A3   // 吸塵器繼電器 (實際硬體)

// 例如：改變 Serial Pin 腳
#define PIN_SERIAL_RX 4
#define PIN_SERIAL_TX 2
```

```bash
# 2. 重新編譯並上傳
arduino-cli compile --fqbn arduino:avr:uno arduino/main
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno arduino/main
```

#### 修改程式邏輯

```bash
# 1. 編輯對應的檔案
nano ~/mechtronic_4/arduino/main/motor_driver.cpp

# 2. 修改後重新上傳
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno arduino/main
```

**常見修改位置**：

| 需求 | 修改檔案 | 修改內容 |
|------|---------|---------|
| 改 Pin 腳定義 | `config.h` | 所有 `#define PIN_*` |
| 改 Baud Rate | `config.h` | `#define SERIAL_BAUDRATE` |
| 改超時時間 | `config.h` | `#define COMMAND_TIMEOUT` |
| 改感測器頻率 | `config.h` | `#define SENSOR_UPDATE_INTERVAL` |
| 改馬達控制邏輯 | `motor_driver.cpp` | `setMotors()` 方法 |
| 改超音波邏輯 | `ultrasonic_sensor.cpp` | `getDistance()` 方法 |

---

## 🐛 Part 3: 常見問題診斷

### 問題 1：車子完全不動

**診斷步驟**：

```bash
# Step 1: 檢查遙控器是否有訊號
python3 test_joystick.py
# 移動搖桿看數值是否變化

# Step 2: 檢查是否有發送指令給 Arduino
python3 test_motor_only.py
# 看是否有 "Sending command" 訊息

# Step 3: 檢查 Arduino 是否收到指令
# 開啟 Arduino Serial Monitor (9600 baud)
# 應該看到 [CMD] L:XXX R:XXX V:XXX

# Step 4: 檢查馬達接線
# 用三用電表量 L298N 輸出是否有電壓
```

**可能原因**：
- ❌ 遙控器未連接 → 回第 2.1 層測試
- ❌ Serial 未連接 → 回第 2.3 層測試
- ❌ Arduino 未上傳程式 → 重新上傳
- ❌ 馬達驅動電源未接 → 檢查 L298N 12V 電源

---

### 問題 2：車子轉向相反

**快速修正**：

```python
# 編輯 config.py
MOTOR_LEFT_SCALE = -1.0   # 左輪反轉
# 或
MOTOR_RIGHT_SCALE = -1.0  # 右輪反轉

# 儲存後重新執行
python3 main.py
```

**不需要改 Arduino 程式**，只改 Python 就好。

---

### 問題 3：搖桿沒反應或太敏感

**診斷**：

```bash
# 執行測試看軸編號是否正確
python3 test_joystick.py
```

**修正**：

```python
# 編輯 config.py

# 如果軸編號不對
JOYSTICK_AXIS_LINEAR = 1   # 改成測試出來的編號
JOYSTICK_AXIS_ANGULAR = 0  # 改成測試出來的編號

# 如果太敏感（輕碰就動）
JOYSTICK_DEADZONE = 0.2    # 加大死區（預設 0.1）

# 如果不夠靈敏（要推很大力才動）
JOYSTICK_DEADZONE = 0.05   # 縮小死區（預設 0.1）
```

---

### 問題 4：Serial 通訊錯誤

**Arduino 顯示**：
```
[ERROR] Invalid packet header: XX
[ERROR] Checksum mismatch
```

**診斷步驟**：

```bash
# 1. 檢查 baud rate 是否一致
# RPi config.py
SERIAL_BAUDRATE = 57600

# Arduino config.h
#define SERIAL_BAUDRATE 57600
# ↑ 兩邊必須一樣

# 2. 檢查接線
# Arduino Pin 2 (TX) → RPi GPIO 15 (RXD)
# Arduino Pin 4 (RX) → RPi GPIO 14 (TXD)
# Arduino GND → RPi GND

# 3. 降低 baud rate 測試
# 兩邊都改成 9600 試試
```

---

### 問題 5：超音波感測器顯示 ✗

**可能原因**：

1. **感測器接線錯誤**
   ```bash
   # 檢查 config.h 中的 Pin 定義
   #define PIN_US_LEFT_TRIG  7
   #define PIN_US_LEFT_ECHO  8
   #define PIN_US_RIGHT_TRIG A1
   #define PIN_US_RIGHT_ECHO A2
   ```

2. **感測器故障或前方無障礙物**
   ```bash
   # 在 Arduino Serial Monitor 中查看原始數值
   # 開啟 DEBUG_VERBOSE
   ```

3. **測距超出範圍**
   - HC-SR04 有效範圍：2-400cm
   - 太近或太遠都會顯示 ✗

---

## 📝 Part 4: 版本控制建議（選用）

### 使用 Git 管理程式版本

```bash
# 1. 初始化 Git（只需做一次）
cd ~/mechtronic_4
git init

# 2. 加入所有檔案
git add .

# 3. 建立第一個版本
git commit -m "Initial version - working remote control"

# 4. 之後每次修改後提交
git add config.py
git commit -m "Fix: reverse left motor direction"

# 5. 查看修改歷史
git log --oneline

# 6. 回到之前的版本
git checkout <commit-hash>
```

**好處**：
- ✅ 可以看到每次改了什麼
- ✅ 改壞了可以回到之前的版本
- ✅ 可以建立不同的分支測試新功能

---

## 🎯 除錯流程圖

```
發現問題
   ↓
【第 1 步】車子能動嗎？
   ├─ 能動 → 跳到第 2 步
   └─ 不能動 → 從第 1 層開始測試（硬體）

【第 2 步】遙控器有反應嗎？
   ├─ 有反應 → 跳到第 3 步
   └─ 沒反應 → 測試第 2.1 層（test_joystick.py）

【第 3 步】方向正確嗎？
   ├─ 正確 → 跳到第 4 步
   └─ 不正確 → 修改 config.py 的 MOTOR_*_SCALE

【第 4 步】感測器正常嗎？
   ├─ 正常 → 完成！
   └─ 不正常 → 測試第 4.1 層（Arduino Serial Monitor）
```

---

## 📋 快速參考表

### 測試指令清單

| 測試目標 | 指令 | 檔案位置 |
|---------|------|---------|
| 遙控器 | `python3 test_joystick.py` | [raspberry_pi/test_joystick.py](raspberry_pi/test_joystick.py:1) |
| Serial + 馬達 | `python3 test_motor_only.py` | [raspberry_pi/test_motor_only.py](raspberry_pi/test_motor_only.py:1) |
| 完整系統 | `python3 main.py` | [raspberry_pi/main.py](raspberry_pi/main.py:1) |
| Arduino 除錯 | `screen /dev/ttyUSB0 9600` | 或用 Arduino IDE Serial Monitor |

### 設定檔位置

| 設定類型 | 檔案 | 說明 |
|---------|------|------|
| RPi 參數 | [raspberry_pi/config.py](raspberry_pi/config.py:1) | 所有 Python 程式的參數 |
| Arduino 參數 | [arduino/main/config.h](arduino/main/config.h:1) | 所有 Arduino 的參數 |

### 修改後需要重新上傳嗎？

| 修改檔案 | RPi 重新執行 | Arduino 重新上傳 |
|---------|-------------|-----------------|
| `raspberry_pi/*.py` | ✅ 需要 | ❌ 不需要 |
| `arduino/main/config.h` | ❌ 不需要 | ✅ 需要 |
| `arduino/main/*.cpp` | ❌ 不需要 | ✅ 需要 |

---

## 💡 除錯心法

1. **從簡單的開始測試**
   - 先測遙控器，再測馬達，最後測整合

2. **善用測試腳本**
   - 不要每次都跑完整系統，用 `test_*.py` 快速測試單一功能

3. **看除錯訊息**
   - Arduino Serial Monitor 和 Python 的輸出都有詳細資訊

4. **一次改一個地方**
   - 不要同時改很多東西，改了一個測試沒問題再改下一個

5. **備份能動的版本**
   - 程式能動的時候先備份（用 Git 或複製資料夾）

6. **記錄改了什麼**
   - 寫註解或用 Git commit message 記錄修改原因

---

**版本**: 1.0
**更新日期**: 2025-11-04
**作者**: Claude Code

**相關文件**：
- [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md) - 部署指南
- [TESTING_GUIDE.md](TESTING_GUIDE.md) - 測試指南
- [IMPROVEMENTS.md](IMPROVEMENTS.md) - 改進建議
