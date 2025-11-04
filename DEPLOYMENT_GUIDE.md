# 部署指南 (Deployment Guide)

本指南說明如何將程式部署到 Raspberry Pi 和 Arduino Uno。

---

## 📦 部署總覽

### 需要部署的檔案

| 目標裝置 | 來源資料夾 | 檔案數量 | 部署方式 |
|---------|-----------|---------|---------|
| **Raspberry Pi** | `raspberry_pi/` | 8 個 Python 檔案 | 複製檔案 + 安裝套件 |
| **Arduino Uno** | `arduino/main/` | 9 個檔案 (.ino + .h + .cpp) | Arduino IDE 上傳 |

---

## 🍓 Part 1: Raspberry Pi 部署

### 方法 A：直接在 RPi 上開發（推薦）

如果您直接在 Raspberry Pi 上操作：

```bash
# 1. 確認您在專案目錄
cd /home/han/claude_project/mechtronic_4

# 2. 檢查 Python 版本（需要 Python 3.7+）
python3 --version

# 3. 安裝相依套件
cd raspberry_pi
pip3 install -r requirements.txt

# 4. 測試安裝是否成功
python3 -c "import pygame, serial; print('安裝成功！')"

# 完成！程式已經在正確位置
```

### 方法 B：從其他電腦傳輸到 RPi

如果您在其他電腦上開發，需要傳輸到 RPi：

#### B-1. 使用 SCP 傳輸（透過網路）

```bash
# 在您的開發電腦上執行（假設 RPi IP 是 192.168.1.100）

# 傳輸整個 raspberry_pi 資料夾
scp -r /home/han/claude_project/mechtronic_4/raspberry_pi pi@192.168.1.100:~/robot/

# 傳輸 docs 資料夾（選用）
scp -r /home/han/claude_project/mechtronic_4/docs pi@192.168.1.100:~/robot/

# 傳輸測試指南（選用）
scp /home/han/claude_project/mechtronic_4/TESTING_GUIDE.md pi@192.168.1.100:~/robot/
```

然後 SSH 登入 RPi 安裝套件：

```bash
# SSH 登入 RPi
ssh pi@192.168.1.100

# 進入專案目錄
cd ~/robot/raspberry_pi

# 安裝相依套件
pip3 install -r requirements.txt
```

#### B-2. 使用 USB 隨身碟傳輸（無網路）

1. 將整個 `raspberry_pi/` 資料夾複製到 USB 隨身碟
2. 將 USB 插入 Raspberry Pi
3. 在 RPi 上執行：

```bash
# 掛載 USB（通常會自動掛載到 /media/pi/）
ls /media/pi/

# 複製檔案到 RPi
mkdir -p ~/robot
cp -r /media/pi/YOUR_USB_NAME/raspberry_pi ~/robot/

# 進入目錄並安裝套件
cd ~/robot/raspberry_pi
pip3 install -r requirements.txt
```

### ✅ 驗證 RPi 部署成功

```bash
cd ~/robot/raspberry_pi

# 檢查檔案是否都存在
ls -l
# 應該看到：
# config.py
# differential_drive.py
# usb_24g_receiver.py
# arduino_controller.py
# robot_controller.py
# main.py
# test_joystick.py
# test_motor_only.py
# requirements.txt

# 測試 Python 模組能否載入
python3 -c "from config import *; from differential_drive import *; print('模組載入成功！')"
```

---

## 🤖 Part 2: Arduino 部署

### 需要上傳的檔案清單

Arduino 專案位於 `arduino/main/` 資料夾：

```
arduino/main/
├── main.ino                 # 主程式
├── config.h                 # 配置檔（Pin 定義、參數）
├── motor_driver.h           # 馬達驅動標頭檔
├── motor_driver.cpp         # 馬達驅動實作
├── ultrasonic_sensor.h      # 超音波感測器標頭檔
├── ultrasonic_sensor.cpp    # 超音波感測器實作
├── vacuum_controller.h      # 吸塵器控制標頭檔
├── vacuum_controller.cpp    # 吸塵器控制實作
├── serial_protocol.h        # Serial 協定標頭檔
└── serial_protocol.cpp      # Serial 協定實作
```

### 方法 A：使用 Arduino IDE（最簡單）

#### Step 1: 準備檔案

**選項 1 - 直接在 RPi 上操作**：
- 檔案已經在 `/home/han/claude_project/mechtronic_4/arduino/main/`

**選項 2 - 從其他電腦**：
- 將整個 `arduino/main/` 資料夾複製到您的電腦

#### Step 2: 開啟 Arduino IDE

```bash
# 在 RPi 或您的電腦上啟動 Arduino IDE
arduino
```

#### Step 3: 開啟專案

1. File → Open
2. 選擇 `arduino/main/main.ino`
3. Arduino IDE 會自動載入同資料夾內的所有 .h 和 .cpp 檔案
4. 檢查 IDE 底部的分頁，應該會看到：
   - main.ino
   - config.h
   - motor_driver.h
   - motor_driver.cpp
   - ultrasonic_sensor.h
   - ultrasonic_sensor.cpp
   - vacuum_controller.h
   - vacuum_controller.cpp
   - serial_protocol.h
   - serial_protocol.cpp

#### Step 4: 配置 Arduino IDE

1. **選擇開發板**：Tools → Board → Arduino AVR Boards → **Arduino Uno**
2. **選擇連接埠**：Tools → Port → `/dev/ttyUSB0` 或 `/dev/ttyACM0`
   - 在 Windows 上可能是 `COM3`, `COM4` 等
   - 在 Mac 上可能是 `/dev/cu.usbserial-*`

#### Step 5: 編譯與上傳

1. 點擊 **Verify (✓)** 按鈕編譯程式
   - 檢查是否有錯誤訊息
   - 應該顯示：「Done compiling」

2. 點擊 **Upload (→)** 按鈕上傳程式
   - 等待上傳完成
   - 應該顯示：「Done uploading」

#### Step 6: 驗證上傳成功

打開 Serial Monitor（Tools → Serial Monitor）：
- 設定 Baud Rate: **9600**（因為 DEBUG_SERIAL_ENABLED 使用硬體 Serial）
- 應該會看到 Arduino 的除錯訊息：

```
[INFO] System initialized
[INFO] Waiting for commands...
```

### 方法 B：使用 arduino-cli（命令列，適合 RPi）

如果您在 Raspberry Pi 上且想用命令列：

#### Step 1: 安裝 arduino-cli

```bash
# 下載並安裝 arduino-cli
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# 初始化配置
arduino-cli config init

# 更新核心索引
arduino-cli core update-index

# 安裝 Arduino AVR 核心
arduino-cli core install arduino:avr
```

#### Step 2: 編譯程式

```bash
cd /home/han/claude_project/mechtronic_4/arduino/main

# 編譯
arduino-cli compile --fqbn arduino:avr:uno .
```

#### Step 3: 上傳程式

```bash
# 找出 Arduino 的連接埠
arduino-cli board list

# 上傳程式（假設連接埠是 /dev/ttyUSB0）
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno .
```

#### Step 4: 監看 Serial 輸出

```bash
# 安裝 screen（如果還沒有）
sudo apt-get install screen

# 監看 Serial 輸出（Baud rate: 9600）
screen /dev/ttyUSB0 9600

# 離開 screen: 按 Ctrl+A 再按 K，然後按 Y
```

### ✅ 驗證 Arduino 部署成功

成功的指標：

1. **LED 閃爍**：Arduino Uno 板載 LED（Pin 13）會在上傳時閃爍
2. **Serial 輸出**：Serial Monitor 顯示初始化訊息
3. **無錯誤訊息**：編譯和上傳過程無錯誤

---

## 🔗 Part 3: 整合測試

當兩邊都部署完成後：

### Step 1: 連接硬體

1. **Arduino → RPi Serial 連接**：
   - Arduino Pin 2 (TX) → RPi GPIO 15 (RXD)
   - Arduino Pin 4 (RX) → RPi GPIO 14 (TXD)
   - Arduino GND → RPi GND

2. **USB 遙控器**：
   - 接收器插入 RPi 的 USB 埠

3. **電源**：
   - Arduino 透過 USB 供電（連接到電腦或行動電源）
   - RPi 使用 5V/3A 電源供應器

### Step 2: 執行測試

```bash
cd ~/robot/raspberry_pi

# 測試 1: 遙控器測試
python3 test_joystick.py

# 測試 2: 馬達控制測試（不需遙控器）
python3 test_motor_only.py

# 測試 3: 完整系統測試
python3 main.py
```

### Step 3: 參考測試指南

詳細的測試步驟請參考 [TESTING_GUIDE.md](TESTING_GUIDE.md)。

---

## 📁 建議的 RPi 目錄結構

```
/home/pi/
└── robot/                          # 專案主目錄
    ├── raspberry_pi/               # Python 程式
    │   ├── config.py
    │   ├── differential_drive.py
    │   ├── usb_24g_receiver.py
    │   ├── arduino_controller.py
    │   ├── robot_controller.py
    │   ├── main.py
    │   ├── test_joystick.py
    │   ├── test_motor_only.py
    │   └── requirements.txt
    ├── docs/                       # 文件（選用）
    │   ├── 01_SRS_軟體需求規格書.md
    │   ├── 02_SA_系統分析.md
    │   ├── 03_SD_系統設計.md
    │   ├── 04_ICD_介面規格.md
    │   └── 05_TDD_測試計畫.md
    └── TESTING_GUIDE.md           # 測試指南（選用）
```

---

## 🛠️ 常見問題

### Q1: Arduino IDE 找不到連接埠？

**解決方法**：
```bash
# 檢查 Arduino 是否連接
ls /dev/ttyUSB* /dev/ttyACM*

# 如果沒有權限
sudo usermod -a -G dialout $USER
# 登出後重新登入
```

### Q2: pip3 安裝失敗？

**解決方法**：
```bash
# 更新 pip
pip3 install --upgrade pip

# 使用 sudo（如果需要）
sudo pip3 install -r requirements.txt
```

### Q3: 如何確認檔案傳輸成功？

**解決方法**：
```bash
# 檢查檔案數量
ls raspberry_pi/*.py | wc -l
# 應該顯示 8

ls arduino/main/*.ino arduino/main/*.h arduino/main/*.cpp | wc -l
# 應該顯示 9
```

### Q4: Arduino 編譯錯誤？

**確認事項**：
- 所有 .h 和 .cpp 檔案都在 `arduino/main/` 資料夾內
- 資料夾名稱必須是 `main`，檔名必須是 `main.ino`
- 開發板選擇正確：Arduino Uno

### Q5: 想要修改 Pin 腳配置？

**修改位置**：
- 編輯 `arduino/main/config.h`
- 所有 Pin 定義都集中在這個檔案
- 修改後重新編譯並上傳

---

## 📋 快速部署檢查清單

### Raspberry Pi
- [ ] 檔案已複製到 RPi
- [ ] Python 3.7+ 已安裝
- [ ] pip3 已安裝
- [ ] pygame 已安裝（`pip3 install pygame`）
- [ ] pyserial 已安裝（`pip3 install pyserial`）
- [ ] 可以執行 `python3 test_joystick.py`

### Arduino
- [ ] Arduino IDE 或 arduino-cli 已安裝
- [ ] 開發板選擇：Arduino Uno
- [ ] 連接埠已選擇
- [ ] 所有 9 個檔案在同一資料夾
- [ ] 編譯成功（無錯誤）
- [ ] 上傳成功
- [ ] Serial Monitor 顯示訊息

### 硬體連接
- [ ] Arduino TX → RPi RX
- [ ] Arduino RX → RPi TX
- [ ] 共地（GND 連接）
- [ ] USB 遙控器插入 RPi
- [ ] 電源供應正常

---

## 🎯 下一步

部署完成後，請參考 [TESTING_GUIDE.md](TESTING_GUIDE.md) 進行系統測試。

---

**版本**: 1.0
**更新日期**: 2025-11-04
**作者**: Claude Code
