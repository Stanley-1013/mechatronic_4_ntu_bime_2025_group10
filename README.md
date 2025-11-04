# 機電小車遙控系統

基於 Raspberry Pi 4 + Arduino Uno 的自主導航機器人系統。

## 📋 系統概述

- **硬體平台**: Raspberry Pi 4 + Arduino Uno
- **驅動系統**: L298N 雙馬達差動驅動
- **感測器**: HC-SR04 超聲波 × 2（左右側）
- **遙控方式**: 2.4G USB 遙控器（即插即用）
- **通訊協定**: UART Serial 二進位封包（57600 bps）

## 📖 文件導覽

根據您的角色和需求，請參考以下文件：

### 🔧 硬體組必讀
- **[ARDUINO_HARDWARE_TESTING.md](ARDUINO_HARDWARE_TESTING.md)** - Arduino 硬體接線與測試指南
  - 詳細接線對照表（L298N、超聲波、吸塵器）
  - 逐步測試流程（感測器、馬達、吸塵器）
  - 常見問題排除（無法上傳、感測器無反應、馬達不轉等）
  - 適合硬體組完成接線後的功能驗證

### 🚀 快速上手
- **[QUICK_START.md](QUICK_START.md)** - 快速開始指南
  - 完整環境設定步驟
  - 系統啟動與操作說明
  - 適合第一次使用本系統的使用者

### 🏗️ 系統架構
- **[SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)** - 系統架構文件
  - 硬體架構設計
  - 軟體模組說明
  - Serial 通訊協定

- **[REMOTE_CONTROL_ARCHITECTURE.md](REMOTE_CONTROL_ARCHITECTURE.md)** - 遙控方案設計
  - USB 遙控器整合方案
  - 差動驅動演算法

### 🧪 測試與部署
- **[TESTING_GUIDE.md](TESTING_GUIDE.md)** - 完整測試指南
  - 單元測試
  - 整合測試
  - 系統測試流程

- **[DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)** - 部署指南
  - 生產環境部署
  - 開機自動啟動設定

### 📋 軟體工程文件（docs/ 目錄）
- **[01_SRS_軟體需求規格書.md](docs/01_SRS_軟體需求規格書.md)** - 功能與非功能需求
- **[02_SA_系統分析.md](docs/02_SA_系統分析.md)** - 系統分析與用例圖
- **[03_SD_系統設計.md](docs/03_SD_系統設計.md)** - 類別圖與序列圖
- **[04_ICD_介面規格.md](docs/04_ICD_介面規格.md)** - Serial 通訊協定詳細規格
- **[05_TDD_測試計畫.md](docs/05_TDD_測試計畫.md)** - 測試計畫與測試案例

### 📊 專案管理
- **[PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)** - 專案總覽與進度追蹤
- **[IMPROVEMENTS.md](IMPROVEMENTS.md)** - 未來改進建議
- **[DEBUG_MAINTENANCE_GUIDE.md](DEBUG_MAINTENANCE_GUIDE.md)** - 除錯與維護指南

## 🗂️ 專案結構

```
mechtronic_4/
├── docs/                           # 軟體工程文件
│   ├── 01_SRS_軟體需求規格書.md
│   ├── 02_SA_系統分析.md
│   ├── 03_SD_系統設計.md
│   ├── 04_ICD_介面規格.md
│   └── 05_TDD_測試計畫.md
│
├── raspberry_pi/                   # Raspberry Pi 程式
│   ├── config.py                   # 系統設定
│   ├── differential_drive.py       # 差動驅動演算法
│   ├── usb_24g_receiver.py         # USB 遙控器輸入
│   ├── arduino_controller.py       # Serial 通訊控制
│   ├── robot_controller.py         # 主控制邏輯
│   ├── main.py                     # 主程式進入點
│   ├── requirements.txt            # Python 依賴套件
│   └── tests/                      # 單元測試
│
├── arduino/main/                   # Arduino 程式
│   ├── main.ino                    # 主程式
│   ├── motor_driver.h/cpp          # L298N 馬達驅動
│   ├── ultrasonic_sensor.h/cpp     # HC-SR04 超聲波
│   ├── vacuum_controller.h/cpp     # 吸塵器控制
│   └── serial_protocol.h/cpp       # Serial 協定
│
├── SYSTEM_ARCHITECTURE.md          # 系統架構文件
├── REMOTE_CONTROL_ARCHITECTURE.md  # 遙控方案設計
├── PROJECT_SUMMARY.md              # 專案總覽
├── QUICK_START.md                  # 快速開始指南
└── README.md                       # 本文件
```

## 🚀 快速開始

### 1. Raspberry Pi 環境設定

```bash
# 安裝系統套件
sudo apt update
sudo apt install python3-pygame python3-serial joystick

# 安裝 Python 依賴
cd raspberry_pi
pip3 install -r requirements.txt

# 測試遙控器連接
jstest /dev/input/js0

# 啟用 Serial UART (需重開機)
sudo raspi-config
# → Interface Options → Serial Port
# → Login shell over serial: No
# → Serial port hardware: Yes
sudo reboot
```

### 2. Arduino 程式上傳

```bash
# 使用 Arduino IDE
# 1. 開啟 arduino/main/main.ino
# 2. 選擇板子: Arduino Uno
# 3. 選擇序列埠: /dev/ttyUSB0 (或 /dev/ttyACM0)
# 4. 上傳

# 或使用 arduino-cli
arduino-cli compile --fqbn arduino:avr:uno arduino/main/
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno arduino/main/
```

### 3. 硬體接線與測試

**🔧 硬體組請優先閱讀**：[ARDUINO_HARDWARE_TESTING.md](ARDUINO_HARDWARE_TESTING.md)

該文件包含：
- 完整的接線對照表
- Arduino 獨立測試步驟
- 常見問題排除方法

**快速參考接線**：
- **L298N**: ENA→D3, IN1→D5, IN2→D6, ENB→D11, IN3→D9, IN4→D10
- **左側超聲波**: Trig→D7, Echo→D8
- **右側超聲波**: Trig→A1, Echo→A2
- **吸塵器**: D12
- **Serial**: Arduino TX(D2)→Pi RX(GPIO15), Arduino RX(D4)→Pi TX(GPIO14)
- **⚠️ 共地**: 所有 GND 必須連接

完整架構說明請參考 [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)

### 4. 啟動系統

```bash
# 啟動機器人控制程式
cd raspberry_pi
python3 main.py

# 啟用除錯模式
python3 main.py --debug

# 不顯示感測器資料
python3 main.py --no-sensor
```

## 🎮 操作說明

- **左搖桿 Y 軸**: 前進/後退
- **左搖桿 X 軸**: 左轉/右轉
- **A 按鈕 (Button 0)**: 吸塵器開關（toggle）
- **Ctrl+C**: 停止程式

## 🧪 測試

### Python 單元測試

```bash
cd raspberry_pi

# 測試差動驅動演算法
python3 differential_drive.py

# 測試 USB 遙控器（需插入遙控器）
python3 usb_24g_receiver.py

# 測試 Serial 通訊（需連接 Arduino）
python3 arduino_controller.py

# 執行完整測試套件（需安裝 pytest）
pytest tests/ -v
```

### Arduino 測試

**硬體接線測試**：參考 [ARDUINO_HARDWARE_TESTING.md](ARDUINO_HARDWARE_TESTING.md)

**軟體測試程式**：參考 [docs/05_TDD_測試計畫.md](docs/05_TDD_測試計畫.md)

## 📡 Serial 通訊協定

### Pi → Arduino 馬達指令封包 (8 bytes)

| Byte | 欄位 | 說明 |
|------|------|------|
| 0 | Header | 0xAA |
| 1-2 | Left PWM | -255 ~ +255 (int16, little-endian) |
| 3-4 | Right PWM | -255 ~ +255 (int16, little-endian) |
| 5 | Flags | bit0=vacuum, bit1-7=reserved |
| 6 | Checksum | XOR of bytes 1-5 |
| 7 | Footer | 0x55 |

### Arduino → Pi 感測器封包 (8 bytes)

| Byte | 欄位 | 說明 |
|------|------|------|
| 0 | Header | 0xBB |
| 1-2 | Left Distance | 2-400 cm (uint16), 999=無效 |
| 3-4 | Right Distance | 2-400 cm (uint16), 999=無效 |
| 5 | Status | bit0=left_valid, bit1=right_valid |
| 6 | Checksum | XOR of bytes 1-5 |
| 7 | Footer | 0x66 |

詳細協定請參考 [docs/04_ICD_介面規格.md](docs/04_ICD_介面規格.md)

## ⚠️ 故障排除

### 問題 1: 找不到遙控器 `/dev/input/js0`

```bash
# 載入 joystick 核心模組
sudo modprobe joydev

# 檢查 USB 設備
lsusb

# 查看核心訊息
dmesg | tail -20
```

### 問題 2: Serial 無法通訊

```bash
# 檢查 Serial 是否啟用
ls -l /dev/serial0

# 測試 Serial 連接
sudo minicom -D /dev/serial0 -b 57600

# 檢查權限
sudo usermod -a -G dialout $USER
# 登出後重新登入
```

### 問題 3: 馬達不轉

- 檢查 L298N 跳線是否拔除（ENA/ENB）
- 檢查電源電壓（9-12V）
- 檢查接線是否正確
- 使用 Arduino Serial Monitor 查看除錯訊息

### 問題 4: 超聲波讀值不穩定

- 檢查 VCC 是否為 5V
- 增加測距間隔時間
- 避免兩側同時觸發
- 檢查安裝角度（建議 45°）

## 📚 參考文件

### 硬體與測試
- **[ARDUINO_HARDWARE_TESTING.md](ARDUINO_HARDWARE_TESTING.md)** - Arduino 硬體接線與測試指南 ⭐ 硬體組必讀
- **[TESTING_GUIDE.md](TESTING_GUIDE.md)** - 完整測試指南
- **[DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)** - 部署指南

### 快速上手與架構
- **[QUICK_START.md](QUICK_START.md)** - 快速開始指南
- **[SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)** - 系統架構文件
- **[REMOTE_CONTROL_ARCHITECTURE.md](REMOTE_CONTROL_ARCHITECTURE.md)** - 遙控方案設計

### 專案管理
- **[PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)** - 專案總覽與進度追蹤
- **[IMPROVEMENTS.md](IMPROVEMENTS.md)** - 未來改進建議
- **[DEBUG_MAINTENANCE_GUIDE.md](DEBUG_MAINTENANCE_GUIDE.md)** - 除錯與維護指南
- **[INSTALLATION.md](INSTALLATION.md)** - 安裝指南

### 軟體工程文件（docs/ 目錄）
- **[docs/01_SRS_軟體需求規格書.md](docs/01_SRS_軟體需求規格書.md)** - 軟體需求規格
- **[docs/02_SA_系統分析.md](docs/02_SA_系統分析.md)** - 系統分析
- **[docs/03_SD_系統設計.md](docs/03_SD_系統設計.md)** - 系統設計
- **[docs/04_ICD_介面規格.md](docs/04_ICD_介面規格.md)** - 介面規格
- **[docs/05_TDD_測試計畫.md](docs/05_TDD_測試計畫.md)** - 測試計畫

## 🎯 開發階段

### ✅ Phase 1: 遙控模式（當前）
- [x] 硬體接線與測試
- [x] Arduino 馬達控制
- [x] Arduino 超聲波讀取
- [x] Serial 二進位通訊
- [x] USB 遙控器輸入
- [x] 差動驅動演算法
- [x] 完整系統整合
- [ ] 實機測試與調校

### 🔜 Phase 2: 視覺自走（未來）
- [ ] Pi Camera 影像擷取
- [ ] OpenCV 路徑偵測
- [ ] 自主導航邏輯
- [ ] 障礙迴避策略

## 📝 授權

本專案為機電整合四專題作品，僅供教育與學習用途。

## 👥 作者

臺大生機 2025 機整四 第十組
Mechatronics Team

---

**版本**: 1.0
**最後更新**: 2025-10-31
