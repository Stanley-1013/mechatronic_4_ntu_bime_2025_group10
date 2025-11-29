# 機電小車自走清掃系統

**NTU BIME 2025 機電整合四 Group 10**

Raspberry Pi 4 + Arduino Uno 自走清掃機器人，支援遙控與自走兩種模式。

**當前版本：** v2.0 (2025-11-29)
**狀態：** ✅ 期中遙控測試完成、期末自走功能開發中

---

## 📋 系統概述

| 項目 | 規格 |
|-----|------|
| 主控 | Raspberry Pi 4 + Arduino Uno |
| 驅動 | L298N 差動驅動（TT馬達 × 2）|
| 感測器 | 超聲波 × 2（前方/右側）、MPU6050 IMU、USB Camera |
| 通訊 | USB Serial 115200 bps |
| 遙控 | 2.4G USB 遙控器（雙搖桿控制）|
| 自走策略 | 沿右牆 + 紅色區域迴避 |

---

## 🚀 快速開始

### 1. Raspberry Pi

```bash
cd ~/final/raspberry_pi
pip3 install -r requirements.txt
```

### 2. Arduino

用 Arduino IDE 開啟 `arduino/main/main.ino` 並上傳

### 3. 執行

```bash
# 遙控模式
python3 main.py

# 自走模式
python3 autonomous_main.py

# 自走模式（無相機）
python3 autonomous_main.py --no-camera

# 除錯模式
python3 autonomous_main.py --debug
```

---

## 🔌 硬體接線

```
L298N 馬達驅動:
  ENA → D3 (PWM)     IN1 → D6     IN2 → D5
  ENB → D11 (PWM)    IN3 → D9     IN4 → D10

超聲波:
  前方: Trig → D7, Echo → D8
  右側: Trig → A1, Echo → A2

MPU6050 IMU:
  SDA → A4, SCL → A5

吸塵器: Relay → A3

⚠️ 所有 GND 必須共地
```

---

## 📖 文件導覽

### 📚 核心技術文檔

| 文檔 | 說明 | 讀者 |
|-----|------|------|
| **[TECHNICAL_REFERENCE.md](TECHNICAL_REFERENCE.md)** | 完整技術規格（架構、接線、協定、演算法、演進歷史）| 開發者 |
| **[DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md)** | 開發實務（環境建置、測試流程、除錯、維護）| 開發者 |

### 📊 專案報告

| 文檔 | 說明 |
|-----|------|
| **[MIDTERM_REPORT.md](MIDTERM_REPORT.md)** | 期中報告（2025-11-14 測試結果）|
| **[FINAL_AUTONOMOUS_PLAN.md](FINAL_AUTONOMOUS_PLAN.md)** | 期末競賽設計方案（沿右牆策略、時間預算）|

### 📋 軟體工程文件（docs/ 目錄）

這些文件遵循軟體工程標準流程，記錄專案的需求分析、系統設計與測試計畫：

| 文檔 | 說明 |
|-----|------|
| **[01_SRS_軟體需求規格書.md](docs/01_SRS_軟體需求規格書.md)** | 定義系統的功能需求與非功能需求（效能、可靠性等）|
| **[02_SA_系統分析.md](docs/02_SA_系統分析.md)** | 用例圖、活動圖，分析使用者與系統的互動流程 |
| **[03_SD_系統設計.md](docs/03_SD_系統設計.md)** | 類別圖、序列圖，描述軟體模組的設計與互動 |
| **[04_ICD_介面規格.md](docs/04_ICD_介面規格.md)** | **Serial 通訊協定詳細規格**（封包格式、編碼規則、範例）|
| **[05_TDD_測試計畫.md](docs/05_TDD_測試計畫.md)** | 測試策略、測試案例、驗收標準 |

---

## 🗂️ 專案結構

```
mechtronic_4/
├── arduino/main/           # Arduino 程式
│   ├── main.ino            # 主程式
│   ├── config.h            # 硬體參數設定
│   ├── motor_driver.*      # L298N 馬達驅動
│   ├── ultrasonic_sensor.* # HC-SR04 超聲波
│   ├── mpu6050_sensor.*    # MPU6050 IMU (v2.0)
│   ├── vacuum_controller.* # 吸塵器控制
│   └── serial_protocol.*   # Serial 通訊協定
│
├── raspberry_pi/           # Raspberry Pi 程式
│   ├── main.py             # 遙控模式入口
│   ├── autonomous_main.py  # 自走模式入口 (v1.1)
│   ├── config.py           # 系統參數
│   ├── arduino_controller.py
│   ├── differential_drive.py
│   ├── wall_follower.py    # 沿牆控制器 (v1.1)
│   ├── red_detector.py     # 紅色偵測 (v2.0 背景執行緒)
│   ├── imu_processor.py    # IMU 處理 (v2.0)
│   └── test_*.py           # 測試程式
│
├── docs/                   # 軟體工程文件
│   ├── 01_SRS_軟體需求規格書.md
│   ├── 02_SA_系統分析.md
│   ├── 03_SD_系統設計.md
│   ├── 04_ICD_介面規格.md
│   └── 05_TDD_測試計畫.md
│
├── TECHNICAL_REFERENCE.md  # 技術參考（架構+接線+協定）
├── DEVELOPMENT_GUIDE.md    # 開發指南（測試+除錯+維護）
├── MIDTERM_REPORT.md       # 期中報告
└── FINAL_AUTONOMOUS_PLAN.md # 期末計畫
```

---

## 📜 演進歷史

### 版本時間線

```
2025-10-31  v1.0  專案啟動
    ├─ 建立基礎架構
    ├─ L298N 馬達控制
    ├─ 雙超聲波（左/右側）
    ├─ GPIO UART 通訊 (57600 bps)
    └─ 2.4G USB 遙控器整合

2025-11-14  v1.0.1  期中測試
    ├─ 遙控模式驗證完成
    ├─ Serial 通訊穩定性修復
    └─ 期中報告撰寫

2025-11-28  v1.1  期末自走準備
    ├─ 新增 autonomous_main.py
    ├─ 超聲波改為 前方/右側
    ├─ 通訊改為 USB Serial (115200)
    ├─ 新增沿右牆策略
    └─ 新增紅色偵測基礎

2025-11-29  v2.0  MPU6050 整合 ← 當前版本
    ├─ 新增 MPU6050 IMU 支援
    ├─ 感測器封包擴展為 12 bytes
    ├─ 紅色偵測改為背景執行緒
    └─ 更新通訊協定文檔至 v2.0
```

### 關鍵設計決策演變

| 項目 | 初始設計 | 當前設計 | 原因 |
|-----|---------|---------|------|
| 通訊方式 | GPIO UART 57600 | **USB Serial 115200** | 更穩定、免接線 |
| 超聲波配置 | 左側 + 右側 | **前方 + 右側** | 配合沿右牆策略 |
| 遙控方案 | NRF24L01 自製 | **USB 遙控器** | 即插即用、省時 |
| 感測器封包 | 8 bytes | **12 bytes** | 新增 IMU 資料 |
| 紅色偵測 | 同步阻塞 | **背景執行緒** | 避免阻塞控制迴圈 |

---

## 🧪 測試程式

```bash
cd raspberry_pi

# 遙控器測試
python3 test_joystick.py

# 超聲波測試
python3 test_ultrasonic.py

# IMU 測試
python3 test_mpu6050.py

# 紅色偵測測試（顯示視窗）
python3 red_detector.py
```

---

## ⚠️ 故障排除

| 問題 | 解決方法 |
|-----|---------|
| 遙控器找不到 | `sudo modprobe joydev` |
| Serial 連接失敗 | 檢查 `/dev/ttyACM0` 權限 |
| 馬達不轉 | 檢查 L298N 跳線帽、GND 共地 |
| 超聲波讀值 999 | 檢查 Trig/Echo 接線 |
| IMU 無效 | 執行 `i2cdetect -y 1` 確認 0x68 |

詳細除錯指南請參考 [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md#4-除錯指南)

---

## 🎮 操作說明

**遙控模式：**
| 操作 | 控制 |
|-----|------|
| 左搖桿 Y 軸 | 前進/後退 |
| 左搖桿 X 軸 | 左轉/右轉 |
| A 按鈕 | 吸塵器開關 (toggle) |
| Ctrl+C | 停止程式 |

---

## 👥 團隊

**臺大生機 2025 機整四 第十組**

---

**版本：** 2.0
**最後更新：** 2025-11-29
