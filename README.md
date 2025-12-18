# Mechtronic 4 - 機電整合自走車

**NTU BIME 2025 機電整合四 Group 10**

Arduino + Raspberry Pi 雙控制器架構，支援遙控與自走兩種模式的機器人清掃系統。

**當前版本：** v5_competition (2025-12-14)
**狀態：** ✅ 期中遙控完成、🏆 期末競賽完成 (113/120)

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

## 🗂️ 專案結構

```
mechtronic_4/
├── README.md              # 專案總覽（本文件）
├── CLAUDE.md              # Claude Code 配置
├── image/                 # 圖片資源
│
├── midterm/               # 期中階段 - 遙控成功版
│   ├── v1_remote_control/ # 遙控模式實作
│   ├── docs/              # 軟體工程文件（SRS、SA、SD、ICD、TDD）
│   ├── MIDTERM_REPORT.md  # 期中報告
│   ├── DEVELOPMENT_GUIDE.md
│   └── README.md          # 期中導覽
│
├── final/                 # 期末階段 - 自走版本
│   ├── v2_autonomous/     # v2 Pi 決策版（參考）
│   ├── v3_stable/         # v3 Arduino 自主版（穩定）
│   ├── v4_simple/         # v4 簡化實驗版
│   ├── v5_competition/    # 🏆 v5 競賽實戰版（113/120）
│   ├── FINAL_AUTONOMOUS_PLAN.md
│   ├── TECHNICAL_REFERENCE.md
│   ├── EVOLUTION.md       # 版本演進史
│   └── README.md          # 期末導覽
│
└── archive/               # 過時程式碼（供參考）
    ├── arduino/
    └── raspberry_pi/
```

---

## 🚀 快速開始

### 1. 環境準備

```bash
# Raspberry Pi 環境
cd final/v3_stable/raspberry_pi
pip3 install -r requirements.txt
```

### 2. Arduino 上傳

用 Arduino IDE 開啟 `final/v3_stable/arduino/main/main.ino` 並上傳至 Arduino Uno

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

詳細接線圖請參考 [TECHNICAL_REFERENCE.md](final/TECHNICAL_REFERENCE.md#硬體接線)

---

## 📖 階段導覽

### 期中階段（Midterm）

**目標：** 完成遙控模式基礎系統（已達成 ✅）

- **[midterm/README.md](midterm/README.md)** - 期中成果總覽
- **[midterm/MIDTERM_REPORT.md](midterm/MIDTERM_REPORT.md)** - 期中報告（2025-11-14 測試結果）
- **[midterm/docs/](midterm/docs/)** - 軟體工程文件（SRS、SA、SD、ICD、TDD）
- **[midterm/DEVELOPMENT_GUIDE.md](midterm/DEVELOPMENT_GUIDE.md)** - 開發實務指南

### 期末階段（Final）

**目標：** 自走清掃 + 紅色迴避 + 沿牆導航（✅ 已完成）

**競賽成績：** 🏆 113 / 120 (94.2%)
- ✅ 時間分：滿分（2:30 內完成）
- ✅ 清掃分：良好（吸力足夠）
- � 扣分：-7 分（撞牆 ×1、調整 ×1、其他）

- **[final/README.md](final/README.md)** - 期末導覽
- **[final/FINAL_AUTONOMOUS_PLAN.md](final/FINAL_AUTONOMOUS_PLAN.md)** - 競賽設計方案
- **[final/TECHNICAL_REFERENCE.md](final/TECHNICAL_REFERENCE.md)** - 完整技術規格
- **[final/EVOLUTION.md](final/EVOLUTION.md)** - 版本演進史

### 版本選擇

| 版本 | 推薦度 | 用途 | 成績 |
|------|--------|------|------|
| **v5_competition** | 🏆⭐⭐⭐⭐⭐ | 競賽實戰版（已驗證） | 113/120 |
| **v3_stable** | ⭐⭐⭐⭐⭐ | 穩定開發版（推薦） | - |
| **v4_simple** | ⭐⭐⭐ | 快速調試、故障排查 | - |
| **v2_autonomous** | ⭐⭐⭐⭐ | 學習參考、控制論文檔 | - |

### v5 競賽版亮點

**延續 v4 簡化架構，參考 v3 調參經驗**

#### 核心特色
- ✅ **純 Arduino 控制**：無需 Pi 即時介入，反應更快
- ✅ **固定入口動作**：針對競賽場地優化的進場策略
- ✅ **優化角落策略**：擺頭 + 倒退 + 直走組合動作
- ✅ **PID 沿牆控制**：基於雙超聲波差值（右前 - 右後）
- ✅ **簡化感測器**：移除 IMU 依賴，降低故障風險

#### 競賽表現分析
| 項目 | 表現 | 說明 |
|------|------|------|
| 時間管理 | ⭐⭐⭐⭐⭐ | 完美達成 <2:30 時間要求 |
| 吸塵效能 | ⭐⭐⭐⭐ | 主吸嘴設計有效，吸力充足 |
| 路徑規劃 | ⭐⭐⭐⭐ | 沿牆策略穩定，覆蓋率良好 |
| 避碰能力 | ⭐⭐⭐ | 基本功能達成，有改進空間 |

#### 改進方向
- 🔧 吸嘴尺寸可進一步擴大，提升覆蓋範圍
- 🔧 四角落清掃策略可更精細化
- 🔧 避碰邏輯可加強穩健性，減少撞牆

詳見：[final/v5_competition/README.md](final/v5_competition/README.md) 和 [final/EVOLUTION.md](final/EVOLUTION.md)

---

## 🧪 測試程式

```bash
cd final/v3_stable/raspberry_pi

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

詳細除錯指南請參考 [DEVELOPMENT_GUIDE.md](midterm/DEVELOPMENT_GUIDE.md#4-除錯指南)

---

## 🎮 操作說明

### 遙控模式

| 操作 | 控制 |
|-----|------|
| 左搖桿 Y 軸 | 前進/後退 |
| 左搖桿 X 軸 | 左轉/右轉 |
| A 按鈕 | 吸塵器開關 (toggle) |
| Ctrl+C | 停止程式 |

### 自走模式

系統會自動執行沿右牆清掃策略，詳見 [FINAL_AUTONOMOUS_PLAN.md](final/FINAL_AUTONOMOUS_PLAN.md)

---

## 👥 團隊

**臺大生機 2025 機電整合四 第十組**

---

**版本：** 5.0 (v5_competition)
**最後更新：** 2025-12-14
**競賽日期：** 2025-12-14
