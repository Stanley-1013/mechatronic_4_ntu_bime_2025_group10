# 期末階段成果總覽

**機電整合自走清掃系統 - 競賽版本彙總**

**階段目標：** 自走清掃 + 紅色迴避 + 沿牆導航
**競賽日期：** 2025-12-14
**狀態：** 🏆 競賽完成

---

## 一、競賽目標

### 1.1 場地規格
- **場地大小：** 300cm × 300cm
- **紙屑分布：** 4 角落（藍色 0.6分/片）+ 中央（綠色 0.2分/片）
- **禁區：** 右上角紅色區域（-0.5分/片）

### 1.2 評分標準

| 項目 | 分數 | 說明 |
|------|------|------|
| **速度** | 70 分 | 完成時間決定（< 2:30 = 滿分）|
| **吸力** | 40 分 | 紙屑回收量 |
| **外觀** | 5 分 | 完整性 + 美觀 |
| **總分** | **120 分** | |

### 1.3 策略概述

```
快速沿右牆清掃 > 完美覆蓋
├─ 優先目標：< 2:30 內完成繞一圈
├─ 四角落掃描確保紙屑收集
├─ 紅色區域迴避（安全優先）
└─ 全程開啟吸塵器
```

詳見：[FINAL_AUTONOMOUS_PLAN.md](./FINAL_AUTONOMOUS_PLAN.md)

---

## 二、版本概覽

| 版本 | 狀態 | 架構 | 主要特色 | 檔案位置 |
|------|------|------|--------|---------|
| **v2_autonomous** | 期末實驗 | Pi 決策 + Arduino 執行 | IMU PID 控制、紅色偵測 | [`v2_autonomous/`](./v2_autonomous/) |
| **v3_stable** | ✅ 穩定版 | Arduino 自主控制 | 雙超聲波沿牆、PD 參數、角落擺頭 | [`v3_stable/`](./v3_stable/) |
| **v4_simple** | 🧪 對比版 | 簡化超聲波方案 | 防突波濾波、快速原型 | [`v4_simple/`](./v4_simple/) |
| **v5_competition** | 🏆 實戰版 | Arduino 自主控制 | 競賽驗證版本 | `v5_competition/` |

### 版本選擇指南

#### ✅ 推薦：v3_stable（期末主版本）
- **架構：** Arduino 獨立決策，實時性好
- **特色：**
  - 雙超聲波（前方 + 右側）沿牆控制
  - MPU6050 IMU 航向輔助（選項）
  - PD 控制距離修正
  - 角落擺頭清掃
  - 無牆自動補償
- **適用：** 正式競賽
- **穩定性：** ⭐⭐⭐⭐⭐

#### 🧪 備選：v4_simple（快速驗證）
- **架構：** 簡化超聲波方案
- **特色：**
  - 純基於感測器差值（rightFront - rightRear）
  - 無需 IMU
  - 程式碼簡潔
  - 適合故障排查
- **適用：** 調試、對比測試
- **穩定性：** ⭐⭐⭐

#### 📚 參考：v2_autonomous（控制論文檔）
- **架構：** Pi 高階決策 + Arduino 低階執行
- **特色：**
  - 完整 IMU PID 實裝
  - 感測器融合設計
  - 紅色偵測背景執行緒
  - 文檔完整
- **適用：** 學習參考
- **穩定性：** ⭐⭐⭐⭐

---

## 三、快速開始

### 3.1 環境準備

**硬體檢查清單：**
```
✅ Arduino Uno + USB 線
✅ Raspberry Pi 4 + SD 卡
✅ HC-SR04 × 2（前方 + 右側）
✅ MPU6050 IMU（v3 可選）
✅ L298N 馬達驅動 + TT 馬達 × 2
✅ USB Camera（紅色偵測）
```

**軟體環境：**
```bash
# Raspberry Pi 端
python3 --version  # >= 3.8
pip3 install opencv-python numpy

# Arduino 端
# Arduino IDE 1.8.13+ 或 VS Code + PlatformIO
```

### 3.2 部署 v3_stable（推薦）

**步驟 1：上傳 Arduino 程式**
```bash
# 使用 Arduino IDE 或 PlatformIO
cd v3_stable/arduino/main/
# 上傳 main.ino 到 Arduino Uno
```

**步驟 2：配置 Raspberry Pi（如需紅色偵測）**
```bash
cd v3_stable/raspberry_pi/
python3 main.py --mode autonomous
```

**步驟 3：驗證通訊**
```bash
python3 test_connection.py
# 預期輸出：感測器數據正常更新
```

詳細步驟見：[v3_stable/README.md](./v3_stable/README.md)

---

## 四、技術架構

### 4.1 硬體配置

```
┌─────────────────────────────────────┐
│  電源：18650 電池 3S (11.1V)         │
│  └─ L298N + Arduino (5V)            │
│  └─ Raspberry Pi 4 (5V/3A)          │
└─────────────────────────────────────┘
           │              │
      ┌────▼──────────────▼────┐
      │   Arduino Uno (主控)     │
      │   ┌──────────────────┐  │
      │   │ • HC-SR04 × 2    │  │  前方 + 右側
      │   │ • MPU6050 (選項)  │  │  IMU 航向
      │   │ • L298N 驅動     │  │  PWM 控制
      │   │ • Relay 吸塵器   │  │
      │   └──────────────────┘  │
      └────┬──────────────┬─────┘
           │              │
    ┌──────▼──┐    ┌──────▼──────┐
    │  左右馬達 │    │ USB Camera  │
    │(TT 馬達) │    │(紅色偵測)   │
    └──────────┘    └──────┬──────┘
                           │
                    ┌──────▼────┐
                    │ Raspberry  │
                    │ Pi 4       │
                    │(監測層)    │
                    └───────────┘
```

### 4.2 通訊協定

**Arduino ↔ Raspberry Pi (USB Serial 115200 bps)**

**Pi → Arduino（8 bytes）：**
```
[0xAA] [左輪PWM-L] [左輪PWM-H] [右輪PWM-L] [右輪PWM-H] [FLAGS] [CHK] [0x55]
```

**Arduino → Pi（12 bytes）：**
```
[0xBB] [前距H] [前距L] [右距H] [右距L] [YawH] [YawL] [GyroZ] [STATUS] [_] [CHK] [0x66]
```

詳見：[TECHNICAL_REFERENCE.md §4](./TECHNICAL_REFERENCE.md)

### 4.3 控制流程

**v3_stable（Arduino 自主模式）：**
```
感測器讀取（超聲波 + IMU）
         ↓
沿牆控制邏輯（PD 距離校正）
         ↓
左右輪 PWM 輸出
         ↓
馬達驅動 L298N
```

**決策優先級：**
1. **紅色偵測**（安全）→ 迴避
2. **前方避障**（前方 < 18cm） → 左轉
3. **右側沿牆**（距離修正）→ PD 控制
4. **IMU 輔助**（直線穩定）→ 角度補償

---

## 五、軟體工程文件

### v3_stable 期末文件（5 份）

位置：[`v3_stable/docs/`](./v3_stable/docs/)

| 編號 | 文件 | 說明 |
|------|------|------|
| 01 | **SRS_軟體需求規格** | 功能需求、非功能需求、系統範圍 |
| 02 | **SA_系統分析** | 設計目標、系統邊界、演員分析 |
| 03 | **SD_系統設計** | 架構設計、狀態機、演算法設計 |
| 04 | **ICD_介面控制** | 硬體腳位、通訊協定、感測器規格 |
| 05 | **TDD_測試設計** | 測試計畫、測試案例、驗收標準 |

---

## 六、目錄結構

```
final/
├── README.md                        ← 本文件（期末導覽）
│
├── FINAL_AUTONOMOUS_PLAN.md        ← 競賽規劃文檔
│   └─ 場地分析、路線設計、時間預算
│
├── TECHNICAL_REFERENCE.md          ← 技術參考
│   └─ 系統規格、硬體配置、通訊協定
│
├── EVOLUTION.md                    ← 版本演進史
│   └─ v1~v5 詳細歷程、決策軌跡
│
├── v2_autonomous/                  ← v2：Pi 決策版（參考）
│   ├── README.md
│   ├── DESIGN.md                   ← 詳細設計文檔
│   ├── raspberry_pi/
│   │   ├── autonomous_main.py
│   │   ├── wall_follower.py
│   │   ├── red_detector.py
│   │   ├── imu_processor.py
│   │   └── ...
│   ├── arduino/main/
│   │   └── main.ino
│   └── tests/
│
├── v3_stable/                      ← v3：Arduino 自主版（推薦）
│   ├── README.md
│   ├── arduino/main/
│   │   ├── main.ino                ← 入口程式
│   │   ├── config.h                ← 參數配置
│   │   └── ...
│   ├── raspberry_pi/               ← 可選：紅色偵測
│   │   └── main.py
│   └── docs/                       ← 期末軟工文件 (5 份)
│       ├── 01_SRS_軟體需求規格.md
│       ├── 02_SA_系統分析.md
│       ├── 03_SD_系統設計.md
│       ├── 04_ICD_介面控制.md
│       └── 05_TDD_測試設計.md
│
├── v4_simple/                      ← v4：簡化對比版
│   ├── README.md
│   ├── arduino/main/
│   │   └── main.ino
│   └── pi/
│       └── main.py
│
└── v5_competition/                 ← v5：實戰版（競賽驗證）
    └── 基於 v3_stable 競賽驗證
```

---

## 七、技術文檔索引

### 核心文檔

| 文檔 | 用途 | 讀者 |
|------|------|------|
| **[FINAL_AUTONOMOUS_PLAN.md](./FINAL_AUTONOMOUS_PLAN.md)** | 競賽規劃、場地分析、路線設計 | 產品經理、隊長 |
| **[TECHNICAL_REFERENCE.md](./TECHNICAL_REFERENCE.md)** | 系統規格、硬體配置、通訊協定 | 工程師 |
| **[EVOLUTION.md](./EVOLUTION.md)** | 版本演進、技術決策、經驗總結 | 技術 Lead、後續維護者 |

### 版本文檔

| 版本 | 主文檔 | 位置 |
|------|--------|------|
| v2 | [v2_autonomous/DESIGN.md](./v2_autonomous/DESIGN.md) | 架構設計、PID 參數 |
| v3 | [v3_stable/docs/](./v3_stable/docs/) | 5 份期末軟工文件 |
| v4 | [v4_simple/README.md](./v4_simple/README.md) | 簡化版說明 |

---

## 八、期末成績

## 期末競賽成績

| 項目 | 得分 | 說明 |
|------|------|------|
| **總分** | **113 / 120** | **94.2% 達成** |
| **時間分** | 滿分 | 完成時間符合要求 |
| **清掃分** | 良好 | 吸力足夠，紙屑回收率高 |
| **扣分項目** | -7 | 撞牆 ×1 (-1分)、調整 ×1 (-2分)、其他 (-4分) |

### 成績分析

**達成項目：**
- ✅ 時間管理完美：在 2:30 內完成繞場一圈
- ✅ 吸塵表現良好：主吸嘴設計有效，吸力充足
- ✅ 基本迴避功能：紅色區域未進入

**改進空間：**
- 🔧 吸嘴尺寸優化：可擴大有效吸取範圍
- 🔧 清掃路徑策略：四角落清掃邏輯可更精細
- 🔧 避碰能力：減少不必要的撞牆

---

## 九、後續工作

### 短期（v5 競賽版）
- [x] 基礎版本驗證完成
- [x] 時間控制達成
- [ ] 進一步優化吸嘴設計
- [ ] 角落清掃策略精細化

### 中期
- [ ] 卡爾曼濾波（IMU + 超聲波融合）
- [ ] 地圖記錄與自適應優化
- [ ] 故障檢測與自動降級

### 長期
- [ ] 完整 SLAM 實裝
- [ ] 多車協調
- [ ] 機器學習路徑優化

---

## 十、常見問題

### Q: 應該用 v3 還是 v4？
**A:** 正式競賽用 **v3_stable**（更穩定）。v4 適合快速調試或故障排查。

### Q: 怎樣啟用 IMU？
**A:** v3 中 IMU 為可選。需要時檢查 `config.h` 中的 `#define IMU_ENABLED`。

### Q: 紅色偵測怎麼運作？
**A:** 見 [FINAL_AUTONOMOUS_PLAN.md §5.5](./FINAL_AUTONOMOUS_PLAN.md)（控制邏輯）和 [TECHNICAL_REFERENCE.md §5.3](./TECHNICAL_REFERENCE.md)（實裝細節）。

### Q: 超聲波資料異常？
**A:** 檢查：
1. 硬體接線（D7/D8/A1/A2）
2. 通訊同步（test_connection.py）
3. config.h 中的腳位定義

詳細排查見：[TECHNICAL_REFERENCE.md §7.3](./TECHNICAL_REFERENCE.md)

---

## 十一、開發環境

### Arduino IDE 設置

```
Board:        Arduino Uno
Processor:    ATmega328P
Speed:        115200 bps
```

### Python 環境（Raspberry Pi）

```bash
Python 3.8+
opencv-python
numpy
pyserial
```

### 推薦編輯器

- **Arduino：** Arduino IDE 或 VS Code + PlatformIO
- **Python：** VS Code + Python 擴充
- **文檔：** Markdown 預覽器

---

## 十二、關鍵聯絡資訊

**技術文檔：**
- 硬體問題 → [TECHNICAL_REFERENCE.md §2](./TECHNICAL_REFERENCE.md)
- 通訊協定 → [TECHNICAL_REFERENCE.md §4](./TECHNICAL_REFERENCE.md)
- 除錯指南 → [EVOLUTION.md §5](./EVOLUTION.md)

**版本決策：**
- 架構對比 → [EVOLUTION.md §3-4](./EVOLUTION.md)
- 參數調優 → [EVOLUTION.md §6](./EVOLUTION.md)

**競賽準備：**
- 場地分析 → [FINAL_AUTONOMOUS_PLAN.md §2](./FINAL_AUTONOMOUS_PLAN.md)
- 路線設計 → [FINAL_AUTONOMOUS_PLAN.md §4](./FINAL_AUTONOMOUS_PLAN.md)
- 時間預算 → [FINAL_AUTONOMOUS_PLAN.md §4.2](./FINAL_AUTONOMOUS_PLAN.md)

---

## 更新日誌

| 日期 | 版本 | 更新內容 |
|------|------|---------|
| 2025-12-13 | 1.0 | 初版期末導覽，整合 v2/v3/v4 |
| 2025-12-14 | 1.1 | 補充期末成績、競賽成果總結 |

---

**專案名稱：** 機電小車自走清掃系統
**團隊：** NTU BIME 2025 機電整合四 Group 10
**最後更新：** 2025-12-14
