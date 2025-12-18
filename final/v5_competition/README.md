# v5_competition - 期末競賽實戰版

**版本：** 5.0
**狀態：** 🏆 競賽驗證
**成績：** 113 / 120 (94.2%)

---

## 一、版本特色

v5 是期末競賽的最終實戰版本，**延續 v4 簡化架構，參考 v3 調參經驗**：

| 特性 | v3_stable | v5_competition |
|------|-----------|----------------|
| 架構 | Arduino 自主 + Pi 監控 | Arduino 獨立運作 |
| 超聲波 | 前 + 右前 + 右後 | 前 + 右前 + 右後 |
| IMU | MPU6050 航向鎖定 | ❌ 移除（簡化） |
| 紅色偵測 | Pi 背景執行緒 | Pi 獨立程式（備用） |
| 入口策略 | 無 | ✅ 固定入口動作 |
| 角落策略 | 擺頭清掃 | 擺頭 + 倒退 + 直走 |

### 簡化決策

競賽現場驗證後，決定移除 IMU 依賴：
- 減少感測器故障風險
- 簡化程式邏輯
- PID 沿牆控制已足夠穩定

---

## 二、競賽成績

| 項目 | 得分 | 說明 |
|------|------|------|
| **總分** | **113 / 120** | 94.2% |
| 時間 | 滿分 | 完成時間符合要求 |
| 清掃 | 良好 | 吸力足夠 |
| 扣分 | -3 | 撞牆×1 (-1)、調整×1 (-2) |

### 改進方向
- 吸嘴尺寸優化（增加覆蓋範圍）
- 清掃路徑策略改進（減少遺漏區域）

---

## 三、程式結構

```
v5_competition/
├── arduino/
│   └── main/
│       └── main.ino      # Arduino 主程式（競賽版）
├── raspberry_pi/
│   └── red_detector.py   # 紅色偵測（備用，未實際使用）
└── README.md             # 本文件
```

---

## 四、核心邏輯

### 4.1 入口動作（只執行一次）

```
直走 2000ms → 右轉 950ms → 直走 1200ms
```

針對競賽場地入口設計，確保順利進入清掃區域。

### 4.2 沿牆 PID 控制

```cpp
error = dist_RF - dist_RB;  // 右前 - 右後
P = error * Kp;             // Kp = 10.0
D = (error - last_error) * Kd;  // Kd = 2.0
output = P + D;

speed_L = BASE_SPEED_L + output;  // 64 + output
speed_R = BASE_SPEED_R - output;  // 68 - output
```

### 4.3 角落處理（turnLeftCombo）

```
右擺頭 1100ms → 倒退回來 1300ms → 倒退 1000ms → 直走 1000ms → 左轉 1100ms
```

確保角落區域清掃覆蓋。

---

## 五、參數配置

| 參數 | 值 | 說明 |
|------|-----|------|
| BASE_SPEED_L | 64 | 左輪基準速度 |
| BASE_SPEED_R | 68 | 右輪基準速度（補償偏差） |
| Kp | 10.0 | 比例增益 |
| Kd | 2.0 | 微分增益 |
| TURN_THRESHOLD | 23 cm | 前方轉彎門檻 |
| WALL_MISSING_DIST | 30 cm | 右牆消失判定 |

---

## 六、硬體接線

### 馬達 (L298N)
| 功能 | Arduino Pin |
|------|-------------|
| 左輪 PWM | 3 |
| 左輪 IN1/IN2 | 6 / 5 |
| 右輪 PWM | 11 |
| 右輪 IN1/IN2 | 10 / 9 |

### 超聲波
| 感測器 | TRIG | ECHO |
|--------|------|------|
| 前方 | 7 | 8 |
| 右前 | A1 | A2 |
| 右後 | 2 | 4 |

### 繼電器（吸塵器）
| 功能 | Arduino Pin |
|------|-------------|
| RELAY | A3 |

---

## 七、與其他版本的關係

```
v4_simple (簡化版)  ──────────────┐
    │                            │ 主要延續
    │ 純 Arduino 架構            │
    │                            ▼
    │                    v5_competition (競賽版) ← 你在這裡
    │                            │
v3_stable ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─┘
    (調參經驗參考)               │
                                 ├── 延續 v4 簡化架構
                                 ├── 參考 v3 調參經驗
                                 └── 新增入口動作 + 角落策略
```

→ 詳細演進歷史：[EVOLUTION.md](../EVOLUTION.md)

---

## 八、使用說明

### Arduino 上傳
```bash
cd final/v5_competition/arduino/main
# 使用 Arduino IDE 開啟 main.ino 並上傳
```

### 紅色偵測（備用）
```bash
cd final/v5_competition/raspberry_pi
python3 red_detector.py
```

---

**競賽日期：** 2025-12-14
**團隊：** NTU BIME 2025 機電整合四 Group 10
