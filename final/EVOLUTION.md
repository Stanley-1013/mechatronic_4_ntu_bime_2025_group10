# 版本演進史

**專案名稱：** 機電小車自走清掃系統
**團隊：** NTU BIME 2025 機電整合四 Group 10
**文檔版本：** 1.1
**最後更新：** 2025-12-14

---

## 目錄

1. [演進總覽](#1-演進總覽)
2. [期中階段 (v1.x)](#2-期中階段-v1x)
3. [期末階段 (v2~v4)](#3-期末階段-v2v4)
4. [關鍵技術決策](#4-關鍵技術決策)
5. [問題解決歷程](#5-問題解決歷程)
6. [參數演進](#6-參數演進)
7. [經驗總結](#7-經驗總結)

---

## 1. 演進總覽

### 時間線

```
2025-10-31  ┬─ v1.0  遙控模式啟動 (期中準備)
            │   ├─ L298N 馬達驅動
            │   ├─ 雙超聲波 (左/右側)
            │   ├─ GPIO UART 通訊 (57600 bps)
            │   └─ USB 2.4G 遙控器整合
            │
2025-11-14  ├─ v1.0.1  期中測試成功
            │   ├─ 遙控模式驗證完成
            │   ├─ Serial timeout 修復
            │   └─ 降低通訊頻率至 20Hz
            │
2025-11-28  ├─ v1.1  期末自走準備
            │   ├─ 新增 autonomous_main.py
            │   ├─ 超聲波改為 前方/右側
            │   ├─ 通訊升級至 USB Serial 115200 bps
            │   └─ 新增 wall_follower.py
            │
2025-11-29  ├─ v2.0  MPU6050 整合 (v2 架構開始)
            │   ├─ IMU Yaw 積分算法
            │   ├─ 感測器封包擴展至 12 bytes
            │   ├─ 紅色偵測背景執行緒
            │   └─ 更新 ICD 至 v2.0
            │
2025-12-03  ├─ v1.8 (實驗分支)
~ 12-04     ├─ v1.9  右輪腳位對調修正
            ├─ v2: 重構 Arduino 為 IDE 專案
            │
2025-12-05  ├─ v2.2 (v2 穩定化)
~ 12-07     ├─ IMU 航向 PD 控制完整實現
            ├─ Phase 1: 航向 PID
            ├─ Phase 2: 超聲波監督修正
            ├─ Phase 3: 轉彎重置航向
            │
2025-12-10  ├─ v3.x (Arduino 自主控制版)
~ 12-13     ├─ v3.0: 架構切換，棄用 Pi 控制
            ├─ v3: 雙超聲波沿牆控制
            ├─ v3.12-3.14: PD 參數優化
            ├─ 左右輪獨立 PWM
            └─ 角落擺頭、無牆補償
            │
2025-12-14  ├─ v5_competition (競賽實戰)
            ├─ 基於 v3_stable 驗證
            ├─ 期末成績：113 / 120 (94.2%)
            └─ 時間滿分、吸力良好
```

---

## 2. 期中階段 (v1.x)

### 2.1 v1.0 - 遙控模式基礎 (2025-10-31)

**定位：** 期中專案啟動，確保基礎控制可運行

**核心功能：**
- ✅ 2.4G USB 遙控器整合（無線控制）
- ✅ 差動驅動算法（前進/後退/轉向）
- ✅ L298N 馬達 PWM 控制（±255 速度範圍）
- ✅ 吸塵器繼電器控制（開/關）
- ✅ 二進位 Serial 協定（8-byte 封包）
- ✅ GPIO UART 通訊（57600 bps）

**硬體配置：**
- 主控：Raspberry Pi 4 (Python)
- 微控制器：Arduino Uno (C++)
- 馬達驅動：L298N H-Bridge
- 超聲波：HC-SR04 × 2（左側 + 右側，防撞用）
- 通訊：GPIO UART (SoftwareSerial)

**通訊協定 (v1.0)：**
```
Pi → Arduino (8 bytes):
[0xAA] [L_PWM_L] [L_PWM_H] [R_PWM_L] [R_PWM_H] [FLAGS] [CHK] [0x55]

Arduino → Pi (8 bytes):
感測器資料（未使用）
```

詳見 [TECHNICAL_REFERENCE.md §4.2-4.3](TECHNICAL_REFERENCE.md)

---

### 2.2 v1.0.1 - 期中測試修復 (2025-11-14)

**狀態：** ✅ 期中評測通過

**測試成果：**
- 遙控操控 5/5 ✅
- 馬達控制 5/5 ✅
- 吸塵器功能 5/5 ✅
- 長時間運行穩定性 5/5 ✅

**主要修復：**

1. **Serial Timeout 問題** → 降低控制頻率
   - 問題：運行 7 秒後程式卡死 (`Write timeout`)
   - 原因：9600 bps 頻寬不足（Arduino DEBUG 輸出佔頻寬）
   - 解決：
     - 控制頻率 50Hz → 20Hz
     - 加入 write timeout (0.1 秒)
     - 關閉 Arduino DEBUG 輸出
   - 詳見 [TECHNICAL_REFERENCE.md §7.3](#serial-timeout-問題-v101)

2. **超聲波阻塞問題** → 暫時停用
   - 現象：pulseIn() 阻塞 20-30ms，導致 timeout
   - 臨時方案：停用超聲波讀取
   - 后续改進方向：非阻塞式讀取 (Timer-based)

3. **硬體腳位驗證** → 完成接線確認
   - 逐一測試 Arduino 各腳位輸出
   - 馬達控制方向確認

詳見 [MIDTERM_REPORT.md §3](../midterm/MIDTERM_REPORT.md)

---

### 2.3 v1.1 - 期末自走準備 (2025-11-28)

**目標：** 為自走模式轉向做準備，切換為 Pi 主導的自走架構

**架構變更：**
```
v1.0 (遙控模式)           v1.1 (自走準備)
Joystick Input    →       自走策略 (wall_follower)
    ↓                           ↓
RPi (高階)        →       RPi (決策層)
    ↓                           ↓
Arduino (低階)    →       Arduino (執行層)
    ↓                           ↓
Motors/Sensors    →       Motors/Sensors
```

**新增功能：**
1. **autonomous_main.py** - 自走模式入口
   - 沿右牆清掃策略
   - 紅色區域迴避邏輯

2. **wall_follower.py** - 沿牆控制器
   - 目標距離：18cm
   - Bang-Bang 控制（前期）

3. **紅色偵測** - 視覺迴避
   - Pi Camera HSV 色彩空間識別
   - 關閉吸塵器避開紅色區域

4. **通訊升級：GPIO UART → USB Serial**
   - 更可靠的連接
   - 速率提升至 115200 bps
   - 通訊協定版本 → v1.1

**感測器配置變更：**
| 項目 | v1.0 | v1.1 |
|-----|------|------|
| 超聲波位置 | 左側 + 右側 | **前方 + 右側** |
| 超聲波用途 | 防撞 | 前方避障 + 沿牆距離 |
| 超聲波更新 | 停用 | 交替讀取（50ms） |

詳見 [TECHNICAL_REFERENCE.md §2.4, §7.2](TECHNICAL_REFERENCE.md)

---

## 3. 期末階段 (v2~v4)

### 3.1 v2 架構 - Pi 主導自走 + IMU 整合

#### v2.0 - MPU6050 整合 (2025-11-29)

**定位：** 首次整合 IMU，建立三層控制架構

**架構圖：**
```
┌─────────────────────────────────┐
│    Raspberry Pi 4 (Python)      │
│  • 紅色偵測（背景執行緒）        │
│  • Wall Follower 決策           │
│  • 感測器融合                   │
└──────────┬──────────────────────┘
           │ USB Serial 115200 bps
┌──────────▼──────────────────────┐
│    Arduino Uno (C++)            │
│  • 馬達 PWM 控制                │
│  • 超聲波讀取 (交替)             │
│  • IMU 資料讀取 (Yaw 積分)      │
│  • 感測器資料封包 (12 bytes)    │
└──────────┬──────────────────────┘
           │
    ┌──────┼──────┬─────────┐
    ▼      ▼      ▼         ▼
  Motors  US_F   US_R      IMU
```

**新增硬體：**
- MPU6050 6 軸 IMU (I2C @0x68)
- 實時 Yaw 角度（陀螺儀積分）
- 漂移率 ~0.77°/分鐘（實測）

**通訊協定升級 (v2.0)：**

Pi → Arduino（8 bytes，無變化）：
```
[0xAA] [L_L] [L_H] [R_L] [R_H] [FLAGS] [CHK] [0x55]
```

Arduino → Pi（12 bytes，**新增 IMU 資料**）：
```
[0xBB] [FRONT_H] [FRONT_L] [RIGHT_H] [RIGHT_L]
       [YAW_H] [YAW_L] [GYROZ] [STATUS] [_] [CHK] [0x66]

其中：
- FRONT/RIGHT：距離 (cm)
- YAW：角度 × 10 (0.1° 解析度)
- GYROZ：角速度 (°/s)
- STATUS：感測器有效性 bit flag
```

詳見 [TECHNICAL_REFERENCE.md §4.3](TECHNICAL_REFERENCE.md)

**新增功能：**
1. **紅色偵測改為背景執行緒**
   - 避免阻塞主控制迴圈
   - 即時 HSV 色彩識別

2. **imu_processor.py**
   - Yaw 積分補償
   - 漂移檢測與修正

3. **感測器融合**
   - 優先級：紅色 > 前方 > 右側 > IMU
   - 詳見 [TECHNICAL_REFERENCE.md §6.3](TECHNICAL_REFERENCE.md)

**參數設置：**
```cpp
// IMU 配置 (config.h)
#define IMU_I2C_ADDR    0x68
#define IMU_UPDATE_HZ   50      // 20ms 更新
#define YAW_DEADZONE    1.0     // 1 度死區
```

詳見 [TECHNICAL_REFERENCE.md §2.5](TECHNICAL_REFERENCE.md)

---

#### v2.1-v2.2 - 航向 PID 控制優化 (2025-12-05 ~ 12-07)

**定位：** 從 Bang-Bang 升級至 PID 控制，提升直線穩定性

**三層控制架構（最終版）：**
```
優先級 1（前方避障）：
  front_dist < 20cm → 左轉 (weight=1.0)

優先級 2（IMU 航向 PID）：
  yaw_error × Kp + gyroZ × Kd → 角速度修正 (持續)

優先級 3（超聲波監督）：
  偏離目標距離 > 2 秒 → 慢速修正 IMU 漂移
```

**Phase 1: IMU 航向 PID 控制**
```cpp
// 鎖定初始航向
if (!_yawLocked && imuValid) {
    _targetYaw = yaw;
    _yawLocked = true;
}

// PID 輸出
float yawError = _normalizeAngle(yaw - _targetYaw);
float yawCorrection = g_yawKp * yawError;
angular += yawCorrection * (1.0 - obstacleWeight);
```

**Phase 2: 超聲波監督修正**
```cpp
// 防止 IMU 長期漂移
if (!isTurning && _driftTimer > 2.0) {
    if (right > TARGET + DEADZONE) {
        _targetYaw -= YAW_DRIFT_RATE * dt;
    } else if (right < TARGET - DEADZONE) {
        _targetYaw += YAW_DRIFT_RATE * dt;
    }
}
```

**Phase 3: 轉彎重置航向**
```cpp
// 記錄轉彎開始
if (front < FRONT_STOP && !_cornerLocked) {
    _cornerLocked = true;
    _lastTurnYaw = yaw;
}

// 轉彎完成後重置
if (turnedAngle > 70.0) {
    _targetYaw = yaw;  // 重新鎖定
}
```

**PID 參數演進：**
| 版本 | Kp | Ki | Kd | YAW_DRIFT | 狀態 |
|-----|----|----|----|-----------|----|
| v2.1-dev | 0.5 | 0.0 | 0.1 | 0.5°/s | 過快，震盪 |
| v2.1 final | 0.35 | 0.0 | 0.08 | 0.15°/s | ✅ 平衡 |

詳見 [final/v2_autonomous/DESIGN.md §3.1-3.3](final/v2_autonomous/DESIGN.md)

---

### 3.2 v3 架構 - Arduino 自主控制版

#### v3.0 - 架構切換 (2025-12-10)

**定位：** **關鍵決策** - 棄用 Pi 控制，改為 Arduino 自主決策

**為何切換？**
- v2 依賴 Pi 即時決策，整體延遲較大
- Arduino 直接控制反應更快
- 減少 Serial 通訊可靠性問題
- 簡化系統複雜度

**架構對比：**

```
v2 (Pi 主導)                v3 (Arduino 自主)
─────────────────────────────────────────
RPi (決策層)                RPi (監測層/備援)
  ↓ 命令                      ↓ 監測資料
Arduino (執行層)  →  Arduino (決策 + 執行)
  ↓                           ↓
Motors / Sensors              Motors / Sensors
```

**v3.0 初始配置：**
- **双超聲波沿牆**：前方 + 右側（同 v2）
- **距離 + 角度雙控制**：改 Arduino 實現
- **雙超聲波獨立讀取**：避免回波干擾

詳見 git log:
```
ffeb7cb v3: 實作雙超音波沿牆控制 - 距離+角度雙控制
2ce5ddd v3: 新增右側雙超音波支援
```

---

#### v3.x - 參數調優 (2025-12-10 ~ 12-13)

**演進路線：**
```
v3.0 (基礎雙超聲波)
    ↓
v3: 改為雙超聲波獨立讀取
    ↓
v3: 實作 PD 控制
    ↓
v3.12: 簡化轉彎邏輯，強化 PD 參數
    ↓
v3.14: 簡化擺頭流程，無牆右偏補償
```

**關鍵改進：**

1. **左右輪獨立 PWM** (v3)
   - 不再依賴差動驅動公式
   - 直接輸出左右輪速度
   - 更靈活的馬達配平

   ```cpp
   // v3 馬達控制
   setMotor(LEFT_MOTOR, left_speed);
   setMotor(RIGHT_MOTOR, right_speed);
   ```

2. **PD 控制 (v3.x)**
   - 替代原 Bang-Bang 控制
   - 距離誤差 → 平滑修正
   - 參數：
     ```
     Kp = 2.5  (距離比例增益)
     Kd = 0.5  (微分增益，平滑輸出)
     ```

3. **角落擺頭 (v3.14)**
   - 在牆角停留並左右擺頭清掃
   - 偵測標記：front < 15cm && right < 12cm
   - 擺頭周期：1 秒

4. **無牆右偏補償 (v3.14)**
   - 當右側無牆時，主動向右偏轉
   - 自動尋找牆壁
   - 修正漂移問題

**參數對比：**

| 項目 | v3.0-3.11 | v3.12 | v3.14 |
|-----|-----------|-------|-------|
| 前方停止距離 | 20cm | 18cm | 18cm |
| 右側目標距離 | 18cm | 15cm | **12-18cm** |
| Kp | 1.5 | 2.5 | 2.5 |
| Kd | 0.3 | 0.5 | 0.5 |
| 擺頭 | 否 | 否 | ✅ 是 |
| 無牆補償 | 否 | 否 | ✅ 是 |

詳見 git log:
```
4049d7a v3.14: 簡化擺頭流程 + 無牆右偏補償
8ce28fd v3.14: 新增角落擺頭清掃 + 參數微調
d5ed829 v3.12: 簡化轉彎邏輯 + 加強 PD 參數
c75118b v3.10 + v4_simple: 沿牆改加減法 PD + 新增簡化版
```

---

### 3.3 v4 架構 - 簡化實驗版

#### v4_simple - 對比測試 (2025-12-10)

**定位：** 參考別組實測程式碼，驗證簡化策略的可行性

**核心特色：**
1. **超簡單 PD 沿牆**
   ```
   error = rightFront - rightRear
   ```
   - 使用雙超聲波的**前後差值**替代單點控制
   - 更穩健的直線維持

2. **防突波濾波**
   ```cpp
   if (abs(error_delta) > 8) {
       error = error_prev;  // 濾除突波
   }
   ```
   - 限制誤差變化，平滑控制

3. **出場判斷**
   ```cpp
   if (rightFront > 30cm) {
       // 輕微右轉找牆
   }
   ```

4. **紅色偵測 (Pi)**
   - Pi Camera 背景執行緒
   - 偵測到紅色 → 發送停車命令

**參數設置：**
```
BASE_SPEED_L:    64
BASE_SPEED_R:    77
Kp:              10.0
Kd:              2.0
轉彎時間:        1200ms
```

詳見 [final/v4_simple/README.md](final/v4_simple/README.md)

**v4 vs v3 對比：**
| 項目 | v3 (IMU + 前後超聲波) | v4 (簡化雙超聲波) |
|-----|------------------|----------------|
| 硬體複雜度 | ⭐⭐⭐ (需 IMU) | ⭐⭐ (純超聲波) |
| 直線穩定性 | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| 轉彎精度 | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| 程式碼複雜度 | 高 | 低 |
| 目標應用 | 精密清掃 | 快速驗證 |

---

### 3.4 v5 競賽版 (2025-12-14)

**定位：** 實戰驗證版本，基於 v3_stable 競賽部署

**期末成績：**

| 項目 | 得分 | 說明 |
|------|------|------|
| **總分** | **113 / 120** | **94.2% 達成** |
| **時間分** | **滿分** | 2:30 內完成 |
| **清掃分** | **良好** | 吸力足夠 |
| **扣分明細** | -7 | 撞牆 ×1 (-1)、調整 ×1 (-2)、其他 (-4) |

**成績解讀：**
- ✅ **時間管理**：完美達成 < 2:30 時間要求
- ✅ **吸塵效能**：主吸嘴設計有效，吸力足夠
- ✅ **基本避碰**：成功迴避紅色禁區
- 🔧 **改進空間**：
  - 吸嘴尺寸可進一步擴大
  - 四角落清掃策略可更精細
  - 避碰邏輯可加強穩健性

**使用版本：** v3_stable (v3.14 參數)

**驗證檢查項：**
- ✅ 馬達控制穩定
- ✅ 超聲波感測正常
- ✅ IMU 輔助導航（可選）
- ✅ 吸塵器工作正常
- ✅ 紅色迴避邏輯生效

---

## 4. 關鍵技術決策

### 4.1 通訊方式演變

| 階段 | 方式 | 鮑率 | 協定版本 | 原因 |
|-----|------|------|---------|------|
| v1.0 | GPIO UART | 57600 | v1.0 | SoftwareSerial，實驗性 |
| v1.0.1 | GPIO UART | 9600 | v1.0 | 降低頻率修復 timeout |
| **v1.1+** | **USB Serial** | **115200** | **v1.1** | 更穩定，免接線 |
| v2.0+ | USB Serial | 115200 | v2.0 | 擴展為 12-byte 傳 IMU |

**決策理由：**
- GPIO UART 頻寬受限，容易 timeout
- USB Serial 內建緩衝與校驗
- 升級至 115200 bps 為後續擴展預留空間

詳見 [TECHNICAL_REFERENCE.md §4.1](TECHNICAL_REFERENCE.md)

---

### 4.2 超聲波配置

| 階段 | 位置 | 更新率 | 讀取方式 | 原因 |
|-----|------|--------|---------|------|
| v1.0 | 左側 + 右側 | 停用 | - | 防撞考量，但阻塞 Serial |
| v1.1 | **前方 + 右側** | 交替 10Hz | 50ms 交替 | 配合沿牆策略 |
| v3+ | 前方 + 右側 | 獨立 10Hz | 雙感測器獨立 | 避免回波干擾 |

**改為前方 + 右側的核心理由：**
1. 前方：偵測牆壁，觸發轉彎
2. 右側：監督牆距，實現沿牆
3. **刪除左側：** 沿右牆不需要左側感應

詳見 [TECHNICAL_REFERENCE.md §2.4, §7.2](TECHNICAL_REFERENCE.md)

---

### 4.3 控制層次變遷

**演變軌跡：**

```
v1.0~v1.1
遙控器 → Pi (搖桿映射) → Arduino (PWM) → 馬達

v2.0~v2.2 (三層架構)
感測器 → Pi (決策) → Arduino (執行) → 馬達

v3+ (自主化)
感測器 → Arduino (決策 + 執行) → 馬達
  ↑
 Pi (監測層，可選)
```

**決策對比：**

| 方案 | 延遲 | 可靠性 | 程式碼 | 擴展性 |
|-----|------|--------|---------|---------|
| v1 (遙控) | 低 | 高 | 簡 | 低 |
| v2 (Pi決策) | 中 | 中 | 中 | 高 |
| v3 (Arduino決策) | 低 | 高 | 高 | 中 |

**選擇 v3 理由：**
- 自走模式不需遙控延遲容限
- Arduino 反應時間 < 20ms
- Pi Serial 通訊可能引入不穩定因素
- 簡化整體架構

---

## 5. 問題解決歷程

### 5.1 Serial Timeout 問題 (v1.0.1)

**現象：** 運行 7 秒後程式卡死，顯示 `Write timeout`

**根本原因分析：**
1. Arduino 使用同一條 Serial 進行通訊和 DEBUG 輸出
2. 9600 bps 理論頻寬：960 bytes/sec
3. Pi 控制迴圈：50Hz × 8 bytes = 400 bytes/sec
4. Arduino DEBUG：~500 bytes/sec
5. **總計：900 bytes/sec > 可用頻寬** → 發送緩衝滿 → timeout

**解決方案：**
```python
# 方案 A: 降低控制頻率
CONTROL_LOOP_FREQUENCY = 20  # Hz (從 50Hz)
CONTROL_LOOP_PERIOD = 50     # ms

# 方案 B: 加入 write timeout
serial_port = serial.Serial(
    port=SERIAL_PORT,
    baudrate=9600,
    timeout=0.1,
    write_timeout=0.1
)
```

```cpp
// 方案 C: 關閉 DEBUG 輸出
// #define DEBUG_SERIAL_ENABLED
// #define DEBUG_SHOW_COMMANDS

// 方案 D: 緩衝區保護
int packetsProcessed = 0;
while (Serial.available() && packetsProcessed < 3) {
    // 處理封包
    packetsProcessed++;
}
```

**成果：** 系統可長時間穩定運行 ✅

詳見 [TECHNICAL_REFERENCE.md §7.3](TECHNICAL_REFERENCE.md)

---

### 5.2 超聲波阻塞問題 (v1.0~v1.1)

**現象：** 加入超聲波讀取後，系統仍會 timeout

**阻塞分析：**
```cpp
void updateSensors() {
    // pulseIn() 可能阻塞 20-30ms
    leftDistance = leftUltrasonic.getDistance();
    delay(10);  // ← 額外 10ms
    rightDistance = rightUltrasonic.getDistance();
    // 總計：30-40ms 無法處理 Serial
}

// 控制迴圈無法執行
void loop() {
    updateSensors();  // ← 每 100ms 卡 30-40ms
    processSerial();  // ← 無法及時執行
}
```

**解決方案演進：**

1. **v1.0.1：暫時停用** (權宜之計)
   ```cpp
   /*
   if (currentTime - lastSensorTime >= SENSOR_UPDATE_INTERVAL) {
       updateSensors();
   }
   */
   ```

2. **v1.1：交替讀取** (部分改善)
   ```cpp
   // 每 50ms 只讀一個感測器
   if (readFront) {
       frontDistance = frontUltrasonic.getDistance();
   } else {
       rightDistance = rightUltrasonic.getDistance();
   }
   readFront = !readFront;
   ```

3. **v3+：獨立讀取迴圈** (完整改善)
   - Arduino 內部時序管理
   - 確保每個感測器讀取不阻塞主迴圈

**經驗：** 阻塞操作（delay, pulseIn）是嵌入式系統大忌

詳見 [MIDTERM_REPORT.md §3.1](../midterm/MIDTERM_REPORT.md)

---

### 5.3 IMU 資料同步問題 (v2.0)

**現象：** Pi 端收到的 IMU 資料持續無效 (status bit = 0)

**原因：** Arduino 開機時串列輸出的啟動訊息干擾了封包同步
```
啟動訊息: "Arduino Initialized\r\n"
        ↓
破壞了首個感測器封包的對齊
        ↓
Pi 永遠無法正確解析後續封包
```

**解決方案：** 顯式同步機制
```python
def _sync_to_packet(self):
    """同步到有效封包起始"""
    while True:
        byte = self.serial.read(1)
        if byte == b'\xbb':  # 尋找 header
            remaining = self.serial.read(11)
            if len(remaining) == 11 and remaining[-1] == 0x66:  # 驗證 footer
                return b'\xbb' + remaining
        # 繼續搜尋直到找到有效封包
```

**成果：** IMU 資料成功整合 ✅

詳見 [TECHNICAL_REFERENCE.md §7.3](TECHNICAL_REFERENCE.md)

---

### 5.4 馬達特性差異問題 (v3.x)

**現象：** 同樣 PWM 值，左右輪速度不一，導致 C 字軌跡

**原因分析：**
- 左輪較右輪快約 5-10%
- TT 馬達個體差異大
- L298N 供電不均勻

**解決方案：** 馬達補償係數
```cpp
// 在 config.h 中
#define RIGHT_MOTOR_SCALE    0.95f

// 在馬達控制時
void setMotorSpeed(Motor motor, int16_t speed) {
    if (motor == RIGHT) {
        speed = (int16_t)(speed * RIGHT_MOTOR_SCALE);
    }
    // 輸出 PWM
}
```

**成果：** 左右輪速度平衡 ✅

詳見 git log: `f34cddb v3: 左右輪獨立 PWM + 右前保護公式調整`

---

### 5.5 IMU 長期漂移問題 (v2.2)

**現象：** 連續行駛 2-3 分鐘後，累積轉角誤差達 10-20°

**漂移來源：**
- 陀螺儀零偏漂移：~0.77°/分鐘（實測）
- 積分累積誤差

**解決方案 (v2.2 Phase 2)：** 超聲波監督修正
```cpp
// 當右側距離持續偏離 > 2 秒時
if (_driftTimer > 2.0) {
    if (right > TARGET + DEADZONE) {
        _targetYaw -= YAW_DRIFT_RATE * dt;  // 0.15°/s
    } else if (right < TARGET - DEADZONE) {
        _targetYaw += YAW_DRIFT_RATE * dt;
    }
}
```

**驗收標準：** 5 分鐘連續運行，右側距離保持 15±5cm ✅

詳見 [final/v2_autonomous/DESIGN.md §3.2](final/v2_autonomous/DESIGN.md)

---

## 6. 參數演進

### 6.1 控制迴圈頻率

| 版本 | 控制頻率 | 原因 | 穩定性 |
|-----|---------|------|--------|
| v1.0 | 50 Hz | 初始設計 | ❌ timeout |
| v1.0.1 | **20 Hz** | 修復 Serial | ✅ 穩定 |
| v1.1+ | 20 Hz | 兼容 |  ✅ 穩定 |
| v2.0+ | 20 Hz | Pi 決策延遲 | ✅ 穩定 |
| v3+ | **20 Hz** (Arduino) | Arduino 自主 | ✅ 很穩定 |

---

### 6.2 沿牆控制參數

#### 距離目標值演變
```
v1.1: 目標 18cm (距離 ± 5cm)
  ↓
v2.0: 目標 18cm (改用 PID)
  ↓
v3.12: 目標 15cm (縮小距離，更貼牆)
  ↓
v3.14: 目標 12-18cm (動態調整)
```

#### PD 控制參數
| 版本 | Kp | Kd | 說明 |
|-----|----|----|------|
| v2.0 (Bang-Bang) | - | - | 不適用 |
| v2.1 | 0.5 | 0.1 | 初版 (過快) |
| v2.2 final | 0.35 | 0.08 | 平衡版 |
| v3.0-3.11 | 2.5 | 0.5 | Arduino 距離 PD |
| v3.12+ | 2.5 | 0.5 | 保持 |

**參數調優經驗：**
- Kp 過大：震盪增加
- Kp 過小：反應遲鈍
- Kd 用於平滑微分項，抑制高頻振盪
- 實測最佳平衡點：Kp=0.35, Kd=0.08 (IMU) 或 Kp=2.5, Kd=0.5 (距離)

---

### 6.3 前方轉彎參數

| 版本 | STOP_DIST | SLOW_DIST | 轉彎角度 | 說明 |
|-----|-----------|-----------|---------|------|
| v1.1 | 20 cm | 40 cm | 自動 | 基於超聲波 |
| v2.0 | 20 cm | 40 cm | ~70° | 基於 IMU |
| v3.0-11 | 20 cm | 40 cm | ~70° | Arduino 自主 |
| v3.12 | **18 cm** | **40 cm** | ~70° | 更緊湊 |
| v3.14 | 18 cm | 40 cm | ~70° | 保持 |

---

### 6.4 吸塵器控制

| 版本 | 控制方式 | 觸發條件 |
|-----|---------|---------|
| v1.0+ | 繼電器開關 | A 按鈕 (遙控) |
| v1.1+ | 繼電器開關 | 自動（除非紅色檢測） |
| v2.0+ | 繼電器開關 | Pi 監控紅色偵測 |
| v3+ | 繼電器開關 | Arduino 自主 (v3 暫無紅色整合) |

---

## 7. 經驗總結

### 7.1 技術經驗

#### 1. 通訊設計
- **頻寬很重要：** 9600 bps 對嵌入式系統太低
  - 方案：升級至 115200 bps 或改用 SPI/I2C
  - 經驗：每秒傳輸量 ≈ 頻率 × 封包大小 × 安全係數 (0.7)

- **協定版本管理：** 預留擴展空間
  - v1.0 (8 bytes) → v2.0 (12 bytes) → 未來可再擴展
  - 採用 Header/Footer/Checksum 三重驗證

#### 2. 感測器整合
- **阻塞操作是大敵**
  - pulseIn() 可能阻塞 30ms，足以破壞即時系統
  - 解決方案：
    - 交替讀取（降低單次阻塞時間）
    - Timer-based 讀取（使用中斷計時）
    - 背景執行緒（Pi 端）

- **感測器融合優先級很關鍵**
  - 優先級 1：紅色偵測（安全考量）
  - 優先級 2：前方避障（立即威脅）
  - 優先級 3：側向修正（長期穩定）

#### 3. 控制策略
- **架構演進：** 集中式 → 自主分散
  - v1 (遙控)：簡單直接
  - v2 (Pi決策)：靈活但延遲
  - v3 (Arduino自主)：快速可靠

- **PID vs Bang-Bang**
  - Bang-Bang：邏輯簡單，但易震盪
  - PID：平滑控制，需要調參
  - 混合方案：前方避障用 Bang-Bang，距離控制用 PD

#### 4. 除錯技巧
- **同步機制重要**
  - Serial 封包同步：等待 Header → 讀完 Footer 驗證
  - 不要盲目相信 Serial.available()

- **參數隔離**
  - 集中管理 config.h，避免魔數散落
  - 提供獨立測試程式驗證單個模組

---

### 7.2 開發流程經驗

#### 1. 迭代週期
**最佳實踐：** 小步快走，定期測試

```
設計 → 實裝 → 測試 → 分析 → 優化 (循環)
1 天   1 天   1 天   0.5 天  0.5 天
```

**本專案實例：**
- v1.0 → v1.0.1：1 周迭代（修復 timeout）
- v1.1 → v2.0：1 周迭代（IMU 整合）
- v2.0 → v3：2-3 周迭代（架構切換）

#### 2. 版本管理
- **使用語義化版本**
  - vMAJOR.MINOR：主版本變更
  - 補充說明：功能改進 vs 問題修復 vs 架構變更

- **git commit 規範**
  ```
  v3.14: 簡化擺頭流程 + 無牆右偏補償
  v3.14: 新增角落擺頭清掃 + 參數微調
  ```

#### 3. 文檔維護
- **同步性很重要**
  - 代碼變更 → 同步更新 DESIGN.md / TECHNICAL_REFERENCE.md
  - 參數改動 → 更新 config.h 註釋

- **版本歷史表**
  - 記錄每個版本的關鍵改進
  - 便於後續追溯決策

---

### 7.3 項目管理經驗

#### 1. 時間規劃
```
期中 (v1.0~v1.0.1)：3 週
  └─ 遙控基礎、問題修復

期末 (v1.1~v3.14)：5 週
  ├─ v1.1: 自走準備
  ├─ v2: Pi 決策架構
  ├─ v3: Arduino 自主化
  └─ v3.x: 參數優化迭代
```

**經驗：** 控制系統需要大量調試時間，不能只看代碼行數

#### 2. 風險管理
- **備援方案很重要**
  - v2 (complex) vs v4 (simple) 並行開發
  - IMU 失效時降級至純超聲波
  - 遙控模式作為最終備援

- **測試環境準備**
  - 專注在實驗室環境測試（環境變數少）
  - 儘早發現硬體問題（接線錯誤等）

#### 3. 團隊協作
- **模組邊界清楚**
  - Pi 端 (Python)、Arduino 端 (C++) 可獨立開發
  - 通訊協定是關鍵依賴，需早期定案

- **集中測試**
  - 整合後的系統測試需在實車上進行
  - 不能純粹依賴模擬

---

### 7.4 未來改進方向

#### 短期 (v5 競賽版)
1. **入場進場邏輯**
   - 從外部進場 → 找到牆壁 → 開始沿牆

2. **完整紅色迴避**
   - v3 中整合 Pi Camera 檢測
   - 與 Arduino 自主控制協調

3. **參數更細緻調優**
   - 不同場地的動態參數調整
   - 溫度、電池電壓補償

#### 中期
1. **更優的感測融合**
   - 卡爾曼濾波（IMU + 超聲波）
   - 多重感測器加權決策

2. **學習與適應**
   - 記錄清掃軌跡
   - 下次執行時自動優化參數

3. **冗餘設計**
   - 多IMU、多超聲波的異常檢測
   - 故障後自動隔離有問題的傳感器

#### 長期
1. **完整的 SLAM**
   - 環境地圖構建
   - 最優路徑規劃

2. **多車協調**
   - 分散式清掃任務
   - 避免重複清掃

---

## 附錄：版本與檔案對應

| 版本 | 時間 | 主要檔案 | 狀態 |
|-----|------|---------|------|
| v1.0 | 2025-10-31 | `v1_remote/` | ✅ 期中基礎 |
| v1.0.1 | 2025-11-14 | `v1_remote/` | ✅ 期中通過 |
| v1.1 | 2025-11-28 | `v1_remote/` (autonomous_main.py 新增) | ✅ 自走準備 |
| v2.0 | 2025-11-29 | `final/v2_autonomous/` | ✅ IMU 整合 |
| v2.2 | 2025-12-07 | `final/v2_autonomous/` | ✅ PID 完整 |
| v3 | 2025-12-10+ | `v3_stable/` | ✅ Arduino 自主 |
| v4 | 2025-12-10+ | `final/v4_simple/` | ✅ 簡化對比 |
| v5 | 2025-12-14 | `final/v5_competition/` | ✅ 競賽驗證 (113/120) |

---

## 相關文檔

- **[TECHNICAL_REFERENCE.md](TECHNICAL_REFERENCE.md)** - 系統技術規格、硬體配置、通訊協定
- **[../midterm/MIDTERM_REPORT.md](../midterm/MIDTERM_REPORT.md)** - 期中成果總結、問題解決紀錄
- **[v2_autonomous/DESIGN.md](v2_autonomous/DESIGN.md)** - v2 架構詳細設計、PID 參數說明
- **[v4_simple/README.md](v4_simple/README.md)** - v4 簡化版説明

---

**文檔維護者：** Mechatronics Team
**最後更新：** 2025-12-14
**版本：** 1.1
