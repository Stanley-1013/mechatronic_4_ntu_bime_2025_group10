# v3_stable - Arduino 自主控制版

**版本：** 3.14
**狀態：** ✅ 穩定版（推薦）
**架構：** Arduino 獨立決策

---

## 一、版本特色

v3_stable 是完整的自走清掃版本，具備以下核心能力：

- **MPU6050 IMU 航向鎖定** - Yaw 角度積分，輔助轉彎判斷
- **PD 沿牆控制** - 維持目標距離 15cm，精確跟隨邊界
- **右側雙超聲波** - 右前 + 右後感測，提高牆面探測精度
- **角落擺頭清掃** - 轉彎前原地擺動，確保邊角覆蓋
- **100% Arduino 獨立** - 全數位邏輯在微控制器執行，Pi 僅做遙控轉發

### 與其他版本比較

| 特性 | v1.0 | v2.0 | v3_stable |
|------|------|------|-----------|
| 馬達控制 | 基礎 PWM | PWM | 左右獨立 PWM |
| 超聲波 | 左+右側 | 前+右側 | **右前+右後** |
| IMU | ❌ | ✅ (基礎) | **✅ (完整 Yaw)** |
| 沿牆演算法 | 簡單 IF | 基礎 | **PD 控制** |
| 轉彎策略 | 固定時間 | 傳感器判斷 | **角度+距離雙判** |
| 擺頭清掃 | ❌ | ❌ | **✅** |
| 推薦指數 | 🌟 | 🌟🌟 | **🌟🌟🌟🌟🌟** |

---

## 二、快速開始

### 2.1 硬體檢查清單

在上傳程式前，確認以下硬體連接：

```
✅ L298N 馬達驅動
   ├─ ENA (D3) - 左輪 PWM
   ├─ ENB (D11) - 右輪 PWM
   ├─ IN1-4 (D6,D5,D10,D9) - 方向控制
   └─ GND - 與 Arduino 共地

✅ 超聲波感測器
   ├─ HC-SR04 #1 (前方)：D7 (Trig), D8 (Echo)
   ├─ HC-SR04 #2 (右前)：A1 (Trig), A2 (Echo)
   └─ HC-SR04 #3 (右後)：D2 (Trig), D4 (Echo)

✅ MPU6050 IMU
   ├─ SDA → A4
   ├─ SCL → A5
   └─ INT → 未連接 (輪詢模式)

✅ 吸塵器繼電器
   └─ Control → A3

✅ 電源
   ├─ 18650 3S (11.1V) → L298N +12V
   └─ DC-DC 5V/3A → Arduino + Raspberry Pi
```

詳細接線圖見 [TECHNICAL_REFERENCE.md](../TECHNICAL_REFERENCE.md#2-硬體架構)

### 2.2 Arduino 上傳步驟

1. **安裝 Arduino IDE** (v1.8.19 或更新)
   ```bash
   # Ubuntu
   sudo apt install arduino

   # 或從官方下載：https://www.arduino.cc/software
   ```

2. **開啟主程式**
   ```bash
   cd v3_stable/arduino/main
   open main.ino  # 在 Arduino IDE 中開啟
   ```

3. **安裝必需庫**
   Arduino IDE → 工具 → 管理庫
   - **MPU6050** by InvenSense (v1.0.11+)
     ```
     搜尋 "MPU6050"，選擇 InvenSense 官方版本
     ```

4. **配置開發板**
   Arduino IDE → 工具
   ```
   開發板：Arduino Uno
   處理器：ATmega328P
   埠：/dev/ttyACM0 (Linux) 或 COM3 (Windows)
   速率：115200 bps
   ```

5. **編譯 & 上傳**
   ```
   素描圖 → 驗證 (編譯)
   素描圖 → 上傳 (上傳到 Arduino)
   ```

6. **驗證連線**
   Arduino IDE → 工具 → 序列埠監視器 (115200 bps)

   應看到啟動訊息：
   ```
   === Arduino Control System Started ===
   ```

### 2.3 Raspberry Pi 執行

**前提：** Arduino 已成功上傳程式

1. **進入 Pi 端目錄**
   ```bash
   cd v3_stable/raspberry_pi
   ```

2. **安裝 Python 依賴**
   ```bash
   pip install -r requirements.txt
   ```

3. **執行自走模式**
   ```bash
   # 完整模式（有相機紅色偵測）
   python3 autonomous_main.py

   # 無相機模式（僅超聲波）
   python3 autonomous_main.py --no-camera
   ```

4. **監控執行狀態**

   程式會每 100ms 列印一次傳感器讀值：
   ```
   Front: 45.2cm, Right: 18.5cm, Yaw: 12.3°, Mode: forward
   Front: 44.8cm, Right: 18.2cm, Yaw: 12.5°, Mode: forward
   ```

### 2.4 測試模式

獨立測試各個子系統：

```bash
# 測試超聲波感測器
python3 test_ultrasonic.py

# 測試 IMU (MPU6050)
python3 test_mpu6050.py

# 測試馬達 (需手動監視)
python3 test_motors.py
```

---

## 三、參數調整指南

所有核心參數位於 `arduino/main/config.h`，以下說明關鍵調參項目。

### 3.1 直走平衡（最重要）

**目標：** 不給控制指令時，小車應能直走而不偏向左或右。

**相關參數：**
```c
#define BASE_PWM_L      64      // 左輪直走 PWM
#define BASE_PWM_R      77      // 右輪直走 PWM
```

**調參方法：**

1. 讓小車靜止在平坦地面，前方 2 米處有固定參考點（如線條）
2. 給予 `BASE_PWM_L` 和 `BASE_PWM_R` 相同值，執行前進 10 秒
3. 觀察小車是否偏向左或右
4. 若向右偏：**增加 `BASE_PWM_L`** 或 **減少 `BASE_PWM_R`**
5. 若向左偏：**增加 `BASE_PWM_R`** 或 **減少 `BASE_PWM_L`**
6. 重複直到小車完全直走

**預期值範圍：**
- `BASE_PWM_L`: 60-70
- `BASE_PWM_R`: 75-85

### 3.2 沿牆控制參數

**目標：** 小車應能平行於牆壁，保持 15cm 距離不抖動。

**相關參數：**
```c
#define TARGET_DIST     15.0f   // 目標右側距離 cm
#define KP_DIST         0.5f    // 距離→目標角度
#define KP_ANGLE        0.04f   // 角度比例增益
#define KD_ANGLE        0.02f   // 角度微分增益
#define MAX_TARGET_ANGLE 10.0f  // 目標角度限幅
```

**調參流程：**

1. **驗證超聲波有效**
   - 右側應持續得到有效距離讀值（10-40cm）
   - 若讀數抖動，檢查感測器安裝是否牢固

2. **調 TARGET_DIST**
   - 若希望更靠近牆：減少到 12cm
   - 若希望更遠離牆：增加到 18cm

3. **調 PD 增益**（距離正常但抖動）

   **症狀：頻繁左右搖擺**
   - 降低 `KP_ANGLE` (0.04 → 0.03)
   - 增加 `KD_ANGLE` (0.02 → 0.03)

   **症狀：反應遲鈍，偏離後久久才修正**
   - 增加 `KP_ANGLE` (0.04 → 0.05)
   - 降低 `KD_ANGLE` (0.02 → 0.01)

4. **調距離→角度轉換**（偏離距離多時反應太強/太弱）
   - 若 7cm 誤差時修正力度不足：增加 `KP_DIST` (0.5 → 0.7)
   - 若修正過度（往回擺）：降低 `KP_DIST` (0.5 → 0.3)

### 3.3 前方避障參數

```c
#define FRONT_STOP      16.0f   // 觸發轉彎距離
#define FRONT_SLOW      30.0f   // 減速距離
```

**調參：**
- 若經常卡在角落無法脫困：降低 `FRONT_STOP` (16 → 14)
- 若轉彎時太急促：增加 `FRONT_SLOW` (30 → 40)

### 3.4 轉彎完成判斷

```c
#define TURN_FRONT_CLEAR    30.0f   // 轉彎完成：前方暢通
#define TURN_RIGHT_MIN      10.0f   // 轉彎完成：右側最小
#define TURN_RIGHT_MAX      40.0f   // 轉彎完成：右側最大
#define TURN_ANGLE_TOL      10.0f   // 角度容許誤差
```

**調參：**
- 若轉完彎後馬上卡到前方障礙：增加 `TURN_FRONT_CLEAR` (30 → 35)
- 若轉彎時卡角：調整 `TURN_RIGHT_MIN/MAX` 範圍

### 3.5 擺頭清掃參數

```c
#define SWEEP_ANGLE     30.0f   // 擺頭角度 (度)
#define SWEEP_PWM       70      // 擺頭 PWM
#define SWEEP_TIMEOUT   166     // 擺頭超時 (≈5秒)
```

**調參：**
- 若角落未清掃完整：增加 `SWEEP_ANGLE` (30 → 40)
- 若擺頭太慢：增加 `SWEEP_PWM` (70 → 80)

---

## 四、目錄結構

```
v3_stable/
│
├── README.md                    # ⭐ 本檔案（版本總覽）
│
├── arduino/main/
│   ├── main.ino                # Arduino 主程式入口
│   ├── config.h                # 所有參數定義
│   ├── motor_driver.h/cpp      # L298N 馬達控制
│   ├── ultrasonic_sensor.h/cpp # HC-SR04 超聲波
│   ├── mpu6050_sensor.h/cpp    # MPU6050 IMU
│   ├── vacuum_controller.h/cpp # 吸塵器繼電器
│   └── serial_protocol.h/cpp   # Serial 通訊協定
│
├── raspberry_pi/
│   ├── autonomous_main.py      # 自走模式主程式
│   ├── config.py               # Pi 端參數設定
│   ├── arduino_controller.py   # Serial 通訊驅動
│   ├── differential_drive.py   # 差動驅動演算法
│   ├── wall_follower.py        # 沿牆控制器
│   ├── red_detector.py         # 紅色物體偵測
│   ├── imu_processor.py        # IMU 資料融合
│   ├── test_*.py               # 各項測試程式
│   └── requirements.txt        # Python 依賴清單
│
└── docs/
    ├── 01_SRS_軟體需求規格.md      # 功能需求
    ├── 02_SA_系統分析.md           # 需求分析
    ├── 03_SD_系統設計.md           # 設計文件
    ├── 04_ICD_介面控制.md          # 通訊協定規格
    └── 05_TDD_測試設計.md          # 測試計畫
```

---

## 五、軟工文件索引

本版本包含 5 份完整的軟體工程文件：

### 5.1 需求 & 分析

| 文件 | 內容概要 | 適讀者 |
|------|---------|--------|
| **01_SRS_軟體需求規格.md** | 功能需求、非功能需求、使用案例 | 需求方、PM |
| **02_SA_系統分析.md** | 系統架構、資料流、模組分解 | 架構師、開發者 |

### 5.2 設計 & 實現

| 文件 | 內容概要 | 適讀者 |
|------|---------|--------|
| **03_SD_系統設計.md** | 詳細設計、類別圖、序列圖、狀態機 | 開發者、測試者 |
| **04_ICD_介面控制.md** | Serial 通訊協定、封包格式、時序 | 嵌入式工程師 |

### 5.3 測試

| 文件 | 內容概要 | 適讀者 |
|------|---------|--------|
| **05_TDD_測試設計.md** | 測試計畫、測試用例、驗收標準 | QA、測試工程師 |

**快速查詢：** 見 [docs/README.md](docs/README.md)

---

## 六、詳細技術文件

本版本基於完整的技術參考文檔構建。以下是重點主題：

### 系統設計核心
→ [TECHNICAL_REFERENCE.md](../TECHNICAL_REFERENCE.md)
- 完整硬體架構圖（含電源分配）
- 通訊協定 v2.0（12-byte 感測器封包）
- 控制演算法細節（差動驅動、沿牆 PD、紅色迴避）
- 感測器整合與融合策略

### 版本演進歷史
→ [EVOLUTION.md](../EVOLUTION.md)
- v1.0 → v3_stable 完整演變過程
- 各版本間的設計決策與改進
- 已知問題與解決方案匯總

### 即時狀態表
→ [STATUS.md](../STATUS.md)
- 各版本當前維護狀態
- 已知限制與缺陷
- 未來改進方向

---

## 七、常見問題 (FAQ)

### Q1: 小車轉彎時卡住怎麼辦？

**可能原因：**
1. 右側超聲波被卡住 → 檢查傳感器安裝，清理障礙物
2. `FRONT_STOP` 設定太高 → 降低到 14-15cm
3. 馬達功率不足 → 增加 `TURN_PWM` (70 → 80)

**驗證方法：**
```bash
python3 test_ultrasonic.py
# 觀察右側距離是否持續有效讀值
```

### Q2: 沿牆時左右搖擺，無法穩定跟蹤怎麼辦？

**調整步驟：**
1. 驗證超聲波感測器無卡頓 (見 Q1)
2. 降低 `KP_ANGLE` 減小過度修正
3. 增加 `KD_ANGLE` 增強阻尼

```c
// 原設定
#define KP_ANGLE  0.04f
#define KD_ANGLE  0.02f

// 改為
#define KP_ANGLE  0.032f
#define KD_ANGLE  0.025f
```

### Q3: Raspberry Pi 無法與 Arduino 通訊怎麼辦？

**檢查清單：**
```bash
# 1. 驗證串列埠連線
ls /dev/ttyACM*
# 應該看到 /dev/ttyACM0

# 2. 檢查 USB 連線
dmesg | tail -20
# 應該看到 "CH340" 或 "FTDI" 驅動訊息

# 3. 驗證 Arduino 程式已上傳
# Arduino IDE 中執行 upload，應無錯誤

# 4. 測試通訊
python3 -c "
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
data = ser.read()
print(f'Received: {data}')
ser.close()
"
```

**若仍無法通訊：**
- 重新上傳 Arduino 程式
- 重新啟動 Raspberry Pi
- 檢查 USB 纜線品質（嘗試更換纜線）

### Q4: 怎麼調整沿牆時的目標距離？

編輯 `config.h`：
```c
#define TARGET_DIST  15.0f   // 改為 12.0f (更靠近) 或 18.0f (更遠離)
```

然後重新編譯上傳 Arduino。

---

## 八、維護與支援

### 版本維護狀態

| 版本 | 狀態 | 最後更新 | 支援期限 |
|------|------|---------|---------|
| v3_stable | **活躍** | 2025-11-29 | 2026-06-30 |
| v2.0 | 存檔 | 2025-11-29 | 2025-12-31 |
| v1.1 | 已棄用 | 2025-11-28 | 2025-11-30 |

### 回報問題

若遇到問題，請提供以下資訊：

1. 版本號（config.h 頂部）
2. 硬體清單（超聲波數量、IMU 型號等）
3. 詳細現象描述
4. 序列埠監視器輸出（複製 5-10 行）

### 貢獻指南

改進建議歡迎提交！請：
1. 清楚說明改進內容
2. 提供測試結果
3. 更新對應文件（config.h 註釋、此 README 等）

---

## 九、相關資源

### 官方數據手冊
- [L298N 馬達驅動器](https://www.st.com/resource/en/datasheet/l298.pdf)
- [HC-SR04 超聲波模組](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- [MPU6050 IMU 寄存器表](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

### 相關教學
- [Arduino 官方文件](https://www.arduino.cc/reference/)
- [RPi 官方指南](https://www.raspberrypi.com/documentation/)

---

## 十、版本記錄

```
2025-11-29  v3.14  簡化擺頭流程 + 無牆右偏補償
├─ 新增 SEARCH_ANGULAR 參數（找牆時輕微右弧）
├─ 擺頭流程改進（角落更精確）
└─ 轉彎超時保護加強

2025-11-28  v3.13  新增角落擺頭清掃 + 參數微調
├─ 轉彎前進行 ±30° 擺頭
├─ 調整 TURN_ANGLE_TOL (15° → 10°)
└─ BASE_PWM 微調

2025-11-27  v3.12  簡化轉彎邏輯 + 加強 PD 參數
├─ 轉彎判斷改為雙條件（前方距離 + 右側範圍）
├─ 優化 PD 增益（KP=0.04, KD=0.02）
└─ 降低 TURN_ANGLE_TOL 至 10°

2025-11-20  v3.10+  沿牆改加減法 PD + 新增簡化版
├─ 距離誤差轉目標角度（PD 控制角度而非速度差）
├─ 新增 v4_simple 簡化版本
└─ 完整測試與驗收

2025-11-15  v3.0  MPU6050 整合
├─ 新增 IMU 支援
├─ Yaw 角度積分
└─ 轉彎判斷加入角度條件
```

---

**文檔版本：** 1.0
**最後更新：** 2025-11-29
**維護者：** Mechatronics Team

---

*若發現文檔過時或有誤，請立即回報！*
