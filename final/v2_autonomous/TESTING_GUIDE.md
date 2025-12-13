# v2 實車測試與調優指南

本指南說明如何將 v2_autonomous 系統部署到實車，並針對不同場地進行參數調優。

---

## 1. 部署前檢查清單

### 硬體需求
- [ ] Arduino Uno/Mega 開發板
- [ ] Raspberry Pi 4 (建議) 或 Pi 3B+
- [ ] 超聲波感測器 x2 (前方、右側)
- [ ] MPU6050 IMU 模組
- [ ] L298N 馬達驅動
- [ ] DC 馬達 x2 (左、右輪)
- [ ] USB 攝影機 (Pi 紅色偵測用)
- [ ] 電源供應 (Arduino 5V + 馬達 12V)

### 軟體需求

**Arduino 端：**
- Arduino IDE 1.8.x 或 2.x
- Wire.h (MPU6050，內建)

**Pi 端：**
```bash
# Python 3
sudo apt-get install python3 python3-pip

# OpenCV (影像處理)
pip3 install opencv-python

# PySerial (串列通訊)
pip3 install pyserial
```

### 連線確認
- [ ] Arduino USB 連接到 Pi (檢查 `/dev/ttyACM0` 或 `/dev/ttyUSB0`)
- [ ] USB 攝影機連接到 Pi (檢查 `/dev/video0`)
- [ ] Pi Serial 權限設定：
  ```bash
  sudo usermod -a -G dialout $USER
  # 登出再登入生效
  ```

---

## 2. Arduino 部署步驟

### 2.1 開啟專案

```
Arduino IDE → File → Open
選擇: v2_autonomous/arduino/main.ino
```

### 2.2 設定開發板與腳位

1. **選擇開發板**：
   - `Tools → Board → Arduino AVR Boards → Arduino Uno` (或 Mega)

2. **選擇 COM Port**：
   - `Tools → Port → /dev/ttyACM0` (Linux)
   - 或 `COM3` (Windows)

3. **確認腳位設定** (檢查 `config.h`)：
   ```cpp
   // 超聲波感測器
   #define PIN_US_FRONT_TRIG  7
   #define PIN_US_FRONT_ECHO  8
   #define PIN_US_RIGHT_TRIG  12
   #define PIN_US_RIGHT_ECHO  13

   // L298N 馬達驅動
   #define PIN_ENA  9   // 左馬達 PWM
   #define PIN_IN1  2
   #define PIN_IN2  4
   #define PIN_ENB  10  // 右馬達 PWM
   #define PIN_IN3  5
   #define PIN_IN4  6

   // IMU (I2C)
   #define MPU6050_ENABLED
   // SDA → A4
   // SCL → A5
   ```

### 2.3 編譯與上傳

```
1. 點擊 "Verify" 按鈕 (✓) - 確認編譯無誤
2. 點擊 "Upload" 按鈕 (→) - 上傳到 Arduino
3. 等待 "Done uploading" 訊息
```

### 2.4 驗證啟動

```
1. 點擊 Serial Monitor 按鈕 (右上角放大鏡圖示)
2. 設定波特率: 115200
3. 應該看到啟動訊息:
   =========================================
    v2 Autonomous Wall Follower
    Arduino Controller
   =========================================
   [OK] Motor driver initialized
   [OK] Ultrasonic sensors initialized
   [OK] Vacuum controller initialized
   [IMU] Initializing MPU6050... OK
   [OK] Serial handler ready
   Ready for commands from Raspberry Pi
```

---

## 3. Raspberry Pi 部署步驟

### 3.1 檔案部署

```bash
# 方式 1: 直接在 Pi 上 git clone (建議)
cd ~
git clone <專案 repo URL>
cd mechtronic_4/v2_autonomous/raspberry_pi

# 方式 2: 從開發機透過 scp 傳送
scp -r v2_autonomous/raspberry_pi pi@<Pi_IP>:~/
```

### 3.2 執行整合測試

**目的：驗證 Serial 連線、紅色偵測、指令收發是否正常**

```bash
cd ~/mechtronic_4/v2_autonomous/raspberry_pi
python3 test_integration.py
```

**預期輸出：**
```
========================================
v2 整合測試
========================================
[1] 測試 Serial 連線... ✓ Serial port /dev/ttyACM0 opened
[2] 測試紅色偵測... ✓ RedDetector initialized (camera opened)
[3] 發送 START 指令... ✓ Command sent
[4] 接收狀態封包... ✓ State: FORWARD, Corners: 0, Front: 25cm, Right: 15cm
[5] 發送 STOP 指令... ✓ Command sent
========================================
整合測試通過！
```

如果出現錯誤，參考「6. 常見問題排除」。

### 3.3 啟動控制器

```bash
# 啟用紅色偵測 (預設)
python3 controller.py

# 禁用紅色偵測 (測試用)
python3 controller.py --no-red

# 指定 Serial Port
python3 controller.py --port /dev/ttyUSB0
```

**互動指令：**
```
> start          # 啟動沿牆模式
> stop           # 停止
> red            # 手動觸發避紅色
> vacuum on      # 開啟吸塵器
> vacuum off     # 關閉吸塵器
> status         # 查看當前狀態
> red_status     # 查看紅色偵測狀態
> quit           # 退出
```

---

## 4. 調優參數說明

### 4.1 Arduino 沿牆參數 (`wall_follower.cpp`)

| 參數 | 預設值 | 說明 | 調優建議 |
|------|--------|------|----------|
| `TARGET_RIGHT_DISTANCE` | 15 cm | 目標右牆距離 | **場地寬**：提高到 20 cm；**場地窄**：降低到 12 cm |
| `FRONT_STOP_DISTANCE` | 15 cm | 前方停止距離 (觸發轉彎) | **車速快**：提高到 20 cm；**場地小**：降低到 12 cm |
| `FRONT_SLOW_DISTANCE` | 40 cm | 前方減速距離 | 根據 BASE_LINEAR_SPEED 調整，速度越快距離越遠 |
| `BASE_LINEAR_SPEED` | 0.6 | 基礎前進速度 (0.0~1.0) | **初次測試**：0.3；**穩定後**：0.5~0.7 |
| `SLOW_LINEAR_SPEED` | 0.35 | 減速時速度 | 約為 BASE_LINEAR_SPEED 的 50%~60% |
| `TURN_ANGULAR_SPEED` | 0.7 | 原地轉彎角速度 (0.0~1.0) | **轉彎不足**：提高到 0.8；**轉彎過頭**：降低到 0.6 |
| `WALL_FOLLOW_KP` | 0.025 | 沿牆 P 控制增益 | **震盪**：降低到 0.015；**反應慢**：提高到 0.035 |
| `BACKUP_DURATION_MS` | 300 ms | 角落後退時間 | **撞牆**：提高到 400 ms；**效率低**：降低到 200 ms |
| `TURN_DURATION_MS` | 800 ms | 原地左轉時間 (IMU 備援) | **轉彎不足**：提高到 1000 ms |
| `YAW_TOLERANCE` | 5.0° | IMU 角度誤差容許 | 視 IMU 穩定度調整 |

**調優方式：**

1. 修改 `v2_autonomous/arduino/wall_follower.cpp` 檔案中的參數
2. 重新編譯並上傳到 Arduino
3. 觀察實車表現，迭代調整

**常見調優情境：**

| 情境 | 現象 | 調整參數 |
|------|------|----------|
| **沿牆震盪** | 車身左右搖擺 | 降低 `WALL_FOLLOW_KP` (0.025 → 0.018) |
| **轉彎不足** | 轉 90° 後角度不夠 | 提高 `TURN_ANGULAR_SPEED` (0.7 → 0.8) |
| **轉彎過頭** | 轉彎後偏移太多 | 降低 `TURN_ANGULAR_SPEED` (0.7 → 0.6) |
| **撞牆** | 角落時撞上前牆 | 提高 `FRONT_STOP_DISTANCE` (15 → 20) |
| **離牆太遠** | 與右牆距離太遠 | 降低 `TARGET_RIGHT_DISTANCE` (15 → 12) |
| **右輪一直轉** | FIND_WALL 找不到牆 | 檢查右側感測器是否正常 |

### 4.2 Pi 紅色偵測參數 (`controller.py`)

| 參數 | 預設值 | 說明 | 調優建議 |
|------|--------|------|----------|
| `RED_AREA_THRESHOLD` | 2000 | 紅色面積閾值 (像素) | **誤觸發太多**：提高到 3000；**反應不靈敏**：降低到 1500 |
| `RED_CHECK_INTERVAL` | 0.5 s | 紅色檢查間隔 | **反應慢**：降低到 0.3 s；**CPU 占用高**：提高到 0.8 s |

**調優方式：**

1. 修改 `v2_autonomous/raspberry_pi/controller.py` 檔案開頭的常數
2. 重新啟動 controller
3. 使用 `red_status` 指令監控偵測狀態

**紅色偵測調優流程：**

```bash
# 1. 測試紅色偵測視窗（顯示偵測框與面積）
python3 red_detector.py
# 按 'q' 離開

# 2. 放置紅色物體，觀察面積值
# 3. 根據實測面積調整 RED_AREA_THRESHOLD
# 4. 重啟 controller 驗證
```

### 4.3 紅色偵測 HSV 範圍調整 (`red_detector.py`)

如果環境光線或紅色物體顏色導致偵測不準，可調整 HSV 閾值：

```python
# v2_autonomous/raspberry_pi/red_detector.py
class RedDetector:
    # 紅色範圍 (H: 0~180, S: 0~255, V: 0~255)
    LOWER_RED1 = np.array([0, 100, 100])    # 第一段 (0~10)
    UPPER_RED1 = np.array([10, 255, 255])
    LOWER_RED2 = np.array([160, 100, 100])  # 第二段 (160~180)
    UPPER_RED2 = np.array([180, 255, 255])

    MIN_AREA = 800  # 最小輪廓面積（過濾雜訊）
```

**調優建議：**
- **偵測不到深紅色**：降低 V (Value) 下限 (100 → 80)
- **偵測到橘色/粉紅色**：縮小 H (Hue) 範圍 (0~10 → 0~8)
- **雜訊過多**：提高 S (Saturation) 下限 (100 → 120)

---

## 5. 測試流程

### 5.1 靜態測試（桌上測試）

**目的：驗證感測器與馬達功能，不需要完整場地**

```
1. 架高車體，讓輪子懸空
2. 啟動 Arduino，開啟 Serial Monitor
3. 用手/障礙物觸發超聲波感測器，觀察讀值
4. 啟動 Pi controller，發送 "start" 指令
5. 觀察輪子是否轉動 (FIND_WALL 應右前方移動)
6. 發送 "stop" 指令，輪子應停止
```

**驗收標準：**
- 超聲波感測器讀值正確 (0~200 cm)
- IMU yaw 角度穩定 (±1°)
- 馬達轉向正確 (前進/後退/左轉/右轉)

### 5.2 單項測試（模擬場景）

**目的：測試各狀態機狀態的正確性**

| 狀態 | 測試方式 | 預期行為 |
|------|----------|----------|
| `FIND_WALL` | 啟動時右側無牆 | 右前方移動 (右轉+前進) |
| `FORWARD` | 右側有牆 (15~20 cm) | 直行沿牆，保持 15 cm |
| `BACKUP` | 前方+右側同時有牆 (< 15 cm) | 後退 0.3 秒 |
| `TURN_LEFT` | 前方有牆 (< 15 cm) | 原地左轉 90° |

**驗收標準：**
- 狀態轉換正確 (Serial Monitor 輸出狀態名稱)
- 馬達輸出符合預期 (左右 PWM 值)

### 5.3 整合測試（完整沿牆）

**目的：在簡單場地測試完整沿牆流程**

```
測試場地: 2x2 公尺矩形區域
1. 車體放置於場地右側，右側緊鄰牆壁
2. 啟動 "start" 指令
3. 觀察：
   - 直行沿牆 (FORWARD)
   - 到達角落 (BACKUP → TURN_LEFT)
   - 轉彎後繼續沿牆
   - 完整繞場一圈
4. 記錄角落計數 (應為 4)
```

**驗收標準：**
- 完整繞場不撞牆
- 角落計數正確
- 沿牆距離穩定 (12~18 cm)

### 5.4 紅色避障測試

**目的：驗證紅色偵測與避障功能**

```
1. 在場地中放置紅色物體 (A4 紅紙或紅色障礙物)
2. 啟動沿牆模式
3. 觀察車體接近紅色物體時：
   - Pi 印出 "偵測到紅色區域 (面積=XXX)"
   - 車體觸發避障 (後退+轉彎)
```

**驗收標準：**
- 紅色物體距離 30~50 cm 時觸發
- 車體成功避開紅色區域
- 避障後繼續沿牆

---

## 6. 常見問題排除

| 問題 | 可能原因 | 解決方案 |
|------|----------|----------|
| **車子不動** | Serial 連線失敗 | 檢查 `/dev/ttyACM0` 是否存在；確認 Pi 使用者在 `dialout` 群組 |
| **右輪一直轉** | FIND_WALL 方向錯誤 | 檢查右側超聲波是否接線正確；確認 `_setMotorOutput(0.6, -0.15)` 中 angular 符號 |
| **沿牆震盪** | KP 增益太高 | 降低 `WALL_FOLLOW_KP` (0.025 → 0.015) |
| **撞前牆** | 停止距離太小 | 提高 `FRONT_STOP_DISTANCE` (15 → 20) |
| **轉彎不足** | 角速度太低或時間太短 | 提高 `TURN_ANGULAR_SPEED` (0.7 → 0.8) 或 `TURN_DURATION_MS` (800 → 1000) |
| **轉彎過頭** | 角速度太高 | 降低 `TURN_ANGULAR_SPEED` (0.7 → 0.6) |
| **IMU 角度飄移** | 未校正 | 開啟 `MPU6050_CALIBRATE_ON_BOOT` in `config.h`，重新上傳 |
| **紅色誤觸發** | 閾值太低 | 提高 `RED_AREA_THRESHOLD` (2000 → 3000) |
| **紅色偵測失效** | 攝影機未開啟 | 檢查 `/dev/video0` 是否存在；重新啟動 controller |
| **Serial Buffer Overflow** | 通訊速率過高 | 本架構不應出現此問題，檢查 Arduino Serial.print 頻率 |
| **馬達死區** | PWM 太低 | 確認 `MIN_EFFECTIVE_PWM = 60`，低於此值自動設為 0 |

---

## 7. 效能調優指南

### 7.1 提升沿牆穩定性

**目標：減少震盪、平滑沿牆軌跡**

1. **降低 P 控制增益**：
   ```cpp
   static const float WALL_FOLLOW_KP = 0.018;  // 從 0.025 降低
   ```

2. **增加超聲波取樣穩定性**（修改 `ultrasonic_sensor.cpp`）：
   ```cpp
   // 取中位數而非單次讀值
   int distances[3];
   for (int i = 0; i < 3; i++) {
       distances[i] = getDistance();
       delay(10);
   }
   sort(distances, distances + 3);
   return distances[1];  // 中位數
   ```

3. **降低車速**（初次測試）：
   ```cpp
   static const float BASE_LINEAR_SPEED = 0.4;
   ```

### 7.2 提升轉彎精度

**目標：確保 90° 轉彎準確**

1. **啟用 IMU 角度控制**（預設已啟用）：
   - 確認 `config.h` 中 `#define MPU6050_ENABLED`
   - 確認 `YAW_TOLERANCE = 5.0°`

2. **校正 IMU**：
   ```cpp
   // config.h
   #define MPU6050_CALIBRATE_ON_BOOT
   #define IMU_CALIBRATION_SAMPLES 1000
   ```
   重新上傳，開機時靜置 10 秒

3. **調整轉彎角度**（如果轉彎不足）：
   ```cpp
   static const float TURN_TARGET_YAW_OFFSET = -95.0;  // 從 -90.0 調整
   ```

### 7.3 提升紅色偵測穩定性

**目標：避免誤觸發、提高反應速度**

1. **調整面積閾值**（實測調整）：
   ```python
   # controller.py
   RED_AREA_THRESHOLD = 2500  # 根據實測紅色物體面積
   ```

2. **增加連續偵測次數**（避免單幀誤觸發）：
   ```python
   # 修改 controller.py 增加計數器
   self._red_consecutive_count = 0

   # 在 _check_red_detection 中
   if detected and area > RED_AREA_THRESHOLD:
       self._red_consecutive_count += 1
       if self._red_consecutive_count >= 3:  # 連續 3 次才觸發
           self.send_avoid_red()
   else:
       self._red_consecutive_count = 0
   ```

3. **調整檢查頻率**（根據場景）：
   ```python
   RED_CHECK_INTERVAL = 0.3  # 從 0.5 降低到 0.3s
   ```

---

## 8. 實車測試檢查表

### 硬體檢查
- [ ] 所有接線牢固（特別是跳線）
- [ ] 電池電量充足 (馬達 12V > 11V, Arduino 5V 穩定)
- [ ] 超聲波感測器固定穩固（避免震動）
- [ ] 輪子接觸地面良好（無滑動）
- [ ] IMU 模組水平安裝

### 軟體檢查
- [ ] Arduino Serial Monitor 啟動正常
- [ ] Pi 可接收狀態封包 (status 指令有輸出)
- [ ] 紅色偵測視窗可開啟 (python3 red_detector.py)

### 功能測試
- [ ] FIND_WALL：右前方移動正常
- [ ] FORWARD：沿牆距離穩定 (12~18 cm)
- [ ] BACKUP：角落後退正常
- [ ] TURN_LEFT：原地左轉 90° (±5°)
- [ ] 紅色偵測：30~50 cm 觸發避障
- [ ] 完整繞場：4 個角落正確轉彎

---

## 9. 進階功能（選配）

### 9.1 日誌記錄（Pi 端）

記錄測試數據供分析：

```python
# 修改 controller.py 增加日誌
import logging

logging.basicConfig(
    filename='robot_log.txt',
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)

# 在 _parse_state 中
logging.info(f"State: {state_name}, Front: {front_dist}, Right: {right_dist}, Yaw: {yaw}")
```

### 9.2 遠端監控（透過 SSH）

```bash
# 從開發機監控 Pi Serial 輸出
ssh pi@<Pi_IP> "cd ~/mechtronic_4/v2_autonomous/raspberry_pi && python3 controller.py"
```

### 9.3 自動參數調優（PID Auto-Tuning）

實作 Ziegler-Nichols 方法自動調整 KP（進階）。

---

## 附錄 A：參數速查表

### Arduino 關鍵參數

```cpp
// wall_follower.cpp
TARGET_RIGHT_DISTANCE = 15 cm       // 沿牆距離
FRONT_STOP_DISTANCE = 15 cm         // 轉彎觸發距離
BASE_LINEAR_SPEED = 0.6             // 前進速度
TURN_ANGULAR_SPEED = 0.7            // 轉彎速度
WALL_FOLLOW_KP = 0.025              // P 控制增益
BACKUP_DURATION_MS = 300 ms         // 後退時間
TURN_DURATION_MS = 800 ms           // 轉彎時間
```

### Pi 關鍵參數

```python
# controller.py
RED_AREA_THRESHOLD = 2000           # 紅色面積閾值
RED_CHECK_INTERVAL = 0.5            # 紅色檢查間隔

# red_detector.py
LOWER_RED1 = [0, 100, 100]          # HSV 紅色下限 1
UPPER_RED1 = [10, 255, 255]         # HSV 紅色上限 1
LOWER_RED2 = [160, 100, 100]        # HSV 紅色下限 2
UPPER_RED2 = [180, 255, 255]        # HSV 紅色上限 2
MIN_AREA = 800                      # 最小輪廓面積
```

---

## 附錄 B：通訊協定快速參考

### Pi → Arduino 指令封包

```
格式: [0xAA] [CMD] [LEN] [DATA...] [CHECKSUM]
```

| 指令 | CMD 值 | 說明 |
|------|--------|------|
| START | 0x01 | 啟動沿牆 |
| STOP | 0x02 | 停止 |
| AVOID_RED | 0x03 | 避紅色 |
| SET_VACUUM | 0x04 | 吸塵器 ON/OFF |
| QUERY_STATE | 0x05 | 查詢狀態 |

### Arduino → Pi 狀態封包

```
格式 (12 bytes):
[0xBB] [STATE] [CORNER] [FRONT_H] [FRONT_L] [RIGHT_H] [RIGHT_L]
[YAW_H] [YAW_L] [FLAGS] [CHECKSUM] [0x0A]
```

| 欄位 | 說明 |
|------|------|
| STATE | 當前狀態 (0x00~0x04) |
| CORNER | 角落計數 (0~255) |
| FRONT | 前方距離 (cm) |
| RIGHT | 右側距離 (cm) |
| YAW | 角度 x10 (int16, 帶符號) |
| FLAGS | 旗標 (bit0=VACUUM) |

---

**文檔版本：1.0**
**最後更新：2025-12-02**
**作者：Mechtronic 4 Team**
