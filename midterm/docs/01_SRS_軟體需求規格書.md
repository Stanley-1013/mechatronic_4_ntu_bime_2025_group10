# SRS - 軟體需求規格書
**Software Requirements Specification**

---

## 文件資訊

| 項目 | 內容 |
|------|------|
| 專案名稱 | 機電小車遙控系統 |
| 文件版本 | 1.1 |
| 建立日期 | 2025-10-31 |
| 最後更新 | 2025-10-31 |
| 作者 | Mechatronics Team |
| 硬體平台 | Raspberry Pi 4 + Arduino Uno |

**修訂歷史**：
- v1.1 (2025-10-31): 優化 FR3/FR5/FR6，明確 Serial 協定、吸塵器腳位、超聲波公式
- v1.0 (2025-10-31): 初始版本

---

## 1. 系統概述

### 1.1 系統目標

開發一個基於 Raspberry Pi 4 和 Arduino Uno 的遙控車系統，具備以下能力：
- 透過 2.4G USB 遙控器進行即時控制
- 搖桿控制 TT 馬達實現差動驅動（前進、後退、轉向）
- 按鈕控制第三顆馬達（吸塵器模組）開關
- 超聲波感測器偵測左右兩側障礙物
- 低延遲、高可靠性的控制響應

### 1.2 系統範圍

**階段 1（當前）：遙控模式**
- 硬體整合：Pi、Arduino、馬達驅動、感測器
- 遙控器輸入處理
- 馬達控制輸出
- 感測器資料回傳

**階段 2（未來）：自主導航**
- Pi Camera 視覺導航
- 路徑規劃

---

## 2. 功能需求 (Functional Requirements)

### FR1: 遙控器輸入處理
**描述**：系統應能讀取 2.4G USB 遙控器的搖桿和按鈕輸入。

**詳細需求**：
- FR1.1：系統應自動偵測並連接 USB 遙控器（`/dev/input/js0`）
- FR1.2：系統應讀取搖桿 X 軸（左轉/右轉）和 Y 軸（前進/後退）數值
- FR1.3：系統應正規化搖桿數值到 -1.0 ~ +1.0 範圍
- FR1.4：系統應實作死區過濾（deadzone < 0.1）防止搖桿飄移
- FR1.5：系統應讀取按鈕狀態控制吸塵器馬達

**輸入**：
- 搖桿 X 軸：-1.0 ~ +1.0（左 → 右）
- 搖桿 Y 軸：-1.0 ~ +1.0（後 → 前）
- 按鈕：ON/OFF

**輸出**：
- `VehicleCommand` 物件：
  - `linear_velocity`：線性速度 (-1.0 ~ +1.0)
  - `angular_velocity`：角速度 (-1.0 ~ +1.0)
  - `vacuum_motor`：吸塵器狀態 (True/False)

**優先級**：高（核心功能）

---

### FR2: 差動驅動運算
**描述**：系統應將線性速度和角速度轉換為左右輪速度。

**詳細需求**：
- FR2.1：系統應實作差動驅動公式：
  - `left_speed = linear_velocity - angular_velocity`
  - `right_speed = linear_velocity + angular_velocity`
- FR2.2：系統應將速度值轉換為 PWM 範圍（-255 ~ +255）
- FR2.3：系統應限制輸出範圍防止溢位

**輸入**：
- `linear_velocity`：-1.0 ~ +1.0
- `angular_velocity`：-1.0 ~ +1.0

**輸出**：
- `left_pwm`：-255 ~ +255
- `right_pwm`：-255 ~ +255

**優先級**：高（核心功能）

---

### FR3: Serial 通訊協定
**描述**：Raspberry Pi 與 Arduino 應透過 UART Serial 進行二進位通訊。

**詳細需求**：
- FR3.1：系統應使用 57600 bps 鮑率，8N1 格式
- FR3.2：Pi → Arduino 指令封包格式（8 bytes）：
  - Byte 0 - Header: `0xAA`
  - Byte 1 - Left PWM Low: `left_pwm & 0xFF`
  - Byte 2 - Left PWM High: `(left_pwm >> 8) & 0xFF`
  - Byte 3 - Right PWM Low: `right_pwm & 0xFF`
  - Byte 4 - Right PWM High: `(right_pwm >> 8) & 0xFF`
  - Byte 5 - Flags: `bit0=vacuum_motor, bit1-7=reserved`
  - Byte 6 - Checksum: `XOR of bytes 1-5`
  - Byte 7 - Footer: `0x55`
- FR3.3：Arduino → Pi 感測器封包格式（8 bytes）：
  - Byte 0 - Header: `0xBB`
  - Byte 1 - Left Distance Low: `left_distance & 0xFF`
  - Byte 2 - Left Distance High: `(left_distance >> 8) & 0xFF`
  - Byte 3 - Right Distance Low: `right_distance & 0xFF`
  - Byte 4 - Right Distance High: `(right_distance >> 8) & 0xFF`
  - Byte 5 - Reserved: `0x00`
  - Byte 6 - Checksum: `XOR of bytes 1-5`
  - Byte 7 - Footer: `0x66`
- FR3.4：系統應驗證 Checksum，錯誤時丟棄封包
- FR3.5：PWM 數值使用 int16 (2's complement)，範圍 -255 ~ +255
- FR3.6：距離數值使用 uint16，範圍 0 ~ 999（999 = 無效值）

**優先級**：高（核心功能）

---

### FR4: 馬達控制
**描述**：Arduino 應根據 Pi 的指令控制 L298N 驅動 TT 馬達。

**詳細需求**：
- FR4.1：系統應控制左輪馬達（IN1/IN2, ENA）
- FR4.2：系統應控制右輪馬達（IN3/IN4, ENB）
- FR4.3：系統應根據 PWM 正負值控制方向：
  - PWM > 0：前進（IN1=HIGH, IN2=LOW）
  - PWM < 0：後退（IN1=LOW, IN2=HIGH）
  - PWM = 0：停止（IN1=LOW, IN2=LOW）
- FR4.4：系統應輸出絕對值 PWM 到 ENA/ENB

**輸入**：
- `left_pwm`：-255 ~ +255
- `right_pwm`：-255 ~ +255

**輸出**：
- L298N 控制信號（GPIO + PWM）

**優先級**：高（核心功能）

---

### FR5: 吸塵器馬達控制
**描述**：Arduino 應根據按鈕狀態控制第三顆馬達（吸塵器模組）。

**詳細需求**：
- FR5.1：系統應接收 flags byte 的 bit0 作為開關信號
- FR5.2：系統應使用 GPIO D12 控制吸塵器馬達
- FR5.3：系統應支援切換（toggle）模式
- FR5.4：驅動方式：直接 GPIO 控制（ON/OFF，無 PWM 調速）
- FR5.5：電氣特性：
  - 電壓：5V（由 Arduino 5V pin 供電）
  - 電流：< 500mA（建議加裝 MOSFET 或繼電器）
  - 保護：建議加入二極體防止反向電動勢

**輸入**：
- `vacuum_motor`：True/False（來自 flags byte bit0）

**輸出**：
- Arduino D12 GPIO：HIGH (5V) / LOW (0V)

**硬體接線**：
```
Arduino D12 → MOSFET Gate → MOSFET Drain → Vacuum Motor (+)
                            MOSFET Source → GND
Vacuum Motor (-) → GND
```

**優先級**：中（次要功能）

---

### FR6: 超聲波感測器讀取
**描述**：Arduino 應定期讀取左右兩側 HC-SR04 超聲波感測器距離。

**詳細需求**：
- FR6.1：系統應觸發左側超聲波（Trig D7, Echo D8）
- FR6.2：系統應觸發右側超聲波（Trig A1, Echo A2）
- FR6.3：系統應計算距離公式：
  ```cpp
  // 聲速 340 m/s = 0.034 cm/μs
  // 距離 = (時間 × 聲速) / 2（來回）
  distance_cm = pulse_duration_us * 0.034 / 2
  ```
- FR6.4：系統應過濾超出範圍值：
  - 距離 < 2cm → 設為 999（無效值）
  - 距離 > 400cm → 設為 999（無效值）
  - 逾時（無回波 > 30ms）→ 設為 999
- FR6.5：系統應每 100ms 更新感測器資料
- FR6.6：測量流程：
  1. 發送 10μs HIGH 脈衝到 Trig pin
  2. 等待 Echo pin 變 HIGH
  3. 測量 Echo HIGH 持續時間（μs）
  4. 計算距離（cm）

**輸出**：
- `left_distance`：2 ~ 400 cm（有效）或 999（無效）
- `right_distance`：2 ~ 400 cm（有效）或 999（無效）

**優先級**：高（核心功能）

---

### FR7: 感測器資料回傳
**描述**：Arduino 應定期將感測器資料透過 Serial 傳送給 Pi。

**詳細需求**：
- FR7.1：系統應每 100ms 傳送一次感測器封包
- FR7.2：系統應使用 FR3.3 定義的封包格式
- FR7.3：Pi 應能解析並顯示感測器資料

**優先級**：中（監控功能）

---

## 3. 非功能需求 (Non-Functional Requirements)

### NFR1: 控制延遲
**描述**：系統的端對端控制延遲應小於 50ms。

**衡量標準**：
- 遙控器輸入 → Pi 處理 → Serial 傳輸 → Arduino 執行 → 馬達響應
- 總延遲 < 50ms

**優先級**：高

---

### NFR2: 通訊可靠性
**描述**：Serial 通訊應具備錯誤偵測能力。

**衡量標準**：
- Checksum 驗證成功率 > 99%
- 錯誤封包應被丟棄並記錄

**優先級**：高

---

### NFR3: 系統穩定性
**描述**：系統應能連續穩定運行。

**衡量標準**：
- 連續運行 30 分鐘無當機
- 記憶體洩漏 < 1MB/小時

**優先級**：高

---

### NFR4: 可擴充性
**描述**：系統架構應支援未來功能擴充。

**衡量標準**：
- 模組化設計（接收器、控制器、驅動分離）
- 支援替換遙控方案（Bluetooth、Web UI）

**優先級**：中

---

### NFR5: 易用性
**描述**：系統應易於操作和調試。

**衡量標準**：
- 插入 USB 遙控器後自動連接（< 5 秒）
- 提供即時狀態顯示（速度、距離）
- 提供測試工具（jstest、軸編號測試）

**優先級**：中

---

## 4. 系統限制 (Constraints)

### 4.1 硬體限制
- **Arduino Uno**：
  - SRAM：2KB（限制變數使用）
  - Flash：32KB（限制程式大小）
  - 無硬體浮點運算
- **Raspberry Pi 4**：
  - 需外部 5V/3A 供電
  - GPIO 電壓 3.3V（需與 Arduino 5V 共地）
- **L298N**：
  - 最大電流 2A per channel
  - 馬達電壓 9-12V（3S 18650 = 11.1V）

### 4.2 電源限制
- 3S 18650 電池容量有限（約 2200mAh）
- 運行時間估計 1-2 小時（視馬達負載）

### 4.3 通訊限制
- UART 鮑率：57600 bps
- 封包大小：8 bytes
- 最大傳輸頻率：50 Hz（考慮處理時間）

### 4.4 環境限制
- 超聲波感測器受環境影響（溫度、濕度、材質）
- 2.4G 遙控器有效範圍約 10-30 公尺

---

## 5. 驗收標準 (Acceptance Criteria)

### 5.1 功能驗收
- ✅ 遙控器連接成功率 100%
- ✅ 馬達可前進、後退、左轉、右轉
- ✅ 吸塵器按鈕正常開關
- ✅ 超聲波讀取距離誤差 < 5%
- ✅ Serial 通訊無明顯掉包（< 1%）

### 5.2 性能驗收
- ✅ 控制延遲 < 50ms
- ✅ 連續運行 30 分鐘無當機
- ✅ CPU 使用率 < 50%（Pi）

### 5.3 整合驗收
- ✅ 端對端測試：遙控器 → Pi → Arduino → 馬達
- ✅ 感測器資料正確回傳並顯示
- ✅ 緊急停止功能正常（搖桿歸零）

---

## 6. 假設與依賴 (Assumptions and Dependencies)

### 6.1 假設
- 使用者熟悉基本 Linux 操作
- Raspberry Pi 已安裝 Raspberry Pi OS
- 已安裝 Python 3.x 和 pygame 庫
- Arduino IDE 或 PlatformIO 已安裝

### 6.2 依賴
- **Python 套件**：
  - `pygame`：遙控器輸入
  - `pyserial`：Serial 通訊
- **Arduino 函式庫**：
  - `SoftwareSerial`：UART 通訊
- **工具**：
  - `jstest`：測試遙控器
  - `minicom` / `screen`：Serial 除錯

---

## 7. 風險評估

| 風險 | 可能性 | 影響 | 應對策略 |
|------|--------|------|----------|
| Serial 通訊掉包 | 中 | 高 | 實作 Checksum 驗證、重傳機制 |
| 馬達電流過載 | 低 | 高 | 加裝保險絲、監控電流 |
| 遙控器不相容 | 中 | 中 | 提供軸編號測試工具、支援多種遙控器 |
| 超聲波誤判 | 高 | 低 | 多次採樣平均、設定合理閾值 |
| Arduino 記憶體不足 | 低 | 中 | 優化程式碼、移除不必要變數 |

---

## 8. 參考文件

- [SYSTEM_ARCHITECTURE.md](../SYSTEM_ARCHITECTURE.md) - 系統架構與接線圖
- [REMOTE_CONTROL_ARCHITECTURE.md](../REMOTE_CONTROL_ARCHITECTURE.md) - 遙控方案設計
- [PROJECT_SUMMARY.md](../PROJECT_SUMMARY.md) - 專案總覽

---

**文件結束**
