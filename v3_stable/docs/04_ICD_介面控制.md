# V3 自走吸塵車 - 介面控制文件 (ICD)

**版本**: 2.0
**日期**: 2025-12-07
**關聯文件**: 01_SRS, 02_SA, 03_SD

---

## 1. 文件範圍

本文件定義 V3 自走吸塵車系統中所有硬體和軟體介面的規格。

---

## 2. 硬體介面

### 2.1 Arduino Uno 腳位分配

#### 2.1.1 總覽

| 腳位 | 類型 | 功能 | 模組 |
|------|------|------|------|
| D2 | DO | 右後超聲波 TRIG | HC-SR04 (新增) |
| D3 | PWM | PIN_ENA (左輪速度) | L298N |
| D4 | DI | 右後超聲波 ECHO | HC-SR04 (新增) |
| D5 | DO | PIN_IN2 (左輪方向 B) | L298N |
| D6 | DO | PIN_IN1 (左輪方向 A) | L298N |
| D7 | DO | 前方超聲波 TRIG | HC-SR04 |
| D8 | DI | 前方超聲波 ECHO | HC-SR04 |
| D9 | DO | PIN_IN4 (右輪方向 B) | L298N |
| D10 | DO | PIN_IN3 (右輪方向 A) | L298N |
| D11 | PWM | PIN_ENB (右輪速度) | L298N |
| A1 | DO | 右前超聲波 TRIG | HC-SR04 |
| A2 | DI | 右前超聲波 ECHO | HC-SR04 |
| A3 | DO | 吸塵器繼電器 | Relay |
| RX (0) | - | 保留 (USB 除錯) | Serial |
| TX (1) | - | 保留 (USB 除錯) | Serial |

#### 2.1.2 腳位圖

```
                    ┌─────────────────┐
                    │   Arduino Uno   │
                    │                 │
        Serial ───▶ │ RX  ○     ○ TX │ ◀─── Serial
                    │                 │
RIGHT_R_TRIG ◀──────│ D2  ○     ○ D3 │ ───▶ PIN_ENA (左輪 PWM)
RIGHT_R_ECHO ◀──────│ D4  ○     ○ D5 │ ───▶ PIN_IN2
  PIN_IN1 ◀─────────│ D6  ○     ○ D7 │ ───▶ FRONT_TRIG
  FRONT_ECHO ◀──────│ D8  ○     ○ D9 │ ───▶ PIN_IN4
  PIN_IN3 ◀─────────│ D10 ○     ○ D11│ ───▶ PIN_ENB (右輪 PWM)
                    │ D12 ○     ○ D13│
                    │                 │
                    │ A0  ○     ○ A1 │ ───▶ RIGHT_F_TRIG
RIGHT_F_ECHO ◀──────│ A2  ○     ○ A3 │ ───▶ VACUUM
                    │ A4  ○     ○ A5 │
                    │                 │
                    └─────────────────┘
```

### 2.2 L298N 馬達驅動連接

#### 2.2.1 接線表

| L298N 端子 | 連接 | 說明 |
|------------|------|------|
| 12V | 電池 + | 7.4-12V 電源 |
| GND | 共地 | Arduino GND + 電池 - |
| 5V | - | 不使用 (外接 5V 給 Arduino) |
| IN1 | Arduino D6 | 左輪方向 A |
| IN2 | Arduino D5 | 左輪方向 B |
| IN3 | Arduino D10 | 右輪方向 A |
| IN4 | Arduino D9 | 右輪方向 B |
| ENA | Arduino D3 | 左輪 PWM (移除 Jumper) |
| ENB | Arduino D11 | 右輪 PWM (移除 Jumper) |
| OUT1 | 左馬達 + | |
| OUT2 | 左馬達 - | |
| OUT3 | 右馬達 + | |
| OUT4 | 右馬達 - | |

#### 2.2.2 馬達控制邏輯

**左輪**：

| IN1 (D6) | IN2 (D5) | ENA (D3) | 動作 |
|----------|----------|----------|------|
| HIGH | LOW | PWM | 前進 |
| LOW | HIGH | PWM | 後退 |
| LOW | LOW | X | 停止 |
| HIGH | HIGH | X | 煞車 |

**右輪**：

| IN3 (D10) | IN4 (D9) | ENB (D11) | 動作 |
|-----------|----------|-----------|------|
| HIGH | LOW | PWM | 前進 |
| LOW | HIGH | PWM | 後退 |
| LOW | LOW | X | 停止 |
| HIGH | HIGH | X | 煞車 |

#### 2.2.3 接線圖

```
              ┌─────────────────────────────┐
              │          L298N              │
              │                             │
  電池 +  ───▶│ 12V              OUT1 OUT2 │───▶ 左馬達
  共地    ───▶│ GND              OUT3 OUT4 │───▶ 右馬達
              │ 5V (不接)                   │
              │                             │
  D6 ────────▶│ IN1          ENA  ENB      │◀─── D3 (PWM)
  D5 ────────▶│ IN2                        │◀─── D11 (PWM)
  D10 ───────▶│ IN3          [移除 Jumper] │
  D9 ────────▶│ IN4                        │
              │                             │
              └─────────────────────────────┘
```

### 2.3 HC-SR04 超聲波連接

#### 2.3.1 接線表

| 感測器 | VCC | GND | TRIG | ECHO | 位置說明 |
|--------|-----|-----|------|------|----------|
| 前方 | 5V | GND | D7 | D8 | 車頭正前方 |
| 右前 | 5V | GND | A1 | A2 | 車身右側前端 |
| 右後 | 5V | GND | D2 | D4 | 車身右側後端 (新增) |

#### 2.3.2 雙右側超音波安裝

```
俯視圖：
                        牆壁
    ─────────────────────────────────────

             d_front         d_rear
               ↓               ↓
    ┌──────────●───────────────●──────┐
    │       右前 US          右後 US   │
    │          ←───── L ─────→        │
    │         (A1,A2)       (D2,D4)    │
    │                                  │
    │            車身                  │
    │                                  │
    └──────────────────────────────────┘
              ↑
         前方 US (D7,D8)
```

**安裝要求**：
- 兩個右側超音波間距 L = 15cm (需精確測量並設定 US_SPACING)
- 兩者安裝高度需一致
- 感測方向需垂直於車身

#### 2.3.3 時序規格

```
TRIG:   ────┐     ┌────────────────────
            │10μs │
        ────┘     └────

ECHO:   ────────────┐              ┌────
                    │  duration    │
        ────────────┘              └────

距離 (cm) = duration (μs) / 58
超時 = 8000 μs (~136 cm)
```

#### 2.3.4 輪流讀取時序

由於多個超音波同時發射會互相干擾，採用輪流讀取策略：

```
迴圈 1: 讀取前方超音波
        │
迴圈 2: 讀取右前超音波
        │
迴圈 3: 讀取右後超音波
        │
迴圈 4: 讀取前方超音波 (循環)
        ...
```

每個迴圈 50ms，每個感測器更新週期 = 150ms (約 6.7Hz)

### 2.4 吸塵器繼電器連接

| 端子 | 連接 |
|------|------|
| VCC | 5V |
| GND | GND |
| IN | Arduino A3 |
| COM | 吸塵器電源 + |
| NO | 電池 + |

**控制邏輯**：
- A3 = HIGH → 繼電器吸合 → 吸塵器開啟
- A3 = LOW → 繼電器釋放 → 吸塵器關閉

### 2.5 Raspberry Pi 連接

#### 2.5.1 Pi ↔ Arduino 連接

| Pi 腳位 | Arduino 腳位 | 說明 |
|---------|--------------|------|
| GPIO14 (TXD) | RX (D0) | Pi TX → Arduino RX |
| GPIO15 (RXD) | TX (D1) | Pi RX ← Arduino TX |
| GND | GND | 共地 |

**注意**: 需透過電壓轉換器 (3.3V ↔ 5V) 或直接使用 USB 連接。

#### 2.5.2 Pi Camera 連接

- 使用 CSI 排線連接 Camera Module
- 確保排線藍色面朝向 PCB

---

## 3. 軟體介面

### 3.1 Pi → Arduino Serial 協定

#### 3.1.1 通訊參數

| 參數 | 數值 |
|------|------|
| 鮑率 | 115200 |
| 資料位元 | 8 |
| 停止位元 | 1 |
| 同位檢查 | 無 |
| 流量控制 | 無 |

#### 3.1.2 封包格式

```
┌────────┬─────────┬──────────┬────────┐
│ Header │ Command │ Checksum │ Footer │
│  0xAA  │  1 byte │  1 byte  │  0x55  │
└────────┴─────────┴──────────┴────────┘

Checksum = Header XOR Command
```

**總長度**: 4 bytes

#### 3.1.3 指令定義

| 指令 | 代碼 | 說明 |
|------|------|------|
| VACUUM_ON | 0x01 | 開啟吸塵器 |
| VACUUM_OFF | 0x02 | 關閉吸塵器 |

#### 3.1.4 封包範例

**開啟吸塵器**:
```
Header:   0xAA
Command:  0x01
Checksum: 0xAA ^ 0x01 = 0xAB
Footer:   0x55

封包: [0xAA, 0x01, 0xAB, 0x55]
```

**關閉吸塵器**:
```
Header:   0xAA
Command:  0x02
Checksum: 0xAA ^ 0x02 = 0xA8
Footer:   0x55

封包: [0xAA, 0x02, 0xA8, 0x55]
```

#### 3.1.5 錯誤處理

| 情況 | Arduino 行為 |
|------|--------------|
| Checksum 錯誤 | 丟棄封包 |
| Header 不符 | 繼續等待 Header |
| 封包不完整 | 重新等待 |

### 3.2 Arduino 內部介面

#### 3.2.1 模組間資料流

```
┌──────────────┐
│  ultrasonic  │
│  .getFiltered()  ───────────────────┐
└──────────────┘                      │
                                      ▼
                              ┌───────────────┐
                              │   behavior    │
                              │   .update()   │
                              └───────┬───────┘
                                      │
                              MotorCommand
                                      │
                                      ▼
                              ┌───────────────┐
                              │    motor      │
                              │    .set()     │
                              └───────────────┘
```

#### 3.2.2 資料結構

**MotorCommand**:
```cpp
struct MotorCommand {
    int leftPWM;    // -255 ~ +255
    int rightPWM;   // -255 ~ +255
    bool stop;      // true = 強制停止
};
```

**BehaviorOutput**:
```cpp
struct BehaviorOutput {
    float angular;  // -1.0 ~ +1.0
    float weight;   // 0.0 ~ 1.0
};
```

### 3.3 Raspberry Pi 內部介面

#### 3.3.1 模組間介面

```python
# RedDetector → V3Controller
detector.get_status() -> (detected: bool, area: int)

# SerialComm
serial.vacuum_on() -> bool
serial.vacuum_off() -> bool
```

---

## 4. 電氣規格

### 4.1 電源需求

| 元件 | 電壓 | 電流 (典型) | 電流 (峰值) |
|------|------|-------------|-------------|
| Arduino Uno | 5V | 50mA | 200mA |
| Raspberry Pi 4 | 5V | 600mA | 1.5A |
| L298N | 7-12V | - | - |
| DC 馬達 (x2) | 7-12V | 200mA | 1A |
| HC-SR04 (x2) | 5V | 30mA | 30mA |
| 吸塵器 | 12V | 500mA | 1A |

### 4.2 接地規則

所有模組必須共地：
- Arduino GND
- Raspberry Pi GND
- L298N GND
- 電池 GND
- 超聲波 GND
- 繼電器 GND

```
        電池 -
           │
           ├───▶ Arduino GND
           │
           ├───▶ L298N GND
           │
           ├───▶ Pi GND
           │
           └───▶ 所有感測器 GND
```

---

## 5. 時序約束

### 5.1 系統時序

| 事件 | 時間約束 |
|------|----------|
| Arduino 主迴圈 | 50ms (20Hz) |
| 超聲波單次讀取 | < 8ms |
| 紅色偵測迴圈 | 100ms (10Hz) |
| Serial 封包處理 | < 1ms |

### 5.2 啟動序列

```
t=0      上電
         │
t=0~3s   延遲等待 (避免 EMI)
         │
t=3s     初始化感測器
         │
t=3.1s   開啟吸塵器
         │
t=3.2s   開始主迴圈
```

---

## 6. 附錄

### 6.1 零件清單

| 零件 | 數量 | 規格 |
|------|------|------|
| Arduino Uno | 1 | R3 或相容板 |
| Raspberry Pi | 1 | Model 4B |
| L298N | 1 | 雙 H 橋驅動 |
| HC-SR04 | 3 | 超聲波模組 (前方 1 + 右側 2) |
| TT 馬達 | 2 | 3-6V DC |
| 繼電器模組 | 1 | 5V 單路 |
| Pi Camera | 1 | V2 或 V3 |
| 電池 | 1 | 11.1V 3S LiPo |

### 6.2 線材顏色建議

| 顏色 | 用途 |
|------|------|
| 紅 | 電源正極 |
| 黑 | 接地 |
| 黃 | 信號線 |
| 藍 | Serial TX |
| 綠 | Serial RX |

---

## 附錄 B：修訂歷史

| 版本 | 日期 | 變更說明 |
|------|------|----------|
| 1.0 | 2025-12-06 | 初版 |
| 2.0 | 2025-12-07 | 新增右後超音波 (D2/D4)，更新腳位圖和零件清單 |
