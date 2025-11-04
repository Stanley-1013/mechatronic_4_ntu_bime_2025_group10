# 機電小車專題總結

## 📋 專題資訊

**專題名稱：** 自主導航機器人
**硬體平台：** Raspberry Pi 4 + Arduino Uno
**階段目標：**
- **階段 1**：遙控模式（手動駕駛）
- **階段 2**：視覺自走模式（自主導航）

## 🎯 系統配置確認

### 硬體清單

| 元件 | 型號/規格 | 數量 | 用途 |
|------|----------|------|------|
| 主控制器 | Raspberry Pi 4 | 1 | 高階決策與視覺處理 |
| 微控制器 | Arduino Uno | 1 | 即時馬達/感測器控制 |
| 馬達驅動 | L298N H-Bridge | 1 | 雙馬達 PWM 控制 |
| 直流馬達 | TT 馬達（無編碼器） | 2 | 差動驅動 |
| 超聲波感測器 | HC-SR04 | 2 | 左右側障礙偵測 |
| 相機 | Pi Camera V2/V3 | 1 | 前方視覺導航 |
| 電池 | 18650 鋰電池 3S | 3 | 11.1V 電源（9-12.6V） |
| 降壓模組 | DC-DC 降壓 5V/3A | 1 | 供電給 Pi + Arduino |
| 遙控模組 | 2.4G USB 遙控器 | 1 組 | 無線遙控（即插即用） |

### 關鍵配置

- ✅ **馬達**：開環控制（無編碼器）
- ✅ **電源**：3S 18650 (11.1V)
- ✅ **超聲波**：左右兩側 45° 安裝
- ✅ **相機**：前方視覺導航
- ✅ **通訊**：二進位 Serial 協定
- ✅ **遙控**：2.4G USB 接收器（即插即用），備案藍牙/Web UI

## 📁 文檔結構

```
mechtronic_4/
├── SYSTEM_ARCHITECTURE.md          # 系統架構與接線圖
├── REMOTE_CONTROL_ARCHITECTURE.md  # 遙控方案設計
├── PROJECT_SUMMARY.md              # 本文檔
├── INSTALLATION.md                 # Skill 安裝說明
│
├── output/
│   └── mechtronic-robotics/        # 已安裝的 Claude Skill
│       ├── SKILL.md
│       ├── README.md
│       └── references/
│           ├── motor_control.md
│           ├── sensor_integration.md
│           └── computer_vision_slam.md
│
└── ~/.claude/skills/
    └── mechtronic-robotics/         # Claude Code 已載入
```

## 🔌 接線總覽

### L298N → Arduino

| L298N | Arduino | 功能 |
|-------|---------|------|
| IN1 | D5 | 左輪方向 A |
| IN2 | D6 | 左輪方向 B |
| IN3 | D9 | 右輪方向 A |
| IN4 | D10 | 右輪方向 B |
| ENA | D3 (PWM) | 左輪速度 |
| ENB | D11 (PWM) | 右輪速度 |
| +12V | 電池 11.1V | 馬達電源 |
| GND | GND | **共地** |

### 超聲波 → Arduino

**左側 (HC-SR04 #1):**
- Trig → D7
- Echo → D8

**右側 (HC-SR04 #2):**
- Trig → A1
- Echo → A2

### Arduino ↔ Raspberry Pi

- Arduino TX (D2) → Pi RXD (GPIO15)
- Arduino RX (D4) → Pi TXD (GPIO14)
- GND → GND (**共地**)

### 遙控模組 (2.4G USB)

**連接方式：**
```
2.4G 遙控器 → 2.4GHz 無線 → USB 接收器 → Raspberry Pi USB 埠
```

**優勢：**
- ✅ 即插即用（自動識別為 `/dev/input/js0`）
- ✅ 無需焊接與接線
- ✅ 使用 pygame 函式庫讀取
- ✅ 低延遲 < 20ms

### 電源架構

```
18650 電池 (3S = 11.1V)
    ├─→ L298N 馬達驅動 (11.1V 直供)
    └─→ DC-DC 降壓模組
        ├─→ Arduino Uno (5V)
        └─→ Raspberry Pi 4 (5V/3A)
```

**⚠️ 重要：** 所有 GND 必須連接在一起（共地）

## 🎮 遙控方案

### 階段 1 優先：2.4G 遙控器

**優勢：**
- ✅ 低延遲 (< 20ms)
- ✅ 穩定可靠
- ✅ 距離遠 (100m+)
- ✅ 無配對困擾

**實作：**
- 遙控器端：Arduino Nano + NRF24L01 + 搖桿
- 接收端：Raspberry Pi + NRF24L01
- 通訊：二進位封包（8 bytes）

### 備案 A：藍牙搖桿

- PS4 / Xbox 控制器
- Pi 內建藍牙
- 延遲 30-50ms

### 備案 B：手機 Web UI

- Flask Web 伺服器
- 手機瀏覽器控制
- WiFi 連線

### 統一介面設計

所有遙控方式輸出**統一格式**：

```python
class VehicleCommand:
    linear_velocity: float   # -1.0 ~ 1.0 (前進/後退)
    angular_velocity: float  # -1.0 ~ 1.0 (左轉/右轉)
    mode: str                # "MANUAL" / "AUTO" / "STOP"
```

**切換遙控方式只需改變啟動參數：**

```bash
python3 robot_controller.py --mode 2.4g       # 2.4G 遙控
python3 robot_controller.py --mode bluetooth  # 藍牙搖桿
python3 robot_controller.py --mode web        # Web UI
```

## 📡 通訊協定

### Pi → Arduino 二進位封包

**8 Bytes 封包格式：**

| Byte | 內容 | 說明 |
|------|-----|------|
| 0 | 0xAA | Header |
| 1-2 | 左輪 PWM | -255 ~ 255 (int16) |
| 3-4 | 右輪 PWM | -255 ~ 255 (int16) |
| 5 | Flags | 控制旗標 |
| 6 | Checksum | XOR 校驗 |
| 7 | 0x55 | Footer |

**優點：**
- 高效率（50Hz 更新率）
- 低延遲
- 錯誤偵測

### Arduino → Pi 感測器資料

**8 Bytes 封包格式：**

| Byte | 內容 | 說明 |
|------|-----|------|
| 0 | 0xBB | Header |
| 1-2 | 左側距離 | cm (uint16) |
| 3-4 | 右側距離 | cm (uint16) |
| 5 | 狀態 | 保留 |
| 6 | Checksum | XOR 校驗 |
| 7 | 0x66 | Footer |

## 🚀 開發階段規劃

### Phase 1：基礎遙控（當前目標）

**目標：** 手動遙控車輛移動，驗證硬體功能

**任務清單：**
- [ ] 完成電路焊接與組裝
- [ ] Arduino 馬達控制程式（L298N）
- [ ] Arduino 超聲波讀取程式
- [ ] 二進位 Serial 通訊實作（Pi ↔ Arduino）
- [ ] ~~2.4G 遙控器製作（已改用 USB 遙控器）~~
- [ ] 安裝 pygame 並測試 USB 遙控器（`jstest`）
- [ ] Raspberry Pi 遙控接收程式（pygame）
- [ ] 統一控制 API 整合
- [ ] 測試與調校

**驗收標準：**
- ✅ 馬達可前進/後退/左轉/右轉
- ✅ 超聲波可讀取距離
- ✅ 遙控延遲 < 50ms
- ✅ 可切換備案遙控方式

### Phase 2：視覺自走（進階目標）

**目標：** 基於視覺的自主導航

**任務清單：**
- [ ] Pi Camera 影像擷取
- [ ] OpenCV 路徑偵測演算法
- [ ] 超聲波 + 視覺融合
- [ ] 自主決策邏輯
- [ ] 障礙迴避策略
- [ ] 測試與優化

**驗收標準：**
- ✅ 可辨識路徑並追蹤
- ✅ 遇障礙物自動迴避
- ✅ 可完成指定路線

### Phase 3：進階功能（選配）

- [ ] IMU 姿態估計
- [ ] 速度校準（無編碼器補償）
- [ ] ORB-SLAM2 視覺 SLAM
- [ ] ROS2 導航堆疊
- [ ] 地圖建構與路徑規劃

## 🛠️ 開發工具與環境

### Raspberry Pi 軟體

- **OS**: Raspberry Pi OS (Bookworm)
- **Python**: 3.11+
- **函式庫**:
  - `pyserial` - Serial 通訊
  - `RF24` - NRF24L01 驅動
  - `picamera2` - 相機介面
  - `opencv-python` - 影像處理
  - `flask` - Web 伺服器（備案）
  - `pygame` - 藍牙搖桿（備案）

### Arduino 軟體

- **IDE**: Arduino IDE 2.x
- **板子**: Arduino Uno
- **函式庫**:
  - `RF24` - NRF24L01 發射器
  - 標準函式庫（無需額外安裝）

### 開發環境

```bash
# Raspberry Pi 設定
sudo raspi-config
# 啟用 Camera, Serial, SPI, I2C

# 安裝 Python 套件
pip3 install pyserial opencv-python picamera2

# 安裝 RF24 函式庫
git clone https://github.com/nRF24/RF24.git
cd RF24
./configure
make
sudo make install
```

## 📚 參考資源

### 已安裝 Claude Skill

Claude Code 已載入 `mechtronic-robotics` skill，可直接詢問：
- L298N 接線與程式碼
- HC-SR04 超聲波使用
- Serial 通訊協定
- OpenCV 影像處理
- 等等

**使用方式：** 直接向 Claude Code 提問相關問題即可自動啟用

### 文檔位置

- [系統架構](SYSTEM_ARCHITECTURE.md) - 完整接線圖與說明
- [遙控方案](REMOTE_CONTROL_ARCHITECTURE.md) - 三種遙控方式實作
- [Skill 文檔](~/.claude/skills/mechtronic-robotics/) - 技術參考手冊

### 外部資源

- Arduino Forum: https://forum.arduino.cc
- Raspberry Pi Forums: https://forums.raspberrypi.com
- RF24 Documentation: https://nrf24.github.io/RF24/

## ⚠️ 注意事項

### 電源相關

1. **共地連接**：所有 GND 必須連在一起
2. **電壓保護**：NRF24L01 只能 3.3V，不可接 5V
3. **電流容量**：Pi 4 需要 5V/3A，使用足夠的降壓模組
4. **電池監控**：定期檢查電池電壓，避免過放

### 馬達控制

1. **死區設定**：搖桿加入死區避免飄移
2. **PWM 頻率**：Arduino 預設 490Hz，可能造成抖動（可提升至 1kHz）
3. **方向測試**：先測試各馬達方向，必要時調整接線
4. **速度校準**：左右馬達可能不同速，需個別校準

### 通訊相關

1. **鮑率設定**：Pi 和 Arduino 須一致（建議 57600）
2. **校驗和驗證**：務必檢查封包完整性
3. **逾時處理**：超過 1 秒沒收到指令應自動停止
4. **緩衝區清空**：啟動時清空 Serial 緩衝區

### 感測器相關

1. **超聲波角度**：45° 朝外最佳，避免直角反射
2. **測距頻率**：避免兩側同時觸發（交替讀取）
3. **濾波處理**：連續讀取取中位數，避免雜訊

## 🎓 學習建議

### 開發順序

1. **Week 1-2**: 基礎電路與馬達控制
   - L298N 接線與測試
   - Arduino 馬達程式
   - 手動測試前進/後退/轉向

2. **Week 3-4**: 感測器與通訊
   - 超聲波讀取程式
   - Serial 二進位通訊
   - Pi-Arduino 資料交換

3. **Week 5-6**: 遙控系統
   - 2.4G 遙控器製作
   - 統一控制 API
   - 整合測試

4. **Week 7-8**: 視覺導航（進階）
   - Pi Camera 影像擷取
   - 路徑偵測演算法
   - 自主導航測試

### 除錯技巧

1. **分模組測試**：先獨立測試各模組再整合
2. **Serial Monitor**：善用 Arduino/Python 的 Serial 輸出
3. **LED 指示**：加入 LED 顯示狀態
4. **邏輯分析儀**：檢查 Serial/SPI 訊號（選配）

## 📞 支援

如有問題，可直接詢問 Claude Code：
- "L298N 馬達不轉怎麼辦？"
- "NRF24L01 無法通訊"
- "超聲波讀值不穩定"
- "Serial 通訊封包格式"

Claude Code 會自動使用已安裝的 mechatronics skill 提供專業建議。

---

**文檔版本：** 1.0
**最後更新：** 2025-10-31
**專題狀態：** 規劃階段 → 開發階段 Phase 1
