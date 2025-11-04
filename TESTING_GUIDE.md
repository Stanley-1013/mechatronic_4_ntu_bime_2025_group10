# 測試指南

> 循序漸進測試機電小車系統
> 版本: 1.0
> 日期: 2025-10-31

---

## 📋 測試流程總覽

```
Step 1: 測試遙控器 (5分鐘)
   └─→ 確認遙控器可被辨識，找出軸編號

Step 2: 上傳 Arduino 程式 (5分鐘)
   └─→ 確認 Arduino 程式正常運作

Step 3: 測試 Serial 通訊 (10分鐘)
   └─→ 確認 Pi 與 Arduino 可以通訊

Step 4: 測試馬達控制 (15分鐘)
   └─→ 確認馬達可以正常轉動

Step 5: 完整系統測試 (30分鐘)
   └─→ 使用遙控器控制小車
```

---

## 🎮 Step 1: 測試遙控器

### 目的
確認遙控器可被 Raspberry Pi 辨識，並找出正確的軸編號。

### 步驟

```bash
cd /home/han/claude_project/mechtronic_4/raspberry_pi

# 1. 檢查遙控器是否被辨識
ls /dev/input/js*

# 應顯示: /dev/input/js0

# 2. 使用系統工具測試
jstest /dev/input/js0

# 3. 使用測試腳本（更直觀）
python3 test_joystick.py
```

### 預期結果
- ✅ 可以看到遙控器名稱
- ✅ 移動搖桿時，Axis 數值會變化
- ✅ 按按鈕時，Button 編號會顯示

### 記錄資訊
```
遙控器名稱: _______________
左搖桿 Y (前後): Axis ___
左搖桿 X (左右): Axis ___
吸塵器按鈕: Button ___
```

### 如果有問題

**問題 1: 找不到 /dev/input/js0**
```bash
# 載入 joystick 模組
sudo modprobe joydev

# 檢查 USB 設備
lsusb

# 查看 kernel 訊息
dmesg | tail -20
```

**問題 2: 權限不足**
```bash
# 將使用者加入 input 群組
sudo usermod -a -G input $USER

# 登出後重新登入生效
```

---

## 🔌 Step 2: 上傳 Arduino 程式

### 目的
將 Arduino 程式上傳至 Arduino Uno。

### 步驟

#### 使用 Arduino IDE

1. 開啟 Arduino IDE
2. 開啟檔案: `arduino/main/main.ino`
3. 選擇板子: **工具 → 板子 → Arduino Uno**
4. 選擇序列埠: **工具 → 序列埠 → /dev/ttyUSB0** (或 /dev/ttyACM0)
5. 上傳程式: **草稿碼 → 上傳** (或按 Ctrl+U)

#### 使用 arduino-cli

```bash
cd /home/han/claude_project/mechtronic_4

# 編譯
arduino-cli compile --fqbn arduino:avr:uno arduino/main/

# 上傳
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno arduino/main/
```

### 驗證上傳成功

```bash
# 開啟 Serial Monitor (Arduino IDE)
# 或使用 screen
screen /dev/ttyUSB0 115200

# 應該會看到:
=========================================
 Arduino Robot Controller v1.1
=========================================
Initializing...
[OK] Serial @ 57600 bps
[OK] Motor driver
[OK] Ultrasonic sensors
[OK] Vacuum controller

[Config]
  Sensor interval: 100 ms
  Command timeout: 200 ms

=========================================
 System Ready - Waiting for commands
=========================================
```

**離開 screen**: 按 `Ctrl+A` 再按 `K`，然後按 `Y`

### 如果有問題

**問題 1: 找不到序列埠**
```bash
# 列出所有序列埠
ls /dev/tty*

# 通常是 /dev/ttyUSB0 或 /dev/ttyACM0
```

**問題 2: 權限不足**
```bash
sudo usermod -a -G dialout $USER
# 登出後重新登入
```

---

## 🔗 Step 3: 測試 Serial 通訊

### 目的
確認 Raspberry Pi 與 Arduino 之間的 Serial 通訊正常。

### 前置作業

**檢查接線**:
```
Arduino D2 (TX) → Pi GPIO15 (RXD, Pin 10)
Arduino D4 (RX) → Pi GPIO14 (TXD, Pin 8)
Arduino GND → Pi GND (Pin 6, 9, 14 任一)
```

**⚠️ 重要**: 確認 Pi 的 UART 已啟用
```bash
# 檢查 /dev/serial0 是否存在
ls -l /dev/serial0

# 如果不存在，執行:
sudo raspi-config
# → Interface Options → Serial Port
# → Login shell over serial: No
# → Serial port hardware: Yes
sudo reboot
```

### 測試步驟

```bash
cd /home/han/claude_project/mechtronic_4/raspberry_pi

# 執行測試程式
python3 test_motor_only.py
```

### 預期結果

```
========== 馬達獨立測試程式 ==========

[1/1] 連接 Arduino...
✅ Serial 連接成功: /dev/serial0 @ 57600 bps

開始測試序列...

[測試 1/12] 前進全速
  指令: MotorCommand(left=+255, right=+255, vacuum=False)
  時間: 2.0s | 感測器: SensorData(left= 25cm ✓, right= 30cm ✓)

[測試 2/12] 停止
...
```

### 如果有問題

**問題 1: Serial 連接失敗**
```bash
# 檢查 Serial 是否被其他程式佔用
sudo lsof /dev/serial0

# 檢查權限
ls -l /dev/serial0
```

**問題 2: 沒有收到感測器資料**
- 這是正常的，因為還沒接超聲波
- 應該會顯示 `SensorData(left=999cm ✗, right=999cm ✗)`

---

## 🚗 Step 4: 測試馬達控制

### 目的
確認馬達可以正常轉動，並調整方向/速度。

### 準備工作

1. **⚠️ 安全檢查**:
   - 將小車架高（輪子離地）
   - 或確保前方有足夠空間
   - 準備好隨時按 Ctrl+C 停止

2. **電源檢查**:
   - 電池電壓: 9-12V (3S 18650 = 11.1V)
   - L298N 跳線帽已拔除（ENA/ENB）
   - 所有 GND 已連接

### 測試步驟

**方法 1: 使用獨立測試腳本（推薦）**

```bash
cd raspberry_pi
python3 test_motor_only.py
```

觀察：
- ✅ 前進時，兩輪應同向轉動
- ✅ 後退時，兩輪應反向轉動
- ✅ 左轉時，左輪反轉、右輪前進
- ✅ 右轉時，左輪前進、右輪反轉

**方法 2: 使用 Python 互動模式**

```python
python3 -i arduino_controller.py

# 測試前進
>>> from differential_drive import MotorCommand
>>> cmd = MotorCommand(255, 255, False)
>>> controller.send_command(cmd)

# 停止
>>> cmd = MotorCommand(0, 0, False)
>>> controller.send_command(cmd)
```

### 常見問題處理

**問題 1: 馬達不轉**
- 檢查 L298N 接線
- 檢查電池電壓
- 檢查 L298N 跳線帽是否拔除
- 開啟 Arduino Serial Monitor 查看是否收到指令

**問題 2: 馬達轉向相反**
- 解決方法 1: 交換馬達接線（OUT1 ↔ OUT2）
- 解決方法 2: 修改程式碼（在 motor_driver.cpp 中反轉邏輯）

**問題 3: 左右輪速度不一致**
```bash
# 修改 config.py 調整校準係數
# 例如左輪較慢：
MOTOR_LEFT_SCALE = 1.1
MOTOR_RIGHT_SCALE = 1.0
```

---

## 🎮 Step 5: 完整系統測試

### 目的
使用遙控器控制小車，驗證所有功能。

### 步驟

```bash
cd raspberry_pi

# 基本模式
python3 main.py

# 除錯模式（看更多資訊）
python3 main.py --debug

# 調整參數
python3 main.py --deadzone 0.15 --left-scale 1.1
```

### 測試項目清單

| 項目 | 測試方法 | 通過 ✓ |
|------|---------|-------|
| 前進 | 推搖桿向上 | ☐ |
| 後退 | 拉搖桿向下 | ☐ |
| 左轉 | 推搖桿向左 | ☐ |
| 右轉 | 推搖桿向右 | ☐ |
| 停止 | 搖桿歸零 | ☐ |
| 吸塵器 | 按 Button 0 | ☐ |
| 感測器 | 手放在超聲波前，距離顯示應改變 | ☐ |
| 緊急停止 | 按 Ctrl+C，馬達應立即停止 | ☐ |

### 預期畫面

```
==================================================
 機電小車控制系統 | 運行時間: 10.5s | 迴圈: 525
==================================================
[遙控器] 線性: +0.75 | 角速度: -0.20 | 吸塵器: ⚪ OFF
[馬達]   左輪: +140 | 右輪: +240
[感測器] 左側: 🟢  25cm | 右側: 🟢  30cm
[Serial] TX: 525 | RX: 52 | 錯誤: 0+0 | 成功率: 100.0%
==================================================
```

---

## 📊 問題診斷

### 控制延遲過大

**症狀**: 操作搖桿後，馬達反應明顯延遲

**可能原因**:
1. Serial 錯誤率高 → 檢查 Serial 統計，成功率應 > 99%
2. CPU 負載過高 → 降低頻率 `--frequency 30`
3. 搖桿讀取問題 → 檢查 pygame 版本

### 馬達抖動

**症狀**: 馬達轉動不順，有抖動

**可能原因**:
1. PWM 頻率太低 → 修改 `motor_driver.cpp` 提升頻率
2. 電壓不穩 → 檢查電池電壓
3. 接線鬆動 → 重新檢查接線

### 超聲波讀值不穩定

**症狀**: 距離顯示忽大忽小

**可能原因**:
1. 安裝角度不對 → 調整為 45° 朝外
2. 兩側干擾 → 已有 10ms 延遲，應已避免
3. 環境因素 → 避免測量透明物體或鏡面

---

## 📝 測試記錄表

```
測試日期: __________
測試人員: __________

[ ] Step 1: 遙控器測試
    遙控器型號: __________
    軸編號: Linear=__, Angular=__, Button=__

[ ] Step 2: Arduino 上傳
    上傳成功時間: __________

[ ] Step 3: Serial 通訊
    成功率: _____% (應 > 99%)

[ ] Step 4: 馬達控制
    [ ] 前進正常
    [ ] 後退正常
    [ ] 左轉正常
    [ ] 右轉正常
    [ ] 校準係數: L=____, R=____

[ ] Step 5: 完整系統
    [ ] 遙控器控制正常
    [ ] 感測器讀值正常
    [ ] 無異常錯誤

備註:
_________________________________
_________________________________
_________________________________
```

---

**測試指南版本**: 1.0
**最後更新**: 2025-10-31
