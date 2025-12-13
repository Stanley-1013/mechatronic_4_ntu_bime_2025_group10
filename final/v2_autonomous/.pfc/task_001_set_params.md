# Task 001: CMD_SET_PARAMS 實作

## 任務目標
實作 `CMD_SET_PARAMS` 指令，讓 Pi 可以在開機時從 config.py 讀取所有沿牆控制參數，一次性發送給 Arduino。

## 需求分析
- **動機**: 減少 Arduino 重新上傳次數，所有調參只需改 Pi 端 config.py
- **範圍**: 僅修改 `/home/han/claude_project/mechtronic_4/v2_autonomous/` 內的檔案
- **相容性**: 保持現有 CMD_SET_PID 不變，增加更全面的 CMD_SET_PARAMS

## 參數清單

根據 `/home/han/claude_project/mechtronic_4/v2_autonomous/arduino/main/config.h` (行 65-95)，需傳輸的參數：

### A. 距離閾值 (4 個 uint8_t, 單位: cm)
1. TARGET_RIGHT_DIST = 15
2. FRONT_STOP_DIST = 20
3. FRONT_SLOW_DIST = 40
4. CORNER_RIGHT_DIST = 30

### B. 速度參數 (5 個 float, 傳輸用 int16_t * 100)
1. BASE_LINEAR_SPEED = 0.6
2. BACKUP_SPEED = -0.4
3. TURN_ANGULAR_SPEED = 0.7
4. FIND_WALL_LINEAR = 0.6
5. FIND_WALL_ANGULAR = -0.05

### C. PID 參數 (3 個 float, 傳輸用 uint16_t * 1000)
**注意**: 已有 CMD_SET_PID (0x06)，本任務保持現有機制，但 CMD_SET_PARAMS 也包含 PID

1. DEFAULT_KP = 0.025
2. DEFAULT_KI = 0.002
3. DEFAULT_KD = 0.010

### D. 其他參數 (4 個)
1. MIN_EFFECTIVE_PWM (uint8_t) = 45
2. CORNER_TURN_ANGLE (uint8_t, 度) = 85
3. RED_AVOID_ANGLE (uint8_t, 度) = 45
4. BACKUP_DURATION_MS (uint16_t, ms) = 300
5. TURN_TIMEOUT_MS (uint16_t, ms) = 3000

## Payload 設計

### CMD_SET_PARAMS (0x07)

採用 **固定長度 payload** (32 bytes)，方便解析：

```
Offset | Size | Name                    | Type    | Multiplier | Range
-------+------+-------------------------+---------+------------+-----------
  0    | 1    | TARGET_RIGHT_DIST       | uint8   | 1          | 0-255 cm
  1    | 1    | FRONT_STOP_DIST         | uint8   | 1          | 0-255 cm
  2    | 1    | FRONT_SLOW_DIST         | uint8   | 1          | 0-255 cm
  3    | 1    | CORNER_RIGHT_DIST       | uint8   | 1          | 0-255 cm
-------+------+-------------------------+---------+------------+-----------
  4    | 2    | BASE_LINEAR_SPEED       | int16   | 100        | -327~327
  6    | 2    | BACKUP_SPEED            | int16   | 100        | -327~327
  8    | 2    | TURN_ANGULAR_SPEED      | int16   | 100        | -327~327
 10    | 2    | FIND_WALL_LINEAR        | int16   | 100        | -327~327
 12    | 2    | FIND_WALL_ANGULAR       | int16   | 100        | -327~327
-------+------+-------------------------+---------+------------+-----------
 14    | 2    | DEFAULT_KP              | uint16  | 1000       | 0-65.535
 16    | 2    | DEFAULT_KI              | uint16  | 1000       | 0-65.535
 18    | 2    | DEFAULT_KD              | uint16  | 1000       | 0-65.535
-------+------+-------------------------+---------+------------+-----------
 20    | 1    | MIN_EFFECTIVE_PWM       | uint8   | 1          | 0-255
 21    | 1    | CORNER_TURN_ANGLE       | uint8   | 1          | 0-255°
 22    | 1    | RED_AVOID_ANGLE         | uint8   | 1          | 0-255°
 23    | 1    | (padding)               | uint8   | -          | 0x00
 24    | 2    | BACKUP_DURATION_MS      | uint16  | 1          | 0-65535
 26    | 2    | TURN_TIMEOUT_MS         | uint16  | 1          | 0-65535
-------+------+-------------------------+---------+------------+-----------
 28    | 4    | (reserved)              | uint8[] | -          | 0x00
-------+------+-------------------------+---------+------------+-----------
Total: 32 bytes
```

### 完整封包格式

```
[0xAA][0x07][0x20][...32 bytes payload...][checksum][0x55]
Total length: 5 + 32 = 37 bytes
```

### 數據範圍驗證

- **距離**: 0-255 cm (uint8)
- **速度**: -3.27 ~ +3.27 (int16 * 100)
- **PID**: 0 ~ 65.535 (uint16 * 1000)
- **角度**: 0-255° (uint8)
- **時間**: 0-65535 ms (uint16)

## 架構分析

### 修改檔案清單

#### Arduino 端 (4 個檔案)
1. `/home/han/claude_project/mechtronic_4/v2_autonomous/arduino/main/protocol.h`
   - 新增 `#define CMD_SET_PARAMS 0x07`

2. `/home/han/claude_project/mechtronic_4/v2_autonomous/arduino/main/config.h`
   - 將靜態 `#define` 改為可修改的全域變數 (extern)

3. `/home/han/claude_project/mechtronic_4/v2_autonomous/arduino/main/wall_follower.h`
   - 新增 `void setParams(...)` 成員函數

4. `/home/han/claude_project/mechtronic_4/v2_autonomous/arduino/main/wall_follower.cpp`
   - 實作 `setParams(...)` 函數
   - 更新全域參數變數

5. `/home/han/claude_project/mechtronic_4/v2_autonomous/arduino/main/main.ino`
   - 在指令處理器增加 `CMD_SET_PARAMS` 分支

#### Pi 端 (3 個檔案)
1. `/home/han/claude_project/mechtronic_4/v2_autonomous/raspberry_pi/protocol.py`
   - 新增 `CMD_SET_PARAMS = 0x07`
   - 新增 `create_cmd_set_params(...)` 函數

2. `/home/han/claude_project/mechtronic_4/v2_autonomous/raspberry_pi/config.py` (新建)
   - 定義所有可調參數
   - 提供 `get_all_params()` 函數

3. `/home/han/claude_project/mechtronic_4/v2_autonomous/raspberry_pi/controller.py`
   - 新增 `send_set_params()` 方法
   - 在 `start()` 時自動發送參數

## 子任務分解

### 階段 1: Arduino 基礎架構 (3 個子任務)

#### Task 1.1: 更新 protocol.h
- 新增 `CMD_SET_PARAMS` 定義
- 優先級: 9 (基礎)

#### Task 1.2: 改造 config.h 為可修改變數
- 將距離/速度/角度/時間參數改為 extern 全域變數
- 保持 `#define` 作為預設值
- 優先級: 9 (關鍵)
- 依賴: 1.1

#### Task 1.3: 實作 WallFollower::setParams()
- 在 wall_follower.h 宣告
- 在 wall_follower.cpp 實作
- 接收解析後的參數，更新全域變數
- 優先級: 8
- 依賴: 1.2

### 階段 2: Arduino 指令處理 (1 個子任務)

#### Task 2.1: main.ino 增加 CMD_SET_PARAMS 處理
- 解析 32 bytes payload
- 呼叫 wallFollower.setParams()
- 發送確認回應 (可選)
- 優先級: 8
- 依賴: 1.3

### 階段 3: Pi 端實作 (3 個子任務)

#### Task 3.1: 更新 protocol.py
- 新增 `CMD_SET_PARAMS` 常數
- 實作 `create_cmd_set_params()` 函數
- 實作 payload 序列化邏輯 (struct.pack)
- 優先級: 8
- 依賴: 無 (可並行)

#### Task 3.2: 創建 config.py
- 定義所有參數常數
- 實作 `get_all_params()` 函數返回 dict
- 優先級: 7
- 依賴: 無 (可並行)

#### Task 3.3: 整合到 controller.py
- 新增 `send_set_params()` 方法
- 在 `start()` 時讀取 config.py 並自動發送
- 優先級: 7
- 依賴: 3.1, 3.2

### 階段 4: 測試與驗證 (2 個子任務)

#### Task 4.1: 單元測試 (Pi 端)
- 測試 payload 序列化正確性
- 測試封包 checksum
- 驗證資料範圍
- 優先級: 6
- 依賴: 3.1

#### Task 4.2: 整合測試
- 上傳 Arduino 程式
- 執行 controller.py
- 驗證參數是否正確傳輸
- 修改 config.py 參數，驗證即時生效
- 優先級: 5
- 依賴: 2.1, 3.3

## 執行計畫

### 自動執行模式
使用 Executor agent 自動執行，每完成一個階段回報進度。

### 階段執行順序
1. 階段 1 (Arduino 基礎) - 3 個子任務依序執行
2. 階段 2 (Arduino 指令) - 1 個子任務
3. 階段 3 (Pi 端) - 3 個子任務可並行
4. 階段 4 (測試) - 2 個子任務

### Micro-Nap 觸發點
- 階段 2 完成後 (已處理 4 個子任務)
- 階段 4 完成前 (context 累積)

## 風險與注意事項

### 風險
1. **記憶體限制**: Arduino Uno RAM 只有 2KB，需確認全域變數不超限
2. **序列化錯誤**: 大小端問題 (Arduino 為小端，struct.pack 需使用 '<' 前綴)
3. **參數範圍**: Pi 端需驗證參數範圍，避免無效值

### 注意事項
1. **保持向後相容**: CMD_SET_PID (0x06) 必須繼續運作
2. **預設值保留**: config.h 的 #define 作為 fallback
3. **錯誤處理**: payload 長度不符時，Arduino 應忽略並回報錯誤

## 成功標準

1. Arduino 可接收並解析 CMD_SET_PARAMS 封包
2. Pi 端 config.py 修改參數後，無需重新上傳 Arduino 程式即可生效
3. 參數傳輸正確性驗證 (單元測試 + 整合測試)
4. 沿牆控制行為符合預期 (實車測試)

## 文檔更新

完成後需更新：
- `/home/han/claude_project/mechtronic_4/docs/04_ICD_介面規格.md` - 新增 CMD_SET_PARAMS 規格
- `/home/han/claude_project/mechtronic_4/v2_autonomous/README.md` - 新增使用說明

## 附錄：參考資料

### Python struct.pack 格式字串
```python
# Little-endian (Arduino Uno 為小端)
'<4B10h3H3BxHH4x'

解析:
  <    : 小端序
  4B   : 4 個 uint8 (距離閾值)
  10h  : 10 個 int16 (5 個速度 * 2 bytes)
  3H   : 3 個 uint16 (PID)
  3B   : 3 個 uint8 (PWM, 角度)
  x    : 1 byte padding
  HH   : 2 個 uint16 (時間)
  4x   : 4 bytes reserved
```

### Arduino 解析範例
```cpp
uint8_t target_right = payload[0];
int16_t base_linear_raw = (int16_t)(payload[4] | (payload[5] << 8));
float base_linear = base_linear_raw / 100.0f;
```

---

**任務建立時間**: 2025-12-06
**預估工時**: 2-3 小時 (含測試)
**優先級**: 8/10 (高優先)
