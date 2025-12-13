# V2 自走吸塵車 - 系統設計文件

> 最後更新：2025-12-07
> 版本：v2.2（IMU 航向控制已實現）

---

## 1. 系統概述

### 1.1 任務目標
- 在 300cm x 300cm 矩形場地沿右牆清掃
- 閃避隨機放置的紅色區域（關吸塵器）
- 從場外進場，手動停止結束

### 1.2 硬體架構
```
┌─────────────┐     UART 115200     ┌─────────────┐
│ Raspberry Pi │◄──────────────────►│ Arduino Uno │
│  - 視覺處理   │                     │  - 馬達控制  │
│  - 紅色偵測   │                     │  - 感測器    │
└─────────────┘                     └─────────────┘
```

### 1.3 感測器配置
| 感測器 | 位置 | 更新率 | 用途 |
|-------|------|-------|------|
| 超聲波 (前) | 車頭 | 10 Hz | 偵測前方牆壁、觸發轉彎 |
| 超聲波 (右) | 右側 | 10 Hz | 監督沿牆距離、修正漂移 |
| MPU6050 | 車身 | 50 Hz | **主控：航向鎖定** |
| Pi Camera | 車頭 | - | 紅色偵測 |

---

## 2. 控制架構

### 2.1 三層控制架構
```
┌────────────────────────────────────────────────────┐
│                    控制融合                         │
├────────────────────────────────────────────────────┤
│  高優先級：前方避障（超聲波 front < 20cm）           │
│  主控制層：IMU 航向 PID（保持直線）                  │
│  監督修正：超聲波距離（慢速修正 IMU 漂移）           │
└────────────────────────────────────────────────────┘
```

### 2.2 控制邏輯流程
```
每次 update():
  1. 檢查前方距離 → 觸發避障/轉彎
  2. IMU 航向 PID → 計算角速度修正
  3. 超聲波監督 → 慢速調整目標航向
  4. 融合輸出 → 設定馬達 PWM
```

### 2.3 實現狀態
- [x] 超聲波 Bang-Bang 控制（已實現，作為備援）
- [x] **IMU 航向 PID 控制**（✅ 2025-12-07 已實現）
- [x] **超聲波監督修正**（✅ 2025-12-07 已實現）
- [x] **轉彎後重置航向**（✅ 2025-12-07 已實現）
- [x] 紅色偵測關吸塵器（已實現）
- [ ] 入口進場邏輯（待實現）
- [x] dt 保護機制（✅ 2025-12-07 已實現）

---

## 3. 待實現功能

### 3.1 🔴 Phase 1：IMU 航向 PID 控制（必須）

**目的**：用 IMU 高頻率 (50Hz) 保持直線，取代超聲波低頻 (10Hz) 控制

**位置**：`wall_follower.cpp` → `update()` 函數

**邏輯**：
```cpp
// 1. 首次進入時鎖定當前航向
if (!_yawLocked && imuValid) {
    _targetYaw = yaw;
    _yawLocked = true;
}

// 2. 計算航向誤差（正規化到 [-180, 180]）
float yawError = _normalizeAngle(yaw - _targetYaw);

// 3. PID 控制輸出角速度修正（僅在 IMU 有效且非轉彎時）
if (imuValid && _yawLocked && obstacleWeight < 0.5) {
    float yawCorrection = g_yawKp * yawError;
    angular += yawCorrection * (1.0 - obstacleWeight);
}
```

**參數**（經 Critic 修正）：
| 參數 | 建議值 | 說明 |
|-----|-------|------|
| **YAW_KP** | **0.35** | 1° 偏差 → 0.35 角速度修正 |
| YAW_KI | 0.0 | 暫不使用積分 |
| YAW_KD | 0.08 | 抑制震盪 |

**輔助函數**：
```cpp
float WallFollower::_normalizeAngle(float angle) {
    angle = fmod(angle + 180.0, 360.0);
    if (angle < 0) angle += 360.0;
    return angle - 180.0;
}
```

**驗收標準**：
- 直行軌跡震盪 < ±5cm（目前 ±10-15cm）
- 無明顯 C 字形軌跡

---

### 3.2 🔴 Phase 2：超聲波監督修正（必須）

**目的**：防止 IMU 長期漂移導致偏離牆壁

**位置**：`wall_follower.cpp` → `update()` 函數

**邏輯**（經 Critic 修正）：
```cpp
// 超聲波監督：慢速修正 IMU 漂移
// 條件：非轉彎狀態、連續偏離超過 2 秒

bool isTurning = (abs(angular) > 0.3) || _cornerLocked;

if (!isTurning && imuValid && right >= 2 && right < 100) {
    if (right > TARGET_RIGHT_DIST + ERROR_DEADZONE) {
        _driftTimer += dt;
        if (_driftTimer > 2.0) {  // 連續 2 秒才修正
            _targetYaw -= YAW_DRIFT_RATE * dt;
        }
    } else if (right < TARGET_RIGHT_DIST - ERROR_DEADZONE) {
        _driftTimer += dt;
        if (_driftTimer > 2.0) {
            _targetYaw += YAW_DRIFT_RATE * dt;
        }
    } else {
        _driftTimer = 0;  // 在死區內，重置計時器
    }
}
```

**參數**（經 Critic 修正）：
| 參數 | 建議值 | 說明 |
|-----|-------|------|
| **YAW_DRIFT_RATE** | **0.15** | 度/秒（原 0.5 過快） |
| DRIFT_THRESHOLD_TIME | 2.0 | 連續偏離 2 秒才觸發 |

**新增成員變數**：
```cpp
float _driftTimer;  // 漂移計時器
```

**驗收標準**：
- 連續運行 5 分鐘後，右側距離仍在 10-20cm 範圍內

---

### 3.3 🔴 Phase 3：轉彎後重置航向（必須）

**目的**：每次轉彎後重新鎖定航向，避免累積誤差

**位置**：`wall_follower.cpp` → `update()` 函數

**邏輯**（經 Critic 修正）：
```cpp
// 記錄轉彎起始角度
if (front < FRONT_STOP_DIST && !_cornerLocked) {
    _cornerLocked = true;
    _cornerCount++;
    _lastTurnYaw = yaw;  // 記錄轉彎開始時的航向
}

// 判斷轉彎完成：已轉過 70° 且前方安全
if (_cornerLocked && front > FRONT_SLOW_DIST) {
    float turnedAngle = _angleDiff(yaw, _lastTurnYaw);
    if (turnedAngle > 70.0) {
        // 轉彎完成，重置航向
        _targetYaw = yaw;
        _yawLocked = true;
        _cornerLocked = false;
        _driftTimer = 0;
    }
}
```

**新增成員變數**：
```cpp
float _lastTurnYaw;  // 轉彎起始航向
```

**驗收標準**：
- 4 次轉彎後，累積角度誤差 < 10°

---

### 3.4 🟡 Phase 4：EMI 降級策略（建議）

**目的**：IMU 因 EMI 失效時不停車，改用純超聲波模式

**位置**：`wall_follower.cpp` → `update()` 函數

**邏輯**：
```cpp
// 在 update() 開頭判斷控制模式
bool useImuControl = imuValid && _yawLocked;

if (useImuControl) {
    // 使用 IMU 航向 PID
    float yawError = _normalizeAngle(yaw - _targetYaw);
    float yawCorrection = g_yawKp * yawError;
    angular += yawCorrection * (1.0 - obstacleWeight);
} else {
    // 降級到 Bang-Bang 控制
    if (right >= 2 && right < 100) {
        float error = right - TARGET_RIGHT_DIST;
        if (abs(error) >= ERROR_DEADZONE) {
            angular += (error > 0) ? -GENTLE_ANGULAR : GENTLE_ANGULAR;
        }
    }
}
```

**驗收標準**：
- IMU 失效後能自動降級，不停車
- 降級後仍能基本沿牆行駛

---

### 3.5 🟡 入口進場邏輯（建議）

**目的**：從場外進場到右下角開始沿牆

**方式**：時間觸發（不使用狀態機，保持連續控制架構）

**邏輯**：
```cpp
unsigned long elapsedTime = millis() - _startupGracePeriod;

if (elapsedTime < 2000) {
    // Phase 1: 直走進場 (0-2秒)
    _setMotorOutput(0.4, 0);
    return;
} else if (elapsedTime < 4000) {
    // Phase 2: 右轉找牆 (2-4秒)
    _setMotorOutput(0.2, -0.3);
    return;
} else {
    // Phase 3: 正常沿牆模式
    // ... 航向 PID + 監督修正
}
```

---

## 4. 參數配置

### 4.1 新增參數（config.h）
```cpp
// ==================== 航向 PID 參數 ====================
#define DEFAULT_YAW_KP           0.35f   // 航向比例增益
#define DEFAULT_YAW_KI           0.0f    // 航向積分增益（暫不使用）
#define DEFAULT_YAW_KD           0.08f   // 航向微分增益
#define YAW_DRIFT_RATE           0.15f   // 漂移修正速度（度/秒）
#define DRIFT_THRESHOLD_TIME     2.0f    // 連續偏離觸發時間（秒）

extern float g_yawKp;
extern float g_yawKi;
extern float g_yawKd;
```

### 4.2 新增成員變數（wall_follower.h）
```cpp
private:
    float _targetYaw;      // 目標航向（度）
    bool _yawLocked;       // 航向是否已鎖定
    float _driftTimer;     // 漂移計時器（秒）
    float _lastTurnYaw;    // 轉彎起始航向
```

### 4.3 dt 保護
```cpp
// 限制 dt 範圍（避免異常修正）
float dt = (now - _lastUpdateTime) / 1000.0;
if (dt < 0.01 || dt > 0.2) dt = 0.02;  // 異常時使用預設值
```

---

## 5. 腳位配置

### 5.1 Arduino Uno
| 腳位 | 功能 | 說明 |
|-----|------|------|
| D2 | SERIAL_TX | → Pi RXD (GPIO15) |
| D3 | ENA | 左輪 PWM |
| D4 | SERIAL_RX | ← Pi TXD (GPIO14) |
| D5 | IN2 | 左輪方向 B |
| D6 | IN1 | 左輪方向 A |
| D7 | FRONT_TRIG | 前方超聲波 |
| D8 | FRONT_ECHO | 前方超聲波 |
| D9 | IN4 | 右輪方向 B (已對調) |
| D10 | IN3 | 右輪方向 A (已對調) |
| D11 | ENB | 右輪 PWM |
| A1 | RIGHT_TRIG | 右側超聲波 |
| A2 | RIGHT_ECHO | 右側超聲波 |
| A3 | VACUUM | 吸塵器繼電器 |
| A4 | SDA | MPU6050 |
| A5 | SCL | MPU6050 |

---

## 6. 通訊協定

### 6.1 Pi → Arduino
```
[0xAA][CMD][LEN][PAYLOAD][CHECKSUM][0x55]

CMD:
  0x01 = START      開始沿牆
  0x02 = STOP       停止
  0x03 = AVOID_RED  紅色閃避（目前只關吸塵器）
  0x04 = SET_VACUUM 吸塵器開關
```

### 6.2 Arduino → Pi
```
[0xBB][STATE][CORNERS][FRONT_H][FRONT_L][RIGHT_H][RIGHT_L][YAW_H][YAW_L][FLAGS][CHK][0x66]

共 12 bytes
```

---

## 7. 已知問題與經驗

| 問題 | 原因 | 解決方案 | 狀態 |
|-----|------|---------|------|
| C 字形軌跡 | 超聲波延遲 + Bang-Bang 震盪 | 改用 IMU 航向控制 | ✅ 已解決 |
| 右輪偏慢 | 馬達特性差異 | RIGHT_MOTOR_SCALE = 0.95 | ✅ 已解決 |
| IN3/IN4 反向 | 接線問題 | 已在 config.h 對調 | ✅ 已解決 |
| EMI 干擾 | 馬達啟動影響 I2C | Watchdog + 降級策略 | 部分解決 |
| IMU 漂移 | 陀螺儀積分誤差 | 超聲波監督修正 | ✅ 已解決 |

---

## 8. 驗收標準總覽

| 項目 | 標準 |
|-----|------|
| 直行軌跡 | 震盪 < ±5cm，無 C 字形 |
| 長期穩定 | 5 分鐘內右側距離保持 15 ± 5 cm |
| 轉彎精度 | 4 次轉彎累積誤差 < 10° |
| EMI 容錯 | IMU 失效後自動降級，不停車 |
| 超聲波噪音 | 單次錯誤讀數不觸發修正 |

---

## 9. 實現順序

| 階段 | 內容 | 檔案 | 預估時間 |
|-----|------|------|---------|
| **Phase 1** | IMU 航向 PID | config.h, wall_follower.cpp/h | 1-2 天 |
| **Phase 2** | 超聲波監督修正 | wall_follower.cpp | 1 天 |
| **Phase 3** | 轉彎重置航向 | wall_follower.cpp | 1 天 |
| Phase 4 | EMI 降級策略 | wall_follower.cpp | 0.5 天 |
| Phase 5 | 入口進場邏輯 | wall_follower.cpp | 0.5 天 |

---

## 10. 版本歷史

| 日期 | 版本 | 變更 |
|-----|------|------|
| 2025-12-07 | v2.2 | **IMU 航向控制完整實現**（Phase 1-3 + dt 保護） |
| 2025-12-06 | v2.1-dev | 經 Critic 驗證，修正參數和邏輯 |
| 2025-12-06 | v2.0-dev | 初版設計文件 |
| 2025-12-05 | v2.0 | Bang-Bang 控制 + 死區 |
| 2025-12-04 | v1.9 | 修正右輪腳位 IN3/IN4 |
| 2025-12-03 | v1.8 | 加入 EMI 保護 |
