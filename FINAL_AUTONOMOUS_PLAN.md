# 期末自走競賽系統設計方案 v2.0

**更新日期：** 2025-11-28
**競賽目標：** 計時清掃紙屑並離場（速度優先！）

---

## 一、競賽規則分析

### 1.1 評分標準（關鍵！）

| 項目 | 分數 | 說明 |
|------|------|------|
| **Speed (速度)** | **70 分** | 完成時間決定，佔比最高！ |
| **Suction Power** | **40 分** | 吸到多少紙屑 |
| **Exterior Integrity** | 5 分 | 跑完還能動 |
| **Aesthetics** | 5 分 | 外觀設計 |
| **總分** | **120 分** | |

### 1.2 時間獎勵表（速度是關鍵！）

| 完成時間 | 速度分 | 策略 |
|---------|--------|------|
| **< 2:30** | **70 分** | 🎯 目標！ |
| 2:30 ~ 3:30 | 60 分 | 可接受 |
| 3:30 ~ 4:30 | 50 分 | 普通 |
| 4:30 ~ 6:00 | 40 分 | 較差 |
| > 6:00 | 30 分 | 最低 |

### 1.3 扣分項目

| 違規 | 扣分 | 上限 |
|------|------|------|
| 撞牆 | -1 分/次 | -5 分 |
| 人工介入 | -2 分/次 | -10 分 |

### 1.4 關鍵結論

**速度 > 完美覆蓋！**
- 2:30 內完成 + 吸 60% 紙屑 ≈ **104 分**
- 6:00+ 完成 + 吸 100% 紙屑 ≈ **80 分**

---

## 二、場地分析

### 2.1 場地佈局（300cm × 300cm）

```
┌─────────────────────────────────────────────────────┐
│                     300 cm                          │
│                                                     │
│   🔵 藍 0.6分    ┌────────────┐    🔴 紅 -0.5分    │
│   20片           │   60cm     │    20片 (禁區!)    │
│   ↖距牆15cm     │   禁區     │                    │
│                  └────────────┘                    │
│                                        🔴 紅圓標記  │
│                    🟢 綠 0.2分         (20cm)      │
│                    20片 (中央)         隨機角落    │
│                                                    │
│  入口⟹                                            │
│  出口⟸ (同一個!)                                   │
│                                                    │
│   🔵 藍 0.6分                      🔵 藍 0.6分     │
│   20片                             20片            │
│   ↙距牆15cm                       ↘距牆15cm       │
└─────────────────────────────────────────────────────┘
```

### 2.2 紙屑計分

| 顏色 | 分數/片 | 數量 | 位置 | 策略 |
|------|--------|------|------|------|
| 🔵 藍 | +0.6 | 80片 (4角) | 四角落，距牆15cm | ✅ 優先！ |
| 🟢 綠 | +0.2 | 20片 | 中央 | ⚠️ 低價值，可跳過 |
| 🔴 紅 | **-0.5** | 20片 | 右上角禁區 | ❌ 絕對避開！ |

### 2.3 紙屑大小

**0.8cm × 0.8cm** → 極小！視覺偵測不可行！

### 2.4 紅圓標記

- 直徑 20cm，在牆上
- **隨機放在某個角落**
- 需要辨識並反應（蜂鳴、警告、轉向）

---

## 三、硬體配置

### 3.1 感測器佈局

```
         前進方向
            ↑
     ┌──────┴──────┐
     │ [前超聲波]   │  ← 偵測前方牆壁（轉彎用）
     │ [相機]  ↑   │  ← 偵測紅圓標記
     │             │
     │  車    [右超]→  ← 偵測右側牆壁（沿牆用）
     │  體    聲波 │
     │             │
     │  [吸塵器]   │
     └─────────────┘
```

### 3.2 腳位配置（需更新 config.h）

```cpp
// 超聲波感測器 - 改為前+右
#define PIN_US_FRONT_TRIG  7   // 前方 Trig
#define PIN_US_FRONT_ECHO  8   // 前方 Echo
#define PIN_US_RIGHT_TRIG  A1  // 右側 Trig
#define PIN_US_RIGHT_ECHO  A2  // 右側 Echo
```

---

## 四、核心策略：沿右牆快速掃描

### 4.1 設計理念

**不依賴視覺偵測紙屑！**（0.8cm 太小看不到）

- ✅ 超聲波沿右牆行駛
- ✅ 時間控制轉彎（不需 MPU6050）
- ✅ 角落小範圍繞動確保吸到紙屑
- ✅ 相機只用於偵測紅圓標記
- ✅ 全程開啟吸塵器

### 4.2 路線規劃（沿右牆繞一圈）

```
┌─────────────────────────────────────────────────────┐
│          ⑦←←←←←←←←←⑥ 掃左上角                      │
│          ↓          ↑                              │
│          ↓          ↑    🔴 看到紅圓               │
│          ↓          ↑    → 左轉離開                │
│          ↓          ⑤    → 右前方找回牆            │
│          ↓          ↑                              │
│    ⑧←←←←←          ④←←←←③ 掃右下角                │
│    ↓ 掃左下角              ↑                       │
│  出⟸⑨               ①⟹入 ②→→→→→→→→→→→→↗          │
│                              沿下牆前進            │
└─────────────────────────────────────────────────────┘

時間預算 (目標 < 2:30 = 150秒):
① 入口出發: 0s
② 沿下牆到右下角: ~15s
③ 右下角掃描+轉彎: ~20s
④ 沿右牆往上: ~15s
⑤ 偵測紅圓/到右上前: ~10s
⑥ 跳過禁區,到左上角: ~25s
⑦ 左上角掃描+轉彎: ~20s
⑧ 沿左牆到左下角: ~15s
⑨ 左下角掃描+返回入口: ~25s
─────────────────────────
預估總時間: ~145s ✅
```

### 4.3 角落掃描動作

到達角落時，小範圍移動確保吸塵器涵蓋紙屑區域：

```
紙屑分布: 距牆15cm的區域內

角落掃描模式:
    ┌─────┐
    │ ↗↘  │  1. 小幅前進
    │ ↙↖  │  2. 左右擺動
    │     │  3. 小幅後退
    └─────┘  4. 總時間 ~15-20秒

目的: 讓吸塵器在角落區域繞動，確保覆蓋紙屑
```

---

## 五、控制邏輯

### 5.1 主狀態機

```python
class RobotState(Enum):
    INIT = 0              # 初始化
    FOLLOW_WALL = 1       # 沿右牆前進
    CORNER_SWEEP = 2      # 角落掃描
    TURN_LEFT = 3         # 左轉90度
    AVOID_RED = 4         # 紅圓迴避
    RETURN_TO_WALL = 5    # 找回右牆
    EXIT = 6              # 返回出口
    DONE = 7              # 完成
```

### 5.2 沿右牆控制

```python
class WallFollower:
    """沿右牆行駛控制器"""

    # 目標: 保持右側距牆 15-20cm
    TARGET_DIST = 18      # cm
    TOLERANCE = 5         # ±5cm

    FRONT_STOP = 20       # 前方 < 20cm 要轉彎
    FRONT_SLOW = 40       # 前方 < 40cm 開始減速

    def compute(self, front_dist, right_dist, base_speed=200):
        """
        計算馬達 PWM

        Returns:
            (left_pwm, right_pwm, action)
            action: 'forward', 'turn_left', 'adjust_left', 'adjust_right'
        """
        # 1. 前方有牆 → 需要左轉
        if front_dist < self.FRONT_STOP:
            return (0, 0, 'turn_left')

        # 2. 前方接近 → 減速
        speed = base_speed
        if front_dist < self.FRONT_SLOW:
            speed = int(base_speed * 0.6)

        # 3. 沿右牆修正
        error = right_dist - self.TARGET_DIST

        if error > self.TOLERANCE:
            # 離牆太遠 → 右偏 (右輪慢)
            left_pwm = speed
            right_pwm = int(speed * 0.7)
            action = 'adjust_right'
        elif error < -self.TOLERANCE:
            # 離牆太近 → 左偏 (左輪慢)
            left_pwm = int(speed * 0.7)
            right_pwm = speed
            action = 'adjust_left'
        else:
            # 距離剛好 → 直走
            left_pwm = speed
            right_pwm = speed
            action = 'forward'

        return (left_pwm, right_pwm, action)
```

### 5.3 左轉90度（時間控制）

```python
class TurnController:
    """時間控制轉彎"""

    # 需要實測調整!
    TURN_90_TIME = 0.6    # 秒
    TURN_PWM = 180

    def turn_left_90(self, motor_controller):
        """原地左轉90度"""
        # 左輪後退，右輪前進
        motor_controller.set_motors(-self.TURN_PWM, self.TURN_PWM)
        time.sleep(self.TURN_90_TIME)
        motor_controller.set_motors(0, 0)
        time.sleep(0.1)  # 穩定
```

### 5.4 角落掃描

```python
class CornerSweeper:
    """角落區域掃描"""

    SWEEP_TIME = 15       # 秒
    SWEEP_SPEED = 150

    def sweep_corner(self, motor_controller):
        """在角落小範圍移動掃描"""
        start = time.time()
        phase = 0

        while time.time() - start < self.SWEEP_TIME:
            elapsed = time.time() - start

            # 交替動作
            if phase == 0:  # 小幅前進
                motor_controller.set_motors(self.SWEEP_SPEED, self.SWEEP_SPEED)
                if elapsed > 2:
                    phase = 1
            elif phase == 1:  # 右轉
                motor_controller.set_motors(self.SWEEP_SPEED, -self.SWEEP_SPEED)
                if elapsed > 3.5:
                    phase = 2
            elif phase == 2:  # 小幅前進
                motor_controller.set_motors(self.SWEEP_SPEED, self.SWEEP_SPEED)
                if elapsed > 5:
                    phase = 3
            elif phase == 3:  # 左轉
                motor_controller.set_motors(-self.SWEEP_SPEED, self.SWEEP_SPEED)
                if elapsed > 6.5:
                    phase = 4
            elif phase == 4:  # 小幅後退
                motor_controller.set_motors(-self.SWEEP_SPEED, -self.SWEEP_SPEED)
                if elapsed > 8:
                    phase = 5
            else:  # 繼續小範圍動作
                # 左右擺動
                swing = int(100 * math.sin((elapsed - 8) * 3))
                motor_controller.set_motors(100 + swing, 100 - swing)

            time.sleep(0.05)

        motor_controller.set_motors(0, 0)
```

### 5.5 紅圓偵測與迴避

```python
class RedCircleDetector:
    """偵測牆上的紅圓標記 (20cm直徑)"""

    def __init__(self):
        # 紅色 HSV 範圍
        self.red_lower1 = np.array([0, 120, 70])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 120, 70])
        self.red_upper2 = np.array([180, 255, 255])

        self.DETECT_THRESHOLD = 0.03  # 紅色佔畫面 3% 以上

    def detect(self, frame):
        """
        偵測紅圓

        Returns:
            (detected: bool, ratio: float, position: str)
            position: 'left', 'center', 'right'
        """
        if frame is None:
            return False, 0, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 紅色遮罩
        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = mask1 | mask2

        # 計算比例
        h, w = frame.shape[:2]
        red_pixels = cv2.countNonZero(red_mask)
        ratio = red_pixels / (h * w)

        if ratio < self.DETECT_THRESHOLD:
            return False, ratio, None

        # 計算位置
        moments = cv2.moments(red_mask)
        if moments['m00'] > 0:
            cx = moments['m10'] / moments['m00']
            if cx < w * 0.33:
                position = 'left'
            elif cx > w * 0.67:
                position = 'right'
            else:
                position = 'center'
        else:
            position = 'center'

        return True, ratio, position

    def on_detected(self):
        """偵測到紅圓的反應"""
        print("⚠️ RED CIRCLE DETECTED!")
        # TODO: 蜂鳴器警告
        # buzzer.beep()


class RedAvoidance:
    """紅圓迴避策略"""

    def avoid(self, motor_controller, red_position):
        """
        迴避紅圓

        紅圓在角落牆上，看到時:
        1. 左轉離開
        2. 往右前方前進找回右牆
        """
        # 1. 左轉離開 (約45-60度)
        motor_controller.set_motors(-150, 150)
        time.sleep(0.4)
        motor_controller.set_motors(0, 0)

        # 2. 右前方前進找牆
        # 稍微右偏的前進，直到右超聲波偵測到牆
        return 'finding_wall'
```

### 5.6 找回右牆

```python
class WallFinder:
    """紅圓迴避後找回右牆"""

    TARGET_DIST = 18
    MAX_TIME = 5  # 秒

    def find_wall(self, front_dist, right_dist, motor_controller):
        """
        右前方前進直到找到右牆

        Returns:
            'found' 或 'searching'
        """
        # 如果右側已經偵測到合理距離的牆
        if 10 < right_dist < 50:
            return 'found'

        # 右前方前進 (右輪稍慢)
        motor_controller.set_motors(180, 150)
        return 'searching'
```

---

## 六、主程式架構

```python
#!/usr/bin/env python3
"""
autonomous_main.py - 期末自走競賽主程式
"""

import cv2
import time
import serial
from enum import Enum

class AutonomousRobot:
    def __init__(self):
        # 硬體
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)

        # 控制器
        self.wall_follower = WallFollower()
        self.turn_controller = TurnController()
        self.corner_sweeper = CornerSweeper()
        self.red_detector = RedCircleDetector()
        self.red_avoidance = RedAvoidance()
        self.wall_finder = WallFinder()

        # 狀態
        self.state = RobotState.INIT
        self.corner_count = 0  # 已掃描的角落數
        self.start_time = None

        # 計時
        self.TARGET_TIME = 140  # 目標 2:20 完成
        self.MAX_TIME = 150     # 最長 2:30

    def run(self):
        """主控制迴圈"""
        self.start_time = time.time()
        self.vacuum_on()

        print("🚀 自走模式啟動!")

        while True:
            # 讀取感測器
            front_dist, right_dist = self.read_ultrasonic()
            ret, frame = self.camera.read()

            # 計時檢查
            elapsed = time.time() - self.start_time
            if elapsed > self.MAX_TIME:
                print("⏰ 超時! 強制返回")
                self.state = RobotState.EXIT

            # 狀態機
            if self.state == RobotState.INIT:
                self.state = RobotState.FOLLOW_WALL
                print("▶ 開始沿牆行駛")

            elif self.state == RobotState.FOLLOW_WALL:
                self._handle_follow_wall(front_dist, right_dist, frame)

            elif self.state == RobotState.CORNER_SWEEP:
                self._handle_corner_sweep()

            elif self.state == RobotState.TURN_LEFT:
                self._handle_turn_left()

            elif self.state == RobotState.AVOID_RED:
                self._handle_avoid_red()

            elif self.state == RobotState.RETURN_TO_WALL:
                self._handle_return_to_wall(front_dist, right_dist)

            elif self.state == RobotState.EXIT:
                self._handle_exit(front_dist, right_dist)

            elif self.state == RobotState.DONE:
                break

            time.sleep(0.05)  # 20 Hz

        # 完成
        self.stop()
        self.vacuum_off()
        elapsed = time.time() - self.start_time
        print(f"✅ 完成! 總時間: {elapsed:.1f}秒")

    def _handle_follow_wall(self, front_dist, right_dist, frame):
        """沿牆行駛處理"""

        # 檢查紅圓
        detected, ratio, position = self.red_detector.detect(frame)
        if detected and ratio > 0.05:
            print(f"🔴 偵測到紅圓! ({position})")
            self.red_detector.on_detected()
            self.state = RobotState.AVOID_RED
            return

        # 沿牆控制
        left_pwm, right_pwm, action = self.wall_follower.compute(
            front_dist, right_dist
        )

        if action == 'turn_left':
            # 到達角落
            self.stop()
            self.corner_count += 1
            print(f"📍 到達角落 #{self.corner_count}")

            if self.corner_count >= 4:
                # 繞完一圈，準備離開
                self.state = RobotState.EXIT
            else:
                # 進入角落掃描
                self.state = RobotState.CORNER_SWEEP
        else:
            self.set_motors(left_pwm, right_pwm)

    def _handle_corner_sweep(self):
        """角落掃描處理"""
        print("🧹 角落掃描中...")
        self.corner_sweeper.sweep_corner(self)
        print("✓ 角落掃描完成")
        self.state = RobotState.TURN_LEFT

    def _handle_turn_left(self):
        """左轉處理"""
        print("↰ 左轉90度")
        self.turn_controller.turn_left_90(self)
        self.state = RobotState.FOLLOW_WALL

    def _handle_avoid_red(self):
        """紅圓迴避處理"""
        print("⚠️ 迴避紅圓...")
        self.red_avoidance.avoid(self, 'center')
        self.state = RobotState.RETURN_TO_WALL

    def _handle_return_to_wall(self, front_dist, right_dist):
        """找回右牆處理"""
        result = self.wall_finder.find_wall(front_dist, right_dist, self)
        if result == 'found':
            print("✓ 找到右牆")
            self.state = RobotState.FOLLOW_WALL

    def _handle_exit(self, front_dist, right_dist):
        """返回出口處理"""
        # 沿右牆回到入口 (入口在起點，繞一圈後會回來)
        left_pwm, right_pwm, action = self.wall_follower.compute(
            front_dist, right_dist, base_speed=220  # 稍快
        )

        if action == 'turn_left':
            # 可能是回到入口了
            self.state = RobotState.DONE
        else:
            self.set_motors(left_pwm, right_pwm)

    # ===== 硬體控制方法 =====

    def set_motors(self, left_pwm, right_pwm):
        """發送馬達指令到 Arduino"""
        # 使用現有的二進位封包格式
        # ... (沿用期中的 serial protocol)
        pass

    def stop(self):
        self.set_motors(0, 0)

    def vacuum_on(self):
        # ...
        pass

    def vacuum_off(self):
        # ...
        pass

    def read_ultrasonic(self):
        """讀取超聲波數據"""
        # 從 Arduino 接收感測器封包
        # 返回 (front_dist, right_dist)
        return 100, 20  # placeholder


if __name__ == '__main__':
    robot = AutonomousRobot()
    robot.run()
```

---

## 七、Arduino 修改需求

### 7.1 config.h - 更新超聲波命名

```cpp
// 超聲波感測器 - 前方 + 右側
#define PIN_US_FRONT_TRIG  7   // 前方 Trig (原 LEFT)
#define PIN_US_FRONT_ECHO  8   // 前方 Echo
#define PIN_US_RIGHT_TRIG  A1  // 右側 Trig
#define PIN_US_RIGHT_ECHO  A2  // 右側 Echo
```

### 7.2 main.ino - 啟用超聲波

需要解決的問題：
- `pulseIn()` 阻塞最多 30ms
- 兩個感測器 = 最多 60ms 阻塞

解決方案：**交替讀取**

```cpp
// main.ino 修改

// 全域變數
uint16_t frontDistance = 999;
uint16_t rightDistance = 999;
bool readFront = true;  // 交替讀取

// loop() 中
if (currentTime - lastSensorTime >= 50) {  // 50ms = 20Hz
    if (readFront) {
        frontDistance = frontUltrasonic.getDistance();
    } else {
        rightDistance = rightUltrasonic.getDistance();
        sendSensorData();  // 兩個都讀完才發送
    }
    readFront = !readFront;
    lastSensorTime = currentTime;
}
```

### 7.3 Serial 封包格式（沿用現有）

**Pi → Arduino (馬達指令):**
```
| 0xAA | leftPWM(2) | rightPWM(2) | flags | checksum | 0x55 |
```

**Arduino → Pi (感測器):**
```
| 0xBB | frontDist(2) | rightDist(2) | status | checksum | 0x66 |
```

---

## 八、開發優先順序

### Phase 1: 硬體準備 (Day 1)
- [ ] 確認超聲波安裝位置（前方 + 右側）
- [ ] 更新 config.h 命名
- [ ] 啟用超聲波（交替讀取）
- [ ] 測試超聲波讀值穩定性

### Phase 2: 基礎控制 (Day 2-3)
- [ ] 實作沿右牆控制 (`WallFollower`)
- [ ] 測試直線行駛穩定性
- [ ] 實作時間控制左轉
- [ ] 調校轉彎時間參數

### Phase 3: 角落掃描 (Day 4)
- [ ] 實作角落掃描動作
- [ ] 測試紙屑覆蓋率
- [ ] 調整掃描時間與範圍

### Phase 4: 紅圓偵測 (Day 5)
- [ ] 實作紅圓 HSV 偵測
- [ ] 測試偵測距離與準確度
- [ ] 實作迴避邏輯
- [ ] 實作找回右牆

### Phase 5: 整合測試 (Day 6-7)
- [ ] 完整路線測試
- [ ] 計時優化
- [ ] 極端情況處理
- [ ] 參數微調

---

## 九、關鍵參數（需實測調校）

```python
# 速度參數
BASE_SPEED = 200          # 基礎 PWM (0-255)
CORNER_SPEED = 150        # 角落掃描速度
TURN_SPEED = 180          # 轉彎速度

# 沿牆參數
TARGET_WALL_DIST = 18     # 目標距牆距離 (cm)
WALL_TOLERANCE = 5        # 容差 (cm)
FRONT_STOP_DIST = 20      # 前方停止距離 (cm)
FRONT_SLOW_DIST = 40      # 前方減速距離 (cm)

# 轉彎參數
TURN_90_TIME = 0.6        # 90度轉彎時間 (秒) ⚠️ 需實測!

# 角落掃描參數
CORNER_SWEEP_TIME = 15    # 每個角落掃描時間 (秒)

# 紅圓偵測參數
RED_DETECT_THRESHOLD = 0.03  # 紅色佔畫面比例閾值
```

---

## 十、預期成績

| 項目 | 預期 | 說明 |
|------|------|------|
| 完成時間 | ~2:20 | 目標 < 2:30 |
| 速度分 | 70 | 滿分 |
| 紙屑覆蓋 | ~70% | 4角落 + 部分中央 |
| 吸力分 | ~28 | 70片 × 0.4 |
| 外觀 + 完整 | 10 | |
| 撞牆扣分 | -2 | 預估2次 |
| **總分** | **~106** | |

---

## 十一、風險與應對

| 風險 | 可能性 | 應對 |
|------|--------|------|
| 轉彎角度不準 | 高 | 多次實測調校時間參數 |
| 超聲波干擾 | 中 | 交替讀取避免同時觸發 |
| 紅圓漏偵測 | 中 | 降低閾值、放慢速度 |
| 紙屑吸不乾淨 | 中 | 增加角落掃描時間/範圍 |
| 找不回右牆 | 低 | 超時保護，強制前進 |

---

## 十二、與期中系統的相容性

| 模組 | 期中 | 期末 | 狀態 |
|------|------|------|------|
| Serial 協定 | 二進位封包 | 沿用 | ✅ |
| 馬達控制 | L298N | 沿用 | ✅ |
| 差動驅動 | 已實作 | 沿用 | ✅ |
| 吸塵器控制 | 已實作 | 沿用 | ✅ |
| 超聲波 | 左/右(停用) | 前/右(啟用) | ⚠️ 需改 |
| 相機 | MJPEG串流 | OpenCV處理 | ⚠️ 需改 |

**結論：大部分基礎設施可直接沿用，只需修改超聲波配置和新增視覺處理。**
