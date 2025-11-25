# 期末自走競賽系統設計方案

**更新日期：** 2025-11-25
**競賽目標：** 計時清掃紙屑並離場
**評分標準：** 時間級距分數 + 紙屑數量分數
**硬體限制：** 無編碼器、無預布置標記

---

## 核心設計理念：三層融合架構

```
┌─────────────────────────────────────────────────────────────┐
│                    混合導航系統                              │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: 安全層（最高優先，平順修正）                       │
│  ├─ 超聲波即時測距 → 平滑減速 + 轉向修正（不急停）          │
│  └─ 視覺紅色偵測 → 平順繞開（不是「嚇到」的緊急迴避）       │
│      💡 看到紅色 = 「這邊不要去」，平順轉向而非急煞         │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: 相對定位層（MPU6050）                              │
│  ├─ 航向角追蹤 → 知道「面向哪裡」                           │
│  ├─ 轉彎角度累計 → 知道「轉了多少」                         │
│  └─ 配合預載地圖 → 粗略知道「在哪個區域」                   │
├─────────────────────────────────────────────────────────────┤
│  Layer 3: 預載區域資訊（減輕視覺壓力）                       │
│  ├─ 紙屑區域位置（提前輸入 4 角落 + 1 中央）                │
│  ├─ 入口方向記錄                                            │
│  └─ 場地大小估計                                            │
├─────────────────────────────────────────────────────────────┤
│  Layer 4: 視覺確認層（輔助而非主導）                         │
│  ├─ 紅色膠帶偵測 → 確認禁區，平順避開                       │
│  ├─ 紙屑偵測 → 區域內精確清掃                               │
│  └─ 黑色膠帶偵測 → 確認到達目標區（可選）                   │
└─────────────────────────────────────────────────────────────┘
```

**設計哲學：先求穩**
- 安全層 = 平順修正，不是「被嚇到」的急煞
- 用 IMU 提供粗略位置感知 → 減少視覺依賴
- 預載區域資訊 → 機器人「知道」紙屑在哪，只需確認
- 視覺降級為「確認」而非「發現」 → 降低相機品質影響

### 感測器配置

```
            [相機]
         前方視野
         黑牆/紅色/紙屑
              ↓
        ┌───────────────┐
        │               │
    ←───┤   [機器人]    ├───→
   左超聲波            右超聲波
   (左側牆)            (右側牆)
        │               │
        └───────────────┘
```

| 感測器 | 用途 | 優先級 |
|--------|------|--------|
| **左超聲波** | 左側牆距，斜向移動保護 | 高 |
| **右超聲波** | 右側牆距，斜向移動保護 | 高 |
| **相機-黑色** | 前方黑牆偵測（簡單判斷） | 中 |
| **相機-紅色** | 紅色禁區偵測 | 高 |
| **相機-白色** | 紙屑偵測 | 清掃時 |
| **相機-黑膠帶** | 確認到達區域（optional） | 低 |
| **MPU6050** | 航向追蹤 | 導航時 |

---

## 零、混合架構核心模組

### 0.1 預載區域地圖（紅區隨機版）

```python
import numpy as np
import cv2

class PreloadedFieldMap:
    """
    預載場地資訊 - 減輕視覺壓力的關鍵

    場地佈局：
         入口
          ↓
    ┌─────────────────┐
    │ [C1]       [C2] │   C1-C4: 四角落（其中 3 個有紙屑，1 個是紅區）
    │   ╲         ╱   │   Z5: 中央紙屑區
    │     ╲     ╱     │
    │       [Z5]      │   ⚠️ 紅區位置隨機，需視覺確認！
    │     ╱     ╲     │
    │   ╱         ╲   │
    │ [C3]       [C4] │
    └─────────────────┘

    策略：
    - 預載 4 個角落 + 1 中央的「位置」
    - 到達角落前，用視覺確認是紅區還是紙屑區
    - 紅區 → 跳過，紙屑區 → 清掃
    """

    def __init__(self, field_size=2.0):
        self.field_size = field_size
        self.half = field_size / 2

        # 4 個角落位置（還不知道哪個是紅區）
        # 格式: (angle_from_entry, distance_from_entry)
        self.corners = {
            'C1': {'angle': -45, 'distance': 1.4, 'status': 'unknown'},   # 左前角
            'C2': {'angle': 45, 'distance': 1.4, 'status': 'unknown'},    # 右前角
            'C3': {'angle': -135, 'distance': 1.4, 'status': 'unknown'},  # 左後角
            'C4': {'angle': 135, 'distance': 1.4, 'status': 'unknown'},   # 右後角
        }

        # 中央區域（一定是紙屑區，低分）
        self.center = {
            'Z5': {'angle': 0, 'distance': 1.0, 'status': 'paper', 'priority': 3}
        }

        # 入口資訊
        self.entry_heading = 0

    def mark_corner_as_red(self, corner_id):
        """視覺確認後，標記某角落為紅區"""
        if corner_id in self.corners:
            self.corners[corner_id]['status'] = 'red'
            print(f"[地圖] {corner_id} 標記為紅區")

    def mark_corner_as_paper(self, corner_id):
        """視覺確認後，標記某角落為紙屑區"""
        if corner_id in self.corners:
            self.corners[corner_id]['status'] = 'paper'
            print(f"[地圖] {corner_id} 標記為紙屑區")

    def mark_corner_cleaned(self, corner_id):
        """標記角落已清掃"""
        if corner_id in self.corners:
            self.corners[corner_id]['status'] = 'cleaned'

    def get_next_target(self):
        """
        選擇下一個目標

        優先順序：
        1. 未確認的角落（需要先去看看是什麼）
        2. 已確認的紙屑角落（清掃）
        3. 中央區域（最後，分數低）

        Returns:
            (zone_id, zone_info) 或 None
        """
        # 1. 優先處理未確認的角落（由近到遠）
        unknown_corners = [
            (cid, cinfo) for cid, cinfo in self.corners.items()
            if cinfo['status'] == 'unknown'
        ]
        if unknown_corners:
            # 按距離排序，先去近的
            return min(unknown_corners, key=lambda x: x[1]['distance'])

        # 2. 處理已確認的紙屑角落
        paper_corners = [
            (cid, cinfo) for cid, cinfo in self.corners.items()
            if cinfo['status'] == 'paper'
        ]
        if paper_corners:
            return min(paper_corners, key=lambda x: x[1]['distance'])

        # 3. 最後處理中央
        if self.center['Z5']['status'] == 'paper':
            return ('Z5', self.center['Z5'])

        return None

    def get_zone_direction(self, zone_id, current_heading):
        """計算到目標區域需要轉多少度"""
        if zone_id in self.corners:
            target_angle = self.corners[zone_id]['angle']
        elif zone_id == 'Z5':
            target_angle = self.center['Z5']['angle']
        else:
            return None

        turn_angle = target_angle - current_heading

        # 正規化到 -180 ~ 180
        while turn_angle > 180:
            turn_angle -= 360
        while turn_angle < -180:
            turn_angle += 360

        return turn_angle

    def get_return_direction(self, current_heading):
        """計算返回入口需要轉的角度（入口在 180° 方向）"""
        turn_angle = 180 - current_heading
        while turn_angle > 180:
            turn_angle -= 360
        while turn_angle < -180:
            turn_angle += 360
        return turn_angle

    def get_all_paper_zones_cleaned(self):
        """檢查是否所有紙屑區都清掃完畢"""
        for cinfo in self.corners.values():
            if cinfo['status'] == 'paper':  # 還有未清的紙屑區
                return False
        if self.center['Z5']['status'] == 'paper':
            return False
        return True


class IMUTracker:
    """
    MPU6050 航向追蹤器

    用途：
    - 追蹤機器人「面向哪裡」（相對於入口方向）
    - 配合預載地圖，知道該往哪轉
    - 短時間內精度足夠
    """

    def __init__(self):
        self.heading = 0.0  # 當前航向（度），0 = 入口方向
        self.gyro_z_offset = 0.0

    def update(self, gyro_z, dt):
        """
        更新航向角

        Args:
            gyro_z: Z 軸角速度（度/秒）
            dt: 時間間隔（秒）
        """
        corrected_gyro = gyro_z - self.gyro_z_offset
        self.heading += corrected_gyro * dt
        self.heading = self.heading % 360

    def reset_heading(self):
        """重置航向為 0"""
        self.heading = 0.0

    def get_heading(self):
        """取得當前航向"""
        return self.heading


class SafetyLayer:
    """
    安全層 - 平順修正

    感測器配置：
    - 左超聲波 + 右超聲波（兩側牆距）
    - 相機（前方黑牆、紅色禁區）

    設計哲學：
    - 看到紅色 = 「這邊不要去」，平順轉向
    - 不是急煞，是優雅繞開
    """

    def __init__(self):
        # ===== 超聲波參數（左+右）=====
        self.WALL_SLOW_DIST = 40      # cm，開始減速
        self.WALL_TURN_DIST = 25      # cm，開始轉向修正
        self.WALL_STOP_DIST = 10      # cm，停止

        # ===== 紅色視覺參數 =====
        self.RED_AVOID_THRESHOLD = 0.08  # 紅色佔畫面比例閾值
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])

        # ===== 前方黑牆視覺參數 =====
        self.BLACK_WALL_THRESHOLD = 0.35  # 畫面下半部黑色比例閾值
        self.black_lower = np.array([0, 0, 0])
        self.black_upper = np.array([180, 255, 50])

    def check_and_modify(self, base_cmd, frame, left_dist, right_dist):
        """
        檢查安全狀態並平順修正指令

        Args:
            base_cmd: (linear, angular, vacuum)
            frame: 相機影像
            left_dist: 左超聲波距離 (cm)
            right_dist: 右超聲波距離 (cm)

        Returns:
            modified_cmd, status ('safe'/'adjusting'/'blocked')
        """
        linear, angular, vacuum = base_cmd
        status = 'safe'

        # ===== 1. 左右超聲波檢查 =====
        if left_dist < self.WALL_STOP_DIST or right_dist < self.WALL_STOP_DIST:
            # 某側太近，停止
            return (-0.15, 0, vacuum), 'blocked'

        # 左側太近
        if left_dist < self.WALL_TURN_DIST:
            linear *= 0.5
            angular += 0.25  # 右轉
            status = 'adjusting'
        elif left_dist < self.WALL_SLOW_DIST:
            linear *= 0.8
            status = 'adjusting'

        # 右側太近
        if right_dist < self.WALL_TURN_DIST:
            linear *= 0.5
            angular -= 0.25  # 左轉
            status = 'adjusting'
        elif right_dist < self.WALL_SLOW_DIST:
            linear *= 0.8
            status = 'adjusting'

        # ===== 2. 前方黑牆檢查（相機，區分牆 vs 膠帶）=====
        if frame is not None and linear > 0:
            is_wall, black_ratio, is_tape = self._is_wall_close(frame)
            if is_wall:
                # 前方有牆，減速
                if black_ratio > 0.5:
                    linear *= 0.3  # 很近了
                else:
                    linear *= 0.6
                status = 'adjusting'
            # is_tape = True 時不減速，表示到達黑膠帶區（可選用於定位確認）

        # ===== 3. 紅色視覺檢查 =====
        if frame is not None and linear > 0:
            red_info = self._detect_red_region(frame)

            if red_info['detected']:
                # 平順繞開
                if red_info['position'] == 'left':
                    angular = max(angular, 0.35)
                elif red_info['position'] == 'right':
                    angular = min(angular, -0.35)
                elif red_info['position'] == 'center':
                    linear *= 0.5
                    angular = 0.4 if red_info['center_x'] < 0.5 else -0.4
                status = 'adjusting'

        linear = np.clip(linear, -0.5, 0.5)
        angular = np.clip(angular, -1.0, 1.0)

        return (linear, angular, vacuum), status

    def _is_wall_close(self, frame):
        """
        分區判斷：區分黑牆 vs 黑膠帶

        原理：
        - 黑牆：畫面中段+下段都有黑色（垂直延伸）
        - 黑膠帶：只有最底部有黑色（水平條狀在地板上）

        Returns:
            (is_wall: bool, black_ratio: float, is_tape: bool)
        """
        h, w = frame.shape[:2]

        # 分成三區
        mid_region = frame[h//3:2*h//3, :]     # 中 1/3
        bottom_region = frame[2*h//3:, :]      # 下 1/3

        def get_black_ratio(region):
            hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
            black_mask = cv2.inRange(hsv, self.black_lower, self.black_upper)
            return cv2.countNonZero(black_mask) / black_mask.size

        mid_black = get_black_ratio(mid_region)
        bottom_black = get_black_ratio(bottom_region)

        # 黑牆：中段+底部都黑（垂直延伸）
        is_wall = mid_black > 0.25 and bottom_black > 0.30

        # 黑膠帶：只有底部黑，中段不黑
        is_tape = bottom_black > 0.30 and mid_black < 0.15

        return is_wall, bottom_black, is_tape

    def _detect_red_region(self, frame):
        """偵測紅色區域"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        h, w = frame.shape[:2]
        red_pixels = cv2.countNonZero(red_mask)
        red_ratio = red_pixels / (h * w)

        if red_ratio < self.RED_AVOID_THRESHOLD:
            return {'detected': False, 'position': None, 'ratio': red_ratio}

        moments = cv2.moments(red_mask)
        if moments['m00'] > 0:
            cx = moments['m10'] / moments['m00']
            center_x = cx / w

            if center_x < 0.35:
                position = 'left'
            elif center_x > 0.65:
                position = 'right'
            else:
                position = 'center'
        else:
            position = 'center'
            center_x = 0.5

        return {
            'detected': True,
            'position': position,
            'ratio': red_ratio,
            'center_x': center_x
        }

    def is_red_zone_ahead(self, frame):
        """
        判斷前方是否是紅區（用於角落確認）

        Returns:
            True 如果前方大片紅色（紅區）
        """
        if frame is None:
            return False

        red_info = self._detect_red_region(frame)
        # 更高閾值，確認是真的紅區而非路過
        return red_info['detected'] and red_info['ratio'] > 0.2


class HybridNavigator:
    """
    混合導航控制器

    流程：
    1. 從預載地圖選擇下一個角落
    2. 用 IMU 計算轉向角度
    3. 前進到角落附近
    4. 視覺確認：紅區還是紙屑區？
       - 紅區 → 標記並跳過
       - 紙屑區 → 清掃
    5. 重複直到所有區域完成
    6. 返回入口
    """

    def __init__(self):
        self.field_map = PreloadedFieldMap()
        self.imu = IMUTracker()
        self.safety = SafetyLayer()

        self.current_target = None
        self.state = 'INIT'
        # INIT, TURNING, APPROACHING, CONFIRMING, CLEANING, RETURNING, DONE

        self.approach_start_time = None
        self.APPROACH_TIMEOUT = 5.0  # 前進超時（秒）

    def compute_command(self, frame, left_dist, right_dist, gyro_z, dt):
        """
        主控制函數

        Args:
            frame: 相機影像
            left_dist, right_dist: 超聲波距離 (cm)
            gyro_z: 陀螺儀 Z 軸角速度
            dt: 時間間隔

        Returns:
            (linear, angular, vacuum)
        """
        # 更新 IMU
        self.imu.update(gyro_z, dt)
        current_heading = self.imu.get_heading()

        if self.state == 'INIT':
            self.imu.reset_heading()
            self._select_next_target()
            if self.current_target:
                self.state = 'TURNING'
                print(f"[導航] 目標: {self.current_target[0]}")
            else:
                self.state = 'RETURNING'
            return (0, 0, False)

        elif self.state == 'TURNING':
            zone_id, zone_info = self.current_target
            turn_angle = self.field_map.get_zone_direction(zone_id, current_heading)

            if abs(turn_angle) < 15:  # 對準了
                self.state = 'APPROACHING'
                self.approach_start_time = None
                print(f"[導航] 對準 {zone_id}，開始前進")
                return (0, 0, False)
            else:
                angular = np.clip(turn_angle / 45, -0.4, 0.4)
                return (0, angular, False)

        elif self.state == 'APPROACHING':
            # 記錄開始時間
            import time
            if self.approach_start_time is None:
                self.approach_start_time = time.time()

            zone_id, zone_info = self.current_target

            # 超時檢查（應該已經到了）
            elapsed = time.time() - self.approach_start_time
            if elapsed > self.APPROACH_TIMEOUT:
                self.state = 'CONFIRMING'
                print(f"[導航] 到達 {zone_id} 附近，確認中...")
                return (0, 0, False)

            # 基礎指令：前進
            base_cmd = (0.35, 0, False)

            # 安全層修正
            safe_cmd, status = self.safety.check_and_modify(
                base_cmd, frame, left_dist, right_dist
            )

            return safe_cmd

        elif self.state == 'CONFIRMING':
            zone_id, zone_info = self.current_target

            # 視覺確認：紅區還是紙屑區？
            if self.safety.is_red_zone_ahead(frame):
                # 是紅區！標記並跳過
                self.field_map.mark_corner_as_red(zone_id)
                print(f"[導航] {zone_id} 是紅區，跳過")
                self._select_next_target()
                if self.current_target:
                    self.state = 'TURNING'
                else:
                    self.state = 'RETURNING'
            else:
                # 是紙屑區，開始清掃
                self.field_map.mark_corner_as_paper(zone_id)
                self.state = 'CLEANING'
                print(f"[導航] {zone_id} 是紙屑區，開始清掃")

            return (0, 0, False)

        elif self.state == 'CLEANING':
            zone_id, zone_info = self.current_target

            # 偵測紙屑
            paper_info = self._detect_paper(frame)

            if paper_info['found']:
                # 追蹤紙屑
                angular = paper_info['offset_x'] * 0.4
                base_cmd = (0.25, angular, True)
                safe_cmd, _ = self.safety.check_and_modify(
                    base_cmd, frame, left_dist, right_dist
                )
                return safe_cmd
            else:
                # 沒看到紙屑，完成這區
                self.field_map.mark_corner_cleaned(zone_id)
                print(f"[導航] {zone_id} 清掃完成")
                self._select_next_target()
                if self.current_target:
                    self.state = 'TURNING'
                else:
                    self.state = 'RETURNING'
                return (0, 0, False)

        elif self.state == 'RETURNING':
            turn_angle = self.field_map.get_return_direction(current_heading)

            if abs(turn_angle) > 15:
                angular = np.clip(turn_angle / 45, -0.4, 0.4)
                return (0, angular, False)
            else:
                base_cmd = (0.4, 0, False)
                safe_cmd, _ = self.safety.check_and_modify(
                    base_cmd, frame, left_dist, right_dist
                )
                return safe_cmd

        return (0, 0, False)

    def _select_next_target(self):
        """選擇下一個目標"""
        self.current_target = self.field_map.get_next_target()

    def _detect_paper(self, frame):
        """偵測紙屑"""
        if frame is None:
            return {'found': False}

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        white_lower = np.array([0, 0, 180])
        white_upper = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        valid_papers = []
        h, w = frame.shape[:2]

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 200 < area < 3000:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    valid_papers.append({
                        'cx': cx, 'cy': cy,
                        'offset_x': (cx - w/2) / (w/2)
                    })

        if valid_papers:
            nearest = max(valid_papers, key=lambda p: p['cy'])
            return {'found': True, 'offset_x': nearest['offset_x']}

        return {'found': False}
```

### 0.2 Arduino 端 MPU6050 整合

```cpp
// Arduino 新增 MPU6050
#include <Wire.h>

const int MPU_ADDR = 0x68;
float gyro_z_offset = 0;

void setupMPU6050() {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();

    // ±250°/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();

    calibrateGyro();
}

void calibrateGyro() {
    float sum = 0;
    for (int i = 0; i < 200; i++) {
        sum += readRawGyroZ();
        delay(10);
    }
    gyro_z_offset = sum / 200;
}

float readRawGyroZ() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    int16_t raw = Wire.read() << 8 | Wire.read();
    return raw / 131.0;
}

float getGyroZ() {
    return readRawGyroZ() - gyro_z_offset;
}

// Serial 格式: "L:xx,R:xx,G:xx.xx"
void sendSensorData() {
    Serial.print("L:");
    Serial.print(leftDistance);
    Serial.print(",R:");
    Serial.print(rightDistance);
    Serial.print(",G:");
    Serial.println(getGyroZ(), 2);
}
```

### 0.3 系統整合範例

```python
# autonomous_main.py

import cv2
import time
import serial

class AutonomousRobot:
    def __init__(self):
        self.navigator = HybridNavigator()
        self.camera = cv2.VideoCapture(0)
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)

        self.last_time = time.time()

    def parse_sensor_data(self, line):
        """解析 Arduino 數據: L:xx,R:xx,G:xx.xx"""
        try:
            parts = line.strip().split(',')
            left = int(parts[0].split(':')[1])
            right = int(parts[1].split(':')[1])
            gyro = float(parts[2].split(':')[1])
            return left, right, gyro
        except:
            return 100, 100, 0  # 預設值

    def send_command(self, linear, angular, vacuum):
        """發送指令到 Arduino"""
        # 轉換成馬達 PWM
        left_pwm = int((linear - angular) * 255)
        right_pwm = int((linear + angular) * 255)
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        vac = 1 if vacuum else 0
        cmd = f"M:{left_pwm},{right_pwm},{vac}\n"
        self.arduino.write(cmd.encode())

    def run(self):
        print("[系統] 自走模式啟動")

        while True:
            # 計算 dt
            now = time.time()
            dt = now - self.last_time
            self.last_time = now

            # 讀取相機
            ret, frame = self.camera.read()
            if not ret:
                frame = None

            # 讀取感測器
            if self.arduino.in_waiting:
                line = self.arduino.readline().decode()
                left_dist, right_dist, gyro_z = self.parse_sensor_data(line)
            else:
                left_dist, right_dist, gyro_z = 100, 100, 0

            # 計算導航指令
            linear, angular, vacuum = self.navigator.compute_command(
                frame, left_dist, right_dist, gyro_z, dt
            )

            # 發送指令
            self.send_command(linear, angular, vacuum)

            # 檢查完成
            if self.navigator.state == 'DONE':
                print("[系統] 任務完成！")
                break

            time.sleep(0.05)  # 20 Hz

        self.send_command(0, 0, False)
        self.camera.release()

if __name__ == '__main__':
    robot = AutonomousRobot()
    robot.run()
```

---

## 一、競賽分析

### 1.1 競賽規則

#### 評分系統
- **時間分（級距式）**
  - 級距制評分（例：< 2:00 滿分，2:00-2:30 次級，> 2:30 再次級）
  - **警告：** 若紙屑 < 30 片，時間分打 **八折**

- **紙屑分**
  - 目標：80 片（滿分）
  - 中間區域紙屑分數較低（期中經驗）
  - **策略啟示：** 優先清掃角落 4 區，中間區視時間決定

- **紅色禁區**
  - 碰到會扣分/失格

#### 關鍵約束
- 🚫 **無編碼器** - 無法精確航位推算
- 🚪 **必須離場** - 未離場視為失敗

### 1.2 場地資訊（根據期中經驗）
- 📍 **紙屑分布**：集中在 5 個區域（4 個角落不貼牆 + 中央），總計約 80 片
- ⬛ **黑色膠帶標記**：紙屑集中區域會用黑色膠帶標示
- 🔴 **紅色膠帶標記**：紅色禁區會用紅色膠帶標示
- 📐 **場地形狀**：方形場地（期中），期末應類似

### 1.3 可用資源
- ✅ **USB Camera** - 640x480 @ 30 FPS，低位安裝可看到近處紙屑與整個場地
- ✅ **超聲波 × 2** - 左右測距（需改非阻塞）
- ✅ **差動驅動** - 已驗證穩定
- ✅ **Raspberry Pi 4** - OpenCV 實時處理

---

## 二、核心策略：區域目標導向清掃

### 2.1 關鍵洞察

**膠帶標記 = 導航地標 + 目標指示**

❌ **傳統盲目覆蓋**：
- 沿牆/螺旋/Zigzag 浪費時間
- 無編碼器會累積誤差
- 難以返航離場

✅ **智慧目標導向**：
- 🎯 **黑色膠帶 = 紙屑區域**（直接前往清掃）
- 🚫 **紅色膠帶 = 禁區**（避開）
- 📷 **視覺確認 = 閉環控制**（不累積誤差）
- ⚡ **效率優先**：直達目標，不做無用功

### 2.2 三階段策略

```
┌─────────────────────────────────────────────────┐
│ Phase 1: 入場全域掃描 (5-10 秒)                 │
│ ─────────────────────────────────────────────   │
│ 目標：識別黑色紙屑區、紅色禁區、入口特徵       │
│                                                 │
│ 1. 在入口停留，360° 原地旋轉掃描               │
│ 2. 偵測並記錄「黑色膠帶區域」（紙屑目標）     │
│    → 記錄方向、距離、大小                      │
│ 3. 偵測並記錄「紅色膠帶區域」（禁區）         │
│    → 記錄方向、距離（用於避障）               │
│ 4. 記錄入口視覺特徵（SIFT/ORB，用於返回）     │
│ 5. 規劃區域訪問順序（由近到遠、避開紅區）     │
└─────────────────────────────────────────────────┘
         ↓
┌─────────────────────────────────────────────────┐
│ Phase 2: 區域導向清掃 (主要時間)               │
│ ─────────────────────────────────────────────   │
│ 目標：逐一前往黑色標記區域，清掃紙屑           │
│                                                 │
│ 【區域間導航】                                  │
│ 1. 選擇下一個未清掃的黑色區域                   │
│ 2. 視覺導航前往（持續追蹤黑色膠帶）           │
│ 3. 過程中即時紅色避障（視覺+超聲波）           │
│                                                 │
│ 【區域內清掃】                                  │
│ 1. 到達黑色區域後，啟動吸塵器                   │
│ 2. 視覺識別紙屑（白色/淺色物體）               │
│ 3. 逐個靠近並清掃紙屑                           │
│ 4. 小範圍移動確保完全清掃                       │
│ 5. 視覺確認無紙屑 → 前往下一區域               │
└─────────────────────────────────────────────────┘
         ↓
┌─────────────────────────────────────────────────┐
│ Phase 3: 視覺返航 (10-15 秒)                   │
│ ─────────────────────────────────────────────   │
│ 目標：快速返回出口離場                         │
│                                                 │
│ 1. 所有區域清掃完成 → 旋轉尋找入口特徵         │
│ 2. 視覺特徵匹配（SIFT matching）               │
│ 3. 對準入口方向並前進                           │
│ 4. 視覺確認到達 → 停止                         │
└─────────────────────────────────────────────────┘
```

---

## 三、技術實現

### 3.1 視覺偵測系統

#### 黑色膠帶區域偵測（紙屑目標區）
```python
class BlackTapeZoneDetector:
    """偵測黑色膠帶標記的紙屑集中區域"""

    def __init__(self):
        # 黑色 HSV 範圍
        self.black_lower = np.array([0, 0, 0])
        self.black_upper = np.array([180, 255, 50])

        # 已發現的區域
        self.zones = []

    def detect_zones(self, frame, robot_heading=0):
        """
        偵測黑色膠帶區域並記錄

        Args:
            frame: 當前影像
            robot_heading: 當前朝向角度 (度)

        Returns:
            zones: 黑色區域列表 [(angle, distance, size), ...]
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 黑色遮罩
        black_mask = cv2.inRange(hsv, self.black_lower, self.black_upper)

        # 形態學處理（填補斷裂的膠帶）
        kernel = np.ones((5, 5), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        # 找輪廓
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        detected_zones = []
        h, w = frame.shape[:2]

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # 過濾太小的雜訊（膠帶應該有一定大小）
            if area < 800:
                continue

            # 計算重心
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # 計算相對角度（畫面中心 = 正前方）
            angle_offset = (cx - w/2) / (w/2) * 30  # 假設相機 FOV 60°
            absolute_angle = (robot_heading + angle_offset) % 360

            # 估算距離（越靠下越近）
            distance = self._estimate_distance(area, cy, h)

            zone = {
                'angle': absolute_angle,
                'distance': distance,
                'size': area,
                'pixel_pos': (cx, cy),
                'cleaned': False
            }

            detected_zones.append(zone)

        return detected_zones

    def _estimate_distance(self, area, cy, frame_height):
        """基於視覺大小和垂直位置估算距離"""
        # 垂直位置因子（越靠近畫面底部越近）
        vertical_factor = cy / frame_height

        # 面積因子（需實際校準）
        if area > 3000:
            base_dist = 0.4  # 40cm
        elif area > 1500:
            base_dist = 0.8  # 80cm
        else:
            base_dist = 1.2  # 1.2m

        # 綜合修正
        distance = base_dist * (2.0 - vertical_factor)
        return max(0.2, distance)  # 最小 20cm

    def add_zones(self, new_zones):
        """將新偵測到的區域加入地圖（去重）"""
        for new_zone in new_zones:
            # 檢查是否已存在（角度相近 ±15°）
            is_duplicate = False
            for existing in self.zones:
                angle_diff = abs(new_zone['angle'] - existing['angle'])
                if angle_diff < 15 or angle_diff > 345:
                    is_duplicate = True
                    break

            if not is_duplicate:
                self.zones.append(new_zone)

    def get_nearest_unvisited_zone(self):
        """獲取最近的未清掃區域"""
        unvisited = [z for z in self.zones if not z['cleaned']]
        if not unvisited:
            return None
        return min(unvisited, key=lambda z: z['distance'])

    def mark_zone_cleaned(self, zone):
        """標記區域已清掃"""
        zone['cleaned'] = True


class RedTapeZoneDetector:
    """偵測紅色膠帶禁區"""

    def __init__(self):
        # 紅色 HSV 範圍（兩段，因為紅色跨越 0°）
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])

        self.red_zones = []

    def detect_red_zones(self, frame, robot_heading=0):
        """偵測紅色禁區"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 紅色遮罩（兩段合併）
        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # 形態學處理
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        detected_zones = []
        h, w = frame.shape[:2]

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            angle_offset = (cx - w/2) / (w/2) * 30
            absolute_angle = (robot_heading + angle_offset) % 360

            detected_zones.append({
                'angle': absolute_angle,
                'distance': self._estimate_distance(area, cy, h),
                'size': area,
                'pixel_pos': (cx, cy)
            })

        return detected_zones

    def _estimate_distance(self, area, cy, frame_height):
        """同黑色膠帶的距離估算"""
        vertical_factor = cy / frame_height
        if area > 3000:
            base_dist = 0.3
        elif area > 1500:
            base_dist = 0.7
        else:
            base_dist = 1.0
        return max(0.2, base_dist * (2.0 - vertical_factor))

    def is_red_ahead(self, frame):
        """即時偵測：前方是否有紅色（避障用）"""
        zones = self.detect_red_zones(frame, 0)

        for zone in zones:
            # 前方 ±20° 範圍內有紅色
            if abs(zone['angle']) < 20 and zone['distance'] < 0.5:
                return True, zone['angle']

        return False, None
```

#### 入口視覺特徵記錄
```python
class EntranceDetector:
    def __init__(self):
        self.entrance_features = None
        self.entrance_sift = cv2.SIFT_create()

    def memorize_entrance(self, frame):
        """記錄入口的視覺特徵"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 提取 SIFT 特徵點
        keypoints, descriptors = self.entrance_sift.detectAndCompute(gray, None)

        self.entrance_features = {
            'keypoints': keypoints,
            'descriptors': descriptors,
            'template': gray.copy()
        }

    def find_entrance_direction(self, frame):
        """
        在當前畫面中尋找入口方向

        Returns:
            angle: 入口相對角度 (度)，None 表示未找到
        """
        if self.entrance_features is None:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, desc = self.entrance_sift.detectAndCompute(gray, None)

        # 特徵匹配
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(self.entrance_features['descriptors'], desc, k=2)

        # Lowe's ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        if len(good_matches) < 10:  # 匹配點太少
            return None

        # 計算匹配點的平均位置
        h, w = frame.shape[:2]
        avg_x = np.mean([kp[m.trainIdx].pt[0] for m in good_matches])

        # 轉換為角度
        angle_offset = (avg_x - w/2) / (w/2) * 30  # 視野 60°

        return angle_offset
```

### 3.2 紙屑偵測與定位

```python
class PaperDetector:
    def __init__(self):
        # 白色/淺色紙屑的 HSV 範圍
        self.paper_lower = np.array([0, 0, 180])
        self.paper_upper = np.array([180, 50, 255])

    def detect_papers(self, frame):
        """
        偵測紙屑位置

        Returns:
            papers: [(x, y, size), ...] 相對位置
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 白色遮罩
        paper_mask = cv2.inRange(hsv, self.paper_lower, self.paper_upper)

        # 形態學處理
        kernel = np.ones((5,5), np.uint8)
        paper_mask = cv2.morphologyEx(paper_mask, cv2.MORPH_CLOSE, kernel)
        paper_mask = cv2.morphologyEx(paper_mask, cv2.MORPH_OPEN, kernel)

        # 找輪廓
        contours, _ = cv2.findContours(paper_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        papers = []
        h, w = frame.shape[:2]

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # 過濾大小 (調整範圍)
            if 200 < area < 3000:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # 計算相對位置
                    rel_x = (cx - w/2) / (w/2)  # -1 ~ +1
                    rel_y = (cy - h/2) / (h/2)

                    papers.append({
                        'rel_x': rel_x,
                        'rel_y': rel_y,
                        'size': area,
                        'pixel_pos': (cx, cy)
                    })

        return papers

    def get_nearest_paper(self, papers):
        """取得最近的紙屑（畫面最下方）"""
        if not papers:
            return None
        return max(papers, key=lambda p: p['rel_y'])
```

### 3.3 智能任務管理器

```python
class MissionPlanner:
    """
    智能任務規劃與時間管理

    根據競賽規則動態調整策略：
    - 時間分級距制
    - < 30 片紙屑 → 時間分打八折
    - 中間區域分數低 → 優先角落
    """

    def __init__(self):
        # 競賽規則參數
        self.PAPER_TOTAL = 80           # 總紙屑數
        self.PAPER_PENALTY_THRESHOLD = 30  # 低於此數量時間分打八折

        # 時間級距（需根據實際規則調整）
        self.TIME_TIER_1 = 120  # 2:00 (滿分)
        self.TIME_TIER_2 = 150  # 2:30 (次級)
        self.TIME_TIER_3 = 180  # 3:00 (再次級)

        self.RETURN_TIME_MARGIN = 20  # 返航安全餘裕（秒）

        # 區域優先級（角落 > 中間）
        self.ZONE_PRIORITY = {
            'corner': 1.0,    # 角落區域優先級高
            'center': 0.5     # 中間區域優先級低（分數低）
        }

        # 任務狀態
        self.mission_start_time = None
        self.cleaned_papers = 0
        self.visited_zones = set()  # 已訪問區域 ID

    def classify_zone(self, zone):
        """
        分類區域（角落 vs 中間）

        簡化：假設中間區域角度在 45°, 135°, 225°, 315° 附近
        """
        angle = zone['angle']

        # 檢查是否接近中間（假設中間在 0° 方向，需根據實際調整）
        center_angles = [0, 90, 180, 270]

        for center_angle in center_angles:
            angle_diff = abs(angle - center_angle)
            if angle_diff < 20:  # ±20° 範圍視為可能是中間
                # 進一步檢查距離（中間應該較遠）
                if zone['distance'] > 1.0:
                    return 'center'

        return 'corner'

    def should_return_now(self, current_time, zones, cleaned_papers):
        """
        決策：是否應該立即返航

        Returns:
            (should_return: bool, reason: str)
        """
        elapsed = current_time - self.mission_start_time
        remaining = self.TIME_TIER_1 - self.RETURN_TIME_MARGIN - elapsed

        unvisited = [z for z in zones if not z['cleaned']]

        # 1. 時間緊迫 - 接近第一級距
        if remaining < 0:
            # 檢查是否達到 30 片門檻
            if cleaned_papers >= self.PAPER_PENALTY_THRESHOLD:
                return True, f"時間即將超過 Tier 1，已達 {cleaned_papers} 片，立即返航"
            else:
                # 未達 30 片，會被打八折，評估是否值得繼續
                if remaining > -20:  # 還有少許時間
                    return False, f"雖超時但未達 30 片，繼續清掃"
                else:
                    return True, f"超時過多且僅 {cleaned_papers} 片，返航止損"

        # 2. 所有角落區域已訪問 + 紙屑充足
        corner_zones = [z for z in zones if self.classify_zone(z) == 'corner']
        unvisited_corners = [z for z in corner_zones if not z['cleaned']]

        if not unvisited_corners and cleaned_papers >= self.PAPER_PENALTY_THRESHOLD:
            return True, f"所有角落已訪問，{cleaned_papers} 片充足，返航"

        # 3. 所有區域已訪問
        if not unvisited:
            return True, f"所有區域已訪問，{cleaned_papers} 片，返航"

        # 4. 時間即將進入下一級距
        if elapsed > self.TIME_TIER_2 - self.RETURN_TIME_MARGIN:
            return True, f"接近 Tier 2 邊界，{cleaned_papers} 片，返航"

        return False, None

    def select_next_zone(self, zones, current_time):
        """
        智能選擇下一個區域

        策略：
        1. 優先訪問角落（分數高）
        2. 確保所有區域都至少訪問一次
        3. 時間緊迫時選最近的
        """
        elapsed = current_time - self.mission_start_time
        remaining = self.TIME_TIER_1 - self.RETURN_TIME_MARGIN - elapsed

        unvisited = [z for z in zones if not z['cleaned']]
        if not unvisited:
            return None

        # 時間緊迫（< 40秒）：選最近的未訪問區域
        if remaining < 40:
            return min(unvisited, key=lambda z: z['distance'])

        # 時間充足：優先選角落區域
        corner_zones = [z for z in unvisited if self.classify_zone(z) == 'corner']

        if corner_zones:
            # 在角落中選最近的
            return min(corner_zones, key=lambda z: z['distance'])
        else:
            # 角落都訪問完了，選中間區域
            return min(unvisited, key=lambda z: z['distance'])

    def get_mission_status(self, current_time):
        """獲取任務狀態摘要"""
        elapsed = current_time - self.mission_start_time

        # 判斷當前時間級距
        if elapsed < self.TIME_TIER_1:
            time_tier = "Tier 1 (滿分)"
        elif elapsed < self.TIME_TIER_2:
            time_tier = "Tier 2 (次級)"
        elif elapsed < self.TIME_TIER_3:
            time_tier = "Tier 3 (再次級)"
        else:
            time_tier = "超時"

        # 檢查紙屑懲罰
        paper_penalty = "" if self.cleaned_papers >= self.PAPER_PENALTY_THRESHOLD else " [⚠️ 八折]"

        return {
            'elapsed': elapsed,
            'time_tier': time_tier,
            'paper_count': self.cleaned_papers,
            'paper_penalty': paper_penalty,
            'visited_count': len(self.visited_zones)
        }
```

### 3.4 融合避障系統

```python
class FusedObstacleAvoidance:
    """
    融合視覺紅色偵測與超聲波測距的平滑避障

    策略：
    - 視覺紅色：高優先級，緊急避障
    - 超聲波：平滑修正，防止撞牆
    """

    def __init__(self):
        # 超聲波避障參數
        self.ULTRASONIC_SAFE_DISTANCE = 30  # cm（開始修正）
        self.ULTRASONIC_DANGER_DISTANCE = 15  # cm（強制減速）
        self.MAX_CORRECTION_ANGULAR = 0.5   # 最大角速度修正

    def compute_safe_command(self, base_cmd, red_detector, sensor_data, frame):
        """
        計算融合避障後的安全指令

        Args:
            base_cmd: 基礎導航指令（來自狀態機）
            red_detector: 紅色偵測器
            sensor_data: 超聲波數據
            frame: 當前影像

        Returns:
            VehicleCommand: 修正後的安全指令
        """
        linear = base_cmd.linear
        angular = base_cmd.angular
        vacuum = base_cmd.vacuum_motor

        # 1. 視覺紅色避障（最高優先 - 覆寫所有指令）
        is_red, red_angle = red_detector.is_red_ahead(frame)
        if is_red and linear > 0:  # 只在前進時避紅
            # 緊急避障：後退 + 大角度轉向
            return VehicleCommand(-0.3, 0.8 if red_angle > 0 else -0.8, vacuum)

        # 2. 超聲波平滑避牆
        left_dist = sensor_data.left_distance if sensor_data else 100
        right_dist = sensor_data.right_distance if sensor_data else 100

        # 計算左右修正量（越近修正越大）
        left_correction = 0
        right_correction = 0

        if left_dist < self.ULTRASONIC_SAFE_DISTANCE:
            # 左側太近，向右修正
            left_correction = (self.ULTRASONIC_SAFE_DISTANCE - left_dist) / self.ULTRASONIC_SAFE_DISTANCE
            left_correction *= self.MAX_CORRECTION_ANGULAR

        if right_dist < self.ULTRASONIC_SAFE_DISTANCE:
            # 右側太近，向左修正
            right_correction = -(self.ULTRASONIC_SAFE_DISTANCE - right_dist) / self.ULTRASONIC_SAFE_DISTANCE
            right_correction *= self.MAX_CORRECTION_ANGULAR

        # 融合修正
        total_correction = left_correction + right_correction
        modified_angular = angular + total_correction

        # 3. 接近危險距離時減速
        min_dist = min(left_dist, right_dist)
        speed_factor = 1.0

        if min_dist < self.ULTRASONIC_DANGER_DISTANCE:
            speed_factor = 0.4  # 減速到 40%
        elif min_dist < self.ULTRASONIC_SAFE_DISTANCE:
            # 線性減速
            speed_factor = 0.4 + 0.6 * (min_dist - self.ULTRASONIC_DANGER_DISTANCE) / \
                          (self.ULTRASONIC_SAFE_DISTANCE - self.ULTRASONIC_DANGER_DISTANCE)

        modified_linear = linear * speed_factor

        # 限制角速度範圍
        modified_angular = np.clip(modified_angular, -1.0, 1.0)

        return VehicleCommand(modified_linear, modified_angular, vacuum)
```

### 3.5 區域導向控制狀態機（完整版）

```python
from enum import Enum
import time
import numpy as np

class State(Enum):
    INIT = 0
    SCANNING = 1              # 360° 掃描建立區域地圖
    NAVIGATING_TO_ZONE = 2    # 前往黑色區域
    CLEANING_ZONE = 3         # 清掃區域內紙屑
    AVOIDING_RED = 4          # 緊急避紅
    RETURNING = 5             # 返回出口
    FINISHED = 6

class ZoneBasedAutonomousController:
    """
    基於黑色膠帶區域的智能目標導向控制器

    特色：
    - 智能任務管理（時間級距、紙屑門檻、區域優先級）
    - 融合避障（視覺紅色 + 超聲波平滑避牆）
    - 視覺閉環導航（無累積誤差）
    """

    def __init__(self):
        self.state = State.INIT

        # 視覺偵測模組
        self.black_zone_detector = BlackTapeZoneDetector()
        self.red_zone_detector = RedTapeZoneDetector()
        self.paper_detector = PaperDetector()
        self.entrance_detector = EntranceDetector()

        # 新增：任務管理器與融合避障
        self.mission_planner = MissionPlanner()
        self.obstacle_avoidance = FusedObstacleAvoidance()

        # 導航狀態
        self.scan_angle = 0
        self.current_target_zone = None

        # 時間記錄
        self.start_time = None
        self.phase_start_time = None
        self.zone_entry_time = None
        self.zone_paper_count = 0  # 本區域清掃數量

    def run(self):
        """主控制迴圈"""
        self.start_time = time.time()

        while True:
            # 取得感測器資料
            frame = self.camera.get_frame()
            sensor_data = self.arduino.receive_sensor_data()

            # 狀態機
            if self.state == State.INIT:
                self._state_init(frame)

            elif self.state == State.SCANNING:
                self._state_scanning(frame)

            elif self.state == State.NAVIGATING_TO_ZONE:
                self._state_navigating_to_zone(frame, sensor_data)

            elif self.state == State.CLEANING_ZONE:
                self._state_cleaning_zone(frame)

            elif self.state == State.AVOIDING_RED:
                self._state_avoiding_red(frame)

            elif self.state == State.RETURNING:
                self._state_returning(frame)

            elif self.state == State.FINISHED:
                break

            time.sleep(0.05)  # 20 Hz

        print(f"[完成] 總時間：{time.time() - self.start_time:.1f} 秒")
        print(f"[完成] 清掃紙屑：{self.cleaned_papers} 個")

    def _state_init(self, frame):
        """初始化：記錄入口特徵"""
        print("[INIT] 記錄入口視覺特徵...")

        # 記錄入口（用於返航）
        self.entrance_detector.memorize_entrance(frame)

        # 進入掃描階段
        self.state = State.SCANNING
        self.scan_angle = 0
        self.phase_start_time = time.time()
        print("[INIT→SCAN] 開始 360° 掃描")

    def _state_scanning(self, frame):
        """掃描階段：360° 建立黑色區域與紅色禁區地圖"""

        # 原地慢速旋轉
        cmd = VehicleCommand(0, 0.3, vacuum_motor=False)
        self.robot.send_command(cmd)

        # 偵測黑色紙屑區域
        black_zones = self.black_zone_detector.detect_zones(frame, self.scan_angle)
        self.black_zone_detector.add_zones(black_zones)

        # 偵測紅色禁區
        red_zones = self.red_zone_detector.detect_red_zones(frame, self.scan_angle)
        # 記錄但不需要去重（只用於避障）

        # 更新掃描角度（0.3 rad/s * 0.05s ≈ 0.86° per cycle）
        self.scan_angle += 0.86

        # 掃描完成條件
        if self.scan_angle >= 360 or time.time() - self.phase_start_time > 10:
            # 停止旋轉
            self.robot.send_command(VehicleCommand(0, 0, False))
            time.sleep(0.5)

            # 啟動任務計時
            self.mission_planner.mission_start_time = time.time()

            zones = self.black_zone_detector.zones
            print(f"[SCAN] 完成！發現 {len(zones)} 個黑色區域")

            # 分類區域
            corner_count = sum(1 for z in zones if self.mission_planner.classify_zone(z) == 'corner')
            center_count = len(zones) - corner_count
            print(f"[SCAN] 角落: {corner_count} | 中間: {center_count}")

            # 智能選擇第一個區域（優先角落）
            self.current_target_zone = self.mission_planner.select_next_zone(zones, time.time())

            if self.current_target_zone:
                zone_type = self.mission_planner.classify_zone(self.current_target_zone)
                self.state = State.NAVIGATING_TO_ZONE
                self.phase_start_time = time.time()
                print(f"[SCAN→NAV] 前往 {zone_type} 區域 (角度: {self.current_target_zone['angle']:.0f}°)")
            else:
                # 沒找到黑色區域，直接返回
                print("[SCAN→RETURN] 未發現紙屑區域，返航")
                self.state = State.RETURNING

    def _state_navigating_to_zone(self, frame, sensor_data):
        """導航階段：前往目標黑色區域（加入融合避障與時間管理）"""

        # 1. 檢查是否應該返航（時間管理）
        should_return, reason = self.mission_planner.should_return_now(
            time.time(),
            self.black_zone_detector.zones,
            self.mission_planner.cleaned_papers
        )

        if should_return:
            print(f"[NAV→RETURN] {reason}")
            self._print_mission_status()
            self.state = State.RETURNING
            return

        # 2. 視覺閉環導航
        black_zones = self.black_zone_detector.detect_zones(frame, 0)

        if black_zones:
            # 找最接近目標方向的區域
            closest_zone = min(black_zones, key=lambda z: abs(z['angle']))

            # 到達區域判斷
            if closest_zone['distance'] < 0.3 and abs(closest_zone['angle']) < 15:
                print(f"[NAV→CLEAN] 到達區域！")
                self.state = State.CLEANING_ZONE
                self.zone_entry_time = time.time()
                self.zone_paper_count = 0
                self.robot.send_command(VehicleCommand(0, 0, vacuum_motor=True))
                time.sleep(0.3)
                return

            # 計算基礎導航指令
            angle_error = closest_zone['angle']
            if abs(angle_error) > 10:
                base_cmd = VehicleCommand(0.2, angle_error / 30, False)
            else:
                base_cmd = VehicleCommand(0.4, 0, False)

        else:
            # 視野中沒黑色區域，盲目前進
            base_cmd = VehicleCommand(0.3, 0, False)

            # 超時保護
            if time.time() - self.phase_start_time > 5:
                print("[NAV] 超時，重新掃描")
                self.state = State.SCANNING
                self.scan_angle = 0
                return

        # 3. 融合避障修正
        safe_cmd = self.obstacle_avoidance.compute_safe_command(
            base_cmd,
            self.red_zone_detector,
            sensor_data,
            frame
        )

        self.robot.send_command(safe_cmd)

    def _state_cleaning_zone(self, frame):
        """清掃階段：在黑色區域內清掃紙屑（加入計數與時間管理）"""

        # 偵測紙屑
        papers = self.paper_detector.detect_papers(frame)

        if papers:
            # 更新計數（簡化：每看到一次加 1，實際可能需要去重）
            self.zone_paper_count += len(papers)

            # 找最近的紙屑
            nearest = self.paper_detector.get_nearest_paper(papers)

            # 對準並靠近
            angle_error = nearest['rel_x']

            if abs(angle_error) > 0.15:
                cmd = VehicleCommand(0.15, angle_error * 0.5, vacuum_motor=True)
            else:
                cmd = VehicleCommand(0.25, 0, vacuum_motor=True)

            self.robot.send_command(cmd)

        else:
            # 沒看到紙屑，小範圍搜尋
            elapsed = time.time() - self.zone_entry_time

            if elapsed < 5:
                # 前 5 秒：緩慢前進
                cmd = VehicleCommand(0.2, 0, vacuum_motor=True)
            elif elapsed < 8:
                # 5-8 秒：旋轉搜尋
                cmd = VehicleCommand(0, 0.3, vacuum_motor=True)
            else:
                # 超過 8 秒，清掃完成
                self.mission_planner.cleaned_papers += self.zone_paper_count
                self.mission_planner.visited_zones.add(id(self.current_target_zone))

                print(f"[CLEAN] 本區: {self.zone_paper_count} 片 | 總計: {self.mission_planner.cleaned_papers} 片")

                # 標記區域已清掃
                self.black_zone_detector.mark_zone_cleaned(self.current_target_zone)

                # 關閉吸塵器
                self.robot.send_command(VehicleCommand(0, 0, vacuum_motor=False))

                # 智能選擇下一區域
                self.current_target_zone = self.mission_planner.select_next_zone(
                    self.black_zone_detector.zones,
                    time.time()
                )

                # 檢查是否應該返航
                should_return, reason = self.mission_planner.should_return_now(
                    time.time(),
                    self.black_zone_detector.zones,
                    self.mission_planner.cleaned_papers
                )

                if should_return:
                    print(f"[CLEAN→RETURN] {reason}")
                    self._print_mission_status()
                    self.state = State.RETURNING
                elif self.current_target_zone:
                    zone_type = self.mission_planner.classify_zone(self.current_target_zone)
                    self.state = State.NAVIGATING_TO_ZONE
                    self.phase_start_time = time.time()
                    print(f"[CLEAN→NAV] 前往 {zone_type} 區域")
                else:
                    # 所有區域清完
                    print("[CLEAN→RETURN] 所有區域完成")
                    self._print_mission_status()
                    self.state = State.RETURNING

            self.robot.send_command(cmd)

    def _print_mission_status(self):
        """輸出任務狀態摘要"""
        status = self.mission_planner.get_mission_status(time.time())
        print(f"[狀態] 時間: {status['elapsed']:.0f}s ({status['time_tier']})")
        print(f"[狀態] 紙屑: {status['paper_count']}/80{status['paper_penalty']}")
        print(f"[狀態] 已訪問: {status['visited_count']}/{len(self.black_zone_detector.zones)} 區域")

    def _state_returning(self, frame):
        """返航階段：視覺導航回到入口（加入融合避障）"""

        # 尋找入口特徵
        entrance_angle = self.entrance_detector.find_entrance_direction(frame)

        if entrance_angle is None:
            # 未找到入口，旋轉搜尋
            print("[RETURN] 搜尋入口...")
            base_cmd = VehicleCommand(0, 0.4, False)
        elif abs(entrance_angle) > 10:
            # 轉向對準入口
            base_cmd = VehicleCommand(0.2, entrance_angle / 30, False)
        else:
            # 對準入口，前進
            base_cmd = VehicleCommand(0.5, 0, False)

        # 融合避障（包含紅色避障）
        safe_cmd = self.obstacle_avoidance.compute_safe_command(
            base_cmd,
            self.red_zone_detector,
            None,  # 返航時可能沒有sensor_data
            frame
        )

        self.robot.send_command(safe_cmd)

        # TODO: 判斷到達入口（特徵匹配度高 + 距離近）

        # 超時保護
        if time.time() - self.start_time > 180:  # 3分鐘
            print("[RETURN→FINISH] 超時，強制結束")
            self._print_mission_status()
            self.state = State.FINISHED
```

---

## 四、技術創新總結

### 4.1 不需要 YOLO
- ✅ 傳統 HSV 顏色偵測足夠準確
- ✅ 黑色、紅色、白色在各種光線下都容易辨識
- ✅ Pi4 處理速度快（30 FPS）
- ✅ 易於調試與現場校準

### 4.2 融合避障系統
```
層級 1（最高優先）：視覺紅色偵測 → 緊急避障
層級 2：超聲波測距 → 平滑避牆修正
層級 3：基礎導航指令 → 目標追蹤
```

**優勢：**
- 無視覺死角撞牆
- 平滑控制不急停
- 紅色優先確保不扣分

### 4.3 智能時間管理
| 情境 | 紙屑數 | 時間 | 策略 |
|------|--------|------|------|
| 角落清完 | ≥ 30 | < Tier 1 | 返航拿滿分 |
| 角落清完 | < 30 | < Tier 2 | 清中間區拼 30 片 |
| 時間緊迫 | ≥ 30 | 接近 Tier 1 | 立即返航 |
| 時間緊迫 | < 30 | 接近 Tier 2 | 評估是否值得繼續 |

### 4.4 區域優先級
- **角落區（4個）：** 高優先，先訪問（分數高）
- **中間區（1個）：** 低優先，視時間決定（分數低）
- **確保覆蓋：** 所有區域至少訪問一次

---

## 五、實作優先級（更新版）

### Phase 1: 視覺偵測基礎（2-3 天）
- [ ] **相機視角確認** - 拍攝黑/紅膠帶、白紙屑照片
- [ ] **HSV 標定工具** - 開發互動式 HSV 調整工具
- [ ] 黑色膠帶偵測調校（HSV 範圍、面積閾值）
- [ ] 紅色膠帶偵測調校（HSV 兩段範圍）
- [ ] 紙屑偵測調校（白色/淺色、大小過濾）
- [ ] 入口特徵記錄（SIFT/ORB）
- [ ] 相機視角確認與參數校準

### Phase 2: 掃描與區域地圖（2 天）
- [ ] 360° 旋轉掃描邏輯
- [ ] 黑色區域記錄與去重
- [ ] 紅色禁區記錄
- [ ] 區域優先級排序（由近到遠）

### Phase 3: 區域導航（2-3 天）
- [ ] 視覺伺服導航（持續追蹤黑色膠帶）
- [ ] 到達判斷邏輯
- [ ] 紅色即時避障
- [ ] 超聲波輔助避障

### Phase 4: 區域內清掃（2 天）
- [ ] 紙屑視覺追蹤
- [ ] 對準與靠近控制
- [ ] 小範圍搜尋策略
- [ ] 清掃完成判斷

### Phase 5: 返航與整合（2-3 天）
- [ ] 入口視覺特徵匹配
- [ ] 返航導航邏輯
- [ ] 完整流程整合測試
- [ ] 參數優化（速度、閾值、超時）

### Phase 6: 競賽準備（1-2 天）
- [ ] 模擬場地測試
- [ ] 極端情況測試（光線、膠帶不清楚等）
- [ ] 性能優化（提升速度、減少清掃時間）

**總預估時間：** 11-15 天

---

## 五、關鍵優勢

### 5.1 相比盲目覆蓋（沿牆/螺旋/Zigzag）
- ✅ **效率優先** - 直達紙屑集中區域，不做無用功
- ✅ **無累積誤差** - 視覺閉環導航，持續修正
- ✅ **可靠返航** - 入口視覺特徵匹配
- ✅ **適合計時賽** - 最小化移動距離與時間

### 5.2 相比複雜 SLAM
- ✅ **實作簡單** - 不需要複雜建圖與定位
- ✅ **快速啟動** - 10 秒掃描即可開始任務
- ✅ **低計算量** - Pi4 可穩定運行（已驗證期中遙控）
- ✅ **魯棒性高** - 即使部分區域偵測失敗，可繼續其他區域

### 5.3 利用競賽提供的資訊
- ✅ **黑色膠帶 = 導航目標** - 不需自己找紙屑位置
- ✅ **紅色膠帶 = 明確禁區** - 視覺偵測即可避開
- ✅ **紙屑集中** - 區域內搜尋更高效
- ✅ **場地固定** - 入口視覺特徵穩定

### 5.4 視覺閉環控制
- ✅ **持續視覺確認** - 不累積誤差
- ✅ **即時反應** - 紅色避障延遲 < 100ms
- ✅ **自我修正** - 偏離目標可立即視覺糾正

---

## 六、風險與應對

| 風險 | 可能性 | 影響 | 應對策略 |
|------|--------|------|---------|
| 光線變化影響 HSV 偵測 | 中 | 高 | 1. 自動曝光補償<br>2. 現場校準 HSV 範圍<br>3. 備用：放寬閾值 |
| 黑色膠帶偵測失敗 | 低 | 高 | 1. 形態學處理填補斷裂<br>2. 降低面積閾值<br>3. 備用：掃描時多角度記錄 |
| 紙屑顏色與地面相近 | 中 | 中 | 1. 調整 HSV/亮度範圍<br>2. 使用邊緣檢測輔助<br>3. 小範圍盲目清掃 |
| 入口特徵匹配失敗 | 低 | 中 | 1. 使用多種特徵（SIFT + 顏色）<br>2. 備用：返回初始大致位置 |
| 紅色避障過於敏感 | 中 | 低 | 調整距離閾值（50cm → 30cm） |
| 區域清掃超時 | 低 | 低 | 設定合理超時（8 秒），避免卡死 |

---

## 七、技術創新點

1. **膠帶標記雙重利用**
   - 黑色 = 目標導航點
   - 紅色 = 避障標誌
   - 充分利用競賽提供的視覺資訊

2. **區域目標導向策略**
   - 不同於傳統覆蓋式清掃
   - 直達目標，效率優先
   - 適合計時競賽

3. **視覺閉環導航**
   - 無需編碼器與精確定位
   - 持續視覺修正，不累積誤差
   - 魯棒性高

4. **分層狀態機設計**
   - 清晰的狀態轉換
   - 易於調試與優化
   - 可應對突發狀況（紅色避障）

---

## 八、與您期中方案的對比

| 項目 | 期中（遙控） | 期末（自走） |
|------|------------|------------|
| 控制方式 | 人工遙控 | 自主視覺導航 |
| 視覺功能 | MJPG 串流（人眼判斷） | OpenCV 自動偵測 |
| 導航策略 | 無（人工決策） | 區域目標導向 |
| 避障方式 | 人眼判斷 | 紅色視覺 + 超聲波 |
| 相同基礎 | 差動驅動、Serial 通訊、穩定硬體 | ✅ 完全相容 |

**優勢：** 期中驗證的硬體與通訊架構直接沿用，只需新增視覺決策層！

---

## 九、下一步行動

建議按以下順序推進：

### 立即開始（Phase 1）
1. **相機視角確認**
   - 拍攝地面黑色膠帶、紅色膠帶、白色紙屑的照片
   - 確認視野範圍與清晰度

2. **HSV 範圍標定**
   - 用 OpenCV 測試工具找出最佳 HSV 範圍
   - 黑色、紅色、白色分別標定

3. **基礎偵測模組實作**
   - 實現 `BlackTapeZoneDetector`
   - 實現 `RedTapeZoneDetector`
   - 實現 `PaperDetector`
   - 單獨測試各模組準確度

### 建議
如果您同意這個方案，我可以開始實作第一個視覺偵測模組的程式碼。我們可以先從黑色膠帶偵測開始，因為這是整個系統的核心！
