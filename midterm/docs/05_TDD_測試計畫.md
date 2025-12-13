# TDD - 測試驅動開發計畫
**Test-Driven Development Plan**

---

## 文件資訊

| 項目 | 內容 |
|------|------|
| 專案名稱 | 機電小車遙控系統 |
| 文件版本 | 1.0 |
| 建立日期 | 2025-10-31 |
| 最後更新 | 2025-10-31 |
| 作者 | Mechatronics Team |
| 硬體平台 | Raspberry Pi 4 + Arduino Uno |

**修訂歷史**：
- v1.0 (2025-10-31): 初始版本

---

## 1. 測試策略

### 1.1 測試金字塔

本專案採用經典的測試金字塔策略：

```
        ╱╲
       ╱  ╲        ← Level 3: 系統測試 (10%)
      ╱────╲         端對端整合驗證
     ╱      ╲
    ╱────────╲     ← Level 2: 整合測試 (30%)
   ╱          ╲      模組間介面測試
  ╱────────────╲
 ╱              ╲  ← Level 1: 單元測試 (60%)
╱────────────────╲   函式/類別級測試
```

### 1.2 測試層級定義

| 測試層級 | 範圍 | 目標 | 比例 | 執行頻率 |
|---------|------|------|------|---------|
| **單元測試** | 個別函式/類別 | 驗證基本邏輯正確性 | 60% | 每次提交 |
| **整合測試** | 模組間介面 | 驗證模組協作 | 30% | 每日 |
| **系統測試** | 完整系統 | 驗證需求符合度 | 10% | 每週 |

### 1.3 測試覆蓋率目標

| 項目 | 目標 | 工具 |
|------|------|------|
| Python 程式碼覆蓋率 | > 80% | `pytest-cov` |
| Arduino 程式碼覆蓋率 | > 70% | 手動驗證 |
| 功能需求覆蓋率 | 100% | 測試追溯矩陣 |

---

## 2. 單元測試 (Unit Tests)

### 2.1 Python 單元測試

#### 2.1.1 測試框架設定

```bash
# 安裝測試依賴
pip3 install pytest pytest-cov pytest-mock

# 執行測試
pytest tests/ --cov=raspberry_pi --cov-report=html

# 產生覆蓋率報告
open htmlcov/index.html
```

**目錄結構**：

```
raspberry_pi/
├── config.py
├── usb_24g_receiver.py
├── differential_drive.py
├── arduino_controller.py
└── tests/
    ├── __init__.py
    ├── test_usb_24g_receiver.py
    ├── test_differential_drive.py
    └── test_arduino_controller.py
```

---

#### 2.1.2 差動驅動單元測試 (test_differential_drive.py)

```python
#!/usr/bin/env python3
"""test_differential_drive.py - 差動驅動單元測試"""

import pytest
from differential_drive import DifferentialDrive, VehicleCommand, MotorCommand


class TestDifferentialDrive:
    """差動驅動演算法測試"""

    def setup_method(self):
        """測試前置作業"""
        self.drive = DifferentialDrive()

    # ========== TC-DD-001: 直線前進 ==========
    def test_forward_full_speed(self):
        """測試案例 TC-DD-001: 全速前進"""
        cmd = VehicleCommand(linear_velocity=1.0, angular_velocity=0.0)
        result = self.drive.convert(cmd)

        assert result.left_pwm == 255, "左輪 PWM 應為 255"
        assert result.right_pwm == 255, "右輪 PWM 應為 255"

    def test_forward_half_speed(self):
        """測試案例 TC-DD-002: 半速前進"""
        cmd = VehicleCommand(linear_velocity=0.5, angular_velocity=0.0)
        result = self.drive.convert(cmd)

        assert result.left_pwm == 127 or result.left_pwm == 128  # 允許四捨五入誤差
        assert result.right_pwm == 127 or result.right_pwm == 128

    # ========== TC-DD-003: 直線後退 ==========
    def test_backward_full_speed(self):
        """測試案例 TC-DD-003: 全速後退"""
        cmd = VehicleCommand(linear_velocity=-1.0, angular_velocity=0.0)
        result = self.drive.convert(cmd)

        assert result.left_pwm == -255, "左輪 PWM 應為 -255"
        assert result.right_pwm == -255, "右輪 PWM 應為 -255"

    # ========== TC-DD-004: 原地左轉 ==========
    def test_turn_left_in_place(self):
        """測試案例 TC-DD-004: 原地左轉"""
        cmd = VehicleCommand(linear_velocity=0.0, angular_velocity=-1.0)
        result = self.drive.convert(cmd)

        assert result.left_pwm == -255, "左輪應反轉"
        assert result.right_pwm == 255, "右輪應前進"

    # ========== TC-DD-005: 原地右轉 ==========
    def test_turn_right_in_place(self):
        """測試案例 TC-DD-005: 原地右轉"""
        cmd = VehicleCommand(linear_velocity=0.0, angular_velocity=1.0)
        result = self.drive.convert(cmd)

        assert result.left_pwm == 255, "左輪應前進"
        assert result.right_pwm == -255, "右輪應反轉"

    # ========== TC-DD-006: 前進左轉 ==========
    def test_forward_left_turn(self):
        """測試案例 TC-DD-006: 前進同時左轉"""
        cmd = VehicleCommand(linear_velocity=1.0, angular_velocity=-0.5)
        result = self.drive.convert(cmd)

        assert result.left_pwm < result.right_pwm, "左輪應比右輪慢"
        assert result.left_pwm == 127 or result.left_pwm == 128
        assert result.right_pwm == 255

    # ========== TC-DD-007: 前進右轉 ==========
    def test_forward_right_turn(self):
        """測試案例 TC-DD-007: 前進同時右轉"""
        cmd = VehicleCommand(linear_velocity=1.0, angular_velocity=0.5)
        result = self.drive.convert(cmd)

        assert result.left_pwm > result.right_pwm, "右輪應比左輪慢"
        assert result.left_pwm == 255
        assert result.right_pwm == 127 or result.right_pwm == 128

    # ========== TC-DD-008: 邊界值測試 ==========
    def test_clamping_positive_overflow(self):
        """測試案例 TC-DD-008: 正值溢位限制"""
        cmd = VehicleCommand(linear_velocity=1.0, angular_velocity=1.0)
        result = self.drive.convert(cmd)

        # linear + angular = 2.0 → 應被限制在 1.0
        assert result.left_pwm <= 255, "左輪不應超過 255"
        assert result.right_pwm <= 255, "右輪不應超過 255"

    def test_clamping_negative_overflow(self):
        """測試案例 TC-DD-009: 負值溢位限制"""
        cmd = VehicleCommand(linear_velocity=-1.0, angular_velocity=-1.0)
        result = self.drive.convert(cmd)

        # linear - angular = -2.0 → 應被限制在 -1.0
        assert result.left_pwm >= -255, "左輪不應低於 -255"
        assert result.right_pwm >= -255, "右輪不應低於 -255"

    # ========== TC-DD-010: 停止狀態 ==========
    def test_stop(self):
        """測試案例 TC-DD-010: 停止"""
        cmd = VehicleCommand(linear_velocity=0.0, angular_velocity=0.0)
        result = self.drive.convert(cmd)

        assert result.left_pwm == 0, "左輪應停止"
        assert result.right_pwm == 0, "右輪應停止"

    # ========== TC-DD-011: 死區過濾 ==========
    def test_deadzone_filtering(self):
        """測試案例 TC-DD-011: 搖桿死區過濾"""
        cmd = VehicleCommand(linear_velocity=0.05, angular_velocity=0.03)
        # 假設死區為 0.1
        result = self.drive.convert(cmd)

        # 預期死區內數值被過濾為 0
        assert abs(result.left_pwm) < 25, "死區內應接近 0"
        assert abs(result.right_pwm) < 25, "死區內應接近 0"
```

**預期測試結果**：

```
pytest tests/test_differential_drive.py -v

============== test session starts ==============
tests/test_differential_drive.py::TestDifferentialDrive::test_forward_full_speed PASSED    [ 9%]
tests/test_differential_drive.py::TestDifferentialDrive::test_forward_half_speed PASSED    [18%]
tests/test_differential_drive.py::TestDifferentialDrive::test_backward_full_speed PASSED   [27%]
tests/test_differential_drive.py::TestDifferentialDrive::test_turn_left_in_place PASSED    [36%]
tests/test_differential_drive.py::TestDifferentialDrive::test_turn_right_in_place PASSED   [45%]
tests/test_differential_drive.py::TestDifferentialDrive::test_forward_left_turn PASSED     [54%]
tests/test_differential_drive.py::TestDifferentialDrive::test_forward_right_turn PASSED    [63%]
tests/test_differential_drive.py::TestDifferentialDrive::test_clamping_positive_overflow PASSED [72%]
tests/test_differential_drive.py::TestDifferentialDrive::test_clamping_negative_overflow PASSED [81%]
tests/test_differential_drive.py::TestDifferentialDrive::test_stop PASSED                 [90%]
tests/test_differential_drive.py::TestDifferentialDrive::test_deadzone_filtering PASSED   [100%]

============== 11 passed in 0.25s ==============
```

---

#### 2.1.3 Serial 封包建構單元測試 (test_arduino_controller.py)

```python
#!/usr/bin/env python3
"""test_arduino_controller.py - Serial 通訊單元測試"""

import pytest
from arduino_controller import ArduinoController, MotorCommand


class TestSerialPacket:
    """Serial 封包建構測試"""

    def setup_method(self):
        """測試前置作業"""
        self.controller = ArduinoController(port='/dev/serial0', baudrate=57600)

    # ========== TC-SP-001: 全速前進封包 ==========
    def test_forward_packet(self):
        """測試案例 TC-SP-001: 全速前進封包"""
        cmd = MotorCommand(left_pwm=255, right_pwm=255, vacuum=False)
        packet = self.controller.build_packet(cmd)

        assert packet[0] == 0xAA, "Header 應為 0xAA"
        assert packet[1] == 0xFF, "Left PWM Low"
        assert packet[2] == 0x00, "Left PWM High"
        assert packet[3] == 0xFF, "Right PWM Low"
        assert packet[4] == 0x00, "Right PWM High"
        assert packet[5] == 0x00, "Flags (vacuum=0)"
        assert packet[6] == 0x00, "Checksum"
        assert packet[7] == 0x55, "Footer 應為 0x55"

    # ========== TC-SP-002: 後退封包 (負值 PWM) ==========
    def test_backward_packet(self):
        """測試案例 TC-SP-002: 全速後退封包"""
        cmd = MotorCommand(left_pwm=-255, right_pwm=-255, vacuum=False)
        packet = self.controller.build_packet(cmd)

        # -255 = 0xFF01 (2's complement)
        assert packet[0] == 0xAA
        assert packet[1] == 0x01, "Left PWM Low (-255)"
        assert packet[2] == 0xFF, "Left PWM High (-255)"
        assert packet[3] == 0x01, "Right PWM Low (-255)"
        assert packet[4] == 0xFF, "Right PWM High (-255)"
        assert packet[7] == 0x55

    # ========== TC-SP-003: 吸塵器開啟 ==========
    def test_vacuum_on_packet(self):
        """測試案例 TC-SP-003: 吸塵器開啟封包"""
        cmd = MotorCommand(left_pwm=127, right_pwm=127, vacuum=True)
        packet = self.controller.build_packet(cmd)

        assert packet[5] == 0x01, "Flags bit0 應為 1 (vacuum=1)"

    # ========== TC-SP-004: Checksum 驗證 ==========
    def test_checksum_calculation(self):
        """測試案例 TC-SP-004: Checksum 計算正確性"""
        cmd = MotorCommand(left_pwm=100, right_pwm=200, vacuum=False)
        packet = self.controller.build_packet(cmd)

        # 手動計算 Checksum
        expected_checksum = packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5]
        assert packet[6] == expected_checksum, "Checksum 不符"

    # ========== TC-SP-005: 封包長度驗證 ==========
    def test_packet_length(self):
        """測試案例 TC-SP-005: 封包長度必須為 8 bytes"""
        cmd = MotorCommand(left_pwm=0, right_pwm=0, vacuum=False)
        packet = self.controller.build_packet(cmd)

        assert len(packet) == 8, "封包長度必須為 8 bytes"

    # ========== TC-SP-006: 邊界值測試 ==========
    @pytest.mark.parametrize("left_pwm,right_pwm", [
        (255, 255),    # 最大正值
        (-255, -255),  # 最大負值
        (0, 0),        # 零
        (127, -127),   # 混合正負
        (-1, 1),       # 最小正負
    ])
    def test_boundary_values(self, left_pwm, right_pwm):
        """測試案例 TC-SP-006: 邊界值測試"""
        cmd = MotorCommand(left_pwm=left_pwm, right_pwm=right_pwm, vacuum=False)
        packet = self.controller.build_packet(cmd)

        # 驗證封包格式
        assert packet[0] == 0xAA
        assert packet[7] == 0x55

        # 驗證 Checksum
        checksum = packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5]
        assert packet[6] == checksum
```

---

#### 2.1.4 USB 遙控器輸入測試 (test_usb_24g_receiver.py)

```python
#!/usr/bin/env python3
"""test_usb_24g_receiver.py - USB 遙控器輸入測試"""

import pytest
from unittest.mock import Mock, patch
from usb_24g_receiver import USB24GReceiver, VehicleCommand


class TestUSB24GReceiver:
    """USB 遙控器輸入處理測試"""

    def setup_method(self):
        """測試前置作業"""
        self.receiver = USB24GReceiver()

    # ========== TC-USB-001: 搖桿正規化 ==========
    def test_normalize_joystick_max(self):
        """測試案例 TC-USB-001: 搖桿最大值正規化"""
        # 假設搖桿原始值範圍 -32767 ~ +32767
        normalized = self.receiver.normalize_axis(32767)
        assert pytest.approx(normalized, 0.01) == 1.0

    def test_normalize_joystick_min(self):
        """測試案例 TC-USB-002: 搖桿最小值正規化"""
        normalized = self.receiver.normalize_axis(-32767)
        assert pytest.approx(normalized, 0.01) == -1.0

    def test_normalize_joystick_center(self):
        """測試案例 TC-USB-003: 搖桿中心值正規化"""
        normalized = self.receiver.normalize_axis(0)
        assert normalized == 0.0

    # ========== TC-USB-004: 死區過濾 ==========
    def test_deadzone_filtering_inside(self):
        """測試案例 TC-USB-004: 死區內數值應被過濾為 0"""
        # 假設死區為 0.1，輸入 0.05
        filtered = self.receiver.apply_deadzone(0.05, deadzone=0.1)
        assert filtered == 0.0, "死區內應返回 0"

    def test_deadzone_filtering_outside(self):
        """測試案例 TC-USB-005: 死區外數值應保留"""
        filtered = self.receiver.apply_deadzone(0.5, deadzone=0.1)
        assert filtered != 0.0, "死區外應保留原值"

    # ========== TC-USB-006: 按鈕狀態 ==========
    @patch('pygame.event.get')
    def test_button_press(self, mock_event):
        """測試案例 TC-USB-006: 按鈕按下事件"""
        # 模擬按鈕按下事件
        mock_event.return_value = [
            Mock(type=10, button=0)  # pygame.JOYBUTTONDOWN, button 0
        ]

        cmd = self.receiver.get_command()
        assert cmd.vacuum_motor == True, "按鈕按下應切換吸塵器狀態"

    # ========== TC-USB-007: 軸反轉 ==========
    def test_invert_axis(self):
        """測試案例 TC-USB-007: Y 軸反轉（某些遙控器需要）"""
        # 假設 Y 軸需要反轉（向上推為正值）
        normalized = self.receiver.normalize_axis(-32767, invert=True)
        assert pytest.approx(normalized, 0.01) == 1.0
```

---

### 2.2 Arduino 單元測試

Arduino 無法直接執行自動化單元測試，採用**手動測試 + Serial 輸出驗證**。

#### 2.2.1 馬達控制單元測試 (test_motor_driver.ino)

```cpp
/*
 * test_motor_driver.ino - 馬達驅動單元測試
 *
 * 測試方式：上傳此程式至 Arduino，透過 Serial Monitor 觀察輸出
 */

#include "motor_driver.h"

MotorDriver motor(3, 5, 6, 11, 9, 10);  // ENA, IN1, IN2, ENB, IN3, IN4

void setup() {
  Serial.begin(115200);
  Serial.println("========== Motor Driver Unit Test ==========");

  delay(2000);  // 等待 2 秒讓使用者準備

  // ========== TC-MD-001: 全速前進 ==========
  Serial.println("[TC-MD-001] Forward Full Speed (255, 255)");
  motor.setLeftMotor(255);
  motor.setRightMotor(255);
  delay(2000);
  motor.setLeftMotor(0);
  motor.setRightMotor(0);
  delay(1000);

  // ========== TC-MD-002: 全速後退 ==========
  Serial.println("[TC-MD-002] Backward Full Speed (-255, -255)");
  motor.setLeftMotor(-255);
  motor.setRightMotor(-255);
  delay(2000);
  motor.setLeftMotor(0);
  motor.setRightMotor(0);
  delay(1000);

  // ========== TC-MD-003: 原地左轉 ==========
  Serial.println("[TC-MD-003] Turn Left In Place (-255, 255)");
  motor.setLeftMotor(-255);
  motor.setRightMotor(255);
  delay(2000);
  motor.setLeftMotor(0);
  motor.setRightMotor(0);
  delay(1000);

  // ========== TC-MD-004: 原地右轉 ==========
  Serial.println("[TC-MD-004] Turn Right In Place (255, -255)");
  motor.setLeftMotor(255);
  motor.setRightMotor(-255);
  delay(2000);
  motor.setLeftMotor(0);
  motor.setRightMotor(0);
  delay(1000);

  // ========== TC-MD-005: 半速前進 ==========
  Serial.println("[TC-MD-005] Half Speed Forward (127, 127)");
  motor.setLeftMotor(127);
  motor.setRightMotor(127);
  delay(2000);
  motor.setLeftMotor(0);
  motor.setRightMotor(0);
  delay(1000);

  Serial.println("========== Test Complete ==========");
}

void loop() {
  // 測試完成，空迴圈
}
```

**驗證清單**：

| 測試編號 | 輸入 | 預期行為 | 通過 ✓ |
|---------|------|---------|-------|
| TC-MD-001 | (255, 255) | 雙輪全速前進 2 秒 | ☐ |
| TC-MD-002 | (-255, -255) | 雙輪全速後退 2 秒 | ☐ |
| TC-MD-003 | (-255, 255) | 原地左轉 2 秒 | ☐ |
| TC-MD-004 | (255, -255) | 原地右轉 2 秒 | ☐ |
| TC-MD-005 | (127, 127) | 半速前進 2 秒 | ☐ |

---

#### 2.2.2 超聲波感測器單元測試 (test_ultrasonic.ino)

```cpp
/*
 * test_ultrasonic.ino - 超聲波感測器單元測試
 */

#include "ultrasonic_sensor.h"

UltrasonicSensor left_sensor(7, 8);   // Trig D7, Echo D8
UltrasonicSensor right_sensor(A1, A2);  // Trig A1, Echo A2

void setup() {
  Serial.begin(115200);
  Serial.println("========== Ultrasonic Sensor Unit Test ==========");
}

void loop() {
  // ========== TC-US-001: 左側感測器讀取 ==========
  uint16_t left_distance = left_sensor.getDistance();
  Serial.print("[TC-US-001] Left Distance: ");
  Serial.print(left_distance);
  Serial.println(" cm");

  // ========== TC-US-002: 右側感測器讀取 ==========
  uint16_t right_distance = right_sensor.getDistance();
  Serial.print("[TC-US-002] Right Distance: ");
  Serial.print(right_distance);
  Serial.println(" cm");

  // ========== TC-US-003: 無效值檢查 ==========
  if (left_distance == 999) {
    Serial.println("[TC-US-003] Left sensor: INVALID (999)");
  }
  if (right_distance == 999) {
    Serial.println("[TC-US-003] Right sensor: INVALID (999)");
  }

  // ========== TC-US-004: 範圍檢查 ==========
  if (left_distance >= 2 && left_distance <= 400) {
    Serial.println("[TC-US-004] Left sensor: VALID range");
  }
  if (right_distance >= 2 && right_distance <= 400) {
    Serial.println("[TC-US-004] Right sensor: VALID range");
  }

  Serial.println("----------------------------------------");
  delay(500);  // 每 500ms 測試一次
}
```

**驗證清單**：

| 測試編號 | 測試條件 | 預期輸出 | 通過 ✓ |
|---------|---------|---------|-------|
| TC-US-001 | 左側放置障礙物 @ 10cm | 顯示 10±2 cm | ☐ |
| TC-US-002 | 右側放置障礙物 @ 50cm | 顯示 50±5 cm | ☐ |
| TC-US-003 | 移除所有障礙物 | 顯示 999 (INVALID) | ☐ |
| TC-US-004 | 距離 2-400cm | 顯示 VALID range | ☐ |

---

#### 2.2.3 Serial 封包解析單元測試 (test_serial_protocol.ino)

```cpp
/*
 * test_serial_protocol.ino - Serial 協定單元測試
 */

#include "serial_protocol.h"

SerialProtocol protocol;

// 測試封包集
uint8_t test_packets[][8] = {
  // TC-SP-007: 全速前進
  {0xAA, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x55},

  // TC-SP-008: 全速後退 (-255, -255)
  {0xAA, 0x01, 0xFF, 0x01, 0xFF, 0x00, 0xFE, 0x55},

  // TC-SP-009: 吸塵器開啟
  {0xAA, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x55},

  // TC-SP-010: Header 錯誤（應拒絕）
  {0xBB, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x55},

  // TC-SP-011: Checksum 錯誤（應拒絕）
  {0xAA, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x55},
};

void setup() {
  Serial.begin(115200);
  Serial.println("========== Serial Protocol Unit Test ==========");

  delay(2000);

  for (int i = 0; i < 5; i++) {
    Serial.print("[Test ");
    Serial.print(i + 1);
    Serial.print("] ");

    int16_t left_pwm, right_pwm;
    bool vacuum;

    bool result = protocol.parseMotorPacket(test_packets[i], left_pwm, right_pwm, vacuum);

    if (result) {
      Serial.print("PASS - Left: ");
      Serial.print(left_pwm);
      Serial.print(", Right: ");
      Serial.print(right_pwm);
      Serial.print(", Vacuum: ");
      Serial.println(vacuum ? "ON" : "OFF");
    } else {
      Serial.println("FAIL - Packet rejected");
    }
  }

  Serial.println("========== Test Complete ==========");
}

void loop() {}
```

**預期輸出**：

```
========== Serial Protocol Unit Test ==========
[Test 1] PASS - Left: 255, Right: 255, Vacuum: OFF
[Test 2] PASS - Left: -255, Right: -255, Vacuum: OFF
[Test 3] PASS - Left: 0, Right: 0, Vacuum: ON
[Test 4] FAIL - Packet rejected
[Test 5] FAIL - Packet rejected
========== Test Complete ==========
```

---

## 3. 整合測試 (Integration Tests)

### 3.1 Pi ↔ Arduino 通訊整合測試

#### 3.1.1 測試案例 TC-INT-001: 端對端封包傳輸

**目的**：驗證 Pi 發送封包 → Arduino 接收 → Arduino 回應感測器資料 → Pi 接收

**測試步驟**：

1. Pi 發送馬達指令封包：`AA FF 00 FF 00 00 00 55`
2. Arduino 接收並驗證封包
3. Arduino 執行馬達控制
4. Arduino 發送感測器封包：`BB 64 00 64 00 03 06 66`
5. Pi 接收並解析感測器資料

**測試程式 (test_integration_serial.py)**：

```python
#!/usr/bin/env python3
"""test_integration_serial.py - Pi ↔ Arduino 整合測試"""

import serial
import time

def test_bidirectional_communication():
    """測試案例 TC-INT-001: 雙向通訊"""

    print("========== Integration Test: Serial Communication ==========")

    # 開啟 Serial Port
    ser = serial.Serial('/dev/serial0', 57600, timeout=1.0)
    time.sleep(0.5)  # 等待連接穩定

    # 清空緩衝區
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # 步驟 1: 發送馬達指令
    motor_packet = bytes([0xAA, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x55])
    print(f"[TX] Motor Command: {motor_packet.hex(' ')}")
    ser.write(motor_packet)

    # 步驟 2: 等待 Arduino 處理
    time.sleep(0.1)

    # 步驟 3: 接收感測器封包
    sensor_packet = ser.read(8)

    if len(sensor_packet) == 8:
        print(f"[RX] Sensor Data: {sensor_packet.hex(' ')}")

        # 步驟 4: 驗證封包格式
        assert sensor_packet[0] == 0xBB, "Header 應為 0xBB"
        assert sensor_packet[7] == 0x66, "Footer 應為 0x66"

        # 步驟 5: 驗證 Checksum
        checksum = sensor_packet[1] ^ sensor_packet[2] ^ sensor_packet[3] ^ sensor_packet[4] ^ sensor_packet[5]
        assert sensor_packet[6] == checksum, "Checksum 錯誤"

        # 步驟 6: 解析資料
        left_distance = sensor_packet[1] | (sensor_packet[2] << 8)
        right_distance = sensor_packet[3] | (sensor_packet[4] << 8)
        print(f"[OK] Left: {left_distance} cm, Right: {right_distance} cm")

        return True
    else:
        print(f"[FAIL] 接收資料不足，僅 {len(sensor_packet)} bytes")
        return False

    ser.close()

if __name__ == "__main__":
    result = test_bidirectional_communication()
    print("========== Test Result:", "PASS" if result else "FAIL", "==========")
```

**驗證清單**：

| 步驟 | 檢查項目 | 通過 ✓ |
|------|---------|-------|
| 1 | Pi 成功發送 8 bytes | ☐ |
| 2 | Arduino 正確接收並解析 | ☐ |
| 3 | Arduino 馬達開始轉動 | ☐ |
| 4 | Arduino 發送感測器封包 | ☐ |
| 5 | Pi 正確接收 8 bytes | ☐ |
| 6 | Checksum 驗證通過 | ☐ |

---

### 3.2 搖桿 → 馬達整合測試

#### 3.2.1 測試案例 TC-INT-002: 端對端控制迴圈

**目的**：驗證完整控制流程：搖桿輸入 → 差動運算 → Serial 傳輸 → 馬達輸出

**測試步驟**：

1. 推搖桿全前 (Y=1.0, X=0.0)
2. 觀察雙輪是否同速前進
3. 推搖桿左轉 (Y=0.0, X=-1.0)
4. 觀察是否原地左轉
5. 測量控制延遲

**測試程式 (test_integration_control.py)**：

```python
#!/usr/bin/env python3
"""test_integration_control.py - 端對端控制整合測試"""

import pygame
import time
from differential_drive import DifferentialDrive, VehicleCommand
from arduino_controller import ArduinoController

def test_control_loop():
    """測試案例 TC-INT-002: 端對端控制迴圈"""

    print("========== Integration Test: Control Loop ==========")

    # 初始化
    pygame.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    drive = DifferentialDrive()
    arduino = ArduinoController('/dev/serial0', 57600)

    print("[INFO] 系統初始化完成")
    print("[INFO] 請操作搖桿，按 Ctrl+C 退出")

    try:
        while True:
            # 讀取搖桿
            pygame.event.pump()
            x_axis = joystick.get_axis(0)  # 左右
            y_axis = joystick.get_axis(1)  # 前後

            # 測量開始時間
            start_time = time.time()

            # 建立控制指令
            cmd = VehicleCommand(
                linear_velocity=y_axis,
                angular_velocity=x_axis,
                vacuum_motor=False
            )

            # 差動運算
            motor_cmd = drive.convert(cmd)

            # 發送至 Arduino
            arduino.send_command(motor_cmd)

            # 測量結束時間
            end_time = time.time()
            latency_ms = (end_time - start_time) * 1000

            # 顯示狀態
            print(f"[Joystick] X:{x_axis:+.2f} Y:{y_axis:+.2f} | "
                  f"[Motor] L:{motor_cmd.left_pwm:+4d} R:{motor_cmd.right_pwm:+4d} | "
                  f"[Latency] {latency_ms:.2f} ms", end='\r')

            time.sleep(0.02)  # 50Hz

    except KeyboardInterrupt:
        print("\n[INFO] 測試中止")
        # 停止馬達
        stop_cmd = VehicleCommand(0.0, 0.0, False)
        arduino.send_command(drive.convert(stop_cmd))

if __name__ == "__main__":
    test_control_loop()
```

**驗收標準**：

| 項目 | 標準 | 通過 ✓ |
|------|------|-------|
| 前進響應 | 推搖桿後 < 50ms 內馬達啟動 | ☐ |
| 後退響應 | 拉搖桿後 < 50ms 內馬達反轉 | ☐ |
| 左轉響應 | 搖桿左推後車輛左轉 | ☐ |
| 右轉響應 | 搖桿右推後車輛右轉 | ☐ |
| 停止響應 | 搖桿歸零後 < 100ms 內停止 | ☐ |
| 控制延遲 | 平均延遲 < 10ms | ☐ |

---

### 3.3 感測器資料回傳整合測試

#### 3.3.1 測試案例 TC-INT-003: 感測器資料更新

**目的**：驗證超聲波測距資料能正確回傳至 Pi 並顯示

**測試步驟**：

1. 啟動系統
2. 在左側放置障礙物 @ 20cm
3. 在右側放置障礙物 @ 50cm
4. 觀察 Pi 顯示的距離是否正確
5. 移除障礙物，觀察是否顯示 999 (無效)

**測試程式 (test_integration_sensor.py)**：

```python
#!/usr/bin/env python3
"""test_integration_sensor.py - 感測器整合測試"""

import serial
import time

def test_sensor_feedback():
    """測試案例 TC-INT-003: 感測器資料回傳"""

    print("========== Integration Test: Sensor Feedback ==========")

    ser = serial.Serial('/dev/serial0', 57600, timeout=1.0)
    time.sleep(0.5)

    print("[INFO] 等待感測器資料...")
    print("[INFO] 請在左右兩側放置障礙物（左 20cm，右 50cm）")
    print("[INFO] 按 Ctrl+C 退出")

    try:
        while True:
            # 接收感測器封包
            packet = ser.read(8)

            if len(packet) == 8 and packet[0] == 0xBB and packet[7] == 0x66:
                # 驗證 Checksum
                checksum = packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5]

                if checksum == packet[6]:
                    # 解析距離
                    left_distance = packet[1] | (packet[2] << 8)
                    right_distance = packet[3] | (packet[4] << 8)
                    status = packet[5]

                    # 顯示資料
                    left_status = "VALID" if (status & 0x01) else "INVALID"
                    right_status = "VALID" if (status & 0x02) else "INVALID"

                    print(f"[Sensor] Left: {left_distance:3d} cm ({left_status}) | "
                          f"Right: {right_distance:3d} cm ({right_status})", end='\r')
                else:
                    print("[ERROR] Checksum mismatch")

            time.sleep(0.1)  # 10Hz

    except KeyboardInterrupt:
        print("\n[INFO] 測試結束")

    ser.close()

if __name__ == "__main__":
    test_sensor_feedback()
```

**驗收標準**：

| 測試條件 | 預期顯示 | 通過 ✓ |
|---------|---------|-------|
| 左側 20cm | Left: 18-22 cm (VALID) | ☐ |
| 右側 50cm | Right: 45-55 cm (VALID) | ☐ |
| 移除障礙物 | 999 cm (INVALID) | ☐ |
| 更新頻率 | 每 100ms 更新一次 | ☐ |

---

## 4. 系統測試 (System Tests)

### 4.1 功能需求驗證測試

#### 4.1.1 測試需求追溯矩陣

| 需求編號 | 需求描述 | 測試案例 | 狀態 |
|---------|---------|---------|------|
| FR1.1 | 自動偵測 USB 遙控器 | TC-SYS-001 | ☐ |
| FR1.2 | 讀取搖桿 X/Y 軸 | TC-SYS-002 | ☐ |
| FR1.3 | 正規化搖桿數值 | TC-SYS-003 | ☐ |
| FR1.4 | 死區過濾 | TC-SYS-004 | ☐ |
| FR1.5 | 讀取按鈕狀態 | TC-SYS-005 | ☐ |
| FR2 | 差動驅動運算 | TC-SYS-006 | ☐ |
| FR3 | Serial 通訊協定 | TC-SYS-007 | ☐ |
| FR4 | 馬達控制 | TC-SYS-008 | ☐ |
| FR5 | 吸塵器馬達控制 | TC-SYS-009 | ☐ |
| FR6 | 超聲波感測器讀取 | TC-SYS-010 | ☐ |
| FR7 | 感測器資料回傳 | TC-SYS-011 | ☐ |

---

#### 4.1.2 TC-SYS-001: 遙控器自動偵測

**測試步驟**：

1. 拔除 USB 遙控器
2. 啟動 Pi 程式
3. 插入 USB 遙控器
4. 觀察程式是否自動識別

**驗收標準**：
- [ ] 插入後 < 5 秒內顯示 "Joystick detected"
- [ ] 顯示正確的 `/dev/input/js0`
- [ ] 顯示遙控器名稱

---

#### 4.1.3 TC-SYS-006: 差動驅動完整測試

**測試步驟**：

| 搖桿輸入 | 預期行為 | 實際結果 | 通過 ✓ |
|---------|---------|---------|-------|
| Y=+1.0, X=0.0 | 直線全速前進 | | ☐ |
| Y=-1.0, X=0.0 | 直線全速後退 | | ☐ |
| Y=0.0, X=-1.0 | 原地左轉 | | ☐ |
| Y=0.0, X=+1.0 | 原地右轉 | | ☐ |
| Y=+0.7, X=-0.3 | 前進同時左轉 | | ☐ |
| Y=+0.7, X=+0.3 | 前進同時右轉 | | ☐ |
| Y=0.0, X=0.0 | 停止 | | ☐ |

---

#### 4.1.4 TC-SYS-009: 吸塵器控制測試

**測試步驟**：

1. 按下遙控器按鈕
2. 觀察吸塵器是否啟動
3. 再次按下按鈕
4. 觀察吸塵器是否停止

**驗收標準**：
- [ ] 按下按鈕後 < 50ms 內吸塵器啟動
- [ ] 聽到馬達運轉聲
- [ ] 再次按下後吸塵器停止
- [ ] 吸塵器運作時不影響車輛控制

---

### 4.2 非功能需求驗證測試

#### 4.2.1 NFR1: 控制延遲測試

**測試工具**：

```python
#!/usr/bin/env python3
"""test_nfr_latency.py - 控制延遲測試"""

import time
import pygame
from arduino_controller import ArduinoController
from differential_drive import DifferentialDrive, VehicleCommand

def test_control_latency():
    """測試案例 TC-NFR-001: 控制延遲 < 50ms"""

    pygame.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    drive = DifferentialDrive()
    arduino = ArduinoController('/dev/serial0', 57600)

    latencies = []

    for _ in range(100):  # 測試 100 次
        pygame.event.pump()

        start = time.perf_counter()

        # 讀取搖桿
        x = joystick.get_axis(0)
        y = joystick.get_axis(1)

        # 差動運算
        cmd = VehicleCommand(y, x, False)
        motor_cmd = drive.convert(cmd)

        # 發送封包
        arduino.send_command(motor_cmd)

        end = time.perf_counter()
        latency = (end - start) * 1000  # ms
        latencies.append(latency)

        time.sleep(0.02)  # 50Hz

    # 統計
    avg_latency = sum(latencies) / len(latencies)
    max_latency = max(latencies)
    min_latency = min(latencies)

    print(f"平均延遲: {avg_latency:.2f} ms")
    print(f"最大延遲: {max_latency:.2f} ms")
    print(f"最小延遲: {min_latency:.2f} ms")

    assert avg_latency < 50, f"平均延遲 {avg_latency:.2f} ms 超過 50ms"
    print("[PASS] 控制延遲符合要求")

if __name__ == "__main__":
    test_control_latency()
```

**驗收標準**：
- [ ] 平均延遲 < 10ms
- [ ] 最大延遲 < 50ms
- [ ] 99% 樣本 < 20ms

---

#### 4.2.2 NFR2: 通訊可靠性測試

**測試工具**：

```python
#!/usr/bin/env python3
"""test_nfr_reliability.py - 通訊可靠性測試"""

import serial
import time

def test_communication_reliability():
    """測試案例 TC-NFR-002: Checksum 驗證成功率 > 99%"""

    ser = serial.Serial('/dev/serial0', 57600, timeout=0.1)

    total_packets = 0
    valid_packets = 0
    invalid_packets = 0

    print("========== 通訊可靠性測試 ==========")
    print("[INFO] 發送 1000 個封包並統計成功率...")

    for i in range(1000):
        # 發送測試封包
        packet = bytes([0xAA, 0xFF, 0x00, 0x7F, 0x00, 0x00, 0x80, 0x55])
        ser.write(packet)
        total_packets += 1

        time.sleep(0.02)  # 50Hz

        # 嘗試接收感測器封包
        response = ser.read(8)
        if len(response) == 8 and response[0] == 0xBB:
            checksum = response[1] ^ response[2] ^ response[3] ^ response[4] ^ response[5]
            if checksum == response[6]:
                valid_packets += 1
            else:
                invalid_packets += 1

        if (i + 1) % 100 == 0:
            print(f"[Progress] {i + 1}/1000 packets sent")

    success_rate = (valid_packets / total_packets) * 100

    print(f"\n========== 測試結果 ==========")
    print(f"總封包數: {total_packets}")
    print(f"有效封包: {valid_packets}")
    print(f"無效封包: {invalid_packets}")
    print(f"成功率: {success_rate:.2f}%")

    assert success_rate > 99.0, f"成功率 {success_rate:.2f}% 低於 99%"
    print("[PASS] 通訊可靠性符合要求")

if __name__ == "__main__":
    test_communication_reliability()
```

**驗收標準**：
- [ ] Checksum 成功率 > 99%
- [ ] 無封包遺失（發送 1000 個，接收 > 990 個）
- [ ] 錯誤封包能被正確丟棄

---

#### 4.2.3 NFR3: 系統穩定性測試

**測試步驟**：

1. 啟動完整系統
2. 連續運行 30 分鐘
3. 期間隨機操作搖桿
4. 監控 CPU 使用率、記憶體使用
5. 檢查是否有當機或錯誤

**監控工具**：

```bash
# 監控 CPU 使用率
top -p $(pgrep -f robot_controller.py)

# 監控記憶體使用
watch -n 1 'ps aux | grep robot_controller.py | grep -v grep | awk "{print \$6}"'

# 監控 Serial 錯誤
journalctl -f | grep "serial"
```

**驗收標準**：
- [ ] 連續運行 30 分鐘無當機
- [ ] CPU 使用率 < 50%
- [ ] 記憶體使用 < 100MB
- [ ] 記憶體增長 < 1MB/小時（無洩漏）
- [ ] 無 Serial 錯誤訊息

---

## 5. 測試執行計畫

### 5.1 測試時程表

| 階段 | 測試項目 | 預計時間 | 負責人 |
|------|---------|---------|-------|
| Week 1 | 單元測試 - Python | 2 天 | 開發者 |
| Week 1 | 單元測試 - Arduino | 2 天 | 開發者 |
| Week 2 | 整合測試 - Serial 通訊 | 1 天 | 開發者 |
| Week 2 | 整合測試 - 控制迴圈 | 1 天 | 開發者 |
| Week 2 | 整合測試 - 感測器 | 1 天 | 開發者 |
| Week 3 | 系統測試 - 功能需求 | 2 天 | 測試人員 |
| Week 3 | 系統測試 - 非功能需求 | 2 天 | 測試人員 |
| Week 3 | 缺陷修復與回歸測試 | 1 天 | 開發者 |

---

### 5.2 測試環境設定

#### 5.2.1 Python 測試環境

```bash
# 安裝測試依賴
pip3 install pytest pytest-cov pytest-mock

# 執行所有測試
pytest tests/ -v --cov=raspberry_pi --cov-report=html

# 執行特定測試
pytest tests/test_differential_drive.py -v

# 執行特定測試案例
pytest tests/test_differential_drive.py::TestDifferentialDrive::test_forward_full_speed -v
```

#### 5.2.2 Arduino 測試環境

```bash
# 編譯測試程式
arduino-cli compile --fqbn arduino:avr:uno test_motor_driver/

# 上傳至 Arduino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno test_motor_driver/

# 監控 Serial 輸出
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

---

### 5.3 測試資料管理

**測試記錄表格式**：

| 測試編號 | 日期 | 測試人員 | 結果 | 備註 |
|---------|------|---------|------|------|
| TC-DD-001 | 2025-11-01 | 開發者 A | PASS | - |
| TC-SP-001 | 2025-11-01 | 開發者 A | PASS | - |
| TC-INT-001 | 2025-11-02 | 開發者 B | FAIL | Checksum 錯誤，已修復 |
| TC-SYS-006 | 2025-11-03 | 測試人員 | PASS | 所有方向控制正常 |

**缺陷追蹤表格式**：

| 缺陷編號 | 嚴重性 | 描述 | 發現測試 | 狀態 | 修復日期 |
|---------|-------|------|---------|------|---------|
| BUG-001 | High | Serial Checksum 計算錯誤 | TC-INT-001 | Fixed | 2025-11-02 |
| BUG-002 | Medium | 超聲波偶發 999 | TC-INT-003 | Open | - |

---

## 6. 驗收測試 (Acceptance Tests)

### 6.1 最終驗收清單

#### 6.1.1 功能驗收

| 驗收項目 | 標準 | 通過 ✓ |
|---------|------|-------|
| 遙控器連接 | 插入後 < 5 秒自動識別 | ☐ |
| 前進控制 | 搖桿全推後車輛直線前進 | ☐ |
| 後退控制 | 搖桿全拉後車輛直線後退 | ☐ |
| 左轉控制 | 搖桿左推後車輛左轉 | ☐ |
| 右轉控制 | 搖桿右推後車輛右轉 | ☐ |
| 吸塵器開關 | 按鈕控制吸塵器正常開關 | ☐ |
| 超聲波左側 | 測距誤差 < 5% | ☐ |
| 超聲波右側 | 測距誤差 < 5% | ☐ |
| Serial 通訊 | 封包掉包率 < 1% | ☐ |
| 緊急停止 | 搖桿歸零後立即停止 | ☐ |

#### 6.1.2 性能驗收

| 驗收項目 | 標準 | 實測值 | 通過 ✓ |
|---------|------|-------|-------|
| 控制延遲 | < 50ms | | ☐ |
| 連續運行 | 30 分鐘無當機 | | ☐ |
| CPU 使用率 | < 50% | | ☐ |
| 記憶體使用 | < 100MB | | ☐ |

#### 6.1.3 整合驗收

| 驗收項目 | 測試方法 | 通過 ✓ |
|---------|---------|-------|
| 端對端測試 | 搖桿 → Pi → Arduino → 馬達 | ☐ |
| 感測器顯示 | 距離正確顯示在 Pi 終端 | ☐ |
| 緊急停止 | 斷開 Serial 後馬達自動停止 | ☐ |

---

## 7. 測試工具與腳本

### 7.1 自動化測試腳本

**run_all_tests.sh**：

```bash
#!/bin/bash
# run_all_tests.sh - 執行所有自動化測試

echo "=========================================="
echo "  機電小車測試套件 - 自動化測試"
echo "=========================================="

# Python 單元測試
echo "[1/3] 執行 Python 單元測試..."
pytest raspberry_pi/tests/ -v --cov=raspberry_pi --cov-report=term-missing
if [ $? -ne 0 ]; then
    echo "[FAIL] Python 單元測試失敗"
    exit 1
fi

# 整合測試
echo "[2/3] 執行整合測試..."
python3 tests/test_integration_serial.py
if [ $? -ne 0 ]; then
    echo "[FAIL] 整合測試失敗"
    exit 1
fi

# 系統測試
echo "[3/3] 執行系統測試..."
python3 tests/test_nfr_latency.py
python3 tests/test_nfr_reliability.py

echo "=========================================="
echo "  所有測試完成"
echo "=========================================="
```

---

### 7.2 測試報告生成

```bash
# 生成 HTML 覆蓋率報告
pytest --cov=raspberry_pi --cov-report=html

# 生成 JUnit XML 報告（CI/CD 使用）
pytest --junitxml=test-results.xml

# 生成測試摘要
pytest --tb=short --no-header -q
```

---

## 8. 參考文件

- [01_SRS_軟體需求規格書.md](01_SRS_軟體需求規格書.md) - 需求定義
- [02_SA_系統分析.md](02_SA_系統分析.md) - 系統分析
- [03_SD_系統設計.md](03_SD_系統設計.md) - 設計規格
- [04_ICD_介面規格.md](04_ICD_介面規格.md) - 介面定義

---

**文件結束**
