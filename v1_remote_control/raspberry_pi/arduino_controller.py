#!/usr/bin/env python3
"""
arduino_controller.py - Arduino Serial 通訊控制器
版本: 2.0 (新增 IMU 資料)
日期: 2025-11-29

負責與 Arduino 進行 Serial 通訊，發送馬達指令並接收感測器資料。
參考: 04_ICD_介面規格.md - 2. Serial 通訊介面

v2.0 變更:
  - 感測器封包從 8 bytes 擴充為 12 bytes
  - 新增 IMU 資料 (Yaw 角度、Z軸角速度)
"""

import serial
import time
import config
from differential_drive import MotorCommand


class SensorData:
    """感測器資料 (v2.0 - 含 IMU)"""

    def __init__(self, front_distance=999, right_distance=999,
                 yaw=0.0, gyro_z=0.0, status=0):
        """
        初始化感測器資料

        Args:
            front_distance: 前方距離 (cm)
            right_distance: 右側距離 (cm)
            yaw: Yaw 角度 (度, -180 ~ +180)
            gyro_z: Z軸角速度 (deg/s)
            status: 狀態旗標
        """
        self.front_distance = front_distance
        self.right_distance = right_distance
        self.yaw = yaw
        self.gyro_z = gyro_z
        self.status = status
        self.timestamp = time.time()

    @property
    def front_valid(self):
        """前方感測器資料是否有效"""
        return (self.status & 0x01) != 0 and self.front_distance != 999

    @property
    def right_valid(self):
        """右側感測器資料是否有效"""
        return (self.status & 0x02) != 0 and self.right_distance != 999

    @property
    def imu_valid(self):
        """IMU 資料是否有效"""
        return (self.status & 0x04) != 0

    @property
    def vacuum_on(self):
        """吸塵器是否開啟"""
        return (self.status & 0x08) != 0

    def __repr__(self):
        front_str = f"{self.front_distance:3d}cm {'✓' if self.front_valid else '✗'}"
        right_str = f"{self.right_distance:3d}cm {'✓' if self.right_valid else '✗'}"
        imu_str = f"Yaw:{self.yaw:+6.1f}° Gz:{self.gyro_z:+5.1f}°/s" if self.imu_valid else "IMU:N/A"
        return f"SensorData(front={front_str}, right={right_str}, {imu_str})"


class ArduinoController:
    """Arduino Serial 通訊控制器"""

    def __init__(self, port=None, baudrate=None, timeout=None):
        """
        初始化 Arduino 控制器

        Args:
            port: Serial 埠（預設從 config 讀取）
            baudrate: 鮑率（預設從 config 讀取）
            timeout: 逾時（預設從 config 讀取）
        """
        self.port = port if port is not None else config.SERIAL_PORT
        self.baudrate = baudrate if baudrate is not None else config.SERIAL_BAUDRATE
        self.timeout = timeout if timeout is not None else config.SERIAL_TIMEOUT

        # 統計資訊
        self.stats = {
            'tx_packets': 0,
            'rx_packets': 0,
            'tx_errors': 0,
            'rx_errors': 0,
            'checksum_errors': 0,
        }

        # 最新感測器資料
        self.latest_sensor_data = SensorData()

        # 初始化 Serial 連接
        self.serial = None
        self._connect()

    def _connect(self):
        """建立 Serial 連接"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=1.0  # 寫入逾時設長一點，避免 buffer 滿時報錯
            )

            # 等待 Arduino 重置（開啟 Serial 會觸發 Arduino 重置）
            time.sleep(2.0)

            # 清空緩衝區
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            print(f"✅ Serial 連接成功: {self.port} @ {self.baudrate} bps")

        except serial.SerialException as e:
            raise ConnectionError(f"無法開啟 Serial Port {self.port}: {e}")

    def send_command(self, motor_cmd: MotorCommand):
        """
        發送馬達指令至 Arduino

        Args:
            motor_cmd: MotorCommand 物件

        參考: 04_ICD_介面規格.md - 2.2 Pi → Arduino 指令封包
        """
        try:
            packet = self._build_motor_packet(motor_cmd)

            if config.VERBOSE_SERIAL:
                print(f"[TX] {packet.hex(' ')}")

            self.serial.write(packet)
            self.stats['tx_packets'] += 1

        except serial.SerialException as e:
            self.stats['tx_errors'] += 1
            print(f"❌ Serial 發送錯誤: {e}")

    def _build_motor_packet(self, motor_cmd: MotorCommand) -> bytes:
        """
        建構馬達指令封包（8 bytes）

        封包格式:
            Byte 0: Header (0xAA)
            Byte 1-2: Left PWM (int16, little-endian)
            Byte 3-4: Right PWM (int16, little-endian)
            Byte 5: Flags (bit0=vacuum, bit1=ultrasonic_enable)
            Byte 6: Checksum (XOR of bytes 1-5)
            Byte 7: Footer (0x55)

        Args:
            motor_cmd: MotorCommand 物件

        Returns:
            bytes: 8-byte 封包

        參考: 04_ICD_介面規格.md - 2.2.1
        """
        packet = bytearray(8)

        # Byte 0: Header
        packet[0] = config.PACKET_MOTOR_HEADER

        # Byte 1-2: Left PWM (int16, 2's complement)
        left_pwm = motor_cmd.left_pwm & 0xFFFF  # 轉換為 uint16
        packet[1] = left_pwm & 0xFF             # Low byte
        packet[2] = (left_pwm >> 8) & 0xFF      # High byte

        # Byte 3-4: Right PWM (int16, 2's complement)
        right_pwm = motor_cmd.right_pwm & 0xFFFF
        packet[3] = right_pwm & 0xFF
        packet[4] = (right_pwm >> 8) & 0xFF

        # Byte 5: Flags (bit0=vacuum, bit1=ultrasonic_enable)
        flags = 0x00
        if motor_cmd.vacuum:
            flags |= 0x01
        if motor_cmd.ultrasonic_enable:
            flags |= 0x02
        packet[5] = flags

        # Byte 6: Checksum (XOR of bytes 1-5)
        packet[6] = packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5]

        # Byte 7: Footer
        packet[7] = config.PACKET_MOTOR_FOOTER

        return bytes(packet)

    def receive_sensor_data(self) -> SensorData:
        """
        接收 Arduino 感測器資料（非阻塞）

        Returns:
            SensorData 物件（若無新資料則返回最新快取）

        參考: 04_ICD_介面規格.md - 2.3 Arduino → Pi 感測器封包

        v2.0: 封包大小從 8 bytes 變更為 12 bytes
        """
        try:
            # 封包同步：跳過非 Header 的資料（處理 Arduino 開機訊息干擾）
            sync_attempts = 0
            max_sync_attempts = 50  # 最多跳過 50 bytes

            while self.serial.in_waiting > 0 and sync_attempts < max_sync_attempts:
                # 偷看第一個 byte
                peek = self.serial.read(1)
                if len(peek) == 0:
                    break

                if peek[0] == config.PACKET_SENSOR_HEADER:
                    # 找到 Header，讀取剩餘封包
                    if self.serial.in_waiting >= config.PACKET_SENSOR_SIZE - 1:
                        rest = self.serial.read(config.PACKET_SENSOR_SIZE - 1)
                        packet = peek + rest

                        if config.VERBOSE_SERIAL:
                            print(f"[RX] {packet.hex(' ')}")

                        # 解析封包 (v2.0)
                        sensor_data = self._parse_sensor_packet_v2(packet)

                        if sensor_data is not None:
                            self.latest_sensor_data = sensor_data
                            self.stats['rx_packets'] += 1
                            break
                        else:
                            self.stats['rx_errors'] += 1
                    else:
                        # 資料不足，把 Header 放回去（實際上做不到，下次再試）
                        break
                else:
                    # 跳過非 Header byte
                    sync_attempts += 1
                    if config.DEBUG_MODE and sync_attempts == 1:
                        print(f"[SYNC] 跳過干擾資料...")

        except serial.SerialException as e:
            self.stats['rx_errors'] += 1
            print(f"❌ Serial 接收錯誤: {e}")

        return self.latest_sensor_data

    def _parse_sensor_packet_v2(self, packet: bytes) -> SensorData:
        """
        解析感測器封包 v2.0 (含 IMU)

        封包格式 (12 bytes):
            Byte 0: Header (0xBB)
            Byte 1-2: Front Distance (uint16, little-endian, cm)
            Byte 3-4: Right Distance (uint16, little-endian, cm)
            Byte 5-6: Yaw Angle (int16, little-endian, 度數 × 10)
            Byte 7: Gyro Z (int8, deg/s)
            Byte 8: Status (bit0=front_valid, bit1=right_valid, bit2=imu_valid, bit3=vacuum)
            Byte 9: Reserved
            Byte 10: Checksum (XOR of bytes 1-9)
            Byte 11: Footer (0x66)

        Args:
            packet: 12-byte 封包

        Returns:
            SensorData 物件，若封包無效則返回 None
        """
        # 驗證長度
        if len(packet) != 12:
            if config.DEBUG_MODE:
                print(f"❌ 封包長度錯誤: {len(packet)} (expected 12)")
            return None

        # 驗證 Header
        if packet[0] != config.PACKET_SENSOR_HEADER:
            if config.DEBUG_MODE:
                print(f"❌ Header 錯誤: {packet[0]:02X} (expected {config.PACKET_SENSOR_HEADER:02X})")
            return None

        # 驗證 Footer
        if packet[11] != config.PACKET_SENSOR_FOOTER:
            if config.DEBUG_MODE:
                print(f"❌ Footer 錯誤: {packet[11]:02X} (expected {config.PACKET_SENSOR_FOOTER:02X})")
            return None

        # 驗證 Checksum (XOR of bytes 1-9)
        checksum = 0
        for i in range(1, 10):
            checksum ^= packet[i]
        if checksum != packet[10]:
            self.stats['checksum_errors'] += 1
            if config.DEBUG_MODE:
                print(f"❌ Checksum 錯誤: {packet[10]:02X} (expected {checksum:02X})")
            return None

        # 解析資料
        front_distance = packet[1] | (packet[2] << 8)
        right_distance = packet[3] | (packet[4] << 8)

        # Yaw: int16, 度數 × 10
        yaw_raw = packet[5] | (packet[6] << 8)
        # 處理負數 (2's complement)
        if yaw_raw > 32767:
            yaw_raw -= 65536
        yaw = yaw_raw / 10.0

        # Gyro Z: int8
        gyro_z = packet[7]
        if gyro_z > 127:
            gyro_z -= 256

        status = packet[8]

        return SensorData(front_distance, right_distance, yaw, gyro_z, status)

    def _parse_sensor_packet(self, packet: bytes) -> SensorData:
        """
        解析感測器封包 v1.0 (向下相容，無 IMU)

        封包格式 (8 bytes):
            Byte 0: Header (0xBB)
            Byte 1-2: Front Distance (uint16, little-endian)
            Byte 3-4: Right Distance (uint16, little-endian)
            Byte 5: Status
            Byte 6: Checksum (XOR of bytes 1-5)
            Byte 7: Footer (0x66)

        Args:
            packet: 8-byte 封包

        Returns:
            SensorData 物件，若封包無效則返回 None

        @deprecated 請使用 _parse_sensor_packet_v2
        """
        # 驗證長度
        if len(packet) != 8:
            return None

        # 驗證 Header
        if packet[0] != config.PACKET_SENSOR_HEADER:
            if config.DEBUG_MODE:
                print(f"❌ Header 錯誤: {packet[0]:02X} (expected {config.PACKET_SENSOR_HEADER:02X})")
            return None

        # 驗證 Footer
        if packet[7] != config.PACKET_SENSOR_FOOTER:
            if config.DEBUG_MODE:
                print(f"❌ Footer 錯誤: {packet[7]:02X} (expected {config.PACKET_SENSOR_FOOTER:02X})")
            return None

        # 驗證 Checksum
        checksum = packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5]
        if checksum != packet[6]:
            self.stats['checksum_errors'] += 1
            if config.DEBUG_MODE:
                print(f"❌ Checksum 錯誤: {packet[6]:02X} (expected {checksum:02X})")
            return None

        # 解析資料 (封包格式: front, right)
        front_distance = packet[1] | (packet[2] << 8)
        right_distance = packet[3] | (packet[4] << 8)
        status = packet[5]

        return SensorData(front_distance, right_distance, 0.0, 0.0, status)

    def get_stats(self):
        """取得通訊統計資訊"""
        return self.stats.copy()

    def reset_stats(self):
        """重置統計資訊"""
        for key in self.stats:
            self.stats[key] = 0

    def close(self):
        """關閉 Serial 連接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("✅ Serial 連接已關閉")


# ==================== 測試程式 ====================
if __name__ == "__main__":
    print("========== Arduino Serial 通訊測試 ==========\n")

    try:
        controller = ArduinoController()

        print("[INFO] 開始測試馬達指令發送...")
        print("[INFO] 每秒發送一次測試指令，按 Ctrl+C 退出\n")

        test_commands = [
            MotorCommand(255, 255, False),    # 全速前進
            MotorCommand(-255, -255, False),  # 全速後退
            MotorCommand(-255, 255, False),   # 左轉
            MotorCommand(255, -255, False),   # 右轉
            MotorCommand(0, 0, True),         # 停止 + 吸塵器開啟
            MotorCommand(0, 0, False),        # 停止
        ]

        idx = 0

        while True:
            # 發送測試指令
            cmd = test_commands[idx % len(test_commands)]
            print(f"[TX] {cmd}")
            controller.send_command(cmd)

            # 等待並接收感測器資料
            time.sleep(0.1)
            sensor_data = controller.receive_sensor_data()
            print(f"[RX] {sensor_data}")

            # 顯示統計
            stats = controller.get_stats()
            print(f"[Stats] TX:{stats['tx_packets']} RX:{stats['rx_packets']} "
                  f"TX_ERR:{stats['tx_errors']} RX_ERR:{stats['rx_errors']} "
                  f"CHKSUM_ERR:{stats['checksum_errors']}")
            print()

            idx += 1
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n✋ 測試中止")
        controller.close()

    except ConnectionError as e:
        print(f"❌ 連接錯誤: {e}")

    except Exception as e:
        print(f"❌ 發生錯誤: {e}")
        import traceback
        traceback.print_exc()
