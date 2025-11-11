#!/usr/bin/env python3
"""
arduino_controller.py - Arduino Serial 通訊控制器
版本: 1.0
日期: 2025-10-31

負責與 Arduino 進行 Serial 通訊，發送馬達指令並接收感測器資料。
參考: 04_ICD_介面規格.md - 2. Serial 通訊介面
"""

import serial
import time
import config
from differential_drive import MotorCommand


class SensorData:
    """感測器資料"""

    def __init__(self, left_distance=999, right_distance=999, status=0):
        """
        初始化感測器資料

        Args:
            left_distance: 左側距離 (cm)
            right_distance: 右側距離 (cm)
            status: 狀態旗標
        """
        self.left_distance = left_distance
        self.right_distance = right_distance
        self.status = status
        self.timestamp = time.time()

    @property
    def left_valid(self):
        """左側感測器資料是否有效"""
        return (self.status & 0x01) != 0 and self.left_distance != 999

    @property
    def right_valid(self):
        """右側感測器資料是否有效"""
        return (self.status & 0x02) != 0 and self.right_distance != 999

    def __repr__(self):
        left_str = f"{self.left_distance:3d}cm {'✓' if self.left_valid else '✗'}"
        right_str = f"{self.right_distance:3d}cm {'✓' if self.right_valid else '✗'}"
        return f"SensorData(left={left_str}, right={right_str})"


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
                write_timeout=self.timeout  # 加入寫入逾時，避免阻塞
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
            Byte 5: Flags (bit0=vacuum)
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

        # Byte 5: Flags
        flags = 0x01 if motor_cmd.vacuum else 0x00
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
        """
        try:
            # 檢查是否有資料可讀
            if self.serial.in_waiting >= config.PACKET_SENSOR_SIZE:
                packet = self.serial.read(config.PACKET_SENSOR_SIZE)

                if config.VERBOSE_SERIAL:
                    print(f"[RX] {packet.hex(' ')}")

                # 解析封包
                sensor_data = self._parse_sensor_packet(packet)

                if sensor_data is not None:
                    self.latest_sensor_data = sensor_data
                    self.stats['rx_packets'] += 1
                else:
                    self.stats['rx_errors'] += 1

        except serial.SerialException as e:
            self.stats['rx_errors'] += 1
            print(f"❌ Serial 接收錯誤: {e}")

        return self.latest_sensor_data

    def _parse_sensor_packet(self, packet: bytes) -> SensorData:
        """
        解析感測器封包

        封包格式:
            Byte 0: Header (0xBB)
            Byte 1-2: Left Distance (uint16, little-endian)
            Byte 3-4: Right Distance (uint16, little-endian)
            Byte 5: Status
            Byte 6: Checksum (XOR of bytes 1-5)
            Byte 7: Footer (0x66)

        Args:
            packet: 8-byte 封包

        Returns:
            SensorData 物件，若封包無效則返回 None

        參考: 04_ICD_介面規格.md - 2.3.1
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

        # 解析資料
        left_distance = packet[1] | (packet[2] << 8)
        right_distance = packet[3] | (packet[4] << 8)
        status = packet[5]

        return SensorData(left_distance, right_distance, status)

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
