#!/usr/bin/env python3
"""
v2 高階控制器 - Pi 端
負責：發送指令、接收狀態、紅色偵測觸發
"""

import serial
import threading
import time
from protocol import (
    CommandPacket, StatePacket,
    CMD_START, CMD_STOP, CMD_AVOID_RED, CMD_SET_VACUUM, CMD_QUERY_STATE,
    STATE_IDLE, PKT_HEADER_STATE, PKT_STATE_LENGTH,
    STATE_NAMES, VACUUM_ON, VACUUM_OFF,
    create_cmd_start, create_cmd_stop, create_cmd_avoid_red,
    create_cmd_set_vacuum, create_cmd_query_state,
    verify_state_packet
)
from red_detector import RedDetector


# 紅色偵測參數
RED_AREA_THRESHOLD = 2000  # 紅色面積閾值（需實測調整）
RED_CHECK_INTERVAL = 0.5   # 紅色檢查間隔（秒）


class RobotController:
    """機器人高階控制器"""

    def __init__(self, port='/dev/ttyACM0', baudrate=115200, enable_red_detection=True):
        """
        初始化控制器

        Args:
            port: 串列埠 (預設 /dev/ttyACM0)
            baudrate: 波特率 (預設 115200)
            enable_red_detection: 是否啟用紅色偵測 (預設 True)
        """
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        self.running = False
        self.enable_red_detection = enable_red_detection

        # 狀態
        self.current_state = STATE_IDLE
        self.corner_count = 0
        self.front_dist = 0
        self.right_dist = 0
        self.yaw = 0.0
        self.flags = 0

        # 回調
        self.on_state_change = None
        self.on_corner = None
        self.on_red_detected = None

        # 接收執行緒
        self._rx_thread = None
        self._lock = threading.Lock()

        # 紅色偵測器
        self.red_detector = None
        if enable_red_detection:
            self.red_detector = RedDetector(camera_index=0, width=320, height=240)
            print("[Controller] 紅色偵測器已初始化")

        # 紅色偵測狀態
        self._last_red_check = 0
        self._red_avoiding = False

    def start(self):
        """啟動控制器"""
        self.running = True
        self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._rx_thread.start()

        # 啟動紅色偵測背景執行緒
        if self.red_detector:
            if self.red_detector.start():
                print("[Controller] 紅色偵測背景執行緒已啟動")
            else:
                print("[Controller] 警告：紅色偵測啟動失敗")

    def stop(self):
        """停止控制器"""
        self.running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)

        # 停止紅色偵測背景執行緒
        if self.red_detector:
            self.red_detector.stop()
            print("[Controller] 紅色偵測背景執行緒已停止")

    # === 指令發送 ===

    def send_start(self):
        """發送開始指令"""
        packet = create_cmd_start()
        self.serial.write(packet.serialize())
        self._red_avoiding = False  # 重置紅色避障狀態

    def send_stop(self):
        """發送停止指令"""
        packet = create_cmd_stop()
        self.serial.write(packet.serialize())

    def send_avoid_red(self):
        """發送避紅色指令"""
        packet = create_cmd_avoid_red()
        self.serial.write(packet.serialize())
        self._red_avoiding = True

    def send_set_vacuum(self, on: bool):
        """
        設定吸塵器

        Args:
            on: True = 開啟, False = 關閉
        """
        packet = create_cmd_set_vacuum(on)
        self.serial.write(packet.serialize())

    def send_query_state(self):
        """查詢當前狀態"""
        packet = create_cmd_query_state()
        self.serial.write(packet.serialize())

    # === 狀態接收 ===

    def _receive_loop(self):
        """接收執行緒"""
        buffer = bytearray()

        while self.running:
            try:
                data = self.serial.read(64)
                if data:
                    buffer.extend(data)
                    self._process_buffer(buffer)

                # 定期檢查紅色偵測
                self._check_red_detection()

            except Exception as e:
                print(f"[RX Error] {e}")
                time.sleep(0.1)

    def _check_red_detection(self):
        """檢查紅色偵測結果並觸發避障"""
        if not self.red_detector or not self.enable_red_detection:
            return

        # 控制檢查頻率
        now = time.time()
        if now - self._last_red_check < RED_CHECK_INTERVAL:
            return
        self._last_red_check = now

        # 取得紅色偵測結果
        detected, area = self.red_detector.get_result()

        # 當偵測到大面積紅色且未在避障狀態時，發送避紅色指令
        if detected and area > RED_AREA_THRESHOLD and not self._red_avoiding:
            print(f"[Controller] 偵測到紅色區域 (面積={area})，發送避障指令")
            self.send_avoid_red()

            # 觸發回調
            if self.on_red_detected:
                self.on_red_detected(area)

        # 如果紅色消失，重置避障狀態
        elif not detected and self._red_avoiding:
            print("[Controller] 紅色區域消失，重置避障狀態")
            self._red_avoiding = False

    def _process_buffer(self, buffer):
        """
        處理接收緩衝區

        尋找狀態封包並解析
        """
        while len(buffer) >= PKT_STATE_LENGTH:
            # 尋找 Header
            try:
                idx = buffer.index(PKT_HEADER_STATE)
                if idx > 0:
                    del buffer[:idx]  # 丟棄 Header 前的資料
            except ValueError:
                buffer.clear()
                return

            if len(buffer) < PKT_STATE_LENGTH:
                return

            # 提取封包
            packet = bytes(buffer[:PKT_STATE_LENGTH])

            if verify_state_packet(packet):
                self._parse_state(packet)
                del buffer[:PKT_STATE_LENGTH]
            else:
                del buffer[0]  # 丟棄無效 Header，繼續尋找

    def _parse_state(self, packet):
        """
        解析狀態封包

        Args:
            packet: 狀態封包 (bytes, 12 bytes)
        """
        with self._lock:
            old_state = self.current_state
            old_corners = self.corner_count

            # 解析封包內容
            self.current_state = packet[1]
            self.corner_count = packet[2]
            self.front_dist = (packet[3] << 8) | packet[4]
            self.right_dist = (packet[5] << 8) | packet[6]
            raw_yaw = (packet[7] << 8) | packet[8]
            # 轉換為有號整數 (補碼)
            if raw_yaw > 32767:
                raw_yaw -= 65536
            self.yaw = raw_yaw / 10.0
            self.flags = packet[9]

        # 觸發回調
        if self.on_state_change and old_state != self.current_state:
            self.on_state_change(self.current_state)

        if self.on_corner and old_corners != self.corner_count:
            self.on_corner(self.corner_count)

    # === 狀態查詢 ===

    def get_state(self):
        """
        取得當前狀態

        Returns:
            dict 包含所有狀態資訊
        """
        with self._lock:
            return {
                'state': self.current_state,
                'state_name': STATE_NAMES.get(self.current_state, 'UNKNOWN'),
                'corner_count': self.corner_count,
                'front_dist': self.front_dist,
                'right_dist': self.right_dist,
                'yaw': self.yaw,
                'flags': self.flags
            }

    def get_red_detection_status(self):
        """
        取得紅色偵測狀態

        Returns:
            dict: {'detected': bool, 'area': int, 'avoiding': bool}
        """
        if not self.red_detector:
            return {'detected': False, 'area': 0, 'avoiding': False}

        detected, area = self.red_detector.get_result()
        return {
            'detected': detected,
            'area': area,
            'avoiding': self._red_avoiding
        }

    def close(self):
        """關閉連接"""
        self.stop()
        if self.red_detector:
            self.red_detector.close()
        self.serial.close()


# === 主程式 ===
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='v2 Robot Controller')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--no-red', action='store_true', help='Disable red detection')
    args = parser.parse_args()

    controller = RobotController(port=args.port, enable_red_detection=not args.no_red)
    controller.start()

    print("v2 Robot Controller")
    print("Commands: start, stop, red, vacuum on/off, query, status, red_status, quit")
    print()

    try:
        while True:
            cmd = input("> ").strip().lower()

            if cmd == 'start':
                controller.send_start()
                print("Sent START")
            elif cmd == 'stop':
                controller.send_stop()
                print("Sent STOP")
            elif cmd == 'red':
                controller.send_avoid_red()
                print("Sent AVOID_RED")
            elif cmd == 'vacuum on':
                controller.send_set_vacuum(True)
                print("Vacuum ON")
            elif cmd == 'vacuum off':
                controller.send_set_vacuum(False)
                print("Vacuum OFF")
            elif cmd == 'query':
                controller.send_query_state()
                print("Sent QUERY_STATE")
            elif cmd == 'status':
                state = controller.get_state()
                print(f"State: {state['state_name']} (0x{state['state']:02X})")
                print(f"  Corners: {state['corner_count']}")
                print(f"  Front: {state['front_dist']}cm, Right: {state['right_dist']}cm")
                print(f"  Yaw: {state['yaw']:.1f}°")
                print(f"  Flags: 0x{state['flags']:02X}")
            elif cmd == 'red_status':
                red_status = controller.get_red_detection_status()
                print(f"Red Detection: {red_status['detected']}")
                print(f"  Area: {red_status['area']}")
                print(f"  Avoiding: {red_status['avoiding']}")
            elif cmd == 'quit':
                break
            elif cmd == '':
                continue
            else:
                print("Unknown command. Try: start, stop, red, vacuum on/off, query, status, red_status, quit")
    except KeyboardInterrupt:
        print()
    finally:
        controller.close()
        print("Controller closed")
