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
    CMD_START, CMD_STOP, CMD_AVOID_RED, CMD_SET_VACUUM, CMD_QUERY_STATE, CMD_SET_PID, CMD_SET_PARAMS,
    STATE_IDLE, PKT_HEADER_STATE, PKT_STATE_LENGTH,
    STATE_NAMES, VACUUM_ON, VACUUM_OFF,
    create_cmd_start, create_cmd_stop, create_cmd_avoid_red,
    create_cmd_set_vacuum, create_cmd_query_state, create_cmd_set_pid, create_cmd_set_params,
    verify_state_packet
)
from red_detector import RedDetector
import config as cfg


# 紅色偵測參數 (從 config.py 讀取)
RED_AREA_THRESHOLD = cfg.RED_AREA_THRESHOLD
RED_CHECK_INTERVAL = cfg.RED_CHECK_INTERVAL


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
        self._red_clear_time = 0  # 紅色消失的時間點 (用於延遲重開吸塵器)

    def start(self):
        """啟動控制器"""
        self.running = True
        self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._rx_thread.start()

        # 發送控制參數到 Arduino
        time.sleep(0.1)  # 等待串列連接穩定
        self.send_set_params()
        print("[Controller] 控制參數已同步到 Arduino")

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
        self.serial.flush()
        self._red_avoiding = False  # 重置紅色避障狀態

    def send_stop(self):
        """發送停止指令"""
        packet = create_cmd_stop()
        self.serial.write(packet.serialize())
        self.serial.flush()

    def send_avoid_red(self):
        """發送避紅色指令"""
        packet = create_cmd_avoid_red()
        self.serial.write(packet.serialize())
        self.serial.flush()
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

    def send_set_pid(self, kp: float, ki: float, kd: float):
        """
        設定 PID 參數

        Args:
            kp: 比例增益 (推薦範圍 0.01 ~ 0.1)
            ki: 積分增益 (推薦範圍 0.001 ~ 0.01)
            kd: 微分增益 (推薦範圍 0.005 ~ 0.05)
        """
        packet = create_cmd_set_pid(kp, ki, kd)
        self.serial.write(packet.serialize())
        self.serial.flush()

    def send_set_params(self):
        """
        發送所有控制參數到 Arduino

        從 config.py 讀取參數並發送 CMD_SET_PARAMS 指令
        """
        params = cfg.get_all_params()
        packet = create_cmd_set_params(
            target_right_dist=params['target_right_dist'],
            front_stop_dist=params['front_stop_dist'],
            front_slow_dist=params['front_slow_dist'],
            corner_right_dist=params['corner_right_dist'],
            base_linear_speed=params['base_linear_speed'],
            backup_speed=params['backup_speed'],
            turn_angular_speed=params['turn_angular_speed'],
            find_wall_linear=params['find_wall_linear'],
            left_motor_scale=params['left_motor_scale'],
            right_motor_scale=params['right_motor_scale'],
            kp=params['kp'],
            ki=params['ki'],
            kd=params['kd'],
            min_effective_pwm=params['min_effective_pwm'],
            corner_turn_angle=params['corner_turn_angle'],
            red_avoid_angle=params['red_avoid_angle'],
            backup_duration_ms=params['backup_duration_ms'],
            turn_timeout_ms=params['turn_timeout_ms'],
        )
        self.serial.write(packet.serialize())
        self.serial.flush()
        print(f"[Controller] 已發送控制參數: {params}")

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

        red_now = detected and area > RED_AREA_THRESHOLD

        if not self._red_avoiding:
            # 尚未進入避障狀態
            if red_now:
                # 偵測到紅色，開始避障
                strategy = cfg.RED_AVOID_STRATEGY

                if strategy == "vacuum_off":
                    print(f"[Controller] 偵測到紅色區域 (面積={area})，關閉吸塵器")
                    self.send_set_vacuum(False)
                else:  # "turn_avoid"
                    print(f"[Controller] 偵測到紅色區域 (面積={area})，發送避障指令")
                    self.send_avoid_red()

                self._red_avoiding = True

                if self.on_red_detected:
                    self.on_red_detected(area)
        else:
            # 已在避障狀態中
            if red_now:
                # 紅色還在，重置延遲計時
                self._red_clear_time = 0
            else:
                # 紅色消失
                strategy = cfg.RED_AVOID_STRATEGY

                if strategy == "vacuum_off":
                    # 延遲重開吸塵器
                    if self._red_clear_time == 0:
                        self._red_clear_time = now
                        print("[Controller] 紅色區域消失，等待延遲後重開吸塵器...")
                    elif now - self._red_clear_time >= cfg.RED_CLEAR_DELAY:
                        print(f"[Controller] 延遲 {cfg.RED_CLEAR_DELAY}s 後，重新開啟吸塵器")
                        self.send_set_vacuum(True)
                        self._red_avoiding = False
                        self._red_clear_time = 0
                else:
                    # turn_avoid 模式直接重置
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
    parser.add_argument('--port', default=cfg.SERIAL_PORT, help='Serial port')
    parser.add_argument('--no-red', action='store_true', help='Disable red detection')
    args = parser.parse_args()

    controller = RobotController(port=args.port, baudrate=cfg.SERIAL_BAUDRATE, enable_red_detection=not args.no_red)
    controller.start()

    print("v2 Robot Controller")
    print("1=START  2=STOP  3=RED  4=VACUUM ON  5=VACUUM OFF")
    print("6=STATUS (once)  7=MONITOR (continuous)  8=SET PID")
    print("9=SYNC PARAMS (re-send config.py)  0=QUIT")
    print()

    def print_status_once():
        """印出一次狀態"""
        state = controller.get_state()
        red = controller.get_red_detection_status()
        print(f"State: {state['state_name']} (0x{state['state']:02X})")
        print(f"  Corners: {state['corner_count']}")
        print(f"  Front: {state['front_dist']}cm, Right: {state['right_dist']}cm")
        print(f"  Yaw: {state['yaw']:.1f}°")
        print(f"  Flags: 0x{state['flags']:02X}")
        print(f"  Red: detected={red['detected']}, area={red['area']}, avoiding={red['avoiding']}")

    def continuous_monitor():
        """持續監控模式 (按 Ctrl+C 退出)"""
        print("Continuous monitor (Ctrl+C to stop)...")
        print("-" * 60)
        try:
            while True:
                state = controller.get_state()
                red = controller.get_red_detection_status()
                # 使用 \r 覆蓋同一行，加上固定寬度
                line = (f"\rState: {state['state_name']:10s} | "
                        f"F:{state['front_dist']:3d}cm R:{state['right_dist']:3d}cm | "
                        f"Yaw:{state['yaw']:6.1f}° | "
                        f"Corners:{state['corner_count']} | "
                        f"Red:{int(red['area']):5d}")
                print(line, end='', flush=True)
                time.sleep(0.2)  # 5Hz 更新
        except KeyboardInterrupt:
            print("\n" + "-" * 60)
            print("Monitor stopped")

    def set_pid_interactive():
        """互動式 PID 調參"""
        print("\n=== PID 調參 ===")
        print("預設值: Kp=0.025, Ki=0.002, Kd=0.010")
        print("輸入格式: Kp Ki Kd (空白分隔)")
        print("範例: 0.03 0.002 0.015")
        print("直接 Enter 使用預設值，輸入 q 取消")

        try:
            line = input("PID> ").strip()
            if line == '' or line.lower() == 'q':
                print("取消")
                return

            parts = line.split()
            if len(parts) != 3:
                print("錯誤：需要 3 個參數 (Kp Ki Kd)")
                return

            kp = float(parts[0])
            ki = float(parts[1])
            kd = float(parts[2])

            # 驗證範圍
            if not (0 <= kp <= 1.0):
                print(f"警告：Kp={kp} 超出建議範圍 0~1.0")
            if not (0 <= ki <= 0.1):
                print(f"警告：Ki={ki} 超出建議範圍 0~0.1")
            if not (0 <= kd <= 0.5):
                print(f"警告：Kd={kd} 超出建議範圍 0~0.5")

            controller.send_set_pid(kp, ki, kd)
            print(f"已發送 PID: Kp={kp}, Ki={ki}, Kd={kd}")

        except ValueError as e:
            print(f"錯誤：無效的數值 - {e}")

    try:
        while True:
            cmd = input("> ").strip()

            if cmd == '1':
                controller.send_start()
                print("Sent START")
            elif cmd == '2':
                controller.send_stop()
                print("Sent STOP")
            elif cmd == '3':
                controller.send_avoid_red()
                print("Sent AVOID_RED")
            elif cmd == '4':
                controller.send_set_vacuum(True)
                print("Vacuum ON")
            elif cmd == '5':
                controller.send_set_vacuum(False)
                print("Vacuum OFF")
            elif cmd == '6':
                print_status_once()
            elif cmd == '7':
                continuous_monitor()
            elif cmd == '8':
                set_pid_interactive()
            elif cmd == '9':
                controller.send_set_params()
                print("參數已同步")
            elif cmd == '0':
                break
            elif cmd == '':
                continue
            else:
                print("1=START 2=STOP 3=RED 4=VAC_ON 5=VAC_OFF 6=STATUS 7=MONITOR 8=PID 9=SYNC 0=QUIT")
    except KeyboardInterrupt:
        print()
    finally:
        controller.close()
        print("Controller closed")
