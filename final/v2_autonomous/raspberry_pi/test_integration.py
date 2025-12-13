#!/usr/bin/env python3
"""
test_integration.py - Pi-Arduino 整合測試
版本: 1.0
日期: 2025-12-02

測試項目:
1. 通訊測試 - 驗證封包格式和 checksum
2. 指令-回應測試 - 驗證指令改變系統狀態
3. 狀態監控測試 - 驗證持續狀態回報

需要實際硬體連接 Arduino 才能執行。
"""

import sys
import time
import serial
sys.path.insert(0, '.')

from protocol import (
    CommandPacket, StatePacket,
    CMD_START, CMD_STOP, CMD_SET_VACUUM, CMD_QUERY_STATE,
    STATE_IDLE, STATE_FIND_WALL, STATE_FORWARD,
    FLAG_VACUUM_ENABLED, PKT_HEADER_STATE, PKT_STATE_LENGTH,
    create_cmd_start, create_cmd_stop, create_cmd_set_vacuum,
    verify_state_packet
)

# 測試參數
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUDRATE = 115200
TIMEOUT = 2.0  # 等待回應的超時時間（秒）


class IntegrationTester:
    """Pi-Arduino 整合測試器"""

    def __init__(self, port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE):
        """初始化測試器"""
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.test_results = {
            'passed': 0,
            'failed': 0,
            'errors': []
        }

    def connect(self):
        """連接到 Arduino"""
        try:
            print(f"[連接] 嘗試連接 {self.port} at {self.baudrate} baud...")
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)  # 等待 Arduino 重啟（DTR reset）

            # 清空緩衝區
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            print("[連接] 成功連接")
            return True
        except Exception as e:
            print(f"[連接] 失敗: {e}")
            return False

    def disconnect(self):
        """斷開連接"""
        if self.serial and self.serial.is_open:
            # 發送停止指令（安全）
            self.send_command(create_cmd_stop())
            time.sleep(0.2)
            self.serial.close()
            print("[連接] 已斷開")

    def send_command(self, cmd_packet):
        """
        發送指令封包

        Args:
            cmd_packet: CommandPacket 物件
        """
        data = cmd_packet.serialize()
        self.serial.write(data)
        self.serial.flush()
        print(f"[TX] {cmd_packet} - {' '.join(f'0x{b:02X}' for b in data)}")

    def receive_state(self, timeout=TIMEOUT):
        """
        接收狀態封包

        Args:
            timeout: 超時時間（秒）

        Returns:
            StatePacket 物件，若超時或驗證失敗返回 None
        """
        buffer = bytearray()
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            if self.serial.in_waiting > 0:
                data = self.serial.read(self.serial.in_waiting)
                buffer.extend(data)

                # 尋找狀態封包
                while len(buffer) >= PKT_STATE_LENGTH:
                    try:
                        idx = buffer.index(PKT_HEADER_STATE)
                        if idx > 0:
                            del buffer[:idx]
                    except ValueError:
                        buffer.clear()
                        break

                    if len(buffer) < PKT_STATE_LENGTH:
                        break

                    packet = bytes(buffer[:PKT_STATE_LENGTH])

                    if verify_state_packet(packet):
                        state_pkt = StatePacket.deserialize(packet)
                        print(f"[RX] {state_pkt}")
                        del buffer[:PKT_STATE_LENGTH]
                        return state_pkt
                    else:
                        print(f"[RX] Checksum 錯誤，丟棄封包")
                        del buffer[0]

            time.sleep(0.01)

        print("[RX] 超時，未收到有效狀態封包")
        return None

    def assert_test(self, condition, message):
        """
        斷言測試結果

        Args:
            condition: 測試條件（True = 通過）
            message: 測試描述
        """
        if condition:
            print(f"  [PASS] {message}")
            self.test_results['passed'] += 1
        else:
            print(f"  [FAIL] {message}")
            self.test_results['failed'] += 1
            self.test_results['errors'].append(message)

    # ========== 測試項目 ==========

    def test_communication(self):
        """測試 1: 基本通訊"""
        print("\n" + "="*50)
        print("測試 1: 基本通訊")
        print("="*50)

        # 發送 QUERY_STATE 指令
        cmd = CommandPacket(CMD_QUERY_STATE)
        self.send_command(cmd)

        # 等待狀態回報
        state = self.receive_state()
        self.assert_test(state is not None, "收到狀態封包")

        if state:
            # 驗證封包格式
            self.assert_test(0 <= state.state <= 0xFF, "狀態值在有效範圍")
            self.assert_test(0 <= state.corner_count <= 255, "角落計數在有效範圍")
            self.assert_test(0 <= state.front_distance <= 65535, "前方距離在有效範圍")
            self.assert_test(0 <= state.right_distance <= 65535, "右方距離在有效範圍")
            self.assert_test(-3600 <= state.yaw <= 3600, "偏航角在有效範圍（-360.0 ~ 360.0度）")

    def test_command_response(self):
        """測試 2: 指令-回應"""
        print("\n" + "="*50)
        print("測試 2: 指令-回應")
        print("="*50)

        # 2.1 確認初始為 IDLE
        print("\n[2.1] 檢查初始狀態")
        state = self.receive_state()
        if state:
            self.assert_test(state.state == STATE_IDLE, f"初始狀態應為 IDLE (0x{STATE_IDLE:02X})")

        # 2.2 發送 START，檢查狀態變化
        print("\n[2.2] 發送 START 指令")
        self.send_command(create_cmd_start())
        time.sleep(0.5)  # 等待狀態機啟動

        state = self.receive_state()
        if state:
            # 狀態應變為 FIND_WALL 或 FORWARD（取決於初始感測器讀值）
            is_running = state.state in [STATE_FIND_WALL, STATE_FORWARD]
            self.assert_test(is_running, f"START 後狀態應為 FIND_WALL 或 FORWARD，實際: {state.get_state_name()}")

        # 2.3 發送 STOP，檢查回到 IDLE
        print("\n[2.3] 發送 STOP 指令")
        self.send_command(create_cmd_stop())
        time.sleep(0.3)

        state = self.receive_state()
        if state:
            self.assert_test(state.state == STATE_IDLE, f"STOP 後狀態應為 IDLE，實際: {state.get_state_name()}")

        # 2.4 測試吸塵器控制
        print("\n[2.4] 測試吸塵器控制 - 開啟")
        self.send_command(create_cmd_set_vacuum(True))
        time.sleep(0.3)

        state = self.receive_state()
        if state:
            vacuum_on = state.has_flag(FLAG_VACUUM_ENABLED)
            self.assert_test(vacuum_on, "SET_VACUUM(1) 後 flags 應含 VACUUM_ENABLED")

        print("\n[2.5] 測試吸塵器控制 - 關閉")
        self.send_command(create_cmd_set_vacuum(False))
        time.sleep(0.3)

        state = self.receive_state()
        if state:
            vacuum_off = not state.has_flag(FLAG_VACUUM_ENABLED)
            self.assert_test(vacuum_off, "SET_VACUUM(0) 後 flags 應不含 VACUUM_ENABLED")

    def test_status_monitoring(self):
        """測試 3: 狀態監控"""
        print("\n" + "="*50)
        print("測試 3: 狀態監控")
        print("="*50)

        print("\n[3.1] 連續接收 10 個狀態封包")

        success_count = 0
        checksum_errors = 0
        total_attempts = 10

        for i in range(total_attempts):
            state = self.receive_state(timeout=1.0)
            if state:
                success_count += 1
                print(f"  [{i+1}/{total_attempts}] 成功 - 前方: {state.front_distance}cm, "
                      f"右方: {state.right_distance}cm, yaw: {state.yaw*0.1:.1f}度")
            else:
                checksum_errors += 1
                print(f"  [{i+1}/{total_attempts}] 失敗")

            time.sleep(0.1)

        # 統計
        success_rate = (success_count / total_attempts) * 100
        print(f"\n[統計]")
        print(f"  成功: {success_count}/{total_attempts} ({success_rate:.1f}%)")
        print(f"  失敗: {checksum_errors}/{total_attempts}")

        self.assert_test(success_rate >= 80, f"成功率應 >= 80%，實際: {success_rate:.1f}%")

    def run_all_tests(self):
        """執行所有測試"""
        print("="*50)
        print("  Pi-Arduino 整合測試")
        print("="*50)

        if not self.connect():
            print("\n[錯誤] 無法連接到 Arduino，請檢查:")
            print("  1. Arduino 是否已連接")
            print("  2. 串列埠位置是否正確（預設: /dev/ttyACM0）")
            print("  3. 是否有權限訪問串列埠（可能需要加入 dialout 群組）")
            return

        try:
            # 執行測試
            self.test_communication()
            self.test_command_response()
            self.test_status_monitoring()

            # 輸出總結
            print("\n" + "="*50)
            print("測試總結")
            print("="*50)
            print(f"通過: {self.test_results['passed']}")
            print(f"失敗: {self.test_results['failed']}")

            if self.test_results['failed'] > 0:
                print("\n失敗項目:")
                for error in self.test_results['errors']:
                    print(f"  - {error}")

            # 整體結果
            if self.test_results['failed'] == 0:
                print("\n[結果] 所有測試通過 ✓")
            else:
                print("\n[結果] 部分測試失敗 ✗")

        except KeyboardInterrupt:
            print("\n\n[中斷] 測試被使用者中斷")
        except Exception as e:
            print(f"\n[錯誤] 測試過程中發生例外: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.disconnect()


def main():
    """主程式入口"""
    import argparse

    parser = argparse.ArgumentParser(description='Pi-Arduino 整合測試')
    parser.add_argument('--port', default=SERIAL_PORT, help='串列埠位置（預設: /dev/ttyACM0）')
    parser.add_argument('--baudrate', type=int, default=SERIAL_BAUDRATE, help='波特率（預設: 115200）')
    args = parser.parse_args()

    tester = IntegrationTester(port=args.port, baudrate=args.baudrate)
    tester.run_all_tests()


if __name__ == "__main__":
    main()
