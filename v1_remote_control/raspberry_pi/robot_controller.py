#!/usr/bin/env python3
"""
robot_controller.py - 機器人主控制邏輯
版本: 1.0
日期: 2025-10-31

整合遙控器輸入、差動驅動運算、Arduino 通訊，實現完整控制迴圈。
參考: 03_SD_系統設計.md - 3.2 RobotController 類別
"""

import time
import config
from usb_24g_receiver import USB24GReceiver
from differential_drive import DifferentialDrive, VehicleCommand, MotorCommand
from arduino_controller import ArduinoController


class RobotController:
    """機器人主控制器"""

    def __init__(self):
        """初始化機器人控制器"""
        print("========== 機器人控制器初始化 ==========\n")

        # 初始化各模組
        print("[1/3] 初始化遙控器...")
        self.receiver = USB24GReceiver()

        print("[2/3] 初始化差動驅動...")
        self.drive = DifferentialDrive()

        print("[3/3] 初始化 Arduino 通訊...")
        self.arduino = ArduinoController()

        # 控制狀態
        self.running = False
        self.last_command_time = time.time()

        # 統計資訊
        self.loop_count = 0
        self.start_time = None

        print("\n✅ 初始化完成！\n")

    def start(self):
        """啟動控制迴圈"""
        self.running = True
        self.start_time = time.time()
        self.loop_count = 0

        print("========== 啟動控制迴圈 ==========")
        print(f"頻率: {config.CONTROL_LOOP_FREQUENCY} Hz")
        print(f"週期: {config.CONTROL_LOOP_PERIOD * 1000:.1f} ms")
        print("按 Ctrl+C 停止\n")

        try:
            while self.running:
                loop_start = time.time()

                # 主控制迴圈
                self._control_loop()

                # 計算延遲並等待下個週期
                elapsed = time.time() - loop_start
                sleep_time = config.CONTROL_LOOP_PERIOD - elapsed

                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif config.DEBUG_MODE:
                    print(f"⚠️  控制迴圈超時: {elapsed * 1000:.2f} ms")

                self.loop_count += 1

        except KeyboardInterrupt:
            print("\n\n✋ 收到停止信號")
            self.stop()

        except Exception as e:
            print(f"\n❌ 發生錯誤: {e}")
            import traceback
            traceback.print_exc()
            self.stop()

    def _control_loop(self):
        """
        主控制迴圈（50Hz）

        流程:
        1. 接收遙控器輸入
        2. 差動驅動運算
        3. 發送指令至 Arduino
        4. 接收感測器資料
        5. 顯示狀態

        參考: 02_SA_系統分析.md - 2.2 資料流圖 (DFD)
        """
        # Step 1: 接收遙控器輸入
        vehicle_cmd = self.receiver.receive()
        self.last_command_time = time.time()

        # Step 2: 差動驅動運算
        motor_cmd = self.drive.convert(vehicle_cmd)

        # Step 3: 發送指令至 Arduino
        self.arduino.send_command(motor_cmd)

        # Step 4: 接收感測器資料（非阻塞）
        sensor_data = self.arduino.receive_sensor_data()

        # Step 5: 顯示狀態
        if config.SHOW_SENSOR_DATA:
            self._display_status(vehicle_cmd, motor_cmd, sensor_data)

        # 看門狗: 檢查通訊逾時
        if config.ENABLE_WATCHDOG:
            self._check_watchdog()

    def _display_status(self, vehicle_cmd, motor_cmd, sensor_data):
        """顯示即時狀態"""
        # 計算運行時間
        elapsed = time.time() - self.start_time if self.start_time else 0

        # 格式化輸出
        status = (
            f"[{elapsed:6.1f}s] "
            f"Joy(L:{vehicle_cmd.linear_velocity:+.2f} A:{vehicle_cmd.angular_velocity:+.2f}) "
            f"| PWM(L:{motor_cmd.left_pwm:+4d} R:{motor_cmd.right_pwm:+4d}) "
            f"| Dist(L:{sensor_data.left_distance:3d}cm R:{sensor_data.right_distance:3d}cm) "
            f"| Vac:{'ON ' if motor_cmd.vacuum else 'OFF'}"
        )

        print(status, end='\r')

    def _check_watchdog(self):
        """
        檢查看門狗（通訊逾時保護）

        若超過 EMERGENCY_STOP_TIMEOUT 未收到指令，則發送停止指令
        """
        elapsed = time.time() - self.last_command_time

        if elapsed > config.EMERGENCY_STOP_TIMEOUT:
            # 發送緊急停止指令
            stop_cmd = MotorCommand(0, 0, False)
            self.arduino.send_command(stop_cmd)

            if config.DEBUG_MODE:
                print(f"\n⚠️  看門狗觸發: {elapsed * 1000:.0f} ms 未收到指令")

    def stop(self):
        """停止控制器"""
        print("\n========== 停止控制器 ==========")

        # 發送停止指令
        print("[1/3] 停止馬達...")
        stop_cmd = MotorCommand(0, 0, False)
        self.arduino.send_command(stop_cmd)
        time.sleep(0.1)

        # 顯示統計資訊
        print("[2/3] 統計資訊:")
        self._print_stats()

        # 關閉連接
        print("[3/3] 關閉連接...")
        self.receiver.close()
        self.arduino.close()

        self.running = False
        print("\n✅ 控制器已停止")

    def _print_stats(self):
        """顯示統計資訊"""
        if self.start_time:
            elapsed = time.time() - self.start_time
            avg_freq = self.loop_count / elapsed if elapsed > 0 else 0

            print(f"   運行時間: {elapsed:.1f} 秒")
            print(f"   迴圈次數: {self.loop_count}")
            print(f"   平均頻率: {avg_freq:.1f} Hz")

        # Arduino 通訊統計
        stats = self.arduino.get_stats()
        print(f"   TX 封包: {stats['tx_packets']}")
        print(f"   RX 封包: {stats['rx_packets']}")
        print(f"   TX 錯誤: {stats['tx_errors']}")
        print(f"   RX 錯誤: {stats['rx_errors']}")
        print(f"   Checksum 錯誤: {stats['checksum_errors']}")

        if stats['tx_packets'] > 0:
            success_rate = (1 - stats['checksum_errors'] / stats['tx_packets']) * 100
            print(f"   成功率: {success_rate:.2f}%")


# ==================== 測試程式 ====================
if __name__ == "__main__":
    controller = RobotController()
    controller.start()
