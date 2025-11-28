#!/usr/bin/env python3
"""
autonomous_main.py - 期末自走競賽主程式
版本: 1.0
日期: 2025-11-28

自走模式入口程式，使用沿右牆策略完成清掃任務。
參考: FINAL_AUTONOMOUS_PLAN.md
"""

import argparse
import sys
import time
from enum import Enum

import config
from differential_drive import DifferentialDrive, VehicleCommand
from arduino_controller import ArduinoController
from wall_follower import WallFollower


class RobotState(Enum):
    """機器人狀態"""
    INIT = 0              # 初始化
    FOLLOW_WALL = 1       # 沿右牆前進
    CORNER_SWEEP = 2      # 角落掃描
    TURN_LEFT = 3         # 左轉90度
    AVOID_RED = 4         # 紅圓迴避
    RETURN_TO_WALL = 5    # 找回右牆
    EXIT = 6              # 返回出口
    DONE = 7              # 完成


class AutonomousController:
    """自走模式控制器"""

    # 計時參數
    TARGET_TIME = 140     # 目標時間 (秒) - 2:20
    MAX_TIME = 150        # 最大時間 (秒) - 2:30

    def __init__(self, enable_camera=True):
        """
        初始化自走控制器

        Args:
            enable_camera: 是否啟用相機 (紅圓偵測)
        """
        print("=" * 60)
        print(" " * 15 + "期末自走競賽系統")
        print(" " * 15 + "Autonomous Cleaning Robot")
        print(" " * 22 + "v1.0")
        print("=" * 60)
        print()

        print("[1/4] 初始化差動驅動...")
        self.drive = DifferentialDrive(ultrasonic_enable=True)  # 自走模式啟用超聲波

        print("[2/4] 初始化 Arduino 通訊...")
        self.arduino = ArduinoController()

        print("[3/4] 初始化沿牆控制器...")
        self.wall_follower = WallFollower()

        print("[4/4] 初始化相機...")
        self.enable_camera = enable_camera
        self.camera = None
        self.red_detector = None
        if enable_camera:
            try:
                self._init_camera()
            except Exception as e:
                print(f"   相機初始化失敗: {e}")
                print("   將停用紅圓偵測功能")
                self.enable_camera = False

        # 狀態
        self.state = RobotState.INIT
        self.corner_count = 0
        self.start_time = None
        self.running = False

        print("\n初始化完成！\n")

    def _init_camera(self):
        """初始化相機"""
        import cv2
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.camera.set(cv2.CAP_PROP_FPS, 15)

        if not self.camera.isOpened():
            raise RuntimeError("無法開啟相機")

        print("   相機已啟用 (320x240 @ 15fps)")

    def run(self):
        """執行自走任務"""
        self.running = True
        self.start_time = time.time()
        self.state = RobotState.FOLLOW_WALL

        print("=" * 60)
        print(" 自走模式啟動！")
        print(f" 目標時間: {self.TARGET_TIME}s ({self.TARGET_TIME//60}:{self.TARGET_TIME%60:02d})")
        print(f" 最大時間: {self.MAX_TIME}s ({self.MAX_TIME//60}:{self.MAX_TIME%60:02d})")
        print(" 按 Ctrl+C 緊急停止")
        print("=" * 60)
        print()

        try:
            while self.running and self.state != RobotState.DONE:
                loop_start = time.time()

                # 主控制迴圈
                self._control_loop()

                # 維持 20Hz
                elapsed = time.time() - loop_start
                sleep_time = 0.05 - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n緊急停止！")

        finally:
            self.stop()

    def _control_loop(self):
        """主控制迴圈"""
        current_time = time.time()
        elapsed = current_time - self.start_time

        # 1. 檢查超時
        if elapsed > self.MAX_TIME:
            print(f"\n超時 ({elapsed:.1f}s)！強制返回出口")
            self.state = RobotState.EXIT

        # 2. 讀取感測器
        sensor_data = self.arduino.receive_sensor_data()
        front_dist = sensor_data.front_distance
        right_dist = sensor_data.right_distance

        # 3. 讀取相機 (若啟用)
        red_detected = False
        if self.enable_camera and self.camera:
            red_detected = self._check_red_circle()

        # 4. 狀態機
        vehicle_cmd = self._process_state(front_dist, right_dist,
                                          red_detected, current_time)

        # 5. 差動驅動轉換
        motor_cmd = self.drive.convert(vehicle_cmd)

        # 6. 發送指令
        self.arduino.send_command(motor_cmd)

        # 7. 顯示狀態
        self._display_status(elapsed, front_dist, right_dist, vehicle_cmd)

    def _process_state(self, front_dist, right_dist, red_detected, current_time):
        """處理狀態機"""

        if self.state == RobotState.FOLLOW_WALL:
            # 檢查紅圓
            if red_detected:
                print(f"\n[{self._elapsed_str()}] 偵測到紅圓！迴避中...")
                self.state = RobotState.AVOID_RED
                return VehicleCommand(0, 0, True)

            # 沿牆控制
            cmd = self.wall_follower.update(front_dist, right_dist, current_time)

            # 檢查是否到達角落 (沿牆控制器進入轉彎狀態)
            if self.wall_follower.get_state() == WallFollower.STATE_TURN_LEFT:
                self.corner_count += 1
                print(f"\n[{self._elapsed_str()}] 到達角落 #{self.corner_count}")

                if self.corner_count >= 4:
                    print(f"[{self._elapsed_str()}] 繞完一圈，準備離開")
                    self.state = RobotState.EXIT

            return cmd

        elif self.state == RobotState.AVOID_RED:
            # 紅圓迴避：左轉
            # TODO: 實作更完整的迴避邏輯
            self.wall_follower.trigger_find_wall(current_time)
            self.state = RobotState.RETURN_TO_WALL
            return VehicleCommand(0, -0.6, True)  # 左轉

        elif self.state == RobotState.RETURN_TO_WALL:
            # 找回右牆
            cmd = self.wall_follower.update(front_dist, right_dist, current_time)
            if self.wall_follower.get_state() == WallFollower.STATE_FORWARD:
                print(f"[{self._elapsed_str()}] 找回右牆，繼續沿牆")
                self.state = RobotState.FOLLOW_WALL
            return cmd

        elif self.state == RobotState.EXIT:
            # 返回出口 (繼續沿牆)
            cmd = self.wall_follower.update(front_dist, right_dist, current_time)

            # 檢測是否回到入口 (再次到達角落)
            if self.wall_follower.get_state() == WallFollower.STATE_TURN_LEFT:
                self.corner_count += 1
                if self.corner_count >= 5:
                    print(f"\n[{self._elapsed_str()}] 回到入口！任務完成")
                    self.state = RobotState.DONE
                    return VehicleCommand(0, 0, False)

            return cmd

        # 預設停止
        return VehicleCommand(0, 0, True)

    def _check_red_circle(self):
        """檢查紅圓 (簡化版)"""
        if not self.camera:
            return False

        import cv2
        import numpy as np

        ret, frame = self.camera.read()
        if not ret:
            return False

        # 轉換到 HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 紅色範圍 (兩段)
        red_lower1 = np.array([0, 120, 70])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 120, 70])
        red_upper2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = mask1 | mask2

        # 計算紅色比例
        red_ratio = cv2.countNonZero(red_mask) / (frame.shape[0] * frame.shape[1])

        # 閾值 3%
        return red_ratio > 0.03

    def _elapsed_str(self):
        """格式化已過時間"""
        if not self.start_time:
            return "00:00"
        elapsed = time.time() - self.start_time
        mins = int(elapsed // 60)
        secs = int(elapsed % 60)
        return f"{mins:02d}:{secs:02d}"

    def _display_status(self, elapsed, front_dist, right_dist, vehicle_cmd):
        """顯示即時狀態"""
        state_names = {
            RobotState.INIT: "INIT",
            RobotState.FOLLOW_WALL: "WALL",
            RobotState.CORNER_SWEEP: "SWEEP",
            RobotState.TURN_LEFT: "TURN",
            RobotState.AVOID_RED: "AVOID",
            RobotState.RETURN_TO_WALL: "FIND",
            RobotState.EXIT: "EXIT",
            RobotState.DONE: "DONE",
        }

        mins = int(elapsed // 60)
        secs = elapsed % 60

        status = (
            f"[{mins:02d}:{secs:05.2f}] "
            f"State:{state_names.get(self.state, '?'):5s} "
            f"| Corner:{self.corner_count} "
            f"| F:{front_dist:3d}cm R:{right_dist:3d}cm "
            f"| Cmd(L:{vehicle_cmd.linear_velocity:+.2f} A:{vehicle_cmd.angular_velocity:+.2f})"
        )

        print(status, end='\r')

    def stop(self):
        """停止機器人"""
        print("\n\n" + "=" * 60)
        print(" 停止中...")

        # 停止馬達
        stop_cmd = VehicleCommand(0, 0, False)
        motor_cmd = self.drive.convert(stop_cmd)
        self.arduino.send_command(motor_cmd)
        time.sleep(0.1)

        # 關閉相機
        if self.camera:
            self.camera.release()

        # 顯示結果
        if self.start_time:
            total_time = time.time() - self.start_time
            mins = int(total_time // 60)
            secs = total_time % 60
            print(f" 總時間: {mins:02d}:{secs:05.2f}")
            print(f" 角落數: {self.corner_count}")

            # 評估速度分
            if total_time < 150:
                speed_score = 70
            elif total_time < 210:
                speed_score = 60
            elif total_time < 270:
                speed_score = 50
            elif total_time < 360:
                speed_score = 40
            else:
                speed_score = 30
            print(f" 預估速度分: {speed_score}")

        # 關閉連接
        self.arduino.close()

        print("=" * 60)
        print(" 系統已停止")
        self.running = False


def parse_args():
    """解析命令列參數"""
    parser = argparse.ArgumentParser(
        description='期末自走競賽系統',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
範例:
  python3 autonomous_main.py                # 正常啟動
  python3 autonomous_main.py --no-camera    # 不使用相機
  python3 autonomous_main.py --debug        # 除錯模式
        """
    )

    parser.add_argument(
        '--no-camera',
        action='store_true',
        help='停用相機 (不偵測紅圓)'
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='啟用除錯模式'
    )

    parser.add_argument(
        '--version',
        action='version',
        version='期末自走競賽系統 v1.0'
    )

    return parser.parse_args()


def main():
    """主程式"""
    args = parse_args()

    if args.debug:
        config.DEBUG_MODE = True
        config.VERBOSE_SERIAL = True
        print("除錯模式已啟用\n")

    try:
        controller = AutonomousController(enable_camera=not args.no_camera)
        controller.run()

    except KeyboardInterrupt:
        print("\n使用者中止程式")
        sys.exit(0)

    except Exception as e:
        print(f"\n程式發生錯誤: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
