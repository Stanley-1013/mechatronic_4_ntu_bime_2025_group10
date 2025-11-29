#!/usr/bin/env python3
"""
autonomous_main.py - 期末自走競賽主程式
版本: 2.0 (新增 IMU 角度控制)
日期: 2025-11-29

自走模式入口程式，使用沿右牆策略完成清掃任務。
參考: FINAL_AUTONOMOUS_PLAN.md

v2.0 變更:
- 整合 MPU6050 IMU 提升轉彎精確度
- 新增 IMU 狀態顯示
"""

import argparse
import sys
import time
from enum import Enum

import config
from differential_drive import DifferentialDrive, VehicleCommand
from arduino_controller import ArduinoController
from wall_follower import WallFollower
from red_detector import RedDetector


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

    def __init__(self, enable_camera=True, use_imu=True):
        """
        初始化自走控制器

        Args:
            enable_camera: 是否啟用相機 (紅圓偵測)
            use_imu: 是否使用 IMU 角度控制 (預設啟用)
        """
        print("=" * 60)
        print(" " * 15 + "期末自走競賽系統")
        print(" " * 15 + "Autonomous Cleaning Robot")
        print(" " * 18 + "v2.0 (IMU Enhanced)")
        print("=" * 60)
        print()

        print("[1/4] 初始化差動驅動...")
        self.drive = DifferentialDrive(ultrasonic_enable=True)  # 自走模式啟用超聲波

        print("[2/4] 初始化 Arduino 通訊...")
        self.arduino = ArduinoController()

        print("[3/4] 初始化沿牆控制器...")
        self.use_imu = use_imu
        self.wall_follower = WallFollower(use_imu=use_imu)

        print("[4/4] 初始化紅色偵測器...")
        self.enable_camera = enable_camera
        self.red_detector = None
        if enable_camera:
            try:
                self.red_detector = RedDetector()
                if not self.red_detector.start():  # 使用背景執行緒模式
                    raise RuntimeError("無法開啟攝影機")
                print("   紅色偵測器已啟用 (背景執行緒)")
            except Exception as e:
                print(f"   紅色偵測器初始化失敗: {e}")
                print("   將停用紅圓偵測功能")
                self.enable_camera = False
                self.red_detector = None

        # 狀態
        self.state = RobotState.INIT
        self.corner_count = 0
        self.start_time = None
        self.running = False

        # 紅色區域偵測
        self.red_zone_ahead = False           # 前方是否有紅色區域
        self.last_turn_time = 0               # 上次轉彎完成時間
        self.red_detection_delay = 1.0        # 轉彎後延遲偵測時間 (秒)

        # IMU 狀態追蹤
        self.imu_available = False
        self.last_yaw = 0.0

        print("\n初始化完成！\n")

    def run(self):
        """執行自走任務"""
        self.running = True
        self.start_time = time.time()
        self.state = RobotState.FOLLOW_WALL
        self.vacuum_on = True  # 自走模式吸塵器常開

        print("=" * 60)
        print(" 自走模式啟動！")
        print(f" 目標時間: {self.TARGET_TIME}s ({self.TARGET_TIME//60}:{self.TARGET_TIME%60:02d})")
        print(f" 最大時間: {self.MAX_TIME}s ({self.MAX_TIME//60}:{self.MAX_TIME%60:02d})")
        print(f" IMU 角度控制: {'啟用' if self.use_imu else '停用'}")
        print(" 吸塵器: 常開")
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

        # 2. 讀取感測器 (含 IMU)
        sensor_data = self.arduino.receive_sensor_data()
        front_dist = sensor_data.front_distance
        right_dist = sensor_data.right_distance
        yaw = sensor_data.yaw
        imu_valid = sensor_data.imu_valid

        # 更新 IMU 狀態
        self.imu_available = imu_valid
        self.last_yaw = yaw

        # 3. 紅色偵測 (若啟用) - 轉彎後延遲偵測
        red_detected = False
        if self.enable_camera and self.red_detector:
            # 轉彎後延遲一段時間才啟用紅色偵測，避免一轉過去就觸發
            time_since_turn = current_time - self.last_turn_time
            if time_since_turn > self.red_detection_delay:
                red_detected, area = self.red_detector.detect()
                if red_detected and not self.red_zone_ahead:
                    self.red_zone_ahead = True
                    print(f"\n[{self._elapsed_str()}] 偵測到紅色區域！(面積:{area}) 標記避開")

        # 4. 狀態機 (傳入 IMU 資料)
        vehicle_cmd = self._process_state(front_dist, right_dist,
                                          red_detected, current_time,
                                          yaw, imu_valid)

        # 5. 差動驅動轉換
        motor_cmd = self.drive.convert(vehicle_cmd)

        # 6. 發送指令
        self.arduino.send_command(motor_cmd)

        # 7. 顯示狀態 (含 IMU)
        self._display_status(elapsed, front_dist, right_dist, vehicle_cmd, yaw, imu_valid)

    def _process_state(self, front_dist, right_dist, red_detected, current_time,
                       yaw=0.0, imu_valid=False):
        """處理狀態機 (吸塵器自走模式常開)"""

        if self.state == RobotState.FOLLOW_WALL:
            # 偵測到紅色且尚未到角落 → 提早左轉離開，避免靠近紅色紙屑
            if self.red_zone_ahead and self.wall_follower.get_state() == WallFollower.STATE_FORWARD:
                # 檢查是否已經很靠近前方（快到角落了）
                if front_dist < 60:  # 60cm 內提早轉彎
                    print(f"\n[{self._elapsed_str()}] 紅色區域前方 {front_dist}cm - 提早左轉避開")
                    self.wall_follower.trigger_turn_left(current_time, yaw if imu_valid else None)
                    self.corner_count += 1
                    self.red_zone_ahead = False
                    self.last_turn_time = current_time
                    return VehicleCommand(0, -0.7, self.vacuum_on)  # 左轉

            # 沿牆控制 (傳入 IMU 資料)
            cmd = self.wall_follower.update(front_dist, right_dist, current_time, yaw, imu_valid)

            # 檢查是否到達角落 (沿牆控制器進入轉彎狀態)
            if self.wall_follower.get_state() == WallFollower.STATE_TURN_LEFT:
                self.corner_count += 1
                print(f"\n[{self._elapsed_str()}] 到達角落 #{self.corner_count}")

                # 判斷是否為紅色區域角落
                if self.red_zone_ahead:
                    # 紅色區域：跳過角落掃描，直接轉彎離開
                    print(f"[{self._elapsed_str()}] 紅色區域角落 - 跳過掃描，直接轉彎")
                    self.red_zone_ahead = False  # 重置標記
                    self.last_turn_time = current_time
                    # 保持 STATE_TURN_LEFT，不觸發 SWEEP
                else:
                    # 正常角落：執行角落掃描確保吸到紙屑
                    print(f"[{self._elapsed_str()}] 正常角落 - 執行角落掃描")
                    self.wall_follower.trigger_sweep(current_time)
                    self.state = RobotState.CORNER_SWEEP

                if self.corner_count >= 4:
                    print(f"[{self._elapsed_str()}] 繞完一圈，準備離開")
                    self.state = RobotState.EXIT

            return cmd

        elif self.state == RobotState.CORNER_SWEEP:
            # 角落掃描中（由 wall_follower 內部狀態機處理）
            cmd = self.wall_follower.update(front_dist, right_dist, current_time, yaw, imu_valid)

            # 掃描完成後回到沿牆
            if self.wall_follower.get_state() == WallFollower.STATE_FORWARD:
                print(f"[{self._elapsed_str()}] 角落掃描完成，繼續沿牆")
                self.state = RobotState.FOLLOW_WALL
                self.last_turn_time = current_time  # 記錄轉彎完成時間

            return cmd

        elif self.state == RobotState.AVOID_RED:
            # 紅圓迴避：左轉（備用，主要靠角落判斷避開）
            self.wall_follower.trigger_find_wall(current_time)
            self.state = RobotState.RETURN_TO_WALL
            return VehicleCommand(0, -0.6, self.vacuum_on)  # 左轉

        elif self.state == RobotState.RETURN_TO_WALL:
            # 找回右牆
            cmd = self.wall_follower.update(front_dist, right_dist, current_time, yaw, imu_valid)
            if self.wall_follower.get_state() == WallFollower.STATE_FORWARD:
                print(f"[{self._elapsed_str()}] 找回右牆，繼續沿牆")
                self.state = RobotState.FOLLOW_WALL
                self.last_turn_time = current_time  # 記錄轉彎完成時間
            return cmd

        elif self.state == RobotState.EXIT:
            # 返回出口 (繼續沿牆)
            cmd = self.wall_follower.update(front_dist, right_dist, current_time, yaw, imu_valid)

            # 檢測是否回到入口 (再次到達角落)
            if self.wall_follower.get_state() == WallFollower.STATE_TURN_LEFT:
                self.corner_count += 1
                self.last_turn_time = current_time  # 記錄轉彎時間
                if self.corner_count >= 5:
                    print(f"\n[{self._elapsed_str()}] 回到入口！任務完成")
                    self.state = RobotState.DONE
                    self.vacuum_on = False  # 任務完成才關閉吸塵器
                    return VehicleCommand(0, 0, False)

            return cmd

        # 預設停止 (但吸塵器保持開啟)
        return VehicleCommand(0, 0, self.vacuum_on)

    def _elapsed_str(self):
        """格式化已過時間"""
        if not self.start_time:
            return "00:00"
        elapsed = time.time() - self.start_time
        mins = int(elapsed // 60)
        secs = int(elapsed % 60)
        return f"{mins:02d}:{secs:02d}"

    def _display_status(self, elapsed, front_dist, right_dist, vehicle_cmd,
                        yaw=0.0, imu_valid=False):
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

        # IMU 狀態字串
        imu_str = f"Yaw:{yaw:+6.1f}°" if imu_valid else "IMU:N/A"

        status = (
            f"[{mins:02d}:{secs:05.2f}] "
            f"State:{state_names.get(self.state, '?'):5s} "
            f"| Corner:{self.corner_count} "
            f"| F:{front_dist:3d}cm R:{right_dist:3d}cm "
            f"| {imu_str} "
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

        # 關閉紅色偵測器
        if self.red_detector:
            self.red_detector.close()

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
        description='期末自走競賽系統 v2.0 (IMU Enhanced)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
範例:
  python3 autonomous_main.py                # 正常啟動 (含 IMU)
  python3 autonomous_main.py --no-camera    # 不使用相機
  python3 autonomous_main.py --no-imu       # 不使用 IMU (時間控制轉彎)
  python3 autonomous_main.py --debug        # 除錯模式
        """
    )

    parser.add_argument(
        '--no-camera',
        action='store_true',
        help='停用相機 (不偵測紅圓)'
    )

    parser.add_argument(
        '--no-imu',
        action='store_true',
        help='停用 IMU 角度控制 (使用時間控制轉彎)'
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='啟用除錯模式'
    )

    parser.add_argument(
        '--version',
        action='version',
        version='期末自走競賽系統 v2.0 (IMU Enhanced)'
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
        controller = AutonomousController(
            enable_camera=not args.no_camera,
            use_imu=not args.no_imu
        )
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
