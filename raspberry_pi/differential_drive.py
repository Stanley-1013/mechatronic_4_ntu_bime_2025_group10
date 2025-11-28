#!/usr/bin/env python3
"""
differential_drive.py - 差動驅動演算法
版本: 1.0
日期: 2025-10-31

實作差動驅動公式，將線性速度和角速度轉換為左右輪 PWM 值。
參考: 03_SD_系統設計.md - 3.3 差動驅動演算法
"""

import config


class VehicleCommand:
    """車輛控制指令（統一格式）"""

    def __init__(self, linear_velocity=0.0, angular_velocity=0.0, vacuum_motor=False):
        """
        初始化車輛指令

        Args:
            linear_velocity: 線性速度 (-1.0 ~ +1.0)
                           正值 = 前進，負值 = 後退
            angular_velocity: 角速度 (-1.0 ~ +1.0)
                            正值 = 右轉，負值 = 左轉
            vacuum_motor: 吸塵器馬達狀態 (True/False)
        """
        self.linear_velocity = self._clamp(linear_velocity, -1.0, 1.0)
        self.angular_velocity = self._clamp(angular_velocity, -1.0, 1.0)
        self.vacuum_motor = vacuum_motor

    @staticmethod
    def _clamp(value, min_val, max_val):
        """限制數值範圍"""
        return max(min_val, min(max_val, value))

    def __repr__(self):
        return (f"VehicleCommand(linear={self.linear_velocity:+.2f}, "
                f"angular={self.angular_velocity:+.2f}, "
                f"vacuum={self.vacuum_motor})")


class MotorCommand:
    """馬達控制指令（PWM 格式）"""

    def __init__(self, left_pwm=0, right_pwm=0, vacuum=False, ultrasonic_enable=False):
        """
        初始化馬達指令

        Args:
            left_pwm: 左輪 PWM (-255 ~ +255)
            right_pwm: 右輪 PWM (-255 ~ +255)
            vacuum: 吸塵器馬達狀態 (True/False)
            ultrasonic_enable: 超聲波啟用狀態 (True/False)
        """
        self.left_pwm = self._clamp(left_pwm, config.MIN_PWM_VALUE, config.MAX_PWM_VALUE)
        self.right_pwm = self._clamp(right_pwm, config.MIN_PWM_VALUE, config.MAX_PWM_VALUE)
        self.vacuum = vacuum
        self.ultrasonic_enable = ultrasonic_enable

    @staticmethod
    def _clamp(value, min_val, max_val):
        """限制數值範圍"""
        return int(max(min_val, min(max_val, value)))

    def __repr__(self):
        return (f"MotorCommand(left={self.left_pwm:+4d}, "
                f"right={self.right_pwm:+4d}, "
                f"vacuum={self.vacuum}, "
                f"ultrasonic={self.ultrasonic_enable})")


class DifferentialDrive:
    """差動驅動控制器"""

    def __init__(self, left_scale=None, right_scale=None, ultrasonic_enable=False):
        """
        初始化差動驅動控制器

        Args:
            left_scale: 左輪速度校準係數（預設從 config 讀取）
            right_scale: 右輪速度校準係數（預設從 config 讀取）
            ultrasonic_enable: 超聲波啟用狀態（自走模式啟用，遙控模式停用）
        """
        self.left_scale = left_scale if left_scale is not None else config.MOTOR_LEFT_SCALE
        self.right_scale = right_scale if right_scale is not None else config.MOTOR_RIGHT_SCALE
        self.ultrasonic_enable = ultrasonic_enable

    def convert(self, vehicle_cmd: VehicleCommand) -> MotorCommand:
        """
        將車輛指令轉換為馬達指令（差動驅動演算法）

        差動驅動公式:
            left_speed = linear_velocity - angular_velocity
            right_speed = linear_velocity + angular_velocity

        Args:
            vehicle_cmd: VehicleCommand 物件

        Returns:
            MotorCommand 物件

        參考: 01_SRS_軟體需求規格書.md - FR2
        """
        # 步驟 1: 差動驅動公式
        left_speed = vehicle_cmd.linear_velocity - vehicle_cmd.angular_velocity
        right_speed = vehicle_cmd.linear_velocity + vehicle_cmd.angular_velocity

        # 步驟 2: 限制範圍 [-1.0, 1.0]
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        # 步驟 3: 應用校準係數
        left_speed *= self.left_scale
        right_speed *= self.right_scale

        # 步驟 4: 轉換為 PWM (-255 ~ 255)
        left_pwm = int(left_speed * config.MAX_PWM_VALUE)
        right_pwm = int(right_speed * config.MAX_PWM_VALUE)

        # 步驟 5: 建立馬達指令
        motor_cmd = MotorCommand(
            left_pwm=left_pwm,
            right_pwm=right_pwm,
            vacuum=vehicle_cmd.vacuum_motor,
            ultrasonic_enable=self.ultrasonic_enable
        )

        return motor_cmd

    def set_calibration(self, left_scale, right_scale):
        """
        設定馬達校準係數

        Args:
            left_scale: 左輪速度倍率
            right_scale: 右輪速度倍率
        """
        self.left_scale = left_scale
        self.right_scale = right_scale


# ==================== 測試程式 ====================
if __name__ == "__main__":
    print("========== 差動驅動演算法測試 ==========\n")

    drive = DifferentialDrive()

    # 測試案例 1: 全速前進
    print("[Test 1] 全速前進")
    cmd = VehicleCommand(linear_velocity=1.0, angular_velocity=0.0)
    result = drive.convert(cmd)
    print(f"輸入: {cmd}")
    print(f"輸出: {result}")
    print()

    # 測試案例 2: 全速後退
    print("[Test 2] 全速後退")
    cmd = VehicleCommand(linear_velocity=-1.0, angular_velocity=0.0)
    result = drive.convert(cmd)
    print(f"輸入: {cmd}")
    print(f"輸出: {result}")
    print()

    # 測試案例 3: 原地左轉
    print("[Test 3] 原地左轉")
    cmd = VehicleCommand(linear_velocity=0.0, angular_velocity=-1.0)
    result = drive.convert(cmd)
    print(f"輸入: {cmd}")
    print(f"輸出: {result}")
    print()

    # 測試案例 4: 原地右轉
    print("[Test 4] 原地右轉")
    cmd = VehicleCommand(linear_velocity=0.0, angular_velocity=1.0)
    result = drive.convert(cmd)
    print(f"輸入: {cmd}")
    print(f"輸出: {result}")
    print()

    # 測試案例 5: 前進左轉
    print("[Test 5] 前進左轉")
    cmd = VehicleCommand(linear_velocity=1.0, angular_velocity=-0.5)
    result = drive.convert(cmd)
    print(f"輸入: {cmd}")
    print(f"輸出: {result}")
    print()

    # 測試案例 6: 前進右轉
    print("[Test 6] 前進右轉")
    cmd = VehicleCommand(linear_velocity=1.0, angular_velocity=0.5)
    result = drive.convert(cmd)
    print(f"輸入: {cmd}")
    print(f"輸出: {result}")
    print()

    # 測試案例 7: 停止
    print("[Test 7] 停止")
    cmd = VehicleCommand(linear_velocity=0.0, angular_velocity=0.0)
    result = drive.convert(cmd)
    print(f"輸入: {cmd}")
    print(f"輸出: {result}")
    print()

    # 測試案例 8: 溢位保護
    print("[Test 8] 溢位保護（linear=1.0, angular=1.0）")
    cmd = VehicleCommand(linear_velocity=1.0, angular_velocity=1.0)
    result = drive.convert(cmd)
    print(f"輸入: {cmd}")
    print(f"輸出: {result}")
    print(f"備註: right_pwm 應被限制在 255")
    print()

    print("========== 測試完成 ==========")
