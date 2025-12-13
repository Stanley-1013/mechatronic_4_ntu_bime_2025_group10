#!/usr/bin/env python3
"""
imu_processor.py - IMU 資料處理器
版本: 1.0
日期: 2025-11-29

處理 MPU6050 IMU 資料，提供：
- 精確角度轉彎控制
- 直線行駛航向修正
- 紅色禁區離開穩定性

使用方式:
    from imu_processor import IMUProcessor

    imu = IMUProcessor()

    # 開始 90 度左轉
    imu.start_turn(-90)

    # 檢查轉彎是否完成
    while not imu.is_turn_complete(current_yaw):
        ...

    # 直線行駛航向修正
    correction = imu.get_heading_correction(current_yaw)
"""

import config


class IMUProcessor:
    """IMU 資料處理器"""

    def __init__(self):
        """初始化 IMU 處理器"""
        # 轉彎控制
        self.turn_active = False
        self.turn_start_yaw = 0.0
        self.turn_target_yaw = 0.0
        self.turn_direction = 0  # -1=左轉, +1=右轉

        # 航向控制
        self.target_heading = None
        self.heading_locked = False

        # 狀態
        self.last_yaw = 0.0
        self.imu_available = False

        # 參數
        self.yaw_tolerance = config.IMU_YAW_TOLERANCE
        self.heading_kp = config.IMU_HEADING_KP

        print("[IMUProcessor] 初始化完成")

    def update(self, yaw: float, imu_valid: bool):
        """
        更新 IMU 資料

        Args:
            yaw: 當前 Yaw 角度 (度)
            imu_valid: IMU 資料是否有效
        """
        self.last_yaw = yaw
        self.imu_available = imu_valid

    def start_turn(self, degrees: float, current_yaw: float = None):
        """
        開始角度控制轉彎

        Args:
            degrees: 轉彎角度 (正=右轉, 負=左轉)
            current_yaw: 當前 Yaw (可選，若不提供則使用 last_yaw)

        Example:
            imu.start_turn(-90)  # 左轉 90 度
            imu.start_turn(45)   # 右轉 45 度
        """
        if current_yaw is None:
            current_yaw = self.last_yaw

        self.turn_active = True
        self.turn_start_yaw = current_yaw
        self.turn_target_yaw = self._normalize_angle(current_yaw + degrees)
        self.turn_direction = 1 if degrees > 0 else -1

        print(f"[IMUProcessor] 開始轉彎: {current_yaw:.1f}° -> {self.turn_target_yaw:.1f}° ({degrees:+.0f}°)")

    def is_turn_complete(self, current_yaw: float = None) -> bool:
        """
        檢查轉彎是否完成

        Args:
            current_yaw: 當前 Yaw (可選)

        Returns:
            bool: True 表示已到達目標角度
        """
        if not self.turn_active:
            return True

        if current_yaw is None:
            current_yaw = self.last_yaw

        # 計算角度差 (處理跨越 ±180° 邊界的情況)
        diff = self._angle_diff(current_yaw, self.turn_target_yaw)

        if abs(diff) < self.yaw_tolerance:
            self.turn_active = False
            print(f"[IMUProcessor] 轉彎完成: 目標={self.turn_target_yaw:.1f}°, 實際={current_yaw:.1f}°, 誤差={diff:.1f}°")
            return True

        return False

    def get_turn_progress(self, current_yaw: float = None) -> float:
        """
        取得轉彎進度

        Args:
            current_yaw: 當前 Yaw (可選)

        Returns:
            float: 進度 0.0 ~ 1.0
        """
        if not self.turn_active:
            return 1.0

        if current_yaw is None:
            current_yaw = self.last_yaw

        total = self._angle_diff(self.turn_start_yaw, self.turn_target_yaw)
        current = self._angle_diff(self.turn_start_yaw, current_yaw)

        if abs(total) < 0.1:
            return 1.0

        progress = current / total
        return max(0.0, min(1.0, progress))

    def cancel_turn(self):
        """取消當前轉彎"""
        self.turn_active = False
        print("[IMUProcessor] 轉彎取消")

    def lock_heading(self, heading: float = None):
        """
        鎖定航向 (用於直線行駛修正)

        Args:
            heading: 目標航向 (可選，若不提供則使用當前 Yaw)
        """
        if heading is None:
            heading = self.last_yaw

        self.target_heading = heading
        self.heading_locked = True
        print(f"[IMUProcessor] 航向鎖定: {heading:.1f}°")

    def unlock_heading(self):
        """解除航向鎖定"""
        self.heading_locked = False
        self.target_heading = None

    def get_heading_correction(self, current_yaw: float = None) -> float:
        """
        計算航向修正量 (用於直線行駛)

        Args:
            current_yaw: 當前 Yaw (可選)

        Returns:
            float: 角速度修正量 (正=右修正, 負=左修正)
                   可直接加到 angular_velocity 上
        """
        if not self.heading_locked or self.target_heading is None:
            return 0.0

        if current_yaw is None:
            current_yaw = self.last_yaw

        # 計算誤差
        error = self._angle_diff(current_yaw, self.target_heading)

        # P 控制
        correction = error * self.heading_kp

        # 限制修正量
        max_correction = 0.3
        correction = max(-max_correction, min(max_correction, correction))

        return correction

    def is_turn_active(self) -> bool:
        """是否正在執行轉彎"""
        return self.turn_active

    def is_heading_locked(self) -> bool:
        """航向是否已鎖定"""
        return self.heading_locked

    def get_target_yaw(self) -> float:
        """取得轉彎目標角度"""
        return self.turn_target_yaw if self.turn_active else self.last_yaw

    def get_target_heading(self) -> float:
        """取得鎖定航向"""
        return self.target_heading if self.heading_locked else self.last_yaw

    def _normalize_angle(self, angle: float) -> float:
        """
        將角度正規化到 -180 ~ +180 範圍

        Args:
            angle: 輸入角度 (度)

        Returns:
            float: 正規化後的角度
        """
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def _angle_diff(self, from_angle: float, to_angle: float) -> float:
        """
        計算角度差 (考慮跨越 ±180° 邊界)

        Args:
            from_angle: 起始角度 (度)
            to_angle: 目標角度 (度)

        Returns:
            float: 角度差 (正=順時針, 負=逆時針)
        """
        diff = to_angle - from_angle
        return self._normalize_angle(diff)


# ==================== 測試程式 ====================
if __name__ == "__main__":
    print("=" * 50)
    print("  IMU 處理器測試")
    print("=" * 50)
    print()

    imu = IMUProcessor()

    # 測試轉彎控制
    print("=== 測試 1: 左轉 90 度 ===")
    imu.start_turn(-90, current_yaw=0)

    test_angles = [0, -20, -45, -70, -85, -90, -92]
    for yaw in test_angles:
        complete = imu.is_turn_complete(yaw)
        progress = imu.get_turn_progress(yaw)
        print(f"  Yaw: {yaw:+6.1f}°, Progress: {progress*100:5.1f}%, Complete: {complete}")

    print()
    print("=== 測試 2: 右轉 45 度 ===")
    imu.start_turn(45, current_yaw=90)

    test_angles = [90, 110, 130, 135, 137]
    for yaw in test_angles:
        complete = imu.is_turn_complete(yaw)
        progress = imu.get_turn_progress(yaw)
        print(f"  Yaw: {yaw:+6.1f}°, Progress: {progress*100:5.1f}%, Complete: {complete}")

    print()
    print("=== 測試 3: 跨越 180° 邊界 ===")
    imu.start_turn(-90, current_yaw=150)
    print(f"  Target: {imu.get_target_yaw():.1f}°")

    test_angles = [150, 170, 180, -170, -150, -130]
    for yaw in test_angles:
        complete = imu.is_turn_complete(yaw)
        print(f"  Yaw: {yaw:+6.1f}°, Complete: {complete}")

    print()
    print("=== 測試 4: 航向修正 ===")
    imu.lock_heading(0)

    test_angles = [0, 5, -5, 10, -10, 20]
    for yaw in test_angles:
        correction = imu.get_heading_correction(yaw)
        print(f"  Yaw: {yaw:+6.1f}°, Correction: {correction:+.3f}")

    imu.unlock_heading()

    print()
    print("=== 測試完成 ===")
