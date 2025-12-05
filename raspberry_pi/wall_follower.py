#!/usr/bin/env python3
"""
wall_follower.py - 沿右牆控制器
版本: 3.0 (簡化角落處理)
日期: 2025-11-29

實作沿右牆行駛演算法，使用前方+右側超聲波感測器。

v3.0 變更:
- 簡化角落處理：到角落 -> 停止 -> 後退左轉
- 移除複雜的角落掃描動作
- 新增後退左轉狀態

場地規格:
- 紙屑距牆 15cm
- 超聲波在車體右側，吸塵器在中間
- 目標距牆 15cm (超聲波對準紙屑線，吸塵器自然涵蓋)
"""

import config
from differential_drive import VehicleCommand
from imu_processor import IMUProcessor


class WallFollower:
    """沿右牆控制器"""

    # 沿牆參數 (紙屑距牆 15cm，超聲波在車體右側)
    TARGET_RIGHT_DISTANCE = 15      # 目標右側距離 (cm) - 超聲波對準紙屑線
    RIGHT_DISTANCE_TOLERANCE = 5    # 右側距離容許誤差 (±5cm)

    # 前方偵測參數
    FRONT_STOP_DISTANCE = 15        # 前方停止距離 (cm) - 到角落
    FRONT_SLOW_DISTANCE = 40        # 前方減速距離 (cm)

    # 速度參數 (normalized: -1.0 ~ +1.0)
    BASE_LINEAR_SPEED = 0.6         # 基礎線性速度
    SLOW_LINEAR_SPEED = 0.35        # 減速時線性速度
    BACKUP_SPEED = -0.4             # 後退速度
    TURN_ANGULAR_SPEED = 0.7        # 轉彎角速度
    WALL_FOLLOW_KP = 0.025          # 沿牆 P 控制增益

    # 狀態機
    STATE_FORWARD = "forward"       # 直行沿牆
    STATE_BACKUP = "backup"         # 後退 (角落時)
    STATE_TURN_LEFT = "turn_left"   # 左轉（後退完或只有前牆）
    STATE_FIND_WALL = "find_wall"   # 尋找右牆 (右前方移動)

    def __init__(self, use_imu=True):
        """
        初始化沿牆控制器

        Args:
            use_imu: 是否使用 IMU 角度控制 (預設啟用)
        """
        self.state = self.STATE_FORWARD
        self.vacuum_on = True  # 預設開啟吸塵器

        # IMU 處理器
        self.use_imu = use_imu
        self.imu = IMUProcessor() if use_imu else None

        # 後退計時器 (角落時小幅後退)
        self.backup_start_time = None
        self.backup_duration = 0.3  # 後退持續時間 (秒) - 只退一點點

        # 轉彎計時器 (IMU 不可用時的備援)
        self.turn_start_time = None
        self.turn_duration = 0.8  # 原地左轉持續時間 (秒)

        # 找牆計時
        self.find_wall_start_time = None
        self.find_wall_timeout = 3.0  # 找牆超時 (秒)

        print("WallFollower v3.0 初始化完成")
        print(f"   目標右側距離: {self.TARGET_RIGHT_DISTANCE} cm")
        print(f"   前方停止距離: {self.FRONT_STOP_DISTANCE} cm")
        print(f"   IMU 角度控制: {'啟用' if use_imu else '停用'}")

    def update(self, front_distance: int, right_distance: int,
               current_time: float, yaw: float = 0.0, imu_valid: bool = False) -> VehicleCommand:
        """
        根據感測器資料更新控制指令

        Args:
            front_distance: 前方距離 (cm), 999=無效
            right_distance: 右側距離 (cm), 999=無效
            current_time: 當前時間 (time.time())
            yaw: 當前 Yaw 角度 (度)
            imu_valid: IMU 資料是否有效

        Returns:
            VehicleCommand: 車輛控制指令
        """
        # 更新 IMU 處理器
        if self.imu:
            self.imu.update(yaw, imu_valid)

        # 處理無效感測器資料
        front_valid = front_distance != 999 and front_distance > 0
        right_valid = right_distance != 999 and right_distance > 0

        # 狀態機邏輯
        if self.state == self.STATE_FORWARD:
            return self._state_forward(front_distance, right_distance,
                                       front_valid, right_valid, current_time, yaw, imu_valid)

        elif self.state == self.STATE_BACKUP:
            return self._state_backup(current_time, yaw, imu_valid)

        elif self.state == self.STATE_TURN_LEFT:
            return self._state_turn_left(current_time, yaw, imu_valid)

        elif self.state == self.STATE_FIND_WALL:
            return self._state_find_wall(right_distance, right_valid, current_time)

        # 預設停止
        return VehicleCommand(0, 0, self.vacuum_on)

    def _state_forward(self, front_dist, right_dist,
                       front_valid, right_valid, current_time,
                       yaw=0.0, imu_valid=False):
        """
        直行沿牆狀態

        邏輯:
        1. 前方+右側都偵測到牆 (角落) -> 後退再左轉
        2. 只有前方偵測到牆 -> 原地左轉
        3. 前後左右都沒牆 -> 右前方找牆
        4. 正常沿右牆行駛 (P 控制)
        """
        linear = self.BASE_LINEAR_SPEED
        angular = 0.0

        # 情況 1: 角落 - 前方有牆 + 右側有牆
        if front_valid and front_dist < self.FRONT_STOP_DISTANCE:
            if right_valid and right_dist < 50:
                # 角落：先後退一點點
                self.state = self.STATE_BACKUP
                self.backup_start_time = current_time
                print(f"[WallFollower] 角落! 前方 {front_dist}cm, 右側 {right_dist}cm -> 後退")
                return VehicleCommand(self.BACKUP_SPEED, 0, self.vacuum_on)
            else:
                # 情況 2: 只有前方有牆，沒有右牆 -> 直接原地左轉
                self.state = self.STATE_TURN_LEFT
                self.turn_start_time = current_time
                if self.imu and imu_valid:
                    self.imu.start_turn(-90, yaw)
                    print(f"[WallFollower] 前方有牆 {front_dist}cm -> 原地左轉 (IMU)")
                else:
                    print(f"[WallFollower] 前方有牆 {front_dist}cm -> 原地左轉 (時間)")
                return VehicleCommand(0, -self.TURN_ANGULAR_SPEED, self.vacuum_on)

        # 情況 3: 前後左右都沒牆 -> 右前方找牆 (入口在下方中間，往右走找右牆)
        if (not front_valid or front_dist > 80) and (not right_valid or right_dist > 80):
            self.state = self.STATE_FIND_WALL
            self.find_wall_start_time = current_time
            print("[WallFollower] 感測器都沒找到牆 -> 右前方尋牆")
            # 右轉+前進: angular 負值讓左輪快、右輪慢 → 右轉
            # left=0.75 (PWM 191), right=0.45 (PWM 115)
            return VehicleCommand(0.6, -0.15, self.vacuum_on)

        # 情況 4: 正常沿牆 - 前方減速
        if front_valid and front_dist < self.FRONT_SLOW_DISTANCE:
            linear = self.SLOW_LINEAR_SPEED

        # 沿右牆控制（P 控制器）
        if right_valid and right_dist < 80:
            error = right_dist - self.TARGET_RIGHT_DISTANCE
            # error > 0: 離牆太遠，需要右轉 → angular 負值 (左快右慢)
            # error < 0: 離牆太近，需要左轉 → angular 正值 (左慢右快)
            # 所以 angular = -error * KP
            angular = -error * self.WALL_FOLLOW_KP
            angular = max(-0.35, min(0.35, angular))

        return VehicleCommand(linear, angular, self.vacuum_on)

    def _state_backup(self, current_time, yaw=0.0, imu_valid=False):
        """
        後退狀態 (角落時小幅後退)

        完成後進入原地左轉
        """
        elapsed = current_time - self.backup_start_time

        if elapsed > self.backup_duration:
            # 後退完成，進入原地左轉
            self.state = self.STATE_TURN_LEFT
            self.turn_start_time = current_time
            if self.imu and imu_valid:
                self.imu.start_turn(-90, yaw)
                print(f"[WallFollower] 後退完成 -> 原地左轉 (IMU: {yaw:.1f}°)")
            else:
                print("[WallFollower] 後退完成 -> 原地左轉 (時間)")
            return VehicleCommand(0, -self.TURN_ANGULAR_SPEED, self.vacuum_on)

        # 繼續後退
        return VehicleCommand(self.BACKUP_SPEED, 0, self.vacuum_on)

    def _state_turn_left(self, current_time, yaw=0.0, imu_valid=False):
        """
        原地左轉狀態

        結束條件 (優先順序):
        1. IMU 模式: 達到目標角度 (±5°)
        2. 時間模式: 轉彎時間到 (IMU 不可用或未啟動時)
        """
        elapsed = current_time - self.turn_start_time

        # === 優先使用 IMU 角度控制 ===
        if self.imu and self.imu.is_turn_active():
            if imu_valid:
                if self.imu.is_turn_complete(yaw):
                    self.state = self.STATE_FORWARD
                    print(f"[WallFollower] 左轉完成 (IMU: {yaw:.1f}°) -> 直行")
                    return VehicleCommand(self.BASE_LINEAR_SPEED, 0, self.vacuum_on)
                # IMU 有效，繼續轉
                return VehicleCommand(0, -self.TURN_ANGULAR_SPEED, self.vacuum_on)
            else:
                # IMU 轉彎中但資料無效，等待資料恢復 (繼續轉)
                # 同時用時間做備援
                if elapsed > self.turn_duration * 1.5:  # 比正常時間長 50% 就強制結束
                    self.imu.cancel_turn()
                    self.state = self.STATE_FORWARD
                    print(f"[WallFollower] 左轉超時 (IMU 資料無效) -> 直行")
                    return VehicleCommand(self.BASE_LINEAR_SPEED, 0, self.vacuum_on)
                return VehicleCommand(0, -self.TURN_ANGULAR_SPEED, self.vacuum_on)

        # === 純時間控制模式 ===
        if elapsed > self.turn_duration:
            self.state = self.STATE_FORWARD
            print(f"[WallFollower] 左轉完成 ({elapsed:.2f}s) -> 直行")
            return VehicleCommand(self.BASE_LINEAR_SPEED, 0, self.vacuum_on)

        # 繼續原地左轉 (linear=0)
        return VehicleCommand(0, -self.TURN_ANGULAR_SPEED, self.vacuum_on)

    def _state_find_wall(self, right_dist, right_valid, current_time):
        """
        尋找右牆狀態

        策略: 右前方移動直到找到右牆
        """
        elapsed = current_time - self.find_wall_start_time

        # 超時保護
        if elapsed > self.find_wall_timeout:
            self.state = self.STATE_FORWARD
            print("[WallFollower] 找牆超時 -> 直行")
            return VehicleCommand(self.BASE_LINEAR_SPEED, 0, self.vacuum_on)

        # 找到右牆
        if right_valid and 10 < right_dist < 50:
            self.state = self.STATE_FORWARD
            print(f"[WallFollower] 找到右牆 ({right_dist}cm) -> 沿牆")
            return VehicleCommand(self.BASE_LINEAR_SPEED, 0, self.vacuum_on)

        # 右前方移動: angular 負值讓左輪快、右輪慢 → 右轉
        # left=0.75 (PWM 191), right=0.45 (PWM 115)
        return VehicleCommand(0.6, -0.15, self.vacuum_on)

    def trigger_turn_left(self, current_time, yaw: float = None):
        """手動觸發原地左轉 (外部呼叫)"""
        self.state = self.STATE_TURN_LEFT
        self.turn_start_time = current_time
        if self.imu and yaw is not None:
            self.imu.start_turn(-90, yaw)
            print(f"[WallFollower] 手動觸發左轉 (IMU: {yaw:.1f}°)")
        else:
            print("[WallFollower] 手動觸發左轉 (時間)")

    def trigger_find_wall(self, current_time):
        """觸發尋找右牆"""
        self.state = self.STATE_FIND_WALL
        self.find_wall_start_time = current_time
        print("[WallFollower] 觸發尋找右牆")

    def set_vacuum(self, on: bool):
        """設定吸塵器狀態"""
        self.vacuum_on = on

    def get_state(self) -> str:
        """取得當前狀態"""
        return self.state

    def is_at_corner(self) -> bool:
        """是否正在角落處理中"""
        return self.state in [self.STATE_BACKUP, self.STATE_TURN_LEFT]

    def reset(self):
        """重置狀態機"""
        self.state = self.STATE_FORWARD
        self.backup_start_time = None
        self.turn_start_time = None
        self.find_wall_start_time = None


# ==================== 測試程式 ====================
if __name__ == "__main__":
    import time

    print("========== WallFollower v3.0 單元測試 ==========\n")

    follower = WallFollower(use_imu=False)  # 時間模式測試

    # 模擬測試案例
    test_cases = [
        # (front_dist, right_dist, description)
        (100, 15, "正常沿牆 (剛好15cm)"),
        (100, 22, "離牆太遠 -> 應右偏"),
        (100, 10, "離牆太近 -> 應左偏"),
        (35, 15, "前方接近 -> 減速"),
        (14, 20, "角落 (前+右有牆) -> 後退"),
        (14, 999, "只有前牆 -> 原地左轉"),
        (100, 999, "都沒牆 -> 右前方找牆"),
    ]

    current_time = time.time()

    for front, right, desc in test_cases:
        follower.reset()
        cmd = follower.update(front, right, current_time)
        print(f"[Test] {desc}")
        print(f"       Front: {front}cm, Right: {right}cm")
        print(f"       State: {follower.get_state()}")
        print(f"       Cmd: linear={cmd.linear_velocity:+.2f}, angular={cmd.angular_velocity:+.2f}")
        print()

    # 測試角落流程：後退 -> 左轉 -> 直行
    print("\n========== 角落流程測試 ==========\n")
    follower.reset()
    current_time = time.time()

    # 進入角落
    cmd = follower.update(14, 20, current_time)
    print(f"[0.0s] 進入角落 -> State: {follower.get_state()}")

    # 後退中
    for i in range(3):
        current_time += 0.1
        cmd = follower.update(14, 20, current_time)
        print(f"[{(i+1)*0.1:.1f}s] State: {follower.get_state()}, linear={cmd.linear_velocity:+.2f}")

    # 後退完成，進入左轉
    current_time += 0.2
    cmd = follower.update(14, 20, current_time)
    print(f"[0.5s] State: {follower.get_state()}, angular={cmd.angular_velocity:+.2f}")

    # 左轉中
    for i in range(4):
        current_time += 0.2
        cmd = follower.update(50, 20, current_time)
        print(f"[{0.5+(i+1)*0.2:.1f}s] State: {follower.get_state()}")

    print(f"\n最終狀態: {follower.get_state()}")
    print("========== 測試完成 ==========")
