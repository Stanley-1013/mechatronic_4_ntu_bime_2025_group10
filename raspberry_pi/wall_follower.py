#!/usr/bin/env python3
"""
wall_follower.py - 沿右牆控制器
版本: 1.0
日期: 2025-11-28

實作沿右牆行駛演算法，使用前方+右側超聲波感測器。
參考: FINAL_AUTONOMOUS_PLAN.md - 沿右牆策略

場地規格:
- 紙屑距牆 15cm
- 超聲波在車體右側，吸塵器在中間
- 目標距牆 15cm (超聲波對準紙屑線，吸塵器自然涵蓋)
"""

import config
from differential_drive import VehicleCommand


class WallFollower:
    """沿右牆控制器"""

    # 沿牆參數 (紙屑距牆 15cm，超聲波在車體右側)
    TARGET_RIGHT_DISTANCE = 15      # 目標右側距離 (cm) - 超聲波對準紙屑線
    RIGHT_DISTANCE_TOLERANCE = 5    # 右側距離容許誤差 (±5cm)

    # 前方偵測參數
    FRONT_STOP_DISTANCE = 20        # 前方停止距離 (cm) - 需要轉彎
    FRONT_SLOW_DISTANCE = 40        # 前方減速距離 (cm)

    # 速度參數 (normalized: -1.0 ~ +1.0)
    BASE_LINEAR_SPEED = 0.6         # 基礎線性速度
    SLOW_LINEAR_SPEED = 0.35        # 減速時線性速度
    TURN_ANGULAR_SPEED = 0.7        # 轉彎角速度
    WALL_FOLLOW_KP = 0.025          # 沿牆 P 控制增益

    # 狀態機
    STATE_FORWARD = "forward"       # 直行沿牆
    STATE_TURN_LEFT = "turn_left"   # 左轉（遇前牆）
    STATE_SWEEP = "sweep"           # 角落掃描動作
    STATE_FIND_WALL = "find_wall"   # 尋找右牆

    def __init__(self):
        """初始化沿牆控制器"""
        self.state = self.STATE_FORWARD
        self.vacuum_on = True  # 預設開啟吸塵器

        # 轉彎計時器
        self.turn_start_time = None
        self.turn_duration = 0.7  # 轉彎持續時間 (秒) - 需實測調校

        # 角落掃描
        self.sweep_start_time = None
        self.sweep_phase = 0
        self.sweep_duration = 5.0  # 角落掃描時間 (秒) - 縮短以節省比賽時間

        # 找牆計時
        self.find_wall_start_time = None
        self.find_wall_timeout = 3.0  # 找牆超時 (秒)

        print("WallFollower 初始化完成")
        print(f"   目標右側距離: {self.TARGET_RIGHT_DISTANCE} cm")
        print(f"   前方轉彎距離: {self.FRONT_STOP_DISTANCE} cm")

    def update(self, front_distance: int, right_distance: int,
               current_time: float) -> VehicleCommand:
        """
        根據感測器資料更新控制指令

        Args:
            front_distance: 前方距離 (cm), 999=無效
            right_distance: 右側距離 (cm), 999=無效
            current_time: 當前時間 (time.time())

        Returns:
            VehicleCommand: 車輛控制指令
        """
        # 處理無效感測器資料
        front_valid = front_distance != 999 and front_distance > 0
        right_valid = right_distance != 999 and right_distance > 0

        # 狀態機邏輯
        if self.state == self.STATE_FORWARD:
            return self._state_forward(front_distance, right_distance,
                                       front_valid, right_valid, current_time)

        elif self.state == self.STATE_TURN_LEFT:
            return self._state_turn_left(front_distance, front_valid,
                                         current_time)

        elif self.state == self.STATE_SWEEP:
            return self._state_sweep(current_time)

        elif self.state == self.STATE_FIND_WALL:
            return self._state_find_wall(right_distance, right_valid,
                                         current_time)

        # 預設停止
        return VehicleCommand(0, 0, self.vacuum_on)

    def _state_forward(self, front_dist, right_dist,
                       front_valid, right_valid, current_time):
        """
        直行沿牆狀態

        邏輯:
        1. 前方 < 20cm -> 左轉
        2. 前方 < 40cm -> 減速
        3. 右側距離修正 (P控制)
        4. 右側無效/太遠 -> 右偏找牆
        """
        linear = self.BASE_LINEAR_SPEED
        angular = 0.0

        # 1. 檢查前方是否需要轉彎
        if front_valid and front_dist < self.FRONT_STOP_DISTANCE:
            # 進入左轉狀態
            self.state = self.STATE_TURN_LEFT
            self.turn_start_time = current_time
            print(f"[WallFollower] 前方 {front_dist}cm < {self.FRONT_STOP_DISTANCE}cm -> 左轉")
            return VehicleCommand(0, -self.TURN_ANGULAR_SPEED, self.vacuum_on)

        # 2. 前方減速
        if front_valid and front_dist < self.FRONT_SLOW_DISTANCE:
            linear = self.SLOW_LINEAR_SPEED

        # 3. 沿右牆控制（P 控制器）
        if right_valid and right_dist < 80:  # 只有右側有效且合理時才修正
            error = right_dist - self.TARGET_RIGHT_DISTANCE
            # error > 0: 離牆太遠，需要右轉（angular > 0）
            # error < 0: 離牆太近，需要左轉（angular < 0）
            angular = error * self.WALL_FOLLOW_KP

            # 限制角速度 (避免過度修正)
            angular = max(-0.35, min(0.35, angular))

        # 4. 如果右側沒有牆（距離太遠或無效），右偏尋找牆壁
        elif not right_valid or right_dist > 80:
            angular = 0.25  # 稍微右轉
            linear = self.SLOW_LINEAR_SPEED  # 減速尋牆

        return VehicleCommand(linear, angular, self.vacuum_on)

    def _state_turn_left(self, front_dist, front_valid, current_time):
        """
        左轉狀態（原地左轉避開前牆）

        結束條件:
        1. 轉彎時間到
        2. 前方已清空 (且已轉一小段時間)
        """
        elapsed = current_time - self.turn_start_time

        # 轉彎完成條件：時間到
        if elapsed > self.turn_duration:
            self.state = self.STATE_FORWARD
            print(f"[WallFollower] 左轉完成 ({elapsed:.2f}s) -> 直行")
            return VehicleCommand(self.BASE_LINEAR_SPEED, 0, self.vacuum_on)

        # 如果前方已經清空且已轉一小段時間，提早結束
        if front_valid and front_dist > self.FRONT_SLOW_DISTANCE and elapsed > 0.3:
            self.state = self.STATE_FORWARD
            print(f"[WallFollower] 前方清空 ({front_dist}cm) -> 直行")
            return VehicleCommand(self.BASE_LINEAR_SPEED, 0, self.vacuum_on)

        # 繼續左轉（原地轉：linear=0）
        return VehicleCommand(0, -self.TURN_ANGULAR_SPEED, self.vacuum_on)

    def _state_sweep(self, current_time):
        """
        角落掃描狀態 - 簡化版本 (約5秒)

        動作序列:
        Phase 0: 小幅前進靠近角落 (1.5s)
        Phase 1: 左右擺動清掃 (2.5s)
        Phase 2: 左轉準備離開 (1s)
        """
        elapsed = current_time - self.sweep_start_time

        if elapsed > self.sweep_duration:
            self.state = self.STATE_FORWARD
            self.sweep_phase = 0
            print("[WallFollower] 角落掃描完成")
            return VehicleCommand(0.4, 0, self.vacuum_on)

        # 動作序列
        if elapsed < 1.5:
            # Phase 0: 小幅前進靠近角落
            return VehicleCommand(0.3, 0, self.vacuum_on)
        elif elapsed < 4.0:
            # Phase 1: 左右擺動清掃
            import math
            swing = 0.4 * math.sin((elapsed - 1.5) * 3.0)
            return VehicleCommand(0.15, swing, self.vacuum_on)
        else:
            # Phase 2: 左轉準備離開
            return VehicleCommand(0, -0.5, self.vacuum_on)

    def _state_find_wall(self, right_dist, right_valid, current_time):
        """
        尋找右牆狀態 (紅圓迴避後使用)

        策略: 右前方前進直到找到右牆
        """
        elapsed = current_time - self.find_wall_start_time

        # 超時保護
        if elapsed > self.find_wall_timeout:
            self.state = self.STATE_FORWARD
            print("[WallFollower] 找牆超時 -> 直行")
            return VehicleCommand(self.BASE_LINEAR_SPEED, 0, self.vacuum_on)

        # 如果右側偵測到合理距離的牆
        if right_valid and 10 < right_dist < 50:
            self.state = self.STATE_FORWARD
            print(f"[WallFollower] 找到右牆 ({right_dist}cm) -> 直行")
            return VehicleCommand(self.BASE_LINEAR_SPEED, 0, self.vacuum_on)

        # 右前方前進 (稍微右偏)
        return VehicleCommand(0.5, 0.2, self.vacuum_on)

    def trigger_turn_left(self, current_time):
        """手動觸發左轉 (外部呼叫)"""
        self.state = self.STATE_TURN_LEFT
        self.turn_start_time = current_time
        print("[WallFollower] 手動觸發左轉")

    def trigger_sweep(self, current_time):
        """觸發角落掃描（外部呼叫）"""
        self.state = self.STATE_SWEEP
        self.sweep_start_time = current_time
        self.sweep_phase = 0
        print("[WallFollower] 觸發角落掃描")

    def trigger_find_wall(self, current_time):
        """觸發尋找右牆 (紅圓迴避後)"""
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
        """是否正在角落（轉彎或掃描中）"""
        return self.state in [self.STATE_TURN_LEFT, self.STATE_SWEEP]

    def reset(self):
        """重置狀態機"""
        self.state = self.STATE_FORWARD
        self.turn_start_time = None
        self.sweep_start_time = None
        self.find_wall_start_time = None
        self.sweep_phase = 0


# ==================== 測試程式 ====================
if __name__ == "__main__":
    import time

    print("========== WallFollower 單元測試 ==========\n")

    follower = WallFollower()

    # 模擬測試案例
    test_cases = [
        # (front_dist, right_dist, description)
        (100, 15, "正常沿牆 (剛好15cm)"),
        (100, 22, "離牆太遠 (22cm) -> 應右偏"),
        (100, 10, "離牆太近 (10cm) -> 應左偏"),
        (35, 15, "前方接近 -> 減速"),
        (15, 15, "前方障礙 -> 轉彎"),
        (100, 999, "右側無效 -> 尋牆"),
        (100, 120, "右側太遠 -> 尋牆"),
    ]

    current_time = time.time()

    for front, right, desc in test_cases:
        cmd = follower.update(front, right, current_time)
        print(f"[Test] {desc}")
        print(f"       Front: {front}cm, Right: {right}cm")
        print(f"       State: {follower.get_state()}")
        print(f"       Cmd: linear={cmd.linear_velocity:+.2f}, angular={cmd.angular_velocity:+.2f}")
        print()

        # 如果進入轉彎狀態，模擬轉彎完成
        if follower.get_state() == WallFollower.STATE_TURN_LEFT:
            current_time += 0.8
            cmd = follower.update(100, 15, current_time)
            print(f"       [After Turn] State: {follower.get_state()}")
            print()

        current_time += 0.1

    # 測試角落掃描
    print("\n========== 角落掃描測試 ==========\n")
    follower.reset()
    follower.trigger_sweep(current_time)
    print(f"[Sweep Test] 觸發角落掃描")
    print(f"             State: {follower.get_state()}")

    for i in range(6):  # 模擬 6 秒
        cmd = follower.update(100, 15, current_time)
        print(f"[{i}s] State: {follower.get_state()}, "
              f"linear={cmd.linear_velocity:+.2f}, angular={cmd.angular_velocity:+.2f}")
        current_time += 1.0

    print(f"\n最終狀態: {follower.get_state()}")
    print("========== 測試完成 ==========")
