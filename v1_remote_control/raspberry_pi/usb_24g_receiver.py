#!/usr/bin/env python3
"""
usb_24g_receiver.py - 2.4G USB 遙控器接收器
版本: 1.0
日期: 2025-10-31

使用 pygame 讀取 USB 遙控器輸入，並轉換為統一的 VehicleCommand 格式。
參考: REMOTE_CONTROL_ARCHITECTURE.md - 方案 1
"""

import pygame
import time
import config
from differential_drive import VehicleCommand


class USB24GReceiver:
    """2.4G USB 遙控器接收器"""

    def __init__(self, joystick_id=None, deadzone=None):
        """
        初始化 USB 遙控器接收器

        Args:
            joystick_id: pygame joystick 編號（預設從 config 讀取）
            deadzone: 搖桿死區範圍（預設從 config 讀取）
        """
        self.joystick_id = joystick_id if joystick_id is not None else config.JOYSTICK_DEVICE_ID
        self.deadzone = deadzone if deadzone is not None else config.JOYSTICK_DEADZONE

        # 吸塵器狀態（toggle 模式）
        self.vacuum_state = False
        self.vacuum_button_pressed = False

        # 初始化 pygame
        pygame.init()
        pygame.joystick.init()

        # 等待遙控器連接
        self._wait_for_joystick()

        # 初始化遙控器
        self.joystick = pygame.joystick.Joystick(self.joystick_id)
        self.joystick.init()

        print(f"✅ 遙控器已連接: {self.joystick.get_name()}")
        print(f"   搖桿軸數: {self.joystick.get_numaxes()}")
        print(f"   按鈕數: {self.joystick.get_numbuttons()}")
        print(f"   死區: {self.deadzone}")

    def _wait_for_joystick(self, timeout=30):
        """
        等待遙控器連接

        Args:
            timeout: 等待時間（秒）
        """
        start_time = time.time()

        while pygame.joystick.get_count() == 0:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                raise TimeoutError(f"等待 {timeout} 秒未偵測到遙控器，請檢查 USB 連接")

            print(f"⏳ 等待遙控器連接... ({int(elapsed)}s)")
            time.sleep(1)
            pygame.joystick.quit()
            pygame.joystick.init()

    def receive(self) -> VehicleCommand:
        """
        接收遙控器輸入並轉換為 VehicleCommand

        Returns:
            VehicleCommand 物件

        參考: 01_SRS_軟體需求規格書.md - FR1
        """
        # 更新 pygame 事件
        pygame.event.pump()

        # 讀取搖桿軸
        linear_raw = self.joystick.get_axis(config.JOYSTICK_AXIS_LINEAR)
        angular_raw = self.joystick.get_axis(config.JOYSTICK_AXIS_ANGULAR)

        # 軸反轉（若需要）
        if config.JOYSTICK_AXIS_INVERT_LINEAR:
            linear_raw = -linear_raw
        if config.JOYSTICK_AXIS_INVERT_ANGULAR:
            angular_raw = -angular_raw

        # 應用死區過濾
        linear_velocity = self._apply_deadzone(linear_raw)
        angular_velocity = self._apply_deadzone(angular_raw)

        # 讀取吸塵器按鈕（toggle 模式）
        button_state = self.joystick.get_button(config.JOYSTICK_BUTTON_VACUUM)
        if button_state and not self.vacuum_button_pressed:
            # 按鈕剛按下（上升沿）
            self.vacuum_state = not self.vacuum_state
            self.vacuum_button_pressed = True
        elif not button_state:
            # 按鈕放開
            self.vacuum_button_pressed = False

        # 建立控制指令
        cmd = VehicleCommand(
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            vacuum_motor=self.vacuum_state
        )

        return cmd

    def _apply_deadzone(self, value):
        """
        應用死區過濾

        Args:
            value: 原始軸值 (-1.0 ~ +1.0)

        Returns:
            過濾後的值

        參考: 01_SRS_軟體需求規格書.md - FR1.4
        """
        if abs(value) < self.deadzone:
            return 0.0
        else:
            # 線性重新映射（死區外的值重新分配到 0.0-1.0）
            sign = 1 if value > 0 else -1
            magnitude = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
            return sign * magnitude

    def get_joystick_info(self):
        """
        取得遙控器詳細資訊

        Returns:
            dict: 遙控器資訊
        """
        return {
            'name': self.joystick.get_name(),
            'num_axes': self.joystick.get_numaxes(),
            'num_buttons': self.joystick.get_numbuttons(),
            'num_hats': self.joystick.get_numhats(),
        }

    def close(self):
        """關閉遙控器連接"""
        self.joystick.quit()
        pygame.quit()
        print("✅ 遙控器連接已關閉")


# ==================== 測試程式 ====================
if __name__ == "__main__":
    print("========== USB 2.4G 遙控器測試 ==========\n")
    print("請操作搖桿和按鈕，按 Ctrl+C 退出\n")

    try:
        receiver = USB24GReceiver()

        while True:
            cmd = receiver.receive()

            # 顯示即時狀態
            print(f"Linear: {cmd.linear_velocity:+.2f}  "
                  f"Angular: {cmd.angular_velocity:+.2f}  "
                  f"Vacuum: {'ON ' if cmd.vacuum_motor else 'OFF'}",
                  end='\r')

            time.sleep(0.02)  # 50Hz

    except KeyboardInterrupt:
        print("\n\n✋ 測試中止")
        receiver.close()

    except TimeoutError as e:
        print(f"\n❌ 錯誤: {e}")

    except Exception as e:
        print(f"\n❌ 發生錯誤: {e}")
        import traceback
        traceback.print_exc()
