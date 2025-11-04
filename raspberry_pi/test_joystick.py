#!/usr/bin/env python3
"""
test_joystick.py - 測試遙控器軸編號與按鈕
用途：找出你的遙控器的正確軸編號和按鈕編號
版本：1.0
"""

import pygame
import time
import sys

def test_joystick():
    """測試遙控器輸入"""
    print("=" * 60)
    print("  遙控器測試程式")
    print("  請移動搖桿和按按鈕來查看編號")
    print("  按 Ctrl+C 結束")
    print("=" * 60)
    print()

    pygame.init()
    pygame.joystick.init()

    # 等待遙控器連接
    while pygame.joystick.get_count() == 0:
        print("⏳ 等待遙控器連接...")
        time.sleep(1)
        pygame.joystick.quit()
        pygame.joystick.init()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"✅ 遙控器已連接: {joystick.get_name()}")
    print(f"   軸數量: {joystick.get_numaxes()}")
    print(f"   按鈕數量: {joystick.get_numbuttons()}")
    print(f"   Hat 數量: {joystick.get_numhats()}")
    print()
    print("開始測試...\n")

    try:
        while True:
            pygame.event.pump()

            # 清除螢幕
            print("\033[2J\033[H", end='')
            print("=" * 60)
            print("  移動搖桿或按按鈕來查看數值")
            print("=" * 60)
            print()

            # 顯示所有軸
            print("【搖桿軸】")
            for i in range(joystick.get_numaxes()):
                value = joystick.get_axis(i)
                bar = "#" * int(abs(value) * 20)
                direction = "→" if value > 0 else "←"
                if abs(value) > 0.1:  # 只顯示有明顯變化的軸
                    print(f"  Axis {i}: {value:+.2f} {direction} {bar}")

            print()

            # 顯示所有按鈕
            print("【按鈕】")
            pressed_buttons = []
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    pressed_buttons.append(i)

            if pressed_buttons:
                print(f"  已按下: {', '.join(f'Button {i}' for i in pressed_buttons)}")
            else:
                print("  (無按鈕按下)")

            print()

            # 顯示 Hat (方向鍵)
            if joystick.get_numhats() > 0:
                print("【方向鍵 (Hat)】")
                for i in range(joystick.get_numhats()):
                    hat = joystick.get_hat(i)
                    if hat != (0, 0):
                        print(f"  Hat {i}: {hat}")
                print()

            # 提示
            print("─" * 60)
            print("💡 建議設定（記錄下來）：")
            print("   JOYSTICK_AXIS_LINEAR  = [移動前後搖桿的軸編號]")
            print("   JOYSTICK_AXIS_ANGULAR = [移動左右搖桿的軸編號]")
            print("   JOYSTICK_BUTTON_VACUUM = [吸塵器按鈕編號]")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n✅ 測試結束")
        pygame.quit()


if __name__ == "__main__":
    test_joystick()
