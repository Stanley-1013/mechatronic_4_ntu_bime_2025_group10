#!/usr/bin/env python3
"""
test_vacuum.py - 吸塵器馬達測試程式
用於驗證吸塵器繼電器控制是否正常運作

使用方式:
  python3 test_vacuum.py           # 互動式測試
  python3 test_vacuum.py --on      # 直接開啟 (5秒後自動關閉)
  python3 test_vacuum.py --off     # 直接關閉
  python3 test_vacuum.py --toggle  # 開關切換測試
  按 Ctrl+C 結束
"""

import argparse
import time
from differential_drive import DifferentialDrive, MotorCommand
from arduino_controller import ArduinoController


def test_interactive(arduino: ArduinoController, drive: DifferentialDrive):
    """互動式測試"""
    print("\n" + "=" * 50)
    print("  吸塵器互動測試")
    print("=" * 50)
    print("\n指令:")
    print("  1 = 開啟吸塵器")
    print("  0 = 關閉吸塵器")
    print("  t = 切換 (Toggle)")
    print("  q = 離開")
    print("\n" + "-" * 50)

    vacuum_state = False

    try:
        while True:
            status = "ON" if vacuum_state else "OFF"
            user_input = input(f"\n吸塵器狀態: [{status}]  輸入指令: ").strip().lower()

            if user_input == 'q':
                break
            elif user_input == '1':
                vacuum_state = True
                print("  → 開啟吸塵器")
            elif user_input == '0':
                vacuum_state = False
                print("  → 關閉吸塵器")
            elif user_input == 't':
                vacuum_state = not vacuum_state
                print(f"  → 切換為: {'ON' if vacuum_state else 'OFF'}")
            else:
                print("  無效指令，請輸入 1/0/t/q")
                continue

            # 發送指令 (馬達停止，只控制吸塵器)
            motor_cmd = MotorCommand(
                left_pwm=0,
                right_pwm=0,
                vacuum=vacuum_state,
                ultrasonic_enable=False
            )
            arduino.send_command(motor_cmd)

    except KeyboardInterrupt:
        print("\n\n中斷測試")


def test_on(arduino: ArduinoController, duration: float = 5.0):
    """開啟吸塵器測試"""
    print(f"\n開啟吸塵器 {duration} 秒...")

    motor_cmd = MotorCommand(
        left_pwm=0,
        right_pwm=0,
        vacuum=True,
        ultrasonic_enable=False
    )
    arduino.send_command(motor_cmd)

    print(f"吸塵器已開啟，等待 {duration} 秒...")

    try:
        time.sleep(duration)
    except KeyboardInterrupt:
        print("\n提前中斷")

    # 關閉
    motor_cmd.vacuum = False
    arduino.send_command(motor_cmd)
    print("吸塵器已關閉")


def test_off(arduino: ArduinoController):
    """關閉吸塵器"""
    print("\n關閉吸塵器...")

    motor_cmd = MotorCommand(
        left_pwm=0,
        right_pwm=0,
        vacuum=False,
        ultrasonic_enable=False
    )
    arduino.send_command(motor_cmd)
    print("吸塵器已關閉")


def test_toggle(arduino: ArduinoController, cycles: int = 3, interval: float = 2.0):
    """開關切換測試"""
    print(f"\n開關切換測試 ({cycles} 次，間隔 {interval} 秒)")
    print("-" * 50)

    try:
        for i in range(cycles):
            # 開啟
            print(f"[{i+1}/{cycles}] 開啟...", end='', flush=True)
            motor_cmd = MotorCommand(0, 0, vacuum=True, ultrasonic_enable=False)
            arduino.send_command(motor_cmd)
            time.sleep(interval)

            # 關閉
            print(" 關閉...", end='', flush=True)
            motor_cmd.vacuum = False
            arduino.send_command(motor_cmd)
            time.sleep(interval)

            print(" OK")

        print("\n切換測試完成")

    except KeyboardInterrupt:
        print("\n\n提前中斷")
        motor_cmd = MotorCommand(0, 0, vacuum=False, ultrasonic_enable=False)
        arduino.send_command(motor_cmd)


def main():
    parser = argparse.ArgumentParser(
        description='吸塵器馬達測試程式',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
範例:
  python3 test_vacuum.py           # 互動式測試
  python3 test_vacuum.py --on      # 開啟 5 秒
  python3 test_vacuum.py --on 10   # 開啟 10 秒
  python3 test_vacuum.py --off     # 關閉
  python3 test_vacuum.py --toggle  # 開關切換測試
        """
    )

    parser.add_argument(
        '--on',
        nargs='?',
        const=5.0,
        type=float,
        metavar='SECONDS',
        help='開啟吸塵器 (預設 5 秒後自動關閉)'
    )

    parser.add_argument(
        '--off',
        action='store_true',
        help='關閉吸塵器'
    )

    parser.add_argument(
        '--toggle',
        nargs='?',
        const=3,
        type=int,
        metavar='CYCLES',
        help='開關切換測試 (預設 3 次)'
    )

    args = parser.parse_args()

    print("=" * 50)
    print("  吸塵器馬達測試")
    print("  Vacuum Motor Test")
    print("=" * 50)
    print()

    print("[1/2] 初始化 Arduino 通訊...")
    arduino = ArduinoController()

    print("[2/2] 初始化差動驅動...")
    drive = DifferentialDrive(ultrasonic_enable=False)

    try:
        if args.on is not None:
            test_on(arduino, args.on)
        elif args.off:
            test_off(arduino)
        elif args.toggle is not None:
            test_toggle(arduino, args.toggle)
        else:
            test_interactive(arduino, drive)

    except KeyboardInterrupt:
        print("\n\n使用者中止")

    finally:
        # 確保吸塵器關閉
        print("\n確保吸塵器關閉...")
        motor_cmd = MotorCommand(0, 0, vacuum=False, ultrasonic_enable=False)
        arduino.send_command(motor_cmd)
        arduino.close()
        print("測試結束")


if __name__ == "__main__":
    main()
