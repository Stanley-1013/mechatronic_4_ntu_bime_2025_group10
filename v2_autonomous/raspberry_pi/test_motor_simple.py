#!/usr/bin/env python3
"""
最簡單的馬達測試 - 直接發送 PWM

用法: python3 test_motor_simple.py [串口]

測試流程:
  1. 左輪前進 2 秒 (車子右轉)
  2. 右輪前進 2 秒 (車子左轉)
  3. 雙輪前進 3 秒 (直走)
  4. 停止

!! 需要先重新上傳 Arduino 程式 (加入 CMD_TEST_MOTOR 指令) !!
"""

import serial
import time
import sys

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

# 協定常數
HEADER = 0xAA
FOOTER = 0x55
CMD_TEST_MOTOR = 0xFE

def build_test_motor_cmd(left_pwm: int, right_pwm: int) -> bytes:
    """
    建構 TEST_MOTOR 指令
    payload: [left_h, left_l, right_h, right_l] (signed int16, big-endian)
    """
    # 轉為 signed int16
    if left_pwm < 0:
        left_pwm = left_pwm & 0xFFFF
    if right_pwm < 0:
        right_pwm = right_pwm & 0xFFFF

    payload = bytes([
        (left_pwm >> 8) & 0xFF,
        left_pwm & 0xFF,
        (right_pwm >> 8) & 0xFF,
        right_pwm & 0xFF,
    ])

    payload_len = len(payload)

    # checksum = CMD ^ LEN ^ payload bytes
    checksum = CMD_TEST_MOTOR ^ payload_len
    for b in payload:
        checksum ^= b

    return bytes([HEADER, CMD_TEST_MOTOR, payload_len]) + payload + bytes([checksum, FOOTER])

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT

    print("=" * 40)
    print(" 馬達方向測試")
    print("=" * 40)
    print(f"串口: {port}")
    print()
    print("!! 需要先上傳新的 Arduino 程式 !!")
    print()
    print("測試流程:")
    print("  1. 左輪 PWM=80 (2秒) → 車子應右轉")
    print("  2. 右輪 PWM=80 (2秒) → 車子應左轉")
    print("  3. 雙輪 PWM=80 (3秒) → 車子應直走")
    print()
    input("按 Enter 開始...")

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=1)
        print(f"[OK] 已連接 {port}")
        time.sleep(2)

        # 讀取啟動訊息
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[Arduino] {line}")

        # === 測試 1: 左輪 ===
        print()
        print("-" * 40)
        print("測試 1: 左輪前進 (車子應右轉)")
        print("-" * 40)

        cmd = build_test_motor_cmd(80, 0)
        print(f"[TX] {cmd.hex()}")

        for _ in range(40):  # 2 秒
            ser.write(cmd)
            time.sleep(0.05)
            # 讀取回應
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line and not line.startswith('\xbb'):
                    print(f"[Arduino] {line}")

        # 停止
        ser.write(build_test_motor_cmd(0, 0))
        time.sleep(1)

        # === 測試 2: 右輪 ===
        print()
        print("-" * 40)
        print("測試 2: 右輪前進 (車子應左轉)")
        print("-" * 40)

        cmd = build_test_motor_cmd(0, 80)
        print(f"[TX] {cmd.hex()}")

        for _ in range(40):
            ser.write(cmd)
            time.sleep(0.05)
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line and not line.startswith('\xbb'):
                    print(f"[Arduino] {line}")

        ser.write(build_test_motor_cmd(0, 0))
        time.sleep(1)

        # === 測試 3: 雙輪 ===
        print()
        print("-" * 40)
        print("測試 3: 雙輪前進 (直走)")
        print("-" * 40)

        cmd = build_test_motor_cmd(80, 80)
        print(f"[TX] {cmd.hex()}")

        for _ in range(60):  # 3 秒
            ser.write(cmd)
            time.sleep(0.05)
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line and not line.startswith('\xbb'):
                    print(f"[Arduino] {line}")

        # 停止
        ser.write(build_test_motor_cmd(0, 0))

        print()
        print("=" * 40)
        print(" 測試完成")
        print("=" * 40)
        print()
        print("預期結果:")
        print("  測試 1: 左輪轉 → 車子右轉")
        print("  測試 2: 右輪轉 → 車子左轉")
        print("  測試 3: 雙輪轉 → 直走")
        print()
        print("如果方向相反:")
        print("  交換 config.h 的 PIN_IN1/PIN_IN2 或 PIN_IN3/PIN_IN4")

        ser.close()

    except serial.SerialException as e:
        print(f"[ERROR] {e}")
    except KeyboardInterrupt:
        print("\n[中斷]")
        if 'ser' in locals():
            ser.write(build_test_motor_cmd(0, 0))
            ser.close()

if __name__ == "__main__":
    main()
