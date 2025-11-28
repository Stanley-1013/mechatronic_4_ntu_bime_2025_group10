#!/usr/bin/env python3
"""
test_ultrasonic.py - 超聲波感測器測試程式
用於驗證 Arduino 超聲波讀值是否正常回傳

使用方式:
  python3 test_ultrasonic.py
  按 Ctrl+C 結束
"""

import time
from differential_drive import DifferentialDrive, VehicleCommand
from arduino_controller import ArduinoController


def main():
    print("=" * 50)
    print("  超聲波感測器測試")
    print("  Ultrasonic Sensor Test")
    print("=" * 50)
    print()

    # 初始化 (啟用超聲波)
    print("[1/2] 初始化差動驅動 (ultrasonic_enable=True)...")
    drive = DifferentialDrive(ultrasonic_enable=True)

    print("[2/2] 初始化 Arduino 通訊...")
    arduino = ArduinoController()

    print("\n開始讀取感測器資料...")
    print("按 Ctrl+C 結束\n")
    print("-" * 50)

    try:
        count = 0
        while True:
            # 發送停止指令 (但啟用超聲波)
            vehicle_cmd = VehicleCommand(0, 0, False)
            motor_cmd = drive.convert(vehicle_cmd)
            arduino.send_command(motor_cmd)

            # 短暫等待讓 Arduino 回傳
            time.sleep(0.1)

            # 讀取感測器
            sensor = arduino.receive_sensor_data()

            count += 1
            front_status = "OK" if sensor.front_valid else "無效"
            right_status = "OK" if sensor.right_valid else "無效"

            print(f"[{count:4d}] 前方: {sensor.front_distance:3d} cm ({front_status})  |  "
                  f"右側: {sensor.right_distance:3d} cm ({right_status})")

            time.sleep(0.1)  # 約 5Hz 顯示

    except KeyboardInterrupt:
        print("\n\n停止測試...")

    finally:
        # 停止馬達並關閉超聲波
        stop_cmd = VehicleCommand(0, 0, False)
        motor_cmd = drive.convert(stop_cmd)
        motor_cmd.ultrasonic_enable = False
        arduino.send_command(motor_cmd)

        arduino.close()
        print("測試結束")


if __name__ == "__main__":
    main()
