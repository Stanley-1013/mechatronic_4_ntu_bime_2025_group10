#!/usr/bin/env python3
"""
test_motor_only.py - 獨立測試馬達控制
用途：不需要遙控器，直接測試馬達是否正常運作
版本：1.0
"""

import sys
import time
from arduino_controller import ArduinoController, MotorCommand

def test_motors():
    """測試馬達控制序列"""
    print("=" * 60)
    print("  馬達獨立測試程式")
    print("=" * 60)
    print()

    try:
        # 初始化 Arduino 控制器
        print("[1/1] 連接 Arduino...")
        controller = ArduinoController()
        print()

        # 測試序列
        test_sequence = [
            ("前進全速", MotorCommand(255, 255, False), 2.0),
            ("停止", MotorCommand(0, 0, False), 1.0),
            ("後退全速", MotorCommand(-255, -255, False), 2.0),
            ("停止", MotorCommand(0, 0, False), 1.0),
            ("原地左轉", MotorCommand(-200, 200, False), 1.5),
            ("停止", MotorCommand(0, 0, False), 1.0),
            ("原地右轉", MotorCommand(200, -200, False), 1.5),
            ("停止", MotorCommand(0, 0, False), 1.0),
            ("前進半速", MotorCommand(127, 127, False), 2.0),
            ("停止", MotorCommand(0, 0, False), 1.0),
            ("測試吸塵器", MotorCommand(0, 0, True), 2.0),
            ("關閉吸塵器", MotorCommand(0, 0, False), 1.0),
        ]

        print("開始測試序列...\n")

        for i, (name, cmd, duration) in enumerate(test_sequence, 1):
            print(f"[測試 {i}/{len(test_sequence)}] {name}")
            print(f"  指令: {cmd}")

            controller.send_command(cmd)

            # 等待指定時間，同時接收感測器資料
            start = time.time()
            while time.time() - start < duration:
                sensor_data = controller.receive_sensor_data()
                elapsed = time.time() - start

                # 顯示即時狀態
                print(f"  時間: {elapsed:.1f}s | 感測器: {sensor_data}", end='\r')
                time.sleep(0.1)

            print()  # 換行
            print()

        # 最後確保停止
        print("[完成] 停止所有馬達...")
        controller.send_command(MotorCommand(0, 0, False))
        time.sleep(0.5)

        # 顯示統計
        print("\n" + "=" * 60)
        print("  通訊統計")
        print("=" * 60)
        stats = controller.get_stats()
        print(f"發送封包: {stats['tx_packets']}")
        print(f"接收封包: {stats['rx_packets']}")
        print(f"發送錯誤: {stats['tx_errors']}")
        print(f"接收錯誤: {stats['rx_errors']}")
        print(f"Checksum 錯誤: {stats['checksum_errors']}")

        if stats['tx_packets'] > 0:
            success_rate = (1 - stats['checksum_errors'] / stats['tx_packets']) * 100
            print(f"成功率: {success_rate:.1f}%")

        print("\n✅ 測試完成")
        controller.close()

    except KeyboardInterrupt:
        print("\n\n⚠️  測試中斷")
        # 停止馬達
        controller.send_command(MotorCommand(0, 0, False))
        controller.close()

    except Exception as e:
        print(f"\n❌ 錯誤: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    test_motors()
