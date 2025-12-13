#!/usr/bin/env python3
"""
test_mpu6050.py - MPU6050 IMU 測試程式
版本: 1.0
日期: 2025-11-29

用於測試 MPU6050 安裝位置與方向，觀察即時數值變化。

功能:
- 即時顯示 Yaw 角度與 Z 軸角速度
- 測試轉彎角度偵測
- 驗證安裝方向是否正確

使用方式:
    python3 test_mpu6050.py           # 基本測試
    python3 test_mpu6050.py --verbose # 顯示原始封包
    python3 test_mpu6050.py --graph   # ASCII 圖形顯示
"""

import argparse
import sys
import time
import config
from arduino_controller import ArduinoController
from differential_drive import MotorCommand


def send_enable_command(arduino):
    """發送啟用超聲波的指令 (馬達停止，但啟用感測器)"""
    cmd = MotorCommand(left_pwm=0, right_pwm=0, vacuum=False, ultrasonic_enable=True)
    arduino.send_command(cmd)


def clear_line():
    """清除當前行"""
    print('\r' + ' ' * 100 + '\r', end='')


def print_header():
    """印出標題"""
    print("=" * 70)
    print(" " * 20 + "MPU6050 IMU 測試程式")
    print(" " * 20 + "安裝方向與數值驗證")
    print("=" * 70)
    print()
    print("測試說明:")
    print("  1. 車體靜止時，Yaw 應該保持穩定 (飄移 < 1°/分鐘)")
    print("  2. 左轉時，Yaw 應該減少 (負方向)")
    print("  3. 右轉時，Yaw 應該增加 (正方向)")
    print("  4. 轉 90° 時，Yaw 變化應該約 90°")
    print()
    print("按 Ctrl+C 結束測試")
    print("-" * 70)
    print()


def draw_yaw_bar(yaw, width=40):
    """繪製 Yaw 角度的 ASCII 圖形"""
    # 將 -180~180 映射到 0~width
    center = width // 2
    pos = int((yaw + 180) / 360 * width)
    pos = max(0, min(width - 1, pos))

    bar = ['-'] * width
    bar[center] = '|'  # 中心標記 (0°)
    bar[pos] = '*'     # 當前位置

    return ''.join(bar)


def draw_gyro_bar(gyro_z, max_val=100, width=40):
    """繪製角速度的 ASCII 圖形"""
    center = width // 2
    # 將 -max_val~max_val 映射到 0~width
    pos = int((gyro_z + max_val) / (2 * max_val) * width)
    pos = max(0, min(width - 1, pos))

    bar = [' '] * width
    bar[center] = '|'  # 中心標記 (0)

    # 繪製條形
    if pos > center:
        for i in range(center + 1, pos + 1):
            bar[i] = '>'
    elif pos < center:
        for i in range(pos, center):
            bar[i] = '<'

    return ''.join(bar)


def test_basic(arduino, verbose=False):
    """基本測試模式"""
    print("模式: 基本數值顯示")
    print("格式: [時間] Yaw: 角度° | GyroZ: 角速度°/s | IMU狀態")
    print()

    start_time = time.time()
    start_yaw = None
    sample_count = 0
    last_cmd_time = 0

    try:
        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            # 每 50ms 發送一次啟用指令 (模擬實際運作)
            if current_time - last_cmd_time >= 0.05:
                send_enable_command(arduino)
                last_cmd_time = current_time

            sensor = arduino.receive_sensor_data()

            if sensor.imu_valid:
                if start_yaw is None:
                    start_yaw = sensor.yaw
                    print(f"初始 Yaw: {start_yaw:.1f}°")
                    print()

                delta_yaw = sensor.yaw - start_yaw if start_yaw else 0
                sample_count += 1

                status = (
                    f"[{elapsed:6.1f}s] "
                    f"Yaw: {sensor.yaw:+7.1f}° "
                    f"(Δ{delta_yaw:+6.1f}°) | "
                    f"GyroZ: {sensor.gyro_z:+4.0f}°/s | "
                    f"IMU: OK"
                )
            else:
                status = (
                    f"[{elapsed:6.1f}s] "
                    f"IMU: 未連接或未初始化"
                )

            if verbose:
                print(status)
            else:
                print(status, end='\r')

            time.sleep(0.02)  # 50Hz (配合指令發送頻率)

    except KeyboardInterrupt:
        print("\n")
        print("-" * 70)
        print(f"測試結束")
        print(f"  總時間: {elapsed:.1f} 秒")
        print(f"  取樣數: {sample_count}")
        if start_yaw is not None:
            final_drift = sensor.yaw - start_yaw
            drift_rate = final_drift / elapsed * 60 if elapsed > 0 else 0
            print(f"  總飄移: {final_drift:.2f}°")
            print(f"  飄移率: {drift_rate:.2f}°/分鐘")
            if abs(drift_rate) < 1:
                print("  評估: ✓ 飄移在可接受範圍內")
            elif abs(drift_rate) < 5:
                print("  評估: △ 飄移略高，建議重新校準")
            else:
                print("  評估: ✗ 飄移過高，請檢查安裝或校準")


def test_graph(arduino):
    """圖形顯示模式"""
    print("模式: ASCII 圖形顯示")
    print()
    print("Yaw:   -180°" + " " * 14 + "0°" + " " * 14 + "+180°")
    print("GyroZ: -100" + " " * 15 + "0" + " " * 15 + "+100 °/s")
    print()

    start_time = time.time()
    start_yaw = None
    last_cmd_time = 0

    try:
        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            # 每 50ms 發送一次啟用指令
            if current_time - last_cmd_time >= 0.05:
                send_enable_command(arduino)
                last_cmd_time = current_time

            sensor = arduino.receive_sensor_data()

            if sensor.imu_valid:
                if start_yaw is None:
                    start_yaw = sensor.yaw

                yaw_bar = draw_yaw_bar(sensor.yaw)
                gyro_bar = draw_gyro_bar(sensor.gyro_z)
                delta = sensor.yaw - start_yaw if start_yaw else 0

                print(f"[{elapsed:5.1f}s] Yaw: [{yaw_bar}] {sensor.yaw:+6.1f}° (Δ{delta:+5.1f}°)", end='\r')
            else:
                print(f"[{elapsed:5.1f}s] IMU: 等待連接...", end='\r')

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n測試結束")


def test_turn_detection(arduino):
    """轉彎偵測測試"""
    print("模式: 轉彎偵測測試")
    print()
    print("請依照指示旋轉車體:")
    print("  1. 按 Enter 開始，保持靜止 3 秒")
    print("  2. 左轉約 90°")
    print("  3. 等待測量結果")
    print()

    input("準備好後按 Enter 開始...")
    print()

    # 階段 1: 靜止校準
    print("階段 1: 靜止校準 (3秒)...")
    start_time = time.time()
    yaw_samples = []
    last_cmd_time = 0

    while time.time() - start_time < 3:
        current_time = time.time()

        # 持續發送啟用指令
        if current_time - last_cmd_time >= 0.05:
            send_enable_command(arduino)
            last_cmd_time = current_time

        sensor = arduino.receive_sensor_data()
        if sensor.imu_valid:
            yaw_samples.append(sensor.yaw)
            print(f"  校準中... Yaw: {sensor.yaw:+.1f}°", end='\r')
        time.sleep(0.02)

    if not yaw_samples:
        print("\n錯誤: 無法讀取 IMU 資料")
        return

    baseline_yaw = sum(yaw_samples) / len(yaw_samples)
    print(f"\n  基準 Yaw: {baseline_yaw:.1f}°")
    print()

    # 階段 2: 等待轉彎
    print("階段 2: 請左轉約 90°...")
    print("  (偵測到轉動後自動記錄)")

    turning = False
    turn_start_yaw = baseline_yaw
    max_delta = 0

    start_time = time.time()
    timeout = 30  # 30 秒超時

    while time.time() - start_time < timeout:
        current_time = time.time()

        # 持續發送啟用指令
        if current_time - last_cmd_time >= 0.05:
            send_enable_command(arduino)
            last_cmd_time = current_time

        sensor = arduino.receive_sensor_data()
        if not sensor.imu_valid:
            time.sleep(0.02)
            continue

        delta = sensor.yaw - baseline_yaw
        # 正規化到 -180~180
        while delta > 180:
            delta -= 360
        while delta < -180:
            delta += 360

        # 偵測開始轉動
        if abs(sensor.gyro_z) > 10 and not turning:
            turning = True
            turn_start_yaw = sensor.yaw
            print(f"  偵測到轉動! 起始: {turn_start_yaw:.1f}°")

        # 記錄最大角度變化
        if abs(delta) > abs(max_delta):
            max_delta = delta

        # 顯示即時狀態
        status = f"  Yaw: {sensor.yaw:+7.1f}° | Δ: {delta:+6.1f}° | GyroZ: {sensor.gyro_z:+4.0f}°/s"
        if turning:
            status += " [轉動中]"
        print(status, end='\r')

        # 偵測停止轉動
        if turning and abs(sensor.gyro_z) < 5:
            # 等待穩定 (繼續發送指令)
            stable_start = time.time()
            while time.time() - stable_start < 0.5:
                if time.time() - last_cmd_time >= 0.05:
                    send_enable_command(arduino)
                    last_cmd_time = time.time()
                time.sleep(0.02)

            sensor = arduino.receive_sensor_data()
            if sensor.imu_valid and abs(sensor.gyro_z) < 5:
                break

        time.sleep(0.02)

    print("\n")
    print("-" * 50)
    print("測試結果:")
    print(f"  基準角度:   {baseline_yaw:+.1f}°")
    print(f"  最終角度:   {sensor.yaw:+.1f}°")
    print(f"  角度變化:   {max_delta:+.1f}°")
    print()

    expected = -90  # 預期左轉 90°
    error = abs(abs(max_delta) - abs(expected))
    print(f"  預期變化:   {expected}°")
    print(f"  誤差:       {error:.1f}°")
    print()

    if max_delta > 0:
        print("  ⚠ 警告: 左轉時 Yaw 增加，請確認安裝方向")
        print("       可能需要將 MPU6050 旋轉 180°")
    elif error < 10:
        print("  ✓ 轉彎偵測正常!")
    elif error < 20:
        print("  △ 誤差略大，建議重新校準或檢查安裝")
    else:
        print("  ✗ 誤差過大，請檢查安裝方向")


def main():
    parser = argparse.ArgumentParser(
        description='MPU6050 IMU 測試程式',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
測試模式:
  基本模式 (預設):  即時顯示 Yaw 和 GyroZ
  --graph:          ASCII 圖形顯示
  --turn:           轉彎偵測測試

安裝方向指南:
  - MPU6050 應水平安裝在車體上
  - 晶片上的軸向標記對應:
    * X 軸: 車體前進方向
    * Y 軸: 車體左方
    * Z 軸: 垂直向上
  - 左轉時 Yaw 應該減少 (負方向)
        """
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='詳細輸出 (每行顯示)'
    )

    parser.add_argument(
        '--graph', '-g',
        action='store_true',
        help='ASCII 圖形顯示模式'
    )

    parser.add_argument(
        '--turn', '-t',
        action='store_true',
        help='轉彎偵測測試模式'
    )

    args = parser.parse_args()

    print_header()

    try:
        print("連接 Arduino...")
        arduino = ArduinoController()
        print("等待 IMU 初始化...")
        time.sleep(1)

        # 發送啟用指令並等待回應
        print("發送啟用指令...")
        for _ in range(20):  # 發送約 1 秒
            send_enable_command(arduino)
            time.sleep(0.05)

        # 檢查 IMU 狀態
        sensor = arduino.receive_sensor_data()
        time.sleep(0.1)
        send_enable_command(arduino)
        time.sleep(0.1)
        sensor = arduino.receive_sensor_data()

        if not sensor.imu_valid:
            print()
            print("⚠ 警告: IMU 資料無效")
            print("  可能原因:")
            print("  1. MPU6050 未正確接線 (檢查 SDA/SCL)")
            print("  2. Arduino 程式未啟用 MPU6050")
            print("  3. I2C 位址錯誤 (預設 0x68)")
            print()
            print("繼續等待 IMU 連接...")
        else:
            print(f"IMU 已連接! 初始 Yaw: {sensor.yaw:.1f}°")
        print()

        # 執行測試
        if args.turn:
            test_turn_detection(arduino)
        elif args.graph:
            test_graph(arduino)
        else:
            test_basic(arduino, verbose=args.verbose)

    except ConnectionError as e:
        print(f"連接錯誤: {e}")
        sys.exit(1)

    except Exception as e:
        print(f"錯誤: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    finally:
        if 'arduino' in dir():
            arduino.close()


if __name__ == "__main__":
    main()
