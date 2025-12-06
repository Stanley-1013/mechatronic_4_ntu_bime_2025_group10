#!/usr/bin/env python3
"""
直走測試程式 (v2 協定)

發送 START 指令讓車子直走，測試馬達方向是否正確。

用法: python3 test_straight.py
"""

import serial
import time
import sys

# 串口設定
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

# 協定常數 (來自 protocol.py)
HEADER = 0xAA
CMD_START = 0x01
CMD_STOP = 0x02
CMD_SET_VACUUM = 0x04

def build_command(cmd: int, payload: bytes = b'') -> bytes:
    """
    建構指令封包
    格式: [HEADER=0xAA][CMD][PAYLOAD_LEN][PAYLOAD][CHECKSUM][FOOTER=0x55]
    CHECKSUM = CMD ^ PAYLOAD_LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n]
    """
    length = len(payload)

    # 計算 checksum: CMD ^ PAYLOAD_LEN ^ PAYLOAD bytes
    checksum = cmd ^ length
    for b in payload:
        checksum ^= b

    # 組合封包: HEADER + CMD + LEN + PAYLOAD + CHECKSUM + FOOTER
    packet = bytes([HEADER, cmd, length]) + payload + bytes([checksum, 0x55])
    return packet

def main():
    print("=" * 50)
    print(" v2 直走測試")
    print("=" * 50)
    print()
    print("此測試會：")
    print("1. 發送 START 指令")
    print("2. 車子會進入沿牆模式並前進")
    print("3. 5 秒後自動停止")
    print()
    print("觀察車子是否直走（或輕微右偏尋牆）")
    print("如果自轉，表示馬達方向或感測器有問題")
    print()

    # 檢查串口
    try:
        port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    except:
        port = SERIAL_PORT

    print(f"使用串口: {port}")
    input("按 Enter 開始測試（確保車子輪子懸空或有足夠空間）...")

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=1)
        print(f"[OK] 已連接 {port}")
        time.sleep(2)  # 等待 Arduino 初始化和 IMU 校準

        # 清空緩衝區並讀取啟動訊息
        ser.reset_input_buffer()

        print()
        print("[Arduino 啟動訊息]")
        deadline = time.time() + 3
        while time.time() < deadline:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"  {line}")
                except:
                    pass
            else:
                time.sleep(0.1)

        print()
        print("-" * 50)
        print("發送 START 指令...")
        print("-" * 50)

        # 發送 START
        packet = build_command(CMD_START)
        ser.write(packet)
        print(f"[TX] START: {' '.join(f'{b:02X}' for b in packet)}")

        # 持續運行 5 秒，同時讀取狀態
        print()
        print("車子運行中... (5 秒)")
        print()

        start_time = time.time()
        last_status_time = 0

        while time.time() - start_time < 5.0:
            # 讀取 Arduino 回傳的狀態
            if ser.in_waiting >= 12:  # 狀態封包 12 bytes
                data = ser.read(ser.in_waiting)
                # 嘗試解析狀態封包
                for i in range(len(data) - 11):
                    if data[i] == 0xBB:  # STATE_HEADER
                        pkt = data[i:i+12]
                        if len(pkt) == 12:
                            # 解析封包 (big-endian: H << 8 | L)
                            # 格式: [0xBB][STATE][CORNER][FRONT_H][FRONT_L][RIGHT_H][RIGHT_L][YAW_H][YAW_L][FLAGS][CHECKSUM][0x66]
                            state = pkt[1]
                            corner_count = pkt[2]
                            front_dist = (pkt[3] << 8) | pkt[4]
                            right_dist = (pkt[5] << 8) | pkt[6]
                            yaw = (pkt[7] << 8) | pkt[8]
                            if yaw > 32767:
                                yaw -= 65536
                            yaw = yaw / 10.0
                            flags = pkt[9]

                            # 每 0.5 秒顯示一次
                            if time.time() - last_status_time > 0.5:
                                print(f"  前方={front_dist:3d}cm  右側={right_dist:3d}cm  Yaw={yaw:6.1f}°  狀態={state}  角落={corner_count}")
                                last_status_time = time.time()
                            break

            time.sleep(0.05)

        print()
        print("-" * 50)
        print("發送 STOP 指令...")
        print("-" * 50)

        # 發送 STOP
        packet = build_command(CMD_STOP)
        ser.write(packet)
        print(f"[TX] STOP: {' '.join(f'{b:02X}' for b in packet)}")

        time.sleep(0.5)

        # 讀取剩餘訊息
        while ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line and not line.startswith('\xbb'):
                    print(f"[Arduino] {line}")
            except:
                ser.read(ser.in_waiting)
                break

        print()
        print("=" * 50)
        print(" 測試完成")
        print("=" * 50)
        print()
        print("預期結果：")
        print("  - 車子應該直走或輕微右偏（尋牆）")
        print("  - 如果原地自轉，檢查：")
        print("    1. 馬達接線是否正確")
        print("    2. 超聲波感測器是否正常")
        print("    3. config.py 的 LEFT/RIGHT_MOTOR_SCALE")

        ser.close()

    except serial.SerialException as e:
        print(f"[ERROR] 串口錯誤: {e}")
        print("提示: 嘗試 python3 test_straight.py /dev/ttyACM1")
    except KeyboardInterrupt:
        print("\n[中斷] 使用者取消")
        if 'ser' in locals():
            # 發送停止指令
            packet = build_command(CMD_STOP)
            ser.write(packet)
            time.sleep(0.1)
            ser.close()
            print("[OK] 已發送停止指令")

if __name__ == "__main__":
    main()
