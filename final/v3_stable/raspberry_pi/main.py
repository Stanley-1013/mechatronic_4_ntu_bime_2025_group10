#!/usr/bin/env python3
# main.py - V3 Raspberry Pi 主程式

import time
import signal
import sys
from red_detector import RedDetector
from serial_comm import SerialComm
from config import RED_CLEAR_DELAY

class V3Controller:
    def __init__(self):
        self.detector = RedDetector()
        self.serial = SerialComm()
        self.running = False

        self.last_red_state = False
        self.red_clear_time = 0

    def start(self):
        """啟動系統"""
        print("V3 自走吸塵車 - Raspberry Pi")
        print("=" * 40)

        # 連接 Arduino
        if not self.serial.connect():
            print("無法連接 Arduino，退出")
            return
        print("Arduino 連接成功")

        # 啟動紅色偵測
        self.detector.start()
        print("紅色偵測已啟動")

        self.running = True
        self._main_loop()

    def stop(self):
        """停止系統"""
        self.running = False
        self.detector.stop()
        self.serial.close()
        print("系統已停止")

    def _main_loop(self):
        """主迴圈"""
        while self.running:
            # 讀取並顯示 Arduino 除錯訊息
            if self.serial.ser and self.serial.ser.in_waiting > 0:
                try:
                    line = self.serial.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[Arduino] {line}")
                except:
                    pass

            detected, area = self.detector.get_status()

            # 紅色狀態變化處理
            if detected and not self.last_red_state:
                # 進入紅色區域 → 關閉吸塵器
                print(f"偵測到紅色 (面積: {area})")
                self.serial.vacuum_off()

            elif not detected and self.last_red_state:
                # 離開紅色區域 → 記錄時間
                self.red_clear_time = time.time()

            elif not detected and self.red_clear_time > 0:
                # 延遲後重新開啟吸塵器
                if time.time() - self.red_clear_time > RED_CLEAR_DELAY:
                    print("紅色消失，重新開啟吸塵器")
                    self.serial.vacuum_on()
                    self.red_clear_time = 0

            self.last_red_state = detected

            time.sleep(0.1)  # 10 Hz 檢查頻率

def signal_handler(sig, frame):
    """信號處理"""
    print("\n收到中斷信號")
    controller.stop()
    sys.exit(0)

if __name__ == '__main__':
    controller = V3Controller()
    signal.signal(signal.SIGINT, signal_handler)

    try:
        controller.start()
    except Exception as e:
        print(f"錯誤: {e}")
        controller.stop()
