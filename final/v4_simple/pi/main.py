#!/usr/bin/env python3
"""
V4 Simple - Pi 端紅色偵測
使用 v3 的紅色偵測邏輯
"""

import cv2
import numpy as np
import serial
import time
import signal
import sys
import threading

# =================== 設定 ===================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# 攝影機
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
DETECTION_FPS = 15

# HSV 紅色範圍
RED_LOWER1 = (0, 100, 100)
RED_UPPER1 = (10, 255, 255)
RED_LOWER2 = (160, 100, 100)
RED_UPPER2 = (180, 255, 255)

# 偵測閾值
MIN_RED_AREA = 1000
RED_CLEAR_DELAY = 2.0  # 紅色消失後延遲 (秒)

# 指令代碼 (配合 V4 Arduino)
CMD_RED_DETECTED = 0x10   # 關吸塵器
CMD_RED_CLEARED = 0x11    # 開吸塵器

# =================== 紅色偵測類別 ===================
class RedDetector:
    def __init__(self):
        self.cap = None
        self.running = False
        self.thread = None
        self.detected = False
        self.area = 0
        self.lock = threading.Lock()

    def start(self):
        """啟動偵測"""
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        self.running = True
        self.thread = threading.Thread(target=self._detection_loop)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        """停止偵測"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()

    def get_status(self):
        """取得偵測狀態"""
        with self.lock:
            return self.detected, self.area

    def _detection_loop(self):
        """偵測迴圈 (背景執行緒)"""
        interval = 1.0 / DETECTION_FPS

        while self.running:
            start = time.time()

            ret, frame = self.cap.read()
            if not ret:
                continue

            # 高斯模糊減少雜訊
            blur = cv2.GaussianBlur(frame, (7, 7), 0)

            # 轉換 HSV
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

            # 紅色遮罩 (兩段)
            mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
            mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
            mask = cv2.bitwise_or(mask1, mask2)

            # 形態學處理
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 找輪廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)

            # 計算最大面積
            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > max_area:
                    max_area = area

            # 更新狀態
            with self.lock:
                self.area = max_area
                self.detected = max_area > MIN_RED_AREA

            # 維持頻率
            elapsed = time.time() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)

# =================== 主控制器 ===================
class V4Controller:
    def __init__(self):
        self.detector = RedDetector()
        self.ser = None
        self.running = False
        self.last_red_state = False
        self.red_clear_time = 0

    def send_command(self, cmd):
        """發送指令 (V4 格式: 0xAA + cmd)"""
        if self.ser is None:
            return False
        try:
            self.ser.write(bytes([0xAA, cmd]))
            self.ser.flush()
            return True
        except Exception as e:
            print(f"發送失敗: {e}")
            return False

    def start(self):
        """啟動系統"""
        print("V4 Simple - Raspberry Pi")
        print("=" * 40)

        # 連接 Arduino
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, write_timeout=0.1)
            self.ser.reset_output_buffer()
            print(f"Arduino 連接成功 ({SERIAL_PORT})")
            time.sleep(2)  # 等待 Arduino 重置
        except Exception as e:
            print(f"無法連接 Arduino: {e}")
            return

        # 啟動紅色偵測
        self.detector.start()
        print("紅色偵測已啟動")

        self.running = True
        self._main_loop()

    def stop(self):
        """停止系統"""
        self.running = False
        self.detector.stop()
        if self.ser:
            self.ser.close()
        print("系統已停止")

    def _main_loop(self):
        """主迴圈"""
        while self.running:
            # 讀取並顯示 Arduino 除錯訊息
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[Arduino] {line}")
                except:
                    pass

            detected, area = self.detector.get_status()

            # 紅色狀態變化處理
            if detected and not self.last_red_state:
                # 進入紅色區域 → 關閉吸塵器
                print(f"偵測到紅色 (面積: {area})")
                self.send_command(CMD_RED_DETECTED)

            elif not detected and self.last_red_state:
                # 離開紅色區域 → 記錄時間
                self.red_clear_time = time.time()

            elif not detected and self.red_clear_time > 0:
                # 延遲後重新開啟吸塵器
                if time.time() - self.red_clear_time > RED_CLEAR_DELAY:
                    print("紅色消失，重新開啟吸塵器")
                    self.send_command(CMD_RED_CLEARED)
                    self.red_clear_time = 0

            self.last_red_state = detected

            time.sleep(0.1)  # 10 Hz 檢查頻率

# =================== 信號處理 ===================
controller = None

def signal_handler(sig, frame):
    print("\n收到中斷信號")
    if controller:
        controller.stop()
    sys.exit(0)

if __name__ == '__main__':
    controller = V4Controller()
    signal.signal(signal.SIGINT, signal_handler)

    try:
        controller.start()
    except Exception as e:
        print(f"錯誤: {e}")
        if controller:
            controller.stop()
