# red_detector.py - 紅色偵測模組

import cv2
import numpy as np
import threading
import time
from config import *

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
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

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
