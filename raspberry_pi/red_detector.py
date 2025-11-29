#!/usr/bin/env python3
"""
red_detector.py - 紅色區域偵測器
版本: 1.0
日期: 2025-11-28

偵測牆上的紅色圓形標記，用於自走模式避開紅色紙屑區域。

使用方式:
    # 作為模組
    from red_detector import RedDetector
    detector = RedDetector()
    detected, area = detector.detect()

    # 獨立測試 (顯示視窗)
    python3 red_detector.py
"""

import cv2
import numpy as np
import time


class RedDetector:
    """紅色區域偵測器"""

    # HSV 紅色範圍 (紅色跨 0 和 180，需要兩段)
    LOWER_RED1 = np.array([0, 100, 100])
    UPPER_RED1 = np.array([10, 255, 255])
    LOWER_RED2 = np.array([160, 100, 100])
    UPPER_RED2 = np.array([180, 255, 255])

    # 偵測參數
    MIN_AREA = 800           # 最小輪廓面積 (過濾雜訊)
    BLUR_KERNEL = (7, 7)     # 高斯模糊核心大小
    MORPH_KERNEL_SIZE = 5    # 形態學處理核心大小

    def __init__(self, camera_index=0, width=320, height=240):
        """
        初始化紅色偵測器

        Args:
            camera_index: 攝影機索引 (0=內建, 1=USB)
            width: 影像寬度
            height: 影像高度
        """
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None
        self.kernel = np.ones((self.MORPH_KERNEL_SIZE, self.MORPH_KERNEL_SIZE), np.uint8)

        # 狀態追蹤
        self.prev_detected = False
        self.last_area = 0

    def open(self):
        """開啟攝影機"""
        if self.cap is not None and self.cap.isOpened():
            return True

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            print(f"[RedDetector] 無法開啟攝影機 (index={self.camera_index})")
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        print(f"[RedDetector] 攝影機已開啟 ({self.width}x{self.height})")
        return True

    def close(self):
        """關閉攝影機"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            print("[RedDetector] 攝影機已關閉")

    def detect(self):
        """
        偵測紅色區域

        Returns:
            tuple: (detected: bool, max_area: int)
                - detected: 是否偵測到紅色
                - max_area: 最大紅色區塊面積 (用於判斷距離)
        """
        if self.cap is None or not self.cap.isOpened():
            if not self.open():
                return False, 0

        ret, frame = self.cap.read()
        if not ret:
            return False, 0

        # 高斯模糊減少雜訊
        blur = cv2.GaussianBlur(frame, self.BLUR_KERNEL, 0)

        # BGR -> HSV
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # 兩段紅色區間
        mask1 = cv2.inRange(hsv, self.LOWER_RED1, self.UPPER_RED1)
        mask2 = cv2.inRange(hsv, self.LOWER_RED2, self.UPPER_RED2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 形態學處理：去除雜點 + 填補空洞
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # 找輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 找最大面積的紅色區塊
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.MIN_AREA and area > max_area:
                max_area = area

        detected = max_area > 0
        self.last_area = max_area

        # 狀態變化時印出
        if detected != self.prev_detected:
            if detected:
                print(f"[RedDetector] 偵測到紅色 (面積: {max_area})")
            else:
                print("[RedDetector] 紅色消失")
            self.prev_detected = detected

        return detected, max_area

    def detect_with_frame(self):
        """
        偵測紅色區域並回傳處理後的影像 (用於除錯/顯示)

        Returns:
            tuple: (detected, max_area, frame, mask)
        """
        if self.cap is None or not self.cap.isOpened():
            if not self.open():
                return False, 0, None, None

        ret, frame = self.cap.read()
        if not ret:
            return False, 0, None, None

        # 高斯模糊
        blur = cv2.GaussianBlur(frame, self.BLUR_KERNEL, 0)

        # BGR -> HSV
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # 兩段紅色區間
        mask1 = cv2.inRange(hsv, self.LOWER_RED1, self.UPPER_RED1)
        mask2 = cv2.inRange(hsv, self.LOWER_RED2, self.UPPER_RED2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 形態學處理
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # 找輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 畫出偵測框
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.MIN_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                if area > max_area:
                    max_area = area

        detected = max_area > 0

        # 顯示文字
        if detected:
            text = f"RED DETECTED (area={max_area})"
            color = (0, 0, 255)
        else:
            text = "No red object"
            color = (255, 255, 255)

        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

        self.last_area = max_area
        self.prev_detected = detected

        return detected, max_area, frame, mask

    def is_opened(self):
        """攝影機是否已開啟"""
        return self.cap is not None and self.cap.isOpened()


# ==================== 測試程式 ====================
if __name__ == "__main__":
    print("=" * 50)
    print("  紅色偵測器測試")
    print("  Red Detector Test")
    print("=" * 50)
    print()
    print("按 'q' 離開")
    print()

    detector = RedDetector()

    if not detector.open():
        print("無法開啟攝影機")
        exit(1)

    try:
        while True:
            detected, area, frame, mask = detector.detect_with_frame()

            if frame is not None:
                cv2.imshow("Camera", frame)
                cv2.imshow("Red Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n使用者中止")

    finally:
        detector.close()
        cv2.destroyAllWindows()
        print("測試結束")
