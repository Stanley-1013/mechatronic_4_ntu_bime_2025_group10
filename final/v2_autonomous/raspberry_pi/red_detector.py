#!/usr/bin/env python3
"""
red_detector.py - 紅色區域偵測器
版本: 2.0 (背景執行緒版本)
日期: 2025-11-29

偵測牆上的紅色圓形標記，用於自走模式避開紅色紙屑區域。

v2.0 變更:
- 新增背景執行緒模式，避免阻塞主控制迴圈
- detect() 變成非阻塞，直接回傳最新結果

使用方式:
    # 作為模組 (背景執行緒)
    from red_detector import RedDetector
    detector = RedDetector()
    detector.start()  # 啟動背景執行緒
    detected, area = detector.get_result()  # 非阻塞取得結果
    detector.stop()   # 停止

    # 獨立測試 (顯示視窗)
    python3 red_detector.py
"""

import cv2
import numpy as np
import time
import threading


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

        # 背景執行緒相關
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._result_detected = False
        self._result_area = 0

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
        self.stop()  # 先停止背景執行緒
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            print("[RedDetector] 攝影機已關閉")

    # ==================== 背景執行緒 API ====================

    def start(self):
        """啟動背景偵測執行緒"""
        if self._running:
            return True

        if not self.open():
            return False

        self._running = True
        self._thread = threading.Thread(target=self._detection_loop, daemon=True)
        self._thread.start()
        print("[RedDetector] 背景執行緒已啟動")
        return True

    def stop(self):
        """停止背景偵測執行緒"""
        if not self._running:
            return

        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        print("[RedDetector] 背景執行緒已停止")

    def get_result(self):
        """
        非阻塞取得最新偵測結果

        Returns:
            tuple: (detected: bool, area: int)
        """
        with self._lock:
            return self._result_detected, self._result_area

    def _detection_loop(self):
        """背景偵測迴圈"""
        while self._running:
            detected, area = self._detect_internal()

            # 更新結果 (thread-safe)
            with self._lock:
                self._result_detected = detected
                self._result_area = area

            # 控制偵測頻率 (~15 FPS)
            time.sleep(0.066)

    def detect(self):
        """
        偵測紅色區域

        如果背景執行緒已啟動，直接回傳最新結果 (非阻塞)
        否則進行同步偵測 (阻塞)

        Returns:
            tuple: (detected: bool, max_area: int)
                - detected: 是否偵測到紅色
                - max_area: 最大紅色區塊面積 (用於判斷距離)
        """
        # 背景模式：直接回傳結果
        if self._running:
            return self.get_result()

        # 同步模式：進行偵測
        return self._detect_internal()

    def _detect_internal(self):
        """
        內部偵測實作 (實際的 OpenCV 處理)

        Returns:
            tuple: (detected: bool, max_area: int)
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
