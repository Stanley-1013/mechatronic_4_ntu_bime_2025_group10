# v5_competition - 紅色偵測模組
# 期末競賽使用版本

import cv2
import numpy as np

# 攝影機索引：
# 0 通常是內建鏡頭
# 1、2 之類的可能是 USB 攝影機
CAMERA_INDEX = 0  # 如果要改用 USB webcam 就把這裡改成 1 試試看

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print("無法開啟攝影機，請確認 CAMERA_INDEX 是否正確")
    exit()

# 設一些 HSV 顏色範圍來抓「紅色」
# 紅色在 HSV 會跨兩個區間，所以要取兩段再合併
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

prev_state = None  # 用來記錄上一幀是否有偵測到紅色（避免一直狂印）

while True:
    ret, frame = cap.read()
    if not ret:
        print("讀取畫面失敗，結束程式")
        break

    # 先做一點模糊，減少雜訊
    blur = cv2.GaussianBlur(frame, (7, 7), 0)

    # BGR -> HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # 兩段紅色區間
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # 合併兩段
    mask = cv2.bitwise_or(mask1, mask2)

    # 做一些形態學處理，去除雜點 & 填補空洞
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 找輪廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_detected = False

    for cnt in contours:
        area = cv2.contourArea(cnt)
        # 過濾掉太小的雜訊，可以依需求調整
        if area > 800:
            x, y, w, h = cv2.boundingRect(cnt)
            # 在原始畫面畫出框框
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            red_detected = True

    # 在畫面上顯示文字
    if red_detected:
        text = "Red object detected"
        color = (0, 0, 255)
    else:
        text = "No red object"
        color = (255, 255, 255)

    cv2.putText(frame, text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)

    # 也顯示紅色 Mask 方便看偵測結果
    cv2.imshow("Webcam", frame)
    cv2.imshow("Red Mask", mask)

    # 如果偵測狀態有變化，就在 console 印出來
    if prev_state != red_detected:
        if red_detected:
            print("偵測到紅色物體")
        else:
            print("紅色物體消失")
        prev_state = red_detected

    # 按下 q 離開
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
