#!/usr/bin/env python3
"""
MJPEG 即時影像串流伺服器
用於期中遙控模式的監控畫面

使用方式:
    python3 mjpeg_stream.py

在同一網路的設備瀏覽器開啟:
    http://<樹莓派IP>:5000/

此程式獨立運行，不影響自走系統
"""

from flask import Flask, Response
import cv2

app = Flask(__name__)


def gen_frames():
    """產生 MJPEG 影像串流"""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        success, frame = cap.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/mjpeg')
def mjpeg():
    """MJPEG 串流端點"""
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    """首頁顯示影像"""
    return '<h1>即時影像串流</h1><img src="/mjpeg">'


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
