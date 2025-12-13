# V4 Simple - 簡化版沿牆車

基於別組實測程式碼，加上紅色偵測功能。

## 特點

- **簡單 PD 沿牆**：`error = rightFront - rightRear`
- **防突波濾波**：限制 error 變化 ±8
- **出場判斷**：右前 > 30cm → 輕微右轉找牆
- **紅色偵測**：Pi 攝影機偵測 → 通知 Arduino 停車

## 參數

| 參數 | 值 | 說明 |
|------|-----|------|
| BASE_SPEED_L | 64 | 左輪基準 |
| BASE_SPEED_R | 77 | 右輪基準 |
| TURN_SPEED_R | 70 | 左轉時右輪 |
| Kp | 10.0 | 比例增益 |
| Kd | 2.0 | 微分增益 |
| TURN_THRESHOLD | 20cm | 前方轉彎距離 |
| 左轉時間 | 1200ms | 固定時間 |

## 檔案結構

```
v4_simple/
├── arduino/main/
│   └── main.ino      # Arduino 主程式
├── pi/
│   └── main.py       # Pi 紅色偵測
└── README.md
```

## 通訊協定

| 命令 | 代碼 | 方向 | 說明 |
|------|------|------|------|
| RED_DETECTED | 0x10 | Pi→Ard | 偵測到紅色 |
| RED_CLEARED | 0x11 | Pi→Ard | 紅色消失 |
| STOP | 0x01 | Pi→Ard | 永久停車 |

格式：`0xAA + CMD`

## 使用

1. 上傳 `main.ino` 到 Arduino
2. 在 Pi 執行 `python3 main.py`
