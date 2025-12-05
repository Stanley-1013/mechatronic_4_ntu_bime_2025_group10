# v1 遙控版本 (期中)

此資料夾保存期中遙控版本的程式碼，作為備份和參考。

## 架構

- **Arduino**: 接收 Pi 的 PWM 指令，控制馬達，回傳感測器資料
- **Raspberry Pi**: 遙控器輸入 → 差速計算 → 發送 PWM 指令

## 通訊流程

```
搖桿 → Pi (20Hz) → Serial → Arduino → 馬達
                              ↓
                         感測器資料 (10Hz)
```

## 特點

- Pi 負責所有控制邏輯
- Arduino 僅執行 PWM 和感測器讀取
- 通訊頻率較高 (20Hz 指令)

## 檔案說明

- `arduino/main/`: Arduino 主程式
- `raspberry_pi/`: Pi 端程式 (遙控 + 自走)

---
*版本凍結於 2025-12-02*
