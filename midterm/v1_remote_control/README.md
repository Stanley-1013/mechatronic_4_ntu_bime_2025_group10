# v1 遙控版本 (期中成功版本)

此資料夾保存 **期中測試成功** 的遙控版本程式碼。

> ⚠️ **重要**: 此版本從 commit `1dd3f0e` (2025-11-19) 恢復，為期中實測成功的版本。

## 版本資訊

| 項目 | 版本 |
|------|------|
| Arduino main.ino | v1.2 |
| 來源 commit | `1dd3f0e` (2025-11-19) |
| Baud Rate | 9600 |
| 超聲波 | 左側 + 右側 |
| IMU | ❌ 無 |

## 架構

- **Arduino**: 接收 Pi 的 PWM 指令，控制馬達，回傳感測器資料
- **Raspberry Pi**: 遙控器輸入 → 差速計算 → 發送 PWM 指令

## 通訊流程

```
搖桿 → Pi (20Hz) → Serial (9600) → Arduino → 馬達
                                      ↓
                                 感測器資料 (10Hz)
```

## 特點

- Pi 負責所有控制邏輯
- Arduino 僅執行 PWM 和感測器讀取
- 通訊頻率 20Hz 指令
- **穩定運行，無 timeout 問題**

## 檔案說明

- `arduino/main/`: Arduino 主程式 (v1.2)
- `raspberry_pi/`: Pi 端程式 (遙控模式)

## 與現行版本差異

現行 `arduino/main/` 已升級為 v2.0，主要差異：
- Baud Rate: 9600 → 115200
- 新增 MPU6050 IMU 支援
- 超聲波: 左+右 → 前+右
- 封包格式: 8-byte → 12-byte

---
*版本恢復自 commit 1dd3f0e (2025-11-19)*
