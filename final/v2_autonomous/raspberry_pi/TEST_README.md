# Pi-Arduino 整合測試說明

## 概述

`test_integration.py` 是 v2_autonomous 架構的整合測試腳本，用於驗證 Raspberry Pi 與 Arduino 之間的通訊協定和控制功能。

## 前置需求

### 硬體連接
- Arduino 已燒錄 `v2_autonomous/arduino/main.ino`
- 透過 USB 連接到 Raspberry Pi（預設串列埠: `/dev/ttyACM0`）
- 超聲波感測器已連接到 Arduino

### 軟體依賴
```bash
pip3 install pyserial
```

### 權限設定
確保有權限訪問串列埠：
```bash
sudo usermod -a -G dialout $USER
# 登出後重新登入生效
```

## 使用方式

### 基本執行
```bash
cd v2_autonomous/raspberry_pi
python3 test_integration.py
```

### 指定串列埠
```bash
python3 test_integration.py --port /dev/ttyUSB0
```

### 指定波特率
```bash
python3 test_integration.py --baudrate 9600
```

## 測試項目

### 1. 通訊測試
- 發送 `CMD_QUERY_STATE` 指令
- 驗證狀態封包格式正確
- 驗證 checksum
- 驗證數值範圍（距離、yaw 角度）

### 2. 指令-回應測試
- **CMD_START**: 驗證狀態從 `IDLE` 變為 `FIND_WALL` 或 `FORWARD`
- **CMD_STOP**: 驗證狀態回到 `IDLE`
- **CMD_SET_VACUUM(1)**: 驗證 `flags` 含 `VACUUM_ENABLED`
- **CMD_SET_VACUUM(0)**: 驗證 `flags` 不含 `VACUUM_ENABLED`

### 3. 狀態監控測試
- 連續接收 10 個狀態封包
- 統計成功率、checksum 錯誤率
- 驗證 yaw、distance 數值範圍
- 成功率應 >= 80%

## 輸出範例

```
==================================================
  Pi-Arduino 整合測試
==================================================
[連接] 嘗試連接 /dev/ttyACM0 at 115200 baud...
[連接] 成功連接

==================================================
測試 1: 基本通訊
==================================================
[TX] CommandPacket(cmd=QUERY_STATE, payload_len=0) - 0xAA 0x05 0x00 0x05 0x55
[RX] StatePacket(state=IDLE, corners=0, front=150cm, right=45cm, yaw=0.0°, flags=0x00)
  [PASS] 收到狀態封包
  [PASS] 狀態值在有效範圍
  [PASS] 角落計數在有效範圍
  [PASS] 前方距離在有效範圍
  [PASS] 右方距離在有效範圍
  [PASS] 偏航角在有效範圍（-360.0 ~ 360.0度）

==================================================
測試 2: 指令-回應
==================================================

[2.1] 檢查初始狀態
[RX] StatePacket(state=IDLE, corners=0, front=150cm, right=45cm, yaw=0.0°, flags=0x00)
  [PASS] 初始狀態應為 IDLE (0x00)

[2.2] 發送 START 指令
[TX] CommandPacket(cmd=START, payload_len=0) - 0xAA 0x01 0x00 0x01 0x55
[RX] StatePacket(state=FORWARD, corners=0, front=150cm, right=45cm, yaw=0.0°, flags=0x00)
  [PASS] START 後狀態應為 FIND_WALL 或 FORWARD，實際: FORWARD

[2.3] 發送 STOP 指令
[TX] CommandPacket(cmd=STOP, payload_len=0) - 0xAA 0x02 0x00 0x02 0x55
[RX] StatePacket(state=IDLE, corners=0, front=150cm, right=45cm, yaw=0.0°, flags=0x00)
  [PASS] STOP 後狀態應為 IDLE，實際: IDLE

[2.4] 測試吸塵器控制 - 開啟
[TX] CommandPacket(cmd=SET_VACUUM, payload_len=1) - 0xAA 0x04 0x01 0x01 0x04 0x55
[RX] StatePacket(state=IDLE, corners=0, front=150cm, right=45cm, yaw=0.0°, flags=0x04)
  [PASS] SET_VACUUM(1) 後 flags 應含 VACUUM_ENABLED

[2.5] 測試吸塵器控制 - 關閉
[TX] CommandPacket(cmd=SET_VACUUM, payload_len=1) - 0xAA 0x04 0x01 0x00 0x05 0x55
[RX] StatePacket(state=IDLE, corners=0, front=150cm, right=45cm, yaw=0.0°, flags=0x00)
  [PASS] SET_VACUUM(0) 後 flags 應不含 VACUUM_ENABLED

==================================================
測試 3: 狀態監控
==================================================

[3.1] 連續接收 10 個狀態封包
  [1/10] 成功 - 前方: 150cm, 右方: 45cm, yaw: 0.0度
  [2/10] 成功 - 前方: 150cm, 右方: 45cm, yaw: 0.0度
  ...
  [10/10] 成功 - 前方: 150cm, 右方: 45cm, yaw: 0.0度

[統計]
  成功: 10/10 (100.0%)
  失敗: 0/10
  [PASS] 成功率應 >= 80%，實際: 100.0%

==================================================
測試總結
==================================================
通過: 14
失敗: 0

[結果] 所有測試通過 ✓
```

## 故障排除

### 問題: 無法連接到 Arduino
**解決方法:**
1. 確認 Arduino 已連接：`ls /dev/ttyACM* /dev/ttyUSB*`
2. 確認權限：`groups | grep dialout`
3. 嘗試不同串列埠：`--port /dev/ttyUSB0`

### 問題: 收不到狀態封包
**解決方法:**
1. 確認 Arduino 程式已正確燒錄
2. 檢查 Arduino Serial Monitor 是否有輸出
3. 增加超時時間（修改 `TIMEOUT` 常數）

### 問題: Checksum 錯誤率高
**解決方法:**
1. 檢查 USB 線材品質
2. 降低波特率：`--baudrate 9600`
3. 檢查 Arduino 電源供應穩定性

### 問題: 狀態不變化
**解決方法:**
1. 確認超聲波感測器已連接
2. 檢查 Arduino Serial Monitor 輸出是否正常
3. 測試單一指令：修改 `test_command_response()` 加入更多 debug 訊息

## 測試結束後

測試腳本會在結束時發送 `CMD_STOP` 指令，確保馬達和吸塵器停止運轉，這是安全的設計。

## 進階用法

### 只執行特定測試
編輯 `run_all_tests()` 函數，註解掉不需要的測試：

```python
def run_all_tests(self):
    if not self.connect():
        return

    try:
        self.test_communication()
        # self.test_command_response()  # 跳過此測試
        self.test_status_monitoring()
    finally:
        self.disconnect()
```

### 增加延遲觀察
在測試項目中增加 `time.sleep()` 可以觀察機器人行為：

```python
self.send_command(create_cmd_start())
time.sleep(5.0)  # 觀察 5 秒
```

## 相關文件

- `protocol.py` - 通訊協定定義
- `controller.py` - Pi 端控制器
- `../arduino/main.ino` - Arduino 主程式
