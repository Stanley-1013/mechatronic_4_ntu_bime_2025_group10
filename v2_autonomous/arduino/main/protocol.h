#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// ============================================================================
// Arduino-Pi 通訊協定定義
// ============================================================================
// 架構: Arduino 自主沿牆控制，Pi 發送高階事件指令
// 通訊頻率: 事件驅動 (低頻率)
// ============================================================================

// --- Pi → Arduino 指令 (事件驅動) ---
#define CMD_START       0x01  // 開始自主沿牆控制
#define CMD_STOP        0x02  // 停止執行
#define CMD_AVOID_RED   0x03  // 迴避紅色區域
#define CMD_SET_VACUUM  0x04  // 設定吸塵器開關 (payload: 0x00 關閉, 0x01 開啟)
#define CMD_QUERY_STATE 0x05  // 查詢當前狀態 (無 payload)
#define CMD_SET_PID     0x06  // 設定 PID 參數 (payload: Kp*1000, Ki*1000, Kd*1000 各 2 bytes)
#define CMD_SET_PARAMS  0x07  // 設定所有控制參數 (payload: 32 bytes，見 Pi config.py)

// --- Arduino → Pi 狀態回報 ---
// 注意: v2.0 簡化為連續控制，只回報 IDLE/RUNNING/DONE
#define STATE_IDLE      0x00  // 空閒/待命
#define STATE_RUNNING   0x02  // 執行中 (連續沿牆控制)
#define STATE_DONE      0x05  // 任務完成
#define STATE_ERROR     0xFF  // 錯誤狀態

// 舊狀態定義 (保留供相容，但不再使用)
// #define STATE_FIND_WALL 0x01
// #define STATE_FORWARD   0x02
// #define STATE_BACKUP    0x03
// #define STATE_TURN_LEFT 0x04

// --- 封包格式常數 ---

// Pi → Arduino 指令封包
//   [HEADER=0xAA][CMD][PAYLOAD_LEN][PAYLOAD][CHECKSUM][FOOTER=0x55]
//   HEADER      : 1 byte (0xAA)
//   CMD         : 1 byte (指令類型)
//   PAYLOAD_LEN : 1 byte (payload 長度, 0-255)
//   PAYLOAD     : 0-255 bytes (指令參數)
//   CHECKSUM    : 1 byte (CMD ^ PAYLOAD_LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n])
//   FOOTER      : 1 byte (0x55)
//   總長度      : 5 + PAYLOAD_LEN bytes

#define PKT_HEADER_CMD       0xAA
#define PKT_FOOTER_CMD       0x55
#define PKT_HEADER_STATE     0xBB
#define PKT_FOOTER_STATE     0x66

// Arduino → Pi 狀態回報封包
//   [HEADER=0xBB][STATE][CORNER_COUNT][FRONT_DIST_H][FRONT_DIST_L]
//   [RIGHT_DIST_H][RIGHT_DIST_L][YAW_H][YAW_L][FLAGS][CHECKSUM][FOOTER=0x66]
//
//   HEADER       : 1 byte (0xBB)
//   STATE        : 1 byte (當前狀態代碼)
//   CORNER_COUNT : 1 byte (已掃描角落數)
//   FRONT_DIST_H : 1 byte (前方距離高位)
//   FRONT_DIST_L : 1 byte (前方距離低位)
//   RIGHT_DIST_H : 1 byte (右方距離高位)
//   RIGHT_DIST_L : 1 byte (右方距離低位)
//   YAW_H        : 1 byte (偏航角高位)
//   YAW_L        : 1 byte (偏航角低位)
//   FLAGS        : 1 byte (狀態旗標)
//                  bit 0: 偵測到紅色 (1=是)
//                  bit 1: 達到目標 (1=是)
//                  bit 2: 吸塵器啟用 (1=啟用)
//                  bit 3: 傳感器故障 (1=故障)
//   CHECKSUM     : 1 byte (所有資料位的 XOR)
//   FOOTER       : 1 byte (0x66)
//   總長度       : 12 bytes (固定)

#define PKT_STATE_LENGTH     12  // Arduino → Pi 狀態封包固定長度
#define PKT_CMD_MIN_LENGTH   5   // Pi → Arduino 最小長度 (header + cmd + payload_len + checksum + footer)
#define PKT_CMD_MAX_LENGTH   260 // Pi → Arduino 最大長度 (header + cmd + len + 255 payload + checksum + footer)

// --- FLAGS 位定義 ---
#define FLAG_RED_DETECTED    0x01  // bit 0: 偵測到紅色
#define FLAG_TARGET_REACHED  0x02  // bit 1: 達到目標
#define FLAG_VACUUM_ENABLED  0x04  // bit 2: 吸塵器啟用
#define FLAG_SENSOR_ERROR    0x08  // bit 3: 傳感器故障

// --- Payload 常數 ---
#define VACUUM_OFF           0x00
#define VACUUM_ON            0x01

// --- Checksum 計算函數 (可在 .cpp 實現) ---
// 計算 Pi → Arduino 指令封包的 checksum
// checksum = CMD ^ PAYLOAD_LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n]
uint8_t calc_cmd_checksum(uint8_t cmd, uint8_t payload_len, const uint8_t* payload);

// 計算 Arduino → Pi 狀態回報的 checksum
// checksum = STATE ^ CORNER_COUNT ^ FRONT_H ^ FRONT_L ^ RIGHT_H ^ RIGHT_L ^ YAW_H ^ YAW_L ^ FLAGS
uint8_t calc_state_checksum(const uint8_t* state_packet);

// 驗證 Pi → Arduino 指令封包
// 檢查 header, footer, checksum
bool verify_cmd_packet(const uint8_t* packet, uint16_t length);

// 驗證 Arduino → Pi 狀態回報封包
// 檢查 header, footer, checksum, 固定長度
bool verify_state_packet(const uint8_t* packet, uint16_t length);

#endif // PROTOCOL_H
