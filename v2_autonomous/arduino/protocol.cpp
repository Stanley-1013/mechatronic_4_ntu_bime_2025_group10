#include "protocol.h"

// ============================================================================
// Checksum 計算函數
// ============================================================================

/**
 * 計算 Pi → Arduino 指令封包的 checksum
 * checksum = CMD ^ PAYLOAD_LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n]
 */
uint8_t calc_cmd_checksum(uint8_t cmd, uint8_t payload_len, const uint8_t* payload) {
    uint8_t checksum = cmd ^ payload_len;
    if (payload != nullptr && payload_len > 0) {
        for (int i = 0; i < payload_len; i++) {
            checksum ^= payload[i];
        }
    }
    return checksum;
}

/**
 * 計算 Arduino → Pi 狀態回報的 checksum
 * checksum = STATE ^ CORNER_COUNT ^ FRONT_H ^ FRONT_L ^ RIGHT_H ^ RIGHT_L ^ YAW_H ^ YAW_L ^ FLAGS
 *
 * state_packet 應指向 packet[1]，包含 9 個位元組的資料
 */
uint8_t calc_state_checksum(const uint8_t* state_packet) {
    // state_packet 應指向 [STATE, CORNER_COUNT, FRONT_H, FRONT_L, RIGHT_H, RIGHT_L, YAW_H, YAW_L, FLAGS]
    // 共 9 個位元組
    uint8_t checksum = 0;
    for (int i = 0; i < 9; i++) {
        checksum ^= state_packet[i];
    }
    return checksum;
}

// ============================================================================
// 封包驗證函數
// ============================================================================

/**
 * 驗證 Pi → Arduino 指令封包
 *
 * 格式: [HEADER=0xAA][CMD][PAYLOAD_LEN][PAYLOAD][CHECKSUM][FOOTER=0x55]
 *
 * @param packet  指令封包 buffer
 * @param length  封包長度 (bytes)
 * @return true 若封包有效，false 若驗證失敗
 */
bool verify_cmd_packet(const uint8_t* packet, uint16_t length) {
    // 最小長度檢查: HEADER + CMD + PAYLOAD_LEN + CHECKSUM + FOOTER = 5 bytes
    if (length < PKT_CMD_MIN_LENGTH) {
        return false;
    }

    // 最大長度檢查
    if (length > PKT_CMD_MAX_LENGTH) {
        return false;
    }

    // Header 檢查
    if (packet[0] != PKT_HEADER_CMD) {
        return false;
    }

    // Footer 檢查
    if (packet[length - 1] != PKT_FOOTER_CMD) {
        return false;
    }

    // 取得 payload 長度
    uint8_t payload_len = packet[2];

    // 長度一致性檢查: 5 (header + cmd + len + checksum + footer) + payload_len = total length
    if (length != 5 + payload_len) {
        return false;
    }

    // Checksum 驗證
    // 期望的 checksum 計算: CMD ^ PAYLOAD_LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n]
    uint8_t expected_checksum = calc_cmd_checksum(packet[1], payload_len, &packet[3]);
    if (packet[length - 2] != expected_checksum) {
        return false;
    }

    return true;
}

/**
 * 驗證 Arduino → Pi 狀態回報封包
 *
 * 格式: [HEADER=0xBB][STATE][CORNER_COUNT][FRONT_H][FRONT_L][RIGHT_H][RIGHT_L]
 *       [YAW_H][YAW_L][FLAGS][CHECKSUM][FOOTER=0x66]
 *
 * @param packet  狀態封包 buffer
 * @param length  封包長度 (bytes) - 應該總是 12
 * @return true 若封包有效，false 若驗證失敗
 */
bool verify_state_packet(const uint8_t* packet, uint16_t length) {
    // 固定長度檢查
    if (length != PKT_STATE_LENGTH) {
        return false;
    }

    // Header 檢查
    if (packet[0] != PKT_HEADER_STATE) {
        return false;
    }

    // Footer 檢查
    if (packet[length - 1] != PKT_FOOTER_STATE) {
        return false;
    }

    // Checksum 驗證
    // checksum 應該是 packet[1] 到 packet[9] 的 XOR (9 個位元組)
    uint8_t expected_checksum = calc_state_checksum(&packet[1]);
    if (packet[length - 2] != expected_checksum) {
        return false;
    }

    return true;
}
