/*
 * serial_protocol.cpp - Serial 通訊協定實作
 * 版本: 2.0 (新增 IMU 資料)
 * 日期: 2025-11-29
 *
 * 參考: 04_ICD_介面規格.md - 2.2, 2.3
 */

#include "serial_protocol.h"

bool parseMotorPacket(uint8_t* packet, int16_t& left_pwm, int16_t& right_pwm, bool& vacuum, bool& ultrasonic_enable) {
    // 驗證 Header
    if (packet[0] != MOTOR_HEADER) {
        return false;
    }

    // 驗證 Footer
    if (packet[7] != MOTOR_FOOTER) {
        return false;
    }

    // 驗證 Checksum (XOR of bytes 1-5)
    uint8_t checksum = packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5];
    if (checksum != packet[6]) {
        return false;
    }

    // 解析 Left PWM (int16, little-endian)
    left_pwm = (int16_t)(packet[1] | (packet[2] << 8));

    // 解析 Right PWM (int16, little-endian)
    right_pwm = (int16_t)(packet[3] | (packet[4] << 8));

    // 解析 Flags
    // bit0 = vacuum, bit1 = ultrasonic_enable
    vacuum = (packet[5] & 0x01) != 0;
    ultrasonic_enable = (packet[5] & 0x02) != 0;

    return true;
}

void buildSensorPacketV2(uint8_t* packet, uint16_t front_distance, uint16_t right_distance,
                         float yaw_deg, float gyro_z, uint8_t status) {
    // Byte 0: Header
    packet[0] = SENSOR_HEADER;

    // Byte 1-2: Front Distance (uint16, little-endian)
    packet[1] = front_distance & 0xFF;
    packet[2] = (front_distance >> 8) & 0xFF;

    // Byte 3-4: Right Distance (uint16, little-endian)
    packet[3] = right_distance & 0xFF;
    packet[4] = (right_distance >> 8) & 0xFF;

    // Byte 5-6: Yaw Angle (int16, little-endian, 度數 × 10)
    // 將 float 轉為 int16，保留一位小數精度
    int16_t yaw_int = (int16_t)(yaw_deg * 10.0f);
    packet[5] = yaw_int & 0xFF;
    packet[6] = (yaw_int >> 8) & 0xFF;

    // Byte 7: Gyro Z (int8, deg/s)
    // 限制範圍 -127 ~ +127
    int8_t gz = (int8_t)constrain((int)gyro_z, -127, 127);
    packet[7] = (uint8_t)gz;

    // Byte 8: Status
    packet[8] = status;

    // Byte 9: Reserved
    packet[9] = 0x00;

    // Byte 10: Checksum (XOR of bytes 1-9)
    packet[10] = packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^
                 packet[5] ^ packet[6] ^ packet[7] ^ packet[8] ^ packet[9];

    // Byte 11: Footer
    packet[11] = SENSOR_FOOTER;
}

// 向下相容的舊版本封包建構函數
void buildSensorPacket(uint8_t* packet, uint16_t front_distance, uint16_t right_distance, uint8_t status) {
    // Byte 0: Header
    packet[0] = SENSOR_HEADER;

    // Byte 1-2: Front Distance (uint16, little-endian)
    packet[1] = front_distance & 0xFF;
    packet[2] = (front_distance >> 8) & 0xFF;

    // Byte 3-4: Right Distance (uint16, little-endian)
    packet[3] = right_distance & 0xFF;
    packet[4] = (right_distance >> 8) & 0xFF;

    // Byte 5: Status
    packet[5] = status;

    // Byte 6: Checksum (XOR of bytes 1-5)
    packet[6] = packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5];

    // Byte 7: Footer
    packet[7] = SENSOR_FOOTER;
}
