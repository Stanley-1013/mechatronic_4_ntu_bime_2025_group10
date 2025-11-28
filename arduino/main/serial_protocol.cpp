/*
 * serial_protocol.cpp - Serial 通訊協定實作
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

void buildSensorPacket(uint8_t* packet, uint16_t left_distance, uint16_t right_distance, uint8_t status) {
    // Byte 0: Header
    packet[0] = SENSOR_HEADER;

    // Byte 1-2: Left Distance (uint16, little-endian)
    packet[1] = left_distance & 0xFF;
    packet[2] = (left_distance >> 8) & 0xFF;

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
