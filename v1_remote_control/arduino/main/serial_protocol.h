/*
 * serial_protocol.h - Serial 通訊協定模組
 * 版本: 2.0 (新增 IMU 資料)
 * 日期: 2025-11-29
 *
 * 參考: 04_ICD_介面規格.md - 2. Serial 通訊介面
 *
 * 封包格式變更:
 *   v1.0: 8 bytes (僅超聲波)
 *   v2.0: 12 bytes (超聲波 + IMU)
 */

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

// 封包格式常數
#define PACKET_SIZE_MOTOR 8       // Pi → Arduino 馬達指令封包 (維持不變)
#define PACKET_SIZE_SENSOR 12     // Arduino → Pi 感測器封包 (擴充為 12 bytes)
#define PACKET_SIZE 8             // 向下相容：馬達指令用

#define MOTOR_HEADER 0xAA
#define MOTOR_FOOTER 0x55
#define SENSOR_HEADER 0xBB
#define SENSOR_FOOTER 0x66

/**
 * @brief 解析馬達指令封包（Pi → Arduino）
 * @param packet 8-byte 封包
 * @param left_pwm 輸出：左輪 PWM (-255 ~ +255)
 * @param right_pwm 輸出：右輪 PWM (-255 ~ +255)
 * @param vacuum 輸出：吸塵器狀態
 * @param ultrasonic_enable 輸出：超聲波啟用狀態
 * @return true = 封包有效，false = 封包無效
 *
 * 封包格式 (8 bytes):
 *   Byte 0: Header (0xAA)
 *   Byte 1-2: Left PWM (int16, little-endian)
 *   Byte 3-4: Right PWM (int16, little-endian)
 *   Byte 5: Flags (bit0=vacuum, bit1=ultrasonic_enable)
 *   Byte 6: Checksum (XOR of bytes 1-5)
 *   Byte 7: Footer (0x55)
 */
bool parseMotorPacket(uint8_t* packet, int16_t& left_pwm, int16_t& right_pwm, bool& vacuum, bool& ultrasonic_enable);

/**
 * @brief 建構感測器資料封包 v2.0（Arduino → Pi）
 * @param packet 輸出：12-byte 封包
 * @param front_distance 前方距離 (cm)
 * @param right_distance 右側距離 (cm)
 * @param yaw_deg Yaw 角度 (度, 範圍 -180 ~ +180)
 * @param gyro_z Z軸角速度 (deg/s)
 * @param status 狀態旗標
 *
 * 封包格式 (12 bytes):
 *   Byte 0: Header (0xBB)
 *   Byte 1-2: Front Distance (uint16, little-endian, cm)
 *   Byte 3-4: Right Distance (uint16, little-endian, cm)
 *   Byte 5-6: Yaw Angle (int16, little-endian, 度數 × 10)
 *   Byte 7: Gyro Z (int8, deg/s, 限制 -127 ~ +127)
 *   Byte 8: Status (bit0=front_valid, bit1=right_valid, bit2=imu_valid, bit3=vacuum)
 *   Byte 9: Reserved (0x00)
 *   Byte 10: Checksum (XOR of bytes 1-9)
 *   Byte 11: Footer (0x66)
 */
void buildSensorPacketV2(uint8_t* packet, uint16_t front_distance, uint16_t right_distance,
                         float yaw_deg, float gyro_z, uint8_t status);

/**
 * @brief 建構感測器資料封包 v1.0（向下相容，無 IMU）
 * @param packet 輸出：8-byte 封包
 * @param front_distance 前方距離 (cm)
 * @param right_distance 右側距離 (cm)
 * @param status 狀態旗標
 *
 * @deprecated 請使用 buildSensorPacketV2
 */
void buildSensorPacket(uint8_t* packet, uint16_t front_distance, uint16_t right_distance, uint8_t status);

#endif // SERIAL_PROTOCOL_H
