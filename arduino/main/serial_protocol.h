/*
 * serial_protocol.h - Serial 通訊協定模組
 * 版本: 1.0
 * 日期: 2025-10-31
 *
 * 參考: 04_ICD_介面規格.md - 2. Serial 通訊介面
 */

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

// 封包格式常數
#define PACKET_SIZE 8
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
 * @return true = 封包有效，false = 封包無效
 */
bool parseMotorPacket(uint8_t* packet, int16_t& left_pwm, int16_t& right_pwm, bool& vacuum);

/**
 * @brief 建構感測器資料封包（Arduino → Pi）
 * @param packet 輸出：8-byte 封包
 * @param left_distance 左側距離 (cm)
 * @param right_distance 右側距離 (cm)
 * @param status 狀態旗標
 */
void buildSensorPacket(uint8_t* packet, uint16_t left_distance, uint16_t right_distance, uint8_t status);

#endif // SERIAL_PROTOCOL_H
