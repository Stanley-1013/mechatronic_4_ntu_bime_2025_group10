/*
 * mpu6050_sensor.cpp - MPU6050 IMU 感測器實作
 * 版本: 1.0
 * 日期: 2025-11-29
 *
 * 使用簡單的陀螺儀積分計算 Yaw 角度。
 * 適用於短時間 (幾分鐘內) 的轉彎控制。
 *
 * 參考: ChatGPT 提供的 MPU6050 讀取範例
 */

#include "mpu6050_sensor.h"
#include "config.h"

// 感測器靈敏度常數
// ±250 deg/s -> 131 LSB/(deg/s)
// ±500 deg/s -> 65.5 LSB/(deg/s)
// ±2g -> 16384 LSB/g
// ±4g -> 8192 LSB/g
static const float GYRO_SENSITIVITY = 131.0f;   // ±250 deg/s
static const float ACCEL_SENSITIVITY = 16384.0f; // ±2g

MPU6050Sensor::MPU6050Sensor()
    : _accelX(0), _accelY(0), _accelZ(0)
    , _gyroX(0), _gyroY(0), _gyroZ(0)
    , _yaw(0)
    , _gyroOffsetX(0), _gyroOffsetY(0), _gyroOffsetZ(0)
    , _lastUpdateTime(0)
    , _initialized(false)
{
}

bool MPU6050Sensor::begin() {
    Wire.begin();

    // 檢查 MPU6050 是否存在
    Wire.beginTransmission(MPU_ADDR);
    uint8_t error = Wire.endTransmission();

    if (error != 0) {
        DEBUG_PRINTLN(F("[MPU6050] Device not found!"));
        return false;
    }

    // 喚醒 MPU6050 (寫 PWR_MGMT_1 = 0)
    writeRegister(REG_PWR_MGMT_1, 0x00);
    delay(100);

    // 設定陀螺儀範圍: ±250 deg/s (最高精度)
    writeRegister(REG_GYRO_CONFIG, 0x00);

    // 設定加速度計範圍: ±2g (最高精度)
    writeRegister(REG_ACCEL_CONFIG, 0x00);

    delay(100);

    _lastUpdateTime = micros();
    _initialized = true;

    DEBUG_PRINTLN(F("[MPU6050] Initialized successfully"));
    return true;
}

void MPU6050Sensor::calibrate(int samples) {
    if (!_initialized) return;

    DEBUG_PRINTLN(F("[MPU6050] Calibrating... Keep sensor still!"));

    float sumGx = 0, sumGy = 0, sumGz = 0;

    for (int i = 0; i < samples; i++) {
        readRawData();
        sumGx += _gyroX;
        sumGy += _gyroY;
        sumGz += _gyroZ;
        delay(2);  // 約 500Hz 取樣
    }

    _gyroOffsetX = sumGx / samples;
    _gyroOffsetY = sumGy / samples;
    _gyroOffsetZ = sumGz / samples;

    DEBUG_PRINT(F("[MPU6050] Calibration done. Offsets: "));
    DEBUG_PRINT(_gyroOffsetX);
    DEBUG_PRINT(F(", "));
    DEBUG_PRINT(_gyroOffsetY);
    DEBUG_PRINT(F(", "));
    DEBUG_PRINTLN(_gyroOffsetZ);
}

void MPU6050Sensor::update() {
    if (!_initialized) return;

    unsigned long currentTime = micros();
    float dt = (currentTime - _lastUpdateTime) / 1000000.0f;  // 轉換為秒
    _lastUpdateTime = currentTime;

    // 避免異常的 dt 值
    if (dt <= 0 || dt > 0.5f) {
        return;
    }

    // 讀取原始資料
    readRawData();

    // 扣除校準偏移量
    _gyroX -= _gyroOffsetX;
    _gyroY -= _gyroOffsetY;
    _gyroZ -= _gyroOffsetZ;

    // 積分計算 Yaw 角度
    // 注意: 車體平放時，Z 軸垂直向上，繞 Z 軸旋轉就是 Yaw
    _yaw += _gyroZ * dt;

    // 將 Yaw 限制在 -180 ~ +180 範圍
    while (_yaw > 180.0f) _yaw -= 360.0f;
    while (_yaw < -180.0f) _yaw += 360.0f;
}

void MPU6050Sensor::readRawData() {
    // 從 0x3B (ACCEL_XOUT_H) 開始連續讀取 14 bytes
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);  // repeated start

    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

    // 讀取加速度 (16-bit, big-endian)
    int16_t rawAccelX = Wire.read() << 8 | Wire.read();
    int16_t rawAccelY = Wire.read() << 8 | Wire.read();
    int16_t rawAccelZ = Wire.read() << 8 | Wire.read();

    // 跳過溫度
    Wire.read();
    Wire.read();

    // 讀取陀螺儀 (16-bit, big-endian)
    int16_t rawGyroX = Wire.read() << 8 | Wire.read();
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

    // 轉換為物理單位
    _accelX = rawAccelX / ACCEL_SENSITIVITY;
    _accelY = rawAccelY / ACCEL_SENSITIVITY;
    _accelZ = rawAccelZ / ACCEL_SENSITIVITY;

    _gyroX = rawGyroX / GYRO_SENSITIVITY;
    _gyroY = rawGyroY / GYRO_SENSITIVITY;
    _gyroZ = rawGyroZ / GYRO_SENSITIVITY;
}

void MPU6050Sensor::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

uint8_t MPU6050Sensor::readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
    return Wire.read();
}
