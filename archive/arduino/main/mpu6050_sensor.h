/*
 * mpu6050_sensor.h - MPU6050 IMU 感測器模組
 * 版本: 1.0
 * 日期: 2025-11-29
 *
 * 功能:
 * - 讀取 MPU6050 加速度計與陀螺儀原始資料
 * - 使用互補濾波器計算 Yaw 角度
 * - 提供角度重置功能 (用於轉彎控制)
 *
 * 接線 (Arduino Uno):
 *   MPU6050 VCC  -> Arduino 5V
 *   MPU6050 GND  -> Arduino GND
 *   MPU6050 SCL  -> Arduino A5
 *   MPU6050 SDA  -> Arduino A4
 *   MPU6050 INT  -> Arduino D2 (可選，DMP 用)
 */

#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050Sensor {
public:
    // 建構子
    MPU6050Sensor();

    /**
     * @brief 初始化 MPU6050
     * @return true = 成功, false = 失敗
     */
    bool begin();

    /**
     * @brief 更新感測器讀數 (需在 loop 中呼叫)
     *
     * 此函數會讀取陀螺儀資料並透過積分計算 Yaw 角度。
     * 建議呼叫頻率: 50-100 Hz
     */
    void update();

    /**
     * @brief 取得當前 Yaw 角度
     * @return Yaw 角度 (度), 範圍 -180 ~ +180
     *         正值 = 順時針旋轉, 負值 = 逆時針旋轉
     */
    float getYaw() const { return _yaw; }

    /**
     * @brief 取得 Z 軸角速度
     * @return 角速度 (deg/s), 正值 = 順時針
     */
    float getGyroZ() const { return _gyroZ; }

    /**
     * @brief 重置 Yaw 角度為 0
     *
     * 用於開始轉彎前記錄起始角度
     */
    void resetYaw() { _yaw = 0.0f; }

    /**
     * @brief 設定 Yaw 角度為指定值
     * @param yaw 角度 (度)
     */
    void setYaw(float yaw) { _yaw = yaw; }

    /**
     * @brief 取得加速度 X (g)
     */
    float getAccelX() const { return _accelX; }

    /**
     * @brief 取得加速度 Y (g)
     */
    float getAccelY() const { return _accelY; }

    /**
     * @brief 取得加速度 Z (g)
     */
    float getAccelZ() const { return _accelZ; }

    /**
     * @brief 校準陀螺儀零偏
     *
     * 呼叫此函數時，MPU6050 應該靜止不動。
     * 會取樣多次計算平均零偏值。
     *
     * @param samples 取樣次數 (預設 500)
     */
    void calibrate(int samples = 500);

    /**
     * @brief 檢查 MPU6050 是否已初始化
     */
    bool isInitialized() const { return _initialized; }

private:
    // MPU6050 I2C 位址
    static const uint8_t MPU_ADDR = 0x68;

    // MPU6050 暫存器位址
    static const uint8_t REG_PWR_MGMT_1 = 0x6B;
    static const uint8_t REG_GYRO_CONFIG = 0x1B;
    static const uint8_t REG_ACCEL_CONFIG = 0x1C;
    static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

    // 感測器資料
    float _accelX, _accelY, _accelZ;  // 加速度 (g)
    float _gyroX, _gyroY, _gyroZ;     // 角速度 (deg/s)
    float _yaw;                        // 航向角 (度)

    // 校準偏移量
    float _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ;

    // 時間追蹤 (用於積分)
    unsigned long _lastUpdateTime;

    // 狀態
    bool _initialized;

    // 內部函數
    void readRawData();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

#endif // MPU6050_SENSOR_H
