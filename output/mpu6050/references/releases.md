# Releases

Version history for this repository (20 releases).

## v1.4.4: v1.4.4
**Published:** 2025-06-24

- Resolution calculation fix when setting accel/gyro range

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.4.4)

---

## v1.4.3: v1.4.3
**Published:** 2025-02-27

- Resolution calculation fix when setting accel/gyro range

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.4.3)

---

## v1.4.2: v1.4.2
**Published:** 2025-02-27

- MPU 92.65 can now be detected by mpu.testConnection() by @Sinamajidi

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.4.2)

---

## v1.4.1: v1.4.1
**Published:** 2024-10-08

- [fixed initialize switch statements](https://github.com/ElectronicCats/mpu6050/commit/c247d358dc6165059eff7fbae07baa676db1c69f) thanks [stefcarpi](https://github.com/ElectronicCats/mpu6050/commits?author=stefcarpi)

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.4.1)

---

## v1.4.0: v1.4.0
**Published:** 2024-10-03

- New examples
- Refactor examples

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.4.0)

---

## v1.3.1: Fix debug compilation bugs
**Published:** 2024-03-19

Bug fixes:

- Print not declared variables
- Use `printf` as Arduino `Serial.print`

Refactors:

- Improve DEBUG macro validation

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.3.1)

---

## v1.3.0: v1.3.0
**Published:** 2024-01-18

- Added sensitivity variation as a new Constructor
- Added get method to retrieve resolution
- Renamed dmpLinearAccelInWorld() method as dmpConvertToWorldFrame() since it needs to be used with gyro data as well
- Added example to show how to extract data required by ROS IMU message
- One doubt is whether 1G acceleration is 8192 or 16384 (At 2G resolution) because MPU datasheet says full range is 32768. For now using 8192 as it is hardcoded as gravity. Similarly using 16384 as max Gyro resolution. Please rectify this if wrong.
- Gravity value needs to be updated to use the new resolution variable to be completely compatible with resolution variation

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.3.0)

---

## v1.2.0: v1.2.0
**Published:** 2023-12-19

- Update library from i2cdev
- add Support multiple Wire objects in MPU6050 device code

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.2.0)

---

## v1.0.2: v1.0.2
**Published:** 2023-12-13

Set fifo rate and MPU address

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.0.2)

---

## v1.0.1: v1.0.1
**Published:** 2023-12-01

Remove debug messages

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.0.1)

---

## v1.0.0: v1.0.0
**Published:** 2023-06-06

- Update examples
- Update README
- New Wiki!!

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v1.0.0)

---

## v0.6.0: v0.6.0
**Published:** 2022-09-09

- adding an IMU namespace
- Update the I2C structure

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v0.6.0)

---

## v0.5.0: v0.5.0
**Published:** 2021-10-11

- Adding support for SAM architecture
- Fixing devicedId value 
Thanks @sylvaneau

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v0.5.0)

---

## v0.4.0: v0.4.0
**Published:** 2021-09-12

- Improved the performance of normalizing quaternions and vectors, thanks @quesswho
- Enhanced device ID checking to allow usage of certain available clones, thanks @holzachr
- And more updates from jrowberg/i2cdevlib

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v0.4.0)

---

## v0.3.0: 
**Published:** 2021-04-25

- Add support Arduino Nano BLE Sense
- Update examples

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v0.3.0)

---

## v0.2.1: 
**Published:** 2020-08-19



[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v0.2.1)

---

## v0.2.0: Added Support for ESP32
**Published:** 2020-08-09

- Added Support for ESP32
- Example for ESP8266 and ESP32

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v0.2.0)

---

## v0.1.0: v0.1.0
**Published:** 2020-05-27

- function replacement _BV for ARM M0 in examples
- add Action Github for test

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v0.1.0)

---

## v0.0.2: 
**Published:** 2019-09-05

The New:

- Bug Fix writeWords() skips every other word
- add support for dtostrf in samd
- Update MPU6050_6Axis_MotionApps20.h
- Added Auto Calibration routine
- Faster Better Tuning +plus
- Update i2cdevlib
- and other Corrections

Thanks to @ZHomeSlice and jrowberg of i2cdevlib

[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v0.0.2)

---

## v0.0.1: Ready!
**Published:** 2019-02-03



[View on GitHub](https://github.com/ElectronicCats/mpu6050/releases/tag/v0.0.1)

---

