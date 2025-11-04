# Mechatronics & Robotics Development Skill

## Description

A comprehensive skill for developing mechatronic systems, robotic platforms, and autonomous vehicles using Arduino, Raspberry Pi, and ESP platforms. Covers motor control, sensor integration, computer vision, SLAM-based navigation, and wireless communication.

## Contents

### Main Skill File
- **SKILL.md**: Primary skill documentation with overview, use cases, and technical knowledge

### Reference Documentation
- **references/motor_control.md**: L298N, L293D, Diablo motor drivers, ROS2 integration, encoder feedback
- **references/sensor_integration.md**: IR sensors, ultrasonic sensors, IMU, Bluetooth, MQTT, RF24 wireless
- **references/computer_vision_slam.md**: Raspberry Pi Camera, OpenCV, ORB-SLAM2, cross-compilation

## Key Topics Covered

### Motor Control
- L298N H-bridge driver configuration and wiring
- PWM speed control and direction control
- Encoder feedback and closed-loop control
- ROS2 serial motor communication
- Multi-motor control with Diablo (55A per channel)

### Sensor Integration
- Infrared obstacle sensors (2-30cm range)
- Ultrasonic distance sensors (HC-SR04, 2-400cm)
- IMU integration (MPU9250 9-axis)
- Event-driven GPIO programming

### Wireless Communication
- Bluetooth RFCOMM smartphone control (Pi 3/4 built-in)
- RF24 wireless sensor networks
- MQTT IoT messaging
- ESP32/ESP8266 WiFi integration

### Computer Vision
- Raspberry Pi Camera setup (V2, V3, HQ)
- Picamera2 Python library
- OpenCV installation and cross-compilation
- Real-time image processing (edge detection, color tracking, object detection)

### Autonomous Navigation
- ORB-SLAM2 visual SLAM implementation
- Camera calibration
- Real-time localization and mapping
- Loop closure and relocalization
- IMU sensor fusion

### Multi-Platform Integration
- Arduino-Raspberry Pi serial communication
- ROS2 driver nodes
- Flask web servers
- Python control applications
- Mobile app interfaces

## Example Projects

1. **L298N DC Motor Control** - Basic motor speed and direction control
2. **ROS2 Differential Drive Robot** - Encoder feedback with velocity control
3. **Bluetooth Smartphone Car** - RFCOMM wireless control via mobile app
4. **Obstacle Avoidance Robot** - Multi-sensor fusion (IR + ultrasonic)
5. **ORB-SLAM2 Navigation** - Visual SLAM with Pi Camera
6. **Diablo Multi-Motor** - High-power motor control via I2C
7. **OpenCV Vision Processing** - Real-time object detection
8. **IoT Motor Control** - MQTT cloud integration with ESP32

## Hardware Platforms

- **Arduino**: Uno, Mega, Nano (motor control, sensor reading)
- **Raspberry Pi**: 3 B+, 4, 5 (high-level control, vision, SLAM)
- **ESP32/ESP8266**: WiFi connectivity, MQTT, sensor nodes
- **Motor Drivers**: L298N, L293D, Diablo
- **Cameras**: Pi Camera V2/V3, HQ Camera
- **Sensors**: IR obstacle, HC-SR04 ultrasonic, MPU9250 IMU
- **Wireless**: RF24, Bluetooth, WiFi

## Software Stack

### Programming Languages
- Python 3 (Raspberry Pi, ESP via MicroPython)
- C++ (Arduino, ROS2)
- Bash (system configuration)

### Key Libraries
- **RPi.GPIO**: Raspberry Pi GPIO control
- **Picamera2**: Camera interface
- **OpenCV**: Computer vision
- **pyserial**: Serial communication
- **python-bluez**: Bluetooth RFCOMM
- **paho-mqtt**: MQTT IoT messaging
- **Flask**: Web servers
- **ROS2**: Robot Operating System

### Tools
- **ORB-SLAM2**: Visual SLAM
- **Pangolin**: 3D visualization
- **Eigen**: Linear algebra
- **CMake**: Build system
- **bluetoothctl**: Bluetooth pairing

## Installation & Setup

### Raspberry Pi Setup
```bash
# Update system
sudo apt update && sudo apt full-upgrade

# Enable camera and I2C
sudo raspi-config  # Interface Options

# Install Python libraries
sudo apt install python3-opencv python3-picamera2 python3-rpi.gpio \
                 python3-bluez python3-serial bluetooth blueman

# Install MQTT
pip install paho-mqtt
```

### Arduino IDE Setup
```bash
# Install Arduino IDE from arduino.cc
# Install libraries: Servo, Wire, SPI (built-in)
```

### Cross-Compilation (Optional)
See references/computer_vision_slam.md for detailed OpenCV cross-compilation instructions.

## Quick Start

### 1. Motor Control Test
Wire L298N to Arduino/Pi, run basic forward/reverse test to verify connections.

### 2. Sensor Reading
Connect IR or ultrasonic sensor, read values to verify GPIO configuration.

### 3. Bluetooth Pairing
Pair smartphone with Raspberry Pi, test RFCOMM communication.

### 4. Camera Test
Capture test image with Picamera2, verify camera operation.

### 5. Integration
Combine sensors, motors, and control logic for autonomous operation.

## Troubleshooting

### Common Issues
- **Motors not spinning**: Check common ground, power supply, jumper settings
- **Sensor not reading**: Verify GPIO pins, check voltage levels, test with multimeter
- **Bluetooth pairing fails**: Restart bluetooth service, remove old pairings
- **Camera not detected**: Enable in raspi-config, check ribbon cable connection
- **ORB-SLAM2 initialization fails**: Ensure textured environment, adequate lighting

See individual reference files for detailed troubleshooting guides.

## Resources

### GitHub Repositories
- [Motor-and-Sensor-Control](https://github.com/GiorgioCitterio/Motor-and-Sensor-Control-on-Arduino-Raspberry-Pi-and-ESP)
- [serial_motor_demo (ROS2)](https://github.com/joshnewans/serial_motor_demo)
- [Diablo Motor Controller](https://github.com/piborg/diablo)
- [ORB-SLAM2 for Raspberry Pi](https://github.com/ss26/ORB-SLAM)

### Documentation
- [Arduino Forum - L298N Threads](https://forum.arduino.cc)
- [OpenCV Cross-Compilation](https://docs.opencv.org/4.x/d3/dd9/tutorial_crosscompile_with_multiarch.html)
- [Circuit Digest - Raspberry Pi Projects](https://circuitdigest.com)
- [Newbiely - Raspberry Pi Tutorials](https://newbiely.com)

### Communities
- Arduino Forum: https://forum.arduino.cc
- Raspberry Pi Forums: https://forums.raspberrypi.com
- ROS Discourse: https://discourse.ros.org

## License

This skill compiles knowledge from various open-source projects, official documentation, and community resources. Please respect the licenses of individual projects referenced within.

## Version

1.0.0 - Initial release covering motor control, sensors, vision, and SLAM for mechatronics development.
