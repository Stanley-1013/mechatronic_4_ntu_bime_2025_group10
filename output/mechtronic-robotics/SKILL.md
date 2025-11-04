---
name: mechtronic-robotics
description: Comprehensive mechatronics development for Arduino, Raspberry Pi, and ESP platforms. Covers motor control (L298N, ROS2, encoders), sensors (IR, ultrasonic, IMU), computer vision (OpenCV, Picamera2), SLAM navigation (ORB-SLAM2), and wireless communication (Bluetooth, MQTT, RF24). Use when building robots, autonomous vehicles, motor control systems, or integrating sensors and vision.
---

# Mechatronics & Robotics Development Skill

## Overview

This skill provides comprehensive knowledge for developing mechatronic systems, robotic platforms, and autonomous vehicles using Arduino, Raspberry Pi, and ESP platforms. It covers motor control, sensor integration, computer vision, and SLAM-based autonomous navigation.

## Core Capabilities

### 1. Motor Control Systems
- **L298N H-Bridge Driver**: DC motor control with PWM speed regulation and direction control
- **Multi-Platform Support**: Arduino, Raspberry Pi, ESP8266/ESP32
- **Encoder Feedback**: Closed-loop velocity control with encoder integration
- **ROS2 Integration**: Serial motor communication for differential-drive robots
- **Python Libraries**: Diablo controller for I2C/PWM multi-motor control (up to 55A per channel)

### 2. Sensor Integration
- **Obstacle Detection**: Infrared (IR) sensors, ultrasonic sensors (HC-SR04)
- **Multi-Interface Support**: I2C, SPI, Serial, GPIO
- **IMU Integration**: MPU9250 and similar inertial measurement units
- **Wireless Communication**: RF24 modules, MQTT, Bluetooth (built-in on Pi 3/4)
- **Data Acquisition**: Real-time sensor reading and processing with event-driven callbacks

### 3. Computer Vision & Image Processing
- **Raspberry Pi Camera**: Picamera2 library for image capture and video recording
- **OpenCV Integration**: Cross-compiled OpenCV 4.x with Python bindings for ARM
- **Real-Time Processing**: Optimized for embedded ARM platforms (armv7/aarch64)
- **Multi-Architecture Support**: Cross-compilation from x86-64 to ARM targets

### 4. Autonomous Navigation
- **ORB-SLAM2**: Monocular visual SLAM for real-time localization and mapping
- **3D Reconstruction**: Environmental geometry mapping using ORB features
- **Loop Closure**: DBoW2-based place recognition
- **Sensor Fusion**: Camera and IMU integration for robust navigation

### 5. Communication & Networking
- **Serial Communication**: Arduino-Raspberry Pi interfacing via UART
- **Bluetooth Control**: RFCOMM protocol for smartphone remote control (built-in on Pi 3/4)
- **Web Interfaces**: Flask-based control servers
- **IoT Protocols**: MQTT messaging for cloud connectivity
- **Voice Control**: Alexa integration for voice-commanded robotics
- **GUI Development**: tkinter-based control interfaces
- **Mobile Apps**: BlueTerm and similar Android/iOS apps for wireless robot control

## When to Use This Skill

### Activate this skill when working on:

1. **Motor Control Projects**
   - Configuring L298N or similar H-bridge motor drivers
   - Implementing PWM speed control for DC motors
   - Setting up encoder feedback systems
   - Troubleshooting motor wiring and power supply issues

2. **Robot Platform Development**
   - Building differential-drive robots
   - Integrating Arduino with Raspberry Pi
   - Developing ROS2 motor control nodes
   - Implementing closed-loop velocity control

3. **Computer Vision Applications**
   - Setting up Raspberry Pi camera modules
   - Cross-compiling OpenCV for ARM platforms
   - Implementing real-time image processing
   - Developing vision-based control systems

4. **Autonomous Navigation Systems**
   - Implementing SLAM algorithms
   - Setting up ORB-SLAM2 on Raspberry Pi
   - Integrating camera and IMU sensors
   - Developing path planning and localization

5. **Multi-Platform Integration**
   - Connecting Arduino, Raspberry Pi, and ESP devices
   - Implementing serial communication protocols
   - Setting up wireless sensor networks
   - Developing distributed control systems

6. **Sensor Integration**
   - Integrating IR obstacle sensors and ultrasonic distance sensors
   - Reading I2C/SPI sensors
   - Implementing data fusion algorithms
   - Setting up RF24 wireless modules
   - Developing MQTT-based IoT systems

7. **Wireless Robot Control**
   - Setting up Bluetooth smartphone control for robots
   - Implementing RFCOMM communication protocol
   - Creating mobile app interfaces for robot control
   - Developing voice-controlled robotics

## Key Technical Knowledge

### L298N Motor Driver Configuration

**Pin Connections:**
- 12V pin → Battery positive terminal (7.4-12V recommended)
- GND → Common ground (Arduino, driver, battery)
- IN1/IN2/IN3/IN4 → Arduino digital pins (direction control)
- ENA/ENB → Arduino PWM pins (speed control via analogWrite())

**Jumper Settings:**
- **InA/InB Jumpers**: Remove to enable PWM speed control from Arduino
- **Regulator Jumper**:
  - With jumper (7.4-12V input): Onboard regulator provides 5V to logic
  - Without jumper (4.5-6V input): External 5V supply required for logic

**Critical Guidelines:**
- Always connect common ground between all components
- Power Arduino and motors from separate power sources
- Use appropriate voltage levels (motors: 7-12V, logic: 5V)
- Monitor heat dissipation on the L298N module

### ROS2 Motor Control Architecture

**Driver Node Configuration:**
```python
ros2 run serial_motor_demo driver --ros-args \
  -p encoder_cpr:=3440 \          # Encoder counts per revolution
  -p loop_rate:=30 \               # Update frequency (Hz)
  -p serial_port:=/dev/ttyUSB0 \   # Serial connection
  -p baud_rate:=57600              # Communication speed
```

**Topic Interface:**
- **Subscribe**: `/motor_command` (MotorCommand messages, rad/sec)
- **Publish**: `/motor_vels` (velocity feedback), `/encoder_vals` (encoder data)

**Control Modes:**
- Raw PWM mode: Direct control (-255 to 255)
- Closed-loop mode: Velocity setpoint control with encoder feedback

### Diablo Motor Controller (Python/Raspberry Pi)

**Installation:**
```bash
git clone https://github.com/piborg/diablo
cd diablo
bash install.sh
```

**Key Features:**
- Dual-channel control (7-36V, 55A per channel)
- I2C communication interface
- Built-in protection (overheat, undervoltage, short circuit)
- Supports DC motors and stepper motors

**Example Programs:**
- `diabloGui.py`: GUI-based slider control
- `diabloJoystick.py`: PS4 controller integration
- `diabloSequence.py`: Multi-motor speed sequences
- `diabloStepper.py`: 4-wire stepper motor control

### ORB-SLAM2 Setup (Raspberry Pi 3 B+)

**Hardware Requirements:**
- Raspberry Pi 3 B+ or better
- Pi Camera V2
- MPU9250 IMU sensor
- Adequate power supply for robot chassis

**Essential Dependencies:**
- OpenCV 3.4.6 (computer vision processing)
- Pangolin (visualization framework)
- Eigen 3.2.10 (linear algebra)
- DBoW2 (loop closure detection, included in ORB-SLAM2)
- g2o (graph optimization, included in ORB-SLAM2)

**Installation Process:**
1. Set up Python 3 virtual environment
2. Compile OpenCV with Python bindings
3. Build ORB-SLAM2 library and examples
4. Validate with public datasets (TUM, KITTI, EUROC)
5. Configure for Pi Camera V2

**Validation Datasets:**
- TUM RGB-D Dataset
- KITTI Vision Benchmark
- EUROC MAV Dataset

### OpenCV Cross-Compilation for Raspberry Pi

**System Requirements:**
- Host: Ubuntu 23.04+ x86-64
- OpenCV 4.8.0+
- Target: Raspberry Pi (armv7/aarch64)

**MultiArch Setup:**
```bash
# Add foreign architecture
sudo dpkg --add-architecture arm64  # or armhf for armv7

# Install cross-compilation tools
sudo apt install crossbuild-essential-arm64 cmake ninja-build pkgconf

# Install target-architecture dependencies
sudo apt install libavcodec-dev:arm64 libavformat-dev:arm64 \
                 libfreetype-dev:arm64 libharfbuzz-dev:arm64
```

**Build Configuration:**
- Use appropriate CMake toolchain file for target architecture
- Set PKG_CONFIG_PATH to target architecture library directories
- Configure Python bindings with target numpy paths
- Ensure host and target Ubuntu versions match

**Deployment:**
- Archive compiled libraries
- Transfer to Raspberry Pi
- Verify dependencies with `ldd`
- Test Python OpenCV import

### Raspberry Pi Camera (Picamera2)

Note: The official Raspberry Pi documentation was inaccessible during scraping, but the Picamera2 library provides the following capabilities:

**Core Functions:**
- Image capture (JPEG, PNG, RAW formats)
- Video recording (H.264, MJPEG)
- Real-time preview
- Camera configuration (resolution, framerate, exposure)
- Integration with NumPy arrays for OpenCV

**Installation:**
```bash
sudo apt install python3-picamera2
```

**Basic Usage Pattern:**
```python
from picamera2 import Picamera2

picam2 = Picamera2()
config = picam2.create_still_configuration()
picam2.configure(config)
picam2.start()

# Capture image
picam2.capture_file("image.jpg")

# Get NumPy array for OpenCV
image = picam2.capture_array()
```

### Multi-Platform Motor & Sensor Control

**Supported Platforms:**
- Arduino boards (Uno, Mega, Nano, etc.)
- Raspberry Pi (all models with GPIO)
- ESP8266 and ESP32 microcontrollers

**Communication Methods:**
- Serial (UART) for Arduino-PC/Raspberry Pi
- I2C for multi-device communication
- SPI for high-speed sensors
- GPIO for direct digital/analog I/O
- RF24 for wireless sensor networks

**Software Stack:**
- **Python Libraries**: pyserial, Flask, paho-mqtt, nrf24, pigpiod
- **Arduino Libraries**: Standard servo, motor, and sensor libraries
- **Web Interfaces**: Flask-based control servers
- **IoT Integration**: MQTT for cloud connectivity

**Development Progression:**
1. Serial communication fundamentals
2. Python control applications
3. GUI development (tkinter)
4. Web server implementation (Flask)
5. Data persistence (JSON storage)
6. Wireless communication (RF24)
7. IoT messaging (MQTT)
8. Advanced interfaces (BLE, Alexa)

## Example Projects & Use Cases

### 1. L298N DC Motor Control (Arduino)

**Scenario**: Control two DC motors with speed and direction.

**Wiring:**
- L298N 12V → 9V battery positive
- L298N GND → Common ground
- L298N IN1 → Arduino Pin 8 (Motor A direction)
- L298N IN2 → Arduino Pin 9 (Motor A direction)
- L298N ENA → Arduino Pin 10 (Motor A PWM speed)
- L298N IN3 → Arduino Pin 11 (Motor B direction)
- L298N IN4 → Arduino Pin 12 (Motor B direction)
- L298N ENB → Arduino Pin 13 (Motor B PWM speed)

**Arduino Code Pattern:**
```cpp
// Motor A
digitalWrite(8, HIGH);  // Direction
digitalWrite(9, LOW);
analogWrite(10, 200);   // Speed (0-255)

// Motor B
digitalWrite(11, HIGH);
digitalWrite(12, LOW);
analogWrite(13, 150);
```

### 2. ROS2 Differential Drive Robot

**Scenario**: Robot with encoders sending velocity commands from ROS2.

**Hardware:**
- Arduino with motor driver and encoders
- Raspberry Pi 4 running ROS2
- USB serial connection

**Setup:**
1. Configure Arduino with encoder counting and serial protocol
2. Launch ROS2 driver node with encoder CPR and serial port
3. Subscribe to motor commands, publish velocities and encoder values
4. Implement closed-loop velocity control

### 3. Bluetooth Smartphone-Controlled Car (Raspberry Pi)

**Scenario**: Remote-controlled car using smartphone via Bluetooth.

**Hardware:**
- Raspberry Pi 3 or 4 (built-in Bluetooth)
- L293D or L298N motor driver
- 2 DC motors
- Mobile power bank
- Android smartphone with BlueTerm app

**Wiring (L293D):**
- GPIO 23, 24 → Motor A (IN1, IN2)
- GPIO 25 → Motor A Enable (PWM)
- GPIO 17, 27 → Motor B (IN3, IN4)
- GPIO 22 → Motor B Enable (PWM)

**Software Setup:**
```bash
sudo apt-get install bluetooth blueman bluez python3-bluez
sudo bluetoothctl
# Enable power, pairing, discoverable
```

**Control Commands (BlueTerm app):**
- F = Forward
- B = Backward
- L = Turn Left
- R = Turn Right
- S = Stop
- Q = Quit

**Features:**
- RFCOMM Bluetooth protocol (emulates serial)
- No external Bluetooth dongle required
- Voice command support via phone keyboard
- Low latency control

**See references/sensor_integration.md for complete Python code.**

### 4. Obstacle Avoidance Robot (IR + Ultrasonic)

**Scenario**: Autonomous robot that stops or turns when detecting obstacles.

**Hardware:**
- Raspberry Pi
- IR obstacle sensor (2-30cm range)
- HC-SR04 ultrasonic sensor (2-400cm range)
- Motor driver + DC motors

**Sensor Configuration:**
- IR OUT → GPIO 17 (digital, LOW = obstacle detected)
- Ultrasonic TRIG → GPIO 23
- Ultrasonic ECHO → GPIO 24 (via voltage divider!)

**Logic:**
- IR sensor: Close-range emergency stop (< 30cm)
- Ultrasonic: Medium-range slow down (30-50cm)
- Multi-sensor fusion for robust detection

**Features:**
- Event-driven callbacks for fast response
- No polling delays
- Integrates with motor control for autonomous navigation

**See references/sensor_integration.md for complete implementation.**

### 5. ORB-SLAM2 Navigation Robot

**Scenario**: Autonomous mobile robot with visual SLAM.

**Components:**
- Raspberry Pi 3 B+ with Pi Camera V2
- MPU9250 IMU for sensor fusion
- Motor driver and differential drive chassis

**Implementation:**
1. Install OpenCV 3.4.6 in virtual environment
2. Compile ORB-SLAM2 with dependencies
3. Calibrate Pi Camera and configure SLAM parameters
4. Run SLAM node for real-time localization and mapping
5. Integrate odometry and IMU data
6. Implement path planning based on SLAM output

### 6. Multi-Motor Control with Diablo (Raspberry Pi)

**Scenario**: Control multiple high-power motors for robotic arm or heavy platform.

**Setup:**
```bash
# Install Diablo library
git clone https://github.com/piborg/diablo
cd diablo
bash install.sh

# Run GUI for testing
python3 diabloGui.py
```

**Features:**
- I2C communication with Raspberry Pi
- Support for 7-36V motors
- Current limiting up to 55A per channel
- Built-in safety features

### 7. OpenCV Vision Processing on Raspberry Pi

**Scenario**: Real-time object detection on Raspberry Pi 4.

**Cross-Compilation Workflow:**
1. Set up MultiArch on Ubuntu 23.04+ host
2. Install ARM architecture dependencies
3. Cross-compile OpenCV with Python bindings
4. Deploy to Raspberry Pi
5. Capture frames with Picamera2
6. Process with OpenCV (contour detection, face recognition, etc.)

### 8. IoT Motor Control System

**Scenario**: Web-based motor control with MQTT cloud integration.

**Architecture:**
- ESP32 with motor driver
- MQTT broker (local or cloud)
- Flask web server for user interface
- Python MQTT client for command relay

**Communication Flow:**
1. User sends command via web interface
2. Flask server publishes to MQTT topic
3. ESP32 subscribes and executes motor control
4. Sensor data published back to MQTT
5. Web interface displays real-time status

## Common Troubleshooting

### Motor Control Issues

**Problem**: Motors not spinning or erratic behavior
**Solutions**:
- Verify common ground connection between Arduino, driver, and battery
- Check power supply voltage (7-12V for motors, 5V for logic)
- Confirm jumper settings on L298N (remove InA/InB for PWM control)
- Test with simple code (full speed forward) before adding complexity
- Monitor heat on motor driver; add heatsink if necessary

**Problem**: L298N overheating
**Solutions**:
- Reduce motor load or duty cycle
- Add heatsink to L298N chip
- Ensure adequate ventilation
- Consider upgrading to higher-capacity driver

### ROS2 Serial Communication Issues

**Problem**: Driver node cannot connect to Arduino
**Solutions**:
- Check serial port permissions: `sudo usermod -a -G dialout $USER`
- Verify correct port: `ls /dev/ttyUSB* /dev/ttyACM*`
- Confirm baud rate matches Arduino sketch
- Test serial connection with `screen /dev/ttyUSB0 57600`

**Problem**: Encoder values not updating
**Solutions**:
- Verify encoder wiring and power supply
- Check encoder CPR parameter matches actual encoder
- Test encoder independently before ROS2 integration
- Confirm Arduino code is reading and transmitting encoder data

### ORB-SLAM2 Issues

**Problem**: SLAM initialization fails
**Solutions**:
- Ensure sufficient visual features in environment (textured surfaces)
- Check camera calibration parameters
- Verify camera is functional with test capture
- Provide adequate lighting conditions
- Move camera slowly during initialization

**Problem**: Tracking lost during operation
**Solutions**:
- Avoid rapid camera movements
- Ensure consistent lighting (avoid backlighting)
- Increase feature extraction parameters
- Check for motion blur; reduce speed if necessary

### OpenCV Cross-Compilation Issues

**Problem**: Compilation fails with missing dependencies
**Solutions**:
- Ensure host and target Ubuntu versions match exactly
- Install all foreign-architecture dev packages: `libXXX-dev:arm64`
- Set PKG_CONFIG_PATH correctly for target architecture
- Use verbose CMake output to identify missing libraries

**Problem**: Compiled OpenCV crashes on Raspberry Pi
**Solutions**:
- Verify all runtime dependencies with `ldd libopencv_core.so`
- Check architecture match (armv7 vs aarch64)
- Ensure Python versions match between build and target
- Test with minimal example before complex applications

### Camera Issues

**Problem**: Pi Camera not detected
**Solutions**:
- Enable camera in `raspi-config`
- Check ribbon cable connection (blue side toward PCB)
- Verify camera is not faulty with `libcamera-hello`
- Update Raspberry Pi firmware: `sudo apt update && sudo apt full-upgrade`

## Best Practices

### Power Management
- Always use separate power supplies for logic (5V) and motors (7-36V)
- Connect all grounds together (common ground)
- Use capacitors across motor terminals to reduce noise
- Size power supply for peak current draw with safety margin

### Code Organization
- Separate motor control logic from sensor reading
- Implement non-blocking code (avoid delay() in Arduino)
- Use state machines for complex behaviors
- Add safety timeouts and emergency stop functionality

### Testing & Debugging
- Test components individually before integration
- Use serial monitoring for debugging embedded systems
- Log data to files for post-analysis
- Implement verbose debug modes that can be disabled

### Real-Time Performance
- Optimize loop frequencies for control stability (30-100 Hz typical)
- Profile code to identify bottlenecks
- Use multi-threading on Raspberry Pi for parallel tasks
- Consider RTOS for hard real-time requirements

### Safety Considerations
- Implement emergency stop mechanisms
- Add current limiting and thermal protection
- Use proper wire gauges for high-current applications
- Secure all mechanical components to prevent accidents
- Test in safe environment before full deployment

## Integration Patterns

### Arduino-Raspberry Pi Serial Interface

**Pattern**: Arduino handles real-time motor/sensor control, Raspberry Pi handles high-level logic.

**Protocol Design**:
- Text-based commands (e.g., "M1:100,M2:-50\n")
- Binary protocol for higher throughput
- Checksum or CRC for error detection
- Regular heartbeat messages for connection monitoring

### Multi-Device I2C Network

**Pattern**: Single master (Raspberry Pi) controlling multiple Arduino/ESP slaves.

**Implementation**:
- Assign unique I2C addresses to each slave (0x08-0x77)
- Use Wire library on Arduino, smbus2 on Raspberry Pi
- Implement request-response or register-based protocol
- Add pull-up resistors (4.7kΩ typical) on SDA and SCL lines

### Wireless Sensor Network

**Pattern**: Multiple sensor nodes (ESP32/RF24) reporting to central controller.

**Architecture**:
- Star topology with central hub
- Time-division multiplexing for multiple nodes
- Sleep modes for battery-powered nodes
- Automatic reconnection on communication failure

### Vision-Based Control Loop

**Pattern**: Camera feedback controls motor actuation.

**Pipeline**:
1. Capture frame (Picamera2)
2. Process image (OpenCV)
3. Extract control parameters (position, angle, etc.)
4. Calculate motor commands (PID or other controller)
5. Send commands via serial/I2C
6. Repeat at control loop frequency (10-30 Hz typical)

## Additional Resources

### GitHub Repositories
- Motor-and-Sensor-Control: https://github.com/GiorgioCitterio/Motor-and-Sensor-Control-on-Arduino-Raspberry-Pi-and-ESP
- serial_motor_demo (ROS2): https://github.com/joshnewans/serial_motor_demo
- Diablo Motor Controller: https://github.com/piborg/diablo
- ORB-SLAM2 Raspberry Pi: https://github.com/ss26/ORB-SLAM

### Documentation
- Arduino Forum L298N Thread: https://forum.arduino.cc/t/l298n-controlling-dc-motors/394751
- L298N Jumper Configuration: https://forum.arduino.cc/t/l298n-h-bridge-jumpers/965521
- OpenCV Cross-Compilation: https://docs.opencv.org/4.x/d3/dd9/tutorial_crosscompile_with_multiarch.html

### Communities & Support
- Arduino Forum: https://forum.arduino.cc
- Raspberry Pi Forums: https://forums.raspberrypi.com
- ROS Discourse: https://discourse.ros.org
- PiBorg Forum: https://www.piborg.org/forum

---

*This skill combines knowledge from official documentation, community forums, and open-source projects to provide comprehensive mechatronics development guidance.*
