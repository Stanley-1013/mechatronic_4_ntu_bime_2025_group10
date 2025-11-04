# Sensor Integration Reference

## Obstacle Detection Sensors

### Infrared (IR) Obstacle Sensor

#### Specifications
- Detection range: 2-30 cm (adjustable via potentiometer)
- Operating voltage: 3.3-5V
- Output: Digital (HIGH/LOW)
- Detection principle: IR reflection from objects

#### Pin Configuration
- **VCC**: Power supply (3.3V or 5V)
- **GND**: Ground
- **OUT**: Digital output signal

#### Output Logic
- **LOW**: Obstacle detected (IR light reflected back)
- **HIGH**: No obstacle (IR light not reflected)

#### Raspberry Pi Wiring

```
IR Sensor          Raspberry Pi
---------          ------------
VCC          →     5V (Pin 2 or 4)
GND          →     GND (Pin 6, 9, 14, 20, 25, 30, 34, 39)
OUT          →     GPIO 17 (or any GPIO pin)
```

**Note**: Use Screw Terminal Block Shield for secure connections.

#### Python Code - Continuous Monitoring

```python
import RPi.GPIO as GPIO
import time

# Pin configuration
SENSOR_PIN = 17

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN)

try:
    while True:
        if GPIO.input(SENSOR_PIN) == GPIO.LOW:
            print("Obstacle detected!")
        else:
            print("No obstacle")
        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
```

#### Python Code - Event Detection

```python
import RPi.GPIO as GPIO

SENSOR_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN)

def obstacle_detected(channel):
    print("Obstacle appeared!")

def obstacle_cleared(channel):
    print("Obstacle cleared!")

# Add event detection
GPIO.add_event_detect(SENSOR_PIN, GPIO.FALLING,
                      callback=obstacle_detected, bouncetime=300)
GPIO.add_event_detect(SENSOR_PIN, GPIO.RISING,
                      callback=obstacle_cleared, bouncetime=300)

try:
    print("Monitoring for obstacles...")
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
```

#### Integration with Motor Control

```python
import RPi.GPIO as GPIO

# Pin configuration
SENSOR_PIN = 17
MOTOR_IN1 = 23
MOTOR_IN2 = 24
MOTOR_ENA = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN)
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)
GPIO.setup(MOTOR_ENA, GPIO.OUT)

pwm = GPIO.PWM(MOTOR_ENA, 1000)
pwm.start(0)

def move_forward():
    GPIO.output(MOTOR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(50)

def stop_motors():
    pwm.ChangeDutyCycle(0)

try:
    while True:
        if GPIO.input(SENSOR_PIN) == GPIO.LOW:
            print("Obstacle! Stopping...")
            stop_motors()
        else:
            print("Path clear, moving forward")
            move_forward()
        time.sleep(0.1)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
```

### Ultrasonic Distance Sensor (HC-SR04)

#### Specifications
- Detection range: 2-400 cm
- Operating voltage: 5V
- Measuring angle: 15 degrees
- Output: Echo pulse width (microseconds)

#### Pin Configuration
- **VCC**: 5V power supply
- **Trig**: Trigger input (10µs pulse to start measurement)
- **Echo**: Echo output (pulse width = distance)
- **GND**: Ground

#### Distance Calculation
Distance (cm) = (Echo pulse width in µs) × 0.034 / 2

Speed of sound = 340 m/s = 0.034 cm/µs
Divide by 2 because signal travels to object and back.

#### Raspberry Pi Wiring

```
HC-SR04            Raspberry Pi
-------            ------------
VCC          →     5V
Trig         →     GPIO 23
Echo         →     GPIO 24 (via voltage divider!)
GND          →     GND
```

**Warning**: HC-SR04 Echo output is 5V, which can damage Raspberry Pi GPIO (3.3V). Use voltage divider:
- 1kΩ resistor between Echo and GPIO 24
- 2kΩ resistor between GPIO 24 and GND
- This creates 3.3V from 5V signal

#### Python Code

```python
import RPi.GPIO as GPIO
import time

TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def measure_distance():
    # Send 10µs pulse to trigger
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG, GPIO.LOW)

    # Wait for echo pulse
    while GPIO.input(ECHO) == GPIO.LOW:
        pulse_start = time.time()

    while GPIO.input(ECHO) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound / 2
    distance = round(distance, 2)

    return distance

try:
    while True:
        dist = measure_distance()
        print(f"Distance: {dist} cm")
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
```

## Inertial Measurement Unit (IMU)

### MPU9250 9-Axis IMU

#### Specifications
- 3-axis accelerometer (±2g to ±16g)
- 3-axis gyroscope (±250°/s to ±2000°/s)
- 3-axis magnetometer (compass)
- I2C and SPI interfaces
- Operating voltage: 3.3V

#### I2C Connection (Raspberry Pi)

```
MPU9250            Raspberry Pi
-------            ------------
VCC          →     3.3V (Pin 1)
GND          →     GND (Pin 6)
SCL          →     GPIO 3 (SCL, Pin 5)
SDA          →     GPIO 2 (SDA, Pin 3)
```

#### Python Code (using mpu9250-i2c library)

```bash
pip install mpu9250-i2c
```

```python
from mpu9250_i2c import *
import time

# Initialize
mpu = MPU9250()

try:
    while True:
        # Read accelerometer (m/s²)
        accel = mpu.readAccelerometerMaster()
        print(f"Accel: X={accel[0]:.2f}, Y={accel[1]:.2f}, Z={accel[2]:.2f}")

        # Read gyroscope (°/s)
        gyro = mpu.readGyroscopeMaster()
        print(f"Gyro: X={gyro[0]:.2f}, Y={gyro[1]:.2f}, Z={gyro[2]:.2f}")

        # Read magnetometer (µT)
        mag = mpu.readMagnetometerMaster()
        print(f"Mag: X={mag[0]:.2f}, Y={mag[1]:.2f}, Z={mag[2]:.2f}")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped")
```

#### Integration with ORB-SLAM2

The MPU9250 can provide inertial odometry to improve SLAM robustness:
- Gyroscope: Track rotation between camera frames
- Accelerometer: Detect sudden movements
- Magnetometer: Absolute heading reference

See ORB-SLAM2 documentation for sensor fusion configuration.

## Wireless Communication

### NRF24L01 RF Module

#### Specifications
- Frequency: 2.4 GHz ISM band
- Range: 100m (open space, with PA/LNA)
- Data rate: 250kbps to 2Mbps
- Operating voltage: 3.3V
- Interface: SPI

#### Raspberry Pi Wiring

```
NRF24L01           Raspberry Pi
--------           ------------
VCC          →     3.3V (Pin 1)
GND          →     GND (Pin 6)
CE           →     GPIO 25 (Pin 22)
CSN          →     GPIO 8 (CE0, Pin 24)
SCK          →     GPIO 11 (SCLK, Pin 23)
MOSI         →     GPIO 10 (MOSI, Pin 19)
MISO         →     GPIO 9 (MISO, Pin 21)
IRQ          →     GPIO 24 (Pin 18, optional)
```

**Important**: NRF24L01 is 3.3V only. Do not connect to 5V!

#### Python Code (using nrf24 library)

```bash
pip install nrf24
```

**Transmitter:**
```python
from nrf24 import NRF24
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 25)  # CE pin
radio.setPayloadSize(32)
radio.setChannel(0x76)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.openWritingPipe(pipes[0])

while True:
    message = "Hello World"
    radio.write(message.encode('utf-8'))
    print(f"Sent: {message}")
    time.sleep(1)
```

**Receiver:**
```python
radio.openReadingPipe(1, pipes[0])
radio.startListening()

while True:
    if radio.available():
        received = []
        radio.read(received, radio.getDynamicPayloadSize())
        message = ''.join(chr(x) for x in received)
        print(f"Received: {message}")
    time.sleep(0.1)
```

## Bluetooth Communication (Raspberry Pi 3/4)

### Built-in Bluetooth Setup

#### Install Required Packages

```bash
sudo apt-get update
sudo apt-get install bluetooth blueman bluez python3-bluez python3-rpi.gpio
```

#### Pairing Process

```bash
sudo bluetoothctl
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# default-agent
[bluetooth]# discoverable on
[bluetooth]# pairable on
[bluetooth]# scan on
```

Find your device MAC address, then:

```bash
[bluetooth]# pair XX:XX:XX:XX:XX:XX
[bluetooth]# trust XX:XX:XX:XX:XX:XX
[bluetooth]# connect XX:XX:XX:XX:XX:XX
```

#### Python RFCOMM Server

```python
import bluetooth
import RPi.GPIO as GPIO

# Motor control pins
IN1 = 23
IN2 = 24
IN3 = 17
IN4 = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

def move_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def move_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def turn_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

# Create Bluetooth socket
server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_socket.bind(("", bluetooth.PORT_ANY))
server_socket.listen(1)

port = server_socket.getsockname()[1]
print(f"Waiting for connection on RFCOMM channel {port}")

client_socket, client_info = server_socket.accept()
print(f"Accepted connection from {client_info}")

try:
    while True:
        data = client_socket.recv(1024).decode('utf-8')
        print(f"Received: {data}")

        if data == 'F':
            move_forward()
        elif data == 'B':
            move_backward()
        elif data == 'L':
            turn_left()
        elif data == 'R':
            turn_right()
        elif data == 'S':
            stop()
        elif data == 'Q':
            break

except KeyboardInterrupt:
    pass

finally:
    client_socket.close()
    server_socket.close()
    GPIO.cleanup()
```

### Android Control App

Use **BlueTerm** app (supports RFCOMM protocol):
- Download from Google Play Store
- Pair with Raspberry Pi via Android Bluetooth settings
- Connect to Raspberry Pi in BlueTerm
- Send single character commands:
  - **F**: Forward
  - **B**: Backward
  - **L**: Turn left
  - **R**: Turn right
  - **S**: Stop
  - **Q**: Quit

## MQTT IoT Integration

### Installation

```bash
pip install paho-mqtt
sudo apt-get install mosquitto mosquitto-clients
```

### MQTT Broker Setup (Raspberry Pi)

```bash
# Start Mosquitto broker
sudo systemctl start mosquitto
sudo systemctl enable mosquitto

# Test with command line
mosquitto_sub -h localhost -t "test/topic"  # Terminal 1
mosquitto_pub -h localhost -t "test/topic" -m "Hello"  # Terminal 2
```

### Python Publisher (Sensor Data)

```python
import paho.mqtt.client as mqtt
import time
import random

broker = "localhost"  # or remote broker IP
port = 1883
topic = "robot/sensors"

client = mqtt.Client("SensorPublisher")
client.connect(broker, port)

try:
    while True:
        # Simulate sensor reading
        distance = random.uniform(10, 100)
        message = f"Distance: {distance:.2f} cm"

        client.publish(topic, message)
        print(f"Published: {message}")

        time.sleep(1)

except KeyboardInterrupt:
    client.disconnect()
```

### Python Subscriber (Motor Control)

```python
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

# Motor pins
IN1 = 23
IN2 = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("robot/motor/command")

def on_message(client, userdata, msg):
    command = msg.payload.decode('utf-8')
    print(f"Received: {command}")

    if command == "FORWARD":
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif command == "REVERSE":
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    elif command == "STOP":
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

broker = "localhost"
client = mqtt.Client("MotorController")
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker, 1883)
client.loop_forever()
```

### ESP32 MQTT Integration

```cpp
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";
const char* mqtt_server = "192.168.1.100";  // Raspberry Pi IP

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message: ");
  Serial.println(message);

  // Control motors based on message
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    client.connect("ESP32Client");
    client.subscribe("robot/motor/command");
  }
  client.loop();
}
```

## Multi-Sensor Integration

### Sensor Fusion Example

```python
import RPi.GPIO as GPIO
from mpu9250_i2c import MPU9250
import time

# Sensor pins
IR_SENSOR = 17
ULTRASONIC_TRIG = 23
ULTRASONIC_ECHO = 24

# Motor pins
MOTOR_IN1 = 27
MOTOR_IN2 = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_SENSOR, GPIO.IN)
GPIO.setup(ULTRASONIC_TRIG, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO, GPIO.IN)
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)

# IMU
mpu = MPU9250()

def read_ultrasonic():
    GPIO.output(ULTRASONIC_TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(ULTRASONIC_TRIG, GPIO.LOW)

    while GPIO.input(ULTRASONIC_ECHO) == GPIO.LOW:
        pulse_start = time.time()

    while GPIO.input(ULTRASONIC_ECHO) == GPIO.HIGH:
        pulse_end = time.time()

    distance = (pulse_end - pulse_start) * 17150
    return round(distance, 2)

def obstacle_avoidance():
    # Check IR sensor (short range)
    ir_detected = GPIO.input(IR_SENSOR) == GPIO.LOW

    # Check ultrasonic (medium range)
    distance = read_ultrasonic()

    # Read IMU for orientation
    gyro = mpu.readGyroscopeMaster()

    # Decision logic
    if ir_detected or distance < 20:
        print("Close obstacle! Emergency stop")
        GPIO.output(MOTOR_IN1, GPIO.LOW)
        GPIO.output(MOTOR_IN2, GPIO.LOW)
        return "STOP"
    elif distance < 50:
        print("Medium distance obstacle, slow down")
        return "SLOW"
    else:
        print("Path clear")
        return "FORWARD"

try:
    while True:
        action = obstacle_avoidance()
        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
```

## Troubleshooting

### IR Sensor Issues
- **Always detecting obstacle**: Adjust sensitivity potentiometer, check for ambient IR interference
- **Not detecting obstacle**: Verify 5V power, clean sensor lens, test with reflective surface

### Ultrasonic Sensor Issues
- **Inconsistent readings**: Add small delay between measurements (60ms minimum)
- **No reading**: Check voltage divider on Echo pin, verify Trig pulse timing

### I2C Communication Errors
- **Device not found**: Enable I2C in raspi-config, check wiring, verify device address with `i2cdetect -y 1`
- **Intermittent communication**: Add pull-up resistors (4.7kΩ) on SDA and SCL lines

### Bluetooth Pairing Problems
- **Cannot discover Pi**: Check bluetoothctl shows "discoverable on"
- **Pairing fails**: Remove old pairings, restart bluetooth service: `sudo systemctl restart bluetooth`

### RF24 Communication Failure
- **No data received**: Verify 3.3V power (not 5V!), check SPI connections, ensure matching pipe addresses
- **Short range**: Add capacitor (10µF) across VCC/GND, use PA/LNA version for longer range
