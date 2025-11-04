# Motor Control Reference

## L298N H-Bridge Motor Driver

### Technical Specifications
- Dual H-bridge motor driver
- Operating voltage: 5V-35V (motor supply)
- Logic voltage: 5V (TTL compatible)
- Maximum current: 2A per channel (4A peak)
- PWM frequency: up to 40kHz

### Pin Configuration

#### Power Pins
- **12V (or VCC)**: Motor power supply input (7.4-12V recommended)
- **5V**: Logic power output (when regulator enabled) or input
- **GND**: Common ground (must be connected to Arduino/Pi and battery ground)

#### Control Pins (per motor)
- **IN1, IN2**: Direction control for Motor A
- **IN3, IN4**: Direction control for Motor B
- **ENA**: Enable/speed control for Motor A (PWM capable)
- **ENB**: Enable/speed control for Motor B (PWM capable)

#### Motor Output
- **OUT1, OUT2**: Motor A terminals
- **OUT3, OUT4**: Motor B terminals

### Jumper Configuration

#### InA and InB Enable Jumpers
- **Jumper ON**: Motor always enabled (full speed, direction only via IN pins)
- **Jumper OFF**: Motor speed controlled via ENA/ENB pins (required for PWM)

**Recommendation**: Remove jumpers and connect ENA/ENB to Arduino PWM pins for speed control.

#### Voltage Regulator Jumper
- **Jumper ON**:
  - Onboard 7805 regulator active
  - Converts input voltage (7.4-12V) to 5V for logic
  - 5V output available on 5V pin
  - Can power Arduino through 5V pin
- **Jumper OFF**:
  - Regulator disabled
  - External 5V supply required for logic
  - Use when motor voltage < 7V or > 12V
  - Prevents overheating of onboard regulator

**Warning**: The L298N voltage regulator can overheat with high motor current or voltages > 12V. Use external 5V supply for demanding applications.

### Direction Control Logic

| IN1 | IN2 | Motor A State |
|-----|-----|---------------|
| LOW | LOW | Stop (brake) |
| LOW | HIGH | Reverse |
| HIGH | LOW | Forward |
| HIGH | HIGH | Stop (brake) |

Same logic applies to Motor B with IN3/IN4.

### Arduino Wiring Example

```
L298N          Arduino
-----          -------
IN1       →    Pin 7
IN2       →    Pin 8
ENA       →    Pin 9 (PWM)
IN3       →    Pin 4
IN4       →    Pin 5
ENB       →    Pin 6 (PWM)
GND       →    GND
5V        →    5V (if using L298N regulator)

12V       →    Battery + (9V recommended)
GND       →    Battery - (common with Arduino GND)
```

### Arduino Code Example

```cpp
// Motor A pins
const int IN1 = 7;
const int IN2 = 8;
const int ENA = 9;

// Motor B pins
const int IN3 = 4;
const int IN4 = 5;
const int ENB = 6;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  // Motor A forward at 75% speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 191);  // 0-255

  // Motor B forward at 50% speed
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 127);

  delay(2000);

  // Stop both motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  delay(1000);
}

// Function to control motor speed and direction
void setMotor(int motorNum, int speed) {
  // speed: -255 (full reverse) to 255 (full forward)
  int in1, in2, en;

  if (motorNum == 1) {
    in1 = IN1; in2 = IN2; en = ENA;
  } else {
    in1 = IN3; in2 = IN4; en = ENB;
  }

  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -speed);
  }
}
```

### Raspberry Pi Control (Python)

```python
import RPi.GPIO as GPIO
import time

# GPIO pin configuration
IN1 = 23
IN2 = 24
ENA = 25
IN3 = 17
IN4 = 27
ENB = 22

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# PWM setup (1000 Hz)
pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

def set_motor(motor, speed):
    """
    Control motor speed and direction
    motor: 'A' or 'B'
    speed: -100 to 100 (negative = reverse)
    """
    if motor == 'A':
        if speed >= 0:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            pwm_a.ChangeDutyCycle(speed)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            pwm_a.ChangeDutyCycle(-speed)
    elif motor == 'B':
        if speed >= 0:
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
            pwm_b.ChangeDutyCycle(speed)
        else:
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
            pwm_b.ChangeDutyCycle(-speed)

# Example usage
try:
    # Forward
    set_motor('A', 75)
    set_motor('B', 75)
    time.sleep(2)

    # Stop
    set_motor('A', 0)
    set_motor('B', 0)
    time.sleep(1)

    # Reverse
    set_motor('A', -50)
    set_motor('B', -50)
    time.sleep(2)

finally:
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
```

## L293D Motor Driver (Alternative)

### Differences from L298N
- Lower current capacity: 600mA per channel (1.2A peak)
- Smaller form factor (16-pin DIP chip)
- No onboard voltage regulator
- Typically requires heatsink
- More suitable for small motors

### Pin Configuration (L293D IC)
- Pin 1, 9 (Enable 1, 2): PWM speed control
- Pin 2, 7 (Input 1A, 2A): Motor A direction
- Pin 10, 15 (Input 3A, 4A): Motor B direction
- Pin 3, 6 (Output 1A, 2A): Motor A terminals
- Pin 11, 14 (Output 3A, 4A): Motor B terminals
- Pin 4, 5, 12, 13: Ground
- Pin 8: Motor power supply (4.5-36V)
- Pin 16: Logic power supply (5V)

## Diablo Motor Controller (High-Power Option)

### Specifications
- Dual-channel motor controller
- Voltage range: 7-36V
- Current capacity: 55A per channel (continuous)
- I2C interface for Raspberry Pi control
- Built-in protection: overheat, undervoltage, short circuit
- Supports DC motors and stepper motors

### Installation

```bash
git clone https://github.com/piborg/diablo
cd diablo
bash install.sh
```

This installs the Python library, GUI tool, and dependencies.

### Python API Example

```python
import diablo

# Initialize
bot = diablo.Diablo()
bot.Init()

# Check battery voltage
voltage = bot.GetBatteryReading()
print(f"Battery: {voltage:.2f}V")

# Set motor speeds (-1.0 to 1.0)
bot.SetMotor1(0.5)   # Motor 1 forward at 50%
bot.SetMotor2(-0.3)  # Motor 2 reverse at 30%

# Stop all motors
bot.MotorsOff()
```

### GUI Control

```bash
python3 diabloGui.py
```

Provides slider-based motor control for testing.

### Joystick Control

```bash
python3 diabloJoystick.py
```

Enables PS4 controller integration via Pygame.

### Stepper Motor Support

```python
import diablo

bot = diablo.Diablo()
bot.Init()

# Configure for stepper motor (4-wire)
# Drive stepper with sequence control
# See diabloStepper.py for complete example
```

## ROS2 Motor Control Integration

### Serial Motor Demo Architecture

**Components**:
- Arduino: Real-time motor control with encoder feedback
- Raspberry Pi: ROS2 driver node for high-level control
- Serial connection: UART communication between devices

### Driver Node Parameters

```bash
ros2 run serial_motor_demo driver --ros-args \
  -p encoder_cpr:=3440 \          # Encoder counts per revolution
  -p loop_rate:=30 \               # Update frequency (Hz)
  -p serial_port:=/dev/ttyUSB0 \   # Serial device
  -p baud_rate:=57600 \            # Communication speed
  -p serial_debug:=false           # Debug output
```

### Topic Interface

**Subscriptions**:
- `/motor_command` (MotorCommand): Velocity commands in rad/sec for each motor

**Publications**:
- `/motor_vels` (MotorVels): Actual motor velocities (feedback)
- `/encoder_vals` (EncoderVals): Raw encoder counts

### Control Modes

#### 1. Raw PWM Mode
Direct PWM control without feedback.

```python
from serial_motor_demo_msgs.msg import MotorCommand

cmd = MotorCommand()
cmd.mot_1_req_rad_sec = 100.0  # PWM value (-255 to 255)
cmd.mot_2_req_rad_sec = 100.0
publisher.publish(cmd)
```

#### 2. Closed-Loop Mode
Velocity control with encoder feedback and PID.

```python
cmd = MotorCommand()
cmd.mot_1_req_rad_sec = 3.14  # rad/sec
cmd.mot_2_req_rad_sec = 3.14
publisher.publish(cmd)
```

Arduino implements PID to achieve target velocity based on encoder feedback.

### GUI Testing Tool

```bash
ros2 run serial_motor_demo gui
```

Provides slider controls for both PWM and velocity modes.

## Encoder Integration

### Quadrature Encoder Basics
- Two output channels (A and B) with 90° phase shift
- Pulses per revolution (PPR) or Counts per revolution (CPR)
- CPR = 4 × PPR (using both edges of both channels)

### Arduino Encoder Reading

```cpp
// Interrupt-based encoder counting
volatile long encoder_count = 0;

void setup() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
}

void encoderISR() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoder_count++;
  } else {
    encoder_count--;
  }
}
```

### Velocity Calculation

```cpp
// Calculate velocity from encoder counts
float calculate_velocity(long current_count, long last_count, float dt) {
  long delta = current_count - last_count;
  float rps = (float)delta / encoder_cpr / dt;  // Revolutions per second
  return rps * 2.0 * PI;  // Convert to rad/sec
}
```

## Troubleshooting

### Motors Not Spinning
1. Check power supply voltage and capacity
2. Verify common ground between all components
3. Confirm jumper settings (remove ENA/ENB jumpers for PWM)
4. Test with simple full-speed code first
5. Check motor driver chip temperature (may be overheated)

### Inconsistent Speed
1. Ensure PWM frequency is appropriate (1-20kHz typical)
2. Check power supply stability under load
3. Add capacitors across motor terminals (0.1µF)
4. Verify PWM pins are used for speed control

### L298N Overheating
1. Add heatsink to chip
2. Reduce duty cycle or motor load
3. Use external 5V supply (disable onboard regulator)
4. Consider upgrading to higher-capacity driver (Diablo)

### Encoder Noise
1. Use pull-up resistors on encoder outputs (4.7kΩ)
2. Add debouncing in software
3. Use shielded cable for encoder signals
4. Keep encoder wires away from motor power wires
5. Add filtering capacitors (0.01µF) on encoder inputs
