"""
Concrete Physics Simulator Implementation

Implements the PhysicsSimulator interface with:
- Differential wheel speed variance (±5% per wheel)
- Motor inertia (exponential approach to target speed)
- Slip effect (2% probability -> 30% of target speed)
- Sensor delay using circular buffer
- Gaussian noise simulation
- Anomaly injection (0/999 values)
- Position and orientation tracking
- IMU data simulation
"""

from collections import deque
import math
import random
import time
from typing import Tuple, Optional
from .physics_simulator import PhysicsSimulator, SensorReading


class ConcretePhysicsSimulator(PhysicsSimulator):
    """
    Concrete implementation of physics simulation with realistic sensor behavior.

    Features:
    - Differential wheel speed variance (±5% per wheel, initialized randomly)
    - Motor inertia with 200ms time constant for acceleration
    - Slip effect: 2% probability of slip, speed drops to 30% of target
    - 60ms configurable sensor delay using circular buffer
    - Gaussian noise (σ=2cm) with speed-dependent scaling
    - 10% anomaly rate (returns 0 or 999cm)
    - Position tracking using differential drive kinematics
    """

    def __init__(self, max_range: float = 500.0):
        """
        Initialize concrete physics simulator.

        Args:
            max_range: Maximum sensor detection range in cm
        """
        # Sensor configuration
        self.sensor_delay_ms = 60  # milliseconds
        self.noise_sigma = 2.0  # cm (Gaussian noise standard deviation)
        self.invalid_rate = 0.1  # 10% anomaly rate
        self.max_range = max_range

        # Circular buffer for delayed readings
        # Buffer size = delay (ms) / update period (ms)
        # At 50Hz: 60ms / 20ms = 3 entries, use 20 for safety margin
        self.reading_buffer = deque(maxlen=20)

        # Motor and velocity state
        self.left_pwm = 0
        self.right_pwm = 0
        self.left_velocity = 0.0  # cm/s (actual velocity)
        self.right_velocity = 0.0  # cm/s (actual velocity)
        self.left_target_velocity = 0.0  # cm/s (target from PWM)
        self.right_target_velocity = 0.0  # cm/s (target from PWM)

        # Motor inertia parameters
        self.inertia_tau = 0.2  # Time constant in seconds (200ms)

        # Wheel variance factors (initialized randomly per wheel)
        self.left_variance_factor = 1.0 + random.uniform(-0.05, 0.05)  # ±5%
        self.right_variance_factor = 1.0 + random.uniform(-0.05, 0.05)  # ±5%
        self.wheel_variance_enabled = True  # Flag to enable/disable

        # Slip state
        self.left_slipping = False
        self.right_slipping = False
        self.slip_probability = 0.02  # 2% probability per wheel per update
        self.slip_factor = 0.3  # Speed drops to 30% when slipping

        self.max_linear_speed = 60.0  # cm/s at max PWM (255)
        self.max_angular_speed = 180.0  # deg/s at max PWM differential

        # Position and orientation
        self.x = 0.0  # cm
        self.y = 0.0  # cm
        self.yaw = 0.0  # degrees (-180 to +180)
        self.wheel_base = 15.0  # cm (distance between wheels)
        self.robot_radius = 12.0  # cm

        # World bounds
        self.world_bounds = (-500.0, -500.0, 2000.0, 2000.0)

        # Virtual sensors (in real implementation, these would come from
        # ray-casting against environment)
        self.front_distance = self.max_range
        self.right_distance = self.max_range
        self.imu_valid = True

        # Time tracking
        self.last_update_time = time.time()
        self.simulation_time = 0.0

        # Initialize buffer with valid readings
        initial_reading = SensorReading(
            front_distance=int(self.max_range),
            right_distance=int(self.max_range),
            yaw=0.0,
            imu_valid=True,
            timestamp=time.time()
        )
        self.reading_buffer.append(initial_reading)

    def set_sensor_delay(self, delay_ms: float) -> None:
        """
        Set sensor reading delay to simulate real-world latency.

        Args:
            delay_ms: Delay in milliseconds (0.0 for instantaneous)
        """
        self.sensor_delay_ms = max(0.0, delay_ms)
        # Adjust buffer size: assume 20ms per update (50Hz)
        # delay_samples = ceil(delay_ms / 20)
        new_maxlen = max(1, int(math.ceil(delay_ms / 20.0)))
        if new_maxlen != self.reading_buffer.maxlen:
            # Preserve existing data when resizing
            old_data = list(self.reading_buffer)
            self.reading_buffer = deque(old_data, maxlen=new_maxlen)

    def set_wheel_variance(self, variance: float) -> None:
        """
        Set wheel speed variance to simulate motor imperfections.

        Each wheel gets a random variance factor in range [1-variance, 1+variance].
        This is initialized once and persists throughout simulation.

        Args:
            variance: Variance factor (0.0 = perfect, 0.05 = ±5% variance)
        """
        if variance > 0.0:
            # Initialize random variance factors for each wheel
            self.left_variance_factor = 1.0 + random.uniform(-variance, variance)
            self.right_variance_factor = 1.0 + random.uniform(-variance, variance)
            self.wheel_variance_enabled = True
        else:
            # Disable variance
            self.left_variance_factor = 1.0
            self.right_variance_factor = 1.0
            self.wheel_variance_enabled = False

    def set_noise_level(self, noise_level: float) -> None:
        """
        Set sensor noise level for distance measurements.

        Args:
            noise_level: Standard deviation of Gaussian noise in cm
        """
        self.noise_sigma = max(0.0, noise_level)

    def update(self, dt: float, left_pwm: int, right_pwm: int) -> None:
        """
        Update physics simulation by dt seconds.

        Implements:
        1. PWM to target velocity conversion
        2. Apply wheel variance factors
        3. Motor inertia: gradual acceleration/deceleration (200ms time constant)
        4. Slip detection and handling (2% probability -> 30% speed)
        5. Position integration using differential drive kinematics

        Args:
            dt: Time step in seconds
            left_pwm: Left motor PWM value (-255 to +255)
            right_pwm: Right motor PWM value (-255 to +255)
        """
        # Clamp PWM values
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        self.left_pwm = left_pwm
        self.right_pwm = right_pwm

        # Step 1: Convert PWM to target velocity
        # Linear mapping: 255 PWM -> max_linear_speed cm/s
        left_target = (left_pwm / 255.0) * self.max_linear_speed
        right_target = (right_pwm / 255.0) * self.max_linear_speed

        # Step 2: Apply wheel variance factors (permanent per wheel)
        if self.wheel_variance_enabled:
            left_target *= self.left_variance_factor
            right_target *= self.right_variance_factor

        self.left_target_velocity = left_target
        self.right_target_velocity = right_target

        # Step 3: Apply motor inertia (exponential approach to target)
        if dt > 0:
            # Exponential decay towards target: v(t) = v_target + (v_current - v_target) * exp(-t/tau)
            # Using discrete approximation: dv = (v_target - v_current) * (1 - exp(-dt/tau))
            inertia_factor = 1.0 - math.exp(-dt / self.inertia_tau)

            self.left_velocity += (left_target - self.left_velocity) * inertia_factor
            self.right_velocity += (right_target - self.right_velocity) * inertia_factor

        # Step 4: Apply slip effect (2% probability per wheel)
        # When slipping occurs, speed drops to 30% of target
        self.left_slipping = random.random() < self.slip_probability
        self.right_slipping = random.random() < self.slip_probability

        if self.left_slipping:
            self.left_velocity *= self.slip_factor
        if self.right_slipping:
            self.right_velocity *= self.slip_factor

        # Step 5: Update position using differential drive kinematics
        if dt > 0:
            # Average velocity and angular velocity
            v = (self.left_velocity + self.right_velocity) / 2.0  # cm/s
            omega = (self.right_velocity - self.left_velocity) / self.wheel_base  # rad/s

            # Convert omega to degrees/s
            omega_deg = math.degrees(omega)

            # Update position
            if abs(omega) < 1e-6:  # Going straight
                dx = v * dt * math.cos(math.radians(self.yaw))
                dy = v * dt * math.sin(math.radians(self.yaw))
            else:  # Turning
                # Use differential drive kinematics
                radius = v / omega if omega != 0 else 0
                dx = radius * math.sin(math.radians(self.yaw + omega_deg * dt)) - \
                     radius * math.sin(math.radians(self.yaw))
                dy = -radius * math.cos(math.radians(self.yaw + omega_deg * dt)) + \
                     radius * math.cos(math.radians(self.yaw))

            self.x += dx
            self.y += dy
            self.yaw += omega_deg * dt

            # Normalize yaw to [-180, 180]
            while self.yaw > 180:
                self.yaw -= 360
            while self.yaw < -180:
                self.yaw += 360

        # Update simulation time
        self.simulation_time += dt

        # Create new sensor reading with current true values
        # In real implementation, these would come from ray-casting
        reading = SensorReading(
            front_distance=int(self.front_distance),
            right_distance=int(self.right_distance),
            yaw=self.yaw,
            imu_valid=self.imu_valid,
            timestamp=time.time()
        )
        self.reading_buffer.append(reading)

    def get_sensor_reading(self) -> SensorReading:
        """
        Get current sensor reading with configured delay and noise.

        Returns:
            SensorReading containing distance and orientation data with:
            - 60ms delay via circular buffer
            - Gaussian noise (σ=2cm)
            - 10% anomaly rate (returns 0 or 999cm)
        """
        # Get delayed reading from buffer
        if len(self.reading_buffer) == 0:
            # Fallback to current values
            delayed_reading = SensorReading(
                front_distance=int(self.front_distance),
                right_distance=int(self.right_distance),
                yaw=self.yaw,
                imu_valid=self.imu_valid,
                timestamp=time.time()
            )
        else:
            # Get reading from circular buffer
            # Buffer stores readings in chronological order: [oldest, ..., newest]
            # We want the reading that is approximately sensor_delay_ms old
            delay_samples = max(1, int(math.ceil(self.sensor_delay_ms / 20.0)))

            # Index from the end: 0 = newest, 1 = 20ms old, 2 = 40ms old, etc.
            buffer_index = min(delay_samples - 1, len(self.reading_buffer) - 1)
            delayed_reading = self.reading_buffer[buffer_index]

        # Apply noise to distance measurements
        front_noisy = self._apply_sensor_noise(delayed_reading.front_distance)
        right_noisy = self._apply_sensor_noise(delayed_reading.right_distance)

        # Apply anomalies (10% chance)
        front_final = self._apply_anomaly(front_noisy)
        right_final = self._apply_anomaly(right_noisy)

        return SensorReading(
            front_distance=front_final,
            right_distance=right_final,
            yaw=delayed_reading.yaw,
            imu_valid=delayed_reading.imu_valid,
            timestamp=delayed_reading.timestamp
        )

    def _apply_sensor_noise(self, distance: int) -> int:
        """
        Apply Gaussian noise to sensor reading.

        Noise is speed-dependent: faster movement = more noise

        Args:
            distance: True distance in cm

        Returns:
            Noisy distance in cm
        """
        # Speed-dependent noise scaling
        # Current velocity magnitude
        current_speed = (abs(self.left_velocity) + abs(self.right_velocity)) / 2.0
        speed_factor = 1.0 + (current_speed / self.max_linear_speed) * 0.5  # Up to 1.5x noise

        # Apply Gaussian noise
        noise = random.gauss(0, self.noise_sigma * speed_factor)
        noisy_distance = distance + noise

        # Clamp to valid range [0, max_range]
        noisy_distance = max(0, min(self.max_range, noisy_distance))

        return int(round(noisy_distance))

    def _apply_anomaly(self, distance: int) -> int:
        """
        Inject anomalies: 10% chance to return 0 or 999.

        Args:
            distance: Noisy distance in cm

        Returns:
            Distance with potential anomaly
        """
        if random.random() < self.invalid_rate:
            # Return anomaly: 50% chance 0, 50% chance 999
            return 0 if random.random() < 0.5 else 999
        return distance

    def get_position(self) -> Tuple[float, float, float]:
        """
        Get current robot position and orientation.

        Returns:
            Tuple of (x, y, yaw) where:
            - x, y: position in cm
            - yaw: orientation in degrees (-180 to +180)
        """
        return (self.x, self.y, self.yaw)

    def get_velocity(self) -> Tuple[float, float, float]:
        """
        Get current robot velocity.

        Returns:
            Tuple of (vx, vy, omega) where:
            - vx, vy: linear velocity in cm/s
            - omega: angular velocity in deg/s
        """
        # Calculate linear velocity components
        v = (self.left_velocity + self.right_velocity) / 2.0
        vx = v * math.cos(math.radians(self.yaw))
        vy = v * math.sin(math.radians(self.yaw))

        # Calculate angular velocity
        omega = math.degrees((self.right_velocity - self.left_velocity) / self.wheel_base)

        return (vx, vy, omega)

    def reset_position(self, x: float, y: float, yaw: float) -> None:
        """
        Reset robot position and orientation.

        Args:
            x: X position in cm
            y: Y position in cm
            yaw: Orientation in degrees
        """
        self.x = x
        self.y = y
        self.yaw = yaw

        # Normalize yaw
        while self.yaw > 180:
            self.yaw -= 360
        while self.yaw < -180:
            self.yaw += 360

        # Clear velocity
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.left_target_velocity = 0.0
        self.right_target_velocity = 0.0

    def is_out_of_bounds(self) -> bool:
        """
        Check if robot has left the simulation boundary.

        Returns:
            True if robot is out of bounds
        """
        min_x, min_y, max_x, max_y = self.world_bounds
        return (self.x < min_x or self.x > max_x or
                self.y < min_y or self.y > max_y)

    def set_world_bounds(self, min_x: float, min_y: float,
                         max_x: float, max_y: float) -> None:
        """
        Set simulation world boundaries.

        Args:
            min_x, min_y: Minimum coordinates in cm
            max_x, max_y: Maximum coordinates in cm
        """
        self.world_bounds = (min_x, min_y, max_x, max_y)

    def set_current_distances(self, front: float, right: float) -> None:
        """
        Set current true distance values (for testing/environment integration).

        In real implementation, these would come from ray-casting against
        the virtual environment.

        Args:
            front: Front sensor distance in cm
            right: Right sensor distance in cm
        """
        self.front_distance = max(0, min(self.max_range, front))
        self.right_distance = max(0, min(self.max_range, right))

    def get_buffer_state(self) -> dict:
        """
        Get internal buffer state for debugging.

        Returns:
            Dictionary with buffer info
        """
        return {
            'buffer_size': len(self.reading_buffer),
            'buffer_maxlen': self.reading_buffer.maxlen,
            'sensor_delay_ms': self.sensor_delay_ms,
            'noise_sigma': self.noise_sigma,
            'invalid_rate': self.invalid_rate,
        }

    def get_motor_state(self) -> dict:
        """
        Get detailed motor and dynamics state for debugging.

        Returns:
            Dictionary with motor dynamics info
        """
        return {
            'left_pwm': self.left_pwm,
            'right_pwm': self.right_pwm,
            'left_velocity': self.left_velocity,
            'right_velocity': self.right_velocity,
            'left_target_velocity': self.left_target_velocity,
            'right_target_velocity': self.right_target_velocity,
            'left_variance_factor': self.left_variance_factor,
            'right_variance_factor': self.right_variance_factor,
            'left_slipping': self.left_slipping,
            'right_slipping': self.right_slipping,
            'inertia_tau': self.inertia_tau,
        }
