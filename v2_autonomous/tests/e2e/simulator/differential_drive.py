"""
DifferentialDrive - High-level physics simulation API

Provides a simpler interface to ConcretePhysicsSimulator with:
- RobotPose dataclass for position/orientation tracking
- Motor command abstraction (linear + angular velocity)
- Differential drive kinematics
- Wheel variance, inertia, and slip effects
"""

from dataclasses import dataclass
from typing import Tuple
import random
import math
from .concrete_physics_simulator import ConcretePhysicsSimulator


@dataclass
class RobotPose:
    """Robot position and orientation in 2D space"""
    x: float = 0.0       # cm
    y: float = 0.0       # cm
    theta: float = 0.0   # degrees, 0=right, counter-clockwise positive


class DifferentialDrive:
    """
    High-level interface for differential drive robot with physics simulation.

    Manages:
    - Motor command conversion to differential wheel speeds
    - Differential drive kinematics (position integration)
    - Wheel variance (±5% per wheel)
    - Motor inertia (200ms time constant)
    - Slip effect (2% probability -> 30% speed)

    Coordinate System:
    - X: positive to the right
    - Y: positive upward
    - Theta: positive counter-clockwise from +X axis (0° = right, 90° = up)
    """

    def __init__(self, wheel_base: float = 15.0,
                 wheel_variance: float = 0.05,
                 max_speed: float = 30.0):
        """
        Initialize differential drive controller.

        Args:
            wheel_base: Distance between wheels in cm (default: 15 cm from Arduino)
            wheel_variance: Wheel speed variance factor, 0.0-1.0 (default: 0.05 = ±5%)
            max_speed: Maximum linear speed in cm/s (default: 30 cm/s)
        """
        self.wheel_base = wheel_base
        self.max_speed = max_speed

        # Initialize physics simulator
        self.simulator = ConcretePhysicsSimulator()

        # Apply configuration
        self.simulator.set_wheel_variance(wheel_variance)
        self.simulator.wheel_variance_enabled = (wheel_variance > 0.0)

        # Motor command state
        self.target_linear = 0.0   # cm/s
        self.target_angular = 0.0  # deg/s
        self.left_target = 0.0     # cm/s
        self.right_target = 0.0    # cm/s

    @property
    def pose(self) -> RobotPose:
        """Get current robot pose (position + orientation)"""
        x, y, theta = self.simulator.get_position()
        return RobotPose(x=x, y=y, theta=theta)

    @property
    def velocity(self) -> Tuple[float, float, float]:
        """
        Get current robot velocity.

        Returns:
            Tuple of (vx, vy, omega) where:
            - vx, vy: linear velocity components in cm/s
            - omega: angular velocity in deg/s
        """
        return self.simulator.get_velocity()

    def set_motor_command(self, linear: float, angular: float) -> None:
        """
        Set motor command using linear and angular velocity.

        Uses differential drive equations to convert to left/right wheel speeds:
        - left_speed = linear - (angular * π/180) * wheel_base / 2
        - right_speed = linear + (angular * π/180) * wheel_base / 2

        Args:
            linear: Linear velocity in cm/s (±max_speed)
            angular: Angular velocity in deg/s (±max_angular_speed)
        """
        # Clamp to valid range
        linear = max(-self.max_speed, min(self.max_speed, linear))

        # Calculate max angular velocity based on differential drive
        # When left wheel is -max_speed and right wheel is +max_speed:
        # omega_rad = (right - left) / wheel_base = 2 * max_speed / wheel_base rad/s
        # omega_deg = omega_rad * 180/π = 2 * max_speed / wheel_base * 180/π deg/s
        max_angular_rad = (2.0 * self.max_speed) / self.wheel_base
        max_angular = math.degrees(max_angular_rad)
        angular = max(-max_angular, min(max_angular, angular))

        self.target_linear = linear
        self.target_angular = angular

        # Convert to differential wheel speeds
        # From: v = (v_left + v_right) / 2, omega_rad = (v_right - v_left) / wheel_base
        # Solve: v_left = v - omega_rad * wheel_base / 2
        #        v_right = v + omega_rad * wheel_base / 2
        omega_rad = math.radians(angular)
        left_speed = linear - omega_rad * self.wheel_base / 2.0
        right_speed = linear + omega_rad * self.wheel_base / 2.0

        # Store target velocities for reference
        self.left_target = left_speed
        self.right_target = right_speed

    def update(self, dt_ms: float) -> None:
        """
        Update physics simulation.

        Integrates position based on current motor commands.

        Args:
            dt_ms: Time step in milliseconds
        """
        dt = dt_ms / 1000.0  # Convert to seconds

        # Convert current target velocities to PWM
        # Clamp to valid range first
        left_speed = max(-self.max_speed, min(self.max_speed, self.left_target))
        right_speed = max(-self.max_speed, min(self.max_speed, self.right_target))

        # Convert to PWM (-255 to +255)
        left_pwm = int((left_speed / self.max_speed) * 255.0) if self.max_speed > 0 else 0
        right_pwm = int((right_speed / self.max_speed) * 255.0) if self.max_speed > 0 else 0

        # Clamp PWM values
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        self.simulator.update(dt, left_pwm, right_pwm)

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """
        Reset robot position and orientation.

        Args:
            x: X position in cm
            y: Y position in cm
            theta: Orientation in degrees (-180 to +180)
        """
        self.simulator.reset_position(x, y, theta)
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.left_target = 0.0
        self.right_target = 0.0

    def get_position(self) -> Tuple[float, float, float]:
        """
        Get current robot position and orientation.

        Returns:
            Tuple of (x, y, theta) in cm and degrees
        """
        return self.simulator.get_position()

    def is_out_of_bounds(self) -> bool:
        """Check if robot has left simulation boundary."""
        return self.simulator.is_out_of_bounds()

    # Configuration methods for testing/experimentation

    def set_sensor_delay(self, delay_ms: float) -> None:
        """Set sensor reading delay in milliseconds."""
        self.simulator.set_sensor_delay(delay_ms)

    def set_noise_level(self, noise_level: float) -> None:
        """Set sensor noise level (standard deviation in cm)."""
        self.simulator.set_noise_level(noise_level)

    def set_wheel_variance(self, variance: float) -> None:
        """Set wheel speed variance factor (0.0 = perfect, 0.05 = ±5%)."""
        self.simulator.set_wheel_variance(variance)

    def set_world_bounds(self, min_x: float, min_y: float,
                         max_x: float, max_y: float) -> None:
        """Set simulation world boundaries in cm."""
        self.simulator.set_world_bounds(min_x, min_y, max_x, max_y)

    def set_current_distances(self, front: float, right: float) -> None:
        """Set current sensor distances for integration with virtual environment."""
        self.simulator.set_current_distances(front, right)

    def get_sensor_reading(self):
        """Get current sensor reading with delay, noise, and anomalies."""
        return self.simulator.get_sensor_reading()

    def get_motor_state(self) -> dict:
        """Get internal motor and dynamics state for debugging."""
        return self.simulator.get_motor_state()

    def get_buffer_state(self) -> dict:
        """Get internal buffer state for debugging."""
        return self.simulator.get_buffer_state()


# Helper function for quick instantiation with common parameters

def create_robot(wheel_base: float = 15.0,
                wheel_variance: float = 0.05) -> DifferentialDrive:
    """
    Create a DifferentialDrive robot with standard parameters.

    Corresponds to the v2_autonomous robot specifications:
    - wheel_base: 15 cm
    - max_speed: 30 cm/s (Arduino MAX_SPEED ≈ 30)
    - wheel_variance: ±5%
    - Motor inertia: 200ms (via ConcretePhysicsSimulator)
    - Slip: 2% probability, 30% speed (via ConcretePhysicsSimulator)

    Args:
        wheel_base: Distance between wheels (cm)
        wheel_variance: Wheel speed variance (0.0-1.0)

    Returns:
        DifferentialDrive instance configured for v2_autonomous
    """
    return DifferentialDrive(
        wheel_base=wheel_base,
        wheel_variance=wheel_variance,
        max_speed=30.0  # Arduino MAX_SPEED
    )
