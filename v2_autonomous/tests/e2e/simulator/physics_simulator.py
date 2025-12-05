"""
Physics Simulator Interface

Simulates the physical behavior of the robot, including:
- Motor dynamics and wheel velocity
- Sensor readings with configurable delays and noise
- IMU data simulation
- Position and orientation tracking
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple, Optional
import time


@dataclass
class SensorReading:
    """Encapsulates sensor readings from the robot"""
    front_distance: int  # cm
    right_distance: int  # cm
    yaw: float  # degrees
    imu_valid: bool  # IMU data validity
    timestamp: float  # seconds since epoch


class PhysicsSimulator(ABC):
    """
    Abstract interface for physics simulation.

    Handles robot dynamics, sensor simulation, and environmental interaction.
    Provides deterministic time control for testing.
    """

    @abstractmethod
    def set_sensor_delay(self, delay_ms: float) -> None:
        """
        Set sensor reading delay to simulate real-world latency.

        Args:
            delay_ms: Delay in milliseconds (0.0 for instantaneous)
        """
        pass

    @abstractmethod
    def set_wheel_variance(self, variance: float) -> None:
        """
        Set wheel speed variance to simulate motor imperfections.

        Higher variance = more differential between left/right wheels.

        Args:
            variance: Variance factor (0.0 = perfect, 0.1 = 10% variance)
        """
        pass

    @abstractmethod
    def set_noise_level(self, noise_level: float) -> None:
        """
        Set sensor noise level for distance measurements.

        Args:
            noise_level: Standard deviation of Gaussian noise in cm
        """
        pass

    @abstractmethod
    def update(self, dt: float, left_pwm: int, right_pwm: int) -> None:
        """
        Update physics simulation by dt seconds.

        Args:
            dt: Time step in seconds
            left_pwm: Left motor PWM value (-255 to +255)
            right_pwm: Right motor PWM value (-255 to +255)
        """
        pass

    @abstractmethod
    def get_sensor_reading(self) -> SensorReading:
        """
        Get current sensor reading with configured delay and noise.

        Sensor readings are delayed by the configured delay_ms value,
        simulating real communication latency.

        Returns:
            SensorReading containing distance and orientation data
        """
        pass

    @abstractmethod
    def get_position(self) -> Tuple[float, float, float]:
        """
        Get current robot position and orientation.

        Returns:
            Tuple of (x, y, yaw) where:
            - x, y: position in cm
            - yaw: orientation in degrees (-180 to +180)
        """
        pass

    @abstractmethod
    def get_velocity(self) -> Tuple[float, float, float]:
        """
        Get current robot velocity.

        Returns:
            Tuple of (vx, vy, omega) where:
            - vx, vy: linear velocity in cm/s
            - omega: angular velocity in deg/s
        """
        pass

    @abstractmethod
    def reset_position(self, x: float, y: float, yaw: float) -> None:
        """
        Reset robot position and orientation.

        Args:
            x: X position in cm
            y: Y position in cm
            yaw: Orientation in degrees
        """
        pass

    @abstractmethod
    def is_out_of_bounds(self) -> bool:
        """
        Check if robot has left the simulation boundary.

        Returns:
            True if robot is out of bounds
        """
        pass
