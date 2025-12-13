"""
Wall Follower Controller Wrapper Interface

Provides a testing interface to the wall-following state machine.
Adapts sensor readings and motor commands for simulation testing.
"""

from abc import ABC, abstractmethod
from enum import IntEnum
from dataclasses import dataclass
from typing import Tuple, Optional


class WallFollowerState(IntEnum):
    """State enumeration matching Arduino implementation"""
    IDLE = 0x00
    FIND_WALL = 0x01
    FORWARD = 0x02
    BACKUP = 0x03
    TURN_LEFT = 0x04
    DONE = 0x05
    ERROR = 0xFF


@dataclass
class MotorCommand:
    """Motor command output from controller"""
    left_pwm: int  # -255 to +255
    right_pwm: int  # -255 to +255
    vacuum_enabled: bool


class WallFollowerWrapper(ABC):
    """
    Abstract interface for wall-following controller testing.

    Wraps the state machine logic and provides methods for feeding
    sensor data and retrieving motor commands.
    """

    @abstractmethod
    def start(self) -> None:
        """
        Start the wall-following controller.

        Initializes to FIND_WALL state and resets internal timers.
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        Stop the controller gracefully.

        Transitions to DONE state and outputs zero motor commands.
        """
        pass

    @abstractmethod
    def update(self, front_distance: int, right_distance: int,
               yaw: float, imu_valid: bool) -> None:
        """
        Update controller state machine with sensor readings.

        Args:
            front_distance: Front ultrasonic sensor reading in cm
            right_distance: Right ultrasonic sensor reading in cm
            yaw: Current orientation from IMU in degrees
            imu_valid: Whether IMU data is valid

        Note:
            Invalid sensor readings are indicated by 0 or 999.
            This matches the protocol definition from protocol.py
        """
        pass

    @abstractmethod
    def get_state(self) -> WallFollowerState:
        """
        Get current controller state.

        Returns:
            Current WallFollowerState enumeration value
        """
        pass

    @abstractmethod
    def get_motor_command(self) -> MotorCommand:
        """
        Get current motor command output.

        Returns:
            MotorCommand containing PWM values and vacuum state
        """
        pass

    @abstractmethod
    def get_corner_count(self) -> int:
        """
        Get number of corners detected so far.

        Returns:
            Number of corners (0-255)
        """
        pass

    @abstractmethod
    def get_state_duration(self) -> float:
        """
        Get how long controller has been in current state.

        Returns:
            Duration in seconds
        """
        pass

    @abstractmethod
    def set_time(self, current_time: float) -> None:
        """
        Set internal controller time (for deterministic testing).

        Args:
            current_time: Simulated time in seconds since epoch
        """
        pass

    @abstractmethod
    def trigger_avoid_red(self) -> None:
        """
        Trigger red avoidance behavior.

        This is a placeholder for future red detection integration.
        """
        pass

    @abstractmethod
    def set_vacuum(self, enabled: bool) -> None:
        """
        Manually set vacuum motor state.

        Args:
            enabled: True to enable vacuum, False to disable
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """
        Reset controller to initial state (IDLE).

        Clears all internal timers and resets corner count.
        """
        pass
