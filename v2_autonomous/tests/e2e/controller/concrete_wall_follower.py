"""
Concrete WallFollower Implementation - Continuous Differential Drive Control

This module provides a Python implementation of continuous differential drive
control for wall-following. It replaces the state machine approach with
proportional control feedback, maintaining the same external interface.

Control Logic:
    - Differential drive: left = linear * MAX_PWM - angular * MAX_PWM
                         right = linear * MAX_PWM + angular * MAX_PWM
    - angular < 0: right turn (left motor faster)
    - angular > 0: left turn (right motor faster)
"""

import time
from .wall_follower_wrapper import (
    WallFollowerWrapper,
    WallFollowerState,
    MotorCommand,
)


# ==================== Constants ====================

# Wall-following parameters (matching C++ implementation)
TARGET_RIGHT_DISTANCE = 15          # cm
FRONT_STOP_DISTANCE = 20            # cm (corner detection threshold)
FRONT_SLOW_DISTANCE = 40            # cm
CORNER_RIGHT_DISTANCE = 30          # cm (corner right threshold)

# Speed parameters (normalized: -1.0 ~ +1.0)
BASE_LINEAR_SPEED = 0.6
BACKUP_SPEED = -0.4
TURN_ANGULAR_SPEED = 0.7

# Control parameters
WALL_FOLLOW_KP = 0.03               # P-controller gain for distance control
FIND_WALL_ANGULAR = 0.15            # Right turn speed to find wall

# Time constants (seconds)
BACKUP_DURATION = 0.3               # Backup time when corner detected
TURN_TIMEOUT = 3.0                  # Maximum turn time (safety limit)

# PWM constants
MAX_PWM = 255
MIN_EFFECTIVE_PWM = 60

# Angle control constants
YAW_TOLERANCE = 5.0                 # degrees
TURN_TARGET_YAW_OFFSET = -90.0      # degrees


class ConcreteWallFollower(WallFollowerWrapper):
    """
    Concrete implementation of wall-following controller using continuous
    differential drive control instead of state machines.

    Uses proportional feedback on distance measurements to maintain continuous
    movement with appropriate speed and direction adjustments.
    """

    def __init__(self):
        """Initialize the controller with default state."""
        self.left_pwm = 0
        self.right_pwm = 0
        self.vacuum_on = True
        self.corner_count = 0
        self.current_time = 0.0

        # Internal state for continuous control (simplified)
        self.running = False

        # Sensor history for filtering (store last 3 valid readings)
        # Initialize with None to indicate no valid readings yet
        self.front_history = [None, None, None]
        self.right_history = [None, None, None]

        # Corner handling state (locked during maneuver)
        self.corner_locked = False
        self.backup_start_time = None
        self.backup_yaw = None

    # ==================== Public Interface ====================

    def start(self) -> None:
        """
        Start the wall-following controller.

        Initializes to running state.
        """
        self.running = True
        self.corner_count = 0

    def stop(self) -> None:
        """
        Stop the controller gracefully.

        Outputs zero motor commands.
        """
        self.running = False
        self.left_pwm = 0
        self.right_pwm = 0

    def reset(self) -> None:
        """
        Reset controller to initial state.

        Clears all internal timers and resets corner count.
        """
        self.running = False
        self.corner_count = 0
        self.current_time = 0.0
        self.left_pwm = 0
        self.right_pwm = 0
        self.corner_locked = False
        self.backup_start_time = None
        self.backup_yaw = None
        self.front_history = [None, None, None]
        self.right_history = [None, None, None]

    def trigger_avoid_red(self) -> None:
        """
        Trigger red avoidance behavior.

        Currently a placeholder for future red detection integration.
        """
        pass

    def set_time(self, current_time: float) -> None:
        """
        Set internal controller time (for deterministic testing).

        Args:
            current_time: Simulated time in seconds since epoch
        """
        self.current_time = current_time

    def set_vacuum(self, enabled: bool) -> None:
        """
        Manually set vacuum motor state.

        Args:
            enabled: True to enable vacuum, False to disable
        """
        self.vacuum_on = enabled

    def update(self, front_distance: int, right_distance: int,
               yaw: float, imu_valid: bool) -> None:
        """
        Update controller with continuous differential drive control.

        Strategy:
        1. Filter sensor readings using median filter
        2. Detect corners (both front and right walls)
        3. Execute continuous control:
           - Corner backup & turn: backup then turn in place
           - Front obstacle: slow down based on distance
           - Right wall: P-controller to maintain target distance
           - No walls: gentle right turn to find wall

        Args:
            front_distance: Front ultrasonic sensor reading in cm
            right_distance: Right ultrasonic sensor reading in cm
            yaw: Current orientation from IMU in degrees
            imu_valid: Whether IMU data is valid
        """
        if not self.running:
            self.left_pwm = 0
            self.right_pwm = 0
            return

        # 1. Filter sensor readings
        front = self._filter_sensor(front_distance, self.front_history)
        right = self._filter_sensor(right_distance, self.right_history)

        # 2. Corner handling (locked state - ignore sensors during maneuver)
        if self.corner_locked:
            elapsed = self.current_time - self.backup_start_time

            # Backup phase (time-based)
            if elapsed < BACKUP_DURATION:
                self._set_motor_output(BACKUP_SPEED, 0)
                return

            # Turn phase (IMU angle-based with timeout)
            turn_elapsed = elapsed - BACKUP_DURATION
            turned = self._angle_diff(yaw, self.backup_yaw)

            # Continue turning if: not turned enough AND not timed out
            if turned < 85.0 and turn_elapsed < TURN_TIMEOUT:
                self._set_motor_output(0, TURN_ANGULAR_SPEED)
                return

            # Corner maneuver complete - unlock (either turned enough or timeout)
            self.corner_locked = False
            self.backup_start_time = None
            self.backup_yaw = None

        # 3. Corner detection (only when not locked)
        if front < FRONT_STOP_DISTANCE and right < CORNER_RIGHT_DISTANCE:
            # Corner detected - lock and start maneuver
            self.corner_locked = True
            self.backup_start_time = self.current_time
            self.backup_yaw = yaw
            self.corner_count += 1
            self._set_motor_output(BACKUP_SPEED, 0)
            return

        # 4. Front obstacle control
        # Map front distance to linear speed: closer = slower
        linear = BASE_LINEAR_SPEED
        if front <= 10:
            # Too close - stop (corner detection should handle this)
            linear = 0.0
        elif front < FRONT_SLOW_DISTANCE:
            # Linear interpolation: 40cm -> 0.6, 20cm -> 0.3
            linear = self._map_range(front, FRONT_STOP_DISTANCE, FRONT_SLOW_DISTANCE,
                                     0.3, BASE_LINEAR_SPEED)

        # 5. Right wall distance control (P-controller)
        angular = 0.0

        if 5 < right < 80:
            # Within effective wall-following range
            # error > 0: too far from wall -> turn right (negative angular)
            # error < 0: too close to wall -> turn left (positive angular)
            error = right - TARGET_RIGHT_DISTANCE
            angular = -error * WALL_FOLLOW_KP
            angular = self._constrain(angular, -0.35, 0.35)
        elif right >= 80:
            # No wall detected - right turn to find wall
            angular = -FIND_WALL_ANGULAR
        else:
            # Too close to wall - left turn to avoid
            angular = 0.2

        self._set_motor_output(linear, angular)

    @property
    def state(self) -> WallFollowerState:
        """
        Get current controller state (property for compatibility).

        Returns simplified state: FORWARD when running, DONE when stopped.
        """
        if self.running:
            return WallFollowerState.FORWARD
        else:
            return WallFollowerState.DONE

    def get_state(self) -> WallFollowerState:
        """
        Get current controller state.

        Returns simplified state: RUNNING or STOPPED.
        """
        return self.state

    def get_motor_command(self) -> MotorCommand:
        """
        Get current motor command output.

        Returns:
            MotorCommand containing PWM values and vacuum state
        """
        return MotorCommand(
            left_pwm=self.left_pwm,
            right_pwm=self.right_pwm,
            vacuum_enabled=self.vacuum_on
        )

    def get_corner_count(self) -> int:
        """
        Get number of corners detected so far.

        Returns:
            Number of corners (0-255)
        """
        return self.corner_count

    def get_state_duration(self) -> float:
        """
        Get elapsed time since controller started.

        Returns:
            Duration in seconds
        """
        if self.running:
            return self.current_time
        return 0.0

    # ==================== Helper Functions ====================

    def _filter_sensor(self, value: int, history: list) -> int:
        """
        Median filter for sensor readings.

        Maintains a history of last 3 readings and returns median.
        Rejects invalid readings (0 or >= 500).

        Args:
            value: New sensor reading
            history: List of 3 previous readings

        Returns:
            Median of valid readings, or raw value if no history
        """
        # Accept valid readings (positive and less than 500)
        if 0 < value < 500:
            history.pop(0)
            history.append(value)

        # Get valid readings from history
        valid = [v for v in history if v is not None]

        if not valid:
            # No valid history yet - return raw value or max range
            return value if 0 < value < 500 else 500

        # Return median of valid readings
        valid.sort()
        return valid[len(valid) // 2]

    def _map_range(self, x: float, in_min: float, in_max: float,
                   out_min: float, out_max: float) -> float:
        """
        Linear mapping from input range to output range.

        Args:
            x: Input value
            in_min: Minimum input
            in_max: Maximum input
            out_min: Minimum output
            out_max: Maximum output

        Returns:
            Mapped output value
        """
        if in_max == in_min:
            return out_min
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def _set_motor_output(self, linear: float, angular: float) -> None:
        """
        Calculate motor PWM from linear and angular velocities.

        Differential drive formula:
            left_pwm = linear * MAX_PWM - angular * MAX_PWM
            right_pwm = linear * MAX_PWM + angular * MAX_PWM

        Notes:
        - angular < 0: right turn (left motor faster)
        - angular > 0: left turn (right motor faster)
        - Dead zone protection: |PWM| < MIN_EFFECTIVE_PWM -> set to 0

        Args:
            linear: Linear velocity (-1.0 to +1.0)
            angular: Angular velocity (-1.0 to +1.0)
        """
        # Calculate raw PWM values
        left_pwm = int(linear * MAX_PWM) - int(angular * MAX_PWM)
        right_pwm = int(linear * MAX_PWM) + int(angular * MAX_PWM)

        # Apply constraints and dead zone
        self.left_pwm = self._constrain_pwm(left_pwm)
        self.right_pwm = self._constrain_pwm(right_pwm)

    def _constrain_pwm(self, pwm: int) -> int:
        """
        Constrain PWM value to valid range with dead zone protection.

        - Range: -255 to +255
        - Dead zone: |PWM| < MIN_EFFECTIVE_PWM -> set to 0

        Args:
            pwm: Raw PWM value

        Returns:
            Constrained PWM value
        """
        # First constrain to [-MAX_PWM, +MAX_PWM]
        constrained = self._constrain(pwm, -MAX_PWM, MAX_PWM)

        # Dead zone protection
        if abs(constrained) < MIN_EFFECTIVE_PWM:
            return 0

        return constrained

    @staticmethod
    def _constrain(value: float, min_val: float, max_val: float) -> float:
        """
        Constrain value to range [min_val, max_val].

        Args:
            value: Value to constrain
            min_val: Minimum value
            max_val: Maximum value

        Returns:
            Constrained value
        """
        if value < min_val:
            return min_val
        if value > max_val:
            return max_val
        return value

    @staticmethod
    def _angle_diff(current_yaw: float, start_yaw: float) -> float:
        """
        Calculate how many degrees turned (absolute value).

        Handles angle wraparound correctly.

        Args:
            current_yaw: Current yaw angle in degrees
            start_yaw: Starting yaw angle in degrees

        Returns:
            Absolute degrees turned (always positive)
        """
        diff = current_yaw - start_yaw
        # Normalize to [-180, 180]
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        # Return absolute value
        return abs(diff)
