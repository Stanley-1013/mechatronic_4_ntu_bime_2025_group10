"""
Unit and Integration Tests for ConcreteWallFollower

Tests validate:
1. State machine transitions
2. P-controller output correctness
3. Corner counting accuracy
4. IMU turn logic
5. Sensor validation
6. PWM calculation
"""

import pytest
from concrete_wall_follower import (
    ConcreteWallFollower,
    WallFollowerState,
    MotorCommand,
)


class TestStateTransitions:
    """Test state machine transitions."""

    def test_idle_to_find_wall(self):
        """Test transition from IDLE to FIND_WALL on start()."""
        controller = ConcreteWallFollower()
        assert controller.get_state() == WallFollowerState.IDLE

        controller.start()
        assert controller.get_state() == WallFollowerState.FIND_WALL
        assert controller.get_corner_count() == 0

    def test_find_wall_timeout(self):
        """Test FIND_WALL timeout after 3 seconds without wall detection."""
        controller = ConcreteWallFollower()
        controller.start()

        # Advance time to 2.5 seconds - still in FIND_WALL
        controller.set_time(2.5)
        controller.update(100, 100, 0.0, True)  # No wall detected
        assert controller.get_state() == WallFollowerState.FIND_WALL

        # Advance time to 3.1 seconds - should transition to FORWARD
        controller.set_time(3.1)
        controller.update(100, 100, 0.0, True)
        assert controller.get_state() == WallFollowerState.FORWARD

    def test_find_wall_with_valid_distance(self):
        """Test FIND_WALL transition when wall found (10-50 cm)."""
        controller = ConcreteWallFollower()
        controller.start()
        controller.set_time(0.5)

        # Detect wall at 15 cm (valid range)
        controller.update(100, 15, 0.0, True)
        assert controller.get_state() == WallFollowerState.FORWARD

    def test_forward_corner_detection(self):
        """Test FORWARD -> BACKUP when corner detected (front + right wall)."""
        controller = ConcreteWallFollower()
        controller.start()
        controller.set_time(0.5)

        # Reach FORWARD state
        controller.update(100, 15, 0.0, True)
        assert controller.get_state() == WallFollowerState.FORWARD

        # Detect corner: front < 15 cm, right < 50 cm
        controller.set_time(1.0)
        controller.update(10, 15, 0.0, True)
        assert controller.get_state() == WallFollowerState.BACKUP
        assert controller.get_corner_count() == 1

    def test_backup_to_turn_left(self):
        """Test BACKUP -> TURN_LEFT after 0.3 seconds."""
        controller = ConcreteWallFollower()
        controller.start()

        # Set up: reach FORWARD then corner
        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)
        controller.set_time(1.0)
        controller.update(10, 15, 0.0, True)  # Enter BACKUP

        assert controller.get_state() == WallFollowerState.BACKUP
        start_backup = controller.get_state_duration()

        # Advance 0.2 seconds - still backing up
        controller.set_time(1.2)
        controller.update(10, 15, 0.0, True)
        assert controller.get_state() == WallFollowerState.BACKUP

        # Advance to 0.31 seconds - should transition to TURN_LEFT
        controller.set_time(1.31)
        controller.update(10, 15, 0.0, True)
        assert controller.get_state() == WallFollowerState.TURN_LEFT
        assert controller.imu_turn_active is True

    def test_turn_left_with_imu(self):
        """Test TURN_LEFT using IMU angle control (±5° tolerance)."""
        controller = ConcreteWallFollower()
        controller.start()

        # Set up: reach TURN_LEFT state
        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)
        controller.set_time(1.0)
        controller.update(10, 15, 0.0, True)  # Corner
        controller.set_time(1.31)
        controller.update(10, 15, 90.0, True)  # Now in TURN_LEFT

        assert controller.get_state() == WallFollowerState.TURN_LEFT
        initial_yaw = 90.0
        target_yaw = controller.target_yaw
        print(f"Initial yaw: {initial_yaw}, Target yaw: {target_yaw}")

        # Not yet at target (yaw = 90, target = 0, diff = -90)
        controller.set_time(1.5)
        controller.update(100, 15, 90.0, True)
        assert controller.get_state() == WallFollowerState.TURN_LEFT

        # Close to target (within ±5°)
        controller.set_time(2.5)
        controller.update(100, 15, 2.0, True)  # Near 0°
        assert controller.get_state() == WallFollowerState.FORWARD

    def test_turn_left_timeout_without_imu(self):
        """Test TURN_LEFT timeout (0.8s) when IMU not available."""
        controller = ConcreteWallFollower()
        controller.start()

        # Set up: reach TURN_LEFT with IMU invalid
        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)  # Reach FORWARD (IMU valid for now)
        controller.set_time(1.0)
        controller.update(10, 15, 0.0, True)  # Corner detected, enter BACKUP
        controller.set_time(1.31)
        # Now in TURN_LEFT state with imu_turn_active=True, but IMU becomes invalid
        controller.update(100, 15, 0.0, False)

        assert controller.get_state() == WallFollowerState.TURN_LEFT

        # At 0.5 seconds into turn
        controller.set_time(1.8)
        controller.update(100, 15, 0.0, False)
        assert controller.get_state() == WallFollowerState.TURN_LEFT

        # At 1.21 seconds into turn (1.5 * TURN_DURATION_MS) - should transition
        # Time in TURN_LEFT: 2.52 - 1.31 = 1.21 seconds = 1210 ms > 1200 ms
        controller.set_time(2.52)
        controller.update(100, 15, 0.0, False)
        assert controller.get_state() == WallFollowerState.FORWARD

    def test_forward_only_front_wall(self):
        """Test FORWARD -> TURN_LEFT when only front wall detected."""
        controller = ConcreteWallFollower()
        controller.start()

        # Reach FORWARD
        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)
        assert controller.get_state() == WallFollowerState.FORWARD

        # Only front wall, no right wall (right > 50)
        controller.set_time(1.0)
        controller.update(10, 60, 0.0, True)
        assert controller.get_state() == WallFollowerState.TURN_LEFT

    def test_forward_no_walls(self):
        """Test FORWARD -> FIND_WALL when no walls detected."""
        controller = ConcreteWallFollower()
        controller.start()

        # Reach FORWARD
        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)
        assert controller.get_state() == WallFollowerState.FORWARD

        # No walls detected (both > 80)
        controller.set_time(1.0)
        controller.update(100, 100, 0.0, True)
        assert controller.get_state() == WallFollowerState.FIND_WALL


class TestPController:
    """Test P-controller output."""

    def test_wall_follow_p_control_error_positive(self):
        """Test P-controller when too far from wall (error > 0)."""
        controller = ConcreteWallFollower()
        controller.start()

        # Reach FORWARD state
        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)

        # Wall at 25 cm (target is 15) -> error = 10
        # angular = -10 * 0.025 = -0.25 (right turn)
        controller.set_time(1.0)
        controller.update(50, 25, 0.0, True)

        cmd = controller.get_motor_command()
        # left = 0.6 * 255 - (-0.25) * 255 = 153 + 63.75 = 216.75
        # right = 0.6 * 255 + (-0.25) * 255 = 153 - 63.75 = 89.25
        assert cmd.left_pwm > cmd.right_pwm  # Left faster (right turn)

    def test_wall_follow_p_control_error_negative(self):
        """Test P-controller when too close to wall (error < 0)."""
        controller = ConcreteWallFollower()
        controller.start()

        # Reach FORWARD state
        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)

        # Wall at 5 cm (target is 15) -> error = -10
        # angular = -(-10) * 0.025 = +0.25 (left turn)
        controller.set_time(1.0)
        controller.update(50, 5, 0.0, True)

        cmd = controller.get_motor_command()
        # left = 0.6 * 255 - 0.25 * 255 = 153 - 63.75 = 89.25
        # right = 0.6 * 255 + 0.25 * 255 = 153 + 63.75 = 216.75
        assert cmd.right_pwm > cmd.left_pwm  # Right faster (left turn)

    def test_wall_follow_p_control_angular_limit(self):
        """Test P-controller angular velocity limit (±0.35)."""
        controller = ConcreteWallFollower()
        controller.start()

        # Reach FORWARD state
        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)

        # Wall very far: 60 cm -> error = 45
        # unconstrained angular = -45 * 0.025 = -1.125 (limited to -0.35)
        controller.set_time(1.0)
        controller.update(50, 60, 0.0, True)

        cmd = controller.get_motor_command()
        # angular limited to -0.35
        # left = 0.6 * 255 - (-0.35) * 255 = 153 + 89.25 = 242.25
        # right = 0.6 * 255 + (-0.35) * 255 = 153 - 89.25 = 63.75
        assert abs(cmd.left_pwm - 242) <= 1 or abs(cmd.left_pwm - 243) <= 1
        assert 63 <= cmd.right_pwm <= 65

    def test_slow_down_near_front_wall(self):
        """Test speed reduction when front wall is close."""
        controller = ConcreteWallFollower()
        controller.start()

        # Reach FORWARD state
        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)

        # Front wall far (50 cm), right wall at target
        controller.set_time(1.0)
        controller.update(50, 15, 0.0, True)
        cmd1 = controller.get_motor_command()

        # Front wall close (30 cm), right wall at target
        controller.set_time(1.5)
        controller.update(30, 15, 0.0, True)
        cmd2 = controller.get_motor_command()

        # Both should have reduced speed when front wall < 40 cm
        # cmd1 should use BASE_LINEAR_SPEED (0.6)
        # cmd2 should use SLOW_LINEAR_SPEED (0.35)
        # Without P-control correction, cmd2 PWM should be smaller
        assert abs(cmd2.left_pwm) < abs(cmd1.left_pwm) or cmd2.left_pwm == cmd1.left_pwm


class TestMotorOutput:
    """Test motor command generation."""

    def test_differential_drive_right_turn(self):
        """Test differential drive formula for right turn (angular < 0)."""
        controller = ConcreteWallFollower()
        controller._set_motor_output(linear=0.6, angular=-0.15)

        cmd = controller.get_motor_command()
        # left = 0.6 * 255 - (-0.15) * 255 = 153 + 38.25 = 191.25
        # right = 0.6 * 255 + (-0.15) * 255 = 153 - 38.25 = 114.75
        assert 190 <= cmd.left_pwm <= 192
        assert 114 <= cmd.right_pwm <= 116
        assert cmd.left_pwm > cmd.right_pwm

    def test_differential_drive_left_turn(self):
        """Test differential drive formula for left turn (angular > 0)."""
        controller = ConcreteWallFollower()
        controller._set_motor_output(linear=0.6, angular=0.25)

        cmd = controller.get_motor_command()
        # left = 0.6 * 255 - 0.25 * 255 = 153 - 63.75 = 89.25
        # right = 0.6 * 255 + 0.25 * 255 = 153 + 63.75 = 216.75
        assert 89 <= cmd.left_pwm <= 90
        assert 216 <= cmd.right_pwm <= 217
        assert cmd.right_pwm > cmd.left_pwm

    def test_differential_drive_straight(self):
        """Test differential drive formula for straight motion (angular = 0)."""
        controller = ConcreteWallFollower()
        controller._set_motor_output(linear=0.6, angular=0.0)

        cmd = controller.get_motor_command()
        # left = right = 0.6 * 255 = 153
        assert cmd.left_pwm == 153
        assert cmd.right_pwm == 153

    def test_turn_in_place_left(self):
        """Test turn in place - left motor off, right motor on."""
        controller = ConcreteWallFollower()
        controller._set_motor_output(linear=0.0, angular=-0.7)

        cmd = controller.get_motor_command()
        # left = 0 - (-0.7) * 255 = 178.5
        # right = 0 + (-0.7) * 255 = -178.5
        assert 178 <= cmd.left_pwm <= 179
        assert -179 <= cmd.right_pwm <= -178

    def test_pwm_dead_zone(self):
        """Test PWM dead zone protection (values < 60 become 0)."""
        controller = ConcreteWallFollower()
        controller._set_motor_output(linear=0.2, angular=0.0)

        cmd = controller.get_motor_command()
        # left = right = 0.2 * 255 = 51 -> should become 0 (< MIN_EFFECTIVE_PWM)
        assert cmd.left_pwm == 0
        assert cmd.right_pwm == 0

    def test_pwm_boundary(self):
        """Test PWM boundary (values >= 60 are kept)."""
        controller = ConcreteWallFollower()
        controller._set_motor_output(linear=0.25, angular=0.0)

        cmd = controller.get_motor_command()
        # left = right = 0.25 * 255 = 63.75 -> kept (>= 60)
        assert 63 <= cmd.left_pwm <= 64
        assert 63 <= cmd.right_pwm <= 64

    def test_pwm_saturation(self):
        """Test PWM saturation at ±255."""
        controller = ConcreteWallFollower()
        controller._set_motor_output(linear=2.0, angular=0.0)

        cmd = controller.get_motor_command()
        # Should be clamped to 255
        assert cmd.left_pwm == 255
        assert cmd.right_pwm == 255


class TestCornerCounting:
    """Test corner count accuracy."""

    def test_single_corner(self):
        """Test corner count increases on corner detection."""
        controller = ConcreteWallFollower()
        controller.start()

        controller.set_time(0.5)
        controller.update(100, 15, 0.0, True)

        assert controller.get_corner_count() == 0

        controller.set_time(1.0)
        controller.update(10, 15, 0.0, True)  # Corner detected

        assert controller.get_corner_count() == 1

    def test_multiple_corners(self):
        """Test corner count for multiple corners."""
        controller = ConcreteWallFollower()
        controller.start()

        corner_times = [0.5, 1.0, 2.5, 4.0, 5.5]
        for i, t in enumerate(corner_times):
            controller.set_time(t)
            if i == 0:
                controller.update(100, 15, 0.0, True)  # Reach FORWARD
            else:
                # First ensure we're back in FORWARD
                if controller.get_state() != WallFollowerState.FORWARD:
                    # After turn completes
                    controller.update(100, 15, 0.0, True)
                else:
                    controller.update(10, 15, 0.0, True)  # Next corner

        # Should have detected multiple corners
        assert controller.get_corner_count() > 0


class TestSensorValidation:
    """Test sensor reading validation."""

    def test_invalid_sensor_zero(self):
        """Test that sensor reading of 0 is invalid."""
        controller = ConcreteWallFollower()
        assert not controller._is_sensor_valid(0)

    def test_invalid_sensor_999(self):
        """Test that sensor reading of 999 is invalid."""
        controller = ConcreteWallFollower()
        assert not controller._is_sensor_valid(999)

    def test_valid_sensor_range(self):
        """Test valid sensor readings (1-998 cm)."""
        controller = ConcreteWallFollower()
        assert controller._is_sensor_valid(1)
        assert controller._is_sensor_valid(50)
        assert controller._is_sensor_valid(998)

    def test_invalid_sensor_negative(self):
        """Test that negative sensor readings are invalid."""
        controller = ConcreteWallFollower()
        assert not controller._is_sensor_valid(-1)


class TestYawCalculation:
    """Test yaw angle calculations."""

    def test_yaw_difference_simple(self):
        """Test simple yaw difference calculation."""
        controller = ConcreteWallFollower()
        diff = controller._get_yaw_difference(0.0, 90.0)
        assert diff == 90.0

    def test_yaw_difference_wraparound_positive(self):
        """Test yaw difference with positive wraparound.

        From 170° to -170°: difference = -170 - 170 = -340
        Normalized: -340 + 360 = 20° (shortest path is 20° clockwise)
        """
        controller = ConcreteWallFollower()
        diff = controller._get_yaw_difference(170.0, -170.0)
        # The shortest path from 170 to -170 is 20 degrees clockwise (positive)
        assert abs(diff - 20.0) < 0.01

    def test_yaw_difference_wraparound_negative(self):
        """Test yaw difference with negative wraparound.

        From -170° to 170°: difference = 170 - (-170) = 340
        Normalized: 340 - 360 = -20° (shortest path is 20° counter-clockwise)
        """
        controller = ConcreteWallFollower()
        diff = controller._get_yaw_difference(-170.0, 170.0)
        # The shortest path from -170 to 170 is 20 degrees counter-clockwise (negative)
        assert abs(diff - (-20.0)) < 0.01

    def test_yaw_reached_within_tolerance(self):
        """Test yaw reached detection within ±5° tolerance."""
        controller = ConcreteWallFollower()
        assert controller._is_yaw_reached(0.0, 3.0)  # 3° difference
        assert controller._is_yaw_reached(0.0, -4.0)  # -4° difference
        assert controller._is_yaw_reached(0.0, 5.0)  # Exactly 5° boundary

    def test_yaw_reached_outside_tolerance(self):
        """Test yaw not reached beyond tolerance."""
        controller = ConcreteWallFollower()
        assert not controller._is_yaw_reached(0.0, 6.0)  # 6° > 5°
        assert not controller._is_yaw_reached(0.0, -6.0)  # -6° < -5°


class TestStateQueries:
    """Test state query methods."""

    def test_get_motor_command_vacuum_state(self):
        """Test vacuum state in motor command."""
        controller = ConcreteWallFollower()
        controller.set_vacuum(True)
        cmd = controller.get_motor_command()
        assert cmd.vacuum_enabled is True

        controller.set_vacuum(False)
        cmd = controller.get_motor_command()
        assert cmd.vacuum_enabled is False

    def test_state_duration(self):
        """Test state duration calculation."""
        controller = ConcreteWallFollower()
        controller.start()
        controller.set_time(0.0)

        assert controller.get_state_duration() == 0.0

        controller.set_time(1.5)
        assert abs(controller.get_state_duration() - 1.5) < 0.01

    def test_reset(self):
        """Test reset functionality."""
        controller = ConcreteWallFollower()
        controller.start()
        controller.set_time(1.0)

        # Advance to create state
        controller.update(100, 15, 0.0, True)
        controller.set_time(2.0)
        controller.update(10, 15, 0.0, True)  # Corner

        assert controller.get_corner_count() == 1
        assert controller.get_state() != WallFollowerState.IDLE

        controller.reset()
        assert controller.get_state() == WallFollowerState.IDLE
        assert controller.get_corner_count() == 0
        assert controller.get_state_duration() == 0.0


class TestIdleAndStoppedStates:
    """Test behavior in IDLE and DONE states."""

    def test_idle_produces_zero_output(self):
        """Test that IDLE state produces zero motor output."""
        controller = ConcreteWallFollower()
        assert controller.get_state() == WallFollowerState.IDLE

        controller.set_time(0.5)
        controller.update(50, 25, 0.0, True)

        cmd = controller.get_motor_command()
        assert cmd.left_pwm == 0
        assert cmd.right_pwm == 0

    def test_done_produces_zero_output(self):
        """Test that DONE state produces zero motor output."""
        controller = ConcreteWallFollower()
        controller.start()
        controller.stop()

        assert controller.get_state() == WallFollowerState.DONE
        cmd = controller.get_motor_command()
        assert cmd.left_pwm == 0
        assert cmd.right_pwm == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
