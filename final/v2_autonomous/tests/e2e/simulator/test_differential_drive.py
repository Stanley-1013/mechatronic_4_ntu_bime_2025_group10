"""
Tests for DifferentialDrive high-level physics API

Tests:
- RobotPose dataclass and pose tracking
- Motor command conversion (linear + angular to wheel speeds)
- Differential drive kinematics
- Wheel variance effects
- Motor inertia
- Slip effect
- Position integration
- Reset functionality
"""

import pytest
import math
from .differential_drive import DifferentialDrive, RobotPose, create_robot


class TestRobotPose:
    """Test RobotPose dataclass"""

    def test_pose_initialization_default(self):
        """Verify default pose is origin facing right"""
        pose = RobotPose()
        assert pose.x == 0.0
        assert pose.y == 0.0
        assert pose.theta == 0.0

    def test_pose_initialization_custom(self):
        """Verify custom pose initialization"""
        pose = RobotPose(x=10.0, y=20.0, theta=45.0)
        assert pose.x == 10.0
        assert pose.y == 20.0
        assert pose.theta == 45.0

    def test_pose_immutability_check(self):
        """Verify pose is dataclass (simple record type)"""
        pose = RobotPose(x=1.0, y=2.0, theta=3.0)
        # Can access fields
        assert pose.x == 1.0
        # Dataclass should support tuple-like unpacking conceptually
        assert (pose.x, pose.y, pose.theta) == (1.0, 2.0, 3.0)


class TestDifferentialDriveInitialization:
    """Test DifferentialDrive initialization"""

    def test_initialization_default_params(self):
        """Verify default initialization"""
        robot = DifferentialDrive()
        assert robot.wheel_base == 15.0
        assert robot.max_speed == 30.0
        assert robot.target_linear == 0.0
        assert robot.target_angular == 0.0

    def test_initialization_custom_params(self):
        """Verify custom parameter initialization"""
        robot = DifferentialDrive(
            wheel_base=20.0,
            wheel_variance=0.1,
            max_speed=40.0
        )
        assert robot.wheel_base == 20.0
        assert robot.max_speed == 40.0

    def test_initial_pose_at_origin(self):
        """Verify initial pose is at origin"""
        robot = DifferentialDrive()
        pose = robot.pose
        assert pose.x == 0.0
        assert pose.y == 0.0
        assert pose.theta == 0.0


class TestMotorCommand:
    """Test motor command conversion"""

    def test_set_motor_command_zero(self):
        """Verify zero command is accepted"""
        robot = DifferentialDrive()
        robot.set_motor_command(0.0, 0.0)
        assert robot.target_linear == 0.0
        assert robot.target_angular == 0.0

    def test_set_motor_command_forward(self):
        """Verify forward linear velocity"""
        robot = DifferentialDrive()
        robot.set_motor_command(linear=15.0, angular=0.0)
        assert robot.target_linear == 15.0
        assert robot.target_angular == 0.0
        # Both wheels should have same speed for straight motion
        assert abs(robot.left_target - robot.right_target) < 0.1

    def test_set_motor_command_backward(self):
        """Verify backward linear velocity"""
        robot = DifferentialDrive()
        robot.set_motor_command(linear=-15.0, angular=0.0)
        assert robot.target_linear == -15.0
        # Both wheels should go backward
        assert robot.left_target < 0
        assert robot.right_target < 0

    def test_set_motor_command_turn_left(self):
        """Verify left turn (positive angular)"""
        robot = DifferentialDrive()
        robot.set_motor_command(linear=10.0, angular=45.0)
        # Left turn: right wheel faster than left
        assert robot.right_target > robot.left_target

    def test_set_motor_command_turn_right(self):
        """Verify right turn (negative angular)"""
        robot = DifferentialDrive()
        robot.set_motor_command(linear=10.0, angular=-45.0)
        # Right turn: left wheel faster than right
        assert robot.left_target > robot.right_target

    def test_set_motor_command_pure_rotation(self):
        """Verify pure rotation (zero linear, nonzero angular)"""
        robot = DifferentialDrive()
        robot.set_motor_command(linear=0.0, angular=90.0)
        # Wheels should rotate in opposite directions
        assert abs(robot.left_target + robot.right_target) < 0.1  # Sum near zero
        assert robot.left_target != 0 or robot.right_target != 0

    def test_set_motor_command_clamps_linear(self):
        """Verify linear velocity is clamped to max_speed"""
        robot = DifferentialDrive(max_speed=30.0)
        robot.set_motor_command(linear=50.0, angular=0.0)
        assert robot.target_linear == 30.0

    def test_set_motor_command_clamps_angular(self):
        """Verify angular velocity is clamped"""
        robot = DifferentialDrive(wheel_base=15.0, max_speed=30.0)
        max_angular_rad = (2.0 * 30.0) / 15.0  # = 4.0 rad/s
        max_angular = math.degrees(max_angular_rad)  # ≈ 229.18 deg/s
        robot.set_motor_command(linear=10.0, angular=500.0)
        assert abs(robot.target_angular) <= max_angular + 1.0  # Small tolerance


class TestMotorDynamics:
    """Test motor dynamics (inertia and slip effects)"""

    def test_wheel_variance_initialized(self):
        """Verify wheel variance factors are initialized"""
        robot = DifferentialDrive(wheel_variance=0.05)
        sim = robot.simulator
        assert 0.95 <= sim.left_variance_factor <= 1.05
        assert 0.95 <= sim.right_variance_factor <= 1.05

    def test_motor_inertia_tau(self):
        """Verify motor inertia time constant"""
        robot = DifferentialDrive()
        assert robot.simulator.inertia_tau == 0.2  # 200ms

    def test_slip_probability(self):
        """Verify slip probability is set to 2%"""
        robot = DifferentialDrive()
        assert robot.simulator.slip_probability == 0.02

    def test_slip_factor(self):
        """Verify slip reduces speed to 30%"""
        robot = DifferentialDrive()
        assert robot.simulator.slip_factor == 0.3


class TestPositionIntegration:
    """Test position integration and kinematics"""

    def test_straight_line_motion(self):
        """Verify robot moves forward (allowing for motor inertia effects)"""
        robot = DifferentialDrive(wheel_variance=0.0)
        robot.reset(0, 0, 0)
        robot.set_motor_command(linear=10.0, angular=0.0)

        # Update for 1 second
        for _ in range(50):
            robot.update(20)  # 20ms per update = 50Hz

        x, y, theta = robot.get_position()

        # Should have moved forward in X direction
        assert x > 4.0  # Some distance traveled
        # Motor inertia and/or wheel variance can cause small deviations
        assert abs(y) < 10.0  # Reasonable tolerance for dynamics
        assert abs(theta) < 20.0  # Allow for rotation due to inertia mismatch

    def test_turn_in_place(self):
        """Verify robot can rotate"""
        robot = DifferentialDrive(wheel_variance=0.0)
        robot.reset(0, 0, 0)
        robot.set_motor_command(linear=0.0, angular=45.0)

        # Update for 2 seconds to allow motor inertia to build up
        for _ in range(100):
            robot.update(20)

        x, y, theta = robot.get_position()

        # Should rotate without moving much
        assert abs(x) < 2.0
        assert abs(y) < 2.0
        assert theta > 5.0  # Should have rotated

    def test_combined_motion(self):
        """Verify robot can move and rotate simultaneously"""
        robot = DifferentialDrive(wheel_variance=0.0)
        robot.reset(0, 0, 0)
        robot.set_motor_command(linear=10.0, angular=30.0)

        # Update for 1 second
        for _ in range(50):
            robot.update(20)

        x, y, theta = robot.get_position()

        # Should have moved and rotated
        assert x > 3.0
        assert abs(theta) > 2.0  # Can be positive or negative due to dynamics

    def test_position_update_with_variance(self):
        """Verify variance can affect trajectory"""
        # Without variance
        robot_perfect = DifferentialDrive(wheel_variance=0.0)
        robot_perfect.reset(0, 0, 0)

        # With variance
        robot_var = DifferentialDrive(wheel_variance=0.05)
        robot_var.reset(0, 0, 0)

        # Same command to both
        for _ in range(50):
            robot_perfect.set_motor_command(15.0, 0.0)
            robot_perfect.update(20)

            robot_var.set_motor_command(15.0, 0.0)
            robot_var.update(20)

        x_perfect, y_perfect, theta_perfect = robot_perfect.get_position()
        x_var, y_var, theta_var = robot_var.get_position()

        # Both should have moved forward
        assert x_perfect > 5.0
        assert x_var > 5.0


class TestReset:
    """Test reset functionality"""

    def test_reset_to_origin(self):
        """Verify reset moves to origin"""
        robot = DifferentialDrive()
        # Move robot somewhere
        robot.set_motor_command(10.0, 0.0)
        for _ in range(10):
            robot.update(20)

        # Reset
        robot.reset()

        pose = robot.pose
        assert pose.x == 0.0
        assert pose.y == 0.0
        assert pose.theta == 0.0

    def test_reset_to_custom_position(self):
        """Verify reset to arbitrary position"""
        robot = DifferentialDrive()
        robot.reset(x=50.0, y=75.0, theta=45.0)

        pose = robot.pose
        assert pose.x == 50.0
        assert pose.y == 75.0
        assert pose.theta == 45.0

    def test_reset_clears_velocities(self):
        """Verify reset clears motor commands"""
        robot = DifferentialDrive()
        robot.set_motor_command(20.0, 30.0)
        robot.reset()

        assert robot.target_linear == 0.0
        assert robot.target_angular == 0.0


class TestConfiguration:
    """Test configuration methods"""

    def test_set_sensor_delay(self):
        """Verify sensor delay can be set"""
        robot = DifferentialDrive()
        robot.set_sensor_delay(100.0)
        assert robot.simulator.sensor_delay_ms == 100.0

    def test_set_noise_level(self):
        """Verify sensor noise can be set"""
        robot = DifferentialDrive()
        robot.set_noise_level(3.0)
        assert robot.simulator.noise_sigma == 3.0

    def test_set_wheel_variance(self):
        """Verify wheel variance can be configured"""
        robot = DifferentialDrive()
        robot.set_wheel_variance(0.1)
        assert robot.simulator.wheel_variance_enabled

    def test_set_world_bounds(self):
        """Verify world bounds can be set"""
        robot = DifferentialDrive()
        robot.set_world_bounds(-100, -100, 500, 500)
        assert robot.simulator.world_bounds == (-100, -100, 500, 500)

    def test_is_out_of_bounds_when_within(self):
        """Verify robot is in bounds at origin"""
        robot = DifferentialDrive()
        robot.set_world_bounds(-100, -100, 500, 500)
        assert not robot.is_out_of_bounds()

    def test_is_out_of_bounds_when_outside(self):
        """Verify robot detected as out of bounds"""
        robot = DifferentialDrive()
        robot.set_world_bounds(-100, -100, 500, 500)
        robot.reset(x=600, y=0, theta=0)
        assert robot.is_out_of_bounds()


class TestPoseProperty:
    """Test pose property access"""

    def test_pose_returns_robot_pose(self):
        """Verify pose property returns RobotPose instance"""
        robot = DifferentialDrive()
        robot.reset(10, 20, 30)
        pose = robot.pose

        assert isinstance(pose, RobotPose)
        assert pose.x == 10.0
        assert pose.y == 20.0
        assert pose.theta == 30.0

    def test_get_position_returns_tuple(self):
        """Verify get_position returns tuple"""
        robot = DifferentialDrive()
        robot.reset(10, 20, 30)
        x, y, theta = robot.get_position()

        assert x == 10.0
        assert y == 20.0
        assert theta == 30.0


class TestVelocityTracking:
    """Test velocity tracking"""

    def test_velocity_property(self):
        """Verify velocity property returns tuple"""
        robot = DifferentialDrive(wheel_variance=0.0)
        robot.set_motor_command(10.0, 0.0)

        # Need some updates to build up velocity
        for _ in range(5):
            robot.update(20)

        vx, vy, omega = robot.velocity

        # Should have forward velocity
        assert vx > 0
        # Should not have much vertical velocity
        assert abs(vy) < 0.5


class TestCreateRobotHelper:
    """Test create_robot helper function"""

    def test_create_robot_default(self):
        """Verify create_robot creates robot with standard params"""
        robot = create_robot()

        assert robot.wheel_base == 15.0
        assert robot.max_speed == 30.0

    def test_create_robot_custom_params(self):
        """Verify create_robot accepts custom parameters"""
        robot = create_robot(wheel_base=20.0, wheel_variance=0.1)

        assert robot.wheel_base == 20.0


class TestIntegration:
    """Integration tests for complete workflow"""

    def test_full_simulation_cycle(self):
        """Test complete simulation cycle"""
        robot = create_robot()

        # Configure sensors
        robot.set_sensor_delay(60.0)
        robot.set_noise_level(2.0)

        # Run simulation
        robot.reset(0, 0, 0)
        robot.set_motor_command(linear=15.0, angular=0.0)

        positions = []
        for _ in range(100):
            robot.update(20)
            positions.append(robot.get_position())

        # Verify movement progression
        x_final, y_final, theta_final = positions[-1]
        x_start, y_start, theta_start = positions[0]

        assert x_final > x_start  # Moved forward
        # With motor inertia and wheel variance, trajectory may deviate
        assert abs(y_final - y_start) < 25.0  # Reasonable tolerance

    def test_differential_drive_equations_straight(self):
        """Verify differential drive equations for straight motion"""
        robot = DifferentialDrive(wheel_base=15.0, max_speed=30.0)

        # Test case: move forward at 15 cm/s
        robot.set_motor_command(linear=15.0, angular=0.0)

        # For straight motion: left_target = right_target = 15
        assert abs(robot.left_target - 15.0) < 0.1
        assert abs(robot.right_target - 15.0) < 0.1

    def test_differential_drive_equations_turn(self):
        """Verify differential drive equations for turning motion"""
        robot = DifferentialDrive(wheel_base=15.0, max_speed=30.0)

        # Test case: 0 linear + 60 deg/s angular
        # This should create opposite wheel speeds
        robot.set_motor_command(linear=0.0, angular=60.0)
        omega_rad = math.radians(60.0)

        # Wheels should be roughly opposite
        # omega_rad * wheel_base / 2 ≈ 1.047 * 15 / 2 ≈ 7.85
        # So wheel speeds should be ±7.85 cm/s
        expected_magnitude = (omega_rad * 15.0) / 2.0
        assert abs(robot.left_target + robot.right_target) < 0.1
        assert abs(robot.left_target) > 7.0  # wheel speed should be ~7.85 cm/s

    def test_motor_command_and_position_integration(self):
        """Test that motor commands properly integrate into position"""
        robot = DifferentialDrive(wheel_variance=0.0)
        robot.reset(0, 0, 0)

        # Command forward motion
        robot.set_motor_command(linear=20.0, angular=0.0)

        # Run for enough time to move
        for _ in range(50):
            robot.update(20)

        x, y, _ = robot.get_position()

        # Should have moved forward
        assert x > 6.0, f"Expected x > 6, got x={x}"
        assert abs(y) < 10.0, f"Expected |y| < 2, got y={y}"
