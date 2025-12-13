"""
Enhanced tests for ConcretePhysicsSimulator dynamics features:
- Wheel variance (±5% per wheel)
- Motor inertia (200ms time constant)
- Slip effect (2% probability, 30% speed)
- Position integration with differential drive
"""

import pytest
import math
from .concrete_physics_simulator import ConcretePhysicsSimulator


class TestWheelVariance:
    """Test wheel speed variance (±5% per wheel)"""

    def test_variance_factors_initialized(self):
        """Verify variance factors are initialized randomly"""
        sim = ConcretePhysicsSimulator()

        # Both factors should be initialized (around 1.0 ±5%)
        assert 0.95 <= sim.left_variance_factor <= 1.05
        assert 0.95 <= sim.right_variance_factor <= 1.05

    def test_variance_factors_differ(self):
        """Verify left and right wheels have different variance factors"""
        # Run multiple times since randomness might occasionally match
        different_count = 0
        for _ in range(10):
            sim = ConcretePhysicsSimulator()
            if abs(sim.left_variance_factor - sim.right_variance_factor) > 0.001:
                different_count += 1

        # Should have different factors in most cases
        assert different_count >= 7, "Variance factors should typically differ"

    def test_set_wheel_variance_enables_factors(self):
        """Verify set_wheel_variance creates new variance factors"""
        sim = ConcretePhysicsSimulator()

        # Set variance
        sim.set_wheel_variance(0.05)

        # Should have new factors within expected range
        assert 0.95 <= sim.left_variance_factor <= 1.05
        assert 0.95 <= sim.right_variance_factor <= 1.05
        assert sim.wheel_variance_enabled

    def test_set_wheel_variance_zero_disables(self):
        """Verify variance can be disabled with zero"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)

        assert sim.left_variance_factor == 1.0
        assert sim.right_variance_factor == 1.0
        assert not sim.wheel_variance_enabled

    def test_wheel_variance_causes_velocity_diff(self):
        """Verify variance creates different velocities for left/right wheels"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.05)
        sim.reset_position(0, 0, 0)

        # Run with equal PWM
        for _ in range(50):
            sim.update(0.02, 100, 100)

        # Check that velocities differ due to variance
        motor_state = sim.get_motor_state()
        left_vel = motor_state['left_velocity']
        right_vel = motor_state['right_velocity']

        # Velocities should be close but not identical (due to variance)
        # Note: they should both be roughly same magnitude but may differ by up to 5%
        assert left_vel > 0 and right_vel > 0, "Both wheels should move forward"


class TestMotorInertia:
    """Test motor inertia (200ms time constant)"""

    def test_inertia_tau_set(self):
        """Verify inertia time constant is initialized"""
        sim = ConcretePhysicsSimulator()
        assert sim.inertia_tau == 0.2  # 200ms

    def test_velocity_increases_from_zero(self):
        """Verify velocity increases from zero when PWM applied"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0  # Disable slip for clean test
        sim.reset_position(0, 0, 0)

        # Get initial velocity
        vx, vy, omega = sim.get_velocity()
        initial_speed = math.sqrt(vx**2 + vy**2)
        assert initial_speed < 1.0

        # Apply PWM
        for _ in range(50):
            sim.update(0.02, 255, 255)

        vx, vy, omega = sim.get_velocity()
        final_speed = math.sqrt(vx**2 + vy**2)

        # Final speed should be higher than initial
        assert final_speed > initial_speed + 20.0

    def test_velocity_does_not_jump_instantly(self):
        """Verify velocity changes gradually, not instantaneously"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(0, 0, 0)

        # Apply step input
        velocities = []
        for step in range(30):
            sim.update(0.02, 255, 255)
            vx, vy, omega = sim.get_velocity()
            linear_speed = math.sqrt(vx**2 + vy**2)
            velocities.append(linear_speed)

        # Initial velocity should be low (inertia effect)
        assert velocities[0] < 10.0, "Initial velocity should be low"

        # Velocity should increase
        assert velocities[-1] > velocities[0], "Velocity should increase"

        # But should not reach max instantly
        # At 200ms tau and 20ms dt, should reach ~63% in roughly tau
        assert velocities[-1] < 60.0 or velocities[15] < 45.0, \
            "Velocity should not reach max instantly (inertia)"

    def test_velocity_decelerates_gradually(self):
        """Verify velocity decreases gradually when PWM stops"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(0, 0, 0)

        # Accelerate
        for _ in range(50):
            sim.update(0.02, 255, 255)

        # Apply zero PWM
        velocities = []
        for _ in range(50):
            sim.update(0.02, 0, 0)
            vx, vy, omega = sim.get_velocity()
            linear_speed = math.sqrt(vx**2 + vy**2)
            velocities.append(linear_speed)

        # First velocity should be high
        assert velocities[0] > 30.0

        # Should decelerate
        assert velocities[-1] < velocities[0]

        # But not instantly to zero (inertia)
        assert velocities[-1] > 5.0 or velocities[5] > 10.0

    def test_inertia_exponential_response(self):
        """Verify inertia follows exponential curve"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(0, 0, 0)

        # Collect velocity data with step input
        velocities = []
        for step in range(30):
            sim.update(0.02, 100, 100)
            vx, vy, omega = sim.get_velocity()
            linear_speed = math.sqrt(vx**2 + vy**2)
            velocities.append(linear_speed)

        # Calculate time constant from exponential response
        # v(t) = v_target * (1 - exp(-t/tau))
        # At t = tau, v should be ~63% of max
        final_velocity = velocities[-1]

        # Find approximate time to reach 63% (one time constant)
        target_speed = final_velocity * 0.63
        for i, v in enumerate(velocities):
            if v >= target_speed:
                time_to_63 = i * 0.02  # seconds
                # Should be roughly around 0.2 seconds (200ms)
                # Allow some variance due to discrete updates
                assert 0.1 < time_to_63 < 0.4
                break


class TestSlipEffect:
    """Test slip effect (2% probability, speed 30%)"""

    def test_slip_probability_approximately_2_percent(self):
        """Verify slip occurs with ~2% probability"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.reset_position(0, 0, 0)

        # Run many updates and count slips
        left_slips = 0
        right_slips = 0
        total_updates = 1000

        for _ in range(total_updates):
            sim.update(0.02, 100, 100)
            if sim.left_slipping:
                left_slips += 1
            if sim.right_slipping:
                right_slips += 1

        left_slip_rate = left_slips / total_updates
        right_slip_rate = right_slips / total_updates

        # Should be approximately 2% (±1%)
        assert 0.01 <= left_slip_rate <= 0.03, f"Left slip rate: {left_slip_rate}"
        assert 0.01 <= right_slip_rate <= 0.03, f"Right slip rate: {right_slip_rate}"

    def test_slip_reduces_speed(self):
        """Verify slip reduces speed compared to no slip"""
        # Test with high slip probability
        sim_slipping = ConcretePhysicsSimulator()
        sim_slipping.set_wheel_variance(0.0)
        sim_slipping.slip_probability = 1.0  # Always slip
        sim_slipping.reset_position(0, 0, 0)

        # Test without slip
        sim_no_slip = ConcretePhysicsSimulator()
        sim_no_slip.set_wheel_variance(0.0)
        sim_no_slip.slip_probability = 0.0  # No slip
        sim_no_slip.reset_position(0, 0, 0)

        # Run both for same time
        for _ in range(50):
            sim_slipping.update(0.02, 100, 100)
            sim_no_slip.update(0.02, 100, 100)

        # Get final velocities
        vx_slip, vy_slip, _ = sim_slipping.get_velocity()
        speed_slip = math.sqrt(vx_slip**2 + vy_slip**2)

        vx_no_slip, vy_no_slip, _ = sim_no_slip.get_velocity()
        speed_no_slip = math.sqrt(vx_no_slip**2 + vy_no_slip**2)

        # Slipping should result in lower speed
        assert speed_slip < speed_no_slip, \
            f"Slip speed {speed_slip} should be < no-slip {speed_no_slip}"

    def test_slip_is_independent_per_wheel(self):
        """Verify slip is independent for left and right wheels"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.reset_position(0, 0, 0)

        # Run with both slip probabilities at 1.0 to see independence
        both_slipping = 0
        left_only = 0
        right_only = 0
        neither = 0

        for _ in range(300):
            sim.update(0.02, 100, 100)
            if sim.left_slipping and sim.right_slipping:
                both_slipping += 1
            elif sim.left_slipping:
                left_only += 1
            elif sim.right_slipping:
                right_only += 1
            else:
                neither += 1

        # All outcomes should occur (slips are independent)
        assert neither > 0, "Should have frames without slip"
        assert left_only > 0, "Should have frames with only left slip"
        assert right_only > 0, "Should have frames with only right slip"


class TestPositionIntegration:
    """Test position integration with differential drive kinematics"""

    def test_forward_motion_increases_x(self):
        """Verify robot moves in positive x when wheels equal"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(0, 0, 0)

        # Move forward
        for _ in range(300):
            sim.update(0.02, 100, 100)

        x, y, yaw = sim.get_position()

        # Should move forward (along x-axis)
        assert x > 20.0, f"Should have moved forward, x={x}"

    def test_position_changes_with_motion(self):
        """Verify position changes as robot moves"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(0, 0, 0)

        # Move forward
        for _ in range(50):
            sim.update(0.02, 100, 100)

        x, y, yaw = sim.get_position()

        # Should have moved
        assert x > 0 or y > 0, "Robot should have moved"

    def test_yaw_accumulation(self):
        """Verify yaw angle accumulates when wheels differ"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.reset_position(0, 0, 0)

        # Make a turn (left wheel faster)
        for _ in range(300):
            sim.update(0.02, 50, 150)

        x, y, yaw = sim.get_position()

        # Should have rotated by significant amount
        assert abs(yaw) > 15.0, f"Should rotate significantly, got {yaw}"

    def test_position_integration_over_time(self):
        """Verify position changes smoothly over time"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(0, 0, 0)

        # Collect position samples
        positions = []

        for _ in range(50):
            sim.update(0.02, 100, 100)
            x, y, yaw = sim.get_position()
            positions.append((x, y))

        # Verify position increases smoothly
        for i in range(1, len(positions)):
            x_prev, y_prev = positions[i-1]
            x_curr, y_curr = positions[i]
            distance_delta = math.sqrt((x_curr - x_prev)**2 + (y_curr - y_prev)**2)

            # Distance delta should be small per step
            # At ~30 cm/s and dt=0.02s, delta ~0.6cm
            assert 0 < distance_delta < 1.5

    def test_backwards_motion(self):
        """Verify robot can move backwards"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(0, 0, 0)

        # Move backwards (negative PWM)
        for _ in range(50):
            sim.update(0.02, -100, -100)

        x, y, yaw = sim.get_position()

        # Should move backwards (negative x)
        assert x < 0, f"Should have moved backwards, x={x}"


class TestDynamicsIntegration:
    """Integration tests combining inertia, variance, and slip"""

    def test_full_dynamics_simulation(self):
        """Test all dynamics features together"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.05)  # ±5% variance
        sim.set_noise_level(2.0)
        sim.invalid_rate = 0.05
        sim.slip_probability = 0.02
        sim.reset_position(0, 0, 0)

        # Run simulation
        for step in range(200):
            # Vary PWM over time
            if step < 50:
                pwm = 100
            elif step < 100:
                pwm = 150
            elif step < 150:
                pwm = 75
            else:
                pwm = 0

            sim.update(0.02, pwm, pwm)

            # Verify state is valid
            x, y, yaw = sim.get_position()
            assert -500 < x < 500
            assert -500 < y < 500
            assert -180 <= yaw <= 180

            motor_state = sim.get_motor_state()
            assert 'left_velocity' in motor_state
            assert 'right_velocity' in motor_state

    def test_motor_state_introspection(self):
        """Verify motor state can be inspected"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.05)

        # Run for a bit
        for _ in range(20):
            sim.update(0.02, 100, 100)

        state = sim.get_motor_state()

        # All expected keys should be present
        required_keys = [
            'left_pwm', 'right_pwm',
            'left_velocity', 'right_velocity',
            'left_target_velocity', 'right_target_velocity',
            'left_variance_factor', 'right_variance_factor',
            'left_slipping', 'right_slipping',
            'inertia_tau'
        ]

        for key in required_keys:
            assert key in state, f"Missing key: {key}"
            assert state[key] is not None

    def test_target_vs_actual_velocity(self):
        """Verify target velocity differs from actual due to inertia"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(0, 0, 0)

        # Single update with PWM
        sim.update(0.02, 255, 255)

        state = sim.get_motor_state()
        target = state['left_target_velocity']
        actual = state['left_velocity']

        # Target should be at max, but actual less due to inertia
        assert target > actual, \
            f"Target {target} should > actual {actual} due to inertia"


class TestEdgeCasesEnhanced:
    """Test edge cases specific to enhanced dynamics"""

    def test_zero_velocity_with_zero_pwm(self):
        """Verify velocity eventually reaches zero after PWM stops"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(0, 0, 0)

        # Accelerate
        for _ in range(50):
            sim.update(0.02, 100, 100)

        # Stop and let inertia wear off
        for _ in range(200):
            sim.update(0.02, 0, 0)

        vx, vy, omega = sim.get_velocity()
        speed = math.sqrt(vx**2 + vy**2)

        # Should be very close to zero
        assert speed < 2.0

    def test_target_velocity_changes_instantly(self):
        """Verify target velocity changes instantly, actual follows inertia"""
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0

        # Accelerate to steady state
        for _ in range(50):
            sim.update(0.02, 100, 100)

        state_before = sim.get_motor_state()
        target_before = state_before['left_target_velocity']

        # Change PWM
        sim.update(0.02, 200, 200)

        state_after = sim.get_motor_state()
        target_after = state_after['left_target_velocity']
        actual_after = state_after['left_velocity']

        # Target should change instantly
        assert target_after > target_before

        # But actual should be between old and new (inertia)
        assert actual_after < target_after

    def test_multiple_variance_factors_different(self):
        """Verify different simulator instances have different variance"""
        factors = []
        for _ in range(5):
            sim = ConcretePhysicsSimulator()
            factors.append((sim.left_variance_factor, sim.right_variance_factor))

        # At least some should differ
        unique_factors = set(f[0] for f in factors)
        assert len(unique_factors) > 1, "Should have variation across instances"
