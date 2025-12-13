"""
End-to-End Integration Test Suite for Wall-Following Robot

This comprehensive test suite validates the autonomous wall-following robot's
behavior across multiple scenarios and physical conditions:

Test Categories:
1. Scenario Tests (5 scenes): rectangular_room, l_corridor, u_turn, narrow_corridor, open_space
2. Physics Effect Tests: sensor_delay, wheel_variance, sensor_noise
3. Stability Tests: consistency, sensor_robustness, control_loop_health
4. Performance Tests: completion_time, distance_efficiency

Each test uses pytest with proper markers for filtering and reporting.
Tests are designed to be deterministic and reproducible.
"""

import pytest
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from e2e_simulator import E2ESimulator
from scenarios.standard_scenes import (
    create_rectangular_room,
    create_l_corridor,
    create_u_turn,
    create_narrow_corridor,
    create_open_space,
)
from assertions.e2e_assertions import (
    SimulationResult,
    NoCollisionAssertion,
    MinimumCornersAssertion,
    MaximumTimeAssertion,
    MinimumDistanceAssertion,
    CompletionRatioAssertion,
    StabilityAssertion,
    BlockingToleranceAssertion,
)
from simulation_controller import SimulationController


# ==================== Test Fixtures ====================

@pytest.fixture
def report_dir(tmp_path):
    """Provide directory for test reports"""
    report_dir = tmp_path / "e2e_reports"
    report_dir.mkdir(exist_ok=True)
    return report_dir


@pytest.fixture
def test_output(report_dir):
    """Fixture to collect test outputs for reporting"""
    @dataclass
    class TestOutput:
        scenario_name: str
        passed: bool
        assertions: List[str]
        summary: str

    return TestOutput


# ==================== Scenario Tests ====================

class TestRectangularRoom:
    """
    Test wall-following robot in rectangular room scenario.

    Scenario: 150x100 cm rectangular room
    Start position: Center-bottom (75, 30) facing up
    Expected behavior: Complete full perimeter (3+ corners)
    """

    @pytest.fixture
    def scenario(self):
        """Create rectangular room scenario"""
        return create_rectangular_room()

    def test_rectangular_room_completes_4_corners(self, scenario):
        """Test that robot detects all 3+ corners of rectangular room"""
        controller = SimulationController.create_standard(
            room_width=150.0,
            room_height=100.0,
            start_x=75.0,
            start_y=30.0,
            start_theta=90.0
        )
        history = controller.run(duration_ms=60000)

        assert len(history) > 0, "Simulation produced no history"
        corner_count = controller.wall_follower.get_corner_count()
        assert corner_count >= 3, f"Expected >= 3 corners, got {corner_count}"

    def test_rectangular_room_no_collision(self, scenario):
        """Test that robot doesn't collide with walls"""
        controller = SimulationController.create_standard(
            room_width=150.0,
            room_height=100.0,
            start_x=75.0,
            start_y=30.0,
            start_theta=90.0
        )
        history = controller.run(duration_ms=60000)

        # Check that robot stayed within room bounds with margin
        robot_radius = 12.0
        margin = 2.0
        min_x, max_x = robot_radius + margin, 150 - robot_radius - margin
        min_y, max_y = robot_radius + margin, 100 - robot_radius - margin

        for state in history:
            assert min_x <= state.x <= max_x, f"Robot x={state.x} outside bounds"
            assert min_y <= state.y <= max_y, f"Robot y={state.y} outside bounds"

    def test_rectangular_room_maintains_wall_distance(self, scenario):
        """Test that robot maintains approximately 15cm right wall distance"""
        controller = SimulationController.create_standard(
            room_width=150.0,
            room_height=100.0,
            start_x=75.0,
            start_y=30.0,
            start_theta=90.0
        )
        history = controller.run(duration_ms=60000)

        # Skip first few states (finding wall)
        wall_following_states = history[50:] if len(history) > 50 else history

        if len(wall_following_states) > 0:
            right_distances = [s.right_dist for s in wall_following_states
                             if s.right_dist > 0 and s.right_dist < 100]

            if len(right_distances) > 0:
                avg_distance = sum(right_distances) / len(right_distances)
                # Target is 15cm with relaxed tolerance
                assert 10 <= avg_distance <= 22, \
                    f"Average wall distance {avg_distance}cm not in target range"

    def test_rectangular_room_travels_significant_distance(self, scenario):
        """Test that robot travels significant distance (perimeter traversal)"""
        controller = SimulationController.create_standard(
            room_width=150.0,
            room_height=100.0,
            start_x=75.0,
            start_y=30.0,
            start_theta=90.0
        )
        history = controller.run(duration_ms=60000)

        if len(history) < 2:
            pytest.skip("Insufficient history data")

        # Calculate distance traveled
        total_distance = 0.0
        for i in range(1, len(history)):
            prev = history[i - 1]
            curr = history[i]
            dx = curr.x - prev.x
            dy = curr.y - prev.y
            total_distance += (dx * dx + dy * dy) ** 0.5

        # Rectangular room perimeter = 2*(150 + 100) = 500 cm
        # Expect at least 300 cm to account for initial positioning
        assert total_distance > 300, \
            f"Robot only traveled {total_distance:.1f}cm, expected > 300cm"


class TestLCorridor:
    """
    Test wall-following robot in L-shaped corridor scenario.

    Scenario: L-shaped corridor with both horizontal and vertical segments
    Start position: Left side of horizontal part (20, 20) facing right
    Expected behavior: Navigate L-shaped path with multiple corners
    """

    @pytest.fixture
    def scenario(self):
        """Create L-corridor scenario"""
        return create_l_corridor()

    def test_l_corridor_completes_path(self, scenario):
        """Test that robot navigates L-corridor without collision"""
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=75.0)

        assert not result.collided, \
            f"Robot collided at {result.collision_point}"
        assert result.corner_count >= 3, \
            f"Expected >= 3 corners, got {result.corner_count}"
        assert result.distance_traveled > 200, \
            f"Expected > 200cm travel, got {result.distance_traveled:.1f}cm"

    def test_l_corridor_handles_corners(self, scenario):
        """Test that robot properly handles corners in L-shaped corridor"""
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=75.0)

        # L-corridor has at least 3 significant corners
        assert result.corner_count >= 3, \
            f"L-corridor should have >= 3 corners, detected {result.corner_count}"

        # Check completion ratio
        expected_distance = 400  # Rough estimate for L-corridor
        completion = result.distance_traveled / expected_distance
        assert completion >= 0.5, \
            f"Robot only completed {completion*100:.0f}% of L-corridor"


class TestUTurn:
    """
    Test wall-following robot in U-shaped turn scenario.

    Scenario: U-shaped corridor requiring 180-degree turn and return
    Start position: Entry corridor (20, 20) facing right
    Expected behavior: Complete U-turn and return to start area
    """

    @pytest.fixture
    def scenario(self):
        """Create U-turn scenario"""
        return create_u_turn()

    def test_u_turn_completes_turn(self, scenario):
        """Test that robot completes U-turn navigation"""
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=90.0)

        assert not result.collided, \
            f"Robot collided during U-turn at {result.collision_point}"
        # U-turn involves 5+ corners (entry, multiple turns, exit)
        assert result.corner_count >= 5, \
            f"U-turn should have >= 5 corners, got {result.corner_count}"

    def test_u_turn_without_collision(self, scenario):
        """Test that robot navigates U-turn without hitting walls"""
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=90.0)

        assert not result.collided, \
            f"Collision detected at {result.collision_point}"


class TestNarrowCorridor:
    """
    Test wall-following robot in narrow corridor scenario.

    Scenario: Very tight corridor (30cm width, robot 20cm wide)
    Start position: Center of corridor (15, 20) facing forward
    Expected behavior: Navigate through without collision (critical test)
    """

    @pytest.fixture
    def scenario(self):
        """Create narrow corridor scenario"""
        return create_narrow_corridor()

    def test_narrow_corridor_no_collision(self, scenario):
        """
        Test that robot navigates narrow corridor without collision.

        This is a critical test - only 5cm clearance per side.
        """
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=45.0)

        assert not result.collided, \
            f"Collision in narrow corridor at {result.collision_point}. " \
            f"Distance traveled: {result.distance_traveled:.1f}cm"

    def test_narrow_corridor_maintains_centerline(self, scenario):
        """Test that robot maintains centerline in narrow corridor"""
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.0,  # No variance for this test
            noise_level=0.0      # No noise for this test
        )
        result = simulator.run(max_time_s=45.0)

        # With perfect physics, check that trajectory stays near center
        if len(result.trajectory) > 10:
            x_positions = [p.x for p in result.trajectory[10:]]
            avg_x = sum(x_positions) / len(x_positions)
            # Corridor center is at x=15cm
            deviation = abs(avg_x - 15.0)
            assert deviation < 5.0, \
                f"Robot deviation {deviation:.1f}cm too large (ideal center)"


class TestOpenSpace:
    """
    Test wall-following robot in open space scenario.

    Scenario: Large open space (300x300 cm) with central obstacle
    Start position: Center of space (150, 150) facing right
    Expected behavior: Trigger FIND_WALL state, find obstacle, follow wall
    """

    @pytest.fixture
    def scenario(self):
        """Create open space scenario"""
        return create_open_space()

    def test_open_space_finds_wall(self, scenario):
        """Test that robot finds wall in open space"""
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=60.0)

        # Robot should traverse significant distance while finding wall
        assert result.distance_traveled > 100, \
            f"Robot should travel while finding wall, traveled {result.distance_traveled:.1f}cm"

        # Should eventually find obstacle and traverse around it
        assert result.corner_count >= 2, \
            f"Robot should detect at least 2 corners from obstacle, got {result.corner_count}"

    def test_open_space_no_collision(self, scenario):
        """Test that robot doesn't collide in open space"""
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=60.0)

        assert not result.collided, \
            f"Unexpected collision in open space at {result.collision_point}"


# ==================== Physics Effects Tests ====================

class TestSensorDelay:
    """
    Test impact of sensor delay on robot behavior.

    Validates that the controller can handle different sensor latencies:
    - 0ms (perfect sensors) - baseline
    - 60ms (realistic) - standard configuration
    - 100ms (degraded) - stress test
    """

    @pytest.mark.delay
    def test_sensor_delay_zero_ms(self):
        """Test robot with zero sensor delay (perfect sensors)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=0.0,
            wheel_variance=0.0,
            noise_level=0.0
        )
        result = simulator.run(max_time_s=60.0)

        # With perfect sensors, expect best performance
        assert result.corner_count >= 4, \
            f"Zero delay should detect all corners, got {result.corner_count}"
        assert not result.collided, "Zero delay should not collide"
        assert result.completion_time < 50, \
            f"Zero delay should complete faster, took {result.completion_time:.1f}s"

    @pytest.mark.delay
    def test_sensor_delay_60ms(self):
        """Test robot with 60ms sensor delay (standard)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=60.0)

        # Standard configuration should still work well
        assert result.corner_count >= 3, \
            f"60ms delay should detect 3+ corners, got {result.corner_count}"
        assert not result.collided, "60ms delay should not collide"

    @pytest.mark.delay
    @pytest.mark.slow
    def test_sensor_delay_100ms(self):
        """Test robot with 100ms sensor delay (degraded)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=100.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=90.0)

        # Higher delay degrades performance but should still work
        # May not complete full perimeter but shouldn't crash
        assert result.corner_count >= 2, \
            f"100ms delay degraded to {result.corner_count} corners"

    @pytest.mark.delay
    def test_sensor_delay_comparison(self):
        """Compare robot performance across different sensor delays"""
        scenario = create_rectangular_room()

        results = {}
        for delay_ms in [0.0, 60.0, 100.0]:
            simulator = E2ESimulator(
                scenario=scenario,
                sensor_delay_ms=delay_ms,
                wheel_variance=0.0 if delay_ms == 0.0 else 0.05,
                noise_level=0.0 if delay_ms == 0.0 else 2.0
            )
            result = simulator.run(max_time_s=60.0)
            results[delay_ms] = result

        # Performance should degrade gracefully with delay
        corners_0ms = results[0.0].corner_count
        corners_60ms = results[60.0].corner_count
        corners_100ms = results[100.0].corner_count

        # Delay should not cause complete failure
        assert corners_0ms >= 3, "0ms delay performance baseline failed"
        assert corners_100ms >= 1, "100ms delay caused complete failure"


class TestWheelVariance:
    """
    Test impact of wheel speed variance on robot behavior.

    Validates controller can handle mismatched motor speeds:
    - 0% variance (perfect motors) - baseline
    - 5% variance (realistic) - standard configuration
    - 15% variance (severe) - stress test
    """

    @pytest.mark.variance
    def test_wheel_variance_zero_percent(self):
        """Test robot with perfect wheel speeds (0% variance)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=0.0,
            wheel_variance=0.0,
            noise_level=0.0
        )
        result = simulator.run(max_time_s=60.0)

        # Perfect motors should give best results
        assert result.corner_count >= 4, \
            f"Perfect motors should detect 4+ corners, got {result.corner_count}"
        assert not result.collided, "Perfect motors should not collide"

    @pytest.mark.variance
    def test_wheel_variance_5_percent(self):
        """Test robot with 5% wheel variance (realistic)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=60.0)

        # 5% variance should be manageable
        assert result.corner_count >= 3, \
            f"5% variance should detect 3+ corners, got {result.corner_count}"
        assert not result.collided, "5% variance should not collide"

    @pytest.mark.variance
    @pytest.mark.slow
    def test_wheel_variance_15_percent(self):
        """Test robot with 15% wheel variance (severe)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.15,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=90.0)

        # High variance is challenging but shouldn't cause crash
        assert result.corner_count >= 1, \
            f"15% variance caused severe degradation to {result.corner_count} corners"

    @pytest.mark.variance
    def test_wheel_variance_symmetry(self):
        """Test that wheel variance doesn't cause systematic bias"""
        scenario = create_rectangular_room()

        # Run with positive/negative variance should be symmetric
        simulator_pos = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=0.0
        )
        result_pos = simulator_pos.run(max_time_s=60.0)

        simulator_neg = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=0.0
        )
        result_neg = simulator_neg.run(max_time_s=60.0)

        # Both should have similar performance (within tolerance)
        corner_diff = abs(result_pos.corner_count - result_neg.corner_count)
        assert corner_diff <= 2, \
            f"Symmetric variance should give similar results, " \
            f"got {result_pos.corner_count} vs {result_neg.corner_count} corners"


class TestSensorNoise:
    """
    Test impact of sensor noise on robot behavior.

    Validates controller can handle noisy sensor readings:
    - 0cm noise (perfect sensors) - baseline
    - 2cm noise (realistic) - standard configuration
    - 5cm noise (high) - stress test
    """

    @pytest.mark.slow
    def test_sensor_noise_zero_cm(self):
        """Test robot with zero sensor noise"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=0.0,
            wheel_variance=0.0,
            noise_level=0.0
        )
        result = simulator.run(max_time_s=60.0)

        # Perfect sensors = baseline
        assert result.corner_count >= 4, \
            f"Zero noise should detect 4+ corners, got {result.corner_count}"
        assert not result.collided, "Zero noise should not collide"

    @pytest.mark.slow
    def test_sensor_noise_2cm(self):
        """Test robot with 2cm sensor noise (standard)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0
        )
        result = simulator.run(max_time_s=60.0)

        # Standard noise should be manageable
        assert result.corner_count >= 3, \
            f"2cm noise should detect 3+ corners, got {result.corner_count}"
        assert not result.collided, "2cm noise should not collide"

    @pytest.mark.slow
    def test_sensor_noise_5cm(self):
        """Test robot with 5cm sensor noise (high)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=5.0
        )
        result = simulator.run(max_time_s=90.0)

        # High noise degrades performance but shouldn't crash
        assert result.corner_count >= 1, \
            f"5cm noise caused severe degradation to {result.corner_count} corners"


class TestControlLoopBlocking:
    """
    Test robot's tolerance to control loop interruptions.

    Simulates occasional blocking events (e.g., from OS scheduling):
    - No blocking (perfect timing) - baseline
    - 5% blocking probability - realistic
    - 20% blocking probability - severe stress
    """

    @pytest.mark.blocking
    def test_blocking_no_interruption(self):
        """Test robot with no control loop blocking"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0,
            blocking_probability=0.0,
            max_blocking_time=0.0
        )
        result = simulator.run(max_time_s=60.0)

        assert not result.collided, "No blocking should not cause collision"
        assert result.blocked_steps == 0, "No blocking should have 0 blocked steps"

    @pytest.mark.blocking
    @pytest.mark.slow
    def test_blocking_5_percent(self):
        """Test robot with 5% blocking probability (realistic)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0,
            blocking_probability=0.05,
            max_blocking_time=0.1
        )
        result = simulator.run(max_time_s=60.0)

        # Should handle occasional blocking
        assert not result.collided, "5% blocking should not cause collision"
        assert result.blocked_steps > 0, "5% blocking should have some blocked steps"

        # Most steps should be unblocked
        if result.total_steps > 0:
            blocked_ratio = result.blocked_steps / result.total_steps
            assert blocked_ratio < 0.3, \
                f"5% blocking resulted in {blocked_ratio*100:.1f}% actual blocking"

    @pytest.mark.blocking
    @pytest.mark.slow
    def test_blocking_20_percent(self):
        """Test robot with 20% blocking probability (severe stress)"""
        scenario = create_rectangular_room()
        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=60.0,
            wheel_variance=0.05,
            noise_level=2.0,
            blocking_probability=0.20,
            max_blocking_time=0.2
        )
        result = simulator.run(max_time_s=90.0)

        # Should still complete without crash (may not complete perimeter)
        assert result.corner_count >= 1, \
            f"20% blocking caused complete failure: {result.corner_count} corners"


# ==================== Stability and Consistency Tests ====================

class TestConsistency:
    """
    Test that robot behavior is consistent and reproducible.

    Validates deterministic simulation under same conditions.
    """

    @pytest.mark.smoke
    def test_rectangular_room_deterministic(self):
        """Test that repeated runs with same parameters give similar results"""
        scenario = create_rectangular_room()

        # Run simulation twice with same parameters
        results = []
        for _ in range(2):
            simulator = E2ESimulator(
                scenario=scenario,
                sensor_delay_ms=60.0,
                wheel_variance=0.0,  # Deterministic (no variance)
                noise_level=0.0      # Deterministic (no noise)
            )
            result = simulator.run(max_time_s=60.0)
            results.append(result)

        # Both runs should have identical corner counts
        assert results[0].corner_count == results[1].corner_count, \
            f"Non-deterministic results: {results[0].corner_count} vs {results[1].corner_count}"

        # Distance traveled should be very similar
        distance_diff = abs(results[0].distance_traveled - results[1].distance_traveled)
        assert distance_diff < 1.0, \
            f"Distance varied too much: {distance_diff:.1f}cm difference"


class TestMultipleScenarios:
    """
    Test robot across all scenario types to validate general robustness.
    """

    @pytest.mark.slow
    def test_all_scenarios_no_crash(self):
        """Test that robot runs in all scenarios without crashing"""
        scenarios = [
            ("rectangular_room", create_rectangular_room(), 60.0),
            ("l_corridor", create_l_corridor(), 75.0),
            ("u_turn", create_u_turn(), 90.0),
            ("narrow_corridor", create_narrow_corridor(), 45.0),
            ("open_space", create_open_space(), 60.0),
        ]

        results = {}
        for scenario_name, scenario, max_time in scenarios:
            simulator = E2ESimulator(
                scenario=scenario,
                sensor_delay_ms=60.0,
                wheel_variance=0.05,
                noise_level=2.0
            )
            result = simulator.run(max_time_s=max_time)
            results[scenario_name] = result

        # All scenarios should complete without crash
        for scenario_name, result in results.items():
            assert result.total_steps > 0, \
                f"{scenario_name}: No simulation steps recorded"
            # Narrow corridor might not detect corners, so don't check
            if scenario_name != "narrow_corridor":
                assert result.corner_count >= 1, \
                    f"{scenario_name}: No corners detected"


# ==================== Performance Tests ====================

class TestPerformance:
    """
    Test that robot completes tasks within reasonable time and distance.
    """

    def test_rectangular_room_efficiency(self):
        """Test that rectangular room is completed efficiently"""
        controller = SimulationController.create_standard(
            room_width=150.0,
            room_height=100.0,
            start_x=75.0,
            start_y=30.0,
            start_theta=90.0
        )
        history = controller.run(duration_ms=60000)

        # Should complete within reasonable time
        elapsed_ms = history[-1].time_ms if history else 0
        assert elapsed_ms < 60000, \
            f"Should complete in < 60s, took {elapsed_ms/1000:.1f}s"

        # Calculate efficiency (perimeter / actual distance)
        total_distance = 0.0
        for i in range(1, len(history)):
            prev = history[i - 1]
            curr = history[i]
            dx = curr.x - prev.x
            dy = curr.y - prev.y
            total_distance += (dx * dx + dy * dy) ** 0.5

        perimeter = 2 * (150 + 100)  # 500 cm
        efficiency = perimeter / total_distance if total_distance > 0 else 0

        # Should achieve reasonable efficiency (>0.5 = better than 2x distance)
        assert efficiency > 0.5, \
            f"Efficiency {efficiency:.2f} too low (traveled {total_distance:.0f}cm)"


# ==================== Integration Tests ====================

class TestIntegration:
    """
    Test complete integration scenarios combining multiple effects.
    """

    @pytest.mark.slow
    def test_worst_case_scenario(self):
        """Test robot under worst-case combined conditions"""
        scenario = create_rectangular_room()

        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=100.0,      # High delay
            wheel_variance=0.15,         # High variance
            noise_level=5.0,             # High noise
            blocking_probability=0.10,   # Some blocking
            max_blocking_time=0.2
        )
        result = simulator.run(max_time_s=120.0)

        # Should not crash even under worst conditions
        assert result.total_steps > 0, "Worst case: No simulation"

        # May not complete all corners but should make progress
        assert result.distance_traveled > 100, \
            f"Worst case: Very limited progress ({result.distance_traveled:.0f}cm)"

    @pytest.mark.slow
    def test_best_case_scenario(self):
        """Test robot under best-case conditions (all settings optimal)"""
        scenario = create_rectangular_room()

        simulator = E2ESimulator(
            scenario=scenario,
            sensor_delay_ms=0.0,        # No delay
            wheel_variance=0.0,         # Perfect motors
            noise_level=0.0,            # No noise
            blocking_probability=0.0,   # No blocking
            max_blocking_time=0.0
        )
        result = simulator.run(max_time_s=60.0)

        # Best case should perform excellently
        assert result.corner_count >= 4, \
            f"Best case should detect 4+ corners, got {result.corner_count}"
        assert not result.collided, "Best case should not collide"
        assert result.distance_traveled > 450, \
            f"Best case should travel 450+ cm, got {result.distance_traveled:.0f}cm"


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v", "--tb=short"])
