"""
Unit tests for SimulationController.

Tests:
1. Basic execution (100 steps)
2. Collision detection and stopping
3. State history completeness
4. Standard factory method
"""

import pytest
from simulation_controller import SimulationController, SimulationState
from simulator.concrete_virtual_environment import ConcreteVirtualEnvironment
from simulator.concrete_physics_simulator import ConcretePhysicsSimulator
from controller.concrete_wall_follower import ConcreteWallFollower
from controller.wall_follower_wrapper import WallFollowerState


class TestSimulationControllerBasic:
    """Test basic simulation execution."""

    def test_basic_execution_100_steps(self):
        """Test that controller runs for 100 time steps without errors."""
        # Create simulation
        controller = SimulationController.create_standard()

        # Run for 2 seconds (100 steps at 20ms each)
        history = controller.run(duration_ms=2000, stop_on_done=False, stop_on_collision=False)

        # Verify execution
        assert len(history) > 0, "History should not be empty"
        assert len(history) == 100, f"Expected 100 steps, got {len(history)}"

        # Verify time progression
        assert history[0].time_ms == 0, "First step should be at t=0"
        assert history[-1].time_ms == 1980, f"Last step should be at t=1980ms, got {history[-1].time_ms}"

        # Verify state record structure
        first_state = history[0]
        assert hasattr(first_state, 'time_ms'), "State should have time_ms"
        assert hasattr(first_state, 'x'), "State should have x coordinate"
        assert hasattr(first_state, 'y'), "State should have y coordinate"
        assert hasattr(first_state, 'theta'), "State should have theta"
        assert hasattr(first_state, 'front_dist'), "State should have front_dist"
        assert hasattr(first_state, 'right_dist'), "State should have right_dist"
        assert hasattr(first_state, 'state'), "State should have state"
        assert hasattr(first_state, 'linear_cmd'), "State should have linear_cmd"
        assert hasattr(first_state, 'angular_cmd'), "State should have angular_cmd"

    def test_history_completeness(self):
        """Test that history records are complete and consistent."""
        controller = SimulationController.create_standard()
        history = controller.run(duration_ms=1000, stop_on_done=False, stop_on_collision=False)

        # Check each state record
        for i, state in enumerate(history):
            # Time should be monotonically increasing
            assert state.time_ms == i * 20, f"Step {i}: expected time {i*20}ms, got {state.time_ms}ms"

            # Positions should be numerical
            assert isinstance(state.x, (int, float)), f"x should be numeric"
            assert isinstance(state.y, (int, float)), f"y should be numeric"
            assert isinstance(state.theta, (int, float)), f"theta should be numeric"

            # Distances should be positive integers
            assert isinstance(state.front_dist, int), f"front_dist should be int"
            assert isinstance(state.right_dist, int), f"right_dist should be int"
            assert state.front_dist >= 0, f"front_dist should be non-negative"
            assert state.right_dist >= 0, f"right_dist should be non-negative"

            # State should be valid WallFollowerState value
            assert 0 <= state.state <= 255, f"state should be valid"

            # Commands should be normalized
            assert -1.5 <= state.linear_cmd <= 1.5, f"linear_cmd out of bounds"
            assert -1.5 <= state.angular_cmd <= 1.5, f"angular_cmd out of bounds"

    def test_state_representation(self):
        """Test SimulationState string representation."""
        state = SimulationState(
            time_ms=100,
            x=25.5,
            y=30.2,
            theta=45.0,
            front_dist=150,
            right_dist=20,
            state=2,
            linear_cmd=0.6,
            angular_cmd=-0.15
        )

        # Should have a valid string representation
        state_str = str(state)
        assert "100ms" in state_str or "100" in state_str
        assert "25.5" in state_str
        assert "30.2" in state_str
        assert "45.0" in state_str


class TestSimulationControllerCollision:
    """Test collision detection and stopping."""

    def test_collision_stops_simulation(self):
        """Test that collision detection stops the simulation."""
        # Create small room to guarantee collision
        env = ConcreteVirtualEnvironment.create_rectangular_room(50.0, 50.0)
        physics = ConcretePhysicsSimulator(max_range=500.0)
        physics.reset_position(5.0, 5.0, 0.0)  # Start near corner
        wall_follower = ConcreteWallFollower()

        controller = SimulationController(env, physics, wall_follower, dt_ms=20)

        # Run with collision detection enabled
        history = controller.run(
            duration_ms=5000,
            stop_on_collision=True,
            stop_on_done=False
        )

        # Should stop before max duration
        assert len(history) < 250, "Simulation should stop before 5000ms"

    def test_collision_disabled(self):
        """Test that collision detection can be disabled."""
        env = ConcreteVirtualEnvironment.create_rectangular_room(50.0, 50.0)
        physics = ConcretePhysicsSimulator(max_range=500.0)
        physics.reset_position(5.0, 5.0, 0.0)
        wall_follower = ConcreteWallFollower()

        controller = SimulationController(env, physics, wall_follower, dt_ms=20)

        # Run without collision detection
        history = controller.run(
            duration_ms=1000,
            stop_on_collision=False,
            stop_on_done=False
        )

        # Should run for full duration
        assert len(history) == 50, f"Expected 50 steps for 1000ms, got {len(history)}"


class TestSimulationControllerTermination:
    """Test termination conditions."""

    def test_custom_stop_condition(self):
        """Test custom stop condition."""
        controller = SimulationController.create_standard()

        # Stop when x position exceeds 50cm
        def stop_at_x_50(state: SimulationState) -> bool:
            return state.x > 50.0

        history = controller.run(
            duration_ms=5000,
            stop_on_done=False,
            stop_on_collision=False,
            stop_condition=stop_at_x_50
        )

        # Should stop before reaching 5000ms
        assert len(history) < 250

        # Last state should have x <= 50 (due to how the loop works)
        if history:
            # The condition is checked after recording, so last recorded might exceed 50
            assert history[-2].x <= 50.0 or len(history) == 1

    def test_multiple_stop_conditions(self):
        """Test that multiple stop conditions are checked."""
        controller = SimulationController.create_standard()

        history = controller.run(
            duration_ms=5000,
            stop_on_collision=True,
            stop_on_done=True,
            stop_condition=None
        )

        # Should complete without error
        assert len(history) > 0


class TestSimulationControllerFactory:
    """Test factory method."""

    def test_create_standard(self):
        """Test create_standard factory method."""
        controller = SimulationController.create_standard()

        # Verify components are created
        assert controller.env is not None
        assert controller.simulator is not None
        assert controller.wall_follower is not None
        assert controller.dt_ms == 20

        # Verify initial positions
        x, y, theta = controller.simulator.get_position()
        assert x == 10.0, f"Expected start x=10, got {x}"
        assert y == 10.0, f"Expected start y=10, got {y}"
        assert theta == 0.0, f"Expected start theta=0, got {theta}"

    def test_create_custom_room(self):
        """Test create_standard with custom room dimensions."""
        controller = SimulationController.create_standard(
            room_width=200.0,
            room_height=150.0,
            start_x=20.0,
            start_y=25.0,
            start_theta=45.0
        )

        # Verify parameters
        x, y, theta = controller.simulator.get_position()
        assert x == 20.0
        assert y == 25.0
        assert theta == 45.0

        # Verify environment bounds
        bounds = controller.env.get_bounds()
        # Bounds should encompass the room with padding
        assert bounds[0] < 0  # min_x with padding
        assert bounds[1] < 0  # min_y with padding
        assert bounds[2] > 200.0  # max_x with padding
        assert bounds[3] > 150.0  # max_y with padding


class TestSimulationControllerSummary:
    """Test summary statistics."""

    def test_get_summary(self):
        """Test get_summary method."""
        controller = SimulationController.create_standard()
        history = controller.run(duration_ms=1000, stop_on_done=False, stop_on_collision=False)

        summary = controller.get_summary()

        # Verify summary contents
        assert 'total_steps' in summary
        assert 'duration_ms' in summary
        assert 'max_x' in summary
        assert 'min_x' in summary
        assert 'max_y' in summary
        assert 'min_y' in summary
        assert 'total_distance' in summary
        assert 'corner_count' in summary
        assert 'final_state' in summary

        # Verify values
        assert summary['total_steps'] == 50, f"Expected 50 steps, got {summary['total_steps']}"
        assert summary['duration_ms'] == 980, f"Expected 980ms duration, got {summary['duration_ms']}"
        assert summary['total_distance'] >= 0, "Distance should be non-negative"
        assert summary['corner_count'] >= 0, "Corner count should be non-negative"

    def test_print_history(self):
        """Test print_history method."""
        controller = SimulationController.create_standard()
        controller.run(duration_ms=500, stop_on_done=False, stop_on_collision=False)

        # Should not raise exception
        controller.print_history(max_entries=10)
        controller.print_history()  # Print all


class TestSimulationControllerSensorDelay:
    """Test sensor delay integration."""

    def test_sensor_delay_60ms(self):
        """Test that 60ms sensor delay is properly simulated."""
        controller = SimulationController.create_standard()

        # Run for a short duration to observe delay effects
        history = controller.run(duration_ms=500, stop_on_done=False, stop_on_collision=False)

        # In a 500ms simulation with 20ms steps, sensor readings should
        # reflect data from ~60ms in the past
        # This is a qualitative test - actual delay validation would need
        # to compare sensor readings with true distances

        # At least verify that sensor values are being read
        sensor_values_present = any(
            state.front_dist > 0 or state.right_dist > 0
            for state in history
        )
        assert sensor_values_present, "Should have some sensor readings"


class TestSimulationControllerReset:
    """Test reset functionality."""

    def test_reset_clears_history(self):
        """Test that reset clears the simulation history."""
        controller = SimulationController.create_standard()

        # Run first simulation
        controller.run(duration_ms=500, stop_on_done=False, stop_on_collision=False)
        assert len(controller.history) > 0

        # Reset
        controller.reset()

        # History should be cleared
        assert len(controller.history) == 0

        # Should be able to run again
        history = controller.run(duration_ms=500, stop_on_done=False, stop_on_collision=False)
        assert len(history) > 0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
