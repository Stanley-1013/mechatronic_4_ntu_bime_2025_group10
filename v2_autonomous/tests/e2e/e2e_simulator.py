"""
End-to-End Simulator for Wall-Following Robot

Provides the E2ESimulator class that:
1. Encapsulates simulation environment, physics, and controller
2. Orchestrates the complete simulation run
3. Converts results to SimulationResult for assertion validation
4. Handles scenario configuration and physics parameters
"""

import math
from typing import List, Optional, Tuple
from dataclasses import dataclass

from simulation_controller import SimulationController, SimulationState
from simulator.concrete_physics_simulator import ConcretePhysicsSimulator
from simulator.concrete_virtual_environment import ConcreteVirtualEnvironment
from controller.concrete_wall_follower import ConcreteWallFollower
from scenarios.standard_scenes import TestScenario
from assertions.e2e_assertions import SimulationResult, TrajectoryPoint


class E2ESimulator:
    """
    Complete end-to-end simulator for wall-following robot testing.

    Encapsulates:
    - Virtual environment (walls, obstacles)
    - Physics simulator (motor dynamics, sensor delay, noise)
    - Wall-follower controller
    - Simulation orchestration
    - Result collection and conversion

    Supports configurable physics parameters:
    - sensor_delay_ms: Sensor reading latency in milliseconds
    - wheel_variance: Motor speed variance as percentage (0.0-1.0)
    - noise_level: Sensor noise standard deviation in cm
    - blocking_probability: Probability of control loop blocking (0.0-1.0)
    - max_blocking_time: Maximum blocking duration in seconds
    """

    def __init__(
        self,
        scenario: TestScenario,
        sensor_delay_ms: float = 60.0,
        wheel_variance: float = 0.05,
        noise_level: float = 2.0,
        blocking_probability: float = 0.0,
        max_blocking_time: float = 0.0
    ):
        """
        Initialize E2E simulator from test scenario.

        Args:
            scenario: TestScenario defining environment and start position
            sensor_delay_ms: Sensor reading latency (default 60ms)
            wheel_variance: Motor speed variance factor (default 0.05 = 5%)
            noise_level: Sensor noise σ in cm (default 2cm)
            blocking_probability: Probability of blocking per step (default 0.0)
            max_blocking_time: Max duration of blocking event in seconds (default 0.0)
        """
        self.scenario = scenario
        self.sensor_delay_ms = sensor_delay_ms
        self.wheel_variance = wheel_variance
        self.noise_level = noise_level
        self.blocking_probability = blocking_probability
        self.max_blocking_time = max_blocking_time

        # Extract scenario parameters
        self.environment = scenario.environment
        start_x, start_y, start_theta = scenario.start_position

        # Create physics simulator
        physics = ConcretePhysicsSimulator(max_range=500.0)
        physics.set_sensor_delay(sensor_delay_ms)
        physics.set_noise_level(noise_level)
        physics.set_wheel_variance(wheel_variance)
        physics.reset_position(start_x, start_y, start_theta)

        # Create wall-follower controller
        wall_follower = ConcreteWallFollower()

        # Create simulation controller
        self.controller = SimulationController(
            environment=self.environment,
            physics=physics,
            wall_follower=wall_follower,
            dt_ms=20
        )

    def run(self, max_time_s: float = 75.0) -> SimulationResult:
        """
        Execute the simulation and return results.

        Args:
            max_time_s: Maximum simulation time in seconds (default 75s)

        Returns:
            SimulationResult with trajectory, metrics, and collision info
        """
        # Convert max time to milliseconds
        duration_ms = int(max_time_s * 1000)

        # Run simulation
        history = self.controller.run(duration_ms=duration_ms)

        # Convert history to trajectory
        trajectory = self._convert_history_to_trajectory(history)

        # Extract metrics
        corner_count = self.controller.wall_follower.get_corner_count()
        completion_time = self._calculate_completion_time(history)
        distance_traveled = self._calculate_distance_traveled(history)
        collided, collision_point = self._detect_collision(history)
        final_position = self._get_final_position(history)
        total_steps = len(history)
        blocked_steps = 0  # Placeholder: not tracking blocking in this version

        # Build and return result
        return SimulationResult(
            trajectory=trajectory,
            corner_count=corner_count,
            completion_time=completion_time,
            collided=collided,
            collision_point=collision_point,
            final_position=final_position,
            distance_traveled=distance_traveled,
            total_steps=total_steps,
            blocked_steps=blocked_steps
        )

    def reset(self) -> None:
        """Reset the simulation state for re-running."""
        self.controller.reset()

    # ==================== Private Helper Methods ====================

    def _convert_history_to_trajectory(
        self,
        history: List[SimulationState]
    ) -> List[TrajectoryPoint]:
        """
        Convert SimulationState history to TrajectoryPoint list.

        Args:
            history: List of SimulationState records

        Returns:
            List of TrajectoryPoint records with complete trajectory data
        """
        trajectory = []

        for state in history:
            # Calculate PWM values from normalized commands
            # left_pwm = (linear - angular) * 255
            # right_pwm = (linear + angular) * 255
            left_pwm = int((state.linear_cmd - state.angular_cmd) * 255)
            right_pwm = int((state.linear_cmd + state.angular_cmd) * 255)

            # Clamp PWM values to valid range
            left_pwm = max(-255, min(255, left_pwm))
            right_pwm = max(-255, min(255, right_pwm))

            # Create trajectory point
            point = TrajectoryPoint(
                x=state.x,
                y=state.y,
                yaw=state.theta,
                front_distance=state.front_dist,
                right_distance=state.right_dist,
                left_pwm=left_pwm,
                right_pwm=right_pwm,
                timestamp=state.time_ms / 1000.0  # Convert ms to seconds
            )
            trajectory.append(point)

        return trajectory

    def _calculate_completion_time(self, history: List[SimulationState]) -> float:
        """
        Calculate simulation completion time in seconds.

        Args:
            history: List of SimulationState records

        Returns:
            Completion time in seconds
        """
        if not history:
            return 0.0
        return history[-1].time_ms / 1000.0

    def _calculate_distance_traveled(self, history: List[SimulationState]) -> float:
        """
        Calculate total distance traveled by robot.

        Uses Euclidean distance between consecutive position samples.

        Args:
            history: List of SimulationState records

        Returns:
            Total distance traveled in cm
        """
        if len(history) < 2:
            return 0.0

        total_distance = 0.0
        for i in range(1, len(history)):
            prev = history[i - 1]
            curr = history[i]

            dx = curr.x - prev.x
            dy = curr.y - prev.y
            distance = math.sqrt(dx * dx + dy * dy)
            total_distance += distance

        return total_distance

    def _detect_collision(
        self,
        history: List[SimulationState]
    ) -> Tuple[bool, Optional[Tuple[float, float]]]:
        """
        Detect if robot collided with walls during simulation.

        Uses environment collision detection on robot radius.

        Args:
            history: List of SimulationState records

        Returns:
            Tuple of (collided: bool, collision_point: Optional[Tuple[x, y]])
        """
        robot_radius = self.controller.simulator.robot_radius

        for state in history:
            if self.environment.check_collision(state.x, state.y, robot_radius):
                return True, (state.x, state.y)

        return False, None

    def _get_final_position(
        self,
        history: List[SimulationState]
    ) -> Optional[Tuple[float, float, float]]:
        """
        Get final robot position (x, y, theta).

        Args:
            history: List of SimulationState records

        Returns:
            Tuple of (x, y, theta) or None if no history
        """
        if not history:
            return None

        final_state = history[-1]
        return (final_state.x, final_state.y, final_state.theta)
