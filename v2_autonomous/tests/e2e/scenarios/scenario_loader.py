"""
Scenario Loader - Unified interface for loading and running test scenarios.

Provides convenient methods to:
1. Load standard test scenarios by name or properties
2. Create simulation controllers pre-configured for scenarios
3. Run complete test simulations with result collection
4. Generate test reports
"""

import sys
import os
from typing import List, Optional, Tuple, Dict, Any
from dataclasses import dataclass

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from .standard_scenes import (
    TestScenario,
    create_rectangular_room,
    create_l_corridor,
    create_u_turn,
    create_narrow_corridor,
    create_open_space,
    get_all_scenarios,
    SCENARIO_METADATA,
)
from simulator.concrete_virtual_environment import ConcreteVirtualEnvironment
from simulator.concrete_physics_simulator import ConcretePhysicsSimulator
from controller.concrete_wall_follower import ConcreteWallFollower
from simulation_controller import SimulationController, SimulationState


@dataclass
class SimulationResult:
    """Results from running a single scenario simulation."""

    scenario_name: str
    success: bool
    duration_ms: int
    total_steps: int
    distance_traveled_cm: float
    corners_detected: int
    collision_detected: bool
    final_state: int
    start_position: Tuple[float, float, float]
    end_position: Tuple[float, float, float]
    message: str = ""
    history: Optional[List[SimulationState]] = None

    def __str__(self) -> str:
        """Format result summary."""
        status = "PASS" if self.success else "FAIL"
        return (
            f"[{status}] {self.scenario_name}: "
            f"duration={self.duration_ms}ms, "
            f"distance={self.distance_traveled_cm:.1f}cm, "
            f"corners={self.corners_detected}, "
            f"collision={self.collision_detected}"
        )


class ScenarioLoader:
    """
    Unified interface for loading and running test scenarios.

    Provides static methods for:
    - Getting scenarios by name or properties
    - Creating simulation controllers for scenarios
    - Running complete simulations
    - Generating reports
    """

    # Sensor configuration defaults for all scenarios
    SENSOR_DELAY_MS = 60.0
    NOISE_LEVEL_CM = 2.0
    WHEEL_VARIANCE = 0.05  # ±5%
    CONTROL_STEP_MS = 20  # 50Hz
    MAX_SENSOR_RANGE_CM = 500.0

    @staticmethod
    def get_all() -> List[TestScenario]:
        """
        Get all standard test scenarios.

        Returns:
            List of all TestScenario objects in order
        """
        return get_all_scenarios()

    @staticmethod
    def get_by_name(name: str) -> Optional[TestScenario]:
        """
        Get a scenario by its name.

        Args:
            name: Scenario name (e.g., "rectangular_room", "l_corridor")

        Returns:
            TestScenario if found, None otherwise

        Examples:
            scenario = ScenarioLoader.get_by_name("rectangular_room")
            scenario = ScenarioLoader.get_by_name("narrow_corridor")
        """
        scenario_map = {
            "rectangular_room": create_rectangular_room,
            "l_corridor": create_l_corridor,
            "u_turn": create_u_turn,
            "narrow_corridor": create_narrow_corridor,
            "open_space": create_open_space,
        }

        if name in scenario_map:
            return scenario_map[name]()
        return None

    @staticmethod
    def get_by_difficulty(difficulty: str) -> List[TestScenario]:
        """
        Get all scenarios of a specific difficulty level.

        Args:
            difficulty: One of "beginner", "intermediate", "advanced", "expert"

        Returns:
            List of TestScenario objects matching the difficulty

        Examples:
            beginner = ScenarioLoader.get_by_difficulty("beginner")
            advanced = ScenarioLoader.get_by_difficulty("advanced")
        """
        scenarios = get_all_scenarios()
        result = []

        for scenario in scenarios:
            metadata = SCENARIO_METADATA.get(scenario.name, {})
            if metadata.get("difficulty") == difficulty:
                result.append(scenario)

        return result

    @staticmethod
    def get_by_test_type(test_type: str) -> List[TestScenario]:
        """
        Get scenarios designed for a specific test purpose.

        Args:
            test_type: Primary test type (e.g., "basic_wall_following",
                      "corner_detection", "collision_avoidance", "wall_finding")

        Returns:
            List of TestScenario objects matching the test type

        Examples:
            corner_tests = ScenarioLoader.get_by_test_type("corner_detection")
            collision_tests = ScenarioLoader.get_by_test_type("collision_avoidance")
        """
        scenarios = get_all_scenarios()
        result = []

        for scenario in scenarios:
            metadata = SCENARIO_METADATA.get(scenario.name, {})
            if metadata.get("primary_test") == test_type:
                result.append(scenario)

        return result

    @staticmethod
    def get_by_feature(feature: str) -> List[TestScenario]:
        """
        Get scenarios that test a specific feature.

        Args:
            feature: Feature name (e.g., "closed_loop", "varied_corners",
                    "tight_space", "find_wall_state")

        Returns:
            List of TestScenario objects with the feature

        Examples:
            closed_loops = ScenarioLoader.get_by_feature("closed_loop")
            tight_spaces = ScenarioLoader.get_by_feature("tight_space")
        """
        scenarios = get_all_scenarios()
        result = []

        for scenario in scenarios:
            metadata = SCENARIO_METADATA.get(scenario.name, {})
            if feature in metadata.get("features", []):
                result.append(scenario)

        return result

    @staticmethod
    def get_quick_scenarios(max_duration_s: float = 60.0) -> List[TestScenario]:
        """
        Get scenarios that complete quickly.

        Args:
            max_duration_s: Maximum duration in seconds (default 60s)

        Returns:
            List of TestScenario objects with duration <= max_duration_s

        Examples:
            quick = ScenarioLoader.get_quick_scenarios()
            very_quick = ScenarioLoader.get_quick_scenarios(30.0)
        """
        scenarios = get_all_scenarios()
        return [s for s in scenarios if s.max_duration_s <= max_duration_s]

    @staticmethod
    def create_controller(
        scenario: TestScenario,
        sensor_delay_ms: Optional[float] = None,
        noise_level_cm: Optional[float] = None,
        wheel_variance: Optional[float] = None,
    ) -> SimulationController:
        """
        Create a simulation controller pre-configured for a scenario.

        Args:
            scenario: TestScenario to configure for
            sensor_delay_ms: Sensor delay in milliseconds (None = use default)
            noise_level_cm: Gaussian noise std deviation (None = use default)
            wheel_variance: Motor variance as fraction (None = use default)

        Returns:
            Configured SimulationController ready to run

        Examples:
            scenario = ScenarioLoader.get_by_name("rectangular_room")
            controller = ScenarioLoader.create_controller(scenario)
            history = controller.run(60000)  # Run 60 seconds
        """
        # Use defaults if not specified
        delay = sensor_delay_ms if sensor_delay_ms is not None else ScenarioLoader.SENSOR_DELAY_MS
        noise = noise_level_cm if noise_level_cm is not None else ScenarioLoader.NOISE_LEVEL_CM
        variance = wheel_variance if wheel_variance is not None else ScenarioLoader.WHEEL_VARIANCE

        # Create physics simulator with scenario's environment
        physics = ConcretePhysicsSimulator(max_range=ScenarioLoader.MAX_SENSOR_RANGE_CM)
        physics.set_sensor_delay(delay)
        physics.set_noise_level(noise)
        physics.set_wheel_variance(variance)

        # Set initial position from scenario
        x, y, theta = scenario.start_position
        physics.reset_position(x, y, theta)

        # Create wall-follower controller
        wall_follower = ConcreteWallFollower()

        # Create and return simulation controller
        return SimulationController(
            environment=scenario.environment,
            physics=physics,
            wall_follower=wall_follower,
            dt_ms=ScenarioLoader.CONTROL_STEP_MS
        )

    @staticmethod
    def run_scenario(
        scenario: TestScenario,
        max_duration_ms: Optional[int] = None,
        sensor_delay_ms: Optional[float] = None,
        noise_level_cm: Optional[float] = None,
        wheel_variance: Optional[float] = None,
        stop_on_collision: bool = True,
        verbose: bool = False,
    ) -> SimulationResult:
        """
        Run a complete scenario simulation.

        Args:
            scenario: TestScenario to run
            max_duration_ms: Maximum simulation duration in ms
                            (None = scenario.max_duration_s)
            sensor_delay_ms: Sensor delay in milliseconds
            noise_level_cm: Gaussian noise std deviation
            wheel_variance: Motor variance as fraction
            stop_on_collision: Stop if collision detected
            verbose: Print status during simulation

        Returns:
            SimulationResult with test outcome and metrics

        Examples:
            result = ScenarioLoader.run_scenario(scenario)
            print(f"Success: {result.success}")
            print(f"Distance: {result.distance_traveled_cm:.1f} cm")
        """
        # Determine simulation duration
        if max_duration_ms is None:
            max_duration_ms = int(scenario.max_duration_s * 1000)

        if verbose:
            print(f"Running scenario: {scenario.name}")
            print(f"  Duration: {max_duration_ms}ms")
            print(f"  Start position: {scenario.start_position}")

        try:
            # Create controller for scenario
            controller = ScenarioLoader.create_controller(
                scenario,
                sensor_delay_ms=sensor_delay_ms,
                noise_level_cm=noise_level_cm,
                wheel_variance=wheel_variance,
            )

            # Run simulation
            history = controller.run(
                max_duration_ms,
                stop_on_collision=stop_on_collision,
                stop_on_done=True
            )

            # Get summary
            summary = controller.get_summary()

            # Check if collision occurred
            collision_detected = False
            for state in history:
                if controller.env.check_collision(state.x, state.y, controller.simulator.robot_radius):
                    collision_detected = True
                    break

            # Get end position
            if history:
                end_pos = (history[-1].x, history[-1].y, history[-1].theta)
            else:
                end_pos = scenario.start_position

            # Create result
            result = SimulationResult(
                scenario_name=scenario.name,
                success=not collision_detected,  # Success if no collision
                duration_ms=summary.get('duration_ms', 0),
                total_steps=summary.get('total_steps', len(history)),
                distance_traveled_cm=summary.get('total_distance', 0.0),
                corners_detected=summary.get('corner_count', 0),
                collision_detected=collision_detected,
                final_state=summary.get('final_state', -1),
                start_position=scenario.start_position,
                end_position=end_pos,
                history=history,
            )

            if verbose:
                print(f"  Result: {result}")

            return result

        except Exception as e:
            # Return failure result
            return SimulationResult(
                scenario_name=scenario.name,
                success=False,
                duration_ms=0,
                total_steps=0,
                distance_traveled_cm=0.0,
                corners_detected=0,
                collision_detected=False,
                final_state=-1,
                start_position=scenario.start_position,
                end_position=scenario.start_position,
                message=f"Error: {str(e)}",
            )

    @staticmethod
    def run_all_scenarios(
        max_duration_ms: Optional[int] = None,
        verbose: bool = False,
    ) -> Dict[str, SimulationResult]:
        """
        Run all standard scenarios sequentially.

        Args:
            max_duration_ms: Maximum duration for each scenario
            verbose: Print status for each scenario

        Returns:
            Dictionary mapping scenario names to SimulationResult objects

        Examples:
            results = ScenarioLoader.run_all_scenarios(verbose=True)
            for name, result in results.items():
                print(f"{name}: {result}")
        """
        scenarios = ScenarioLoader.get_all()
        results = {}

        for scenario in scenarios:
            result = ScenarioLoader.run_scenario(
                scenario,
                max_duration_ms=max_duration_ms,
                verbose=verbose
            )
            results[scenario.name] = result

        return results

    @staticmethod
    def run_difficulty_progression(
        max_duration_ms: Optional[int] = None,
        verbose: bool = False,
    ) -> Dict[str, Dict[str, SimulationResult]]:
        """
        Run all scenarios grouped by difficulty level.

        Args:
            max_duration_ms: Maximum duration for each scenario
            verbose: Print status during execution

        Returns:
            Dictionary mapping difficulty levels to result dictionaries

        Examples:
            results = ScenarioLoader.run_difficulty_progression(verbose=True)
            beginner_results = results["beginner"]
            advanced_results = results["advanced"]
        """
        difficulty_levels = ["beginner", "intermediate", "advanced", "expert"]
        all_results = {}

        for difficulty in difficulty_levels:
            scenarios = ScenarioLoader.get_by_difficulty(difficulty)
            difficulty_results = {}

            for scenario in scenarios:
                result = ScenarioLoader.run_scenario(
                    scenario,
                    max_duration_ms=max_duration_ms,
                    verbose=verbose
                )
                difficulty_results[scenario.name] = result

            if difficulty_results:
                all_results[difficulty] = difficulty_results

        return all_results

    @staticmethod
    def print_scenario_info(scenario: TestScenario) -> None:
        """
        Print detailed information about a scenario.

        Args:
            scenario: TestScenario to display

        Examples:
            scenario = ScenarioLoader.get_by_name("rectangular_room")
            ScenarioLoader.print_scenario_info(scenario)
        """
        metadata = SCENARIO_METADATA.get(scenario.name, {})

        print(f"\n{'='*70}")
        print(f"SCENARIO: {scenario.name.upper()}")
        print(f"{'='*70}")
        print(f"\nDescription: {scenario.description}")
        print(f"\nMetadata:")
        print(f"  Difficulty: {metadata.get('difficulty', 'N/A')}")
        print(f"  Primary Test: {metadata.get('primary_test', 'N/A')}")
        print(f"  Environment Type: {metadata.get('environment_type', 'N/A')}")
        print(f"  Features: {', '.join(metadata.get('features', []))}")
        print(f"\nConfiguration:")
        print(f"  Start Position: x={scenario.start_position[0]:.1f} cm, "
              f"y={scenario.start_position[1]:.1f} cm, "
              f"theta={scenario.start_position[2]:.1f}°")
        print(f"  Expected Corners: {scenario.expected_corners}")
        print(f"  Max Duration: {scenario.max_duration_s:.1f} seconds")
        print(f"  Environment Walls: {scenario.environment.get_wall_count()}")
        bounds = scenario.environment.get_bounds()
        print(f"  Environment Bounds: x=[{bounds[0]:.1f}, {bounds[2]:.1f}], "
              f"y=[{bounds[1]:.1f}, {bounds[3]:.1f}]")
        print(f"\nSuccess Criteria:")
        for i, criterion in enumerate(scenario.success_criteria, 1):
            print(f"  {i}. {criterion}")
        print(f"\nNotes: {scenario.notes}")

    @staticmethod
    def print_all_scenarios_summary() -> None:
        """Print a summary of all scenarios."""
        scenarios = ScenarioLoader.get_all()

        print(f"\n{'='*70}")
        print("STANDARD TEST SCENARIOS SUMMARY")
        print(f"{'='*70}")
        print(f"\nTotal Scenarios: {len(scenarios)}\n")

        for i, scenario in enumerate(scenarios, 1):
            metadata = SCENARIO_METADATA.get(scenario.name, {})
            print(f"{i}. {scenario.name.upper():<25} "
                  f"[{metadata.get('difficulty', 'N/A'):<12}] "
                  f"{metadata.get('primary_test', 'N/A')}")

        # Group by difficulty
        print(f"\n{'By Difficulty:':<25}")
        difficulty_order = ["beginner", "intermediate", "advanced", "expert"]
        for difficulty in difficulty_order:
            count = len(ScenarioLoader.get_by_difficulty(difficulty))
            if count > 0:
                print(f"  {difficulty.capitalize():<20} {count} scenario(s)")

    @staticmethod
    def generate_report(results: Dict[str, SimulationResult]) -> str:
        """
        Generate a formatted test report from results.

        Args:
            results: Dictionary mapping scenario names to SimulationResult objects

        Returns:
            Formatted report string

        Examples:
            results = ScenarioLoader.run_all_scenarios()
            report = ScenarioLoader.generate_report(results)
            print(report)
        """
        passed = sum(1 for r in results.values() if r.success)
        failed = sum(1 for r in results.values() if not r.success)
        total = len(results)

        report = []
        report.append("\n" + "="*70)
        report.append("SIMULATION TEST REPORT")
        report.append("="*70)
        report.append(f"\nTotal Scenarios: {total}")
        report.append(f"Passed: {passed} ({100*passed/total:.1f}%)")
        report.append(f"Failed: {failed} ({100*failed/total:.1f}%)")
        report.append("\n" + "-"*70)
        report.append(f"{'Scenario':<25} {'Status':<10} {'Distance':<15} {'Corners':<10}")
        report.append("-"*70)

        for name, result in sorted(results.items()):
            status = "PASS" if result.success else "FAIL"
            distance_str = f"{result.distance_traveled_cm:.1f} cm"
            corners_str = f"{result.corners_detected}"
            report.append(f"{name:<25} {status:<10} {distance_str:<15} {corners_str:<10}")

        report.append("="*70)
        return "\n".join(report)


if __name__ == "__main__":
    # Example usage
    print("Scenario Loader Examples")
    print("="*70)

    # Get all scenarios
    all_scenarios = ScenarioLoader.get_all()
    print(f"\nTotal scenarios: {len(all_scenarios)}")

    # Get by difficulty
    beginner = ScenarioLoader.get_by_difficulty("beginner")
    print(f"Beginner scenarios: {len(beginner)}")
    for s in beginner:
        print(f"  - {s.name}")

    # Print info for one scenario
    scenario = ScenarioLoader.get_by_name("rectangular_room")
    if scenario:
        ScenarioLoader.print_scenario_info(scenario)

    # Print summary
    ScenarioLoader.print_all_scenarios_summary()
