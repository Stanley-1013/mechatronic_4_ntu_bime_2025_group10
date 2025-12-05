"""
Test Scenario Library

Provides standard test scenarios for wall-follower controller testing.
Each scenario defines an environment, starting position, and success criteria.

Quick Start:
    from scenarios import ScenarioLoader

    # Get a scenario
    scenario = ScenarioLoader.get_by_name("rectangular_room")

    # Run simulation
    result = ScenarioLoader.run_scenario(scenario, max_duration_ms=60000)
    print(f"Success: {result.success}")
    print(f"Distance: {result.distance_traveled_cm:.1f} cm")

    # Run all scenarios
    results = ScenarioLoader.run_all_scenarios()
    report = ScenarioLoader.generate_report(results)
    print(report)
"""

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
from .scenario_loader import (
    ScenarioLoader,
    SimulationResult,
)

__all__ = [
    # TestScenario classes
    'TestScenario',

    # Scenario creation functions
    'create_rectangular_room',
    'create_l_corridor',
    'create_u_turn',
    'create_narrow_corridor',
    'create_open_space',
    'get_all_scenarios',

    # Scenario loader interface
    'ScenarioLoader',
    'SimulationResult',

    # Metadata
    'SCENARIO_METADATA',
]
