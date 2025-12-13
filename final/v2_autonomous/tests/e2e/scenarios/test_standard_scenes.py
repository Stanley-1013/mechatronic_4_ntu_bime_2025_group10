"""
Test Scenario Validation Tests

Validates that all standard test scenarios:
1. Can be created successfully
2. Have valid starting positions (not inside walls)
3. Have reasonable parameters
4. Have correct environment setup
"""

import sys
import os
import math
import pytest

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from scenarios.standard_scenes import (
    TestScenario,
    create_rectangular_room,
    create_l_corridor,
    create_u_turn,
    create_narrow_corridor,
    create_open_space,
    get_all_scenarios,
)
from simulator.concrete_virtual_environment import ConcreteVirtualEnvironment


class TestScenarioCreation:
    """Test that all scenarios can be created without errors."""

    def test_rectangular_room_creation(self):
        """Test rectangular room scenario creation."""
        scenario = create_rectangular_room()
        assert scenario is not None
        assert scenario.name == "rectangular_room"
        assert isinstance(scenario.environment, ConcreteVirtualEnvironment)

    def test_l_corridor_creation(self):
        """Test L-corridor scenario creation."""
        scenario = create_l_corridor()
        assert scenario is not None
        assert scenario.name == "l_corridor"
        assert isinstance(scenario.environment, ConcreteVirtualEnvironment)

    def test_u_turn_creation(self):
        """Test U-turn scenario creation."""
        scenario = create_u_turn()
        assert scenario is not None
        assert scenario.name == "u_turn"
        assert isinstance(scenario.environment, ConcreteVirtualEnvironment)

    def test_narrow_corridor_creation(self):
        """Test narrow corridor scenario creation."""
        scenario = create_narrow_corridor()
        assert scenario is not None
        assert scenario.name == "narrow_corridor"
        assert isinstance(scenario.environment, ConcreteVirtualEnvironment)

    def test_open_space_creation(self):
        """Test open space scenario creation."""
        scenario = create_open_space()
        assert scenario is not None
        assert scenario.name == "open_space"
        assert isinstance(scenario.environment, ConcreteVirtualEnvironment)

    def test_get_all_scenarios(self):
        """Test that get_all_scenarios returns all 5 scenarios."""
        scenarios = get_all_scenarios()
        assert len(scenarios) == 5
        names = {s.name for s in scenarios}
        expected_names = {
            "rectangular_room",
            "l_corridor",
            "u_turn",
            "narrow_corridor",
            "open_space",
        }
        assert names == expected_names


class TestScenarioStructure:
    """Test that all scenarios have valid structure."""

    @pytest.mark.parametrize("create_func", [
        create_rectangular_room,
        create_l_corridor,
        create_u_turn,
        create_narrow_corridor,
        create_open_space,
    ])
    def test_scenario_has_required_fields(self, create_func):
        """Test that scenario has all required fields."""
        scenario = create_func()

        # Check all required fields exist
        assert hasattr(scenario, 'name')
        assert hasattr(scenario, 'description')
        assert hasattr(scenario, 'environment')
        assert hasattr(scenario, 'start_position')
        assert hasattr(scenario, 'expected_corners')
        assert hasattr(scenario, 'max_duration_s')
        assert hasattr(scenario, 'success_criteria')

        # Check field types
        assert isinstance(scenario.name, str)
        assert isinstance(scenario.description, str)
        assert isinstance(scenario.environment, ConcreteVirtualEnvironment)
        assert isinstance(scenario.start_position, tuple)
        assert len(scenario.start_position) == 3
        assert isinstance(scenario.expected_corners, int)
        assert isinstance(scenario.max_duration_s, float)
        assert isinstance(scenario.success_criteria, list)
        assert all(isinstance(c, str) for c in scenario.success_criteria)

    @pytest.mark.parametrize("create_func", [
        create_rectangular_room,
        create_l_corridor,
        create_u_turn,
        create_narrow_corridor,
        create_open_space,
    ])
    def test_position_values_reasonable(self, create_func):
        """Test that start position has reasonable values."""
        scenario = create_func()
        x, y, theta = scenario.start_position

        # Coordinates should be positive and within reasonable bounds
        assert x >= 0, f"{scenario.name}: x={x} is negative"
        assert y >= 0, f"{scenario.name}: y={y} is negative"

        # Theta should be in [-180, 360] degrees range
        assert -180 <= theta <= 360, f"{scenario.name}: theta={theta} out of range"

        # Duration should be reasonable (5-120 seconds)
        assert 5 <= scenario.max_duration_s <= 120, \
            f"{scenario.name}: duration={scenario.max_duration_s}s unreasonable"

    @pytest.mark.parametrize("create_func", [
        create_rectangular_room,
        create_l_corridor,
        create_u_turn,
        create_narrow_corridor,
        create_open_space,
    ])
    def test_corners_count_reasonable(self, create_func):
        """Test that expected corner count is reasonable."""
        scenario = create_func()

        # Expected corners should be 0-8 (reasonable for most scenarios)
        assert 0 <= scenario.expected_corners <= 8, \
            f"{scenario.name}: corners={scenario.expected_corners} unreasonable"

        # Narrow corridor should have 0 corners
        if scenario.name == "narrow_corridor":
            assert scenario.expected_corners == 0

        # Most scenarios should have 4+ corners
        if scenario.name in ["rectangular_room", "l_corridor", "u_turn", "open_space"]:
            assert scenario.expected_corners >= 4


class TestStartPositionValidity:
    """Test that start positions are not inside walls."""

    ROBOT_RADIUS = 10.0  # cm (approximation: 20cm width / 2)

    @pytest.mark.parametrize("create_func,expected_valid", [
        (create_rectangular_room, True),
        (create_l_corridor, True),
        (create_u_turn, True),
        (create_narrow_corridor, True),
        (create_open_space, True),
    ])
    def test_start_position_not_in_collision(self, create_func, expected_valid):
        """Test that starting position does not collide with walls."""
        scenario = create_func()
        x, y, theta = scenario.start_position

        # Check collision at start position
        in_collision = scenario.environment.check_collision(x, y, self.ROBOT_RADIUS)

        if expected_valid:
            assert not in_collision, \
                f"{scenario.name}: Start position ({x}, {y}) is in collision!"

    def test_rectangular_room_start_position(self):
        """Verify rectangular room start position specifically."""
        scenario = create_rectangular_room()
        x, y, theta = scenario.start_position

        # Should be at center horizontally, lower portion vertically
        assert 70 <= x <= 80, f"x={x} not centered"
        assert 20 <= y <= 40, f"y={y} not in lower portion"
        assert 85 <= theta <= 95, f"theta={theta} not facing up"

    def test_narrow_corridor_start_position(self):
        """Verify narrow corridor start position is truly centered."""
        scenario = create_narrow_corridor()
        x, y, theta = scenario.start_position

        # Should be centered in 30cm wide corridor
        assert 10 <= x <= 20, f"x={x} not properly centered"
        assert 10 <= y <= 30, f"y={y} not properly positioned"
        assert 85 <= theta <= 95, f"theta={theta} not facing up"


class TestEnvironmentValidity:
    """Test that environments are properly constructed."""

    def test_rectangular_room_walls(self):
        """Test rectangular room has 4 walls."""
        scenario = create_rectangular_room()
        # A rectangular room should have 4 wall segments
        assert scenario.environment.get_wall_count() == 4

    def test_l_corridor_walls(self):
        """Test L-corridor has appropriate wall count."""
        scenario = create_l_corridor()
        # L-corridor should have more than 4 walls due to two segments
        count = scenario.environment.get_wall_count()
        assert count >= 10, f"L-corridor has {count} walls, expected >= 10"

    def test_u_turn_walls(self):
        """Test U-turn has appropriate wall count."""
        scenario = create_u_turn()
        count = scenario.environment.get_wall_count()
        assert count >= 8, f"U-turn has {count} walls, expected >= 8"

    def test_narrow_corridor_walls(self):
        """Test narrow corridor has 4 walls."""
        scenario = create_narrow_corridor()
        assert scenario.environment.get_wall_count() == 4

    def test_open_space_walls(self):
        """Test open space has boundary and obstacle."""
        scenario = create_open_space()
        # Boundary (4) + central obstacle (4) = 8
        count = scenario.environment.get_wall_count()
        assert count >= 8, f"Open space has {count} walls, expected >= 8"


class TestRaycastingFromStart:
    """Test that raycasting works from start positions."""

    def test_rectangular_room_raycast(self):
        """Test raycasting in rectangular room."""
        scenario = create_rectangular_room()
        x, y, theta = scenario.start_position

        # Should detect walls when raycasting
        for angle_offset in [0, 90, 180, 270]:
            angle = (theta + angle_offset) % 360
            distance = scenario.environment.ray_cast(x, y, angle, max_range=500)

            # Should detect something within reasonable distance
            assert 0 < distance < 500, \
                f"Raycast at {angle}° returned invalid distance {distance}"

    def test_narrow_corridor_raycast(self):
        """Test raycasting in narrow corridor."""
        scenario = create_narrow_corridor()
        x, y, theta = scenario.start_position

        # Forward should detect exit wall (about 180 cm away)
        distance_forward = scenario.environment.ray_cast(x, y, theta, max_range=500)
        assert distance_forward > 100, \
            f"Forward raycast should be > 100cm, got {distance_forward}cm"

        # Left and right should detect nearby walls (~15cm away)
        distance_left = scenario.environment.ray_cast(x, y, theta + 90, max_range=500)
        distance_right = scenario.environment.ray_cast(x, y, theta - 90, max_range=500)

        assert distance_left < 30, f"Left wall should be < 30cm, got {distance_left}cm"
        assert distance_right < 30, f"Right wall should be < 30cm, got {distance_right}cm"


class TestSuccessCriteria:
    """Test that success criteria are well-formed."""

    @pytest.mark.parametrize("create_func", [
        create_rectangular_room,
        create_l_corridor,
        create_u_turn,
        create_narrow_corridor,
        create_open_space,
    ])
    def test_has_success_criteria(self, create_func):
        """Test that scenario has at least one success criterion."""
        scenario = create_func()
        assert len(scenario.success_criteria) > 0, \
            f"{scenario.name}: No success criteria defined"

    @pytest.mark.parametrize("create_func", [
        create_rectangular_room,
        create_l_corridor,
        create_u_turn,
        create_narrow_corridor,
        create_open_space,
    ])
    def test_criteria_are_meaningful(self, create_func):
        """Test that criteria are not empty strings."""
        scenario = create_func()
        for criterion in scenario.success_criteria:
            assert len(criterion) > 0, \
                f"{scenario.name}: Empty success criterion found"
            assert not criterion.isspace(), \
                f"{scenario.name}: Whitespace-only success criterion found"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
