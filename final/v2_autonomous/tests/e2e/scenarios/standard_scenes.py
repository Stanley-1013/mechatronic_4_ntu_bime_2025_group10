"""
Standard Test Scenario Library

Defines 5 standard test scenarios for wall-following controller validation:
1. Rectangular Room - Basic perimeter following
2. L Corridor - Corner handling
3. U Turn - Complex navigation
4. Narrow Corridor - Collision avoidance
5. Open Space - Wall-finding behavior

Each scenario includes:
- Virtual environment definition (walls and obstacles)
- Starting position and orientation (x, y, theta)
- Expected behavior characteristics
- Success criteria
"""

from dataclasses import dataclass
from typing import List, Tuple
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from simulator.concrete_virtual_environment import ConcreteVirtualEnvironment
from simulator.virtual_environment import VirtualEnvironment


@dataclass
class TestScenario:
    """
    Complete test scenario definition.

    Attributes:
        name: Scenario identifier (e.g., "rectangular_room")
        description: Human-readable scenario description
        environment: VirtualEnvironment containing walls and obstacles
        start_position: Tuple of (x, y, theta) in cm and degrees
                       theta: 0° = right, 90° = up, counter-clockwise positive
        expected_corners: Expected number of corners to encounter
        max_duration_s: Maximum test duration in seconds
        success_criteria: List of criteria that must be met for test pass
                         Examples: "no_collision", "corners >= 4", "distance > 100"
        notes: Additional notes about scenario characteristics
    """
    name: str
    description: str
    environment: VirtualEnvironment
    start_position: Tuple[float, float, float]  # (x, y, theta in degrees)
    expected_corners: int
    max_duration_s: float
    success_criteria: List[str]
    notes: str = ""


def create_rectangular_room() -> TestScenario:
    """
    Create rectangular room scenario (300x300 cm) - Competition arena.

    Test case:
    - Room: 300 cm wide × 300 cm tall (competition arena size)
    - Start: Bottom right area (285, 25) facing up (90°)
    - Expected: Complete one full perimeter (4 corners)
    - Success: No collision, detect all 4 corners, travel full perimeter

    Returns:
        TestScenario for rectangular room traversal
    """
    env = ConcreteVirtualEnvironment.create_rectangular_room(300, 300)

    return TestScenario(
        name="rectangular_room",
        description="Competition arena (300x300 cm) perimeter following",
        environment=env,
        start_position=(285, 25, 90),  # Bottom right, facing up, 15cm from right wall (x=300)
        expected_corners=4,
        max_duration_s=120.0,
        success_criteria=[
            "no_collision",
            "corners >= 4",
            "distance_traveled > 1000",  # Perimeter ≈ 1200 cm
        ],
        notes="Competition arena test case for wall-following algorithm. Robot starts at bottom right area, faces up, and should complete full perimeter of 300x300 cm space. Right sensor detects right wall at ~15cm distance (within valid 10-80cm range)."
    )


def create_l_corridor() -> TestScenario:
    """
    Create L-shaped corridor scenario.

    Layout:
    - Horizontal part: 150 cm wide × 40 cm tall
    - Vertical part: 40 cm wide × 120 cm tall
    - Inner corner at (150, 40)
    - Outer corner at (150, 40)

    Test case:
    - Start: Left side of horizontal corridor (20, 20) facing right (0°)
    - Expected: Navigate L-shaped path with inner and outer corners
    - Success: Handle varied corner types correctly

    Returns:
        TestScenario for L-corridor navigation
    """
    env = ConcreteVirtualEnvironment()

    # Horizontal corridor: 150 cm wide × 40 cm tall
    # Bottom wall
    env.add_wall(0, 0, 150, 0)
    # Top wall (left part only)
    env.add_wall(0, 40, 150, 40)
    # Left wall
    env.add_wall(0, 0, 0, 40)
    # Right wall of horizontal part
    env.add_wall(150, 0, 150, 40)

    # Vertical corridor: 40 cm wide × 120 cm tall
    # Left wall (continuation)
    env.add_wall(150, 40, 150, 160)
    # Right wall
    env.add_wall(190, 40, 190, 160)
    # Top wall
    env.add_wall(150, 160, 190, 160)
    # Bottom wall (inner corner side)
    env.add_wall(150, 40, 190, 40)

    # Closing wall to make complete loop
    env.add_wall(190, 0, 190, 40)
    env.add_wall(190, 0, 150, 0)  # Already added as part of horizontal

    return TestScenario(
        name="l_corridor",
        description="L-shaped corridor with inner and outer corners",
        environment=env,
        start_position=(130, 20, 0),  # Left-center of horizontal part, facing right, 20cm from right wall
        expected_corners=4,
        max_duration_s=75.0,
        success_criteria=[
            "no_collision",
            "corners >= 4",
            "visited_right_wall > 50",  # Should traverse significant right wall
        ],
        notes="Tests corner detection and handling for different corner types (convex and concave). Robot starts with right wall at ~20cm distance (within valid 10-80cm range)."
    )


def create_u_turn() -> TestScenario:
    """
    Create U-shaped turn scenario.

    Layout:
    - Entry corridor: 40 cm wide × 60 cm long
    - Curve section: Semicircular path
    - Exit corridor: 40 cm wide × 60 cm long
    - Return path to start

    Test case:
    - Start: Entry corridor entrance (20, 20) facing right (0°)
    - Expected: Complete U-turn and return to start area
    - Success: Navigate tight turn without collision

    Returns:
        TestScenario for U-turn navigation
    """
    env = ConcreteVirtualEnvironment()

    # Entry corridor: 40 cm wide × 80 cm long
    # Bottom wall
    env.add_wall(0, 0, 80, 0)
    # Right wall
    env.add_wall(80, 0, 80, 40)
    # Top wall
    env.add_wall(80, 40, 0, 40)
    # Left wall
    env.add_wall(0, 40, 0, 0)

    # Semicircular turn section (approximated with segments)
    # This creates a U-turn allowing the robot to come back
    # Right side of turn
    env.add_wall(80, 40, 120, 40)   # Horizontal
    env.add_wall(120, 40, 120, 0)   # Vertical (right side)

    # Top connection
    env.add_wall(120, 0, 80, 0)     # Already exists, but explicit

    # Return path (parallel corridor on right side)
    # This creates the "U" shape that robot must navigate
    env.add_wall(120, 40, 160, 40)  # Outer right wall
    env.add_wall(160, 40, 160, 0)   # Back to start
    env.add_wall(160, 0, 120, 0)    # Top of return

    return TestScenario(
        name="u_turn",
        description="U-shaped corridor requiring 180-degree turn and return",
        environment=env,
        start_position=(20, 20, 0),  # Entry, facing right, 60cm from right wall at x=80
        expected_corners=6,  # Multiple turns in U shape
        max_duration_s=90.0,
        success_criteria=[
            "no_collision",
            "corners >= 6",
            "completed_loop = true",
        ],
        notes="Tests handling of complex multi-corner paths and ability to complete closed loops. Robot starts with right wall at ~60cm distance (within valid 10-80cm range)."
    )


def create_narrow_corridor() -> TestScenario:
    """
    Create narrow corridor scenario (30 cm wide).

    Constraint:
    - Robot width: 20 cm
    - Required clearance: 5 cm per side
    - Corridor width: 30 cm (exact minimum safe passage)
    - Length: 200 cm

    Test case:
    - Start: Corridor entrance (15, 20) facing forward (90°)
    - Expected: Navigate through without collision
    - Success: Maintain centerline, no wall contact

    Returns:
        TestScenario for narrow corridor navigation
    """
    env = ConcreteVirtualEnvironment()

    # Narrow corridor: 30 cm wide (0-30) × 200 cm long (0-200)
    # Walls positioned at x=0 and x=30 for full length

    # Left wall
    env.add_wall(0, 0, 0, 200)

    # Right wall
    env.add_wall(30, 0, 30, 200)

    # Bottom wall (entrance)
    env.add_wall(0, 0, 30, 0)

    # Top wall (exit)
    env.add_wall(0, 200, 30, 200)

    return TestScenario(
        name="narrow_corridor",
        description="Tight corridor (30 cm width, robot 20 cm) - collision avoidance test",
        environment=env,
        start_position=(15, 20, 90),  # Corridor center, facing up the corridor, 15cm from right wall
        expected_corners=0,  # No corners, just straight corridor
        max_duration_s=45.0,
        success_criteria=[
            "no_collision",
            "distance_traveled > 150",  # Travel most of corridor length
            "wall_contact_count = 0",
        ],
        notes="Extreme test case for collision detection. Robot maintains centerline with minimal margin (15cm from right wall, within valid 10-80cm range)."
    )


def create_open_space() -> TestScenario:
    """
    Create open space scenario (300x300 cm).

    Challenge:
    - No walls to follow initially
    - Robot must trigger FIND_WALL state
    - Single central obstacle to find

    Test case:
    - Start: Right area of open space (270, 150) facing left (180°)
    - Expected: Trigger FIND_WALL behavior, eventually find a wall
    - Success: Successfully enter wall-following after finding obstacle

    Returns:
        TestScenario for open space exploration
    """
    env = ConcreteVirtualEnvironment()

    # Outer boundary
    env.add_wall(0, 0, 300, 0)        # Bottom
    env.add_wall(300, 0, 300, 300)    # Right
    env.add_wall(300, 300, 0, 300)    # Top
    env.add_wall(0, 300, 0, 0)        # Left

    # Central obstacle (small square in middle)
    # This gives the robot something to find and follow
    center_x, center_y = 150, 150
    obstacle_size = 40

    # Central obstacle
    env.add_wall(center_x - obstacle_size/2, center_y - obstacle_size/2,
                 center_x + obstacle_size/2, center_y - obstacle_size/2)  # Bottom
    env.add_wall(center_x + obstacle_size/2, center_y - obstacle_size/2,
                 center_x + obstacle_size/2, center_y + obstacle_size/2)  # Right
    env.add_wall(center_x + obstacle_size/2, center_y + obstacle_size/2,
                 center_x - obstacle_size/2, center_y + obstacle_size/2)  # Top
    env.add_wall(center_x - obstacle_size/2, center_y + obstacle_size/2,
                 center_x - obstacle_size/2, center_y - obstacle_size/2)  # Left

    return TestScenario(
        name="open_space",
        description="Large open space (300x300 cm) with central obstacle - FIND_WALL test",
        environment=env,
        start_position=(270, 150, 180),  # Right area, facing left (toward right wall at x=300), 30cm from wall
        expected_corners=4,  # From central obstacle
        max_duration_s=60.0,
        success_criteria=[
            "no_collision",
            "entered_find_wall_state = true",
            "eventually_found_wall = true",
        ],
        notes="Tests FIND_WALL state and behavior when no walls are immediately detected. Robot starts 30cm away from right wall (within valid 10-80cm range)."
    )


def get_all_scenarios() -> List[TestScenario]:
    """
    Get all standard test scenarios.

    Returns:
        List of all standard TestScenario objects
    """
    return [
        create_rectangular_room(),
        create_l_corridor(),
        create_u_turn(),
        create_narrow_corridor(),
        create_open_space(),
    ]


# Scenario metadata for reference
SCENARIO_METADATA = {
    "rectangular_room": {
        "difficulty": "beginner",
        "primary_test": "basic_wall_following",
        "environment_type": "enclosed",
        "features": ["closed_loop", "right_walls", "corners"],
    },
    "l_corridor": {
        "difficulty": "intermediate",
        "primary_test": "corner_detection",
        "environment_type": "corridor",
        "features": ["varied_corners", "open_interior", "multiple_segments"],
    },
    "u_turn": {
        "difficulty": "advanced",
        "primary_test": "complex_navigation",
        "environment_type": "corridor",
        "features": ["180_degree_turn", "closed_loop", "multi_corner"],
    },
    "narrow_corridor": {
        "difficulty": "expert",
        "primary_test": "collision_avoidance",
        "environment_type": "constrained",
        "features": ["tight_space", "minimal_margin", "collision_critical"],
    },
    "open_space": {
        "difficulty": "intermediate",
        "primary_test": "wall_finding",
        "environment_type": "open",
        "features": ["no_initial_walls", "find_wall_state", "exploration"],
    },
}
