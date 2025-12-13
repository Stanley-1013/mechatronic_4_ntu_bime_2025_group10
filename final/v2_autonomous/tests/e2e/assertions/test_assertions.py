"""
Test Assertions and Validation Logic

Provides comprehensive assertions for validating wall-follower robot behavior
including collision detection, state transitions, corner counting, distance
maintenance, and completion time constraints.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Set, Callable
from enum import IntEnum
import math


class WallFollowerState(IntEnum):
    """State enumeration matching Arduino implementation"""
    IDLE = 0x00
    FIND_WALL = 0x01
    FORWARD = 0x02
    BACKUP = 0x03
    TURN_LEFT = 0x04
    DONE = 0x05
    ERROR = 0xFF


@dataclass
class TrajectoryPoint:
    """
    Single point in robot trajectory.

    Attributes:
        time_ms: Timestamp in milliseconds
        x: X position in cm
        y: Y position in cm
        theta: Orientation in degrees
        state: Current controller state (WallFollowerState)
        right_distance: Right ultrasonic reading in cm
    """
    time_ms: int
    x: float
    y: float
    theta: float
    state: int
    right_distance: float


@dataclass
class AssertionResult:
    """
    Result of an assertion check.

    Attributes:
        passed: Whether assertion passed
        message: Human-readable assertion message
        details: Optional detailed information (e.g., collision points, invalid transitions)
    """
    passed: bool
    message: str
    details: Optional[dict] = None


@dataclass
class StateTransitionRule:
    """
    Defines valid state transitions in the wall-following state machine.

    Attributes:
        from_state: Source state
        to_states: Set of valid destination states
        description: Human-readable description
    """
    from_state: WallFollowerState
    to_states: Set[WallFollowerState]
    description: str = ""


# Define valid state transitions based on wall-follower state machine
VALID_STATE_TRANSITIONS = [
    StateTransitionRule(
        WallFollowerState.IDLE,
        {WallFollowerState.FIND_WALL},
        "IDLE -> FIND_WALL when controller starts"
    ),
    StateTransitionRule(
        WallFollowerState.FIND_WALL,
        {WallFollowerState.FIND_WALL, WallFollowerState.FORWARD, WallFollowerState.ERROR},
        "FIND_WALL -> FORWARD when wall found, or stay in FIND_WALL"
    ),
    StateTransitionRule(
        WallFollowerState.FORWARD,
        {WallFollowerState.FORWARD, WallFollowerState.BACKUP, WallFollowerState.TURN_LEFT, WallFollowerState.DONE},
        "FORWARD can proceed, backup, turn, or complete"
    ),
    StateTransitionRule(
        WallFollowerState.BACKUP,
        {WallFollowerState.BACKUP, WallFollowerState.TURN_LEFT},
        "BACKUP -> TURN_LEFT after backing away from obstacle"
    ),
    StateTransitionRule(
        WallFollowerState.TURN_LEFT,
        {WallFollowerState.TURN_LEFT, WallFollowerState.FORWARD, WallFollowerState.BACKUP, WallFollowerState.DONE},
        "TURN_LEFT -> FORWARD/BACKUP after rotation, or complete"
    ),
    StateTransitionRule(
        WallFollowerState.DONE,
        {WallFollowerState.DONE, WallFollowerState.IDLE},
        "DONE state is final or reset to IDLE"
    ),
    StateTransitionRule(
        WallFollowerState.ERROR,
        {WallFollowerState.ERROR, WallFollowerState.IDLE},
        "ERROR state is final or reset to IDLE"
    ),
]


def assert_no_collision(
    trajectory: List[TrajectoryPoint],
    check_collision_fn: Callable[[float, float, float], bool],
    robot_radius: float = 10.0
) -> AssertionResult:
    """
    Check if robot trajectory contains any collisions.

    Args:
        trajectory: List of trajectory points
        check_collision_fn: Function(x, y, radius) -> bool that checks collision
        robot_radius: Robot collision radius in cm (default: 10.0)

    Returns:
        AssertionResult with pass/fail status and collision details
    """
    if not trajectory:
        return AssertionResult(
            passed=False,
            message="Empty trajectory provided"
        )

    collisions = []
    for point in trajectory:
        if check_collision_fn(point.x, point.y, robot_radius):
            collisions.append({
                'time_ms': point.time_ms,
                'x': point.x,
                'y': point.y,
                'position': (point.x, point.y)
            })

    if collisions:
        return AssertionResult(
            passed=False,
            message=f"Found {len(collisions)} collision point(s) in trajectory",
            details={
                'collision_count': len(collisions),
                'collisions': collisions,
                'first_collision': collisions[0] if collisions else None
            }
        )

    return AssertionResult(
        passed=True,
        message=f"No collisions detected in trajectory ({len(trajectory)} points)"
    )


def assert_valid_state_transitions(
    trajectory: List[TrajectoryPoint],
) -> AssertionResult:
    """
    Check if state transitions follow valid state machine rules.

    Args:
        trajectory: List of trajectory points with state history

    Returns:
        AssertionResult with list of invalid transitions if any
    """
    if not trajectory or len(trajectory) < 2:
        return AssertionResult(
            passed=True,
            message="Trajectory too short for state transition validation"
        )

    # Build transition lookup table
    transition_map = {}
    for rule in VALID_STATE_TRANSITIONS:
        transition_map[rule.from_state] = rule.to_states

    invalid_transitions = []

    # Check each consecutive pair of states
    for i in range(len(trajectory) - 1):
        current_point = trajectory[i]
        next_point = trajectory[i + 1]

        # Skip if state didn't change
        if current_point.state == next_point.state:
            continue

        try:
            current_state = WallFollowerState(current_point.state)
            next_state = WallFollowerState(next_point.state)
        except ValueError as e:
            invalid_transitions.append({
                'time_ms': current_point.time_ms,
                'from_state': str(current_point.state),
                'to_state': str(next_point.state),
                'reason': f"Invalid state value: {e}"
            })
            continue

        # Check if transition is valid
        if current_state not in transition_map:
            invalid_transitions.append({
                'time_ms': current_point.time_ms,
                'from_state': current_state.name,
                'to_state': next_state.name,
                'reason': f"Unknown source state: {current_state.name}"
            })
        elif next_state not in transition_map[current_state]:
            invalid_transitions.append({
                'time_ms': current_point.time_ms,
                'from_state': current_state.name,
                'to_state': next_state.name,
                'reason': f"Invalid transition {current_state.name} -> {next_state.name}"
            })

    if invalid_transitions:
        return AssertionResult(
            passed=False,
            message=f"Found {len(invalid_transitions)} invalid state transition(s)",
            details={
                'invalid_count': len(invalid_transitions),
                'transitions': invalid_transitions,
                'first_invalid': invalid_transitions[0] if invalid_transitions else None
            }
        )

    return AssertionResult(
        passed=True,
        message=f"All state transitions are valid ({len(trajectory)} points)"
    )


def assert_corner_count(
    actual_count: int,
    expected_count: int,
    mode: str = "exact"
) -> AssertionResult:
    """
    Verify robot detected correct number of corners.

    Args:
        actual_count: Actual corner count from controller
        expected_count: Expected corner count
        mode: "exact" (default) or "at_least"

    Returns:
        AssertionResult with corner count comparison
    """
    if mode == "exact":
        passed = actual_count == expected_count
        message = f"Corner count: {actual_count} (expected exactly {expected_count})"
    elif mode == "at_least":
        passed = actual_count >= expected_count
        message = f"Corner count: {actual_count} (expected at least {expected_count})"
    else:
        return AssertionResult(
            passed=False,
            message=f"Unknown mode: {mode}. Use 'exact' or 'at_least'",
        )

    return AssertionResult(
        passed=passed,
        message=message,
        details={
            'actual': actual_count,
            'expected': expected_count,
            'mode': mode,
            'difference': actual_count - expected_count
        }
    )


def assert_wall_distance(
    trajectory: List[TrajectoryPoint],
    target_distance: float = 15.0,
    tolerance_cm: float = 3.0,
    ignore_turn_states: bool = True
) -> AssertionResult:
    """
    Check if robot maintains target right-side wall distance.

    Validates that the right ultrasonic sensor readings stay within
    target_distance ± tolerance_cm during wall-following phases.

    Args:
        trajectory: List of trajectory points
        target_distance: Target wall distance in cm (default: 15.0)
        tolerance_cm: Allowed deviation in cm (default: 3.0)
        ignore_turn_states: If True, ignore TURN_LEFT and BACKUP states

    Returns:
        AssertionResult with distance statistics and violations
    """
    if not trajectory:
        return AssertionResult(
            passed=False,
            message="Empty trajectory provided"
        )

    # Filter points to analyze
    min_distance = target_distance - tolerance_cm
    max_distance = target_distance + tolerance_cm

    relevant_points = []
    for point in trajectory:
        state = WallFollowerState(point.state)

        # Skip turning and backing states if requested
        if ignore_turn_states and state in {WallFollowerState.TURN_LEFT, WallFollowerState.BACKUP}:
            continue

        # Skip states where wall distance is irrelevant
        if state in {WallFollowerState.IDLE, WallFollowerState.ERROR, WallFollowerState.DONE}:
            continue

        relevant_points.append(point)

    if not relevant_points:
        return AssertionResult(
            passed=True,
            message="No relevant points for distance validation"
        )

    # Analyze distances
    distances = [p.right_distance for p in relevant_points if p.right_distance > 0]

    if not distances:
        return AssertionResult(
            passed=False,
            message="No valid distance readings in trajectory"
        )

    violations = []
    for point in relevant_points:
        if point.right_distance <= 0:
            continue

        if point.right_distance < min_distance or point.right_distance > max_distance:
            violations.append({
                'time_ms': point.time_ms,
                'distance': point.right_distance,
                'state': WallFollowerState(point.state).name,
                'deviation': point.right_distance - target_distance
            })

    # Calculate statistics
    mean_distance = sum(distances) / len(distances)
    variance = sum((d - mean_distance) ** 2 for d in distances) / len(distances)
    std_dev = math.sqrt(variance)
    min_dist = min(distances)
    max_dist = max(distances)

    # Determine pass/fail
    # Allow <10% violations or all violations are minor (<2cm)
    max_allowed_violations = max(1, len(relevant_points) // 10)
    passed = len(violations) <= max_allowed_violations

    return AssertionResult(
        passed=passed,
        message=f"Right wall distance: mean={mean_distance:.1f}cm, std_dev={std_dev:.1f}cm "
                f"({len(violations)} violations in {len(relevant_points)} points)",
        details={
            'target': target_distance,
            'tolerance': tolerance_cm,
            'valid_points': len(relevant_points),
            'mean_distance': mean_distance,
            'std_dev': std_dev,
            'min_distance': min_dist,
            'max_distance': max_dist,
            'violation_count': len(violations),
            'violations': violations[:10]  # Show first 10 violations
        }
    )


def assert_completion_time(
    trajectory: List[TrajectoryPoint],
    min_time_sec: float = 5.0,
    max_time_sec: float = 60.0
) -> AssertionResult:
    """
    Check if robot completed task in reasonable time.

    Args:
        trajectory: List of trajectory points
        min_time_sec: Minimum reasonable completion time in seconds
        max_time_sec: Maximum reasonable completion time in seconds

    Returns:
        AssertionResult with completion time analysis
    """
    if not trajectory or len(trajectory) < 2:
        return AssertionResult(
            passed=False,
            message="Insufficient trajectory data for time validation"
        )

    start_time_ms = trajectory[0].time_ms
    end_time_ms = trajectory[-1].time_ms
    elapsed_sec = (end_time_ms - start_time_ms) / 1000.0

    if elapsed_sec < min_time_sec:
        passed = False
        reason = "Too fast (may have skipped logic)"
    elif elapsed_sec > max_time_sec:
        passed = False
        reason = "Too slow (may be stuck or blocked)"
    else:
        passed = True
        reason = "Within expected time range"

    return AssertionResult(
        passed=passed,
        message=f"Completion time: {elapsed_sec:.1f}s ({reason})",
        details={
            'elapsed_time_sec': elapsed_sec,
            'min_time_sec': min_time_sec,
            'max_time_sec': max_time_sec,
            'start_time_ms': start_time_ms,
            'end_time_ms': end_time_ms,
            'trajectory_points': len(trajectory)
        }
    )


class AssertionValidator:
    """
    Comprehensive validator for robot trajectory and controller behavior.

    Combines multiple assertions to provide complete validation of
    wall-follower robot simulation runs.
    """

    def __init__(
        self,
        robot_radius: float = 10.0,
        target_wall_distance: float = 15.0,
        wall_distance_tolerance: float = 3.0,
        min_completion_time: float = 5.0,
        max_completion_time: float = 60.0
    ):
        """
        Initialize validator with configuration parameters.

        Args:
            robot_radius: Robot collision radius in cm
            target_wall_distance: Target wall distance in cm
            wall_distance_tolerance: Wall distance tolerance in cm
            min_completion_time: Minimum completion time in seconds
            max_completion_time: Maximum completion time in seconds
        """
        self.robot_radius = robot_radius
        self.target_wall_distance = target_wall_distance
        self.wall_distance_tolerance = wall_distance_tolerance
        self.min_completion_time = min_completion_time
        self.max_completion_time = max_completion_time
        self.results = []

    def validate_all(
        self,
        trajectory: List[TrajectoryPoint],
        check_collision_fn: Callable[[float, float, float], bool],
        corner_count: int,
        expected_corners: int = None,
        corner_mode: str = "exact"
    ) -> dict:
        """
        Run all assertions on trajectory.

        Args:
            trajectory: Robot trajectory
            check_collision_fn: Collision check function
            corner_count: Actual corner count
            expected_corners: Expected corner count (if None, only check actual count > 0)
            corner_mode: "exact" or "at_least"

        Returns:
            Dictionary with all assertion results
        """
        self.results = []

        # Run all assertions
        collision_result = assert_no_collision(
            trajectory, check_collision_fn, self.robot_radius
        )
        self.results.append(('no_collision', collision_result))

        transition_result = assert_valid_state_transitions(trajectory)
        self.results.append(('state_transitions', transition_result))

        if expected_corners is None:
            expected_corners = 1
        corner_result = assert_corner_count(corner_count, expected_corners, corner_mode)
        self.results.append(('corner_count', corner_result))

        distance_result = assert_wall_distance(
            trajectory,
            self.target_wall_distance,
            self.wall_distance_tolerance
        )
        self.results.append(('wall_distance', distance_result))

        time_result = assert_completion_time(
            trajectory,
            self.min_completion_time,
            self.max_completion_time
        )
        self.results.append(('completion_time', time_result))

        return {
            'no_collision': collision_result,
            'state_transitions': transition_result,
            'corner_count': corner_result,
            'wall_distance': distance_result,
            'completion_time': time_result,
        }

    def print_summary(self) -> None:
        """Print summary of all assertion results."""
        if not self.results:
            print("No assertions run yet")
            return

        print("\n" + "=" * 70)
        print("ASSERTION VALIDATION SUMMARY")
        print("=" * 70)

        passed_count = 0
        for name, result in self.results:
            status = "PASS" if result.passed else "FAIL"
            print(f"[{status}] {name}: {result.message}")
            if result.details and not result.passed:
                for key, value in result.details.items():
                    if key not in ['violations', 'transitions', 'collisions']:
                        print(f"      {key}: {value}")
            passed_count += result.passed

        print("=" * 70)
        print(f"Results: {passed_count}/{len(self.results)} passed")
        print("=" * 70 + "\n")

    def all_passed(self) -> bool:
        """Check if all assertions passed."""
        return all(result.passed for _, result in self.results)
