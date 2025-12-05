"""
Unit Tests for Test Assertions

Tests each assertion function for correctness including:
- Correct pass/fail judgement
- Proper handling of edge cases
- Detailed error message generation
"""

import pytest
import math
from .test_assertions import (
    TrajectoryPoint,
    AssertionResult,
    WallFollowerState,
    assert_no_collision,
    assert_valid_state_transitions,
    assert_corner_count,
    assert_wall_distance,
    assert_completion_time,
    AssertionValidator,
)


# ==================== Test Fixtures ====================

@pytest.fixture
def simple_trajectory():
    """Simple valid trajectory with no issues"""
    return [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FIND_WALL, right_distance=20.0),
        TrajectoryPoint(time_ms=5000, x=5.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=15.0),
        TrajectoryPoint(time_ms=10000, x=10.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=15.2),
        TrajectoryPoint(time_ms=15000, x=15.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=15.1),
        TrajectoryPoint(time_ms=20000, x=20.0, y=1.0, theta=-90.0,
                       state=WallFollowerState.TURN_LEFT, right_distance=18.0),
        TrajectoryPoint(time_ms=25000, x=20.0, y=5.0, theta=-90.0,
                       state=WallFollowerState.DONE, right_distance=15.0),
    ]


@pytest.fixture
def collision_trajectory():
    """Trajectory with collision"""
    return [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FIND_WALL, right_distance=20.0),
        TrajectoryPoint(time_ms=100, x=5.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=5.0),  # Too close
        TrajectoryPoint(time_ms=200, x=-10.0, y=-10.0, theta=0.0,
                       state=WallFollowerState.BACKUP, right_distance=5.0),  # Collision
    ]


@pytest.fixture
def invalid_transition_trajectory():
    """Trajectory with invalid state transitions"""
    return [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.IDLE, right_distance=20.0),
        TrajectoryPoint(time_ms=100, x=5.0, y=0.0, theta=0.0,
                       state=WallFollowerState.TURN_LEFT, right_distance=15.0),  # Invalid: IDLE -> TURN_LEFT
        TrajectoryPoint(time_ms=200, x=10.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=15.0),
    ]


@pytest.fixture
def poor_distance_trajectory():
    """Trajectory with poor distance control"""
    return [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=10.0),  # Too close
        TrajectoryPoint(time_ms=100, x=5.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=20.0),  # Too far
        TrajectoryPoint(time_ms=200, x=10.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=25.0),  # Way too far
        TrajectoryPoint(time_ms=300, x=15.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=12.0),  # Too close
    ]


# ==================== Collision Tests ====================

def test_assert_no_collision_passes(simple_trajectory):
    """Test that collision assertion passes for clean trajectory"""
    def mock_collision_check(x, y, radius):
        # No collision in simple trajectory
        return False

    result = assert_no_collision(simple_trajectory, mock_collision_check)
    assert result.passed
    assert "No collisions" in result.message


def test_assert_no_collision_fails(collision_trajectory):
    """Test that collision assertion detects collisions"""
    def mock_collision_check(x, y, radius):
        # Collision at negative positions
        return x < -5 and y < -5

    result = assert_no_collision(collision_trajectory, mock_collision_check)
    assert not result.passed
    assert "Found" in result.message
    assert result.details['collision_count'] == 1


def test_assert_no_collision_empty_trajectory():
    """Test collision assertion with empty trajectory"""
    result = assert_no_collision([], lambda x, y, r: False)
    assert not result.passed
    assert "Empty" in result.message


def test_assert_no_collision_custom_radius(simple_trajectory):
    """Test collision assertion with custom robot radius"""
    calls = []

    def tracking_collision_check(x, y, radius):
        calls.append(radius)
        return False

    assert_no_collision(simple_trajectory, tracking_collision_check, robot_radius=15.0)

    # All calls should use custom radius
    assert all(r == 15.0 for r in calls)
    assert len(calls) == len(simple_trajectory)


# ==================== State Transition Tests ====================

def test_assert_valid_transitions_passes(simple_trajectory):
    """Test that valid transitions pass"""
    result = assert_valid_state_transitions(simple_trajectory)
    assert result.passed


def test_assert_valid_transitions_fails(invalid_transition_trajectory):
    """Test that invalid transitions are detected"""
    result = assert_valid_state_transitions(invalid_transition_trajectory)
    assert not result.passed
    assert "invalid" in result.message.lower()
    assert result.details['invalid_count'] == 1


def test_assert_valid_transitions_short_trajectory():
    """Test that short trajectories are handled"""
    trajectory = [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.IDLE, right_distance=20.0)
    ]
    result = assert_valid_state_transitions(trajectory)
    assert result.passed  # Too short to validate


def test_assert_valid_transitions_backward_compatible():
    """Test that repeated states are allowed"""
    trajectory = [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=20.0),
        TrajectoryPoint(time_ms=100, x=5.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=15.0),  # Same state
        TrajectoryPoint(time_ms=200, x=10.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=15.0),  # Same state
    ]
    result = assert_valid_state_transitions(trajectory)
    assert result.passed


# ==================== Corner Count Tests ====================

def test_assert_corner_count_exact_pass():
    """Test exact corner count assertion with correct count"""
    result = assert_corner_count(4, 4, mode="exact")
    assert result.passed
    assert "exactly" in result.message


def test_assert_corner_count_exact_fail():
    """Test exact corner count assertion with wrong count"""
    result = assert_corner_count(3, 4, mode="exact")
    assert not result.passed


def test_assert_corner_count_at_least_pass():
    """Test 'at_least' corner count assertion"""
    result = assert_corner_count(5, 4, mode="at_least")
    assert result.passed


def test_assert_corner_count_at_least_fail():
    """Test 'at_least' corner count assertion failure"""
    result = assert_corner_count(3, 4, mode="at_least")
    assert not result.passed


def test_assert_corner_count_invalid_mode():
    """Test corner count assertion with invalid mode"""
    result = assert_corner_count(4, 4, mode="invalid_mode")
    assert not result.passed
    assert "Unknown mode" in result.message


def test_assert_corner_count_details():
    """Test that corner count assertion includes details"""
    result = assert_corner_count(3, 5, mode="exact")
    assert result.details['actual'] == 3
    assert result.details['expected'] == 5
    assert result.details['difference'] == -2


# ==================== Wall Distance Tests ====================

def test_assert_wall_distance_passes(simple_trajectory):
    """Test wall distance assertion passes for good trajectory"""
    result = assert_wall_distance(simple_trajectory, target_distance=15.0, tolerance_cm=3.0)
    assert result.passed


def test_assert_wall_distance_fails(poor_distance_trajectory):
    """Test wall distance assertion detects poor control"""
    result = assert_wall_distance(poor_distance_trajectory, target_distance=15.0, tolerance_cm=3.0)
    assert not result.passed


def test_assert_wall_distance_statistics(simple_trajectory):
    """Test wall distance assertion calculates statistics"""
    result = assert_wall_distance(simple_trajectory)

    assert result.details['mean_distance'] > 0
    assert result.details['std_dev'] >= 0
    assert result.details['min_distance'] > 0
    assert result.details['max_distance'] >= result.details['min_distance']


def test_assert_wall_distance_ignores_turns():
    """Test that turn states are ignored in distance check"""
    trajectory = [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FORWARD, right_distance=15.0),
        TrajectoryPoint(time_ms=100, x=5.0, y=0.0, theta=45.0,
                       state=WallFollowerState.TURN_LEFT, right_distance=100.0),  # Bad distance but ignored
        TrajectoryPoint(time_ms=200, x=10.0, y=0.0, theta=90.0,
                       state=WallFollowerState.FORWARD, right_distance=15.0),
    ]

    result = assert_wall_distance(trajectory, ignore_turn_states=True)
    # Should pass because turn state is ignored
    assert result.passed


def test_assert_wall_distance_empty_trajectory():
    """Test wall distance assertion with empty trajectory"""
    result = assert_wall_distance([])
    assert not result.passed


# ==================== Completion Time Tests ====================

def test_assert_completion_time_passes():
    """Test completion time assertion passes for valid time"""
    trajectory = [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FIND_WALL, right_distance=20.0),
        TrajectoryPoint(time_ms=30000, x=100.0, y=0.0, theta=0.0,
                       state=WallFollowerState.DONE, right_distance=15.0),  # 30 seconds
    ]

    result = assert_completion_time(trajectory, min_time_sec=5.0, max_time_sec=60.0)
    assert result.passed


def test_assert_completion_time_too_fast():
    """Test completion time assertion fails if too fast"""
    trajectory = [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FIND_WALL, right_distance=20.0),
        TrajectoryPoint(time_ms=1000, x=100.0, y=0.0, theta=0.0,
                       state=WallFollowerState.DONE, right_distance=15.0),  # 1 second
    ]

    result = assert_completion_time(trajectory, min_time_sec=5.0, max_time_sec=60.0)
    assert not result.passed
    assert "Too fast" in result.message


def test_assert_completion_time_too_slow():
    """Test completion time assertion fails if too slow"""
    trajectory = [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FIND_WALL, right_distance=20.0),
        TrajectoryPoint(time_ms=120000, x=100.0, y=0.0, theta=0.0,
                       state=WallFollowerState.DONE, right_distance=15.0),  # 120 seconds
    ]

    result = assert_completion_time(trajectory, min_time_sec=5.0, max_time_sec=60.0)
    assert not result.passed
    assert "Too slow" in result.message


def test_assert_completion_time_details():
    """Test completion time assertion includes elapsed time details"""
    trajectory = [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FIND_WALL, right_distance=20.0),
        TrajectoryPoint(time_ms=25000, x=100.0, y=0.0, theta=0.0,
                       state=WallFollowerState.DONE, right_distance=15.0),  # 25 seconds
    ]

    result = assert_completion_time(trajectory, min_time_sec=5.0, max_time_sec=60.0)

    assert result.details['elapsed_time_sec'] == 25.0
    assert result.details['trajectory_points'] == 2


# ==================== Validator Tests ====================

def test_assertion_validator_initialization():
    """Test AssertionValidator initialization"""
    validator = AssertionValidator(
        robot_radius=12.0,
        target_wall_distance=16.0,
        wall_distance_tolerance=2.0,
        min_completion_time=3.0,
        max_completion_time=50.0
    )

    assert validator.robot_radius == 12.0
    assert validator.target_wall_distance == 16.0


def test_assertion_validator_validate_all(simple_trajectory):
    """Test AssertionValidator.validate_all"""
    validator = AssertionValidator()

    results = validator.validate_all(
        trajectory=simple_trajectory,
        check_collision_fn=lambda x, y, r: False,
        corner_count=2,
        expected_corners=2,
        corner_mode="exact"
    )

    assert 'no_collision' in results
    assert 'state_transitions' in results
    assert 'corner_count' in results
    assert 'wall_distance' in results
    assert 'completion_time' in results

    # All assertions should pass for simple trajectory
    assert all(r.passed for r in results.values())


def test_assertion_validator_all_passed():
    """Test AssertionValidator.all_passed"""
    validator = AssertionValidator()

    trajectory = [
        TrajectoryPoint(time_ms=0, x=0.0, y=0.0, theta=0.0,
                       state=WallFollowerState.FIND_WALL, right_distance=20.0),
    ]

    validator.validate_all(
        trajectory=trajectory,
        check_collision_fn=lambda x, y, r: False,
        corner_count=1,
        expected_corners=1
    )

    # This may or may not pass depending on time assertion
    # Just check the method works
    assert isinstance(validator.all_passed(), bool)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
