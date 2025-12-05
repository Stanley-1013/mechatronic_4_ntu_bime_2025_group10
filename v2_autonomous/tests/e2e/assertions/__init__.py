"""
Test Assertions Module

Provides comprehensive assertions for validating wall-follower robot behavior:
- Collision detection
- State transition validation
- Corner counting
- Distance maintenance
- Completion time constraints
"""

from .test_assertions import (
    TrajectoryPoint,
    AssertionResult,
    assert_no_collision,
    assert_valid_state_transitions,
    assert_corner_count,
    assert_wall_distance,
    assert_completion_time,
    AssertionValidator,
    StateTransitionRule,
    WallFollowerState,
)

__all__ = [
    'TrajectoryPoint',
    'AssertionResult',
    'assert_no_collision',
    'assert_valid_state_transitions',
    'assert_corner_count',
    'assert_wall_distance',
    'assert_completion_time',
    'AssertionValidator',
    'StateTransitionRule',
    'WallFollowerState',
]
