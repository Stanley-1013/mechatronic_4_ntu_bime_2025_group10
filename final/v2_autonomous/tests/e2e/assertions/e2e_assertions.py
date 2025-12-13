"""
E2E Simulation Result Classes and Assertions

Provides dataclasses for E2E test results and validation assertions.
This module complements the existing test_assertions.py for specific
E2E simulation validation.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
import math


@dataclass
class TrajectoryPoint:
    """A single point in the robot's trajectory"""
    x: float
    y: float
    yaw: float  # degrees
    front_distance: int  # cm
    right_distance: int  # cm
    left_pwm: int
    right_pwm: int
    timestamp: float  # seconds


@dataclass
class SimulationResult:
    """Complete result of a simulation run"""
    trajectory: List[TrajectoryPoint]
    corner_count: int
    completion_time: float  # seconds
    collided: bool
    collision_point: Optional[Tuple[float, float]] = None
    final_position: Optional[Tuple[float, float, float]] = None
    distance_traveled: float = 0.0
    total_steps: int = 0
    blocked_steps: int = 0


class SimulationAssertion:
    """Base class for simulation assertions"""

    def __init__(self, name: str, required: bool = True):
        self.name = name
        self.required = required
        self.passed = False
        self.message = ""

    def evaluate(self, result: SimulationResult) -> bool:
        """
        Evaluate the assertion against result.

        Returns:
            True if assertion passes, False otherwise
        """
        raise NotImplementedError

    def __str__(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        required_str = "(required)" if self.required else "(optional)"
        return f"[{status}] {self.name} {required_str}: {self.message}"


class NoCollisionAssertion(SimulationAssertion):
    """Assert that robot did not collide with walls"""

    def __init__(self, required: bool = True):
        super().__init__("no_collision", required)

    def evaluate(self, result: SimulationResult) -> bool:
        self.passed = not result.collided
        if self.passed:
            self.message = f"No collision detected (final position: {result.final_position})"
        else:
            self.message = f"Collision at {result.collision_point}"
        return self.passed


class MinimumCornersAssertion(SimulationAssertion):
    """Assert that robot detected minimum number of corners"""

    def __init__(self, min_corners: int, required: bool = True):
        super().__init__(f"corners >= {min_corners}", required)
        self.min_corners = min_corners

    def evaluate(self, result: SimulationResult) -> bool:
        self.passed = result.corner_count >= self.min_corners
        self.message = f"Detected {result.corner_count} corners (required: {self.min_corners})"
        return self.passed


class MaximumTimeAssertion(SimulationAssertion):
    """Assert that simulation completed within time limit"""

    def __init__(self, max_time_s: float, required: bool = True):
        super().__init__(f"time <= {max_time_s}s", required)
        self.max_time_s = max_time_s

    def evaluate(self, result: SimulationResult) -> bool:
        self.passed = result.completion_time <= self.max_time_s
        self.message = f"Completed in {result.completion_time:.2f}s (limit: {self.max_time_s}s)"
        return self.passed


class MinimumDistanceAssertion(SimulationAssertion):
    """Assert that robot traveled minimum distance"""

    def __init__(self, min_distance: float, required: bool = True):
        super().__init__(f"distance > {min_distance}cm", required)
        self.min_distance = min_distance

    def evaluate(self, result: SimulationResult) -> bool:
        self.passed = result.distance_traveled >= self.min_distance
        self.message = (
            f"Traveled {result.distance_traveled:.1f}cm (required: {self.min_distance}cm)"
        )
        return self.passed


class CompletionRatioAssertion(SimulationAssertion):
    """Assert that robot completed a minimum ratio of expected path"""

    def __init__(self, min_ratio: float, expected_distance: float, required: bool = True):
        super().__init__(f"completion >= {min_ratio*100:.0f}%", required)
        self.min_ratio = min_ratio
        self.expected_distance = expected_distance

    def evaluate(self, result: SimulationResult) -> bool:
        if self.expected_distance <= 0:
            self.passed = True
            self.message = "Expected distance not set"
            return self.passed

        ratio = result.distance_traveled / self.expected_distance
        self.passed = ratio >= self.min_ratio
        self.message = (
            f"Completed {ratio*100:.1f}% of path "
            f"({result.distance_traveled:.1f}cm / {self.expected_distance:.1f}cm)"
        )
        return self.passed


class StabilityAssertion(SimulationAssertion):
    """Assert that simulation ran stably (low jitter in motor commands)"""

    def __init__(self, max_jitter: int = 50, required: bool = False):
        super().__init__(f"stability (jitter <= {max_jitter})", required)
        self.max_jitter = max_jitter

    def evaluate(self, result: SimulationResult) -> bool:
        if len(result.trajectory) < 2:
            self.passed = True
            self.message = "Insufficient trajectory data"
            return self.passed

        # Calculate jitter as standard deviation of PWM changes
        left_diffs = []
        right_diffs = []

        for i in range(1, len(result.trajectory)):
            prev = result.trajectory[i - 1]
            curr = result.trajectory[i]
            left_diffs.append(abs(curr.left_pwm - prev.left_pwm))
            right_diffs.append(abs(curr.right_pwm - prev.right_pwm))

        avg_left_jitter = sum(left_diffs) / len(left_diffs) if left_diffs else 0
        avg_right_jitter = sum(right_diffs) / len(right_diffs) if right_diffs else 0
        max_jitter_observed = max(avg_left_jitter, avg_right_jitter)

        self.passed = max_jitter_observed <= self.max_jitter
        self.message = (
            f"Max jitter: {max_jitter_observed:.1f} PWM units "
            f"(limit: {self.max_jitter})"
        )
        return self.passed


class BlockingToleranceAssertion(SimulationAssertion):
    """Assert that simulation tolerated blocking events"""

    def __init__(self, max_blocked_ratio: float = 0.3, required: bool = False):
        super().__init__(f"blocking_tolerance (<= {max_blocked_ratio*100:.0f}%)", required)
        self.max_blocked_ratio = max_blocked_ratio

    def evaluate(self, result: SimulationResult) -> bool:
        if result.total_steps == 0:
            self.passed = True
            self.message = "No steps recorded"
            return self.passed

        blocked_ratio = result.blocked_steps / result.total_steps
        self.passed = blocked_ratio <= self.max_blocked_ratio
        self.message = (
            f"Blocked {blocked_ratio*100:.1f}% of steps "
            f"({result.blocked_steps}/{result.total_steps})"
        )
        return self.passed


def calculate_trajectory_distance(trajectory: List[TrajectoryPoint]) -> float:
    """Calculate total distance traveled from trajectory"""
    if len(trajectory) < 2:
        return 0.0

    total_distance = 0.0
    for i in range(1, len(trajectory)):
        prev = trajectory[i - 1]
        curr = trajectory[i]
        dx = curr.x - prev.x
        dy = curr.y - prev.y
        distance = math.sqrt(dx * dx + dy * dy)
        total_distance += distance

    return total_distance


def count_corners(trajectory: List[TrajectoryPoint], angle_threshold: float = 15.0) -> int:
    """
    Count significant heading changes in trajectory.

    Args:
        trajectory: List of trajectory points
        angle_threshold: Minimum angle change to count as corner (degrees)

    Returns:
        Number of corners detected
    """
    if len(trajectory) < 3:
        return 0

    corners = 0
    prev_yaw = trajectory[0].yaw

    for point in trajectory[1:]:
        yaw_change = abs(point.yaw - prev_yaw)

        # Normalize to 0-180 range
        if yaw_change > 180:
            yaw_change = 360 - yaw_change

        if yaw_change >= angle_threshold:
            corners += 1
            prev_yaw = point.yaw

    return corners


def check_trajectory_collision(
    trajectory: List[TrajectoryPoint],
    min_distance_to_wall: float = 5.0
) -> Tuple[bool, Optional[Tuple[float, float]]]:
    """
    Check if trajectory has any positions too close to walls.

    This is a heuristic check based on sensor readings.

    Args:
        trajectory: List of trajectory points
        min_distance_to_wall: Minimum safe distance to wall in cm

    Returns:
        Tuple of (collided, collision_point)
    """
    for point in trajectory:
        # Simple heuristic: if either sensor reads dangerously close
        if (point.front_distance < min_distance_to_wall or
            point.right_distance < min_distance_to_wall):
            return True, (point.x, point.y)

    return False, None
