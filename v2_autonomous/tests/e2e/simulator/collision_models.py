"""
Collision Detection Models and Events

Provides data structures and utilities for collision detection and event logging:
- CollisionEvent: Records collision occurrences
- CollisionState: Tracks active collisions
- EdgeCollisionDetector: Detects state transitions (not colliding -> colliding)
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict
from enum import Enum
import time


class CollisionType(Enum):
    """Types of collision events"""
    WALL = "wall"
    OBSTACLE = "obstacle"
    BOUNDARY = "boundary"
    UNKNOWN = "unknown"


@dataclass
class CollisionEvent:
    """
    Records a collision occurrence in the simulation.

    Attributes:
        time_ms: Simulation time when collision occurred (milliseconds)
        x: Robot center x-coordinate (cm)
        y: Robot center y-coordinate (cm)
        wall_id: Index of wall segment that collision occurred with
        collision_type: Type of collision (wall, obstacle, boundary)
        distance_to_wall: Minimum distance from robot to obstacle (cm)
        yaw: Robot orientation at time of collision (degrees)
    """
    time_ms: float
    x: float
    y: float
    wall_id: int
    collision_type: CollisionType = CollisionType.WALL
    distance_to_wall: float = 0.0
    yaw: float = 0.0

    def __repr__(self) -> str:
        return (f"CollisionEvent(t={self.time_ms:.1f}ms, pos=({self.x:.1f}, {self.y:.1f}), "
                f"wall={self.wall_id}, dist={self.distance_to_wall:.1f}cm, yaw={self.yaw:.1f}°)")


@dataclass
class CollisionState:
    """
    Tracks current collision state of the robot.

    Attributes:
        is_colliding: Whether robot is currently in collision
        active_collisions: List of walls currently in collision with
        last_collision_time: Time of last collision event (ms)
        collision_count: Total number of collision events recorded
    """
    is_colliding: bool = False
    active_collisions: List[int] = field(default_factory=list)
    last_collision_time: float = 0.0
    collision_count: int = 0
    min_distance: float = float('inf')  # Closest obstacle distance

    def __repr__(self) -> str:
        return (f"CollisionState(colliding={self.is_colliding}, "
                f"active={len(self.active_collisions)}, count={self.collision_count})")


class EdgeCollisionDetector:
    """
    Detects collision state transitions (false -> true).

    Only generates collision events when the robot transitions from
    non-colliding to colliding state, not for continuous collisions.
    This avoids spam of events for robots stuck against walls.
    """

    def __init__(self, robot_radius: float = 10.0):
        """
        Initialize edge collision detector.

        Args:
            robot_radius: Robot collision radius in cm
        """
        self.robot_radius = robot_radius
        self.previous_collision_state = False
        self.collision_history: List[CollisionEvent] = []
        self.state = CollisionState()

    def update(self, current_collision: bool, event: Optional[CollisionEvent] = None) -> Optional[CollisionEvent]:
        """
        Update collision detector and detect state transitions.

        Args:
            current_collision: Current collision detection result
            event: Optional collision event details if colliding

        Returns:
            CollisionEvent if transition from non-colliding to colliding detected,
            None otherwise (either no collision or already colliding)
        """
        # Detect transition: False -> True (edge detection)
        if current_collision and not self.previous_collision_state:
            # Collision just started
            if event is None:
                # Create a default event if not provided
                event = CollisionEvent(
                    time_ms=0,
                    x=0,
                    y=0,
                    wall_id=-1,
                    collision_type=CollisionType.UNKNOWN
                )

            self.collision_history.append(event)
            self.state.collision_count += 1
            self.state.last_collision_time = event.time_ms
            self.state.is_colliding = True
            self.state.active_collisions = [event.wall_id] if event.wall_id >= 0 else []

            self.previous_collision_state = True
            return event

        elif not current_collision and self.previous_collision_state:
            # Collision just ended
            self.state.is_colliding = False
            self.state.active_collisions = []
            self.previous_collision_state = False

        else:
            # No state change - just update collision state
            self.state.is_colliding = current_collision
            if event:
                self.state.min_distance = event.distance_to_wall

        self.previous_collision_state = current_collision
        return None

    def get_collision_history(self) -> List[CollisionEvent]:
        """
        Get all recorded collision events.

        Returns:
            List of all collision events in chronological order
        """
        return self.collision_history.copy()

    def reset(self) -> None:
        """Clear collision history and state"""
        self.previous_collision_state = False
        self.collision_history.clear()
        self.state = CollisionState()


class CollisionDetector:
    """
    High-level collision detection with event generation.

    Integrates environment and physics to detect collisions and
    generate detailed collision events.
    """

    def __init__(self, robot_radius: float = 10.0):
        """
        Initialize collision detector.

        Args:
            robot_radius: Robot collision radius in cm (default: 10cm)
        """
        self.robot_radius = robot_radius
        self.edge_detector = EdgeCollisionDetector(robot_radius)
        self.simulation_time_ms = 0.0

    def check_collision_and_generate_event(
        self,
        environment,  # VirtualEnvironment instance
        x: float,
        y: float,
        yaw: float = 0.0
    ) -> Optional[CollisionEvent]:
        """
        Check collision and generate event if state transition occurs.

        Args:
            environment: VirtualEnvironment instance for collision checking
            x: Robot x position (cm)
            y: Robot y position (cm)
            yaw: Robot orientation (degrees)

        Returns:
            CollisionEvent if collision just started, None otherwise
        """
        # Check if collision occurs
        is_colliding = environment.check_collision(x, y, self.robot_radius)

        # Find closest wall if colliding
        wall_id = -1
        distance_to_wall = self.robot_radius

        if is_colliding:
            # Find which wall caused collision
            min_dist = float('inf')
            for idx, (x1, y1, x2, y2) in enumerate(environment.walls):
                dist = self._point_to_segment_distance(x, y, x1, y1, x2, y2)
                if dist < min_dist:
                    min_dist = dist
                    wall_id = idx
                    distance_to_wall = dist

        # Create collision event if needed
        event = None
        if is_colliding:
            event = CollisionEvent(
                time_ms=self.simulation_time_ms,
                x=x,
                y=y,
                wall_id=wall_id,
                collision_type=CollisionType.WALL,
                distance_to_wall=distance_to_wall,
                yaw=yaw
            )

        # Update edge detector and get transition event
        return self.edge_detector.update(is_colliding, event)

    def update_simulation_time(self, dt: float) -> None:
        """
        Update internal simulation time.

        Args:
            dt: Time delta in seconds
        """
        self.simulation_time_ms += dt * 1000.0

    def get_state(self) -> CollisionState:
        """Get current collision state"""
        return self.edge_detector.state

    def get_history(self) -> List[CollisionEvent]:
        """Get all collision events"""
        return self.edge_detector.get_collision_history()

    def reset(self) -> None:
        """Reset collision detector state"""
        self.edge_detector.reset()
        self.simulation_time_ms = 0.0

    @staticmethod
    def _point_to_segment_distance(
        px: float, py: float,
        x1: float, y1: float,
        x2: float, y2: float
    ) -> float:
        """
        Calculate minimum distance from point to line segment.

        Args:
            px, py: Point coordinates
            x1, y1: Segment start
            x2, y2: Segment end

        Returns:
            Minimum distance from point to segment
        """
        import math

        # Vector from segment start to end
        dx = x2 - x1
        dy = y2 - y1

        # If segment has zero length
        if abs(dx) < 1e-10 and abs(dy) < 1e-10:
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

        # Parameter t for closest point on line (not necessarily segment)
        length_sq = dx * dx + dy * dy
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))

        # Closest point on segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        # Distance to closest point
        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)
