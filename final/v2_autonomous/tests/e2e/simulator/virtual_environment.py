"""
Virtual Environment Interface

Defines the simulated environment containing walls, obstacles, and
collision detection for the robot simulation.
"""

from abc import ABC, abstractmethod
from typing import List, Tuple


class VirtualEnvironment(ABC):
    """
    Abstract interface for virtual environment.

    Manages the simulated world containing walls and obstacles.
    Provides ray-casting and collision detection for robot sensing.
    """

    @abstractmethod
    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None:
        """
        Add a line segment wall to the environment.

        Args:
            x1, y1: Start point of wall segment in cm
            x2, y2: End point of wall segment in cm
        """
        pass

    @abstractmethod
    def add_rectangle(self, x: float, y: float, width: float, height: float) -> None:
        """
        Add a rectangular obstacle to the environment.

        Args:
            x, y: Center position in cm
            width: Width in cm
            height: Height in cm
        """
        pass

    @abstractmethod
    def ray_cast(self, x: float, y: float, angle: float,
                 max_range: float = 500.0) -> float:
        """
        Cast a ray from position and return distance to nearest obstacle.

        Simulates ultrasonic sensor behavior.

        Args:
            x, y: Starting position in cm
            angle: Ray direction in degrees (0° = forward)
            max_range: Maximum detection range in cm (default: 500)

        Returns:
            Distance to obstacle in cm, or max_range if no obstacle detected
        """
        pass

    @abstractmethod
    def check_collision(self, x: float, y: float, radius: float) -> bool:
        """
        Check if circle at position collides with any wall.

        Args:
            x, y: Center position in cm
            radius: Circle radius in cm

        Returns:
            True if collision detected, False otherwise
        """
        pass

    @abstractmethod
    def get_closest_obstacle(self, x: float, y: float,
                           angle: float) -> Tuple[float, float]:
        """
        Get position of closest obstacle in given direction.

        Args:
            x, y: Starting position in cm
            angle: Direction in degrees

        Returns:
            Tuple of (distance, actual_angle) where:
            - distance: Distance to obstacle in cm
            - actual_angle: Actual angle to closest point
        """
        pass

    @abstractmethod
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        Get simulation world boundaries.

        Returns:
            Tuple of (min_x, min_y, max_x, max_y) in cm
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """
        Clear all walls and obstacles from environment.
        """
        pass

    @abstractmethod
    def get_wall_count(self) -> int:
        """
        Get total number of walls in environment.

        Returns:
            Number of wall segments
        """
        pass
