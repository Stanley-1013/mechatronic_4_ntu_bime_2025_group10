"""
Concrete Virtual Environment Implementation

Implements the VirtualEnvironment interface with:
- 2D coordinate system (x, y) in cm
- Line segment walls with ray-casting
- Circle collision detection
- Rectangular room creation helper
- Ray-line segment intersection algorithm
"""

import math
from typing import List, Tuple, Optional
from .virtual_environment import VirtualEnvironment


class ConcreteVirtualEnvironment(VirtualEnvironment):
    """
    Concrete implementation of virtual environment using 2D walls.

    Features:
    - Line segment walls with ray-casting intersection
    - Gaussian distance field for collision detection
    - Rectangular room creation helper
    - Accurate ray-line intersection calculation
    """

    def __init__(self):
        """Initialize empty virtual environment."""
        self.walls: List[Tuple[float, float, float, float]] = []
        self.bounds = (0.0, 0.0, 100.0, 100.0)  # Default bounds

    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None:
        """
        Add a line segment wall to the environment.

        Args:
            x1, y1: Start point of wall segment in cm
            x2, y2: End point of wall segment in cm
        """
        self.walls.append((x1, y1, x2, y2))
        # Update bounds to include new wall
        self._update_bounds()

    def add_rectangle(self, x: float, y: float, width: float, height: float) -> None:
        """
        Add a rectangular obstacle to the environment.

        Args:
            x, y: Center position in cm
            width: Width in cm
            height: Height in cm
        """
        # Convert center coordinates to corner
        left = x - width / 2
        right = x + width / 2
        bottom = y - height / 2
        top = y + height / 2

        # Add four walls forming rectangle
        self.add_wall(left, bottom, right, bottom)   # Bottom
        self.add_wall(right, bottom, right, top)     # Right
        self.add_wall(right, top, left, top)         # Top
        self.add_wall(left, top, left, bottom)       # Left

    def ray_cast(self, x: float, y: float, angle: float,
                 max_range: float = 500.0) -> float:
        """
        Cast a ray from position and return distance to nearest obstacle.

        Uses ray-line segment intersection algorithm to find the closest
        wall intersection point.

        Args:
            x, y: Starting position in cm
            angle: Ray direction in degrees (0° = right/forward, counterclockwise)
            max_range: Maximum detection range in cm

        Returns:
            Distance to obstacle in cm, or max_range if no obstacle detected
        """
        if not self.walls:
            return max_range

        # Convert angle to radians
        angle_rad = math.radians(angle)

        # Ray direction vector (normalized)
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)

        # Find closest intersection
        min_distance = max_range

        for x1, y1, x2, y2 in self.walls:
            distance = self._ray_segment_intersection(
                x, y, dx, dy, x1, y1, x2, y2, max_range
            )
            if distance < min_distance:
                min_distance = distance

        return min_distance

    def _ray_segment_intersection(
        self,
        ray_x: float, ray_y: float,  # Ray origin
        ray_dx: float, ray_dy: float,  # Ray direction (unit vector)
        seg_x1: float, seg_y1: float,  # Segment start
        seg_x2: float, seg_y2: float,  # Segment end
        max_range: float
    ) -> float:
        """
        Calculate intersection distance between ray and line segment.

        Uses parametric line equations:
        - Ray: P = (ray_x, ray_y) + t * (ray_dx, ray_dy), t >= 0
        - Segment: Q = (seg_x1, seg_y1) + s * (seg_dx, seg_dy), 0 <= s <= 1

        Where t is the distance along the ray, and s is the parameter along
        the segment from 0 to 1.

        Args:
            ray_x, ray_y: Ray origin
            ray_dx, ray_dy: Ray direction (assumed normalized)
            seg_x1, seg_y1: Segment start point
            seg_x2, seg_y2: Segment end point
            max_range: Maximum range to consider

        Returns:
            Distance to intersection or max_range if no intersection
        """
        # Segment direction vector
        seg_dx = seg_x2 - seg_x1
        seg_dy = seg_y2 - seg_y1

        # Vector from ray origin to segment start
        dx_to_seg = seg_x1 - ray_x
        dy_to_seg = seg_y1 - ray_y

        # Use 2D cross product to solve the system:
        # ray_origin + t * ray_dir = seg_start + s * seg_dir
        # Rearranging: t * ray_dir - s * seg_dir = seg_start - ray_origin
        # This is a 2x2 system we can solve using Cramer's rule

        # Determinant of coefficient matrix
        denom = ray_dx * (-seg_dy) - ray_dy * (-seg_dx)
        denom = ray_dx * seg_dy - ray_dy * seg_dx

        # Check if ray is parallel to segment
        if abs(denom) < 1e-10:
            return max_range

        # Solve for parameters using Cramer's rule
        # t = (dx_to_seg * (-seg_dy) - dy_to_seg * (-seg_dx)) / denom
        # s = (dx_to_seg * ray_dy - dy_to_seg * ray_dx) / denom

        t_numer = dx_to_seg * seg_dy - dy_to_seg * seg_dx
        s_numer = dx_to_seg * ray_dy - dy_to_seg * ray_dx

        t = t_numer / denom
        s = s_numer / denom

        # Check if intersection is valid:
        # t > 0: intersection is in front of ray origin (allow small epsilon for numerical stability)
        # 0 <= s <= 1: intersection is on the segment
        epsilon = 1e-6
        if t > -epsilon and 0 <= s <= 1:
            # Return the actual distance (t parameter for normalized ray direction)
            return max(0, t)

        return max_range

    def check_collision(self, x: float, y: float, radius: float) -> bool:
        """
        Check if circle at position collides with any wall.

        Args:
            x, y: Center position in cm
            radius: Circle radius in cm

        Returns:
            True if collision detected, False otherwise
        """
        for x1, y1, x2, y2 in self.walls:
            distance = self._point_to_segment_distance(x, y, x1, y1, x2, y2)
            if distance < radius:
                return True
        return False

    def _point_to_segment_distance(
        self,
        px: float, py: float,  # Point
        x1: float, y1: float,  # Segment start
        x2: float, y2: float   # Segment end
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
        distance = self.ray_cast(x, y, angle)

        # Calculate actual point
        angle_rad = math.radians(angle)
        hit_x = x + distance * math.cos(angle_rad)
        hit_y = y + distance * math.sin(angle_rad)

        # Calculate angle to hit point
        to_hit_x = hit_x - x
        to_hit_y = hit_y - y
        actual_angle = math.degrees(math.atan2(to_hit_y, to_hit_x))

        return (distance, actual_angle)

    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        Get simulation world boundaries.

        Returns:
            Tuple of (min_x, min_y, max_x, max_y) in cm
        """
        return self.bounds

    def reset(self) -> None:
        """Clear all walls and obstacles from environment."""
        self.walls = []
        self.bounds = (0.0, 0.0, 100.0, 100.0)

    def get_wall_count(self) -> int:
        """
        Get total number of walls in environment.

        Returns:
            Number of wall segments
        """
        return len(self.walls)

    def _update_bounds(self) -> None:
        """Update environment bounds to encompass all walls."""
        if not self.walls:
            self.bounds = (0.0, 0.0, 100.0, 100.0)
            return

        min_x = float('inf')
        min_y = float('inf')
        max_x = float('-inf')
        max_y = float('-inf')

        for x1, y1, x2, y2 in self.walls:
            min_x = min(min_x, x1, x2)
            min_y = min(min_y, y1, y2)
            max_x = max(max_x, x1, x2)
            max_y = max(max_y, y1, y2)

        # Add 10% padding
        width = max_x - min_x
        height = max_y - min_y
        padding_x = width * 0.1 if width > 0 else 10
        padding_y = height * 0.1 if height > 0 else 10

        self.bounds = (
            min_x - padding_x,
            min_y - padding_y,
            max_x + padding_x,
            max_y + padding_y
        )

    @classmethod
    def create_rectangular_room(cls, width: float, height: float) -> 'ConcreteVirtualEnvironment':
        """
        Create a rectangular room environment.

        Args:
            width: Room width in cm
            height: Room height in cm

        Returns:
            VirtualEnvironment with rectangular room walls
        """
        env = cls()

        # Add four walls forming rectangle (origin at bottom-left)
        env.add_wall(0, 0, width, 0)           # Bottom wall
        env.add_wall(width, 0, width, height)  # Right wall
        env.add_wall(width, height, 0, height) # Top wall
        env.add_wall(0, height, 0, 0)          # Left wall

        return env
