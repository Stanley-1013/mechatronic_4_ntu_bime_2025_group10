"""
Unit tests for ConcreteVirtualEnvironment

Verifies:
1. Ray-casting intersection algorithm
2. Rectangular room creation
3. Distance measurement (front and sides)
4. Collision detection
5. Boundary calculation
"""

import pytest
import math
from .concrete_virtual_environment import ConcreteVirtualEnvironment


class TestRayIntersection:
    """Test ray-line segment intersection algorithm"""

    def test_horizontal_ray_vertical_wall(self):
        """Test ray hitting vertical wall head-on"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)  # Vertical wall at x=100

        # Ray from (0, 50) pointing right (0°)
        distance = env.ray_cast(0, 50, 0)
        assert abs(distance - 100) < 0.1, f"Expected 100, got {distance}"

    def test_vertical_ray_horizontal_wall(self):
        """Test ray hitting horizontal wall head-on"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(0, 100, 100, 100)  # Horizontal wall at y=100

        # Ray from (50, 0) pointing up (90°)
        distance = env.ray_cast(50, 0, 90)
        assert abs(distance - 100) < 0.1, f"Expected 100, got {distance}"

    def test_diagonal_ray_diagonal_wall(self):
        """Test ray hitting perpendicular wall"""
        env = ConcreteVirtualEnvironment()
        # Vertical wall intersecting at 45 degree angle
        env.add_wall(100, 0, 100, 100)  # Vertical wall at x=100

        # Ray from origin pointing at 45° upward-right
        distance = env.ray_cast(0, 0, 45)
        # Ray hits wall at (100, 100), distance = 100/cos(45°) = 100*sqrt(2)
        expected = 100 * math.sqrt(2)
        assert abs(distance - expected) < 1.0, f"Expected ~{expected}, got {distance}"

    def test_no_intersection(self):
        """Test ray that doesn't hit any wall"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 100, 200, 100)  # Wall far away

        # Ray from origin pointing left (180°)
        distance = env.ray_cast(0, 0, 180, max_range=500)
        assert distance == 500, f"Expected max_range (500), got {distance}"

    def test_ray_parallel_to_wall(self):
        """Test ray parallel to wall (no intersection)"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(0, 100, 100, 100)  # Horizontal wall

        # Ray from (0, 50) pointing right (parallel below wall)
        distance = env.ray_cast(0, 50, 0, max_range=500)
        assert distance == 500, f"Expected max_range, got {distance}"

    def test_ray_behind_start_point(self):
        """Test intersection behind ray origin is ignored"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(-100, 0, -100, 100)  # Wall behind ray origin

        # Ray from (0, 50) pointing right (away from wall)
        distance = env.ray_cast(0, 50, 0, max_range=500)
        assert distance == 500, f"Expected max_range, got {distance}"

    def test_multiple_walls_closest_returned(self):
        """Test multiple walls returns closest distance"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(50, 0, 50, 100)   # Wall at x=50
        env.add_wall(100, 0, 100, 100) # Wall at x=100

        # Ray from origin pointing right
        distance = env.ray_cast(0, 50, 0)
        assert abs(distance - 50) < 0.1, f"Should hit closer wall at x=50, got {distance}"


class TestRectangularRoom:
    """Test rectangular room environment"""

    def test_create_rectangular_room(self):
        """Test room creation"""
        width, height = 200, 300
        env = ConcreteVirtualEnvironment.create_rectangular_room(width, height)

        assert env.get_wall_count() == 4
        bounds = env.get_bounds()
        # Bounds should encompass the room with padding
        assert bounds[0] <= 0 and bounds[2] >= width
        assert bounds[1] <= 0 and bounds[3] >= height

    def test_room_center_distances(self):
        """Test distance measurement from room center"""
        width, height = 200, 300
        env = ConcreteVirtualEnvironment.create_rectangular_room(width, height)

        # Center point
        center_x, center_y = width / 2, height / 2

        # Distance to front (right, 0°)
        front_dist = env.ray_cast(center_x, center_y, 0)
        assert abs(front_dist - width / 2) < 0.1, f"Front: expected {width/2}, got {front_dist}"

        # Distance to back (left, 180°)
        back_dist = env.ray_cast(center_x, center_y, 180)
        assert abs(back_dist - width / 2) < 0.1, f"Back: expected {width/2}, got {back_dist}"

        # Distance to right (up, 90°)
        right_dist = env.ray_cast(center_x, center_y, 90)
        assert abs(right_dist - height / 2) < 0.1, f"Right: expected {height/2}, got {right_dist}"

        # Distance to left (down, 270°/-90°)
        left_dist = env.ray_cast(center_x, center_y, 270)
        assert abs(left_dist - height / 2) < 0.1, f"Left: expected {height/2}, got {left_dist}"

    def test_room_edge_distances(self):
        """Test distance measurement from room edge"""
        width, height = 200, 300
        env = ConcreteVirtualEnvironment.create_rectangular_room(width, height)

        # Point slightly inside from left edge to avoid starting on wall
        edge_x, edge_y = 1.0, height / 2

        # Distance to right wall (approximately width - 1)
        right_dist = env.ray_cast(edge_x, edge_y, 0)
        expected = width - edge_x
        assert abs(right_dist - expected) < 0.1, f"Expected {expected}, got {right_dist}"

    def test_room_diagonal_distance(self):
        """Test diagonal distance from corner"""
        width, height = 200, 200
        env = ConcreteVirtualEnvironment.create_rectangular_room(width, height)

        # From near corner diagonally outward
        # Ray at 45° from (10, 10) towards opposite corner
        distance = env.ray_cast(10, 10, 45)
        # Should hit a wall somewhere
        assert distance > 0 and distance < 500


class TestCollisionDetection:
    """Test collision detection"""

    def test_no_collision_in_empty_room(self):
        """Test no collision in empty space"""
        env = ConcreteVirtualEnvironment()
        assert not env.check_collision(50, 50, 10)

    def test_collision_with_wall(self):
        """Test collision detection with wall"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        # Circle at x=95 with radius 10 should collide (center at 95, wall at 100)
        assert env.check_collision(95, 50, 10)

    def test_no_collision_far_from_wall(self):
        """Test no collision when far from wall"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        # Circle at x=70 with radius 10 should not collide
        assert not env.check_collision(70, 50, 10)

    def test_collision_close_to_wall(self):
        """Test collision when approaching wall"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        # Circle at distance 9 with radius 10 should definitely collide
        assert env.check_collision(91, 50, 10)

    def test_no_collision_just_outside(self):
        """Test no collision when just outside"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        # Circle at distance 11 with radius 10 should not collide
        assert not env.check_collision(89, 50, 10)

    def test_collision_with_rectangle(self):
        """Test collision with rectangular obstacle"""
        env = ConcreteVirtualEnvironment()
        # Rectangle centered at (100, 100), 40x60 size
        env.add_rectangle(100, 100, 40, 60)

        # Point near center (but outside the rectangle walls)
        # Should check distance to wall, not inside point
        # At (90, 100): distance to nearest wall is 10 (to left wall at x=80)
        # Collision with radius 15 should return true
        assert env.check_collision(90, 100, 15)

        # Point far away: should not collide
        assert not env.check_collision(150, 100, 5)


class TestClosestObstacle:
    """Test closest obstacle detection"""

    def test_closest_obstacle_distance(self):
        """Test distance to closest obstacle"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        distance, angle = env.get_closest_obstacle(0, 50, 0)
        assert abs(distance - 100) < 0.1

    def test_closest_obstacle_angle(self):
        """Test actual angle calculation"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        # Ray pointing directly at wall
        distance, angle = env.get_closest_obstacle(0, 50, 0)
        # Angle should be 0° (pointing right)
        assert abs(angle - 0) < 1.0

    def test_closest_obstacle_diagonal(self):
        """Test diagonal obstacle"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(50, 50, 100, 100)

        distance, angle = env.get_closest_obstacle(0, 0, 45)
        # At 45°, should hit diagonal wall
        assert distance > 0
        assert angle >= 0  # Angle is valid


class TestEnvironmentBounds:
    """Test environment bounds tracking"""

    def test_initial_bounds(self):
        """Test initial bounds"""
        env = ConcreteVirtualEnvironment()
        bounds = env.get_bounds()
        assert len(bounds) == 4

    def test_bounds_update_with_wall(self):
        """Test bounds update when adding wall"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(0, 0, 500, 500)

        bounds = env.get_bounds()
        # Bounds should encompass the wall
        assert bounds[0] <= 0 and bounds[2] >= 500
        assert bounds[1] <= 0 and bounds[3] >= 500

    def test_multiple_walls_bounds(self):
        """Test bounds with multiple walls"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(0, 0, 100, 0)
        env.add_wall(100, 0, 100, 200)

        bounds = env.get_bounds()
        assert bounds[0] <= 0 and bounds[2] >= 100
        assert bounds[1] <= 0 and bounds[3] >= 200


class TestReset:
    """Test environment reset"""

    def test_reset_clears_walls(self):
        """Test reset clears all walls"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(0, 0, 100, 100)
        env.add_wall(100, 100, 200, 200)

        assert env.get_wall_count() == 2

        env.reset()
        assert env.get_wall_count() == 0

    def test_ray_cast_after_reset(self):
        """Test ray_cast returns max_range after reset"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)
        env.reset()

        distance = env.ray_cast(0, 50, 0, max_range=500)
        assert distance == 500


class TestEdgeCases:
    """Test edge cases and numerical precision"""

    def test_ray_at_zero_length_wall(self):
        """Test ray against zero-length wall (point)"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 50, 100, 50)  # Point, not a wall

        distance = env.ray_cast(0, 50, 0, max_range=500)
        # Should not hit point (infinitesimal target)
        assert distance == 500

    def test_very_close_ray_start(self):
        """Test ray starting very close to wall"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        # Start at x=99.9
        distance = env.ray_cast(99.9, 50, 0)
        assert abs(distance - 0.1) < 0.01

    def test_ray_segment_endpoint_collision(self):
        """Test ray hitting segment endpoint"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        # Ray at exactly the endpoint height
        distance = env.ray_cast(0, 100, 0)
        assert abs(distance - 100) < 0.1

    def test_negative_coordinates(self):
        """Test negative coordinate system"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(-100, -100, -100, 100)

        distance = env.ray_cast(-200, 0, 0)
        assert abs(distance - 100) < 0.1

    def test_room_max_range_limited(self):
        """Test max_range parameter limits detection"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(1000, 0, 1000, 100)

        distance = env.ray_cast(0, 50, 0, max_range=500)
        # Wall is too far away, should return max_range
        assert distance == 500


class TestIntegration:
    """Integration tests combining multiple features"""

    def test_room_with_obstacles(self):
        """Test room with internal obstacles"""
        env = ConcreteVirtualEnvironment.create_rectangular_room(300, 300)
        # Add internal obstacle
        env.add_rectangle(150, 150, 50, 50)

        # Scan around obstacle
        dist_right = env.ray_cast(100, 150, 0)
        assert dist_right < 300  # Should hit obstacle or wall

        # Scan above obstacle
        dist_up = env.ray_cast(150, 100, 90)
        assert dist_up < 300

    def test_complex_wall_pattern(self):
        """Test complex wall pattern"""
        env = ConcreteVirtualEnvironment()
        # Create maze-like pattern
        env.add_wall(0, 100, 100, 100)   # Horizontal wall
        env.add_wall(150, 100, 200, 100)  # Another segment
        env.add_wall(100, 80, 150, 120)   # Diagonal connection

        # Ray should hit one of the walls
        distance = env.ray_cast(0, 100, 0)
        assert distance > 0
        assert distance < 500

    def test_rectangular_room_scanning(self):
        """Test scanning around rectangular room"""
        width, height = 400, 300
        env = ConcreteVirtualEnvironment.create_rectangular_room(width, height)

        center_x, center_y = width / 2, height / 2

        # 360-degree scan should always hit a wall
        angles = list(range(0, 360, 45))
        for angle in angles:
            distance = env.ray_cast(center_x, center_y, angle)
            assert 0 < distance < 500, f"Failed at angle {angle}: distance={distance}"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
