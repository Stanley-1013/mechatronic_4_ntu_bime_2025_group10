"""
Collision Detection System Tests

Tests for collision detection and boundary protection:
1. Circular bounding box with configurable radius
2. Circle vs line segment collision algorithm
3. Collision point detection
4. Collision handling strategies
5. Physics simulator integration
6. Realistic collision scenarios (corners, walls, high-speed)
"""

import pytest
import math
from .concrete_virtual_environment import ConcreteVirtualEnvironment
from .concrete_physics_simulator import ConcretePhysicsSimulator


class TestBoundingBoxDefinition:
    """Test circular bounding box"""

    def test_robot_radius_exists(self):
        """Verify PhysicsSimulator has robot_radius"""
        sim = ConcretePhysicsSimulator()
        assert hasattr(sim, 'robot_radius')
        assert 10 <= sim.robot_radius <= 15

    def test_radius_affects_collision(self):
        """Verify radius changes collision behavior"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        # Position at distance 5 from wall
        assert not env.check_collision(95, 50, 4)   # radius 4: 5 > 4, no collision
        assert env.check_collision(95, 50, 6)       # radius 6: 5 < 6, collision


class TestCircleVsLineSegment:
    """Test circle vs line segment collision algorithm"""

    def test_perpendicular_approach(self):
        """Test head-on collision"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)
        assert env.check_collision(95, 50, 5.1)

    def test_parallel_approach(self):
        """Test collision approaching wall parallel"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)
        assert env.check_collision(95, 25, 10)

    def test_corner_detection(self):
        """Test collision at corner"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)
        env.add_wall(0, 100, 100, 100)
        assert env.check_collision(90, 90, 15)

    def test_no_collision_far_away(self):
        """Test no collision in open space"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(1000, 0, 1000, 100)
        assert not env.check_collision(0, 0, 50)


class TestCollisionPointDetection:
    """Test finding closest point on walls"""

    def test_distance_to_vertical_wall(self):
        """Test distance calculation to vertical wall"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)
        distance = env._point_to_segment_distance(95, 50, 100, 0, 100, 100)
        assert abs(distance - 5.0) < 0.1

    def test_distance_to_diagonal(self):
        """Test distance to diagonal wall"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(0, 0, 100, 100)
        distance = env._point_to_segment_distance(50, 0, 0, 0, 100, 100)
        expected = math.sqrt((50 - 25)**2 + (0 - 25)**2)
        assert abs(distance - expected) < 0.1

    def test_endpoint_snapping(self):
        """Test distance when closest point is endpoint"""
        env = ConcreteVirtualEnvironment()
        distance = env._point_to_segment_distance(150, 150, 100, 0, 100, 100)
        expected = math.sqrt((150 - 100)**2 + (150 - 100)**2)
        assert abs(distance - expected) < 0.1


class TestCollisionBehavior:
    """Test collision detection behavior"""

    def test_returns_boolean(self):
        """Test collision returns True/False"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)
        result = env.check_collision(95, 50, 10)
        assert isinstance(result, bool)
        assert result is True

    def test_room_edges_collision(self):
        """Test collision at all room edges"""
        env = ConcreteVirtualEnvironment.create_rectangular_room(300, 200)
        assert env.check_collision(5, 100, 6)    # Left edge
        assert env.check_collision(295, 100, 6)  # Right edge
        assert env.check_collision(150, 5, 6)    # Bottom edge
        assert env.check_collision(150, 195, 6)  # Top edge

    def test_room_center_safe(self):
        """Test no collision in room center"""
        env = ConcreteVirtualEnvironment.create_rectangular_room(400, 300)
        assert not env.check_collision(200, 150, 50)


class TestPhysicsSimulatorIntegration:
    """Test collision detection with physics simulator"""

    def test_position_collision_check(self):
        """Test using physics position for collision"""
        env = ConcreteVirtualEnvironment.create_rectangular_room(300, 300)
        sim = ConcretePhysicsSimulator()
        sim.set_wheel_variance(0.0)
        sim.slip_probability = 0.0
        sim.reset_position(150, 150, 0)

        # Move and check
        for _ in range(50):
            sim.update(0.02, 100, 100)

        x, y, yaw = sim.get_position()
        collision = env.check_collision(x, y, sim.robot_radius)
        assert not collision

    def test_fast_motion_collision(self):
        """Test collision detection with fast motion"""
        env = ConcreteVirtualEnvironment.create_rectangular_room(200, 200)
        sim = ConcretePhysicsSimulator()
        sim.reset_position(10, 100, 0)

        for _ in range(100):
            sim.update(0.02, -200, -200)
            x, y, yaw = sim.get_position()
            if env.check_collision(x, y, sim.robot_radius):
                break


class TestCollisionScenarios:
    """Test realistic collision scenarios"""

    def test_wall_following(self):
        """Test parallel wall approach"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        # Test distance-based collision
        distances = [20, 15, 12, 11, 10, 9, 5]
        for dist in distances:
            x = 100 - dist
            collision = env.check_collision(x, 50, 8)
            expected = dist < 8
            assert collision == expected

    def test_corner_approach(self):
        """Test approaching corner"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)
        env.add_wall(0, 100, 100, 100)

        assert env.check_collision(95, 105, 10)  # Close to corner
        assert not env.check_collision(80, 120, 10)  # Far from corner

    def test_l_shaped_walls(self):
        """Test L-shaped wall configuration"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 100, 200, 100)  # Horizontal
        env.add_wall(200, 100, 200, 200)  # Vertical

        assert env.check_collision(190, 95, 10)
        assert not env.check_collision(180, 120, 10)


class TestEdgeCases:
    """Test edge cases and precision"""

    def test_floating_point_boundary(self):
        """Test floating point precision at boundary"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100.0, 0.0, 100.0, 100.0)

        assert env.check_collision(90.00001, 50.0, 9.99999)
        assert not env.check_collision(90.0, 50.0, 10.0)

    def test_very_small_radius(self):
        """Test very small robot radius"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        assert env.check_collision(99.9, 50, 0.1)
        assert not env.check_collision(99.5, 50, 0.1)

    def test_very_large_radius(self):
        """Test very large robot radius"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        assert env.check_collision(0, 50, 200)
        assert env.check_collision(-100, 50, 250)

    def test_negative_coordinates(self):
        """Test collision with negative coordinates"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(-100, -100, -100, 100)

        assert env.check_collision(-95, 0, 5.1)
        assert not env.check_collision(-80, 0, 10)

    def test_consistency(self):
        """Test consistent collision results"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        result1 = env.check_collision(95, 50, 10)
        result2 = env.check_collision(95, 50, 10)
        result3 = env.check_collision(95, 50, 10)

        assert result1 == result2 == result3


class TestRaycastCollisionRelationship:
    """Test raycast and collision detection relationship"""

    def test_raycast_vs_collision(self):
        """Test raycast distance relates to collision"""
        env = ConcreteVirtualEnvironment()
        env.add_wall(100, 0, 100, 100)

        x, y = 50, 50
        distance = env.ray_cast(x, y, 0)
        assert distance == pytest.approx(50, abs=0.1)

        # Collision should trigger near raycast distance
        assert env.check_collision(x + distance - 1, y, 1.1)

    def test_raycast_predicts_collision(self):
        """Test using raycast for collision prediction"""
        env = ConcreteVirtualEnvironment.create_rectangular_room(300, 300)

        x, y = 150, 150
        distance = env.ray_cast(x, y, 0)

        collision_threshold = 50
        if distance > collision_threshold:
            assert not env.check_collision(x, y, collision_threshold)
        else:
            assert env.check_collision(x, y, collision_threshold)


class TestRectangularRoomCollisions:
    """Test collision in rectangular rooms"""

    def test_all_four_walls(self):
        """Test collision with all four walls"""
        env = ConcreteVirtualEnvironment.create_rectangular_room(300, 200)

        assert env.check_collision(5, 100, 6)    # Left
        assert env.check_collision(295, 100, 6)  # Right
        assert env.check_collision(150, 5, 6)    # Bottom
        assert env.check_collision(150, 195, 6)  # Top
        assert not env.check_collision(150, 100, 50)  # Center

    def test_all_four_corners(self):
        """Test collision at all corners"""
        env = ConcreteVirtualEnvironment.create_rectangular_room(200, 150)

        assert env.check_collision(10, 10, 15)      # Bottom-left
        assert env.check_collision(190, 10, 15)     # Bottom-right
        assert env.check_collision(10, 140, 15)     # Top-left
        assert env.check_collision(190, 140, 15)    # Top-right

    def test_center_is_safe(self):
        """Test center point is collision-free"""
        env = ConcreteVirtualEnvironment.create_rectangular_room(400, 300)
        assert not env.check_collision(200, 150, 50)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
