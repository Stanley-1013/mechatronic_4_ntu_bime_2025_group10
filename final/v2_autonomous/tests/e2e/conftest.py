"""
Pytest Configuration for End-to-End Tests

Provides pytest fixtures and configuration for running wall-follower
robot simulations and tests.
"""

import pytest
import sys
from pathlib import Path


# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


class SimulationConfig:
    """Configuration for simulation environment"""

    # Physics parameters
    ROBOT_RADIUS = 12.0  # cm (approximate robot size)
    WHEEL_BASE = 15.0  # cm (distance between wheels)
    MAX_LINEAR_SPEED = 60.0  # cm/s at max PWM
    MAX_ANGULAR_SPEED = 180.0  # deg/s at max PWM

    # Sensor parameters
    SENSOR_UPDATE_FREQ = 50  # Hz
    SENSOR_DELAY_MS = 50  # Default sensor delay
    SENSOR_NOISE_CM = 1.0  # Standard deviation of distance noise
    SENSOR_MAX_RANGE = 500  # cm

    # Simulation parameters
    DEFAULT_TIME_STEP = 0.02  # seconds (50 Hz update rate)
    SIMULATION_MAX_TIME = 60.0  # Maximum simulation time in seconds
    WORLD_BOUNDS = (-500.0, -500.0, 2000.0, 2000.0)  # (min_x, min_y, max_x, max_y)

    # Controller parameters
    TARGET_RIGHT_DISTANCE = 15  # cm
    FRONT_STOP_DISTANCE = 15  # cm
    FRONT_SLOW_DISTANCE = 40  # cm


@pytest.fixture
def simulation_config():
    """Provide default simulation configuration"""
    return SimulationConfig()


@pytest.fixture
def world_bounds():
    """Provide world boundary coordinates"""
    return SimulationConfig.WORLD_BOUNDS


@pytest.fixture
def robot_params():
    """Provide robot physical parameters"""
    return {
        'radius': SimulationConfig.ROBOT_RADIUS,
        'wheel_base': SimulationConfig.WHEEL_BASE,
        'max_linear_speed': SimulationConfig.MAX_LINEAR_SPEED,
        'max_angular_speed': SimulationConfig.MAX_ANGULAR_SPEED,
    }


@pytest.fixture
def sensor_params():
    """Provide sensor parameters"""
    return {
        'update_freq': SimulationConfig.SENSOR_UPDATE_FREQ,
        'delay_ms': SimulationConfig.SENSOR_DELAY_MS,
        'noise_cm': SimulationConfig.SENSOR_NOISE_CM,
        'max_range': SimulationConfig.SENSOR_MAX_RANGE,
    }


@pytest.fixture
def test_output_dir(tmp_path):
    """Provide temporary directory for test outputs"""
    output_dir = tmp_path / "test_results"
    output_dir.mkdir(exist_ok=True)
    return output_dir


@pytest.fixture
def e2e_report_dir():
    """Provide directory for E2E test reports"""
    report_dir = Path(__file__).parent / "test_results"
    report_dir.mkdir(exist_ok=True)
    return report_dir


def pytest_configure(config):
    """Configure pytest with custom markers"""
    config.addinivalue_line(
        "markers", "simulation: mark test as a simulation test"
    )
    config.addinivalue_line(
        "markers", "physics: mark test as testing physics simulation"
    )
    config.addinivalue_line(
        "markers", "controller: mark test as testing wall-follower controller"
    )
    config.addinivalue_line(
        "markers", "slow: mark test as slow running"
    )
    config.addinivalue_line(
        "markers", "smoke: mark test as quick smoke test"
    )
    config.addinivalue_line(
        "markers", "delay: mark test as testing sensor delays"
    )
    config.addinivalue_line(
        "markers", "variance: mark test as testing wheel variance"
    )
    config.addinivalue_line(
        "markers", "blocking: mark test as testing control loop blocking"
    )
    config.addinivalue_line(
        "markers", "stress: mark test as stress testing"
    )


# Pytest collection hooks
def pytest_collection_modifyitems(config, items):
    """Add default markers to test collection"""
    for item in items:
        # Auto-mark tests based on module/class name
        if "test_physics" in item.nodeid:
            item.add_marker(pytest.mark.physics)
        if "test_controller" in item.nodeid:
            item.add_marker(pytest.mark.controller)
        if "test_simulation" in item.nodeid:
            item.add_marker(pytest.mark.simulation)
        if "rectangular_room" in item.nodeid:
            item.add_marker(pytest.mark.simulation)
        if "l_corridor" in item.nodeid:
            item.add_marker(pytest.mark.simulation)
        if "sensor_delay" in item.nodeid:
            item.add_marker(pytest.mark.delay)
        if "wheel_variance" in item.nodeid:
            item.add_marker(pytest.mark.variance)
        if "loop_blocking" in item.nodeid:
            item.add_marker(pytest.mark.blocking)
        if "stress" in item.nodeid:
            item.add_marker(pytest.mark.stress)
        if "quick" in item.nodeid:
            item.add_marker(pytest.mark.smoke)
