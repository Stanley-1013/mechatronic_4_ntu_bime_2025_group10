"""
Unit and integration tests for visualization module.

Tests trajectory plotter, state timeline, sensor plotter, and manager.
"""

import pytest
from pathlib import Path
import tempfile
import os
from typing import List

from assertions.e2e_assertions import SimulationResult, TrajectoryPoint
from .trajectory_plotter import TrajectoryPlotter
from .state_timeline import StateTimeline
from .sensor_plotter import SensorPlotter
from .visualization_manager import VisualizationManager


@pytest.fixture
def sample_trajectory() -> List[TrajectoryPoint]:
    """Create sample trajectory for testing"""
    points = []
    for i in range(100):
        t = i * 0.01  # 0-1 second duration
        x = 50 + t * 100  # Linear movement
        y = 50 + (t * 100) * 0.5  # Slight diagonal
        yaw = t * 45  # Rotation

        point = TrajectoryPoint(
            x=x,
            y=y,
            yaw=yaw,
            front_distance=max(10, 80 - int(t * 100)),
            right_distance=int(15 + (t * 10) * (1 if (i % 2) == 0 else -1)),
            left_pwm=int(100 + 30 * (t % 0.5)),
            right_pwm=int(120 - 20 * (t % 0.5)),
            timestamp=t,
        )
        points.append(point)

    return points


@pytest.fixture
def sample_result(sample_trajectory) -> SimulationResult:
    """Create sample simulation result"""
    return SimulationResult(
        trajectory=sample_trajectory,
        corner_count=4,
        completion_time=1.0,
        collided=False,
        collision_point=None,
        final_position=(150.0, 100.0, 45.0),
        distance_traveled=141.4,
        total_steps=100,
        blocked_steps=0,
    )


@pytest.fixture
def sample_walls() -> List[tuple]:
    """Create sample walls for testing"""
    return [
        (0, 0, 200, 0),      # Bottom wall
        (200, 0, 200, 200),  # Right wall
        (200, 200, 0, 200),  # Top wall
        (0, 200, 0, 0),      # Left wall
    ]


@pytest.fixture
def temp_output_dir() -> Path:
    """Create temporary output directory"""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield Path(tmpdir)


class TestTrajectoryPlotter:
    """Test trajectory plotter"""

    def test_initialization(self):
        """Test plotter initialization"""
        plotter = TrajectoryPlotter()
        assert plotter.figsize == (14, 10)

    def test_custom_figsize(self):
        """Test custom figure size"""
        plotter = TrajectoryPlotter(figsize=(12, 8))
        assert plotter.figsize == (12, 8)

    def test_plot_trajectory(self, sample_result, sample_walls):
        """Test trajectory plot generation"""
        plotter = TrajectoryPlotter()
        fig = plotter.plot_trajectory(
            result=sample_result,
            walls=sample_walls,
            title="Test Trajectory"
        )

        assert fig is not None
        assert len(fig.axes) > 0
        ax = fig.axes[0]
        assert ax.get_xlabel() == 'X Position (cm)'
        assert ax.get_ylabel() == 'Y Position (cm)'

    def test_plot_trajectory_with_collision(self, sample_trajectory, sample_walls):
        """Test trajectory plot with collision"""
        collision_result = SimulationResult(
            trajectory=sample_trajectory,
            corner_count=2,
            completion_time=0.5,
            collided=True,
            collision_point=(100.0, 50.0),
            final_position=(100.0, 50.0, 0.0),
            distance_traveled=70.7,
            total_steps=50,
            blocked_steps=10,
        )

        plotter = TrajectoryPlotter()
        fig = plotter.plot_trajectory(
            result=collision_result,
            walls=sample_walls,
        )

        assert fig is not None

    def test_save_png(self, sample_result, sample_walls, temp_output_dir):
        """Test saving trajectory as PNG"""
        plotter = TrajectoryPlotter()
        fig = plotter.plot_trajectory(
            result=sample_result,
            walls=sample_walls,
        )

        output_file = temp_output_dir / "test_trajectory"
        result_path = plotter.save(fig, str(output_file), dpi=100)

        assert Path(result_path).exists()
        assert result_path.endswith('.png')

    def test_save_pdf(self, sample_result, sample_walls, temp_output_dir):
        """Test saving trajectory as both PNG and PDF"""
        plotter = TrajectoryPlotter()
        fig = plotter.plot_trajectory(
            result=sample_result,
            walls=sample_walls,
        )

        output_file = temp_output_dir / "test_trajectory"
        result_path = plotter.save(fig, str(output_file), dpi=100, pdf=True)

        assert Path(result_path).exists()
        assert Path(f"{output_file}.pdf").exists()

    def test_empty_trajectory(self):
        """Test handling of empty trajectory"""
        empty_result = SimulationResult(
            trajectory=[],
            corner_count=0,
            completion_time=0,
            collided=False,
        )

        plotter = TrajectoryPlotter()
        fig = plotter.plot_trajectory(
            result=empty_result,
            walls=[],
        )

        assert fig is not None


class TestStateTimeline:
    """Test state timeline visualization"""

    def test_initialization(self):
        """Test timeline initialization"""
        timeline = StateTimeline()
        assert timeline.figsize == (14, 6)

    def test_plot_state_timeline(self, sample_result):
        """Test state timeline generation"""
        timeline = StateTimeline()
        fig = timeline.plot_state_timeline(
            result=sample_result,
            title="Test State Timeline"
        )

        assert fig is not None
        assert len(fig.axes) > 0

    def test_state_inference(self, sample_trajectory):
        """Test state inference from motor commands"""
        timeline = StateTimeline()
        states = timeline._infer_states(sample_trajectory)

        assert len(states) == len(sample_trajectory)
        assert all(isinstance(s, int) for s in states)
        assert all(0 <= s <= 5 for s in states)

    def test_transition_detection(self, sample_result):
        """Test state transition detection"""
        timeline = StateTimeline()
        states = timeline._infer_states(sample_result.trajectory)
        times = [p.timestamp for p in sample_result.trajectory]

        transitions = timeline._find_transitions(states, times)

        assert len(transitions) > 0
        assert transitions[0][1] == -1  # Initial marker

    def test_save_timeline(self, sample_result, temp_output_dir):
        """Test saving state timeline"""
        timeline = StateTimeline()
        fig = timeline.plot_state_timeline(result=sample_result)

        output_file = temp_output_dir / "test_timeline"
        result_path = timeline.save(fig, str(output_file), dpi=100)

        assert Path(result_path).exists()

    def test_empty_trajectory_timeline(self):
        """Test timeline with empty trajectory"""
        empty_result = SimulationResult(
            trajectory=[],
            corner_count=0,
            completion_time=0,
            collided=False,
        )

        timeline = StateTimeline()
        fig = timeline.plot_state_timeline(result=empty_result)

        assert fig is not None


class TestSensorPlotter:
    """Test sensor plotter"""

    def test_initialization(self):
        """Test sensor plotter initialization"""
        plotter = SensorPlotter()
        assert plotter.figsize == (14, 10)

    def test_plot_sensor_timeseries(self, sample_result):
        """Test sensor time-series plot"""
        plotter = SensorPlotter()
        fig = plotter.plot_sensor_timeseries(
            result=sample_result,
            title="Test Sensors"
        )

        assert fig is not None
        assert len(fig.axes) == 3  # distance, motor, right

    def test_sensor_timeseries_without_motor(self, sample_result):
        """Test sensor plot without motor data"""
        plotter = SensorPlotter()
        fig = plotter.plot_sensor_timeseries(
            result=sample_result,
            show_motor_commands=False
        )

        assert fig is not None
        assert len(fig.axes) == 2

    def test_sensor_correlation(self, sample_result):
        """Test sensor correlation plot"""
        plotter = SensorPlotter()
        fig = plotter.plot_sensor_correlation(result=sample_result)

        assert fig is not None

    def test_save_sensor_plot(self, sample_result, temp_output_dir):
        """Test saving sensor plot"""
        plotter = SensorPlotter()
        fig = plotter.plot_sensor_timeseries(result=sample_result)

        output_file = temp_output_dir / "test_sensors"
        result_path = plotter.save(fig, str(output_file), dpi=100)

        assert Path(result_path).exists()

    def test_empty_trajectory_sensors(self):
        """Test sensor plot with empty trajectory"""
        empty_result = SimulationResult(
            trajectory=[],
            corner_count=0,
            completion_time=0,
            collided=False,
        )

        plotter = SensorPlotter()
        fig = plotter.plot_sensor_timeseries(result=empty_result)

        assert fig is not None


class TestVisualizationManager:
    """Test visualization manager"""

    def test_initialization(self, temp_output_dir):
        """Test manager initialization"""
        manager = VisualizationManager(output_dir=temp_output_dir)
        assert manager.output_dir == temp_output_dir
        assert manager.dpi == 300
        assert manager.pdf_output == False

    def test_custom_initialization(self, temp_output_dir):
        """Test manager with custom settings"""
        manager = VisualizationManager(
            output_dir=temp_output_dir,
            dpi=150,
            pdf_output=True
        )
        assert manager.dpi == 150
        assert manager.pdf_output == True

    def test_generate_result_visualizations(
        self,
        sample_result,
        sample_walls,
        temp_output_dir
    ):
        """Test generating all visualizations for single result"""
        manager = VisualizationManager(output_dir=temp_output_dir)

        files = manager.generate_result_visualizations(
            result=sample_result,
            scenario_name="test_scenario",
            walls=sample_walls,
            timestamp="20240101_120000"
        )

        assert 'trajectory' in files
        assert 'state_timeline' in files
        assert 'sensors' in files
        assert 'correlation' in files

        for filepath in files.values():
            assert Path(filepath).exists()

    def test_generate_batch_visualizations(
        self,
        sample_result,
        sample_walls,
        temp_output_dir
    ):
        """Test generating visualizations for multiple results"""
        manager = VisualizationManager(output_dir=temp_output_dir)

        results = [sample_result, sample_result]
        scenarios = ["scenario1", "scenario2"]
        walls_list = [sample_walls, sample_walls]

        files = manager.generate_batch_visualizations(
            results=results,
            scenario_names=scenarios,
            walls_list=walls_list,
            timestamp="20240101_120000"
        )

        assert len(files['trajectory']) == 2
        assert len(files['state_timeline']) == 2
        assert len(files['sensors']) == 2

        for filepath_list in files.values():
            for filepath in filepath_list:
                assert Path(filepath).exists()

    def test_summary_report_generation(
        self,
        sample_result,
        temp_output_dir
    ):
        """Test HTML summary report generation"""
        manager = VisualizationManager(output_dir=temp_output_dir)

        results = [sample_result, sample_result]
        scenarios = ["scenario1", "scenario2"]

        report_path = manager.generate_summary_report(
            results=results,
            scenario_names=scenarios,
            timestamp="20240101_120000"
        )

        assert report_path.exists()
        assert report_path.suffix == '.html'

        # Verify content
        with open(report_path, 'r') as f:
            content = f.read()
            assert 'E2E Test Visualization Report' in content
            assert 'scenario1' in content
            assert 'scenario2' in content

    def test_set_output_dir(self, temp_output_dir):
        """Test changing output directory"""
        manager = VisualizationManager(output_dir=temp_output_dir)

        new_dir = temp_output_dir / "new_output"
        manager.set_output_dir(new_dir)

        assert manager.output_dir == new_dir
        assert new_dir.exists()


class TestVisualizationIntegration:
    """Integration tests for complete visualization pipeline"""

    def test_complete_pipeline(
        self,
        sample_result,
        sample_walls,
        temp_output_dir
    ):
        """Test complete visualization pipeline"""
        manager = VisualizationManager(output_dir=temp_output_dir, dpi=100)

        # Generate single result visualizations
        files = manager.generate_result_visualizations(
            result=sample_result,
            scenario_name="integration_test",
            walls=sample_walls,
        )

        # Verify all outputs exist
        assert all(Path(f).exists() for f in files.values())

        # Generate summary report
        report_path = manager.generate_summary_report(
            results=[sample_result],
            scenario_names=["integration_test"],
        )

        assert report_path.exists()

    def test_visualization_with_collisions(
        self,
        sample_trajectory,
        sample_walls,
        temp_output_dir
    ):
        """Test visualization with collision scenario"""
        collision_result = SimulationResult(
            trajectory=sample_trajectory,
            corner_count=2,
            completion_time=0.5,
            collided=True,
            collision_point=(100.0, 50.0),
            final_position=(100.0, 50.0, 0.0),
            distance_traveled=70.7,
            total_steps=50,
            blocked_steps=10,
        )

        manager = VisualizationManager(output_dir=temp_output_dir, dpi=100)

        files = manager.generate_result_visualizations(
            result=collision_result,
            scenario_name="collision_test",
            walls=sample_walls,
        )

        assert all(Path(f).exists() for f in files.values())

    @pytest.mark.parametrize("dpi", [100, 150, 300])
    def test_different_dpi_settings(
        self,
        sample_result,
        sample_walls,
        temp_output_dir,
        dpi
    ):
        """Test visualization with different DPI settings"""
        manager = VisualizationManager(output_dir=temp_output_dir, dpi=dpi)

        files = manager.generate_result_visualizations(
            result=sample_result,
            scenario_name=f"dpi_{dpi}",
            walls=sample_walls,
        )

        assert all(Path(f).exists() for f in files.values())


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
