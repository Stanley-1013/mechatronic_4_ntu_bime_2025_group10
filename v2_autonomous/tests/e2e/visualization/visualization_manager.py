"""
Visualization Manager - Orchestrates test result visualization.

Provides high-level API for generating all visualization outputs:
- Trajectory plots
- State timelines
- Sensor time-series
- Batch processing of multiple results
"""

import os
from pathlib import Path
from typing import List, Optional, Tuple, Dict, Any
from datetime import datetime
import logging

from assertions.e2e_assertions import SimulationResult
from .trajectory_plotter import TrajectoryPlotter
from .state_timeline import StateTimeline
from .sensor_plotter import SensorPlotter


logger = logging.getLogger(__name__)


class VisualizationManager:
    """Manage test result visualizations"""

    def __init__(
        self,
        output_dir: Optional[Path] = None,
        dpi: int = 300,
        pdf_output: bool = False,
    ):
        """
        Initialize visualization manager.

        Args:
            output_dir: Directory for saving visualizations
                       (defaults to ./test_visualizations)
            dpi: Dots per inch for PNG output
            pdf_output: Whether to also generate PDF versions
        """
        self.output_dir = output_dir or Path('./test_visualizations')
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.dpi = dpi
        self.pdf_output = pdf_output

        # Initialize plotter instances
        self.trajectory_plotter = TrajectoryPlotter()
        self.state_timeline = StateTimeline()
        self.sensor_plotter = SensorPlotter()

        logger.info(f"VisualizationManager initialized, output dir: {self.output_dir}")

    def generate_result_visualizations(
        self,
        result: SimulationResult,
        scenario_name: str,
        walls: List[Tuple[float, float, float, float]],
        scenario_bounds: Optional[Tuple[float, float, float, float]] = None,
        timestamp: Optional[str] = None,
    ) -> Dict[str, str]:
        """
        Generate all visualizations for a single test result.

        Args:
            result: SimulationResult to visualize
            scenario_name: Name of the test scenario
            walls: Environment walls
            scenario_bounds: Optional bounds for trajectory plot
            timestamp: Optional timestamp for filenames (defaults to current time)

        Returns:
            Dictionary mapping visualization types to file paths
        """
        if timestamp is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        output_files = {}

        try:
            # Trajectory plot
            traj_fig = self.trajectory_plotter.plot_trajectory(
                result=result,
                walls=walls,
                title=f"{scenario_name} - Robot Trajectory",
                scenario_bounds=scenario_bounds,
            )
            traj_path = self.output_dir / f"{scenario_name}_trajectory_{timestamp}"
            traj_file = self.trajectory_plotter.save(
                traj_fig, str(traj_path), dpi=self.dpi, pdf=self.pdf_output
            )
            output_files['trajectory'] = traj_file
            logger.info(f"Saved trajectory plot: {traj_file}")

            # State timeline
            state_fig = self.state_timeline.plot_state_timeline(
                result=result,
                title=f"{scenario_name} - State Timeline",
            )
            state_path = self.output_dir / f"{scenario_name}_state_timeline_{timestamp}"
            state_file = self.state_timeline.save(
                state_fig, str(state_path), dpi=self.dpi, pdf=self.pdf_output
            )
            output_files['state_timeline'] = state_file
            logger.info(f"Saved state timeline: {state_file}")

            # Sensor plots
            sensor_fig = self.sensor_plotter.plot_sensor_timeseries(
                result=result,
                title=f"{scenario_name} - Sensor Readings",
                show_thresholds=True,
                show_motor_commands=True,
            )
            sensor_path = self.output_dir / f"{scenario_name}_sensors_{timestamp}"
            sensor_file = self.sensor_plotter.save(
                sensor_fig, str(sensor_path), dpi=self.dpi, pdf=self.pdf_output
            )
            output_files['sensors'] = sensor_file
            logger.info(f"Saved sensor plot: {sensor_file}")

            # Sensor correlation
            corr_fig = self.sensor_plotter.plot_sensor_correlation(
                result=result,
                title=f"{scenario_name} - Distance Correlation",
            )
            corr_path = self.output_dir / f"{scenario_name}_correlation_{timestamp}"
            corr_file = self.sensor_plotter.save(
                corr_fig, str(corr_path), dpi=self.dpi, pdf=self.pdf_output
            )
            output_files['correlation'] = corr_file
            logger.info(f"Saved correlation plot: {corr_file}")

        except Exception as e:
            logger.error(f"Error generating visualizations: {e}", exc_info=True)
            raise

        return output_files

    def generate_batch_visualizations(
        self,
        results: List[SimulationResult],
        scenario_names: List[str],
        walls_list: List[List[Tuple[float, float, float, float]]],
        scenario_bounds: Optional[List[Tuple[float, float, float, float]]] = None,
        timestamp: Optional[str] = None,
    ) -> Dict[str, List[str]]:
        """
        Generate visualizations for multiple test results.

        Args:
            results: List of SimulationResult objects
            scenario_names: List of scenario names (one per result)
            walls_list: List of wall lists (one per result)
            scenario_bounds: Optional list of bounds (one per result)
            timestamp: Optional timestamp for filenames

        Returns:
            Dictionary mapping visualization types to lists of file paths
        """
        if timestamp is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        all_files = {
            'trajectory': [],
            'state_timeline': [],
            'sensors': [],
            'correlation': [],
        }

        for i, (result, scenario, walls) in enumerate(zip(results, scenario_names, walls_list)):
            bounds = scenario_bounds[i] if scenario_bounds else None

            try:
                files = self.generate_result_visualizations(
                    result=result,
                    scenario_name=scenario,
                    walls=walls,
                    scenario_bounds=bounds,
                    timestamp=f"{timestamp}_{i:03d}",
                )

                for viz_type, filepath in files.items():
                    all_files[viz_type].append(filepath)

            except Exception as e:
                logger.error(f"Error processing scenario {scenario}: {e}")

        logger.info(f"Batch visualization complete: {len(results)} results processed")
        return all_files

    def generate_summary_report(
        self,
        results: List[SimulationResult],
        scenario_names: List[str],
        timestamp: Optional[str] = None,
    ) -> Path:
        """
        Generate HTML summary report with embedded visualizations.

        Args:
            results: List of simulation results
            scenario_names: List of scenario names
            timestamp: Optional timestamp for report

        Returns:
            Path to generated HTML report
        """
        if timestamp is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        report_path = self.output_dir / f"summary_report_{timestamp}.html"

        # Generate HTML content
        html_content = self._generate_html_report(results, scenario_names, timestamp)

        with open(report_path, 'w') as f:
            f.write(html_content)

        logger.info(f"Generated summary report: {report_path}")
        return report_path

    def _generate_html_report(
        self,
        results: List[SimulationResult],
        scenario_names: List[str],
        timestamp: str,
    ) -> str:
        """Generate HTML report content"""
        html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>E2E Test Visualization Report</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }}
        h1, h2, h3 {{
            color: #333;
        }}
        .summary {{
            background-color: #fff;
            padding: 15px;
            border-radius: 5px;
            margin-bottom: 20px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        .result {{
            background-color: #fff;
            padding: 15px;
            margin-bottom: 30px;
            border-left: 5px solid #4CAF50;
            border-radius: 5px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        .result.failed {{
            border-left-color: #f44336;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin: 15px 0;
        }}
        th, td {{
            padding: 10px;
            text-align: left;
            border-bottom: 1px solid #ddd;
        }}
        th {{
            background-color: #4CAF50;
            color: white;
        }}
        .stat {{
            display: inline-block;
            margin-right: 30px;
            font-family: monospace;
        }}
        .stat-label {{
            font-weight: bold;
        }}
        .stat-value {{
            color: #0066cc;
        }}
    </style>
</head>
<body>
    <h1>E2E Test Visualization Report</h1>
    <div class="summary">
        <h2>Summary</h2>
        <p><strong>Generated:</strong> {timestamp}</p>
        <p><strong>Total Results:</strong> {len(results)}</p>
        <table>
            <tr>
                <th>Metric</th>
                <th>Value</th>
            </tr>
"""

        # Add summary statistics
        total_distance = sum(r.distance_traveled for r in results)
        total_time = sum(r.completion_time for r in results)
        total_corners = sum(r.corner_count for r in results)
        collision_count = sum(1 for r in results if r.collided)

        html += f"""
            <tr>
                <td>Total Distance</td>
                <td>{total_distance:.1f} cm</td>
            </tr>
            <tr>
                <td>Total Time</td>
                <td>{total_time:.2f} s</td>
            </tr>
            <tr>
                <td>Total Corners</td>
                <td>{total_corners}</td>
            </tr>
            <tr>
                <td>Collisions</td>
                <td>{collision_count}</td>
            </tr>
        </table>
        </div>
"""

        # Add per-scenario results
        html += "<h2>Detailed Results</h2>\n"
        for i, (result, scenario_name) in enumerate(zip(results, scenario_names)):
            status_class = "failed" if result.collided else ""
            status_text = "COLLISION" if result.collided else "OK"

            html += f"""
    <div class="result {status_class}">
        <h3>{scenario_name}</h3>
        <div class="stat">
            <span class="stat-label">Distance:</span>
            <span class="stat-value">{result.distance_traveled:.1f} cm</span>
        </div>
        <div class="stat">
            <span class="stat-label">Time:</span>
            <span class="stat-value">{result.completion_time:.2f} s</span>
        </div>
        <div class="stat">
            <span class="stat-label">Corners:</span>
            <span class="stat-value">{result.corner_count}</span>
        </div>
        <div class="stat">
            <span class="stat-label">Status:</span>
            <span class="stat-value">{status_text}</span>
        </div>
    </div>
"""

        html += """
</body>
</html>
"""
        return html

    def set_output_dir(self, output_dir: Path):
        """Change output directory"""
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        logger.info(f"Output directory changed to: {self.output_dir}")
