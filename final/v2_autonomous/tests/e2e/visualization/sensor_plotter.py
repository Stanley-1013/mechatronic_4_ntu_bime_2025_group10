"""
Sensor Plotter - Visualizes sensor readings over time.

Generates time-series plots showing:
- Front distance sensor readings
- Right distance sensor readings
- Threshold reference lines (15cm, 40cm)
- Correlation with state changes
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from typing import List, Tuple, Optional, Dict
from datetime import datetime

from assertions.e2e_assertions import SimulationResult, TrajectoryPoint


class SensorPlotter:
    """Visualize sensor data time series"""

    # Thresholds for wall follower algorithm
    FRONT_STOP_THRESHOLD = 15  # cm
    FRONT_SLOW_THRESHOLD = 40  # cm
    RIGHT_TARGET_DISTANCE = 15  # cm

    STATE_COLORS = {
        0: '#808080',  # IDLE - gray
        1: '#4169E1',  # FIND_WALL - royal blue
        2: '#228B22',  # FORWARD - forest green
        3: '#FF8C00',  # BACKUP - dark orange
        4: '#DC143C',  # TURN_LEFT - crimson red
        5: '#9932CC',  # DONE - dark orchid
    }

    def __init__(self, figsize: Tuple[int, int] = (14, 10)):
        """
        Initialize sensor plotter.

        Args:
            figsize: Figure size as (width, height) in inches
        """
        self.figsize = figsize

    def plot_sensor_timeseries(
        self,
        result: SimulationResult,
        title: str = "Sensor Readings Over Time",
        show_thresholds: bool = True,
        show_motor_commands: bool = True,
    ) -> plt.Figure:
        """
        Plot sensor data time series with multiple subplots.

        Args:
            result: SimulationResult with trajectory
            title: Plot title
            show_thresholds: Show reference threshold lines
            show_motor_commands: Show motor PWM commands on separate subplot

        Returns:
            matplotlib Figure object
        """
        if not result.trajectory:
            fig, ax = plt.subplots(figsize=self.figsize)
            ax.text(
                0.5, 0.5,
                'No trajectory data',
                ha='center', va='center',
                transform=ax.transAxes,
                fontsize=14
            )
            return fig

        # Create subplots
        if show_motor_commands:
            fig, axes = plt.subplots(3, 1, figsize=self.figsize, sharex=True)
            ax_dist, ax_motor, ax_right = axes
        else:
            fig, axes = plt.subplots(2, 1, figsize=self.figsize, sharex=True)
            ax_dist, ax_right = axes

        # Extract data
        times = np.array([p.timestamp for p in result.trajectory])
        front_dists = np.array([p.front_distance for p in result.trajectory])
        right_dists = np.array([p.right_distance for p in result.trajectory])
        left_pwms = np.array([p.left_pwm for p in result.trajectory])
        right_pwms = np.array([p.right_pwm for p in result.trajectory])

        # Plot 1: Front Distance
        ax_dist.plot(
            times, front_dists,
            'b-', linewidth=2, label='Front Distance',
            alpha=0.8
        )

        if show_thresholds:
            ax_dist.axhline(
                y=self.FRONT_STOP_THRESHOLD,
                color='red', linestyle='--', linewidth=2,
                label=f'Stop Threshold ({self.FRONT_STOP_THRESHOLD} cm)',
                alpha=0.7
            )
            ax_dist.axhline(
                y=self.FRONT_SLOW_THRESHOLD,
                color='orange', linestyle='--', linewidth=2,
                label=f'Slow Threshold ({self.FRONT_SLOW_THRESHOLD} cm)',
                alpha=0.7
            )

        ax_dist.fill_between(times, 0, front_dists, alpha=0.2, color='blue')
        ax_dist.set_ylabel('Distance (cm)', fontsize=11, fontweight='bold')
        ax_dist.set_title(title, fontsize=13, fontweight='bold')
        ax_dist.legend(loc='upper right', fontsize=10)
        ax_dist.grid(True, alpha=0.3, linestyle='--')
        ax_dist.set_ylim(0, max(front_dists.max(), self.FRONT_SLOW_THRESHOLD + 20))

        # Plot 2: Motor Commands (if enabled)
        if show_motor_commands:
            ax_motor.plot(
                times, left_pwms,
                'g-', linewidth=1.5, label='Left Motor', alpha=0.8
            )
            ax_motor.plot(
                times, right_pwms,
                'r-', linewidth=1.5, label='Right Motor', alpha=0.8
            )
            ax_motor.axhline(y=0, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
            ax_motor.fill_between(times, 0, left_pwms, alpha=0.1, color='green')
            ax_motor.fill_between(times, 0, right_pwms, alpha=0.1, color='red')
            ax_motor.set_ylabel('PWM Value', fontsize=11, fontweight='bold')
            ax_motor.legend(loc='upper right', fontsize=10)
            ax_motor.grid(True, alpha=0.3, linestyle='--')
            ax_motor.set_ylim(-255, 255)

        # Plot 3: Right Distance
        ax_right.plot(
            times, right_dists,
            'g-', linewidth=2, label='Right Distance',
            alpha=0.8
        )

        if show_thresholds:
            ax_right.axhline(
                y=self.RIGHT_TARGET_DISTANCE,
                color='darkgreen', linestyle='--', linewidth=2,
                label=f'Target Distance ({self.RIGHT_TARGET_DISTANCE} cm)',
                alpha=0.7
            )

        ax_right.fill_between(times, 0, right_dists, alpha=0.2, color='green')
        ax_right.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
        ax_right.set_ylabel('Distance (cm)', fontsize=11, fontweight='bold')
        ax_right.legend(loc='upper right', fontsize=10)
        ax_right.grid(True, alpha=0.3, linestyle='--')
        ax_right.set_ylim(0, max(right_dists.max(), self.RIGHT_TARGET_DISTANCE + 20))

        # Set X-axis limits
        if len(times) > 0:
            for ax in axes:
                ax.set_xlim(times[0], times[-1])

        # Add overall statistics text
        stats_text = (
            f"Front: {front_dists.min():.1f}~{front_dists.max():.1f} cm "
            f"(mean: {front_dists.mean():.1f})\n"
            f"Right: {right_dists.min():.1f}~{right_dists.max():.1f} cm "
            f"(mean: {right_dists.mean():.1f})"
        )
        fig.text(
            0.99, 0.01, stats_text,
            ha='right', va='bottom', fontsize=9,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
            family='monospace'
        )

        fig.tight_layout()
        return fig

    def plot_sensor_correlation(
        self,
        result: SimulationResult,
        title: str = "Front vs Right Distance Correlation",
    ) -> plt.Figure:
        """
        Plot scatter plot of front vs right distance.

        Args:
            result: SimulationResult with trajectory
            title: Plot title

        Returns:
            matplotlib Figure object
        """
        fig, ax = plt.subplots(figsize=(10, 8))

        if not result.trajectory:
            ax.text(
                0.5, 0.5,
                'No trajectory data',
                ha='center', va='center',
                transform=ax.transAxes,
                fontsize=14
            )
            return fig

        # Extract data
        front_dists = np.array([p.front_distance for p in result.trajectory])
        right_dists = np.array([p.right_distance for p in result.trajectory])
        times = np.array([p.timestamp for p in result.trajectory])

        # Create scatter plot with time gradient
        scatter = ax.scatter(
            front_dists, right_dists,
            c=times, cmap='viridis', s=50,
            alpha=0.6, edgecolors='black', linewidth=0.5
        )

        # Add threshold lines
        ax.axvline(
            x=self.FRONT_STOP_THRESHOLD,
            color='red', linestyle='--', linewidth=2,
            label=f'Front Stop ({self.FRONT_STOP_THRESHOLD} cm)',
            alpha=0.7
        )
        ax.axhline(
            y=self.RIGHT_TARGET_DISTANCE,
            color='green', linestyle='--', linewidth=2,
            label=f'Right Target ({self.RIGHT_TARGET_DISTANCE} cm)',
            alpha=0.7
        )

        # Labels and title
        ax.set_xlabel('Front Distance (cm)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Right Distance (cm)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(loc='upper right', fontsize=10)

        # Colorbar for time
        cbar = fig.colorbar(scatter, ax=ax, label='Time (s)')

        fig.tight_layout()
        return fig

    def save(
        self,
        fig: plt.Figure,
        filename: str,
        dpi: int = 300,
        pdf: bool = False
    ) -> str:
        """
        Save figure to file.

        Args:
            fig: matplotlib Figure object
            filename: Output filename (without extension)
            dpi: Dots per inch
            pdf: Also save as PDF

        Returns:
            Path to saved file
        """
        png_path = f"{filename}.png"
        fig.savefig(png_path, dpi=dpi, bbox_inches='tight', facecolor='white')

        if pdf:
            pdf_path = f"{filename}.pdf"
            fig.savefig(pdf_path, bbox_inches='tight', facecolor='white')

        plt.close(fig)

        return png_path
