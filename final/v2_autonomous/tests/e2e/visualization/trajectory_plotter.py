"""
Trajectory Plotter - Visualizes robot movement paths with state information.

Generates 2D trajectory plots showing:
- Robot movement with state-based coloring
- Environment walls
- Start and end positions
- Collision points
- Corner detection events
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
import numpy as np
from typing import List, Optional, Tuple, Dict, Any
from datetime import datetime

from assertions.e2e_assertions import SimulationResult, TrajectoryPoint


class TrajectoryPlotter:
    """Plot robot trajectories with state information"""

    # State colors and names
    STATE_NAMES = {
        0: 'IDLE',
        1: 'FIND_WALL',
        2: 'FORWARD',
        3: 'BACKUP',
        4: 'TURN_LEFT',
        5: 'DONE',
    }

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
        Initialize trajectory plotter.

        Args:
            figsize: Figure size as (width, height) in inches
        """
        self.figsize = figsize

    def plot_trajectory(
        self,
        result: SimulationResult,
        walls: List[Tuple[float, float, float, float]],
        title: str = "Robot Trajectory",
        show_legend: bool = True,
        scenario_bounds: Optional[Tuple[float, float, float, float]] = None,
    ) -> plt.Figure:
        """
        Plot robot trajectory with state coloring and environment.

        Args:
            result: SimulationResult containing trajectory and metadata
            walls: List of wall line segments as [(x1, y1, x2, y2), ...]
            title: Plot title
            show_legend: Whether to show legend
            scenario_bounds: Optional bounds as (x_min, x_max, y_min, y_max)

        Returns:
            matplotlib Figure object
        """
        fig, ax = plt.subplots(figsize=self.figsize)

        if not result.trajectory:
            ax.text(
                0.5, 0.5,
                'No trajectory data',
                ha='center', va='center',
                transform=ax.transAxes,
                fontsize=14
            )
            return fig

        # Draw walls
        self._draw_walls(ax, walls)

        # Extract trajectory arrays
        xs = [p.x for p in result.trajectory]
        ys = [p.y for p in result.trajectory]
        states = [p.yaw for _ in result.trajectory]  # Placeholder - we'll use actual state if available

        # Draw trajectory segments colored by state
        self._draw_trajectory_segments(ax, result.trajectory)

        # Mark start position
        ax.scatter(
            xs[0], ys[0],
            s=200, c='green', marker='o',
            label='Start', zorder=5, edgecolors='darkgreen', linewidth=2
        )

        # Mark end position
        ax.scatter(
            xs[-1], ys[-1],
            s=200, c='red', marker='X',
            label='End', zorder=5, edgecolors='darkred', linewidth=2
        )

        # Mark collision if occurred
        if result.collided and result.collision_point:
            ax.scatter(
                result.collision_point[0], result.collision_point[1],
                s=300, c='red', marker='*',
                label='Collision', zorder=6, edgecolors='darkred', linewidth=2
            )

        # Set labels and title
        ax.set_xlabel('X Position (cm)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Y Position (cm)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=14, fontweight='bold', pad=20)

        # Set equal aspect ratio
        ax.set_aspect('equal', adjustable='box')

        # Set axis limits
        if scenario_bounds:
            ax.set_xlim(scenario_bounds[0], scenario_bounds[1])
            ax.set_ylim(scenario_bounds[2], scenario_bounds[3])
        else:
            # Auto-scale with padding
            all_x = xs + [w[0] for w in walls] + [w[2] for w in walls]
            all_y = ys + [w[1] for w in walls] + [w[3] for w in walls]
            padding = 20
            ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
            ax.set_ylim(min(all_y) - padding, max(all_y) + padding)

        # Add grid
        ax.grid(True, alpha=0.3, linestyle='--')

        # Add statistics text
        stats_text = (
            f"Distance: {result.distance_traveled:.1f} cm\n"
            f"Time: {result.completion_time:.2f} s\n"
            f"Corners: {result.corner_count}\n"
            f"Steps: {result.total_steps}\n"
            f"Status: {'COLLISION' if result.collided else 'OK'}"
        )
        ax.text(
            0.02, 0.98, stats_text,
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
            family='monospace'
        )

        # Add legend with state colors
        if show_legend:
            self._add_state_legend(ax)

        fig.tight_layout()
        return fig

    def _draw_walls(self, ax, walls: List[Tuple[float, float, float, float]]):
        """Draw environment walls"""
        for wall in walls:
            x1, y1, x2, y2 = wall
            ax.plot([x1, x2], [y1, y2], 'k-', linewidth=3, zorder=2)

    def _draw_trajectory_segments(self, ax, trajectory: List[TrajectoryPoint]):
        """Draw trajectory line segments with state-based coloring"""
        for i in range(len(trajectory) - 1):
            p1 = trajectory[i]
            p2 = trajectory[i + 1]

            # Use a default color (no state info in TrajectoryPoint)
            # In future, state info can be added to TrajectoryPoint
            color = self.STATE_COLORS[2]  # Default to FORWARD green

            ax.plot(
                [p1.x, p2.x], [p1.y, p2.y],
                color=color, linewidth=1.5, alpha=0.7, zorder=3
            )

    def _add_state_legend(self, ax):
        """Add state color legend to plot"""
        handles = []
        labels = []

        for state_id in sorted(self.STATE_NAMES.keys()):
            state_name = self.STATE_NAMES[state_id]
            color = self.STATE_COLORS[state_id]

            handle = patches.Patch(facecolor=color, edgecolor='black')
            handles.append(handle)
            labels.append(state_name)

        ax.legend(
            handles, labels,
            loc='upper left', fontsize=10,
            title='States', title_fontsize=11,
            framealpha=0.95
        )

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
            dpi: Dots per inch (300 for publication quality)
            pdf: Also save as PDF vector format

        Returns:
            Path to saved PNG file
        """
        png_path = f"{filename}.png"
        fig.savefig(png_path, dpi=dpi, bbox_inches='tight', facecolor='white')

        if pdf:
            pdf_path = f"{filename}.pdf"
            fig.savefig(pdf_path, bbox_inches='tight', facecolor='white')

        plt.close(fig)

        return png_path
