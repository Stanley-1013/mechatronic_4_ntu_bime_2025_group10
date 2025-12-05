"""
State Timeline - Visualizes state transitions over time.

Generates a timeline plot showing:
- State values as horizontal color blocks
- State transition points marked
- Time on X-axis
- State names and colors on Y-axis
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
import numpy as np
from typing import List, Tuple, Optional, Dict
from datetime import datetime

from assertions.e2e_assertions import SimulationResult, TrajectoryPoint


class StateTimeline:
    """Visualize robot state changes over time"""

    # State definitions
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

    def __init__(self, figsize: Tuple[int, int] = (14, 6)):
        """
        Initialize state timeline plotter.

        Args:
            figsize: Figure size as (width, height) in inches
        """
        self.figsize = figsize

    def plot_state_timeline(
        self,
        result: SimulationResult,
        title: str = "Robot State Timeline",
        infer_states: bool = True,
    ) -> plt.Figure:
        """
        Plot state timeline visualization.

        Args:
            result: SimulationResult with trajectory
            title: Plot title
            infer_states: Whether to infer states from motor commands
                         (since TrajectoryPoint doesn't store state directly)

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

        # Extract timestamps and infer states
        times = [p.timestamp for p in result.trajectory]
        states = self._infer_states(result.trajectory) if infer_states else [2] * len(result.trajectory)

        # Identify state transitions
        transitions = self._find_transitions(states, times)

        # Draw state blocks
        self._draw_state_blocks(ax, transitions, times, states)

        # Set labels
        ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
        ax.set_ylabel('State', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=14, fontweight='bold', pad=20)

        # Set Y-axis
        ax.set_ylim(-0.5, 5.5)
        ax.set_yticks([0, 1, 2, 3, 4, 5])
        ax.set_yticklabels([self.STATE_NAMES.get(i, f'State {i}') for i in range(6)])

        # Set X-axis
        if times:
            ax.set_xlim(times[0] - 0.5, times[-1] + 0.5)

        # Add grid
        ax.grid(True, alpha=0.3, linestyle='--', axis='x')

        # Add transition markers
        self._mark_transitions(ax, transitions)

        # Add statistics
        stats_text = (
            f"Total Time: {result.completion_time:.2f} s\n"
            f"State Changes: {len(transitions)}\n"
            f"Final State: {self.STATE_NAMES.get(states[-1] if states else 0, 'Unknown')}"
        )
        ax.text(
            0.98, 0.97, stats_text,
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
            family='monospace'
        )

        fig.tight_layout()
        return fig

    def _infer_states(self, trajectory: List[TrajectoryPoint]) -> List[int]:
        """
        Infer states from motor commands.

        Heuristic:
        - IDLE (0): both motors ~0
        - BACKUP (3): both motors negative
        - TURN_LEFT (4): left motor low, right motor high
        - FIND_WALL (1): right motor positive, left motor mixed
        - FORWARD (2): both motors positive (default for positive)
        """
        states = []
        for p in trajectory:
            left = p.left_pwm
            right = p.right_pwm

            if abs(left) < 10 and abs(right) < 10:
                state = 0  # IDLE
            elif left < 0 and right < 0:
                state = 3  # BACKUP
            elif left < 30 and right > 100:
                state = 4  # TURN_LEFT
            elif left > right + 30:
                state = 1  # FIND_WALL (turning right)
            elif left > 0 and right > 0:
                state = 2  # FORWARD
            else:
                state = 2  # Default to FORWARD

            states.append(state)

        return states

    def _find_transitions(
        self,
        states: List[int],
        times: List[float]
    ) -> List[Tuple[float, int, int]]:
        """
        Find state transition points.

        Returns:
            List of (time, old_state, new_state) tuples
        """
        transitions = [(times[0], -1, states[0])]  # Initial state

        for i in range(1, len(states)):
            if states[i] != states[i - 1]:
                transitions.append((times[i], states[i - 1], states[i]))

        return transitions

    def _draw_state_blocks(
        self,
        ax,
        transitions: List[Tuple[float, int, int]],
        times: List[float],
        states: List[int]
    ):
        """Draw colored blocks representing states over time"""
        for i, (time, old_state, new_state) in enumerate(transitions):
            # Find end time of this state
            if i + 1 < len(transitions):
                end_time = transitions[i + 1][0]
            else:
                end_time = times[-1] + 0.1

            # Get state color
            state = new_state
            color = self.STATE_COLORS.get(state, '#CCCCCC')

            # Draw rectangle
            rect = patches.Rectangle(
                (time, state - 0.4),
                end_time - time,
                0.8,
                linewidth=2,
                edgecolor='black',
                facecolor=color,
                alpha=0.8,
                zorder=3
            )
            ax.add_patch(rect)

    def _mark_transitions(self, ax, transitions: List[Tuple[float, int, int]]):
        """Mark state transition points with vertical lines"""
        for i, (time, old_state, new_state) in enumerate(transitions):
            if old_state != -1:  # Skip initial state marker
                ax.axvline(x=time, color='gray', linestyle='--', linewidth=1, alpha=0.5, zorder=2)
                # Add transition label
                ax.text(
                    time, 5.7,
                    f"{self.STATE_NAMES.get(old_state, '?')}→{self.STATE_NAMES.get(new_state, '?')}",
                    rotation=45,
                    fontsize=8,
                    ha='left',
                    alpha=0.7
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
