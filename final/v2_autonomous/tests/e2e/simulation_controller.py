"""
Simulation Controller - Orchestrates the end-to-end simulation loop.

Implements the complete control cycle for wall-following robot:
1. Environment perception (ray-cast distances)
2. Sensor simulation with delay and noise
3. State machine update
4. Motor command output
5. Physics update with realistic dynamics
6. State history tracking

The controller manages all timing, sensor latency, and integrates:
- ConcreteVirtualEnvironment: 2D world with walls
- ConcretePhysicsSimulator: Motor dynamics, sensor delay, noise
- ConcreteWallFollower: Wall-following state machine
"""

from dataclasses import dataclass
from typing import List, Callable, Optional, Tuple, Union
from datetime import datetime

from simulator.concrete_virtual_environment import ConcreteVirtualEnvironment
from simulator.concrete_physics_simulator import ConcretePhysicsSimulator
from simulator.differential_drive import DifferentialDrive
from controller.concrete_wall_follower import ConcreteWallFollower
from controller.wall_follower_wrapper import WallFollowerState


@dataclass
class SimulationState:
    """Represents the robot state at a point in time during simulation."""

    time_ms: int                # Elapsed simulation time in milliseconds
    x: float                    # Robot X position in cm
    y: float                    # Robot Y position in cm
    theta: float                # Robot orientation in degrees
    front_dist: int             # Front sensor reading in cm
    right_dist: int             # Right sensor reading in cm
    state: int                  # WallFollower state value
    linear_cmd: float           # Linear motor command (normalized)
    angular_cmd: float          # Angular motor command (normalized)

    def __str__(self) -> str:
        """Format state for logging."""
        return (f"t={self.time_ms:5d}ms pos=({self.x:6.1f},{self.y:6.1f}) "
                f"theta={self.theta:6.1f}° sensors=(F:{self.front_dist:3d} R:{self.right_dist:3d}) "
                f"state={self.state} cmd=(lin:{self.linear_cmd:5.2f} ang:{self.angular_cmd:5.2f})")


class SimulationController:
    """
    Orchestrates the complete simulation loop for wall-following robot.

    Manages:
    - Time stepping (default 50Hz = 20ms per step)
    - Sensor reading acquisition with configurable delay (60ms default)
    - State machine updates
    - Motor command execution
    - Physics simulation
    - Collision detection
    - State history tracking

    Time flow:
        t=0ms -> Read sensors (with 60ms delay) -> Update controller -> Execute motors -> Physics step -> t=20ms
    """

    def __init__(
        self,
        environment: ConcreteVirtualEnvironment,
        physics: Union[ConcretePhysicsSimulator, DifferentialDrive],
        wall_follower: ConcreteWallFollower,
        dt_ms: int = 20
    ):
        """
        Initialize the simulation controller.

        Args:
            environment: Virtual environment with walls for ray-casting
            physics: Physics simulator (ConcretePhysicsSimulator or DifferentialDrive)
            wall_follower: Wall-following state machine controller
            dt_ms: Control loop time step in milliseconds (default 20ms = 50Hz)
        """
        self.env = environment
        self.physics = physics
        self.wall_follower = wall_follower
        self.dt_ms = dt_ms
        self.history: List[SimulationState] = []

        # Extract the actual simulator if wrapped in DifferentialDrive
        if isinstance(physics, DifferentialDrive):
            self.simulator = physics.simulator
            self.use_differential_drive = True
        else:
            self.simulator = physics
            self.use_differential_drive = False

    def run(
        self,
        duration_ms: int,
        stop_on_collision: bool = True,
        stop_on_done: bool = True,
        stop_condition: Optional[Callable[[SimulationState], bool]] = None
    ) -> List[SimulationState]:
        """
        Execute the simulation loop for the specified duration.

        Control loop (runs at dt_ms intervals):
            1. Read true distances from environment
            2. Set true distances to physics simulator
            3. Get sensor reading (with configured delay/noise)
            4. Update wall-follower state machine
            5. Get motor commands from wall-follower
            6. Update physics simulation
            7. Check termination conditions
            8. Record state to history

        Args:
            duration_ms: Total simulation duration in milliseconds
            stop_on_collision: Stop if robot collides with wall
            stop_on_done: Stop if wall-follower reaches DONE state
            stop_condition: Optional custom termination function
                          Takes SimulationState, returns True to stop

        Returns:
            List of SimulationState records for the entire simulation
        """
        self.history = []
        current_time_ms = 0

        # Start the wall-follower state machine
        self.wall_follower.start()

        # Main simulation loop
        while current_time_ms < duration_ms:
            current_time_s = current_time_ms / 1000.0

            # 1. Get current true position
            x, y, theta = self.simulator.get_position()

            # 2. Ray-cast to get true distances
            front_true = self.env.ray_cast(x, y, theta)  # 0° = forward
            right_true = self.env.ray_cast(x, y, theta - 90.0)  # -90° = right

            # 3. Set true distances to simulator (will apply delay/noise later)
            self.simulator.set_current_distances(front_true, right_true)

            # 4. Get sensor reading (includes 60ms delay, noise, anomalies)
            sensor_reading = self.simulator.get_sensor_reading()

            # 5. Update wall-follower with sensor data
            self.wall_follower.set_time(current_time_s)
            self.wall_follower.update(
                sensor_reading.front_distance,
                sensor_reading.right_distance,
                sensor_reading.yaw,
                sensor_reading.imu_valid
            )

            # 6. Get motor commands
            motor_cmd = self.wall_follower.get_motor_command()
            left_pwm = motor_cmd.left_pwm
            right_pwm = motor_cmd.right_pwm

            # Convert PWM to normalized commands for logging
            # These are stored internally after _set_motor_output
            linear_cmd = (left_pwm + right_pwm) / (2.0 * 255.0)
            angular_cmd = (right_pwm - left_pwm) / (2.0 * 255.0)

            # 7. Update physics simulation
            dt_s = self.dt_ms / 1000.0
            self.simulator.update(dt_s, left_pwm, right_pwm)

            # 8. Record current state
            state_record = SimulationState(
                time_ms=current_time_ms,
                x=x,
                y=y,
                theta=theta,
                front_dist=sensor_reading.front_distance,
                right_dist=sensor_reading.right_distance,
                state=int(self.wall_follower.state),
                linear_cmd=linear_cmd,
                angular_cmd=angular_cmd
            )
            self.history.append(state_record)

            # 9. Check termination conditions

            # Collision detection
            if stop_on_collision:
                robot_radius = self.simulator.robot_radius
                if self.env.check_collision(x, y, robot_radius):
                    break

            # Wall-follower done
            if stop_on_done:
                if self.wall_follower.state == WallFollowerState.DONE:
                    break

            # Custom termination condition
            if stop_condition is not None:
                if stop_condition(state_record):
                    break

            # Advance time
            current_time_ms += self.dt_ms

        return self.history

    def print_history(self, max_entries: Optional[int] = None) -> None:
        """
        Print simulation history to console.

        Args:
            max_entries: Maximum number of entries to print (None = all)
        """
        entries = self.history
        if max_entries is not None and len(entries) > max_entries:
            # Show first and last entries
            first_count = max_entries // 2
            last_count = max_entries - first_count
            print(f"Showing first {first_count} and last {last_count} entries...")
            entries = self.history[:first_count] + ["..."] + self.history[-last_count:]

        for entry in entries:
            if entry == "...":
                print("...")
            else:
                print(entry)

    def get_summary(self) -> dict:
        """
        Get simulation summary statistics.

        Returns:
            Dictionary with simulation metrics:
            - total_steps: Number of simulation steps
            - duration_ms: Total simulation duration
            - max_x, max_y: Maximum positions reached
            - min_x, min_y: Minimum positions reached
            - total_distance: Estimated total distance traveled
            - corner_count: Number of corners detected
            - final_state: Final wall-follower state
        """
        if not self.history:
            return {}

        # Calculate distance traveled
        total_distance = 0.0
        for i in range(1, len(self.history)):
            prev = self.history[i - 1]
            curr = self.history[i]
            dx = curr.x - prev.x
            dy = curr.y - prev.y
            total_distance += (dx * dx + dy * dy) ** 0.5

        return {
            'total_steps': len(self.history),
            'duration_ms': self.history[-1].time_ms if self.history else 0,
            'max_x': max(s.x for s in self.history),
            'min_x': min(s.x for s in self.history),
            'max_y': max(s.y for s in self.history),
            'min_y': min(s.y for s in self.history),
            'total_distance': total_distance,
            'corner_count': self.wall_follower.get_corner_count(),
            'final_state': int(self.wall_follower.state),
        }

    @classmethod
    def create_standard(
        cls,
        room_width: float = 150.0,
        room_height: float = 100.0,
        start_x: float = 10.0,
        start_y: float = 10.0,
        start_theta: float = 0.0
    ) -> 'SimulationController':
        """
        Create a standard simulation environment with default parameters.

        Sets up:
        - Rectangular room (default 150x100 cm)
        - Sensor delay: 60ms
        - Noise level: 2cm (σ)
        - Wheel variance: ±5%
        - Control frequency: 50Hz (20ms steps)

        Args:
            room_width: Room width in cm
            room_height: Room height in cm
            start_x: Initial robot X position in cm
            start_y: Initial robot Y position in cm
            start_theta: Initial robot orientation in degrees

        Returns:
            Configured SimulationController instance
        """
        # Create environment
        env = ConcreteVirtualEnvironment.create_rectangular_room(room_width, room_height)

        # Create physics simulator
        physics = ConcretePhysicsSimulator(max_range=500.0)
        physics.set_sensor_delay(60.0)  # 60ms delay
        physics.set_noise_level(2.0)  # 2cm noise
        physics.set_wheel_variance(0.05)  # ±5% variance
        physics.reset_position(start_x, start_y, start_theta)

        # Create wall-follower controller
        wall_follower = ConcreteWallFollower()

        # Create and return simulation controller
        return cls(env, physics, wall_follower, dt_ms=20)

    def reset(self) -> None:
        """Reset the simulation state for re-running."""
        self.history = []
        self.wall_follower.reset()
        # Reset physics position (keep other settings)
        self.simulator.reset_position(0.0, 0.0, 0.0)
