#!/usr/bin/env python3
"""
SDK2 Navigator for Unitree Go2
==============================

Navigation system combining SDK2 interface with path planning and obstacle avoidance.

Features:
- Waypoint navigation
- Obstacle avoidance using built-in range sensors
- Pure pursuit path following
- Velocity ramping for smooth motion
- Safety monitoring (roll/pitch limits, battery check)

Based on:
- unitree_sdk2_python: https://github.com/unitreerobotics/unitree_sdk2_python
- ROS2 Nav2 concepts adapted for direct SDK2 use
"""

import math
import time
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Callable, Tuple, Dict, Any
from enum import Enum
import numpy as np

from sdk2_interface import (
    Go2SDK2Interface,
    RobotState,
    VelocityCommand,
    RobotMode,
    GaitType,
)


class NavigationMode(Enum):
    """Navigation operation modes."""
    IDLE = "idle"
    WAYPOINT = "waypoint"
    CONTINUOUS = "continuous"
    PATROL = "patrol"


class NavigationStatus(Enum):
    """Navigation status."""
    IDLE = "idle"
    PLANNING = "planning"
    MOVING = "moving"
    WAITING = "waiting"
    ARRIVED = "arrived"
    FAILED = "failed"
    ABORTED = "aborted"


@dataclass
class Waypoint:
    """Navigation waypoint."""
    name: str
    x: float
    y: float
    yaw: float = 0.0
    max_velocity: float = 0.5  # m/s
    tolerance: float = 0.2  # m
    pause_duration: float = 0.0  # seconds

    def distance_to(self, x: float, y: float) -> float:
        """Calculate distance to given position."""
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)

    def angle_to(self, x: float, y: float) -> float:
        """Calculate angle to given position."""
        return math.atan2(self.y - y, self.x - x)


@dataclass
class NavigatorConfig:
    """Configuration for navigator."""
    # Velocity limits
    max_linear_velocity: float = 0.5  # m/s
    max_angular_velocity: float = 1.0  # rad/s

    # Acceleration limits
    max_linear_accel: float = 1.0  # m/s^2
    max_angular_accel: float = 2.0  # rad/s^2

    # Navigation tolerances
    position_tolerance: float = 0.2  # m
    angle_tolerance: float = 0.15  # rad (~8.5 degrees)

    # Pure pursuit parameters
    lookahead_distance: float = 0.3  # m
    min_lookahead: float = 0.1  # m
    max_lookahead: float = 0.5  # m

    # Obstacle avoidance
    obstacle_distance: float = 0.4  # m
    obstacle_slowdown_distance: float = 0.7  # m
    enable_obstacle_avoidance: bool = False

    # Safety
    max_roll: float = math.radians(20)  # rad
    max_pitch: float = math.radians(20)  # rad
    min_battery: float = 10.0  # percent

    # Control rate
    control_rate: float = 20.0  # Hz

    # Network interface
    network_interface: str = "enP8p1s0"
    robot_ip: str = "192.168.123.161"


class Go2Navigator:
    """
    Navigator for Unitree Go2 using SDK2 interface.

    Usage:
        config = NavigatorConfig(max_linear_velocity=0.3)
        navigator = Go2Navigator(config)

        navigator.connect()
        navigator.start()

        # Navigate to single goal
        navigator.navigate_to(x=2.0, y=1.5)

        # Or use waypoints
        navigator.add_waypoint(Waypoint("goal1", 2.0, 0.0))
        navigator.add_waypoint(Waypoint("goal2", 2.0, 2.0))
        navigator.run_waypoints()

        navigator.stop()
        navigator.disconnect()
    """

    def __init__(self, config: Optional[NavigatorConfig] = None):
        self.config = config or NavigatorConfig()

        # Interface
        self.interface = Go2SDK2Interface(
            network_interface=self.config.network_interface,
            robot_ip=self.config.robot_ip
        )

        # State
        self._running = False
        self._mode = NavigationMode.IDLE
        self._status = NavigationStatus.IDLE
        self._current_pose = (0.0, 0.0, 0.0)  # x, y, yaw
        self._current_velocity = VelocityCommand()

        # Navigation
        self._waypoints: List[Waypoint] = []
        self._current_waypoint_index = 0
        self._goal_waypoint: Optional[Waypoint] = None
        self._path_index = 0
        self._waiting_until = 0.0

        # Threading
        self._control_thread: Optional[threading.Thread] = None
        self._state_lock = threading.Lock()

        # Callbacks
        self._status_callbacks: List[Callable[[NavigationStatus], None]] = []
        self._waypoint_callbacks: List[Callable[[Waypoint, int], None]] = []

        # Statistics
        self._last_state_time = 0.0
        self._state_count = 0

        # Obstacle avoidance state
        self._obstacle_avoidance_initialized = False

    def connect(self) -> bool:
        """Connect to robot."""
        if not self.interface.connect():
            return False

        # Register state callback
        self.interface.add_state_callback(self._on_state_update)

        # Wait for first state
        if not self.interface.wait_for_state(timeout=5.0):
            print("Timeout waiting for state")
            return False

        # Initialize obstacle avoidance if enabled
        if self.config.enable_obstacle_avoidance:
            if self.interface.enable_obstacle_avoidance():
                self._obstacle_avoidance_initialized = True
                print("Obstacle avoidance enabled 🛡️")
            else:
                print("Warning: Failed to enable obstacle avoidance")

        print(f"Connected to Go2 at {self.config.robot_ip}")
        return True

    def start(self):
        """Start navigation control loop."""
        self._running = True

        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True
        )
        self._control_thread.start()

        print("Navigator started")

    def stop(self):
        """Stop navigation."""
        self._running = False
        self.interface.stop()

        if self._control_thread:
            self._control_thread.join(timeout=2.0)

        print("Navigator stopped")

    def disconnect(self):
        """Disconnect from robot."""
        self.stop()
        self.interface.disconnect()

    def _on_state_update(self, state: RobotState):
        """Handle state update from robot."""
        with self._state_lock:
            self._current_pose = (
                float(state.position[0]),
                float(state.position[1]),
                float(state.orientation[2])
            )
            self._last_state_time = time.time()
            self._state_count += 1

    def get_pose(self) -> Tuple[float, float, float]:
        """Get current pose (x, y, yaw)."""
        with self._state_lock:
            return self._current_pose

    def get_status(self) -> NavigationStatus:
        """Get current navigation status."""
        return self._status

    def add_status_callback(self, callback: Callable[[NavigationStatus], None]):
        """Register callback for status changes."""
        self._status_callbacks.append(callback)

    def add_waypoint_callback(self, callback: Callable[[Waypoint, int], None]):
        """Register callback for waypoint changes (waypoint, index)."""
        self._waypoint_callbacks.append(callback)

    def _set_status(self, status: NavigationStatus):
        """Update status and notify callbacks."""
        if self._status != status:
            self._status = status
            for callback in self._status_callbacks:
                try:
                    callback(status)
                except Exception:
                    pass

    def navigate_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        max_velocity: float = 0.5,
        tolerance: float = 0.2
    ) -> bool:
        """
        Navigate to a single goal.

        Args:
            x: Goal x position
            y: Goal y position
            yaw: Goal yaw angle (rad)
            max_velocity: Maximum velocity (m/s)
            tolerance: Position tolerance (m)

        Returns:
            True if navigation started
        """
        self._waypoints = [
            Waypoint("goal", x, y, yaw, max_velocity, tolerance)
        ]
        self._current_waypoint_index = 0
        self._mode = NavigationMode.WAYPOINT
        self._set_status(NavigationStatus.PLANNING)

        return True

    def add_waypoint(self, waypoint: Waypoint):
        """Add waypoint to the list."""
        self._waypoints.append(waypoint)

    def set_waypoints(self, waypoints: List[Waypoint]):
        """Set complete waypoint list."""
        self._waypoints = waypoints.copy()
        self._current_waypoint_index = 0

    def clear_waypoints(self):
        """Clear all waypoints."""
        self._waypoints = []
        self._current_waypoint_index = 0

    def run_waypoints(self, loop: bool = False) -> bool:
        """
        Start navigating through waypoints.

        Args:
            loop: Loop through waypoints continuously

        Returns:
            True if navigation started
        """
        if not self._waypoints:
            print("No waypoints to navigate")
            return False

        self._current_waypoint_index = 0
        self._mode = NavigationMode.WAYPOINT
        self._set_status(NavigationStatus.PLANNING)

        print(f"Starting navigation through {len(self._waypoints)} waypoints")
        return True

    def abort(self):
        """Abort current navigation."""
        self._mode = NavigationMode.IDLE
        self._set_status(NavigationStatus.ABORTED)
        self.interface.stop()

    def _control_loop(self):
        """Main control loop."""
        dt = 1.0 / self.config.control_rate

        while self._running:
            start_time = time.time()

            try:
                self._control_step()
            except Exception as e:
                print(f"Control error: {e}")

            # Maintain control rate
            elapsed = time.time() - start_time
            if elapsed < dt:
                time.sleep(dt - elapsed)

    def _control_step(self):
        """Single control step."""
        # Check if waiting (pause at waypoint)
        if self._waiting_until > 0:
            if time.time() < self._waiting_until:
                self.interface.stop()
                return
            else:
                self._waiting_until = 0

        # Get current pose
        x, y, yaw = self.get_pose()

        # Check safety
        if not self._check_safety():
            self.interface.stop()
            self._set_status(NavigationStatus.FAILED)
            return

        # Select goal based on mode
        goal = self._select_goal(x, y)

        if goal is None:
            if self._status == NavigationStatus.MOVING:
                self._set_status(NavigationStatus.ARRIVED)
            self.interface.stop()
            return

        # Check if arrived
        if goal.distance_to(x, y) < goal.tolerance:
            self._on_waypoint_reached(goal)
            return

        # Compute velocity command
        self._set_status(NavigationStatus.MOVING)

        velocity = self._compute_velocity(x, y, yaw, goal)

        # Apply obstacle avoidance
        if self.config.enable_obstacle_avoidance:
            velocity = self._apply_obstacle_avoidance(velocity, x, y, yaw)

        # Apply velocity limits
        velocity = self._limit_velocity(velocity)

        # Send command (uses obstacle avoidance if enabled in interface)
        self.interface.send_velocity_auto(velocity.vx, velocity.vy, velocity.vyaw)

        self._current_velocity = velocity

    def _select_goal(self, x: float, y: float) -> Optional[Waypoint]:
        """Select current goal based on mode and waypoints."""
        if self._mode == NavigationMode.IDLE:
            return None

        if self._mode == NavigationMode.WAYPOINT:
            if not self._waypoints:
                return None

            # Check if we need to advance to next waypoint
            if self._goal_waypoint is not None:
                if self._goal_waypoint.distance_to(x, y) < self._goal_waypoint.tolerance:
                    # Already at current waypoint, will be handled in _control_step
                    return self._goal_waypoint

            # Get next waypoint
            if self._current_waypoint_index < len(self._waypoints):
                wp = self._waypoints[self._current_waypoint_index]
                if self._goal_waypoint != wp:
                    self._goal_waypoint = wp
                    # Notify callbacks
                    for callback in self._waypoint_callbacks:
                        try:
                            callback(wp, self._current_waypoint_index)
                        except Exception:
                            pass
                return wp
            else:
                # All waypoints completed
                return None

        return self._goal_waypoint

    def _compute_velocity(
        self,
        x: float,
        y: float,
        yaw: float,
        goal: Waypoint
    ) -> VelocityCommand:
        """Compute velocity command using pure pursuit."""
        # Distance and angle to goal
        distance = goal.distance_to(x, y)
        target_angle = goal.angle_to(x, y)

        # Adjust for robot orientation
        angle_error = self._normalize_angle(target_angle - yaw)

        # Adaptive lookahead based on distance
        lookahead = min(
            self.config.max_lookahead,
            max(self.config.min_lookahead, distance * 0.5)
        )

        # Pure pursuit
        curvature = 2.0 * math.sin(angle_error) / lookahead

        # Convert curvature to velocity
        # v = max_velocity * min(1, distance / lookahead)
        linear_vel = min(goal.max_velocity, self.config.max_linear_velocity)
        linear_vel *= min(1.0, distance / lookahead)

        # Slow down for large angle errors
        angle_factor = max(0.3, 1.0 - abs(angle_error) / math.pi)
        linear_vel *= angle_factor

        angular_vel = curvature * linear_vel
        angular_vel = np.clip(
            angular_vel,
            -self.config.max_angular_velocity,
            self.config.max_angular_velocity
        )

        return VelocityCommand(vx=linear_vel, vy=0.0, vyaw=angular_vel)

    def _apply_obstacle_avoidance(
        self,
        velocity: VelocityCommand,
        x: float,
        y: float,
        yaw: float
    ) -> VelocityCommand:
        """
        Apply obstacle avoidance using built-in ObstaclesAvoidClient.

        Note: When obstacle avoidance is enabled, the robot's built-in
        system handles obstacle detection and avoidance. We just need
        to use the correct API method.
        """
        # Obstacle avoidance is handled by send_velocity_with_avoidance()
        # This method is a placeholder for any custom logic
        return velocity

    def _limit_velocity(self, velocity: VelocityCommand) -> VelocityCommand:
        """Apply velocity and acceleration limits."""
        # Clamp to max velocity
        vx = np.clip(velocity.vx, -self.config.max_linear_velocity, self.config.max_linear_velocity)
        vyaw = np.clip(velocity.vyaw, -self.config.max_angular_velocity, self.config.max_angular_velocity)

        # Apply acceleration limits
        if self._current_velocity is not None:
            dvx = vx - self._current_velocity.vx
            dvyaw = vyaw - self._current_velocity.vyaw

            dt = 1.0 / self.config.control_rate

            if abs(dvx) > self.config.max_linear_accel * dt:
                vx = self._current_velocity.vx + np.sign(dvx) * self.config.max_linear_accel * dt

            if abs(dvyaw) > self.config.max_angular_accel * dt:
                vyaw = self._current_velocity.vyaw + np.sign(dvyaw) * self.config.max_angular_accel * dt

        return VelocityCommand(vx=vx, vy=0.0, vyaw=vyaw)

    def _on_waypoint_reached(self, waypoint: Waypoint):
        """Handle waypoint reached."""
        print(f"Reached waypoint: {waypoint.name}")

        # Pause if configured
        if waypoint.pause_duration > 0:
            print(f"Pausing for {waypoint.pause_duration}s")
            self._waiting_until = time.time() + waypoint.pause_duration
            self._set_status(NavigationStatus.WAITING)
            return

        # Move to next waypoint
        if self._mode == NavigationMode.WAYPOINT:
            self._current_waypoint_index += 1

            if self._current_waypoint_index >= len(self._waypoints):
                print("All waypoints completed!")
                self._set_status(NavigationStatus.ARRIVED)
                self._mode = NavigationMode.IDLE

    def _check_safety(self) -> bool:
        """Check safety conditions."""
        state = self.interface.get_state()

        # Check roll and pitch
        roll, pitch = abs(state.imu.rpy[0]), abs(state.imu.rpy[1])
        if roll > self.config.max_roll or pitch > self.config.max_pitch:
            print(f"Safety: Orientation unsafe (roll={roll:.1f}, pitch={pitch:.1f})")
            return False

        # Check battery
        if state.bms.soc < self.config.min_battery:
            print(f"Safety: Low battery ({state.bms.soc:.1f}%)")
            return False

        return True

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class SimpleNavigator:
    """
    Simplified navigator for basic navigation tasks.

    Easier to use for simple use cases.
    """

    def __init__(self, network_interface: str = "enP8p1s0", robot_ip: str = "192.168.123.161"):
        self.config = NavigatorConfig(network_interface=network_interface, robot_ip=robot_ip)
        self.navigator = Go2Navigator(self.config)

    def connect(self) -> bool:
        """Connect to robot."""
        return self.navigator.connect()

    def start(self):
        """Start navigator."""
        self.navigator.start()

    def stop(self):
        """Stop navigator."""
        self.navigator.stop()

    def disconnect(self):
        """Disconnect from robot."""
        self.navigator.disconnect()

    def go_to(self, x: float, y: float, wait: bool = True) -> bool:
        """
        Go to position.

        Args:
            x: Target x position
            y: Target y position
            wait: Wait for arrival before returning

        Returns:
            True if navigation started
        """
        self.navigator.navigate_to(x, y)

        if wait:
            while self.navigator.get_status() not in (
                NavigationStatus.ARRIVED,
                NavigationStatus.FAILED,
                NavigationStatus.ABORTED
            ):
                time.sleep(0.1)

        return True

    def move_forward(self, distance: float, velocity: float = 0.3) -> bool:
        """Move forward by given distance."""
        x, y, yaw = self.navigator.get_pose()
        target_x = x + distance * math.cos(yaw)
        target_y = y + distance * math.sin(yaw)
        return self.go_to(target_x, target_y)

    def turn(self, angle: float, wait: bool = True) -> bool:
        """
        Turn by given angle.

        Note: This uses the SDK2 API directly for in-place turning.
        """
        import math

        # For pure turning, we need to send a yaw velocity
        # and wait until we've turned the desired amount
        start_yaw = self.navigator.get_pose()[2]
        target_yaw = start_yaw + angle

        turn_rate = 0.5 if angle > 0 else -0.5  # rad/s

        # Start turning
        self.navigator.interface.send_velocity(0.0, 0.0, turn_rate)

        if not wait:
            return True

        # Wait for turn to complete
        while True:
            current_yaw = self.navigator.get_pose()[2]
            yaw_error = Go2Navigator._normalize_angle(target_yaw - current_yaw)

            if abs(yaw_error) < 0.05:  # ~3 degrees
                self.navigator.interface.stop()
                return True

            time.sleep(0.05)
