#!/usr/bin/env python3
"""
High-Level Navigation API for Unitree Go2
==========================================

User-friendly API for common navigation tasks.

This module provides a high-level interface that handles the complexity
of the SDK2 interface and navigator, making it easy to:

- Navigate to specific locations
- Follow waypoint paths
- Perform common movements
- Monitor navigation progress

Example:
    from sdk2_navigation import HighLevelNavigator

    # Create navigator
    nav = HighLevelNavigator()

    # Connect
    nav.connect()

    # Navigate to a location
    result = nav.go_to(x=2.0, y=1.5)

    # Or use waypoints
    nav.add_waypoint("kitchen", 2.0, 0.0)
    nav.add_waypoint("table", 2.0, 2.0)
    nav.run_waypoints()

    # Disconnect
    nav.disconnect()
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Callable, Tuple, Dict, Any, Union
from enum import Enum

from sdk2_interface import (
    Go2SDK2Interface,
    RobotState,
    VelocityCommand,
    RobotMode,
)
from sdk2_navigator import (
    Go2Navigator,
    Waypoint,
    NavigationMode,
    NavigationStatus,
    NavigatorConfig,
)


class TaskStatus(Enum):
    """Status of a navigation task."""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class NavigationGoal:
    """
    Navigation goal specification.

    Can represent a single point or a sequence of waypoints.
    """
    x: float
    y: float
    yaw: float = 0.0
    tolerance: float = 0.2
    max_velocity: float = 0.5
    name: str = "goal"

    # For multi-goal paths
    sub_goals: List['NavigationGoal'] = field(default_factory=list)

    def is_single_goal(self) -> bool:
        """Check if this is a single goal (no sub-goals)."""
        return len(self.sub_goals) == 0

    def get_waypoints(self) -> List[Waypoint]:
        """Convert to list of Waypoint objects."""
        if self.is_single_goal():
            return [Waypoint(self.name, self.x, self.y, self.yaw, self.max_velocity, self.tolerance)]
        else:
            return [Waypoint(g.name, g.x, g.y, g.yaw, g.max_velocity, g.tolerance)
                    for g in self.sub_goals]

    @classmethod
    def from_waypoints(cls, waypoints: List[Waypoint], name: str = "path") -> 'NavigationGoal':
        """Create NavigationGoal from list of Waypoints."""
        if len(waypoints) == 1:
            wp = waypoints[0]
            return cls(wp.x, wp.y, wp.yaw, wp.tolerance, wp.max_velocity, wp.name)
        else:
            # Use first waypoint as primary, rest as sub-goals
            first = waypoints[0]
            sub_goals = [NavigationGoal(wp.x, wp.y, wp.yaw, wp.tolerance, wp.max_velocity, wp.name)
                        for wp in waypoints[1:]]
            return cls(first.x, first.y, first.yaw, first.tolerance, first.max_velocity, name, sub_goals)


@dataclass
class PathPlan:
    """Result of path planning."""
    waypoints: List[Waypoint]
    total_distance: float
    estimated_time: float
    success: bool

    def __len__(self) -> int:
        return len(self.waypoints)


class HighLevelNavigator:
    """
    High-level navigation API for Unitree Go2.

    Provides a simplified interface for common navigation tasks.
    Handles connection management, state monitoring, and navigation tasks.

    Usage:
        nav = HighLevelNavigator()
        nav.connect()

        # Single goal
        nav.go_to(x=2.0, y=1.5)

        # Named locations
        nav.set_location("kitchen", 2.0, 0.0)
        nav.go_to("kitchen")

        # Waypoints
        nav.add_waypoint("point1", 2.0, 0.0)
        nav.add_waypoint("point2", 2.0, 2.0)
        nav.run_waypoints()

        nav.disconnect()
    """

    def __init__(
        self,
        network_interface: str = "enP8p1s0",
        robot_ip: str = "192.168.123.161",
        config: Optional[NavigatorConfig] = None
    ):
        self.config = config or NavigatorConfig(
            network_interface=network_interface,
            robot_ip=robot_ip
        )

        # Components
        self.interface = Go2SDK2Interface(
            network_interface=network_interface,
            robot_ip=robot_ip
        )
        self.navigator = Go2Navigator(self.config)

        # Named locations
        self._locations: Dict[str, Tuple[float, float, float]] = {}

        # Task management
        self._current_task: Optional[NavigationGoal] = None
        self._task_status = TaskStatus.PENDING
        self._task_callbacks: List[Callable[[TaskStatus], None]] = []

        # Connection state
        self._connected = False

    def connect(self, timeout: float = 10.0) -> bool:
        """
        Connect to the robot.

        Args:
            timeout: Connection timeout in seconds

        Returns:
            True if connected successfully
        """
        if not self.navigator.connect():
            return False

        self._connected = True

        # Register status callback
        self.navigator.add_status_callback(self._on_status_change)

        # Start navigator
        self.navigator.start()

        return True

    def disconnect(self):
        """Disconnect from the robot."""
        self._connected = False
        self.navigator.disconnect()

    def is_connected(self) -> bool:
        """Check if connected to robot."""
        return self._connected

    def get_position(self) -> Tuple[float, float, float]:
        """
        Get current robot position.

        Returns:
            (x, y, yaw) tuple
        """
        return self.navigator.get_pose()

    def get_battery(self) -> float:
        """
        Get battery level.

        Returns:
            Battery percentage (0-100)
        """
        state = self.interface.get_state()
        return state.bms.soc

    def get_status(self) -> TaskStatus:
        """Get current task status."""
        return self._task_status

    # ========================================================================
    # Location Management
    # ========================================================================

    def set_location(self, name: str, x: float, y: float, yaw: float = 0.0):
        """
        Define a named location.

        Args:
            name: Location name
            x: X coordinate
            y: Y coordinate
            yaw: Yaw angle (rad)
        """
        self._locations[name] = (x, y, yaw)

    def get_location(self, name: str) -> Optional[Tuple[float, float, float]]:
        """
        Get named location coordinates.

        Args:
            name: Location name

        Returns:
            (x, y, yaw) tuple or None if not found
        """
        return self._locations.get(name)

    def remove_location(self, name: str):
        """Remove a named location."""
        self._locations.pop(name, None)

    def list_locations(self) -> List[str]:
        """List all named locations."""
        return list(self._locations.keys())

    # ========================================================================
    # Navigation Methods
    # ========================================================================

    def go_to(
        self,
        x_or_name: Union[float, str],
        y: Optional[float] = None,
        yaw: float = 0.0,
        wait: bool = True,
        timeout: float = 60.0
    ) -> bool:
        """
        Navigate to a location.

        Args:
            x_or_name: X coordinate or named location
            y: Y coordinate (not needed if using named location)
            yaw: Target yaw angle (rad)
            wait: Wait for navigation to complete
            timeout: Maximum time to wait (seconds)

        Returns:
            True if navigation successful

        Examples:
            nav.go_to(2.0, 1.5)           # Go to coordinates
            nav.go_to(2.0, 1.5, yaw=1.57)  # Go with specific heading
            nav.go_to("kitchen")           # Go to named location
        """
        # Resolve target
        if isinstance(x_or_name, str):
            location = self.get_location(x_or_name)
            if location is None:
                print(f"Unknown location: {x_or_name}")
                return False
            x, y, yaw = location
            name = x_or_name
        else:
            x = float(x_or_name)
            if y is None:
                raise ValueError("y coordinate required when x is numeric")
            y = float(y)
            name = "goal"

        # Create goal
        goal = NavigationGoal(x, y, yaw, name=name)

        return self._navigate(goal, wait, timeout)

    def go_to_relative(
        self,
        dx: float,
        dy: float,
        dyaw: float = 0.0,
        wait: bool = True
    ) -> bool:
        """
        Move relative to current position.

        Args:
            dx: Forward distance (m)
            dy: Lateral distance (m)
            dyaw: Yaw change (rad)
            wait: Wait for completion

        Returns:
            True if successful
        """
        x, y, yaw = self.get_position()
        target_x = x + dx * math.cos(yaw) - dy * math.sin(yaw)
        target_y = y + dx * math.sin(yaw) + dy * math.cos(yaw)
        target_yaw = yaw + dyaw

        return self.go_to(target_x, target_y, target_yaw, wait)

    def move_forward(self, distance: float, velocity: float = 0.3, wait: bool = True) -> bool:
        """
        Move forward by specified distance.

        Args:
            distance: Distance to move (m)
            velocity: Forward velocity (m/s)
            wait: Wait for completion

        Returns:
            True if successful
        """
        return self.go_to_relative(distance, 0.0, 0.0, wait)

    def move_backward(self, distance: float, velocity: float = 0.3, wait: bool = True) -> bool:
        """Move backward by specified distance."""
        return self.go_to_relative(-distance, 0.0, 0.0, wait)

    def move_left(self, distance: float, velocity: float = 0.3, wait: bool = True) -> bool:
        """Move left by specified distance."""
        return self.go_to_relative(0.0, distance, 0.0, wait)

    def move_right(self, distance: float, velocity: float = 0.3, wait: bool = True) -> bool:
        """Move right by specified distance."""
        return self.go_to_relative(0.0, -distance, 0.0, wait)

    def turn(self, angle: float, wait: bool = True) -> bool:
        """
        Turn in place by specified angle.

        Args:
            angle: Angle to turn (rad, positive = counter-clockwise)
            wait: Wait for completion

        Returns:
            True if successful
        """
        x, y, yaw = self.get_position()
        target_yaw = yaw + angle

        # Normalize
        while target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        while target_yaw < -math.pi:
            target_yaw += 2 * math.pi

        return self.go_to(x, y, target_yaw, wait)

    # ========================================================================
    # Waypoint Navigation
    # ========================================================================

    def add_waypoint(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        max_velocity: float = 0.5
    ):
        """Add a waypoint to the path."""
        wp = Waypoint(name, x, y, yaw, max_velocity=max_velocity)
        self.navigator.add_waypoint(wp)

    def clear_waypoints(self):
        """Clear all waypoints."""
        self.navigator.clear_waypoints()

    def run_waypoints(self, loop: bool = False, wait: bool = True) -> bool:
        """
        Navigate through all waypoints.

        Args:
            loop: Loop through waypoints continuously
            wait: Wait for completion

        Returns:
            True if navigation started
        """
        result = self.navigator.run_waypoints(loop)

        if wait and result:
            self._wait_for_completion(timeout=300.0)

        return result

    # ========================================================================
    # Path Planning
    # ========================================================================

    def plan_path(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float
    ) -> Optional[PathPlan]:
        """
        Plan a straight-line path (simple implementation).

        For more complex path planning with obstacle avoidance,
        integrate with SLAM mapping system.

        Args:
            start_x: Start x position
            start_y: Start y position
            goal_x: Goal x position
            goal_y: Goal y position

        Returns:
            PathPlan object
        """
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.sqrt(dx*dx + dy*dy)

        # Simple straight line
        wp = Waypoint("goal", goal_x, goal_y)
        estimated_time = distance / 0.5  # Assume 0.5 m/s

        return PathPlan(
            waypoints=[wp],
            total_distance=distance,
            estimated_time=estimated_time,
            success=True
        )

    # ========================================================================
    # Task Management
    # ========================================================================

    def add_task_callback(self, callback: Callable[[TaskStatus], None]):
        """Register callback for task status changes."""
        self._task_callbacks.append(callback)

    def cancel_navigation(self):
        """Cancel current navigation."""
        self.navigator.abort()
        self._task_status = TaskStatus.CANCELLED

    def _navigate(self, goal: NavigationGoal, wait: bool, timeout: float) -> bool:
        """Execute navigation task."""
        self._current_task = goal
        self._task_status = TaskStatus.RUNNING

        if goal.is_single_goal():
            self.navigator.navigate_to(
                goal.x,
                goal.y,
                goal.yaw,
                goal.max_velocity,
                goal.tolerance
            )
        else:
            self.navigator.set_waypoints(goal.get_waypoints())
            self.navigator.run_waypoints()

        if wait:
            return self._wait_for_completion(timeout)

        return True

    def _wait_for_completion(self, timeout: float) -> bool:
        """Wait for navigation to complete."""
        start_time = time.time()

        while time.time() - start_time < timeout:
            status = self.navigator.get_status()

            if status == NavigationStatus.ARRIVED:
                self._task_status = TaskStatus.COMPLETED
                return True
            elif status in (NavigationStatus.FAILED, NavigationStatus.ABORTED):
                self._task_status = TaskStatus.FAILED
                return False

            time.sleep(0.1)

        # Timeout
        self.navigator.abort()
        self._task_status = TaskStatus.FAILED
        return False

    def _on_status_change(self, status: NavigationStatus):
        """Handle navigator status change."""
        task_status = {
            NavigationStatus.IDLE: TaskStatus.PENDING,
            NavigationStatus.PLANNING: TaskStatus.RUNNING,
            NavigationStatus.MOVING: TaskStatus.RUNNING,
            NavigationStatus.WAITING: TaskStatus.RUNNING,
            NavigationStatus.ARRIVED: TaskStatus.COMPLETED,
            NavigationStatus.FAILED: TaskStatus.FAILED,
            NavigationStatus.ABORTED: TaskStatus.CANCELLED,
        }.get(status, TaskStatus.PENDING)

        if self._task_status != task_status:
            self._task_status = task_status
            for callback in self._task_callbacks:
                try:
                    callback(task_status)
                except Exception:
                    pass

    # ========================================================================
    # Robot Commands
    # ========================================================================

    def stop(self):
        """Stop robot movement."""
        self.interface.stop()

    def stand(self) -> bool:
        """Make robot stand up."""
        return self.interface.stand_up()

    def sit(self) -> bool:
        """Make robot sit down."""
        return self.interface.sit_down()

    def set_velocity(self, vx: float, vy: float, vyaw: float) -> bool:
        """
        Set robot velocity directly.

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            vyaw: Yaw rate (rad/s)

        Returns:
            True if command sent
        """
        return self.interface.send_velocity(vx, vy, vyaw)

    # ========================================================================
    # Context Manager Support
    # ========================================================================

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


class NavigationBuilder:
    """
    Builder pattern for creating complex navigation tasks.

    Example:
        task = (NavigationBuilder()
                .add_waypoint("start", 0, 0)
                .add_waypoint("point1", 2, 0, pause=1.0)
                .add_waypoint("point2", 2, 2)
                .add_waypoint("home", 0, 0)
                .with_max_velocity(0.3)
                .build())

        nav.navigate(task)
    """

    def __init__(self):
        self._waypoints: List[Waypoint] = []
        self._max_velocity: float = 0.5
        self._tolerance: float = 0.2
        self._loop: bool = False

    def add_waypoint(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        pause: float = 0.0
    ) -> 'NavigationBuilder':
        """Add a waypoint."""
        wp = Waypoint(name, x, y, yaw, self._max_velocity, self._tolerance, pause)
        self._waypoints.append(wp)
        return self

    def with_max_velocity(self, velocity: float) -> 'NavigationBuilder':
        """Set maximum velocity for all waypoints."""
        self._max_velocity = velocity
        return self

    def with_tolerance(self, tolerance: float) -> 'NavigationBuilder':
        """Set position tolerance for all waypoints."""
        self._tolerance = tolerance
        return self

    def with_loop(self, loop: bool = True) -> 'NavigationBuilder':
        """Enable waypoint looping."""
        self._loop = loop
        return self

    def build(self) -> NavigationGoal:
        """Build the navigation goal."""
        return NavigationGoal.from_waypoints(self._waypoints, "built_path")


# Convenience functions

def quick_navigate(
    x: float,
    y: float,
    network_interface: str = "enP8p1s0",
    robot_ip: str = "192.168.123.161"
) -> bool:
    """
    Quick navigation to a point.

    Handles connection and disconnection automatically.

    Args:
        x: Target x position
        y: Target y position
        network_interface: Network interface
        robot_ip: Robot IP address

    Returns:
        True if navigation successful
    """
    with HighLevelNavigator(network_interface, robot_ip) as nav:
        return nav.go_to(x, y, wait=True)


def create_patrol(
    points: List[Tuple[float, float]],
    names: Optional[List[str]] = None
) -> NavigationGoal:
    """
    Create a patrol path from a list of points.

    Args:
        points: List of (x, y) tuples
        names: Optional list of waypoint names

    Returns:
        NavigationGoal for the patrol
    """
    if names is None:
        names = [f"wp_{i}" for i in range(len(points))]

    builder = NavigationBuilder()
    for (x, y), name in zip(points, names):
        builder.add_waypoint(name, x, y)

    return builder.build()
