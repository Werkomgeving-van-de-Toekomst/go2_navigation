#!/usr/bin/env python3
"""
SDK2 Navigation Module for Unitree Go2
========================================

Native Python navigation using unitree_sdk2py and DDS communication.

Based on:
- unitree_sdk2_python: https://github.com/unitreerobotics/unitree_sdk2_python
- Unitree Go2 Developer docs: https://support.unitree.com/home/en/developer

Example usage:
    from sdk2_navigation import Go2Navigator, Waypoint

    # Create navigator
    navigator = Go2Navigator(robot_ip="192.168.123.161")

    # Connect and start
    navigator.connect()
    navigator.start()

    # Navigate to goal
    navigator.navigate_to(x=2.0, y=1.5, yaw=0.0)

    # Or use waypoints
    navigator.add_waypoint(Waypoint("point1", 2.0, 0.0))
    navigator.add_waypoint(Waypoint("point2", 2.0, 2.0))
    navigator.run_waypoints()
"""

from sdk2_interface import (
    Go2SDK2Interface,
    RobotState,
    MotorCommand,
    SportModeState,
    BmsState,
)

from sdk2_navigator import (
    Go2Navigator,
    NavigationMode,
    NavigationStatus,
    Waypoint,
    NavigatorConfig,
)

from sdk2_high_level import (
    HighLevelNavigator,
    NavigationGoal,
    PathPlan,
)

from use_cases import (
    MotionRecorder,
    Keyframe,
    GestureRecognizer,
    Gesture,
    GesturePattern,
    Choreographer,
    DanceMove,
    BodyFollower,
    ObstacleCourse,
    ExplorationMode,
    CarryingMode,
    RobotGames,
    GameMode,
    SecurityPatrol,
    PatrolPoint,
    GimbalController,
    SDK2UseCases,
)

__version__ = "1.0.0"
__all__ = [
    # Low-level interface
    "Go2SDK2Interface",
    "RobotState",
    "MotorCommand",
    "SportModeState",
    "BmsState",
    # Navigator
    "Go2Navigator",
    "NavigationMode",
    "NavigationStatus",
    "Waypoint",
    "NavigatorConfig",
    # High-level API
    "HighLevelNavigator",
    "NavigationGoal",
    "PathPlan",
    # Use Cases
    "MotionRecorder",
    "Keyframe",
    "GestureRecognizer",
    "Gesture",
    "GesturePattern",
    "Choreographer",
    "DanceMove",
    "BodyFollower",
    "ObstacleCourse",
    "ExplorationMode",
    "CarryingMode",
    "RobotGames",
    "GameMode",
    "SecurityPatrol",
    "PatrolPoint",
    "GimbalController",
    "SDK2UseCases",
]


def check_sdk2_available() -> bool:
    """Check if unitree_sdk2py is available."""
    try:
        import unitree_sdk2py
        return True
    except ImportError:
        return False


def get_version() -> str:
    """Get module version."""
    return __version__
