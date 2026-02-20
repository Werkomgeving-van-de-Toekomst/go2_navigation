#!/usr/bin/env python3
"""
SDK2 Navigation Configuration
==============================

Default configuration values and presets for different scenarios.
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class NetworkConfig:
    """Network configuration for DDS communication."""
    interface: str = "enP8p1s0"
    domain_id: int = 0
    robot_ip: str = "192.168.123.161"

    # DDS discovery
    enable_multicast: bool = True
    peer_addresses: list = None

    def __post_init__(self):
        if self.peer_addresses is None:
            self.peer_addresses = []


@dataclass
class VelocityLimits:
    """Velocity limits for safety."""
    max_linear: float = 0.5  # m/s
    max_angular: float = 1.0  # rad/s
    max_linear_accel: float = 1.0  # m/s^2
    max_angular_accel: float = 2.0  # rad/s^2


@dataclass
class SafetyConfig:
    """Safety parameters."""
    max_roll: float = math.radians(20)  # rad
    max_pitch: float = math.radians(20)  # rad
    min_battery: float = 10.0  # percent
    obstacle_distance: float = 0.4  # m
    obstacle_slowdown_distance: float = 0.7  # m


@dataclass
class NavigationConfig:
    """Complete navigation configuration."""
    network: NetworkConfig = None
    velocity: VelocityLimits = None
    safety: SafetyConfig = None

    # Navigation tolerances
    position_tolerance: float = 0.2  # m
    angle_tolerance: float = 0.15  # rad

    # Pure pursuit parameters
    lookahead_distance: float = 0.3  # m
    min_lookahead: float = 0.1  # m
    max_lookahead: float = 0.5  # m

    # Control rate
    control_rate: float = 20.0  # Hz

    # Features
    enable_obstacle_avoidance: bool = True

    def __post_init__(self):
        if self.network is None:
            self.network = NetworkConfig()
        if self.velocity is None:
            self.velocity = VelocityLimits()
        if self.safety is None:
            self.safety = SafetyConfig()


# Presets for different scenarios

class Presets:
    """Configuration presets for common scenarios."""

    @staticmethod
    def indoor() -> NavigationConfig:
        """Indoor navigation with moderate speeds."""
        return NavigationConfig(
            velocity=VelocityLimits(
                max_linear=0.3,
                max_angular=0.8,
                max_linear_accel=0.5,
                max_angular_accel=1.5
            ),
            safety=SafetyConfig(
                obstacle_distance=0.5,
                obstacle_slowdown_distance=0.8
            )
        )

    @staticmethod
    def outdoor() -> NavigationConfig:
        """Outdoor navigation with higher speeds."""
        return NavigationConfig(
            velocity=VelocityLimits(
                max_linear=1.0,
                max_angular=1.5,
                max_linear_accel=2.0,
                max_angular_accel=3.0
            ),
            safety=SafetyConfig(
                obstacle_distance=1.0,
                obstacle_slowdown_distance=2.0
            )
        )

    @staticmethod
    def careful() -> NavigationConfig:
        """Careful navigation for tight spaces."""
        return NavigationConfig(
            velocity=VelocityLimits(
                max_linear=0.15,
                max_angular=0.5,
                max_linear_accel=0.3,
                max_angular_accel=1.0
            ),
            safety=SafetyConfig(
                obstacle_distance=0.3,
                obstacle_slowdown_distance=0.5,
                max_roll=math.radians(15),
                max_pitch=math.radians(15)
            ),
            position_tolerance=0.1,
            enable_obstacle_avoidance=True
        )

    @staticmethod
    def fast() -> NavigationConfig:
        """Fast navigation for open spaces."""
        return NavigationConfig(
            velocity=VelocityLimits(
                max_linear=1.5,
                max_angular=2.0,
                max_linear_accel=3.0,
                max_angular_accel=4.0
            ),
            position_tolerance=0.3,
            lookahead_distance=0.5
        )

    @staticmethod
    def simulation() -> NavigationConfig:
        """Configuration for simulation/testing."""
        return NavigationConfig(
            network=NetworkConfig(
                interface="lo"  # Loopback for local testing
            ),
            velocity=VelocityLimits(
                max_linear=2.0,
                max_angular=3.0
            ),
            safety=SafetyConfig(
                min_battery=0.0,  # Ignore battery in sim
                obstacle_distance=0.2
            ),
            enable_obstacle_avoidance=False  # Sim may not have obstacles
        )


# Named locations preset

class LocationPresets:
    """Common location presets for home/office navigation."""

    # Home layout example
    HOME = {
        "charging_station": (0.0, 0.0, 0.0),
        "living_room": (2.0, 0.0, 0.0),
        "kitchen": (2.0, 2.0, math.pi/2),
        "bedroom": (0.0, 2.0, math.pi),
        "bathroom": (0.0, 3.0, -math.pi/2),
    }

    # Office layout example
    OFFICE = {
        "desk": (0.0, 0.0, 0.0),
        "meeting_room": (3.0, 0.0, 0.0),
        "printer": (2.0, 1.0, -math.pi/2),
        "kitchen": (0.0, 2.0, math.pi/2),
        "entrance": (0.0, -1.0, math.pi),
    }


def load_preset_locations(preset: str = "home") -> dict:
    """
    Load preset locations.

    Args:
        preset: "home", "office", or "custom"

    Returns:
        Dictionary of location names to (x, y, yaw) tuples
    """
    presets = {
        "home": LocationPresets.HOME,
        "office": LocationPresets.OFFICE,
    }
    return presets.get(preset.lower, {})


def create_config(
    preset: str = "indoor",
    network_interface: str = "enP8p1s0",
    robot_ip: str = "192.168.123.161"
) -> NavigationConfig:
    """
    Create configuration from preset.

    Args:
        preset: Configuration preset ("indoor", "outdoor", "careful", "fast", "simulation")
        network_interface: Network interface name
        robot_ip: Robot IP address

    Returns:
        NavigationConfig object
    """
    preset_map = {
        "indoor": Presets.indoor,
        "outdoor": Presets.outdoor,
        "careful": Presets.careful,
        "fast": Presets.fast,
        "simulation": Presets.simulation,
    }

    config_func = preset_map.get(preset.lower(), Presets.indoor)
    config = config_func()

    # Override network settings
    config.network.interface = network_interface
    config.network.robot_ip = robot_ip

    return config
