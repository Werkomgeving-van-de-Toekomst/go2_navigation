#!/usr/bin/env python3
"""
SDK2 Interface for Unitree Go2
==============================

Low-level interface using unitree_sdk2py for DDS communication with the Go2 robot.

Topics (DDS):
- rt/lowstate: Low-level motor state (12 DOF joints, IMU, foot force)
- rt/lowcmd: Low-level motor commands
- rt/sportmodestate: High-level robot state (position, velocity, mode)
- rt/lidarstate: LiDAR state (if SLAM is running)
- rt/heightmap: Height map from SLAM (if SLAM is running)
- api/sport/request: High-level API requests

Based on:
- https://github.com/unitreerobotics/unitree_sdk2_python
- https://github.com/unitreerobotics/unitree_ros2
"""

import sys
import os
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable, List, Tuple, Dict, Any
from enum import Enum
import numpy as np

# Add unitree_sdk2_python to path if not already installed
SDK2_PATH = "/home/unitree/unitree_sdk2_python"
if os.path.exists(SDK2_PATH) and SDK2_PATH not in sys.path:
    sys.path.insert(0, SDK2_PATH)


class RobotMode(Enum):
    """Robot operation modes."""
    UNKNOWN = 0
    IDLE = 1
    BALANCE_STAND = 2
    STAND = 2  # Alias
    POS_HOLD = 2
    MOVE = 3
    LOCOMOTION = 3
    CLIMB = 4
    DOWN_STAIR = 5
    JUMP = 6


class GaitType(Enum):
    """Gait types for locomotion."""
    IDLE = 0
    TROT = 1
    RUN = 2
    CLIMB = 3
    DOWN_STAIR = 4


@dataclass
class BmsState:
    """Battery management system state."""
    soc: float = 0.0          # State of charge (0-100%)
    voltage: float = 0.0      # Voltage (V)
    current: float = 0.0      # Current (A)
    capacity: float = 0.0     # Capacity (Ah)
    cycles: int = 0           # Charge cycles
    temperature: float = 0.0  # Temperature (°C)


@dataclass
class IMUState:
    """IMU sensor state."""
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # w, x, y, z
    gyroscope: np.ndarray = field(default_factory=lambda: np.zeros(3))  # rad/s
    accelerometer: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s^2
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll, pitch, yaw (rad)


@dataclass
class MotorState:
    """Single motor state."""
    q: float = 0.0      # Position (rad)
    dq: float = 0.0     # Velocity (rad/s)
    ddq: float = 0.0    # Acceleration (rad/s^2)
    tau_est: float = 0.0  # Estimated torque (Nm)
    temperature: float = 0.0  # Temperature (°C)


@dataclass
class RobotState:
    """
    Complete robot state.

    Combines low-level motor state with high-level position/velocity state.
    """
    # Timestamp
    timestamp: float = 0.0
    tick: int = 0

    # Joint state (12 DOF: 3 joints per leg x 4 legs)
    # Order: FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
    #        RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
    joint_pos: np.ndarray = field(default_factory=lambda: np.zeros(12))
    joint_vel: np.ndarray = field(default_factory=lambda: np.zeros(12))
    joint_torque: np.ndarray = field(default_factory=lambda: np.zeros(12))

    # IMU
    imu: IMUState = field(default_factory=IMUState)

    # Base state (from high-level)
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # x, y, z
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # vx, vy, vz
    orientation: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll, pitch, yaw
    yaw_speed: float = 0.0

    # Robot mode
    mode: RobotMode = RobotMode.UNKNOWN
    gait_type: GaitType = GaitType.IDLE

    # Foot contact (4 feet: FL, FR, RL, RR)
    foot_force: np.ndarray = field(default_factory=lambda: np.zeros(4))
    foot_contact: np.ndarray = field(default_factory=lambda: np.zeros(4))

    # Battery
    bms: BmsState = field(default_factory=BmsState)

    # Range sensors (obstacle detection)
    range_obstacle: np.ndarray = field(default_factory=lambda: np.array([0.4, 0.4, 0.4, 0.4]))  # front, left, right, back

    def get_position_2d(self) -> Tuple[float, float]:
        """Get x, y position."""
        return float(self.position[0]), float(self.position[1])

    def get_yaw(self) -> float:
        """Get yaw angle."""
        return float(self.orientation[2])

    def is_moving(self) -> bool:
        """Check if robot is moving."""
        return np.linalg.norm(self.velocity) > 0.01


@dataclass
class MotorCommand:
    """Motor command for one joint."""
    mode: int = 0x01  # 0x01 = FOC mode
    q: float = 0.0    # Target position (rad)
    dq: float = 0.0   # Target velocity (rad/s)
    kp: float = 0.0   # Position gain
    kd: float = 0.0   # Velocity gain
    tau: float = 0.0  # Feedforward torque (Nm)


@dataclass
class VelocityCommand:
    """Velocity command for high-level control."""
    vx: float = 0.0   # Forward velocity (m/s)
    vy: float = 0.0   # Lateral velocity (m/s)
    vyaw: float = 0.0  # Yaw rate (rad/s)


@dataclass
class SportModeState:
    """
    Sport mode state from rt/sportmodestate topic.

    This is the high-level state used for navigation.
    """
    timestamp: float = 0.0
    error_code: int = 0

    # IMU
    imu: IMUState = field(default_factory=IMUState)

    # Mode
    mode: int = 0
    progress: float = 0.0
    gait_type: int = 0

    # Position and velocity
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw_speed: float = 0.0

    # Body height
    foot_raise_height: float = 0.0
    body_height: float = 0.0

    # Obstacle detection
    range_obstacle: np.ndarray = field(default_factory=lambda: np.zeros(4))

    # Foot force and position
    foot_force: np.ndarray = field(default_factory=lambda: np.zeros(4))
    foot_position_body: np.ndarray = field(default_factory=lambda: np.zeros(12))
    foot_speed_body: np.ndarray = field(default_factory=lambda: np.zeros(12))


class Go2SDK2Interface:
    """
    Interface for Unitree Go2 using unitree_sdk2py.

    Handles DDS communication for both low-level and high-level control.

    Usage:
        interface = Go2SDK2Interface(network_interface="enP8p1s0")
        interface.connect()

        # Subscribe to state updates
        interface.add_state_callback(lambda state: print(state.position))

        # Send commands
        interface.send_velocity(0.2, 0.0, 0.0)  # Forward 0.2 m/s

        interface.disconnect()
    """

    def __init__(
        self,
        network_interface: str = "enP8p1s0",
        domain_id: int = 0,
        robot_ip: str = "192.168.123.161",
        speaker_volume: int = 0,  # 0 = muted, 1-100 = volume level
    ):
        self.network_interface = network_interface
        self.domain_id = domain_id
        self.robot_ip = robot_ip
        self._speaker_volume = speaker_volume

        # State
        self._state = RobotState()
        self._sport_state = SportModeState()
        self._state_lock = threading.Lock()
        self._state_received = threading.Event()
        self._sport_state_received = threading.Event()

        # Callbacks
        self._state_callbacks: List[Callable[[RobotState], None]] = []
        self._sport_callbacks: List[Callable[[SportModeState], None]] = []
        self._slam_callbacks: List[Callable] = []

        # DDS components
        self._channel_factory = None
        self._low_state_sub = None
        self._low_cmd_pub = None
        self._sport_state_sub = None
        self._sport_client = None
        self._obstacle_avoid_client = None
        self._vui_client = None
        self._lidar_sub = None
        self._height_map_sub = None

        # SLAM state
        self._lidar_connected = False
        self._slam_running = False
        self._lidar_state = {
            'timestamp': 0.0,
            'cloud_frequency': 0.0,
            'error_state': 0,
        }
        self._height_map_data = None

        # Connection status
        self._connected = False
        self._running = False
        self._obstacle_avoid_enabled = False

        # Check SDK2 availability
        self._sdk2_available = self._check_sdk2()

    def _check_sdk2(self) -> bool:
        """Check if unitree_sdk2py is available."""
        try:
            import unitree_sdk2py
            return True
        except ImportError:
            print("WARNING: unitree_sdk2py not found. Install with:")
            print("  pip install unitree-sdk2-python")
            return False

    def connect(self) -> bool:
        """
        Establish DDS connection with the robot.

        Returns:
            True if connection successful
        """
        if not self._sdk2_available:
            print("ERROR: unitree_sdk2py not available")
            return False

        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient

            # Initialize channel factory using official API
            if self.network_interface:
                ChannelFactoryInitialize(self.domain_id, self.network_interface)
            else:
                ChannelFactoryInitialize(self.domain_id)

            # Create sport client for high-level control
            self._sport_client = SportClient()
            self._sport_client.Init()

            # Create subscribers
            self._create_subscribers()

            # Set speaker volume
            self._set_speaker_volume()

            self._connected = True
            self._running = True
            print(f"Connected to Go2 on {self.network_interface}")
            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _create_subscribers(self):
        """Create DDS subscribers for state topics."""
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

            # Low state subscriber (motor states, IMU, foot force)
            self._low_state_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._low_state_sub.Init(self._on_low_state, 10)

            # Sport mode state subscriber (high-level state)
            self._sport_state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            self._sport_state_sub.Init(self._on_sport_state, 10)

            print("DDS subscribers created")

        except Exception as e:
            print(f"Failed to create subscribers: {e}")

    def _on_low_state(self, msg):
        """Callback for low state messages."""
        try:
            with self._state_lock:
                self._state.timestamp = time.time()
                self._state.tick = msg.tick

                # Joint states (12 motors)
                for i in range(12):
                    self._state.joint_pos[i] = msg.motor_state[i].q
                    self._state.joint_vel[i] = msg.motor_state[i].dq
                    self._state.joint_torque[i] = msg.motor_state[i].tau_est

                # IMU
                self._state.imu.quaternion = np.array(msg.imu_state.quaternion)
                self._state.imu.gyroscope = np.array(msg.imu_state.gyroscope)
                self._state.imu.accelerometer = np.array(msg.imu_state.accelerometer)
                self._state.imu.rpy = np.array(msg.imu_state.rpy)

                # Foot force
                self._state.foot_force = np.array(msg.foot_force[:4], dtype=float)

                # Update foot contact (threshold at 20N)
                self._state.foot_contact = (self._state.foot_force > 20.0).astype(float)

                # BMS state
                self._state.bms.soc = float(msg.bms_state.soc)
                self._state.bms.current = float(msg.bms_state.current) / 1000.0  # mA to A
                self._state.bms.cycles = int(msg.bms_state.cycle)
                # Temperature from NTC sensors
                if len(msg.bms_state.mcu_ntc) >= 2:
                    self._state.bms.temperature = float(msg.bms_state.mcu_ntc[0])
                # Calculate voltage from cell voltages (sum of all cells, in mV)
                if len(msg.bms_state.cell_vol) >= 15:
                    total_mv = sum(int(v) for v in msg.bms_state.cell_vol)
                    self._state.bms.voltage = total_mv / 1000.0  # mV to V

            self._state_received.set()

            # Notify callbacks
            for callback in self._state_callbacks:
                try:
                    callback(self.get_state())
                except Exception:
                    pass

        except Exception as e:
            print(f"Error in low state callback: {e}")

    def _on_sport_state(self, msg):
        """Callback for sport mode state messages."""
        try:
            with self._state_lock:
                self._sport_state.timestamp = time.time()
                self._sport_state.error_code = msg.error_code

                # IMU from sport state
                self._sport_state.imu.quaternion = np.array(msg.imu_state.quaternion)
                self._sport_state.imu.gyroscope = np.array(msg.imu_state.gyroscope)
                self._sport_state.imu.accelerometer = np.array(msg.imu_state.accelerometer)
                self._sport_state.imu.rpy = np.array(msg.imu_state.rpy)

                # Mode and position
                self._sport_state.mode = msg.mode
                self._sport_state.progress = msg.progress
                self._sport_state.gait_type = msg.gait_type

                # Position (x, y, z) in world frame
                self._sport_state.position = np.array([
                    float(msg.position[0]),
                    float(msg.position[1]),
                    float(msg.position[2])
                ])

                # Velocity (vx, vy, vz)
                self._sport_state.velocity = np.array([
                    float(msg.velocity[0]),
                    float(msg.velocity[1]),
                    float(msg.velocity[2])
                ])

                self._sport_state.yaw_speed = float(msg.yaw_speed)

                # Body info
                self._sport_state.foot_raise_height = float(msg.foot_raise_height)
                self._sport_state.body_height = float(msg.body_height)

                # Obstacle ranges
                self._sport_state.range_obstacle = np.array([
                    float(msg.range_obstacle[0]),
                    float(msg.range_obstacle[1]),
                    float(msg.range_obstacle[2]),
                    float(msg.range_obstacle[3])
                ])

                # Foot force and positions
                self._sport_state.foot_force = np.array(msg.foot_force[:4], dtype=float)

                # Update main state with high-level info
                self._state.position = self._sport_state.position.copy()
                self._state.velocity = self._sport_state.velocity.copy()
                self._state.orientation = self._sport_state.imu.rpy.copy()
                self._state.yaw_speed = self._sport_state.yaw_speed
                self._state.mode = RobotMode(msg.mode)
                self._state.gait_type = GaitType(msg.gait_type)
                self._state.range_obstacle = self._sport_state.range_obstacle.copy()

            self._sport_state_received.set()

            # Notify callbacks
            for callback in self._sport_callbacks:
                try:
                    callback(self.get_sport_state())
                except Exception:
                    pass

        except Exception as e:
            print(f"Error in sport state callback: {e}")

    def get_state(self) -> RobotState:
        """Get current robot state (thread-safe copy)."""
        with self._state_lock:
            return RobotState(
                timestamp=self._state.timestamp,
                tick=self._state.tick,
                joint_pos=self._state.joint_pos.copy(),
                joint_vel=self._state.joint_vel.copy(),
                joint_torque=self._state.joint_torque.copy(),
                imu=IMUState(
                    quaternion=self._state.imu.quaternion.copy(),
                    gyroscope=self._state.imu.gyroscope.copy(),
                    accelerometer=self._state.imu.accelerometer.copy(),
                    rpy=self._state.imu.rpy.copy(),
                ),
                position=self._state.position.copy(),
                velocity=self._state.velocity.copy(),
                orientation=self._state.orientation.copy(),
                yaw_speed=self._state.yaw_speed,
                mode=self._state.mode,
                gait_type=self._state.gait_type,
                foot_force=self._state.foot_force.copy(),
                foot_contact=self._state.foot_contact.copy(),
                bms=BmsState(
                    soc=self._state.bms.soc,
                    voltage=self._state.bms.voltage,
                    current=self._state.bms.current,
                    capacity=self._state.bms.capacity,
                    cycles=self._state.bms.cycles,
                    temperature=self._state.bms.temperature,
                ),
                range_obstacle=self._state.range_obstacle.copy(),
            )

    def get_sport_state(self) -> SportModeState:
        """Get sport mode state."""
        with self._state_lock:
            return SportModeState(
                timestamp=self._sport_state.timestamp,
                error_code=self._sport_state.error_code,
                imu=IMUState(
                    quaternion=self._sport_state.imu.quaternion.copy(),
                    gyroscope=self._sport_state.imu.gyroscope.copy(),
                    accelerometer=self._sport_state.imu.accelerometer.copy(),
                    rpy=self._sport_state.imu.rpy.copy(),
                ),
                mode=self._sport_state.mode,
                progress=self._sport_state.progress,
                gait_type=self._sport_state.gait_type,
                position=self._sport_state.position.copy(),
                velocity=self._sport_state.velocity.copy(),
                yaw_speed=self._sport_state.yaw_speed,
                foot_raise_height=self._sport_state.foot_raise_height,
                body_height=self._sport_state.body_height,
                range_obstacle=self._sport_state.range_obstacle.copy(),
                foot_force=self._sport_state.foot_force.copy(),
                foot_position_body=self._sport_state.foot_position_body.copy(),
                foot_speed_body=self._sport_state.foot_speed_body.copy(),
            )

    def wait_for_state(self, timeout: float = 5.0) -> bool:
        """Wait for first state message."""
        return self._state_received.wait(timeout)

    def add_state_callback(self, callback: Callable[[RobotState], None]):
        """Register callback for state updates."""
        self._state_callbacks.append(callback)

    def add_sport_callback(self, callback: Callable[[SportModeState], None]):
        """Register callback for sport state updates."""
        self._sport_callbacks.append(callback)

    def send_velocity(self, vx: float, vy: float, vyaw: float) -> bool:
        """
        Send velocity command using high-level API.

        Args:
            vx: Forward velocity (m/s), typically -1.0 to 1.0
            vy: Lateral velocity (m/s), typically -1.0 to 1.0
            vyaw: Yaw rate (rad/s), typically -2.0 to 2.0

        Returns:
            True if command sent successfully
        """
        if not self._connected or self._sport_client is None:
            print("Not connected")
            return False

        try:
            self._sport_client.Move(vx, vy, vyaw)
            return True
        except Exception as e:
            print(f"Failed to send velocity command: {e}")
            return False

    def send_velocity_command(self, cmd: VelocityCommand) -> bool:
        """Send velocity command from VelocityCommand object."""
        return self.send_velocity(cmd.vx, cmd.vy, cmd.vyaw)

    def send_velocity_auto(self, vx: float, vy: float, vyaw: float) -> bool:
        """
        Send velocity command (simple version, no auto OA).

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            vyaw: Yaw rate (rad/s)

        Returns:
            True if command sent successfully
        """
        return self.send_velocity(vx, vy, vyaw)

    def stop(self) -> bool:
        """Stop robot movement."""
        if self._sport_client is not None:
            try:
                self._sport_client.StopMove()
                return True
            except Exception as e:
                print(f"Failed to stop: {e}")
        return False

    def stand_up(self) -> bool:
        """Command robot to stand up."""
        if self._sport_client is not None:
            try:
                self._sport_client.StandUp()
                return True
            except Exception as e:
                print(f"Failed to stand up: {e}")
        return False

    def sit_down(self) -> bool:
        """Command robot to sit down."""
        if self._sport_client is not None:
            try:
                self._sport_client.StandDown()
                return True
            except Exception as e:
                print(f"Failed to sit down: {e}")
        return False

    def damping(self) -> bool:
        """Switch to damping mode (passive)."""
        if self._sport_client is not None:
            try:
                self._sport_client.Damp()
                return True
            except Exception as e:
                print(f"Failed to switch to damping: {e}")
        return False

    def balance_stand(self) -> bool:
        """Switch to balance stand mode."""
        if self._sport_client is not None:
            try:
                self._sport_client.BalanceStand()
                return True
            except Exception as e:
                print(f"Failed to switch to balance stand: {e}")
        return False

    def switch_to_low_level(self) -> bool:
        """
        Switch to low-level control mode.

        WARNING: This disables high-level control. Use with caution!
        """
        if not self._connected:
            return False

        try:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_

            # Create low command publisher
            self._low_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            self._low_cmd_pub.Init()

            # Switch mode via sport client first
            self.damping()
            time.sleep(0.5)

            print("Switched to low-level control mode")
            return True

        except Exception as e:
            print(f"Failed to switch to low-level: {e}")
            return False

    def send_low_command(self, commands: List[MotorCommand]) -> bool:
        """
        Send low-level motor commands.

        Only works after switch_to_low_level().

        Args:
            commands: List of 12 MotorCommand objects

        Returns:
            True if command sent successfully
        """
        if self._low_cmd_pub is None:
            print("Not in low-level mode")
            return False

        if len(commands) != 12:
            print(f"Need 12 motor commands, got {len(commands)}")
            return False

        try:
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_

            cmd = unitree_go_msg_dds__LowCmd_()
            cmd.head = [0xFE, 0xEF]
            cmd.level_flag = 0xFF  # Low-level mode
            cmd.gpio = 0

            for i, mc in enumerate(commands):
                cmd.motor_cmd[i].mode = mc.mode
                cmd.motor_cmd[i].q = mc.q
                cmd.motor_cmd[i].dq = mc.dq
                cmd.motor_cmd[i].kp = mc.kp
                cmd.motor_cmd[i].kd = mc.kd
                cmd.motor_cmd[i].tau = mc.tau

            self._low_cmd_pub.Write(cmd)
            return True

        except Exception as e:
            print(f"Failed to send low-level command: {e}")
            return False

    # ========================================================================
    # Obstacle Avoidance Methods
    # ========================================================================

    def _init_obstacle_avoidance(self):
        """Initialize ObstaclesAvoidClient."""
        try:
            from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
            self._obstacle_avoid_client = ObstaclesAvoidClient()
            self._obstacle_avoid_client.SetTimeout(10.0)
            self._obstacle_avoid_client.Init()
            print("ObstacleAvoidClient initialized")
        except Exception as e:
            print(f"Failed to initialize ObstacleAvoidClient: {e}")

    def enable_obstacle_avoidance(self) -> bool:
        """Enable built-in obstacle avoidance."""
        if self._obstacle_avoid_client is None:
            self._init_obstacle_avoidance()
            if self._obstacle_avoid_client is None:
                return False

        try:
            # Check current status
            code, enabled = self._obstacle_avoid_client.SwitchGet()
            if not enabled:
                # Turn on
                for _ in range(50):
                    code, enabled = self._obstacle_avoid_client.SwitchGet()
                    if enabled:
                        break
                    self._obstacle_avoid_client.SwitchSet(True)
                    time.sleep(0.1)

            self._obstacle_avoid_enabled = True
            return enabled
        except Exception as e:
            print(f"Failed to enable obstacle avoidance: {e}")
            return False

    def disable_obstacle_avoidance(self) -> bool:
        """Disable built-in obstacle avoidance."""
        if self._obstacle_avoid_client is None:
            return False

        try:
            self._obstacle_avoid_client.SwitchSet(False)
            self._obstacle_avoid_enabled = False
            return True
        except Exception as e:
            print(f"Failed to disable obstacle avoidance: {e}")
            return False

    def is_obstacle_avoidance_enabled(self) -> bool:
        """Check if obstacle avoidance is enabled."""
        if self._obstacle_avoid_client is None:
            return False
        try:
            code, enabled = self._obstacle_avoid_client.SwitchGet()
            return enabled
        except:
            return False

    def send_velocity_with_avoidance(self, vx: float, vy: float, vyaw: float) -> bool:
        """
        Send velocity command with built-in obstacle avoidance.

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            vyaw: Yaw rate (rad/s)

        Returns:
            True if command sent successfully
        """
        if self._obstacle_avoid_client is None:
            self._init_obstacle_avoidance()
            if self._obstacle_avoid_client is None:
                return False

        # Enable API control (required for obstacle avoidance commands)
        try:
            self._obstacle_avoid_client.UseRemoteCommandFromApi(True)
            time.sleep(0.05)  # Small delay to ensure API control is active
        except Exception as e:
            # If API control fails, fall back to normal velocity
            print(f"API control failed: {e}, falling back to normal velocity")
            return self.send_velocity(vx, vy, vyaw)

        try:
            # The Move API might require obstacle avoidance to be "on" via Switch
            code, enabled = self._obstacle_avoid_client.SwitchGet()
            if not enabled:
                # OA is off, use normal SportClient instead
                return self.send_velocity(vx, vy, vyaw)

            # OA is on, use ObstaclesAvoidClient
            self._obstacle_avoid_client.Move(vx, vy, vyaw)
            return True
        except Exception as e:
            print(f"Failed to send velocity with avoidance: {e}")
            # Fall back to normal velocity
            return self.send_velocity(vx, vy, vyaw)

    def stop_with_avoidance(self) -> bool:
        """Stop movement (with obstacle avoidance)."""
        return self.send_velocity_with_avoidance(0.0, 0.0, 0.0)

    def move_to_relative(self, x: float, y: float, yaw: float) -> bool:
        """
        Move to relative position with obstacle avoidance.

        Args:
            x: Forward distance (m)
            y: Lateral distance (m)
            yaw: Yaw angle (rad)

        Returns:
            True if command sent successfully
        """
        if self._obstacle_avoid_client is None:
            return False

        # Enable API control
        self._obstacle_avoid_client.UseRemoteCommandFromApi(True)
        time.sleep(0.1)

        try:
            self._obstacle_avoid_client.MoveToIncrementPosition(x, y, yaw)
            return True
        except Exception as e:
            print(f"Failed to move to relative position: {e}")
            return False

    def move_to_absolute(self, x: float, y: float, yaw: float) -> bool:
        """
        Move to absolute position with obstacle avoidance.

        Args:
            x: X position (m)
            y: Y position (m)
            yaw: Yaw angle (rad)

        Returns:
            True if command sent successfully
        """
        if self._obstacle_avoid_client is None:
            return False

        # Enable API control
        self._obstacle_avoid_client.UseRemoteCommandFromApi(True)
        time.sleep(0.1)

        try:
            self._obstacle_avoid_client.MoveToAbsolutePosition(x, y, yaw)
            return True
        except Exception as e:
            print(f"Failed to move to absolute position: {e}")
            return False

    def release_api_control(self):
        """Release API control back to remote controller."""
        if self._obstacle_avoid_client is not None:
            try:
                self._obstacle_avoid_client.UseRemoteCommandFromApi(False)
            except:
                pass

    # ========================================================================
    # VUI / Speaker Control Methods
    # ========================================================================

    def _init_vui_client(self):
        """Initialize VuiClient for speaker and LED control."""
        try:
            from unitree_sdk2py.go2.vui.vui_client import VuiClient
            self._vui_client = VuiClient()
            self._vui_client.SetTimeout(10.0)
            self._vui_client.Init()
            return True
        except Exception as e:
            print(f"Failed to initialize VuiClient: {e}")
            return False

    def _set_speaker_volume(self):
        """Set speaker volume (called during connect)."""
        if self._speaker_volume < 0:
            self._speaker_volume = 0
        elif self._speaker_volume > 100:
            self._speaker_volume = 100

        if self._speaker_volume == 0:
            # Mute speaker
            if self._set_speaker_muted(True):
                print("Speaker muted 🔇")
        else:
            # Set volume
            if self._set_volume(self._speaker_volume):
                print(f"Speaker volume set to {self._speaker_volume}% 🔊")

    def _set_speaker_muted(self, muted: bool) -> bool:
        """Set speaker muted state."""
        if self._vui_client is None:
            if not self._init_vui_client():
                return False

        try:
            # enable: 0 = off (muted), 1 = on (unmuted)
            result = self._vui_client.SetSwitch(0 if muted else 1)
            return result == 0
        except Exception as e:
            print(f"Failed to set speaker mute: {e}")
            return False

    def set_speaker_muted(self, muted: bool) -> bool:
        """
        Mute or unmute the robot's speaker.

        Args:
            muted: True to mute, False to unmute

        Returns:
            True if successful
        """
        return self._set_speaker_muted(muted)

    def set_speaker_volume(self, volume: int) -> bool:
        """
        Set speaker volume level.

        Args:
            volume: Volume level (0-100, where 0 mutes the speaker)

        Returns:
            True if successful
        """
        return self._set_volume(volume)

    def _set_volume(self, volume: int) -> bool:
        """Internal method to set volume."""
        if self._vui_client is None:
            if not self._init_vui_client():
                return False

        try:
            result = self._vui_client.SetVolume(volume)
            if result == 0:
                # If volume > 0, make sure speaker is enabled
                if volume > 0:
                    self._vui_client.SetSwitch(1)
                return True
            return False
        except Exception as e:
            print(f"Failed to set volume: {e}")
            return False

    def get_speaker_volume(self) -> Optional[int]:
        """
        Get current speaker volume.

        Returns:
            Volume level (0-100) or None if failed
        """
        if self._vui_client is None:
            if not self._init_vui_client():
                return None

        try:
            code, volume = self._vui_client.GetVolume()
            if code == 0:
                return volume
            return None
        except Exception as e:
            print(f"Failed to get volume: {e}")
            return None

    def get_speaker_muted(self) -> Optional[bool]:
        """
        Get speaker mute state.

        Returns:
            True if muted, False if not muted, None if failed
        """
        if self._vui_client is None:
            if not self._init_vui_client():
                return None

        try:
            code, enabled = self._vui_client.GetSwitch()
            if code == 0:
                return enabled == 0  # 0 = muted
            return None
        except Exception as e:
            print(f"Failed to get speaker state: {e}")
            return None

    def set_led_brightness(self, brightness: int) -> bool:
        """
        Set LED brightness.

        Args:
            brightness: Brightness level (0-100)

        Returns:
            True if successful
        """
        if self._vui_client is None:
            if not self._init_vui_client():
                return False

        try:
            result = self._vui_client.SetBrightness(brightness)
            return result == 0
        except Exception as e:
            print(f"Failed to set LED brightness: {e}")
            return False

    def get_led_brightness(self) -> Optional[int]:
        """
        Get LED brightness.

        Returns:
            Brightness level (0-100) or None if failed
        """
        if self._vui_client is None:
            if not self._init_vui_client():
                return None

        try:
            code, brightness = self._vui_client.GetBrightness()
            if code == 0:
                return brightness
            return None
        except Exception as e:
            print(f"Failed to get LED brightness: {e}")
            return None

    def set_led_color(self, red: int, green: int, blue: int) -> bool:
        """
        Set LED color using LowCmd.

        The Go2 has LED strips that can be controlled via the LowCmd led array.
        The 12-byte led array controls different LED zones.

        Args:
            red: Red component (0-255)
            green: Green component (0-255)
            blue: Blue component (0-255)

        Returns:
            True if successful
        """
        try:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmd_, BmsCmd_

            # Create publisher if it doesn't exist
            if not hasattr(self, '_led_cmd_pub'):
                self._led_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
                self._led_cmd_pub.Init()

            # Create default motor commands (all zeros, no control)
            # MotorCmd_ requires: mode, q, dq, tau, kp, kd, reserve[3]
            motor_cmd_list = []
            for _ in range(20):
                mc = MotorCmd_(
                    mode=0,      # No control
                    q=0.0,       # Position
                    dq=0.0,      # Velocity
                    tau=0.0,     # Torque
                    kp=0.0,      # Position gain
                    kd=0.0,      # Velocity gain
                    reserve=[0, 0, 0]  # Reserved (3 uint32)
                )
                motor_cmd_list.append(mc)

            # Create BMS command (off, reserve[3])
            bms_cmd = BmsCmd_(
                off=0,
                reserve=[0, 0, 0]
            )

            # Build LED array (12 bytes) - set all LEDs to the same color
            led_array = [0] * 12
            # Try a simple pattern: R,G,B repeated
            for i in range(4):
                led_array[i * 3] = red
                led_array[i * 3 + 1] = green
                led_array[i * 3 + 2] = blue

            # Create LowCmd
            cmd = unitree_go_msg_dds__LowCmd_()
            cmd.head = [0xFE, 0xEF]
            cmd.level_flag = 0xFF  # High-level mode flag
            cmd.frame_reserve = 0
            cmd.sn = [0, 0]
            cmd.version = [0, 1]
            cmd.bandwidth = 0
            cmd.motor_cmd = motor_cmd_list
            cmd.bms_cmd = bms_cmd
            cmd.wireless_remote = [0] * 40
            cmd.led = led_array
            cmd.fan = [0, 0]
            cmd.gpio = 0
            cmd.reserve = 0
            cmd.crc = 0

            self._led_cmd_pub.Write(cmd)
            return True

        except Exception as e:
            print(f"Failed to set LED color: {e}")
            return False

    def set_led_animation(self, animation_type: str) -> bool:
        """
        Set LED animation pattern.

        Args:
            animation_type: 'off', 'solid_white', 'solid_green', 'solid_blue'

        Returns:
            True if successful
        """
        # Predefined LED patterns for Go2 (RGB pattern: R,G,B,R,G,B,...)
        patterns = {
            'off': [0] * 12,
            'solid_white': [255, 255, 255] * 4,
            'solid_green': [0, 255, 0] * 4,
            'solid_blue': [0, 0, 255] * 4,
        }

        try:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmd_, BmsCmd_

            # Create publisher if it doesn't exist
            if not hasattr(self, '_led_cmd_pub'):
                self._led_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
                self._led_cmd_pub.Init()

            # Create default motor commands
            motor_cmd_list = []
            for _ in range(20):
                mc = MotorCmd_(
                    mode=0,
                    q=0.0,
                    dq=0.0,
                    tau=0.0,
                    kp=0.0,
                    kd=0.0,
                    reserve=[0, 0, 0]
                )
                motor_cmd_list.append(mc)

            # Create BMS command
            bms_cmd = BmsCmd_(
                off=0,
                reserve=[0, 0, 0]
            )

            # Get pattern
            led_array = patterns.get(animation_type, patterns['off'])

            # Create LowCmd
            cmd = unitree_go_msg_dds__LowCmd_()
            cmd.head = [0xFE, 0xEF]
            cmd.level_flag = 0xFF
            cmd.frame_reserve = 0
            cmd.sn = [0, 0]
            cmd.version = [0, 1]
            cmd.bandwidth = 0
            cmd.motor_cmd = motor_cmd_list
            cmd.bms_cmd = bms_cmd
            cmd.wireless_remote = [0] * 40
            cmd.led = led_array
            cmd.fan = [0, 0]
            cmd.gpio = 0
            cmd.reserve = 0
            cmd.crc = 0

            self._led_cmd_pub.Write(cmd)
            return True

        except Exception as e:
            print(f"Failed to set LED animation: {e}")
            return False

    def start_led_flash(self, color: str = 'green_blue') -> bool:
        """
        Start LED flashing animation in a background thread.

        Args:
            color: Color scheme ('green_blue', 'red', 'custom')

        Returns:
            True if started successfully
        """
        if hasattr(self, '_led_flash_thread') and self._led_flash_thread.is_alive():
            self._led_flashing = False
            self._led_flash_thread.join(timeout=1.0)

        self._led_flashing = True
        self._led_flash_color = color

        self._led_flash_thread = threading.Thread(target=self._led_flash_loop, daemon=True)
        self._led_flash_thread.start()
        return True

    def stop_led_flash(self):
        """Stop LED flashing animation."""
        if hasattr(self, '_led_flashing'):
            self._led_flashing = False

    def _led_flash_loop(self):
        """Internal LED flash loop."""
        colors = {
            'green': [0, 255, 0],
            'blue': [0, 0, 255],
            'green_blue': [0, 255, 255],  # Cyan
        }

        while getattr(self, '_led_flashing', False):
            color = colors.get(self._led_flash_color, [0, 255, 255])
            self.set_led_color(color[0], color[1], color[2])
            time.sleep(0.5)
            self.set_led_color(0, 0, 0)  # Off
            time.sleep(0.5)

    def disconnect(self):
        """Disconnect from robot."""
        self._running = False

        if self._sport_client is not None:
            try:
                self._sport_client.Damp()  # Safe mode before disconnect
            except:
                pass

        self._connected = False
        print("Disconnected from Go2")


# High-level API wrapper for common operations

class Go2HighLevelAPI:
    """
    High-level API wrapper for common Go2 operations.

    Simplified interface for basic robot control without dealing with DDS details.
    """

    def __init__(self, network_interface: str = "enP8p1s0", robot_ip: str = "192.168.123.161"):
        self.interface = Go2SDK2Interface(
            network_interface=network_interface,
            robot_ip=robot_ip
        )

    def connect(self) -> bool:
        """Connect to robot."""
        return self.interface.connect()

    def disconnect(self):
        """Disconnect from robot."""
        self.interface.disconnect()

    def is_connected(self) -> bool:
        """Check if connected."""
        return self.interface._connected

    def get_position(self) -> Tuple[float, float, float]:
        """Get current position (x, y, z)."""
        state = self.interface.get_state()
        return (
            float(state.position[0]),
            float(state.position[1]),
            float(state.position[2])
        )

    def get_pose(self) -> Tuple[float, float, float]:
        """Get current pose (x, y, yaw)."""
        state = self.interface.get_state()
        return (
            float(state.position[0]),
            float(state.position[1]),
            float(state.orientation[2])
        )

    def get_battery(self) -> float:
        """Get battery percentage."""
        state = self.interface.get_state()
        return state.bms.soc

    def is_moving(self) -> bool:
        """Check if robot is currently moving."""
        return self.interface.get_state().is_moving()

    def move(self, forward: float = 0.0, lateral: float = 0.0, turn: float = 0.0) -> bool:
        """
        Move robot.

        Args:
            forward: Forward velocity (m/s), -1.0 to 1.0
            lateral: Lateral velocity (m/s), -1.0 to 1.0
            turn: Turn rate (rad/s), -2.0 to 2.0

        Returns:
            True if command sent successfully
        """
        return self.interface.send_velocity(forward, lateral, turn)

    def stop(self) -> bool:
        """Stop robot."""
        return self.interface.stop()

    def stand(self) -> bool:
        """Make robot stand up."""
        return self.interface.stand_up()

    def sit(self) -> bool:
        """Make robot sit down."""
        return self.interface.sit_down()

    def add_position_callback(self, callback):
        """Add callback for position updates."""
        def wrapper(state):
            callback(state.position[0], state.position[1], state.orientation[2])
        self.interface.add_state_callback(wrapper)
