# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Autonomous navigation system for Unitree Go2 quadruped robot using SDK2. Implements SLAM-based exploration with RealSense camera obstacle detection.

**Status**: Working configuration is **Wander Mode** with C++ native RealSense camera integration.

## Build Commands

```bash
# Build C++ executables
cd /home/unitree/sdk2_navigation/src
mkdir -p build && cd build
cmake ..
make slam_exploration -j$(nproc)    # Main explorer (recommended)
make slam_exploration_full -j$(nproc)  # With Livox LiDAR
make damping_mode -j$(nproc)        # Robot damping control
```

## Running Exploration

```bash
# Set library path (required)
export LD_LIBRARY_PATH=/home/unitree/unitree_sdk2/thirdparty/lib/aarch64:$LD_LIBRARY_PATH

# Recommended: Wander Mode with camera (3 minutes)
./slam_exploration --interface enP8p1s0 --duration 180 --area small --camera

# Parameters
# --interface: Network interface (default: enP8p1s0)
# --duration: Exploration duration in seconds
# --area: Area size (small/medium/large)
# --camera: Enable RealSense camera obstacle detection
# --no-oa: Disable SDK obstacle avoidance
```

## Architecture

### Two-Layer System

**C++ Layer** (`src/`) - Performance-critical robot control:
- `slam_exploration.cpp`: Main explorer with RealSense camera thread
- `slam_exploration_full.cpp`: Full version with Livox LiDAR integration
- `damping_mode.cpp`: Puts robot in relaxed state

**Python Layer** (root) - High-level navigation API:
- `sdk2_interface.py`: Low-level DDS communication with robot
- `sdk2_navigator.py`: Waypoint navigation, pure pursuit path following
- `sdk2_high_level.py`: User-friendly API for common tasks
- `config.py`: Configuration presets (indoor, outdoor, careful, fast)

### Key Components in C++ Explorer

- **RealSenseCamera**: Thread-based depth camera (640x480 @ 30fps), divides view into left/front/right regions
- **SLAMExplorer**: Main control class with SportClient, ObstaclesAvoidClient, escape state machine
- **Escape State Machine**: 4-phase recovery (reverse -> turn -> sidestep -> forward)

### Communication

- Protocol: DDS (Data Distribution Service)
- Robot IP: 192.168.123.161 (configurable)
- Network interface: enP8p1s0 (default on robot)

## Important Constraints

### Navigation Modes
- **Wander Mode (recommended)**: Free exploration without fixed waypoints - more robust
- **Waypoint Mode**: Grid-based lawn-mower pattern - has issues with odometry drift
- **Return-to-Home**: Disabled - odometry drift makes it unreliable

### Known Limitations
- `state.position()` drifts over time - not reliable for precise navigation > 30 seconds
- Waypoint navigation doesn't work well due to odometry drift
- RealSense camera only detects front obstacles (no rear camera)
- No actual SLAM map building in current implementation

### Working Parameters
```cpp
// Obstacle thresholds
frontThreshold = 0.40f;  // 40cm - begin turning
sideThreshold = 0.25f;   // 25cm - for side regions

// Stuck detection
stuckCount > 100  // 2 seconds of < 1cm movement triggers escape

// Base velocity
targetVx = 0.40;  // 40 cm/s forward
```

## Dependencies

### C++ Libraries
- Unitree SDK2: `/home/unitree/unitree_sdk2`
- Livox SDK2: `/home/unitree/Livox-SDK2`
- RealSense SDK (via pkg-config)

### Python
- `unitree-sdk2-python>=0.2.0`
- `numpy>=1.20.0`

## Debug Tips

**Robot not moving**:
1. Check robot is in balance stand mode
2. Wait 3 seconds after StandUp()/BalanceStand()
3. Verify obstacle avoidance isn't blocking

**Camera issues**:
```bash
rs-enumerate-devices --short  # Check camera
fuser /dev/video* -k          # Kill blocking processes
```

**Crashes at startup**:
1. Initialize ChannelFactory BEFORE creating clients
2. Check LD_LIBRARY_PATH is set correctly
3. Verify network interface name matches robot's config
