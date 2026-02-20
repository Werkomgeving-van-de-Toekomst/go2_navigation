cmake_minimum_required(VERSION 3.10)
project(go2_exploration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# SDK Paths
# ============================================================================
# Unitree SDK2
set(UNITREE_SDK2_DIR "/home/unitree/unitree_sdk2")
set(UNITREE_SDK2_BUILD "${UNITREE_SDK2_DIR}/build")

# Livox SDK2
set(LIVOX_SDK2_DIR "/home/unitree/Livox-SDK2")
set(LIVOX_SDK2_CORE "${LIVOX_SDK2_DIR}/sdk_core")
set(LIVOX_SDK2_BUILD "${LIVOX_SDK2_DIR}/build/sdk_core")

# RealSense SDK
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE2 REQUIRED realsense2)

# ============================================================================
# Include directories
# ============================================================================
include_directories(
    # Unitree SDK2
    ${UNITREE_SDK2_DIR}/include
    ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
    ${UNITREE_SDK2_DIR}/thirdparty/include/ddsc
    # Livox SDK2
    ${LIVOX_SDK2_CORE}
    ${LIVOX_SDK2_CORE}/comm
    ${LIVOX_SDK2_CORE}/msg_handler
    ${LIVOX_SDK2_CORE}/driver
    ${LIVOX_SDK2_DIR}/3rdparty/rapidxml-1.13
    ${LIVOX_SDK2_DIR}/3rdparty/log/src
    # RealSense
    ${REALSENSE2_INCLUDE_DIRS}
)

# ============================================================================
# Link directories
# ============================================================================
link_directories(
    ${UNITREE_SDK2_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}
    ${UNITREE_SDK2_BUILD}/thirdparty/dds/cyclonedds/src/core
    ${UNITREE_SDK2_BUILD}/thirdparty/dds/cyclonedds-cxx/src/core/ddscxx
    ${LIVOX_SDK2_BUILD}
    ${REALSENSE2_LIBRARY_DIRS}
)

# ============================================================================
# Executables
# ============================================================================

# Simple SLAM Explorer (robot only, with RealSense camera support)
add_executable(slam_exploration slam_exploration.cpp)
target_link_libraries(slam_exploration
    unitree_sdk2
    ddscxx
    ddsc
    ${REALSENSE2_LIBRARIES}
    pthread
)

# Full SLAM Explorer (robot + Livox LiDAR mapping)
add_executable(slam_exploration_full slam_exploration_full.cpp)
target_link_libraries(slam_exploration_full
    # Unitree SDK2
    unitree_sdk2
    ddscxx
    ddsc
    # Livox SDK2
    livox_lidar_sdk_shared
    # System
    pthread
    m
    dl
)

# Damping mode - put robot in relaxed state
add_executable(damping_mode damping_mode.cpp)
target_link_libraries(damping_mode
    unitree_sdk2
    ddscxx
    ddsc
    pthread
)
