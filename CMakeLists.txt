cmake_minimum_required(VERSION 3.10)
project(livox_sdk2_bridge)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Livox SDK2 paths
set(LIVOX_SDK_ROOT "/home/unitree/Livox-SDK2")
set(LIVOX_SDK_CORE_DIR "${LIVOX_SDK_ROOT}/sdk_core")
set(LIVOX_SDK_INCLUDE_DIR "${LIVOX_SDK_CORE_DIR}")
set(LIVOX_SDK_LIB_DIR "${LIVOX_SDK_ROOT}/build/sdk_core")

# Include directories
include_directories(
    ${LIVOX_SDK_INCLUDE_DIR}
    ${LIVOX_SDK_INCLUDE_DIR}/comm
    ${LIVOX_SDK_INCLUDE_DIR}/msg_handler
    ${LIVOX_SDK_INCLUDE_DIR}/driver
    ${LIVOX_SDK_ROOT}/3rdparty/rapidxml-1.13
    ${LIVOX_SDK_ROOT}/3rdparty/log/src
)

# Link directories
link_directories(${LIVOX_SDK_LIB_DIR})

# Source files
set(SOURCES
    livox_sdk2_bridge.cpp
)

# Create executable
add_executable(livox_sdk2_bridge ${SOURCES})

# Link libraries
target_link_libraries(livox_sdk2_bridge
    livox_lidar_sdk_shared
    pthread
    m
    dl
)
