#!/bin/bash
# ============================================================================
# SLAM Exploration - Two Process Architecture
# ============================================================================
# Process 1: Unitree SDK2 Robot Control (C++)
# Process 2: Livox SDK2 LiDAR Mapping (Python receiver)
# ============================================================================

set -e

# Configuration
INTERFACE="${1:-enP8p1s0}"
DURATION="${2:-30}"
AREA="${3:-small}"
OUTPUT_DIR="/home/unitree/sdk2_navigation/slam_maps"

# Paths
SCRIPT_DIR="/home/unitree/sdk2_navigation"
SLAM_EXPLORER="$SCRIPT_DIR/src/build/slam_exploration"

# Library paths
export LD_LIBRARY_PATH=/home/unitree/unitree_sdk2/thirdparty/lib/aarch64:$LD_LIBRARY_PATH

# Output file
mkdir -p "$OUTPUT_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MAP_FILE="$OUTPUT_DIR/slam_map_$TIMESTAMP.ply"

# PIDs for cleanup
LIVOX_PID=""
EXPLORER_PID=""

# Cleanup function
cleanup() {
    echo ""
    echo "=========================================="
    echo "Stopping processes..."
    echo "=========================================="

    # Stop Python LiDAR receiver
    if [ ! -z "$LIVOX_PID" ] && kill -0 $LIVOX_PID 2>/dev/null; then
        echo "Stopping LiDAR receiver..."
        kill -INT $LIVOX_PID 2>/dev/null || true
        wait $LIVOX_PID 2>/dev/null || true
    fi

    # Stop explorer
    if [ ! -z "$EXPLORER_PID" ] && kill -0 $EXPLORER_PID 2>/dev/null; then
        echo "Stopping robot..."
        kill -INT $EXPLORER_PID 2>/dev/null || true
        wait $EXPLORER_PID 2>/dev/null || true
    fi

    # Check for map file from Python script
    LATEST_MAP=$(ls -t $OUTPUT_DIR/livox_map_*.ply 2>/dev/null | head -1)
    if [ -n "$LATEST_MAP" ] && [ -f "$LATEST_MAP" ]; then
        POINTS=$(grep "element vertex" "$LATEST_MAP" | awk '{print $3}')
        echo ""
        echo "=========================================="
        echo "SLAM Exploration Complete"
        echo "=========================================="
        echo "Map saved: $LATEST_MAP"
        echo "Points: $POINTS"
    else
        echo ""
        echo "=========================================="
        echo "SLAM Exploration Complete"
        echo "=========================================="
        echo "Warning: No map file created"
    fi
}

trap cleanup EXIT INT TERM

# Banner
echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║          SLAM EXPLORATION - Two Process Mode               ║"
echo "╠════════════════════════════════════════════════════════════╣"
echo "║ Interface: $INTERFACE                                      "
echo "║ Duration:  ${DURATION}s                                             "
echo "║ Area:      $AREA                                               "
echo "║ Map dir:   $OUTPUT_DIR"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Start Python LiDAR receiver in background
echo "[1/2] Starting Livox LiDAR receiver (Python)..."
python3 "$SCRIPT_DIR/livox_mid360_slam.py" &
LIVOX_PID=$!
sleep 2

# Check if receiver started
if ! kill -0 $LIVOX_PID 2>/dev/null; then
    echo "Error: Failed to start LiDAR receiver"
    exit 1
fi
echo "      LiDAR receiver started (PID: $LIVOX_PID)"

# Wait for LiDAR connection
echo "[2/2] Waiting for LiDAR data..."
sleep 3

# Start robot exploration
echo ""
echo "=========================================="
echo "Starting Robot Exploration"
echo "=========================================="
echo ""

$SLAM_EXPLORER --interface "$INTERFACE" --duration "$DURATION" --area "$AREA" &
EXPLORER_PID=$!

# Wait for explorer to finish
wait $EXPLORER_PID 2>/dev/null || true

echo ""
echo "Exploration finished"
