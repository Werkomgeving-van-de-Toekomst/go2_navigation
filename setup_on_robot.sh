#!/bin/bash
# Setup script for sdk2_navigation on Unitree Go2
# Adds unitree_sdk2_python to PYTHONPATH

echo "================================"
echo "SDK2 Navigation Setup"
echo "================================"

# Set PYTHONPATH permanently
SDK2_PATH="/home/unitree/unitree_sdk2_python"

# Add to .bashrc if not already there
BASHRC="/home/unitree/.bashrc"
if ! grep -q "$SDK2_PATH" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# SDK2 Navigation" >> "$BASHRC"
    echo "export PYTHONPATH=\$PYTHONPATH:$SDK2_PATH" >> "$BASHRC"
    echo "✅ Added to .bashrc"
else
    echo "✓ Already in .bashrc"
fi

# Add to current session
export PYTHONPATH=$PYTHONPATH:$SDK2_PATH

echo ""
echo "Testing import..."
python3 -c "from unitree_sdk2py.core.channel import ChannelFactory; print('✅ unitree_sdk2py imported successfully')"

echo ""
echo "You can now run:"
echo "  cd /home/unitree/sdk2_navigation"
echo "  python3 example_usage.py --check"
