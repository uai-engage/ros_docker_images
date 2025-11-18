#!/bin/bash
# Install micro-ROS Agent from source for ROS 2 Jazzy
# Run this inside the ROS 2 container

set -e

echo "=========================================="
echo "  micro-ROS Agent Installation Script"
echo "=========================================="
echo ""

# Check if already in workspace
if [ ! -d ~/ros2_ws ]; then
    echo "Error: ~/ros2_ws not found"
    exit 1
fi

cd ~/ros2_ws

# Check if already cloned
if [ -d src/micro-ROS-Agent ]; then
    echo "micro-ROS-Agent already cloned, skipping clone..."
else
    echo "Cloning micro-ROS Agent..."
    cd src
    git clone https://github.com/micro-ROS/micro-ROS-Agent.git
    cd ..
fi

echo ""
echo "Sourcing ROS environment..."
source /opt/ros/jazzy/setup.bash

echo ""
echo "Building micro-ROS Agent..."
echo "This will take 2-3 minutes..."
echo ""

# Build with ROS's Fast DDS
colcon build --packages-select micro_ros_agent \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "  ✅ Build successful!"
    echo "=========================================="
    echo ""
    echo "To use micro-ROS agent:"
    echo "  1. Source the workspace:"
    echo "     source ~/ros2_ws/install/setup.bash"
    echo ""
    echo "  2. Run the agent:"
    echo "     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
    echo ""
    echo "  3. Check for ArduPilot topics:"
    echo "     ros2 topic list"
    echo ""
    echo "To make permanent, add to ~/.bashrc:"
    echo "  echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "  ❌ Build failed!"
    echo "=========================================="
    echo ""
    echo "Check the error messages above."
    echo "Common issues:"
    echo "  - Missing dependencies (should not happen in this container)"
    echo "  - Fast DDS conflicts (see BUILD_FIX_NOTES.md)"
    echo ""
    exit 1
fi
