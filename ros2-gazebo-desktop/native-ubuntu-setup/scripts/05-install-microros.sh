#!/bin/bash
# Install micro-ROS Agent for PX4 ↔ ROS 2 Communication
# Builds micro-ROS agent from source

set -e

echo "==========================================="
echo "  Installing micro-ROS Agent"
echo "==========================================="
echo ""
echo "This will:"
echo "  - Create ~/px4_workspace/micro_ros_ws"
echo "  - Clone micro-ROS agent (Jazzy branch)"
echo "  - Build with colcon"
echo ""
echo "Estimated time: 5 minutes"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled"
    exit 1
fi

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "❌ ROS 2 Jazzy not found!"
    echo "   Please run ./01-install-ros2-jazzy.sh first"
    exit 1
fi

PX4_WORKSPACE=~/px4_workspace
MICROROS_WS=$PX4_WORKSPACE/micro_ros_ws

echo ""
echo "Step 1/3: Creating micro-ROS workspace..."
mkdir -p $MICROROS_WS/src
cd $MICROROS_WS

if [ -d "src/micro-ROS-Agent" ]; then
    echo "⚠️  micro-ROS-Agent already exists"
    read -p "Remove and re-clone? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf src/micro-ROS-Agent
    fi
fi

if [ ! -d "src/micro-ROS-Agent" ]; then
    echo ""
    echo "Step 2/3: Cloning micro-ROS-Agent (Jazzy branch)..."
    cd src
    git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b jazzy
    cd ..
fi

echo ""
echo "Step 3/3: Building micro-ROS agent..."
echo "   (This may take 3-5 minutes...)"
colcon build --cmake-args -DUAGENT_BUILD_EXECUTABLE=OFF

echo ""
echo "==========================================="
echo "  ✅ micro-ROS Agent Installation Complete!"
echo "==========================================="
echo ""
echo "Installed at:"
echo "  $MICROROS_WS"
echo ""
echo "To use micro-ROS agent, source the workspace:"
echo "  source $MICROROS_WS/install/setup.bash"
echo ""
echo "Run micro-ROS agent (after starting PX4):"
echo "  ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888"
echo ""
echo "Add to ~/.bashrc for convenience:"
echo "  echo 'source $MICROROS_WS/install/setup.bash' >> ~/.bashrc"
echo "  echo 'alias microros=\"ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888\"' >> ~/.bashrc"
echo ""
echo "==========================================="
echo "  🎉 ALL INSTALLATIONS COMPLETE!"
echo "==========================================="
echo ""
echo "Summary:"
echo "  ✅ ROS 2 Jazzy"
echo "  ✅ Gazebo Harmonic"
echo "  ✅ PX4 Autopilot v1.16.0"
echo "  ✅ micro-ROS Agent"
echo ""
echo "Next steps:"
echo "  1. Add environment setup to ~/.bashrc:"
echo "     ../configure-environment.sh"
echo ""
echo "  2. Test the system:"
echo "     ../test-installation.sh"
echo ""
echo "  3. See usage guide:"
echo "     cat ../docs/USAGE-GUIDE.md"
echo "==========================================="
