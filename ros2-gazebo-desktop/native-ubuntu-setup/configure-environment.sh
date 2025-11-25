#!/bin/bash
# Configure Environment Variables
# Adds ROS 2, PX4, and Gazebo setup to ~/.bashrc

set -e

echo "==========================================="
echo "  Environment Configuration"
echo "==========================================="
echo ""
echo "This will add to your ~/.bashrc:"
echo "  - ROS 2 Jazzy setup"
echo "  - PX4 environment variables"
echo "  - Gazebo resource paths"
echo "  - Useful aliases"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Configuration cancelled"
    exit 1
fi

BASHRC=~/.bashrc
BACKUP=~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)

echo ""
echo "Creating backup of ~/.bashrc at $BACKUP..."
cp $BASHRC $BACKUP

echo ""
echo "Adding configuration to ~/.bashrc..."

# Check if already configured
if grep -q "# PX4 Native Setup Configuration" $BASHRC; then
    echo "⚠️  Configuration already exists in ~/.bashrc"
    read -p "Replace existing configuration? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # Remove old configuration
        sed -i '/# PX4 Native Setup Configuration/,/# End PX4 Native Setup/d' $BASHRC
    else
        echo "Keeping existing configuration"
        exit 0
    fi
fi

# Add new configuration
cat >> $BASHRC << 'EOF'

# PX4 Native Setup Configuration
# ================================

# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# micro-ROS Agent
if [ -f ~/px4_workspace/micro_ros_ws/install/setup.bash ]; then
    source ~/px4_workspace/micro_ros_ws/install/setup.bash
fi

# PX4 Environment
export PX4_HOME=~/px4_workspace/PX4-Autopilot
export PATH=$PX4_HOME/Tools:$PATH

# Gazebo Resource Paths
export GZ_SIM_RESOURCE_PATH=$PX4_HOME/Tools/simulation/gz/models:$PX4_HOME/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH

# Convenience Aliases
alias px4-sitl='cd ~/px4_workspace/PX4-Autopilot && make px4_sitl gz_x500'
alias px4-console='cd ~/px4_workspace/PX4-Autopilot && make px4_sitl_default none'
alias gz-sim='gz sim'
alias gz-default='gz sim ~/px4_workspace/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf'
alias microros='ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888'
alias px4-topics='ros2 topic list | grep fmu'
alias px4-status='ros2 topic echo /fmu/out/vehicle_status'

# End PX4 Native Setup
# ================================
EOF

echo ""
echo "==========================================="
echo "  ✅ Environment Configuration Complete!"
echo "==========================================="
echo ""
echo "Configuration added to ~/.bashrc"
echo "Backup saved at: $BACKUP"
echo ""
echo "To apply changes now:"
echo "  source ~/.bashrc"
echo ""
echo "Or open a new terminal"
echo ""
echo "Available aliases:"
echo "  px4-sitl      - Start PX4 with Gazebo x500"
echo "  px4-console   - Start PX4 console (no Gazebo)"
echo "  gz-sim        - Start Gazebo simulator"
echo "  gz-default    - Start Gazebo with default world"
echo "  microros      - Start micro-ROS agent"
echo "  px4-topics    - List PX4 ROS 2 topics"
echo "  px4-status    - Show vehicle status"
echo ""
echo "Next step:"
echo "  source ~/.bashrc"
echo "  ./test-installation.sh"
echo "==========================================="
