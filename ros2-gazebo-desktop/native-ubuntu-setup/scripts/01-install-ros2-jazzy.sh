#!/bin/bash
# Install ROS 2 Jazzy on Ubuntu 24.04
# Includes desktop-full, dev tools, and Gazebo integration

set -e

echo "==========================================="
echo "  Installing ROS 2 Jazzy"
echo "==========================================="
echo ""
echo "This will install:"
echo "  - ROS 2 Jazzy Desktop Full"
echo "  - Development tools (colcon, rosdep)"
echo "  - ROS 2 Gazebo packages"
echo ""
echo "Estimated time: 15-20 minutes"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled"
    exit 1
fi

# Check if already installed
if [ -d "/opt/ros/jazzy" ]; then
    echo "⚠️  ROS 2 Jazzy already installed at /opt/ros/jazzy"
    read -p "Reinstall? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Skipping ROS 2 installation"
        exit 0
    fi
fi

echo ""
echo "Step 1/6: Setting up ROS 2 repository..."
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# Add ROS 2 GPG key
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo ""
echo "Step 2/6: Updating package lists..."
sudo apt update

echo ""
echo "Step 3/6: Installing ROS 2 Jazzy Desktop Full..."
echo "   (This may take 10-15 minutes...)"
sudo apt install -y ros-jazzy-desktop-full

echo ""
echo "Step 4/6: Installing development tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    ros-dev-tools

echo ""
echo "Step 5/6: Installing ROS 2 Gazebo packages..."
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gzharmonic \
    ros-jazzy-gz-ros2-control

echo ""
echo "Step 6/6: Initializing rosdep..."
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi
rosdep update

echo ""
echo "==========================================="
echo "  ✅ ROS 2 Jazzy Installation Complete!"
echo "==========================================="
echo ""
echo "To use ROS 2, source the setup file:"
echo "  source /opt/ros/jazzy/setup.bash"
echo ""
echo "Add to ~/.bashrc for automatic sourcing:"
echo "  echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc"
echo ""
echo "Verify installation:"
echo "  ros2 --version"
echo "  ros2 pkg list | head"
echo ""
echo "Next step:"
echo "  ./02-install-gazebo-harmonic.sh"
echo "==========================================="
