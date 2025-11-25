#!/bin/bash
# Clone and Build PX4 Autopilot v1.16.0
# Sets up PX4 workspace and builds SITL

set -e

echo "==========================================="
echo "  Setting Up PX4 Autopilot v1.16.0"
echo "==========================================="
echo ""
echo "This will:"
echo "  - Create ~/px4_workspace directory"
echo "  - Clone PX4-Autopilot v1.16.0 with submodules"
echo "  - Build PX4 SITL for Gazebo"
echo ""
echo "Estimated time: 20-30 minutes (first build)"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled"
    exit 1
fi

# Create workspace directory
PX4_WORKSPACE=~/px4_workspace
echo ""
echo "Step 1/4: Creating workspace at $PX4_WORKSPACE..."
mkdir -p $PX4_WORKSPACE
cd $PX4_WORKSPACE

# Clone PX4
if [ -d "PX4-Autopilot" ]; then
    echo "⚠️  PX4-Autopilot directory already exists"
    read -p "Remove and re-clone? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf PX4-Autopilot
    else
        echo "Using existing PX4-Autopilot directory"
        cd PX4-Autopilot
        git fetch --all
        git checkout v1.16.0
        git submodule update --init --recursive
        cd ..
    fi
fi

if [ ! -d "PX4-Autopilot" ]; then
    echo ""
    echo "Step 2/4: Cloning PX4-Autopilot v1.16.0..."
    echo "   (This includes all submodules, may take 5-10 minutes...)"
    git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.16.0
fi

cd PX4-Autopilot

echo ""
echo "Step 3/4: Running PX4 dependencies script..."
echo "   (This installs additional PX4-specific dependencies...)"
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

echo ""
echo "Step 4/4: Building PX4 SITL..."
echo "   (First build takes 20-25 minutes, subsequent builds are faster...)"
echo "   You can monitor progress - this is normal"
make px4_sitl_default

echo ""
echo "==========================================="
echo "  ✅ PX4 Autopilot Setup Complete!"
echo "==========================================="
echo ""
echo "PX4 installed at:"
echo "  $PX4_WORKSPACE/PX4-Autopilot"
echo ""
echo "Build output:"
echo "  $PX4_WORKSPACE/PX4-Autopilot/build/px4_sitl_default/"
echo ""
echo "Test PX4 SITL (without Gazebo):"
echo "  cd $PX4_WORKSPACE/PX4-Autopilot"
echo "  make px4_sitl_default none"
echo ""
echo "Run PX4 with Gazebo (after next step):"
echo "  cd $PX4_WORKSPACE/PX4-Autopilot"
echo "  make px4_sitl gz_x500"
echo ""
echo "Next step:"
echo "  ./05-install-microros.sh"
echo "==========================================="
