#!/bin/bash
# Install Gazebo Harmonic on Ubuntu 24.04
# Complete installation with all libraries and tools

set -e

echo "==========================================="
echo "  Installing Gazebo Harmonic"
echo "==========================================="
echo ""
echo "This will install:"
echo "  - Gazebo Sim (gz-sim)"
echo "  - Gazebo Transport"
echo "  - All required Gazebo libraries"
echo "  - Command-line tools"
echo ""
echo "Estimated time: 10-15 minutes"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled"
    exit 1
fi

# Check if already installed
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version 2>&1 | head -1 || echo "unknown")
    echo "⚠️  Gazebo already installed: $GZ_VERSION"
    read -p "Reinstall? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Skipping Gazebo installation"
        exit 0
    fi
fi

echo ""
echo "Step 1/4: Adding Gazebo repository..."
sudo apt update
sudo apt install -y wget lsb-release gnupg

# Add Gazebo GPG key
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add Gazebo repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

echo ""
echo "Step 2/4: Updating package lists..."
sudo apt update

echo ""
echo "Step 3/4: Installing Gazebo Harmonic..."
echo "   (This may take 10 minutes...)"
sudo apt install -y gz-harmonic

echo ""
echo "Step 4/4: Verifying installation..."
gz sim --version

echo ""
echo "==========================================="
echo "  ✅ Gazebo Harmonic Installation Complete!"
echo "==========================================="
echo ""
echo "Test Gazebo:"
echo "  gz sim shapes.sdf"
echo ""
echo "Gazebo commands:"
echo "  gz sim    - Start Gazebo simulator"
echo "  gz topic  - List/echo Gazebo topics"
echo "  gz model  - List/info about models"
echo "  gz world  - World information"
echo ""
echo "Next step:"
echo "  ./03-install-px4-deps.sh"
echo "==========================================="
