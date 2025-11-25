#!/bin/bash
# System Requirements Check
# Verifies Ubuntu version, disk space, RAM, and connectivity

set -e

echo "==========================================="
echo "  System Requirements Check"
echo "==========================================="
echo ""

ERRORS=0
WARNINGS=0

# Check Ubuntu version
echo "1. Checking Ubuntu version..."
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" = "24.04" ]; then
        echo "   ✅ Ubuntu $VERSION_ID (Noble Numbat) - Compatible"
    else
        echo "   ⚠️  Ubuntu $VERSION_ID detected"
        echo "      This guide is for Ubuntu 24.04 LTS"
        echo "      Installation may work but is not tested"
        WARNINGS=$((WARNINGS + 1))
    fi
else
    echo "   ❌ Cannot detect Ubuntu version"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Check disk space
echo "2. Checking available disk space..."
AVAILABLE_GB=$(df -BG . | tail -1 | awk '{print $4}' | sed 's/G//')
if [ "$AVAILABLE_GB" -ge 30 ]; then
    echo "   ✅ ${AVAILABLE_GB}GB available (30GB minimum)"
else
    echo "   ❌ Only ${AVAILABLE_GB}GB available (30GB required)"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Check RAM
echo "3. Checking RAM..."
TOTAL_RAM_GB=$(free -g | awk '/^Mem:/{print $2}')
if [ "$TOTAL_RAM_GB" -ge 8 ]; then
    echo "   ✅ ${TOTAL_RAM_GB}GB RAM (8GB minimum)"
else
    echo "   ⚠️  Only ${TOTAL_RAM_GB}GB RAM (8GB recommended)"
    echo "      Installation will work but may be slow"
    WARNINGS=$((WARNINGS + 1))
fi
echo ""

# Check CPU cores
echo "4. Checking CPU cores..."
CPU_CORES=$(nproc)
if [ "$CPU_CORES" -ge 4 ]; then
    echo "   ✅ $CPU_CORES CPU cores (4+ recommended)"
else
    echo "   ⚠️  Only $CPU_CORES CPU cores (4+ recommended for faster builds)"
    WARNINGS=$((WARNINGS + 1))
fi
echo ""

# Check internet connectivity
echo "5. Checking internet connectivity..."
if ping -c 1 packages.ros.org &> /dev/null; then
    echo "   ✅ Internet connection OK"
else
    echo "   ❌ Cannot reach packages.ros.org"
    echo "      Internet connection required for installation"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Check sudo privileges
echo "6. Checking sudo privileges..."
if sudo -n true 2>/dev/null; then
    echo "   ✅ sudo privileges available"
else
    echo "   ⚠️  sudo password may be required during installation"
    WARNINGS=$((WARNINGS + 1))
fi
echo ""

# Check if ROS 2 already installed
echo "7. Checking existing installations..."
if [ -d "/opt/ros/jazzy" ]; then
    echo "   ⚠️  ROS 2 Jazzy already installed at /opt/ros/jazzy"
    echo "      Installation will skip ROS 2 setup"
    WARNINGS=$((WARNINGS + 1))
else
    echo "   ✅ No existing ROS 2 Jazzy installation"
fi

if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version 2>&1 | head -1 || echo "unknown")
    echo "   ⚠️  Gazebo already installed: $GZ_VERSION"
    echo "      Installation will skip Gazebo setup"
    WARNINGS=$((WARNINGS + 1))
else
    echo "   ✅ No existing Gazebo installation"
fi
echo ""

# Summary
echo "==========================================="
echo "  Summary"
echo "==========================================="
if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo "✅ All checks passed! System is ready for installation."
    echo ""
    echo "Proceed with installation:"
    echo "  cd scripts"
    echo "  ./01-install-ros2-jazzy.sh"
elif [ $ERRORS -eq 0 ]; then
    echo "⚠️  $WARNINGS warning(s) detected"
    echo "   Installation should work but may have issues"
    echo ""
    echo "You can proceed with:"
    echo "  cd scripts"
    echo "  ./01-install-ros2-jazzy.sh"
else
    echo "❌ $ERRORS critical error(s) detected"
    echo "   Please fix the errors above before proceeding"
    exit 1
fi
echo "==========================================="
