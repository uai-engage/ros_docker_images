#!/bin/bash
# Install PX4 Autopilot Dependencies
# All required libraries and tools for PX4 v1.16.0

set -e

echo "==========================================="
echo "  Installing PX4 Dependencies"
echo "==========================================="
echo ""
echo "This will install:"
echo "  - Build tools (cmake, ninja, gcc)"
echo "  - Python packages (numpy, jinja2, etc.)"
echo "  - Libraries (OpenCV, ZeroMQ, GStreamer)"
echo "  - MAVLink tools"
echo ""
echo "Estimated time: 5-10 minutes"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled"
    exit 1
fi

echo ""
echo "Step 1/5: Installing build essentials..."
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    curl \
    ca-certificates \
    gnupg \
    lsb-release

echo ""
echo "Step 2/5: Installing Python packages..."
sudo apt install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-setuptools \
    python3-wheel \
    python3-packaging \
    python3-toml \
    python3-numpy \
    python3-empy \
    python3-jinja2 \
    python3-yaml \
    python3-jsonschema

# Install additional Python packages via pip
pip3 install --user --break-system-packages \
    kconfiglib \
    pyserial \
    future \
    cerberus \
    pymavlink \
    MAVProxy \
    pyulog \
    pyros-genmsg

echo ""
echo "Step 3/5: Installing PX4 build dependencies..."
sudo apt install -y \
    astyle \
    exiftool \
    genromfs \
    libxml2-dev \
    libxslt1-dev \
    unzip \
    zip

echo ""
echo "Step 4/5: Installing required libraries..."
sudo apt install -y \
    libzmq3-dev \
    cppzmq-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libopencv-dev

echo ""
echo "Step 5/5: Installing networking tools..."
sudo apt install -y \
    net-tools \
    iputils-ping \
    iproute2 \
    netcat-openbsd \
    socat

echo ""
echo "==========================================="
echo "  ✅ PX4 Dependencies Installation Complete!"
echo "==========================================="
echo ""
echo "Verify Python packages:"
echo "  python3 -c 'import pymavlink; print(pymavlink.__version__)'"
echo "  python3 -c 'import numpy; print(numpy.__version__)'"
echo ""
echo "Next step:"
echo "  ./04-setup-px4.sh"
echo "==========================================="
