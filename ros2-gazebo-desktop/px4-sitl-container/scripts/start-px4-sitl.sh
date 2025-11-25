#!/bin/bash
# PX4 SITL Startup Script
# Supports both embedded Gazebo and external Gazebo modes

set -e

echo "============================================"
echo "  PX4 SITL Container Starting"
echo "============================================"
echo ""
echo "Configuration:"
echo "  PX4_GZ_MODEL:      ${PX4_GZ_MODEL:-x500}"
echo "  PX4_GZ_WORLD:      ${PX4_GZ_WORLD:-default}"
echo "  PX4_SYS_AUTOSTART: ${PX4_SYS_AUTOSTART:-4001}"
echo "  HEADLESS:          ${HEADLESS:-1}"
echo "  EXTERNAL_GAZEBO:   ${EXTERNAL_GAZEBO:-0}"
echo "  Instance:          ${PX4_SIM_INSTANCE:-0}"
echo ""

cd ${PX4_HOME:-/home/px4user/PX4-Autopilot}

# Check if source is mounted and needs rebuild
if [ -f "/.dockerenv" ] && [ -d "/home/px4user/PX4-Autopilot/src" ]; then
    # Check if build exists
    if [ ! -d "build/px4_sitl_default" ]; then
        echo "Building PX4 SITL (first run with mounted source)..."
        make px4_sitl
    fi
fi

# Configure MAVLink broadcast
export PX4_SIM_HOST_ADDR=${PX4_SIM_HOST_ADDR:-0.0.0.0}

# Configure Gazebo connection
export GZ_PARTITION=${GZ_PARTITION:-gazebo}
export GZ_IP=${GZ_IP:-127.0.0.1}

# Mode: External Gazebo (in ROS2 container) or Embedded Gazebo
if [ "${EXTERNAL_GAZEBO:-0}" = "1" ]; then
    echo ""
    echo "============================================"
    echo "  Mode: EXTERNAL GAZEBO"
    echo "============================================"
    echo ""
    echo "Connecting to Gazebo in ROS2 container..."
    echo "  GZ_PARTITION: ${GZ_PARTITION}"
    echo "  GZ_IP:        ${GZ_IP}"
    echo ""
    echo "Make sure Gazebo is running in your ROS2 container:"
    echo "  gz sim -r default.sdf"
    echo ""
    
    # Wait for Gazebo if requested
    if [ "${WAIT_FOR_GAZEBO:-0}" = "1" ]; then
        echo "Waiting for Gazebo transport..."
        while ! gz topic -l 2>/dev/null | grep -q "/clock"; do
            echo "  Gazebo not ready yet, waiting..."
            sleep 2
        done
        echo "Gazebo detected!"
    fi
else
    echo ""
    echo "============================================"
    echo "  Mode: EMBEDDED GAZEBO (Headless)"
    echo "============================================"
    echo ""
    
    # Set up virtual display for headless Gazebo
    if [ "${HEADLESS:-1}" = "1" ]; then
        echo "Starting Xvfb for headless operation..."
        Xvfb :99 -screen 0 1920x1080x24 &
        export DISPLAY=:99
        sleep 2
    fi
fi

echo ""
echo "Starting PX4 SITL..."
echo "============================================"
echo ""
echo "MAVLink Ports:"
echo "  UDP 14550 - QGroundControl (broadcast)"
echo "  UDP 14540 - MAVROS / Offboard control"
echo "  TCP 5760  - QGroundControl (TCP connection)"
echo ""
echo "uXRCE-DDS (for ROS 2):"
echo "  UDP 8888 - Connect micro-ROS agent to this port"
echo "  Command: ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888"
echo ""
echo "============================================"
echo ""

# Create PX4 extras file to add TCP MAVLink
mkdir -p ${PX4_HOME}/build/px4_sitl_default/etc
cat > ${PX4_HOME}/build/px4_sitl_default/etc/extras.txt << 'EOF'
# Additional MAVLink TCP connection for remote QGroundControl
mavlink start -x -o 5760 -r 4000000
EOF

# Run PX4 SITL
if [ "${EXTERNAL_GAZEBO:-0}" = "1" ]; then
    # External Gazebo: Don't start Gazebo, just run PX4 and connect
    echo "Connecting to external Gazebo..."
    echo "Model: ${PX4_GZ_MODEL:-x500}"
    echo "GZ_PARTITION: ${GZ_PARTITION:-gazebo}"
    echo ""

    # Tell PX4 that Gazebo is already running (standalone mode)
    export PX4_GZ_STANDALONE=1

    # Run PX4 SITL without starting Gazebo
    exec make px4_sitl gz_${PX4_GZ_MODEL:-x500}
else
    # Embedded Gazebo: Start both PX4 and Gazebo together
    echo "Starting PX4 with embedded Gazebo..."
    exec make px4_sitl gz_${PX4_GZ_MODEL:-x500}
fi
