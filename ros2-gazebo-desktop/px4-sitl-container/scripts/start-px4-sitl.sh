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

# Modify PX4's rcS to add TCP MAVLink at the end
# This ensures it runs after all other initialization
if [ -f "${PX4_HOME}/build/px4_sitl_default/etc/init.d/rcS" ]; then
    # Check if we haven't already modified it
    if ! grep -q "TCP MAVLink 5760" "${PX4_HOME}/build/px4_sitl_default/etc/init.d/rcS"; then
        echo "" >> "${PX4_HOME}/build/px4_sitl_default/etc/init.d/rcS"
        echo "# TCP MAVLink 5760 for remote QGroundControl (added by start-px4-sitl.sh)" >> "${PX4_HOME}/build/px4_sitl_default/etc/init.d/rcS"
        echo "mavlink start -x -o 5760 -t 0.0.0.0 -m onboard -r 4000000" >> "${PX4_HOME}/build/px4_sitl_default/etc/init.d/rcS"
        echo "Modified rcS to add TCP MAVLink on port 5760"
    else
        echo "rcS already contains TCP MAVLink configuration"
    fi
else
    echo "Warning: rcS not found, will be created on first PX4 build"
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
    echo "  GZ_RELAY:     ${GZ_RELAY:-not set}"
    echo ""
    echo "Verifying Gazebo connection..."
    if gz topic -l 2>/dev/null | grep -q "/clock"; then
        echo "  ✓ Gazebo is running and accessible"
        TOPIC_COUNT=$(gz topic -l 2>/dev/null | wc -l)
        echo "  ✓ Found $TOPIC_COUNT Gazebo topics"
    else
        echo "  ❌ Cannot reach Gazebo!"
        echo ""
        echo "Make sure Gazebo is running in your ROS2 container:"
        echo "  gz sim -r default.sdf"
        echo ""
    fi
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

    # Check if vehicle is already spawned (optional, for manual spawn workflows)
    if [ "${WAIT_FOR_VEHICLE:-0}" = "1" ]; then
        echo "Checking if vehicle is already spawned in Gazebo..."
        MAX_WAIT=30
        WAITED=0
        while [ $WAITED -lt $MAX_WAIT ]; do
            if gz topic -l 2>/dev/null | grep -q "/world/.*/model/${PX4_GZ_MODEL:-x500}"; then
                echo "✓ Vehicle model topics detected in Gazebo"
                sleep 2  # Extra delay for sensor initialization
                break
            fi
            if [ $WAITED -eq 0 ]; then
                echo "Waiting for vehicle ${PX4_GZ_MODEL:-x500} to spawn in Gazebo..."
            fi
            sleep 1
            WAITED=$((WAITED + 1))
        done

        if [ $WAITED -ge $MAX_WAIT ]; then
            echo "⚠️  Warning: Vehicle not detected in Gazebo after ${MAX_WAIT}s"
            echo "   PX4 will spawn the vehicle automatically"
        fi
    else
        echo "PX4 will spawn the vehicle automatically in Gazebo"
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
    # rc.autostart.post will be executed automatically and start TCP MAVLink
    exec make px4_sitl gz_${PX4_GZ_MODEL:-x500}
else
    # Embedded Gazebo: Start both PX4 and Gazebo together
    echo "Starting PX4 with embedded Gazebo..."
    exec make px4_sitl gz_${PX4_GZ_MODEL:-x500}
fi
