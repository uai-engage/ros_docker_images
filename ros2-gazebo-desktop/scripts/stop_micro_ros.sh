#!/bin/bash
# stop_micro_ros.sh
# Complete cleanup script for micro-ROS agent

echo "==========================================="
echo "  Stopping micro-ROS Agent & Clearing Cache"
echo "==========================================="
echo ""

# Find PID
PID=$(pgrep -f micro_ros_agent)

if [ -z "$PID" ]; then
    echo "✓ micro-ROS agent is not running"
else
    echo "Found micro-ROS agent PID: $PID"

    # Try graceful shutdown
    echo "Stopping agent gracefully..."
    kill $PID
    sleep 2

    # Check if still running
    if pgrep -f micro_ros_agent > /dev/null; then
        echo "Process still running, forcing shutdown..."
        pkill -9 -f micro_ros_agent
        sleep 1
    fi

    # Verify stopped
    if pgrep -f micro_ros_agent > /dev/null; then
        echo "✗ ERROR: Failed to stop micro-ROS agent"
        exit 1
    else
        echo "✓ micro-ROS agent stopped"
    fi
fi

# Clear ROS 2 DDS cache (removes ghost topics)
echo ""
echo "Clearing ROS 2 DDS discovery cache..."
ros2 daemon stop
sleep 1
ros2 daemon start
sleep 1

echo ""
echo "Verifying cleanup..."

# Check topics
TOPICS=$(ros2 topic list 2>/dev/null | grep -c "/fmu/")
if [ "$TOPICS" -eq 0 ]; then
    echo "✓ Ghost topics cleared"
else
    echo "✗ Warning: $TOPICS /fmu/ topics still present"
fi

# Check port
if netstat -tuln 2>/dev/null | grep -q ":2019"; then
    echo "✗ Warning: Port 2019 still in use"
else
    echo "✓ Port 2019 is free"
fi

echo ""
echo "==========================================="
echo "  Cleanup Complete!"
echo "==========================================="
exit 0
