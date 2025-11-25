#!/bin/bash
# Gazebo Transport Connection Test
# Run this inside the PX4 container to test Gazebo connectivity

echo "=========================================="
echo "Gazebo Transport Connection Test"
echo "=========================================="
echo ""

echo "1. Environment Variables:"
echo "   GZ_PARTITION: ${GZ_PARTITION}"
echo "   GZ_IP: ${GZ_IP}"
echo "   GZ_RELAY: ${GZ_RELAY}"
echo ""

echo "2. Checking Gazebo topics..."
TOPICS=$(gz topic -l 2>&1)
if [ -z "$TOPICS" ]; then
    echo "   ❌ NO TOPICS FOUND - Gazebo Transport not working!"
    echo ""
    echo "   Possible causes:"
    echo "   - Gazebo not running in ROS2 container"
    echo "   - Network isolation between containers"
    echo "   - GZ_PARTITION mismatch"
    echo ""
    echo "3. Network connectivity test..."
    echo "   Checking if Gazebo Transport discovery service is reachable..."

    # Try to check if port 11345 (default Gazebo Transport) is open
    if command -v nc &> /dev/null; then
        timeout 2 nc -zv 127.0.0.1 11345 2>&1 || echo "   ❌ Cannot reach Gazebo Transport on localhost:11345"
    fi
else
    echo "   ✅ TOPICS FOUND!"
    echo "$TOPICS" | head -10
    echo ""

    echo "3. Checking for /clock topic (required for PX4)..."
    if echo "$TOPICS" | grep -q "/clock"; then
        echo "   ✅ /clock topic exists"

        echo ""
        echo "4. Checking for world topics..."
        if echo "$TOPICS" | grep -q "/world/"; then
            WORLD=$(echo "$TOPICS" | grep "/world/" | head -1 | cut -d'/' -f3)
            echo "   ✅ World detected: $WORLD"
        else
            echo "   ⚠️  No world topics found"
        fi
    else
        echo "   ❌ /clock topic missing!"
    fi
fi

echo ""
echo "=========================================="
echo "Test complete"
echo "=========================================="
