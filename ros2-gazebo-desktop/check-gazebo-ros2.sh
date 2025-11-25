#!/bin/bash
# Check Gazebo status in ROS2 container
# Usage: ./check-gazebo-ros2.sh

echo "=========================================="
echo "Checking Gazebo in ROS2 Container"
echo "=========================================="
echo ""

# Source ROS2 and check gz command
docker exec ros2_dev_station bash -c "
source /opt/ros/jazzy/setup.bash

echo '1. Gazebo Command Available:'
if command -v gz &> /dev/null; then
    echo '   ✅ gz command found'
else
    echo '   ❌ gz command not found'
    exit 1
fi

echo ''
echo '2. Environment Variables:'
echo \"   GZ_PARTITION: \${GZ_PARTITION}\"
echo \"   GZ_RELAY: \${GZ_RELAY}\"
echo \"   GZ_SIM_RESOURCE_PATH: \${GZ_SIM_RESOURCE_PATH}\"

echo ''
echo '3. Gazebo Topics:'
TOPICS=\$(gz topic -l 2>&1)
if [ -z \"\$TOPICS\" ]; then
    echo '   ❌ No topics found - Gazebo may not be running'
    echo ''
    echo '   Start Gazebo with:'
    echo '   docker exec -it ros2_dev_station bash'
    echo '   gz sim -v4 -r ~/px4_gazebo/worlds/default.sdf'
else
    echo '   ✅ Topics found:'
    echo \"\$TOPICS\" | head -10
    echo ''

    if echo \"\$TOPICS\" | grep -q '/clock'; then
        echo '   ✅ /clock topic exists (PX4 requirement)'
    else
        echo '   ❌ /clock topic missing'
    fi
fi
"

echo ""
echo "=========================================="
echo "Check complete"
echo "=========================================="
