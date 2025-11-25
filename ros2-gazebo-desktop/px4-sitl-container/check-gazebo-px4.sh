#!/bin/bash
# Check if PX4 container can see Gazebo
# Usage: ./check-gazebo-px4.sh

echo "=========================================="
echo "Checking Gazebo from PX4 Container"
echo "=========================================="
echo ""

# Check if container is running
if ! docker ps | grep -q px4_sitl; then
    echo "❌ PX4 container is not running!"
    echo ""
    echo "Start it with:"
    echo "docker compose --env-file .env.example up -d"
    exit 1
fi

echo "1. Environment Variables:"
docker exec px4_sitl bash -c "
echo \"   GZ_PARTITION: \${GZ_PARTITION}\"
echo \"   GZ_RELAY: \${GZ_RELAY}\"
echo \"   GZ_IP: \${GZ_IP}\"
echo \"   PX4_GZ_STANDALONE: \${PX4_GZ_STANDALONE}\"
"

echo ""
echo "2. Checking Gazebo Topics from PX4:"
docker exec px4_sitl bash -c "
TOPICS=\$(gz topic -l 2>&1)
if [ -z \"\$TOPICS\" ]; then
    echo '   ❌ NO TOPICS FOUND'
    echo ''
    echo '   PX4 cannot see Gazebo!'
    echo '   Possible causes:'
    echo '   - Gazebo not running in ROS2 container'
    echo '   - GZ_PARTITION mismatch'
    echo '   - GZ_RELAY not enabled'
    echo ''
    echo '   Verify Gazebo is running:'
    echo '   cd .. && ./check-gazebo-ros2.sh'
else
    echo '   ✅ TOPICS FOUND!'
    echo \"\$TOPICS\" | head -10
    echo ''

    if echo \"\$TOPICS\" | grep -q '/clock'; then
        echo '   ✅ /clock topic exists'
        echo '   ✅ PX4 should be able to connect!'
    else
        echo '   ⚠️  /clock topic missing'
    fi
fi
"

echo ""
echo "=========================================="
echo "Check complete"
echo "=========================================="
