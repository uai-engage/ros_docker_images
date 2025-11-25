#!/bin/bash
# Manually enable TCP MAVLink on running PX4 instance
# Use this if automatic TCP MAVLink doesn't start

echo "=========================================="
echo "Enabling TCP MAVLink on port 5760"
echo "=========================================="

if ! docker ps | grep -q px4_sitl; then
    echo "❌ PX4 container is not running!"
    exit 1
fi

echo ""
echo "Sending MAVLink TCP command to PX4..."

# Method 1: Try via PX4 command file
docker exec px4_sitl bash -c "
echo 'mavlink start -x -o 5760 -t 0.0.0.0 -m onboard -r 4000000' > /tmp/px4_cmd.txt
"

# Method 2: Direct command via nuttx shell (if available)
docker exec -it px4_sitl bash -c "
cd /home/px4user/PX4-Autopilot
echo 'mavlink start -x -o 5760 -t 0.0.0.0 -m onboard -r 4000000' | ./build/px4_sitl_default/bin/px4-simulator_shell
" 2>/dev/null || echo "Note: Direct shell method not available"

echo ""
echo "Checking if TCP MAVLink started..."
sleep 2

docker logs px4_sitl 2>&1 | tail -20 | grep -i "tcp.*5760" && \
    echo "✅ TCP MAVLink on port 5760 detected!" || \
    echo "⚠️  TCP MAVLink may not have started - check logs with: docker compose logs -f"

echo ""
echo "=========================================="
echo ""
echo "Try connecting QGroundControl now:"
echo "  Type: TCP"
echo "  Server: <your-server-ip>"
echo "  Port: 5760"
echo ""
echo "=========================================="
