#!/bin/bash
# Emergency fix: Manually start TCP MAVLink and check sensor bridge
# Run this after PX4 container starts

echo "=========================================="
echo "Manual TCP MAVLink & Sensor Fix"
echo "=========================================="

if ! docker ps | grep -q px4_sitl; then
    echo "❌ PX4 container is not running!"
    exit 1
fi

echo ""
echo "1. Checking current MAVLink instances..."
docker exec px4_sitl bash -c "pgrep -fa mavlink"

echo ""
echo "2. Attempting to start TCP MAVLink on port 5760..."
echo ""

# Create a startup command script
docker exec px4_sitl bash -c "cat > /tmp/start_tcp_mavlink.sh << 'EOF'
#!/bin/bash
# Try to send command to PX4 shell
echo 'mavlink start -x -o 5760 -t 0.0.0.0 -m onboard -r 4000000' > /home/px4user/.px4_cmd
sleep 1
# Alternative: Try via commander
cd /home/px4user/PX4-Autopilot
echo 'mavlink start -x -o 5760 -t 0.0.0.0 -m onboard -r 4000000' | build/px4_sitl_default/bin/px4 2>&1 | grep -i mavlink || echo 'Command sent (may show error if already running)'
EOF
chmod +x /tmp/start_tcp_mavlink.sh
/tmp/start_tcp_mavlink.sh
"

echo ""
echo "3. Waiting 5 seconds..."
sleep 5

echo ""
echo "4. Checking if TCP MAVLink started..."
docker logs px4_sitl 2>&1 | tail -30 | grep -i "tcp.*5760" && \
    echo "✅ TCP MAVLink detected!" || \
    echo "⚠️  TCP MAVLink not detected"

echo ""
echo "5. Checking sensor bridge status..."
docker logs px4_sitl 2>&1 | grep -i "imu\|accel\|gyro" | grep -v "missing" | tail -5

echo ""
echo "6. Testing Gazebo sensor data..."
echo "Sampling IMU topic for 3 seconds..."
docker exec px4_sitl timeout 3 gz topic -e -t /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu 2>/dev/null | head -10 || echo "❌ No IMU data from Gazebo"

echo ""
echo "=========================================="
echo "If TCP MAVLink still not working:"
echo "  - Check: docker logs px4_sitl | grep ERROR"
echo "  - The rcS command may be failing silently"
echo ""
echo "Try connecting QGroundControl to:"
echo "  TCP 10.200.10.66:5760"
echo "=========================================="
