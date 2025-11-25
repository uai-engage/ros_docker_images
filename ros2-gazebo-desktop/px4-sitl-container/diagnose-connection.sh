#!/bin/bash
# Diagnose PX4 connection issues
# Check Gazebo bridge, MAVLink ports, and sensor data

echo "=========================================="
echo "PX4 Connection Diagnostics"
echo "=========================================="
echo ""

if ! docker ps | grep -q px4_sitl; then
    echo "❌ PX4 container is not running!"
    exit 1
fi

echo "1. Checking MAVLink TCP port 5760..."
docker exec px4_sitl bash -c "netstat -tulpn 2>/dev/null | grep 5760 || ss -tulpn 2>/dev/null | grep 5760 || echo 'netstat/ss not available, checking process...'"
docker exec px4_sitl bash -c "pgrep -fa mavlink | grep 5760 || echo '❌ MAVLink TCP on 5760 NOT found'"
echo ""

echo "2. Checking if vehicle is spawned in Gazebo..."
docker exec px4_sitl bash -c "gz model -l 2>/dev/null | grep -i x500 || echo '❌ No vehicle model found in Gazebo'"
echo ""

echo "3. Checking Gazebo-PX4 bridge topics..."
docker exec px4_sitl bash -c "gz topic -l 2>/dev/null | grep -E '(imu|clock|mag|baro)' | head -20 || echo '❌ No sensor topics found'"
echo ""

echo "3b. Checking PX4 bridge module status..."
docker logs px4_sitl 2>&1 | grep -i "gz_bridge" | tail -10 || echo "No bridge logs found"
echo ""

echo "4. Checking PX4 MAVLink instances..."
docker exec px4_sitl bash -c "pgrep -fa 'px4' | grep -v grep"
echo ""

echo "5. Recent PX4 logs (MAVLink startup)..."
docker logs px4_sitl 2>&1 | grep -i "mavlink" | tail -10
echo ""

echo "6. Recent PX4 logs (sensor errors)..."
docker logs px4_sitl 2>&1 | grep -E "(Accel|Gyro|baro|ekf2)" | tail -10
echo ""

echo "7. Checking custom startup scripts..."
docker exec px4_sitl bash -c "
if [ -f /home/px4user/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rc.autostart.post ]; then
    echo 'rc.autostart.post:'
    cat /home/px4user/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rc.autostart.post
else
    echo '❌ rc.autostart.post not found'
fi
"
echo ""

echo "8. Checking for TCP MAVLink specifically..."
docker logs px4_sitl 2>&1 | grep -i "tcp.*5760" || echo "❌ No TCP MAVLink on port 5760 found in logs"
echo ""

echo "9. Checking sensor data flow..."
docker logs px4_sitl 2>&1 | grep -E "(accel|gyro|mag|baro)" | grep -v "missing" | tail -5 || echo "❌ No sensor data detected"
echo ""

echo "10. Checking rcS modification..."
docker exec px4_sitl bash -c "grep -A 2 'TCP MAVLink 5760' /home/px4user/PX4-Autopilot/build/px4_sitl_default/etc/init.d/rcS 2>/dev/null || echo '❌ rcS not modified'"
echo ""

echo "=========================================="
echo "Diagnostic complete"
echo ""
echo "Summary:"
echo "--------"
docker logs px4_sitl 2>&1 | tail -5 | head -3
echo "=========================================="
