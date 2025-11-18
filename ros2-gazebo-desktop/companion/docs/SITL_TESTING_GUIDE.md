# SITL Testing Guide - ArduPilot with micro-ROS

This guide shows how to test ArduPilot SITL (Software In The Loop) with micro-ROS agent using UDP communication.

## Architecture for SITL Testing

```
┌─────────────────────────────────────────────────────────┐
│              Ground Station / Development PC            │
│                                                         │
│  ┌──────────────────┐         ┌───────────────────┐   │
│  │  ArduPilot SITL  │   UDP   │  micro-ROS Agent  │   │
│  │  (Copter/Plane)  │◄───────►│   (Docker)        │   │
│  │                  │ :2019   │                   │   │
│  │  DDS_ENABLE=1    │         │  UDP4 port 2019   │   │
│  │  DDS_UDP_PORT    │         │                   │   │
│  └──────────────────┘         └─────────┬─────────┘   │
│                                          │             │
│                                          │ DDS         │
│                                          │             │
│  ┌───────────────────────────────────────▼──────────┐  │
│  │  ROS 2 Desktop Environment                      │  │
│  │  - ros2 topic list                              │  │
│  │  - ros2 topic echo /fmu/battery/status          │  │
│  │  - RViz2 visualization                          │  │
│  │  - Custom ROS 2 nodes                           │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## Quick Start

### 1. Start micro-ROS Agent (UDP Mode)

```bash
# Navigate to directory
cd ros2-gazebo-desktop

# Start companion container in UDP mode (default)
docker compose -f docker-compose.companion.yml up -d

# Check logs
docker logs -f micro_ros_agent_companion
```

You should see:
```
==========================================
  micro-ROS Agent Starting...
==========================================
Connection Mode: udp4
Ground Station IP: 192.168.1.100
ROS Domain ID: 0
==========================================

Starting UDP4 mode on port 2019
Command: ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019

[1731234567.123456] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 2019
[1731234567.123789] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

### 2. Start ArduPilot SITL

**Option A: Using sim_vehicle.py with DDS argument**

```bash
# Navigate to ArduPilot directory
cd ~/ardupilot

# Start SITL with DDS on serial port 3 pointing to UDP 2019
# For Copter:
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -A "--serial3=udp:127.0.0.1:2019"

# For Plane:
./Tools/autotest/sim_vehicle.py -v ArduPlane --console --map -A "--serial3=udp:127.0.0.1:2019"

# For Rover:
./Tools/autotest/sim_vehicle.py -v Rover --console --map -A "--serial3=udp:127.0.0.1:2019"
```

**Option B: Using parameters (after SITL starts)**

```bash
# Start SITL normally
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

# In MAVProxy console, set parameters:
param set DDS_ENABLE 1
param set DDS_UDP_PORT 2019
param set SERIAL3_PROTOCOL 45
param set SERIAL3_BAUD 921600

# Reboot to apply
reboot
```

### 3. Verify Connection

**In micro-ROS agent logs:**
```
[1731234567.456789] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x00000001, participant_id: 0x001
[1731234567.456999] info     | ProxyClient.cpp    | create_topic            | topic created          | client_key: 0x00000001, topic_id: 0x001
```

**Check ROS 2 topics:**
```bash
# List all topics (should see /fmu/* topics)
ros2 topic list

# Expected output:
# /fmu/battery/status
# /fmu/gps/position
# /fmu/imu/data
# /fmu/vehicle/status
# ... and more

# Echo a topic to see data
ros2 topic echo /fmu/battery/status

# Check message rate
ros2 topic hz /fmu/imu/data
```

## Configuration Options

### Environment Variables

Create a `.env` file for easy configuration:

```bash
# Copy example file
cp .env.companion.example .env

# Edit configuration
nano .env
```

**For SITL testing (UDP mode):**
```bash
CONNECTION_MODE=udp4
AGENT_PORT=2019
GROUND_STATION_IP=127.0.0.1  # localhost for SITL
ROS_DOMAIN_ID=0
VERBOSE_LEVEL=0
```

### Changing Port

**If port 2019 is in use:**

1. Update `.env`:
   ```bash
   AGENT_PORT=8888
   ```

2. Update ArduPilot SITL:
   ```bash
   ./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -A "--serial3=udp:127.0.0.1:8888"
   ```

### Verbose Logging (Debugging)

Enable verbose output to see detailed DDS communication:

```bash
# Set verbose level in .env
VERBOSE_LEVEL=6

# Restart container
docker compose -f docker-compose.companion.yml restart

# Watch logs
docker logs -f micro_ros_agent_companion
```

## Common ArduPilot SITL Commands

### Start SITL with Custom Location

```bash
# Sydney, Australia
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map \
  -L CMAC -A "--serial3=udp:127.0.0.1:2019"

# Custom lat/lon/alt/heading
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map \
  --custom-location=51.5074,-0.1278,100,0 -A "--serial3=udp:127.0.0.1:2019"
```

### Start SITL without Graphics

```bash
# No map, no console graphics
./Tools/autotest/sim_vehicle.py -v ArduCopter -A "--serial3=udp:127.0.0.1:2019"
```

### Start Multiple Instances

```bash
# First instance (default ports)
./Tools/autotest/sim_vehicle.py -v ArduCopter -I0 -A "--serial3=udp:127.0.0.1:2019"

# Second instance (offset ports)
./Tools/autotest/sim_vehicle.py -v ArduCopter -I1 -A "--serial3=udp:127.0.0.1:2020"
```

Remember to start multiple micro-ROS agents on different ports!

## Testing Workflow

### 1. Basic Connectivity Test

```bash
# Terminal 1: Start micro-ROS agent
docker compose -f docker-compose.companion.yml up

# Terminal 2: Start ArduPilot SITL
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -A "--serial3=udp:127.0.0.1:2019"

# Terminal 3: Monitor ROS topics
ros2 topic list
ros2 topic echo /fmu/vehicle/status
```

### 2. Visualize in RViz2

```bash
# Start RViz2
rviz2

# Add displays:
# - Add -> By topic -> /fmu/imu/data -> Imu
# - Add -> TF
# - Add -> Axes (for vehicle orientation)
```

### 3. Send Commands from ROS 2

```bash
# Example: Arm vehicle
ros2 topic pub /fmu/vehicle/command std_msgs/msg/String "data: 'ARM'" --once

# Example: Set mode
ros2 topic pub /fmu/mode/command std_msgs/msg/String "data: 'GUIDED'" --once
```

(Note: Actual command topics depend on ArduPilot DDS implementation)

## Troubleshooting

### Agent Starts but No Topics Appear

**Check 1: ArduPilot DDS is enabled**
```bash
# In MAVProxy console
param show DDS_*

# Should show:
# DDS_ENABLE = 1
# DDS_UDP_PORT = 2019
```

**Check 2: Serial protocol is set**
```bash
param show SERIAL3_PROTOCOL

# Should be 45 (DDS)
```

**Check 3: ROS domain matches**
```bash
# Check environment
echo $ROS_DOMAIN_ID

# Should be 0 (or same as agent)
```

**Check 4: Port is correct**
```bash
# Check what port agent is listening on
docker logs micro_ros_agent_companion | grep "port:"

# Check SITL is sending to correct port
# In sim_vehicle.py output, look for --serial3 argument
```

### Port Already in Use

```bash
# Find what's using port 2019
netstat -tulpn | grep 2019

# Kill the process
sudo kill -9 <PID>

# Or use different port (update both agent and SITL)
```

### Topics Appear Then Disappear

This is the "ghost topics" issue:

```bash
# Clear ROS 2 DDS cache
ros2 daemon stop
ros2 daemon start

# Restart micro-ROS agent
docker compose -f docker-compose.companion.yml restart
```

### SITL Won't Start with DDS

```bash
# Check ArduPilot is compiled with DDS support
# Recent ArduPilot master should have it

# If not, rebuild:
cd ~/ardupilot
./waf configure --board sitl
./waf copter

# Check for DDS libraries
ldd build/sitl/bin/arducopter | grep -i dds
```

### No Data on Topics

**Check 1: Vehicle is armed/receiving data**
```bash
# In MAVProxy
status
# Should show ARMED or STANDBY
```

**Check 2: Topics exist but no data**
```bash
# Check topic info
ros2 topic info /fmu/battery/status

# Check publisher count (should be > 0)
ros2 topic info /fmu/battery/status -v
```

**Check 3: Enable verbose logging**
```bash
VERBOSE_LEVEL=6 docker compose -f docker-compose.companion.yml up
```

## Performance Testing

### Check Message Rates

```bash
# IMU (should be high frequency, ~100 Hz)
ros2 topic hz /fmu/imu/data

# GPS (typically 5-10 Hz)
ros2 topic hz /fmu/gps/position

# Battery (typically 1-10 Hz)
ros2 topic hz /fmu/battery/status
```

### Monitor Bandwidth

```bash
# Install bmon if needed
sudo apt install bmon

# Monitor network interface
bmon

# Look for UDP traffic on port 2019
```

## Stopping Everything

```bash
# Stop micro-ROS agent
docker compose -f docker-compose.companion.yml down

# Stop ArduPilot SITL
# In MAVProxy console, press Ctrl+C
# Or in terminal, press Ctrl+C

# Clear ghost topics
ros2 daemon stop && ros2 daemon start
```

## Next Steps

Once SITL testing works:

1. **Test with real hardware**: Switch to serial mode, see `COMPANION_COMPUTER_SETUP.md`
2. **Develop ROS 2 nodes**: Create custom nodes to process /fmu/* topics
3. **Test radio link**: Use two separate machines, see network configuration
4. **Deploy on companion computer**: Use minimal Docker image for drone

## ArduPilot DDS Resources

- ArduPilot DDS Documentation: https://ardupilot.org/dev/docs/ros2.html
- Available Topics: Check `/fmu/` namespace with `ros2 topic list`
- Message Definitions: ArduPilot custom messages in `ap_msgs` package

## Quick Reference Commands

```bash
# Start micro-ROS agent (UDP)
docker compose -f docker-compose.companion.yml up -d

# Start SITL with DDS
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -A "--serial3=udp:127.0.0.1:2019"

# List topics
ros2 topic list

# Echo topic
ros2 topic echo /fmu/battery/status

# Check message rate
ros2 topic hz /fmu/imu/data

# Stop agent
docker compose -f docker-compose.companion.yml down

# Clear ghost topics
ros2 daemon stop && ros2 daemon start
```
