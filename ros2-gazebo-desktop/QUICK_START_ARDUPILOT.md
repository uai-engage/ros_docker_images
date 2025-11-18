# Quick Start: ArduPilot + micro-ROS

**TL;DR**: Connect ArduPilot container to ROS 2 container via micro-ROS agent.

## Prerequisites

- ✅ ArduPilot container running (host network mode)
- ✅ ROS 2 container running (host network mode)
- ✅ Both on same host machine

## 3-Step Setup

### 1. Install micro-ROS Agent (ROS Container)

**NOTE:** `ros-jazzy-micro-ros-agent` is NOT available as apt package.
Must build from source:

```bash
cd ~/ros2_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent.git
cd ~/ros2_ws

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Build (will use ROS Jazzy's built-in Fast DDS)
colcon build --packages-select micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Verify
ros2 run micro_ros_agent micro_ros_agent --help
```

**Build time:** ~2-3 minutes

### 2. Configure ArduPilot (ArduPilot Container)

Set these parameters:
```
SERIAL3_PROTOCOL = 45      # DDS/micro-ROS
SERIAL3_BAUD = 921600
DDS_ENABLE = 1
DDS_PORT = 2019            # Or your chosen port
```

For SITL:
```bash
sim_vehicle.py -v ArduCopter -A "--serial3=uart:127.0.0.1:2019"
```

### 3. Run micro-ROS Agent (ROS Container)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash  # if built from source
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Verify It Works

```bash
# In ROS container, new terminal:
ros2 topic list

# Should see:
# /fmu/battery/status
# /fmu/gps/position
# /fmu/imu/data
# ... etc

# Echo a topic:
ros2 topic echo /fmu/battery/status
```

## Architecture

```
ArduPilot Container          ROS 2 Container
(host network)              (host network)
     |                           |
     | UDP 127.0.0.1:8888       |
     |-------------------------->|
     |  micro-XRCE-DDS          | micro-ROS Agent
     |                           |    ↓ (converts to DDS)
     |                           | ROS 2 Topics (/fmu/*)
```

## Troubleshooting

**No topics?**
- Check agent is running: `ros2 run micro_ros_agent ...` should show "session established"
- Check ArduPilot params: `SERIAL3_PROTOCOL = 45` and `DDS_ENABLE = 1`
- Verify network: `sudo tcpdump -i lo -n udp port 8888`

**Connection drops?**
```bash
# Use verbose mode to debug
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

## Full Documentation

See `ARDUPILOT_MICRO_ROS_INTEGRATION.md` for:
- Detailed architecture
- All installation options
- Advanced configuration
- Launch files
- MAVROS2 integration
- Troubleshooting guide
