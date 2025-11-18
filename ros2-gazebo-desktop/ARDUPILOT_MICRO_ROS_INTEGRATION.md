# ArduPilot + micro-ROS Agent Integration Guide

Complete guide for connecting ArduPilot (running in separate Docker container) to ROS 2 via micro-ROS agent.

## Architecture Overview

```
┌─────────────────────────────────────┐         ┌──────────────────────────────────────┐
│   ArduPilot Container               │         │   ROS 2 + micro-ROS Container        │
│   (Your existing setup)             │         │   (This container)                   │
│                                     │         │                                      │
│  ┌──────────────────────┐           │         │  ┌────────────────────┐              │
│  │   ArduPilot SITL     │           │         │  │   micro-ROS Agent  │              │
│  │   (Copter/Plane/etc) │           │         │  │   (DDS-XRCE)       │              │
│  │                      │           │         │  │   Port: 8888       │              │
│  │   Serial/UDP Output  │◄─────────────────────┼─►│                    │              │
│  │   Port: 2019         │  micro-XRCE         │  └─────────┬──────────┘              │
│  └──────────────────────┘  Protocol           │            │                         │
│                             (UDP)              │            │ DDS                     │
│                                                │            │                         │
└────────────────────────────────────────────────┘            │                         │
             Host Network Mode                               │                         │
             (127.0.0.1 / localhost)                         │                         │
                                                             │                         │
                                                    ┌────────▼─────────┐               │
                                                    │   ROS 2 Nodes    │               │
                                                    │   - /fmu/*       │               │
                                                    │   - Publishers   │               │
                                                    │   - Subscribers  │               │
                                                    └──────────────────┘               │
                                                                                       │
└───────────────────────────────────────────────────────────────────────────────────────┘
```

### Communication Flow

1. **ArduPilot** outputs micro-ROS data via serial/UDP (configured with `SERIAL_PROTOCOL`)
2. **micro-ROS Agent** receives micro-XRCE protocol data
3. **Agent** translates to DDS (Fast DDS)
4. **ROS 2 nodes** see ArduPilot topics as native ROS 2 topics

---

## Part 1: micro-ROS Agent Installation

### Option A: Install from APT (Recommended - Try First)

This is the simplest approach if the package is available:

```bash
# Inside ROS 2 container
sudo apt-get update

# Search for micro-ROS packages
apt-cache search micro-ros

# Install if available (for ROS 2 Jazzy)
sudo apt-get install ros-jazzy-micro-ros-agent

# Verify installation
ros2 run micro_ros_agent micro_ros_agent --help
```

**If this works**, you're done with installation! Skip to Part 2.

**If package not found**, proceed to Option B or C.

---

### Option B: Build from Source (Using ROS Fast DDS)

If ArduPilot's micro-ROS doesn't require a specific Fast DDS version:

```bash
cd ~/ros2_ws/src

# Clone micro-ROS agent
git clone https://github.com/micro-ROS/micro-ROS-Agent.git

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Build using ROS's Fast DDS
cd ~/ros2_ws
colcon build --packages-select micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source ~/ros2_ws/install/setup.bash

# Test
ros2 run micro_ros_agent micro_ros_agent --help
```

---

### Option C: Build with Custom Fast DDS (If Version Specific)

If ArduPilot requires a specific Fast DDS version (check ArduPilot docs):

```bash
cd ~/ros2_ws

# Custom Fast DDS is already built in: src/fastdds_ws/
# (Built during Docker image creation)

# Clone micro-ROS agent if not already cloned
cd ~/ros2_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent.git

# Source environments in order
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/fastdds_ws/install/setup.bash

# Build with explicit paths to custom Fast DDS
cd ~/ros2_ws
colcon build --packages-select micro_ros_agent \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$HOME/ros2_ws/src/fastdds_ws/install:$CMAKE_PREFIX_PATH"

# Check which Fast DDS it linked against
ldd install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent | grep fastrtps

# Should show either:
# - /home/rosuser/ros2_ws/src/fastdds_ws/install/lib/libfastrtps.so (custom)
# - /opt/ros/jazzy/lib/libfastrtps.so (ROS bundled)

# Source the workspace
source ~/ros2_ws/src/fastdds_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash

# Test
ros2 run micro_ros_agent micro_ros_agent --help
```

---

## Part 2: ArduPilot Configuration

### ArduPilot Serial Port Setup

ArduPilot needs to be configured to output micro-ROS data on a serial port or UDP.

#### Check ArduPilot Parameters

In your ArduPilot container, you need these parameters set:

```bash
# Access ArduPilot container
docker exec -it <ardupilot_container_name> bash

# Using MAVProxy or other GCS, set parameters:
# (Or edit parameters file directly)
```

**Key Parameters:**

```
# Enable DDS/micro-ROS on a serial port
SERIAL1_PROTOCOL = 45    # DDS Client (micro-ROS)
SERIAL1_BAUD = 921600    # High baud rate for DDS

# Or for UDP output (better for Docker):
# Enable DDS on network port
SERIALx_PROTOCOL = 45    # Where x is your available serial port
```

#### UDP Configuration (Recommended for Docker)

Since both containers use host network, UDP is simpler than serial passthrough:

**In ArduPilot**, you'll need to configure UDP output. The exact method depends on your ArduPilot setup:

**If using SITL (Software in the Loop):**
```bash
# ArduPilot SITL typically runs with:
sim_vehicle.py --console --map -A "--uartC=uart:127.0.0.1:2019"

# Or add to SITL parameters:
# This makes Serial3 output to UDP port 2019
SERIAL3_PROTOCOL = 45  # DDS Client
```

**If using hardware ArduPilot with serial-to-UDP bridge:**
```bash
# You might use socat or similar to bridge serial to UDP
socat TCP4-LISTEN:2019,fork,reuseaddr /dev/ttyUSB0,b921600,raw
```

---

## Part 3: Network Configuration (Host Mode)

Both containers are using `network_mode: host`, which means they share the host's network stack.

### Benefits:
- ✅ No port mapping needed
- ✅ Direct communication via localhost (127.0.0.1)
- ✅ ROS 2 DDS multicast works automatically
- ✅ Simplest configuration

### Network Topology:

```
Host Machine (Linux)
├── Network Interface: 127.0.0.1 (localhost)
│
├── ArduPilot Container (host network)
│   └── Listens on: 0.0.0.0:2019 (or outputs to 127.0.0.1:2019)
│
└── ROS 2 Container (host network)
    └── micro-ROS Agent listens on: 0.0.0.0:8888
```

### Port Assignments:

| Service | Port | Protocol | Direction |
|---------|------|----------|-----------|
| ArduPilot micro-ROS output | 2019 | UDP | ArduPilot → Agent |
| micro-ROS Agent | 8888 | UDP | Agent ← ArduPilot |
| ROS 2 DDS Discovery | 7400+ | UDP/Multicast | ROS ↔ ROS |
| VNC Server | 5901 | TCP | Client → Container |
| noVNC Web | 6080 | TCP | Browser → Container |

---

## Part 4: Running micro-ROS Agent

### Start the Agent

```bash
# Inside ROS 2 container
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash  # if built from source

# Run agent listening on UDP port 8888
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Alternative: Verbose mode for debugging
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

### Agent Command Options:

```bash
# UDP IPv4 (most common for ArduPilot)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# TCP (if ArduPilot configured for TCP)
ros2 run micro_ros_agent micro_ros_agent tcp4 --port 8888

# Serial (if using serial passthrough to container)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600

# Verbosity levels (-v1 to -v6, higher = more debug info)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

### Running as Background Service

To keep agent running permanently:

**Using systemd inside container** (if available):
```bash
# Create service file
sudo nano /etc/systemd/system/micro-ros-agent.service
```

```ini
[Unit]
Description=micro-ROS Agent for ArduPilot
After=network.target

[Service]
Type=simple
User=rosuser
WorkingDirectory=/home/rosuser
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source /home/rosuser/ros2_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Using tmux/screen** (simpler):
```bash
# Install tmux
sudo apt-get install tmux

# Start agent in tmux session
tmux new-session -d -s micro_ros_agent "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"

# Attach to see output
tmux attach -t micro_ros_agent

# Detach: Ctrl+B, then D
```

**Using ROS 2 launch file** (best practice):
```bash
# Create launch file
mkdir -p ~/ros2_ws/src/ardupilot_bringup/launch
nano ~/ros2_ws/src/ardupilot_bringup/launch/micro_ros_agent.launch.py
```

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='8888',
            description='UDP port for micro-ROS agent'
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['udp4', '--port', LaunchConfiguration('port'), '-v6'],
            respawn=True,
            respawn_delay=2.0
        )
    ])
```

```bash
# Run with launch file
ros2 launch ardupilot_bringup micro_ros_agent.launch.py
```

---

## Part 5: ArduPilot micro-ROS Connection

### ArduPilot Side Configuration

The exact configuration depends on your ArduPilot version. For **ArduPilot 4.5+** with DDS support:

**Parameters to set:**
```
# Enable DDS on Serial port
SERIAL3_PROTOCOL = 45      # DDS/micro-ROS
SERIAL3_BAUD = 921600      # High speed

# DDS configuration
DDS_ENABLE = 1             # Enable DDS subsystem
DDS_PORT = 2019            # UDP port to send to agent
DDS_IP0 = 127              # Target IP: 127.0.0.1
DDS_IP1 = 0
DDS_IP2 = 0
DDS_IP3 = 1

# Or if using external IP (between containers)
# DDS_IP would be the micro-ROS agent's IP
```

### For SITL (Simulation)

```bash
# In ArduPilot container, start SITL with DDS enabled
sim_vehicle.py -v ArduCopter --console --map \
  -A "--serial3=uart:127.0.0.1:2019" \
  --add-param-file=dds_params.parm

# Where dds_params.parm contains:
# SERIAL3_PROTOCOL 45
# SERIAL3_BAUD 921600
```

### Testing Connection

**1. Start micro-ROS agent in ROS container:**
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

**2. Start ArduPilot in ArduPilot container:**
```bash
# Your ArduPilot startup command with DDS enabled
```

**3. Check for topics in ROS container:**
```bash
# In another terminal in ROS container
source /opt/ros/jazzy/setup.bash

# List all topics
ros2 topic list

# Should see topics like:
# /fmu/battery/status
# /fmu/gps/position
# /fmu/imu/data
# /fmu/odometry
# etc.
```

---

## Part 6: Verification and Testing

### 1. Verify Network Connectivity

```bash
# In ROS container
netstat -uln | grep 8888
# Should show: udp 0.0.0.0:8888

# Test UDP reachability
nc -zvu 127.0.0.1 8888
```

### 2. Monitor micro-ROS Agent Output

With verbose mode (-v6), you should see:
```
[INFO] [micro_ros_agent]: Starting micro-ROS agent
[INFO] [micro_ros_agent]: UDP4 agent on port 8888
[INFO] [1234567890.123456] [micro_ros_agent]: session established
[INFO] [1234567890.234567] [micro_ros_agent]: CREATE_CLIENT request received
```

### 3. Check ROS 2 Topics

```bash
# List topics
ros2 topic list

# Echo a topic (example)
ros2 topic echo /fmu/battery/status

# Check topic info
ros2 topic info /fmu/battery/status

# Monitor all traffic
ros2 topic list -v
```

### 4. Verify DDS Communication

```bash
# Check DDS discovery
ros2 daemon stop
ros2 daemon start
ros2 topic list

# Monitor with rqt
rqt
# Go to Plugins → Topics → Topic Monitor
```

### 5. Test Bidirectional Communication

**Subscribe to ArduPilot topics:**
```bash
# Listen to IMU data
ros2 topic echo /fmu/imu/data
```

**Publish to ArduPilot** (if ArduPilot subscribes to commands):
```bash
# Example: Send velocity command
ros2 topic pub /fmu/velocity_setpoint geometry_msgs/msg/Twist \
  '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

---

## Part 7: Common Issues and Troubleshooting

### Issue 1: No Topics Appearing

**Symptoms:**
- `ros2 topic list` shows only `/parameter_events` and `/rosout`
- No `/fmu/*` topics

**Causes and Fixes:**

**A. Agent not receiving data from ArduPilot**
```bash
# Check if agent is receiving data (verbose mode)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6

# Check network connectivity
sudo tcpdump -i lo -n udp port 8888
# Should see packets if ArduPilot is sending
```

**B. ArduPilot not configured correctly**
```bash
# Verify ArduPilot parameters
# Check SERIAL_PROTOCOL = 45 on correct port
# Check DDS_ENABLE = 1
```

**C. Port mismatch**
```bash
# ArduPilot sends to port X, agent listens on port Y
# Make sure ports match:
# - ArduPilot DDS_PORT = 2019 (or your configured port)
# - Agent listens on --port 8888
# These don't need to match, but ArduPilot must send to agent's IP:port
```

**D. Firewall blocking (unlikely with host network)**
```bash
# Check firewall
sudo iptables -L -n | grep 8888
```

---

### Issue 2: Connection Drops Frequently

**Causes:**
- Network instability
- Agent timeout settings

**Fix:**
```bash
# Increase agent timeout
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --timeout 60000

# Use TCP instead of UDP (more reliable)
ros2 run micro_ros_agent micro_ros_agent tcp4 --port 8888
```

---

### Issue 3: Fast DDS Version Conflicts

**Symptoms:**
```
Error: Multiple definitions of target eProsima_atomic
```

**Fix:**
Use Option B (ROS Fast DDS) instead of custom Fast DDS, unless ArduPilot specifically requires a version.

---

### Issue 4: "Address already in use"

**Symptoms:**
```
Error: bind failed: Address already in use
```

**Fix:**
```bash
# Check what's using port 8888
sudo netstat -tulpn | grep 8888

# Kill the process or use different port
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8889
```

---

### Issue 5: ROS_DOMAIN_ID Mismatch

If you have multiple ROS systems:

```bash
# Ensure both containers use same domain
echo $ROS_DOMAIN_ID

# Set in both containers
export ROS_DOMAIN_ID=0

# Or in .bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

---

## Part 8: Integration with MAVROS2

You mentioned you already have MAVROS2 installed. Here's how micro-ROS and MAVROS2 work together:

### Comparison:

| Feature | MAVROS2 | micro-ROS Agent |
|---------|---------|-----------------|
| Protocol | MAVLink | micro-XRCE-DDS |
| ArduPilot Support | Full (traditional) | Newer (4.5+) |
| Data Access | MAVLink messages | Native ROS 2 topics |
| Overhead | Medium | Lower |
| Maturity | Very mature | Newer |

### Using Both Together:

**Scenario 1: MAVROS for control, micro-ROS for data**
```python
# Use MAVROS2 for:
# - Mode changes
# - Arming/disarming
# - Mission waypoints
# - Parameters

# Use micro-ROS for:
# - High-rate sensor data (IMU, GPS)
# - State estimation
# - Low-latency feedback
```

**Scenario 2: Gradual migration**
```python
# Start with MAVROS2 (proven)
# Add micro-ROS for specific high-performance topics
# Migrate functionality as you test
```

**Running both:**
```bash
# Terminal 1: MAVROS2
ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555

# Terminal 2: micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Both will coexist, publishing different topic namespaces
# MAVROS: /mavros/*
# micro-ROS: /fmu/*
```

---

## Part 9: Making it Permanent (Dockerfile)

Once you have a working setup, add to Dockerfile:

```dockerfile
# After Fast DDS build section:

# Clone micro-ROS agent
RUN cd /home/$USERNAME/ros2_ws/src \
    && git clone https://github.com/micro-ROS/micro-ROS-Agent.git \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws/src/micro-ROS-Agent

# Build micro-ROS agent (using whichever approach worked)
USER $USERNAME
RUN cd /home/$USERNAME/ros2_ws \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release
USER root

# Add to bashrc
RUN echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
```

---

## Part 10: Complete Startup Script

Create a startup script to launch everything:

```bash
nano ~/ros2_ws/start_ardupilot_bridge.sh
```

```bash
#!/bin/bash

# Source ROS environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Set ROS domain
export ROS_DOMAIN_ID=0

# Start micro-ROS agent in background
tmux new-session -d -s micro_ros "ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6"

echo "Micro-ROS Agent started in tmux session 'micro_ros'"
echo "Attach with: tmux attach -t micro_ros"
echo ""
echo "Waiting for ArduPilot connection on UDP port 8888..."
echo "Topics should appear in a few seconds after ArduPilot starts"
echo ""
echo "Monitor with: ros2 topic list"
```

```bash
chmod +x ~/ros2_ws/start_ardupilot_bridge.sh
```

---

## Quick Reference

### Start micro-ROS Agent:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Check Topics:
```bash
ros2 topic list
ros2 topic echo /fmu/battery/status
```

### Monitor Connection:
```bash
# Verbose agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6

# Network traffic
sudo tcpdump -i lo -n udp port 8888
```

### Debug:
```bash
# Check ROS environment
echo $ROS_DOMAIN_ID
ros2 doctor

# Check network
netstat -uln | grep 8888
```

---

## Resources

- **ArduPilot DDS Documentation**: https://ardupilot.org/dev/docs/ros2.html
- **micro-ROS Documentation**: https://micro.ros.org/
- **ROS 2 + ArduPilot Tutorial**: https://docs.px4.io/main/en/middleware/micrortps.html
- **Fast DDS Documentation**: https://fast-dds.docs.eprosima.com/

---

Let me know which installation option works and I'll help integrate it into the Dockerfile!
