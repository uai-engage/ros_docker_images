# Companion Computer Quick Start Guide

This guide shows you how to quickly set up a minimal micro-ROS agent installation on your drone's companion computer (Raspberry Pi, Jetson, etc.).

## What's Included

This directory contains everything you need for companion computer setup:

- **`Dockerfile`** - Minimal Docker image (~800 MB vs 5.5 GB desktop)
- **`docker-compose.yml`** - Docker Compose configuration with serial access
- **`.env.example`** - Environment configuration template
- **`systemd/micro-ros-agent.service`** - Auto-start service for native installation
- **`scripts/start_agent.sh`** - Helper script for manual startup
- **`docs/COMPANION_COMPUTER_SETUP.md`** - Complete detailed guide
- **`docs/SITL_TESTING_GUIDE.md`** - ArduPilot SITL testing guide

## Architecture Overview

```
┌─────────────────────────────────────────┐
│         DRONE (In Flight)               │
│  ┌──────────────┐  ┌─────────────────┐ │
│  │Flight Ctrl   │◄►│Companion Computer│ │
│  │(ArduPilot)   │  │micro-ROS Agent   │ │
│  │ USB/Serial   │  │(Minimal Install) │ │
│  └──────────────┘  └────────┬─────────┘ │
└─────────────────────────────┼───────────┘
                              │ Radio Link
┌─────────────────────────────┼───────────┐
│       GROUND STATION        │           │
│  ┌──────────────────────────▼────────┐  │
│  │ ROS 2 Desktop (Full Installation) │  │
│  │ - Gazebo, RViz2, RQt, Analysis    │  │
│  └────────────────────────────────────┘  │
└──────────────────────────────────────────┘
```

## Quick Start - Choose Your Installation Method

### Option 1: Docker (Recommended)

**Easiest to update and manage**

```bash
# 1. Copy entire companion directory to your companion computer
scp -r companion/ ubuntu@companion-computer:~/

# 2. SSH to companion computer
ssh ubuntu@companion-computer
cd ~/companion

# 3. Find your serial device (for serial mode)
ls -l /dev/tty*
# Look for: /dev/ttyUSB0, /dev/ttyACM0, or /dev/ttyAMA0

# 4. Configure (optional - defaults work for SITL UDP mode)
cp .env.example .env
nano .env  # Edit if needed

# 5. Build and run
docker compose build
docker compose up -d

# 6. Check logs
docker logs -f micro_ros_agent_companion

# 7. Verify topics on ground station
ros2 topic list  # Should see /fmu/* topics
```

**Stop the agent:**
```bash
docker compose down
```

---

### Option 2: Native Installation (Best Performance)

**For maximum performance on resource-constrained hardware**

```bash
# 1. Install ROS 2 Base (minimal)
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-ros-base

# 2. Install micro-ROS dependencies
sudo apt install -y \
    ros-jazzy-micro-ros-msgs \
    ros-jazzy-micro-ros-diagnostic-msgs \
    ros-jazzy-micro-ros-diagnostic-bridge

# 3. Build micro-ROS agent
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent.git
cd micro-ROS-Agent
git checkout jazzy

cd ~/micro_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. Setup environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/micro_ros_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Copy and use the helper script
chmod +x scripts/start_agent.sh
./scripts/start_agent.sh
```

**Auto-start on boot:**
```bash
# Install systemd service
sudo cp systemd/micro-ros-agent.service /etc/systemd/system/

# Edit service file with your username and device
sudo nano /etc/systemd/system/micro-ros-agent.service

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable micro-ros-agent.service
sudo systemctl start micro-ros-agent.service

# Check status
sudo systemctl status micro-ros-agent.service
```

---

## Flight Controller Configuration

**ArduPilot parameters (set via Mission Planner / QGroundControl):**

```
SERIAL3_PROTOCOL = 45      # DDS (micro-ROS)
SERIAL3_BAUD = 921600      # High speed
DDS_ENABLE = 1             # Enable DDS output
```

**Hardware connection:**
- Flight Controller USB → Companion Computer USB
- Or TELEM port → Companion GPIO UART

---

## Serial Device Setup

**Find your serial device:**
```bash
# List all serial devices
ls -l /dev/tty*

# Watch for device when you plug in
dmesg | grep tty
```

**Common devices:**
- `/dev/ttyUSB0` - USB serial adapter
- `/dev/ttyACM0` - USB CDC (most ArduPilot boards)
- `/dev/ttyAMA0` - Raspberry Pi UART

**Set permissions:**
```bash
# Add user to dialout group
sudo usermod -aG dialout $USER

# Or set permissions directly (temporary)
sudo chmod 666 /dev/ttyUSB0

# Make permanent (udev rule)
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-serial.rules
sudo udevadm control --reload-rules
```

---

## Helper Script Usage

The `scripts/start_agent.sh` script provides flexible startup options:

```bash
# Serial mode (default)
./scripts/start_agent.sh

# Custom serial device
./scripts/start_agent.sh -d /dev/ttyACM0

# Custom baud rate
./scripts/start_agent.sh -b 115200

# UDP mode (for testing over network)
./scripts/start_agent.sh -m udp4 -p 2019

# Verbose logging (debugging)
./scripts/start_agent.sh -v

# See all options
./scripts/start_agent.sh --help
```

**Environment variable configuration:**
```bash
# Set environment variables
export CONNECTION_MODE=serial
export SERIAL_DEVICE=/dev/ttyUSB0
export BAUD_RATE=921600

# Run script
./scripts/start_agent.sh
```

---

## Verification

**On companion computer:**
```bash
# Check agent is running
ps aux | grep micro_ros_agent

# Check port (if using UDP)
netstat -tulpn | grep 2019
```

**On ground station:**
```bash
# List topics (should see /fmu/* topics)
ros2 topic list

# Echo a topic
ros2 topic echo /fmu/battery/status

# Check data rate
ros2 topic hz /fmu/imu/data
```

---

## Network Configuration (Companion ↔ Ground)

**Ensure both systems:**
1. Are on same network (WiFi, LTE, radio link)
2. Use same `ROS_DOMAIN_ID` (default: 0)
3. Allow UDP multicast traffic
4. Have reachable IP addresses

**On companion computer:**
```bash
export ROS_DOMAIN_ID=0
```

**On ground station:**
```bash
export ROS_DOMAIN_ID=0
```

**Test connectivity:**
```bash
# From ground station, ping companion
ping <companion-ip>

# Check ROS discovery
ros2 topic list  # Should see /fmu/* topics
```

---

## Troubleshooting

### Agent won't start
```bash
# Check serial device exists
ls -l /dev/ttyUSB0

# Check permissions
sudo chmod 666 /dev/ttyUSB0
sudo usermod -aG dialout $USER

# Check if already running
ps aux | grep micro_ros_agent
```

### No topics on ground station
```bash
# Check same ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Should be same on both systems

# Check network connectivity
ping <companion-ip>

# Restart ROS daemon
ros2 daemon stop
ros2 daemon start

# Check agent is running on companion
ssh ubuntu@companion-computer "ps aux | grep micro_ros_agent"
```

### Ghost topics after stopping
```bash
# Clear DDS cache
ros2 daemon stop
ros2 daemon start

# Or use stop script
./stop_micro_ros.sh
```

### Serial connection fails
```bash
# Check ArduPilot parameters
# SERIAL3_PROTOCOL = 45
# SERIAL3_BAUD = 921600
# DDS_ENABLE = 1

# Check baud rate matches
# Agent: -b 921600
# ArduPilot: SERIAL3_BAUD = 921600

# Try verbose mode
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6
```

---

## Resource Usage Comparison

| System | Disk | RAM | Purpose |
|--------|------|-----|---------|
| **Ground Station** (Full) | 5-10 GB | 2-4 GB | Monitoring, visualization, control |
| **Companion** (Minimal) | 0.5-1 GB | 200-500 MB | Data bridge only |

**Savings: 85% smaller!**

---

## Files Reference

- **`Dockerfile`** - Minimal Docker image for companion
- **`docker-compose.yml`** - Docker Compose with serial access
- **`.env.example`** - Environment configuration template
- **`systemd/micro-ros-agent.service`** - Systemd service for auto-start
- **`scripts/start_agent.sh`** - Flexible startup script
- **`docs/COMPANION_COMPUTER_SETUP.md`** - Detailed setup guide (all options)
- **`docs/SITL_TESTING_GUIDE.md`** - ArduPilot SITL testing guide
- **`../scripts/stop_micro_ros.sh`** - Stop agent and clear ghost topics (in parent dir)

---

## Next Steps

1. Choose installation method (Docker or Native)
2. Set up serial connection to flight controller
3. Configure network link to ground station
4. Test end-to-end communication
5. (Optional) Set up auto-start on boot

For more detailed information, see **`docs/COMPANION_COMPUTER_SETUP.md`**.
