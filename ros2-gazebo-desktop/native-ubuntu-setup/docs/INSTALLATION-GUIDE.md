# Native Ubuntu Setup for ROS 2 + Gazebo + PX4

Complete guide to install ROS 2 Jazzy, Gazebo Harmonic, and PX4 Autopilot v1.16.0 directly on Ubuntu 24.04 Desktop.

## Prerequisites

- **Ubuntu 24.04 LTS Desktop** (Noble Numbat)
- **Minimum 8GB RAM** (16GB recommended)
- **30GB free disk space**
- **Internet connection**
- **sudo privileges**

## Quick Start

Run the installation scripts in order:

```bash
cd native-ubuntu-setup/scripts

# 1. Install ROS 2 Jazzy (15-20 minutes)
./01-install-ros2-jazzy.sh

# 2. Install Gazebo Harmonic (10-15 minutes)
./02-install-gazebo-harmonic.sh

# 3. Install PX4 dependencies (5-10 minutes)
./03-install-px4-deps.sh

# 4. Clone and build PX4 Autopilot (20-30 minutes)
./04-setup-px4.sh

# 5. Install micro-ROS agent (5 minutes)
./05-install-microros.sh

# Total time: ~1-1.5 hours
```

## What Gets Installed

### 1. ROS 2 Jazzy
- ROS 2 Desktop Full
- Development tools (colcon, rosdep)
- ROS 2 Gazebo integration packages
- MAVROS2 (optional, for MAVLink ↔ ROS 2 bridge)

### 2. Gazebo Harmonic
- Gazebo Sim (gz-sim)
- Gazebo Transport
- All required Gazebo libraries
- Command-line tools (gz)

### 3. PX4 Autopilot
- PX4 v1.16.0 source code
- All submodules (heatshrink, etc.)
- SITL (Software-In-The-Loop) build
- Gazebo models and worlds
- MAVLink tools

### 4. Dependencies
- Python 3.12 packages (numpy, jinja2, empy, etc.)
- Build tools (cmake, ninja, gcc)
- Libraries (OpenCV, ZeroMQ, GStreamer)

### 5. micro-ROS Agent
- Pre-built micro-ROS agent for PX4 ↔ ROS 2 communication
- Connects via uXRCE-DDS (UDP port 8888)

## Directory Structure

After installation:

```
~/px4_workspace/
├── PX4-Autopilot/              # PX4 source code
│   ├── build/px4_sitl_default/ # SITL build
│   ├── Tools/simulation/gz/   # Gazebo models & worlds
│   └── ...
├── micro_ros_ws/               # micro-ROS agent workspace
│   └── ...
└── ros2_ws/                    # Your ROS 2 workspace (optional)
```

## Running the System

### Terminal 1: Start Gazebo
```bash
source /opt/ros/jazzy/setup.bash
gz sim -r ~/px4_workspace/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
```

### Terminal 2: Start PX4 SITL
```bash
cd ~/px4_workspace/PX4-Autopilot
make px4_sitl gz_x500
```

### Terminal 3: Start micro-ROS Agent (for ROS 2 topics)
```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

### Terminal 4: Monitor ROS 2 Topics
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_status
```

## Connect QGroundControl

### From Same Machine (Local):
- QGroundControl will auto-discover on UDP 14550

### From Remote Machine (Windows):
1. Note your Ubuntu machine's IP: `ip addr show | grep inet`
2. In QGroundControl:
   - Type: **UDP**
   - Port: **14550**
   - Server: **<ubuntu-ip>**

Or use **TCP** (more reliable):
- Type: **TCP**
- Server: **<ubuntu-ip>**
- Port: **5760**

## Environment Setup

Add these to your `~/.bashrc` for convenience:

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# PX4 workspace
export PX4_HOME=~/px4_workspace/PX4-Autopilot
export PATH=$PX4_HOME/Tools:$PATH

# Gazebo resource paths
export GZ_SIM_RESOURCE_PATH=$PX4_HOME/Tools/simulation/gz/models:$PX4_HOME/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH

# Convenience aliases
alias px4='cd ~/px4_workspace/PX4-Autopilot && make px4_sitl gz_x500'
alias gz-sim='gz sim'
alias microros='ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888'
```

## Troubleshooting

### Gazebo won't start
```bash
# Check if installed
gz sim --version

# If missing, reinstall
cd native-ubuntu-setup/scripts
./02-install-gazebo-harmonic.sh
```

### PX4 build fails
```bash
# Clean and rebuild
cd ~/px4_workspace/PX4-Autopilot
make distclean
make px4_sitl
```

### No ROS 2 topics from PX4
```bash
# Check micro-ROS agent is running
ps aux | grep micro_ros_agent

# Check PX4 uXRCE-DDS client
# In PX4 console: uxrce_dds_client status
```

### QGroundControl can't connect
```bash
# Check PX4 is running
ps aux | grep px4

# Check MAVLink ports
netstat -tulpn | grep -E "14550|14540|5760"

# Open firewall (if needed)
sudo ufw allow 14550/udp
sudo ufw allow 5760/tcp
```

## Next Steps

After installation, see:
- `USAGE-GUIDE.md` - Detailed usage instructions
- `QUICK-START.md` - Common commands and workflows
- `EXAMPLES.md` - Example missions and code

## System Requirements Check

Before installation, run:
```bash
./00-check-system.sh
```

This verifies:
- Ubuntu version (24.04 required)
- Available disk space (30GB minimum)
- RAM (8GB minimum)
- Internet connectivity

## Manual Installation

If you prefer manual installation, see:
- `docs/MANUAL-INSTALLATION.md` - Step-by-step manual guide

## Uninstallation

To completely remove:
```bash
./99-uninstall-all.sh  # Removes ROS 2, Gazebo, PX4
```

**Warning**: This removes ALL installations including ROS 2 and Gazebo!
