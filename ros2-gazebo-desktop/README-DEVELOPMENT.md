# ROS 2 Jazzy Development Environment with micro-ROS Agent

Complete guide for setting up and using the ROS 2 Jazzy development environment with micro-ROS agent, MAVROS2, and custom package development.

## Table of Contents

- [Overview](#overview)
- [Pre-installed Packages](#pre-installed-packages)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Detailed Setup Instructions](#detailed-setup-instructions)
- [Development Workflow](#development-workflow)
- [Running micro-ROS Agent](#running-micro-ros-agent)
- [Running MAVROS2](#running-mavros2)
- [Troubleshooting](#troubleshooting)
- [Architecture](#architecture)

---

## Overview

This Docker environment provides a complete ROS 2 development setup with:
- **ROS 2 Jazzy Jalisco** (latest LTS)
- **Gazebo Harmonic** (latest simulator)
- **VNC Desktop** with XFCE (GUI access via browser)
- **micro-ROS agent** (for PX4 DDS communication)
- **MAVROS2** (for ArduPilot/PX4 MAVLink communication)
- **px4_msgs** (PX4 message definitions)
- **Development workspace support** (overlay your custom packages)

---

## Pre-installed Packages

The Docker image comes with these packages **already built and ready to use**:

### Flight Controller Communication
- **micro-ROS agent** (`micro_ros_agent`) - DDS/XRCE agent for PX4
- **micro-ROS msgs** (`micro_ros_msgs`) - Message definitions for micro-ROS
- **MAVROS2** (`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`) - MAVLink to ROS 2 bridge
- **px4_msgs** - PX4 message definitions for ROS 2

### Simulation & Visualization
- **Gazebo Harmonic** (`ros-jazzy-ros-gz-*`) - Robot simulator
- **RViz2** - 3D visualization tool
- **RQt** - Qt-based GUI tools

### Development Tools
- **colcon** - ROS 2 build tool
- **Python 3** - Python development
- **Git** - Version control
- **C++ toolchain** - GCC, CMake, etc.

---

## Prerequisites

### Required Software
- **Docker** (20.10 or later)
- **Docker Compose** (2.0 or later)

### System Requirements
- **Memory**: 8GB RAM minimum (4GB reserved for container)
- **Disk**: ~10GB free space
- **OS**: Linux, Windows (WSL2), or macOS with Docker Desktop

### Installation (if needed)

**Ubuntu/Debian:**
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Install Docker Compose
sudo apt-get install docker-compose-plugin

# Log out and back in for group changes
```

**Windows:**
- Install Docker Desktop for Windows
- Enable WSL2 backend

---

## Quick Start

For the impatient - get up and running in 5 minutes:

```bash
# 1. Navigate to the project directory
cd ros2-gazebo-desktop

# 2. Create development workspace on host
mkdir -p dev_ws/src

# 3. Build Docker image
docker compose -f docker-compose-simple.yml --env-file .env.vnc build

# 4. Start container
docker compose -f docker-compose-simple.yml --env-file .env.vnc up -d

# 5. Access VNC desktop
# Open browser: http://localhost:6080/vnc.html
# Password: rospassword

# 6. Enter container terminal
docker exec -it ros2_dev_station bash

# 7. Run micro-ROS agent
micro-ros-agent udp4 --port 8888
```

That's it! Now continue reading for detailed instructions.

---

## Detailed Setup Instructions

### Step 1: Clone/Navigate to Project Directory

```bash
# If cloning from git
git clone <your-repo-url>
cd ros_docker_images/ros2-gazebo-desktop

# Or navigate to existing directory
cd /path/to/ros_docker_images/ros2-gazebo-desktop
```

### Step 2: Create Development Workspace Structure

The development workspace is where **your custom packages** will live. This workspace is mounted from the host machine, so you can edit files with your favorite IDE.

```bash
# Create development workspace directory structure
mkdir -p dev_ws/src

# Verify structure
tree -L 2 .
# Expected output:
# .
# ├── dev_ws/
# │   └── src/
# ├── docker-compose-simple.yml
# ├── Dockerfile
# └── ... (other files)
```

**Important Notes:**
- `dev_ws/src/` - This is where you'll create your ROS 2 packages
- This directory is **mounted as a volume** - changes on host appear instantly in container
- The built-in packages (micro-ROS agent, px4_msgs) are in a **separate workspace** inside the container

### Step 3: Configure Environment (Optional)

Check the `.env.vnc` file for configuration:

```bash
cat .env.vnc
```

Default settings:
```bash
USERNAME=rosuser
USER_UID=1000
USER_GID=1000
CONTAINER_NAME=ros2_dev_station
VNC_PASSWORD=rospassword
VNC_RESOLUTION=1920x1080
ROS_DOMAIN_ID=0
```

**Customize if needed:**
```bash
# Edit .env.vnc
nano .env.vnc

# Change VNC password, resolution, etc.
VNC_PASSWORD=mypassword
VNC_RESOLUTION=2560x1440
```

### Step 4: Build Docker Image

This builds the Docker image with all pre-installed packages.

```bash
# Build the image (first time: 10-15 minutes)
docker compose -f docker-compose-simple.yml --env-file .env.vnc build

# Watch the build progress - you'll see:
# - Installing system packages
# - Installing ROS 2 packages (MAVROS2, Gazebo)
# - Building micro-ROS msgs
# - Building micro-ROS agent
# - Building px4_msgs
```

**Build Stages:**
```
[1/19] FROM osrf/ros:jazzy-desktop-full-noble
[2/19] Install Gazebo, XFCE, VNC
[3/19] Install MAVROS2
...
[8/19] Clone micro_ros_msgs, micro-ROS-Agent, px4_msgs
[9/19] Build micro_ros_msgs
[10/19] Build micro_ros_agent
[11/19] Build px4_msgs
...
[19/19] Setup VNC startup script
```

**Build Complete!** You should see:
```
✓ Container ros2-dev Built
```

### Step 5: Start Container

```bash
# Start container in detached mode
docker compose -f docker-compose-simple.yml --env-file .env.vnc up -d

# Verify container is running
docker ps

# Expected output:
# CONTAINER ID   IMAGE                              STATUS         PORTS     NAMES
# abc123def456   ros2-gazebo-desktop:jazzy-vnc-px4  Up 10 seconds             ros2_dev_station
```

### Step 6: Access the Environment

You have **three ways** to access the environment:

#### Option A: VNC Desktop (GUI - Recommended for Gazebo/RViz)

1. Open web browser
2. Navigate to: `http://localhost:6080/vnc.html`
3. Click "Connect"
4. Enter password: `rospassword` (or your custom password)
5. You'll see a full XFCE desktop!

**Desktop shortcuts available:**
- Terminal
- Gazebo
- RViz2
- RQt
- Firefox
- File Manager

#### Option B: Terminal (CLI - Recommended for Development)

```bash
# Enter container bash shell
docker exec -it ros2_dev_station bash

# You're now inside the container!
# Working directory: /home/rosuser/ros2_ws
```

#### Option C: Multiple Terminals

```bash
# Terminal 1 - Run micro-ROS agent
docker exec -it ros2_dev_station bash
micro-ros-agent udp4 --port 8888

# Terminal 2 - Run your code
docker exec -it ros2_dev_station bash
ros2 run my_package my_node

# Terminal 3 - Monitor topics
docker exec -it ros2_dev_station bash
ros2 topic list
```

---

## Development Workflow

### Understanding Workspace Layers

The environment uses **ROS 2 workspace overlays** - think of them as layers:

```
┌─────────────────────────────────────────────┐
│  Layer 3: Your Development Workspace        │
│  Location: ~/dev_ws (mounted from host)     │
│  Contains: Your custom packages             │
│  Editable: YES (edit on host)               │
├─────────────────────────────────────────────┤
│  Layer 2: Built-in Workspace               │
│  Location: ~/ros2_ws (inside container)     │
│  Contains: micro-ROS agent, px4_msgs        │
│  Editable: NO (pre-built in Docker image)   │
├─────────────────────────────────────────────┤
│  Layer 1: ROS 2 Base                        │
│  Location: /opt/ros/jazzy                   │
│  Contains: ROS core, MAVROS2, Gazebo        │
│  Editable: NO (system packages)             │
└─────────────────────────────────────────────┘
```

**Key Concept:** Higher layers can **use** packages from lower layers!

### Verify Built-in Packages

```bash
# Enter container
docker exec -it ros2_dev_station bash

# Check built-in workspace
ls ~/ros2_ws/
# Output: build/  install/  log/  src/

# List installed packages
ls ~/ros2_ws/install/
# Output:
# micro_ros_agent/
# micro_ros_msgs/
# px4_msgs/
# setup.bash

# Test micro-ROS agent is available
micro-ros-agent --help
# Should show help output ✅

# Test MAVROS2 is available
ros2 pkg list | grep mavros
# Should show mavros packages ✅
```

### Create Your First Custom Package

#### On Host Machine (Your Computer):

```bash
# Navigate to dev workspace source directory
cd /path/to/ros2-gazebo-desktop/dev_ws/src

# Option 1: Create C++ package
mkdir my_drone_controller
cd my_drone_controller

# Create package.xml
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_drone_controller</name>
  <version>0.0.1</version>
  <description>My custom drone controller</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>px4_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create CMakeLists.txt
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(my_drone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Add executable
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp std_msgs px4_msgs)

# Install
install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
EOF

# Create source directory and node
mkdir src
cat > src/controller_node.cpp << 'EOF'
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", 10,
      std::bind(&ControllerNode::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Controller node started!");
  }

private:
  void topic_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Vehicle arming state: %d", msg->arming_state);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
EOF
```

**Or Option 2: Use ros2 pkg create (easier):**

Inside the container:
```bash
docker exec -it ros2_dev_station bash
cd ~/dev_ws/src

# Create C++ package
ros2 pkg create --build-type ament_cmake my_drone_controller \
  --dependencies rclcpp std_msgs px4_msgs

# Or create Python package
ros2 pkg create --build-type ament_python my_python_controller \
  --dependencies rclpy px4_msgs micro_ros_msgs
```

### Build Your Custom Package

#### Inside Container:

```bash
# Enter container (if not already)
docker exec -it ros2_dev_station bash

# Navigate to development workspace
cd ~/dev_ws

# Source ROS 2 base
source /opt/ros/jazzy/setup.bash

# Source built-in workspace (IMPORTANT - must do this!)
source ~/ros2_ws/install/setup.bash

# Build your packages
colcon build

# If successful, you'll see:
# Summary: X packages finished [Ys]

# Source your development workspace
source install/setup.bash
```

**Important:** Always source in this order:
1. ROS 2 base (`/opt/ros/jazzy/setup.bash`)
2. Built-in workspace (`~/ros2_ws/install/setup.bash`)
3. Development workspace (`~/dev_ws/install/setup.bash`)

### Run Your Custom Package

```bash
# List your packages
ros2 pkg list | grep my_

# Run your node
ros2 run my_drone_controller controller_node

# Or with Python package
ros2 run my_python_controller my_node
```

### Iterative Development Cycle

1. **Edit code on host** (using VS Code, vim, etc.)
   ```bash
   # On host
   code dev_ws/src/my_drone_controller/src/controller_node.cpp
   # Make your changes, save
   ```

2. **Build in container**
   ```bash
   # In container
   cd ~/dev_ws
   source ~/ros2_ws/install/setup.bash  # Only needed once per terminal
   colcon build --packages-select my_drone_controller
   source install/setup.bash
   ```

3. **Test in container**
   ```bash
   ros2 run my_drone_controller controller_node
   ```

4. **Repeat!**

### Auto-source Workspaces (Optional but Recommended)

Add this to your `.bashrc` inside the container for automatic sourcing:

```bash
# Inside container
cat >> ~/.bashrc << 'EOF'

# Auto-source ROS 2 and workspaces
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "✓ Sourced built-in workspace (micro-ROS agent, px4_msgs)"
fi

if [ -f ~/dev_ws/install/setup.bash ]; then
    source ~/dev_ws/install/setup.bash
    echo "✓ Sourced development workspace"
fi

# Helpful aliases
alias build_dev='cd ~/dev_ws && colcon build && source install/setup.bash'
alias build_pkg='colcon build --packages-select'
EOF

# Reload bashrc
source ~/.bashrc
```

Now every new terminal will automatically have access to all packages!

```bash
# Open new terminal
docker exec -it ros2_dev_station bash

# Everything is already sourced!
micro-ros-agent --help  # ✅ Works
ros2 pkg list | grep my_  # ✅ Your packages available
```

---

## Running micro-ROS Agent

micro-ROS agent bridges between micro-ROS clients (like PX4) and ROS 2.

### Basic Usage

```bash
# Enter container
docker exec -it ros2_dev_station bash

# Run micro-ROS agent on UDP port 8888 (default for PX4)
micro-ros-agent udp4 --port 8888
```

**Expected output:**
```
[1234567890.123456] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1234567890.123789] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

### Connection Types

**UDP (most common for PX4 simulation):**
```bash
# Default UDP
micro-ros-agent udp4 --port 8888

# Specific IP and port
micro-ros-agent udp4 --port 8888 -a 0.0.0.0

# Verbose mode
micro-ros-agent udp4 --port 8888 -v 6
```

**Serial (for hardware flight controllers):**
```bash
# USB connection
micro-ros-agent serial --dev /dev/ttyUSB0 --baudrate 921600

# ACM devices (some Pixhawk boards)
micro-ros-agent serial --dev /dev/ttyACM0 --baudrate 921600

# Check available serial devices
ls /dev/tty*
```

**TCP:**
```bash
micro-ros-agent tcp4 --port 8888
```

### Configure PX4 to Connect

In PX4 (SITL or real hardware), set these parameters:

**For UDP:**
```bash
# In PX4 console or QGroundControl parameters:
param set XRCE_DDS_0_CFG 2        # UDP mode
param set XRCE_DDS_PRT 8888       # Port (must match agent)
param set XRCE_DDS_AG_IP 127.0.0.1  # Agent IP (use 127.0.0.1 for localhost)

# Save parameters
param save

# Restart PX4
reboot
```

**For Serial:**
```bash
param set XRCE_DDS_0_CFG 0        # Serial mode
param set XRCE_DDS_PRT /dev/ttyS0 # Serial port in PX4
```

### Verify Connection

Open a **second terminal**:

```bash
docker exec -it ros2_dev_station bash

# List ROS 2 topics - should see PX4 topics
ros2 topic list

# Expected output (if PX4 connected):
# /fmu/in/...
# /fmu/out/battery_status
# /fmu/out/sensor_combined
# /fmu/out/vehicle_status
# /fmu/out/vehicle_odometry
# ... (many more)

# Echo a topic to see data
ros2 topic echo /fmu/out/vehicle_status

# Check message rate
ros2 topic hz /fmu/out/sensor_combined
```

If you see topics and data, **micro-ROS agent is working!** ✅

### Background Execution

```bash
# Run in background
micro-ros-agent udp4 --port 8888 > /tmp/microros.log 2>&1 &

# Check if running
ps aux | grep micro-ros-agent

# View logs
tail -f /tmp/microros.log

# Stop background process
pkill micro-ros-agent
```

---

## Running MAVROS2

MAVROS2 bridges MAVLink (ArduPilot/PX4) to ROS 2.

### For ArduPilot

```bash
# Enter container
docker exec -it ros2_dev_station bash

# Launch MAVROS2 for ArduPilot
ros2 launch mavros apm.launch fcu_url:=udp://:14550@

# For serial connection
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600
```

### For PX4

```bash
ros2 launch mavros px4.launch fcu_url:=udp://:14550@
```

### Connection URLs

| Connection Type | URL Format | Example |
|----------------|------------|---------|
| UDP (receive) | `udp://:<local_port>@` | `udp://:14550@` |
| UDP (send) | `udp://<ip>:<port>@` | `udp://192.168.1.100:14550@` |
| TCP | `tcp://<ip>:<port>` | `tcp://127.0.0.1:5760` |
| Serial | `<device>:<baudrate>` | `/dev/ttyUSB0:921600` |

### Verify MAVROS2 Connection

```bash
# Check MAVROS2 topics
ros2 topic list | grep mavros

# Expected output:
# /mavros/battery
# /mavros/state
# /mavros/imu/data
# ... (many more)

# Check connection state
ros2 topic echo /mavros/state

# Monitor GPS
ros2 topic echo /mavros/global_position/global
```

### Example: Arm Vehicle via MAVROS2

```bash
# Check if armed
ros2 topic echo /mavros/state

# Arm the vehicle (CAREFUL - motors will spin!)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Disarm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

---

## Troubleshooting

### Container Won't Start

**Check Docker is running:**
```bash
docker ps
# If error, start Docker service
sudo systemctl start docker
```

**Check logs:**
```bash
docker compose -f docker-compose-simple.yml --env-file .env.vnc logs
```

**Rebuild from scratch:**
```bash
docker compose -f docker-compose-simple.yml --env-file .env.vnc down
docker compose -f docker-compose-simple.yml --env-file .env.vnc build --no-cache
docker compose -f docker-compose-simple.yml --env-file .env.vnc up -d
```

### VNC Won't Connect

**Check container is running:**
```bash
docker ps | grep ros2_dev_station
```

**Check port 6080 is available:**
```bash
sudo netstat -tulpn | grep 6080
```

**Try connecting directly via VNC client:**
- Host: `localhost:5901`
- Password: `rospassword`

### micro-ROS Agent: "Command not found"

**Source the workspace:**
```bash
source ~/ros2_ws/install/setup.bash
micro-ros-agent --help
```

**If still not found, check if it was built:**
```bash
ls ~/ros2_ws/install/
# Should see: micro_ros_agent/

# Rebuild if missing
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select micro_ros_agent
source install/setup.bash
```

### No PX4 Topics Appear

**Check micro-ROS agent is running:**
```bash
ps aux | grep micro-ros-agent
```

**Check PX4 parameters:**
```bash
# In PX4 console
param show XRCE_DDS_0_CFG  # Should be 2 for UDP
param show XRCE_DDS_PRT    # Should be 8888
```

**Check network connectivity:**
```bash
# In container
sudo netstat -tulpn | grep 8888
# Should show micro-ros-agent listening
```

**Check ROS_DOMAIN_ID matches:**
```bash
# In container
echo $ROS_DOMAIN_ID  # Should be 0 (default)
```

### Build Errors in dev_ws

**Missing dependencies:**
```bash
# Make sure you sourced built-in workspace!
source ~/ros2_ws/install/setup.bash

# Then rebuild
cd ~/dev_ws
colcon build
```

**Package not found:**
```bash
# Check package is available
ros2 pkg list | grep <package_name>

# If in built-in workspace
ls ~/ros2_ws/install/

# If it's your package
ls ~/dev_ws/src/
```

**Clean build:**
```bash
cd ~/dev_ws
rm -rf build/ install/ log/
colcon build
```

### Permission Errors

**Fix ownership of dev_ws:**
```bash
# On host
sudo chown -R $USER:$USER dev_ws/

# Or match container user (1000:1000 by default)
sudo chown -R 1000:1000 dev_ws/
```

---

## Architecture

### Directory Structure

```
ros2-gazebo-desktop/
├── Dockerfile                      # Main Docker image definition
├── Dockerfile.companion            # Companion computer image
├── docker-compose-simple.yml       # Simple single-container setup
├── docker-compose-px4.yml          # PX4 dual-container setup
├── docker-compose-companion.yml    # Companion-only setup
├── startup-vnc.sh                  # VNC startup script
├── .env.vnc                        # Environment variables
├── README-DEVELOPMENT.md           # This file
│
├── dev_ws/                         # YOUR development workspace (host)
│   ├── src/                        # Your custom packages
│   ├── build/                      # Build artifacts (generated)
│   ├── install/                    # Installed packages (generated)
│   └── log/                        # Build logs (generated)
│
└── ros2_ws/                        # Optional - can be empty or not exist
    └── src/                        # (Not used when container runs)
```

**Inside Container:**
```
/home/rosuser/
├── ros2_ws/                        # Built-in workspace (in image)
│   ├── src/
│   │   ├── micro_ros_msgs/         # micro-ROS message definitions
│   │   ├── micro-ROS-Agent/        # micro-ROS agent source
│   │   └── px4_msgs/               # PX4 message definitions
│   ├── build/                      # Build artifacts
│   ├── install/                    # Installed packages
│   │   ├── micro_ros_agent/
│   │   ├── micro_ros_msgs/
│   │   ├── px4_msgs/
│   │   └── setup.bash              # Source this first!
│   └── log/
│
├── dev_ws/                         # Development workspace (mounted from host)
│   ├── src/
│   │   └── your_packages/          # Your code (editable on host)
│   ├── build/
│   ├── install/
│   │   └── setup.bash              # Source this after ros2_ws!
│   └── log/
│
└── Desktop/                        # Desktop shortcuts
    ├── terminal.desktop
    ├── gazebo.desktop
    └── rviz2.desktop
```

### Network Ports

| Port | Service | Access |
|------|---------|--------|
| 5901 | VNC Server | VNC client: `localhost:5901` |
| 6080 | noVNC (web) | Browser: `http://localhost:6080/vnc.html` |
| 8888 | micro-ROS agent | PX4 XRCE-DDS connection |
| 14550 | MAVROS2 | MAVLink (ArduPilot/PX4) |

### System Specifications

- **Base Image**: `osrf/ros:jazzy-desktop-full-noble`
- **OS**: Ubuntu 24.04 Noble
- **ROS 2 Distribution**: Jazzy Jalisco
- **Gazebo Version**: Harmonic
- **Desktop Environment**: XFCE 4
- **DDS Implementation**: Fast DDS 2.x (system)

---

## Common Use Cases

### Use Case 1: Test micro-ROS Agent with PX4 SITL

```bash
# Terminal 1: Start micro-ROS agent
docker exec -it ros2_dev_station bash
micro-ros-agent udp4 --port 8888

# Terminal 2: Run PX4 SITL (outside container or in separate PX4 container)
# Configure PX4 to connect to localhost:8888

# Terminal 3: Monitor topics
docker exec -it ros2_dev_station bash
ros2 topic list
ros2 topic echo /fmu/out/vehicle_status
```

### Use Case 2: Develop Custom PX4 Controller

```bash
# 1. Create package on host
cd dev_ws/src
ros2 pkg create --build-type ament_cmake px4_offboard_controller \
  --dependencies rclcpp px4_msgs

# 2. Write code in your IDE on host
code dev_ws/src/px4_offboard_controller/

# 3. Build in container
docker exec -it ros2_dev_station bash
cd ~/dev_ws
source ~/ros2_ws/install/setup.bash
colcon build --packages-select px4_offboard_controller
source install/setup.bash

# 4. Run your controller
ros2 run px4_offboard_controller offboard_node
```

### Use Case 3: Run MAVROS2 with ArduPilot

```bash
# Terminal 1: Start MAVROS2
docker exec -it ros2_dev_station bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@

# Terminal 2: Monitor data
docker exec -it ros2_dev_station bash
ros2 topic echo /mavros/state
ros2 topic echo /mavros/global_position/global
```

---

## Advanced Topics

### Building Multiple Packages

```bash
cd ~/dev_ws

# Build all packages
colcon build

# Build specific packages
colcon build --packages-select pkg1 pkg2 pkg3

# Build up to a package (including dependencies)
colcon build --packages-up-to my_package

# Parallel builds (faster)
colcon build --parallel-workers 4
```

### Debugging

```bash
# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run with GDB
ros2 run --prefix 'gdb -ex run --args' my_package my_node

# Run with Valgrind
ros2 run --prefix 'valgrind' my_package my_node
```

### Using Gazebo

```bash
# Launch Gazebo from desktop shortcut or terminal
gz sim

# Or with a specific world
gz sim <world_file.sdf>

# List available worlds
ls /usr/share/gazebo/worlds/
```

### Recording Data

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /fmu/out/vehicle_status /fmu/out/sensor_combined

# Play back
ros2 bag play <bag_file>
```

---

## Getting Help

### Documentation
- **ROS 2 Jazzy**: https://docs.ros.org/en/jazzy/
- **micro-ROS**: https://micro.ros.org/
- **MAVROS**: https://github.com/mavlink/mavros/tree/ros2
- **PX4**: https://docs.px4.io/
- **Gazebo**: https://gazebosim.org/

### Check Installed Versions

```bash
# ROS 2 version
printenv ROS_DISTRO  # jazzy

# Gazebo version
gz sim --version

# micro-ROS agent version
micro-ros-agent --version

# Python version
python3 --version

# Check all ROS packages
ros2 pkg list
```

---

## Summary Cheat Sheet

**Start Container:**
```bash
docker compose -f docker-compose-simple.yml --env-file .env.vnc up -d
```

**Access Container:**
```bash
docker exec -it ros2_dev_station bash
```

**Access VNC:**
```
http://localhost:6080/vnc.html
```

**Run micro-ROS Agent:**
```bash
micro-ros-agent udp4 --port 8888
```

**Run MAVROS2:**
```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```

**Build Your Packages:**
```bash
cd ~/dev_ws
source ~/ros2_ws/install/setup.bash
colcon build
source install/setup.bash
```

**Stop Container:**
```bash
docker compose -f docker-compose-simple.yml --env-file .env.vnc down
```

---

## License

This project follows the same license as ROS 2 Jazzy (Apache 2.0).

## Contributing

Contributions are welcome! Please follow ROS 2 coding standards.

---

**Happy Developing! 🚀**

For questions or issues, please refer to the documentation links above or check the troubleshooting section.
