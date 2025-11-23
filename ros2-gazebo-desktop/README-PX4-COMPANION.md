# ROS 2 Jazzy PX4/ArduPilot Development Environment
# Ground Station + Companion Computer Setup

Complete Docker-based development environment for PX4 and ArduPilot with ROS 2 Jazzy, featuring a ground station with GUI and a companion computer for flight controller communication.

## 🎯 Quick Start - Which Setup Do You Need?

### ⚡ Setup 1: Development (Most Users Start Here)

**Use:** `docker-compose-simple.yml` - **Single Container (Ground Station Only)**

```bash
# Start single container with everything
docker compose -f docker-compose-simple.yml up -d

# Access VNC: http://localhost:6080/vnc.html
```

**You get:**
- ✅ Full desktop GUI (Gazebo, RViz2, RQt)
- ✅ micro-ROS agent for PX4
- ✅ MAVROS2 for ArduPilot
- ✅ px4_msgs package
- ✅ All tools in one container

**Use this for:**
- Local development on your laptop/desktop
- PX4 SITL testing
- ArduPilot SITL testing
- Hardware flight controller via USB/Serial
- Learning and experimentation

---

### 🚁 Setup 2: Production - Dual Container (Both Machines Same Location)

**Use:** `docker-compose-px4.yml` - **Ground Station + Companion**

```bash
# Start both containers on same machine
docker compose -f docker-compose-px4.yml up -d
```

**You get:**
- Ground Station (GUI, visualization)
- Companion (FC communication)
- Automatic ROS 2 topic bridging via host network
- Separation of concerns for testing

**Use this for:**
- Testing dual-container architecture on one machine
- Simulating real system architecture
- Development of distributed system
- Preparing for separate deployment

---

### 🛸 Setup 3: Production - Companion Only (Separate Machines)

**Use:** `docker-compose-companion.yml` - **Companion Computer Only**

```bash
# On onboard computer (Raspberry Pi, Jetson, etc.)
cp .env.companion .env
docker compose -f docker-compose-companion.yml build
docker compose -f docker-compose-companion.yml up -d

# On ground station (your laptop) - run Setup 1
docker compose -f docker-compose-simple.yml up -d
```

**You get:**
- Lightweight companion on drone/vehicle
- Ground station on separate laptop
- ROS 2 topics auto-discovered over WiFi/Network
- True distributed system

**Use this for:**
- Real onboard computer deployment (Raspberry Pi, Jetson)
- Production drone/vehicle
- Separate ground station and onboard computer
- Remote operation over network

---

## 🏗️ Architecture Overview

### Single Container Setup (Development)

**File:** `docker-compose-simple.yml`

```
┌─────────────────────────────────────────────────────┐
│          Your Development Machine / Laptop          │
│                                                     │
│  ┌─────────────────────────────────────────────┐   │
│  │   ROS 2 Development Container               │   │
│  │   (ros2_dev_station)                        │   │
│  │                                             │   │
│  │  ✅ Gazebo Harmonic                         │   │
│  │  ✅ RViz2, RQt                              │   │
│  │  ✅ VNC Desktop (XFCE)                      │   │
│  │  ✅ micro-ROS Agent (for PX4)               │   │
│  │  ✅ MAVROS2 (for ArduPilot)                 │   │
│  │  ✅ px4_msgs                                │   │
│  │  ✅ Fast DDS                                │   │
│  │                                             │   │
│  │  All ROS 2 tools available locally          │   │
│  └──────────────────┬──────────────────────────┘   │
│                     │                              │
│                     │ UDP/Serial/USB               │
│                     ▼                              │
│            ┌─────────────────┐                     │
│            │ Flight Controller│                    │
│            │  • PX4 (SITL/HW) │                    │
│            │  • ArduPilot     │                    │
│            └─────────────────┘                     │
│                                                     │
│  Access: http://localhost:6080/vnc.html            │
└─────────────────────────────────────────────────────┘
```

**All in one container** - micro-ROS agent runs inside, publishes topics locally. Perfect for development!

### Dual Container Setup (Production)

**File:** `docker-compose-px4.yml`

```
┌────────────────────────┐         ┌─────────────────────────┐
│   Onboard Computer     │         │   Ground Station        │
│   (Raspberry Pi, etc)  │  WiFi/  │   (Your Laptop)         │
│                        │ Network │                         │
│  ┌──────────────────┐  │  ROS2   │  ┌───────────────────┐  │
│  │ Companion        │  │ Topics  │  │ Desktop Container │  │
│  │ Container        │◄─┼────────►┼─►│                   │  │
│  │                  │  │         │  │ • Gazebo          │  │
│  │ • micro-ROS      │  │         │  │ • RViz2           │  │
│  │   Agent          │  │         │  │ • VNC Desktop     │  │
│  │ • MAVROS2        │  │         │  │ • px4_msgs        │  │
│  │ • px4_msgs       │  │         │  │ • MAVROS2         │  │
│  │                  │  │         │  │ • Monitoring      │  │
│  └────────┬─────────┘  │         │  └───────────────────┘  │
│           │            │         │                         │
│           │ USB/Serial │         │  VNC: :6080             │
│           ▼            │         └─────────────────────────┘
│  ┌─────────────────┐   │
│  │ Flight Ctrl (HW)│   │
│  │  • PX4          │   │
│  │  • ArduPilot    │   │
│  └─────────────────┘   │
└────────────────────────┘
```

**Separate containers** - Companion on drone, Ground on laptop. Topics automatically bridge via ROS 2 DDS.

### Companion Only Setup (Production - Separate Machines)

**File:** `docker-compose-companion.yml`

```
Machine 1: Onboard Computer          Machine 2: Ground Station
(Raspberry Pi / Jetson)               (Your Laptop / Desktop)
┌────────────────────────┐            ┌──────────────────────────┐
│  Companion Container   │            │  Dev Container           │
│                        │   WiFi/    │  (docker-compose-        │
│  • micro-ROS Agent     │◄─Network──►│   simple.yml)            │
│  • MAVROS2             │   ROS 2    │                          │
│  • px4_msgs            │   Topics   │  • Gazebo, RViz2         │
│  • Lightweight         │            │  • VNC Desktop           │
│                        │            │  • px4_msgs              │
│  Port: None (host net) │            │  • Full GUI              │
└───────────┬────────────┘            │                          │
            │                         │  VNC: :6080              │
            │ USB/Serial              └──────────────────────────┘
            ▼
   ┌─────────────────┐
   │ Flight Ctrl (HW)│
   │  • PX4          │
   │  • ArduPilot    │
   └─────────────────┘
```

**Fully distributed** - Companion physically on drone, Ground station on separate laptop. Perfect for real deployments.

**Key Requirements:**
- Both machines on same network (WiFi/Ethernet)
- Same `ROS_DOMAIN_ID` on both (default: 0)
- Network allows UDP multicast (ports 7400-7500)
- Companion: `docker-compose-companion.yml`
- Ground: `docker-compose-simple.yml`

## 📦 What's Included

### Ground Station Container
- **ROS 2 Jazzy Desktop Full** - Complete ROS 2 installation
- **Gazebo Harmonic** - Latest simulation environment
- **VNC + XFCE Desktop** - Full GUI accessible remotely
- **px4_msgs** - PX4 message definitions for monitoring
- **MAVROS2** - For ArduPilot monitoring and visualization
- **RViz2, RQt** - Visualization and debugging tools
- **Desktop shortcuts** - Quick access to common tools

### Companion Computer Container
- **ROS 2 Jazzy Base** - Lightweight ROS 2 installation
- **micro-ROS Agent** - Native PX4 DDS communication
- **px4_msgs** - PX4 message definitions
- **MAVROS2** - ArduPilot MAVLink bridge
- **Fast DDS** - Built from source for compatibility
- **Serial device access** - Direct FC connection support

## 🧪 Complete Testing Guide

### Test Setup 1: Single Container (Development) - Recommended First

**What you need:** Just your computer + this Docker setup

**Steps:**

```bash
# 1. Navigate to directory
cd ros2-gazebo-desktop/

# 2. Build the container
docker compose -f docker-compose-simple.yml build

# 3. Start the container
docker compose -f docker-compose-simple.yml up -d

# 4. Access VNC in browser
# Open: http://localhost:6080/vnc.html
# Password: rospassword
```

**Testing PX4 (SITL):**

```bash
# Terminal 1: Start PX4 SITL (outside container, on your host)
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Wait for PX4 to start, you'll see:
# "INFO [simulator_mavlink] Waiting for simulator to accept connection on TCP port 4560"

# Terminal 2: Inside VNC desktop or via docker exec
docker exec -it ros2_dev_station bash

# Start micro-ROS agent
source /home/rosuser/ros2_ws/src/fastdds_ws/install/setup.bash
source /home/rosuser/ros2_ws/install/setup.bash
micro-ros-agent udp4 -p 8888

# You should see: "Micro XRCE-DDS Agent running..."

# Terminal 3: Inside VNC desktop (open new terminal in VNC)
# Or another docker exec:
docker exec -it ros2_dev_station bash

# Verify topics
ros2 topic list | grep fmu
# You should see: /fmu/out/vehicle_status, /fmu/out/vehicle_local_position, etc.

# Echo a topic
ros2 topic echo /fmu/out/vehicle_status

# Success! You're receiving PX4 messages
```

**Testing ArduPilot (SITL):**

```bash
# Terminal 1: Start ArduPilot SITL (outside container, on your host)
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

# Wait for ArduPilot to start broadcasting MAVLink on UDP :14550

# Terminal 2: Inside VNC or docker exec
docker exec -it ros2_dev_station bash

# Start MAVROS2
ros2 launch mavros apm.launch fcu_url:=udp://:14550@

# You should see: "FCU: Connected to UDP:..."

# Terminal 3: Verify MAVROS topics
docker exec -it ros2_dev_station bash

ros2 topic list | grep mavros
# You should see: /mavros/state, /mavros/battery, /mavros/imu/data, etc.

ros2 topic echo /mavros/state

# Success! You're receiving ArduPilot messages via MAVROS2
```

**What containers are running:**
```bash
docker ps
# You should see: ros2_dev_station (1 container only)
```

---

### Test Setup 2: Dual Container (Production Simulation)

**What you need:** This simulates onboard + ground station on same machine

**Steps:**

```bash
# 1. Navigate to directory
cd ros2-gazebo-desktop/

# 2. Copy environment template
cp .env.px4 .env

# 3. Build both containers
docker compose -f docker-compose-px4.yml build

# 4. Start both containers
docker compose -f docker-compose-px4.yml up -d

# 5. Verify both running
docker ps
# You should see:
#   - ros2_ground_station
#   - ros2_companion
```

**Testing PX4 (SITL) with Dual Containers:**

```bash
# Terminal 1: Start PX4 SITL (on host)
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: Start micro-ROS agent in COMPANION container
docker exec -it ros2_companion bash
source /home/rosuser/ros2_ws/src/fastdds_ws/install/setup.bash
source /home/rosuser/ros2_ws/install/setup.bash
micro-ros-agent udp4 -p 8888

# You should see: "Micro XRCE-DDS Agent running..."

# Terminal 3: Verify topics in GROUND STATION container
docker exec -it ros2_ground_station bash
ros2 topic list | grep fmu
# You should see PX4 topics here!

ros2 topic echo /fmu/out/vehicle_status

# Success! Topics from companion are visible in ground station
# This proves ROS 2 DDS discovery works between containers
```

**Access Ground Station GUI:**
```
http://localhost:6080/vnc.html
Password: rospassword

# Inside VNC, open terminal:
ros2 topic list
rviz2  # Visualize PX4 topics
```

**What containers are running:**
```bash
docker ps
# You should see:
#   - ros2_ground_station (ports 5901, 6080)
#   - ros2_companion (no exposed ports, host network)
```

**How it works:**
```
PX4 SITL (host) → UDP:8888 → micro-ROS agent (companion container)
                              ↓ ROS 2 DDS topics
                              ↓ (auto-discovered via host network)
                              ↓
                         Ground Station Container
                         (ros2 topic echo works!)
```

---

### Test Setup 3: Hardware Flight Controller

**What you need:** Real PX4/ArduPilot board + USB cable

**For PX4 Hardware:**

```bash
# 1. Connect PX4 via USB to your computer
# 2. Verify serial port
ls -l /dev/ttyUSB* /dev/ttyACM*
# Example output: /dev/ttyUSB0 or /dev/ttyACM0

# 3. Start single container (easier for hardware testing)
docker compose -f docker-compose-simple.yml up -d

# 4. Set PX4 parameters (via QGroundControl or console)
# XRCE_DDS_0_CFG = 1  (Serial mode)
# SER_TEL2_BAUD = 921600

# 5. Start micro-ROS agent with serial
docker exec -it ros2_dev_station bash
micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600

# 6. Verify topics
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_status
```

**For ArduPilot Hardware:**

```bash
# 1. Connect ArduPilot via USB
# 2. Verify serial port
ls -l /dev/ttyUSB* /dev/ttyACM*

# 3. Start container
docker compose -f docker-compose-simple.yml up -d

# 4. Start MAVROS2 with serial
docker exec -it ros2_dev_station bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600

# 5. Verify topics
ros2 topic list | grep mavros
ros2 topic echo /mavros/state
```

---

### Decision Matrix: Which Setup to Use?

| Scenario | Container Setup | Where Deployed | Command Files |
|----------|----------------|----------------|---------------|
| **Development/Testing** | Single | One machine | `docker-compose-simple.yml` |
| **PX4 SITL** | Single | One machine | `docker-compose-simple.yml` |
| **ArduPilot SITL** | Single | One machine | `docker-compose-simple.yml` |
| **Hardware FC via USB** | Single | One machine | `docker-compose-simple.yml` |
| **Test Dual Architecture** | Dual (same machine) | One machine | `docker-compose-px4.yml` |
| **Real Onboard Computer** | Companion only | Separate: Drone + Laptop | Drone: `docker-compose-companion.yml`<br/>Laptop: `docker-compose-simple.yml` |
| **Production Deployment** | Companion only | Separate: Drone + Laptop | Drone: `docker-compose-companion.yml`<br/>Laptop: `docker-compose-simple.yml` |

**Rule of Thumb:**
- 🟢 **Single container** (`simple.yml`) = Development, everything on laptop
- 🟡 **Dual container** (`px4.yml`) = Testing distributed architecture on one machine
- 🔵 **Companion only** (`companion.yml`) = Real deployment, companion on drone + simple on laptop

---

## 🚀 Quick Start Reference

### Single Container Setup

```bash
cd ros2-gazebo-desktop/
docker compose -f docker-compose-simple.yml build
docker compose -f docker-compose-simple.yml up -d

# Access VNC: http://localhost:6080/vnc.html (password: rospassword)

# Run micro-ROS agent inside:
docker exec -it ros2_dev_station bash
micro-ros-agent udp4 -p 8888
```

### Dual Container Setup (Same Machine)

```bash
cd ros2-gazebo-desktop/
cp .env.px4 .env
docker compose -f docker-compose-px4.yml build
docker compose -f docker-compose-px4.yml up -d

# Access VNC: http://localhost:6080/vnc.html

# Run micro-ROS agent in companion:
docker exec -it ros2_companion bash
micro-ros-agent udp4 -p 8888

# View topics in ground station (auto-discovered!):
docker exec -it ros2_ground_station bash
ros2 topic list
```

### Companion Only Setup (Separate Machines)

**On Onboard Computer (Raspberry Pi, Jetson):**
```bash
cd ros2-gazebo-desktop/
cp .env.companion .env
nano .env  # Edit ROS_DOMAIN_ID to match ground station

docker compose -f docker-compose-companion.yml build
docker compose -f docker-compose-companion.yml up -d

# Run micro-ROS agent (PX4):
docker exec -it ros2_companion bash
micro-ros-agent udp4 -p 8888

# Or run MAVROS2 (ArduPilot):
docker exec -it ros2_companion bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600
```

**On Ground Station (Your Laptop):**
```bash
cd ros2-gazebo-desktop/
docker compose -f docker-compose-simple.yml build
docker compose -f docker-compose-simple.yml up -d

# Access VNC: http://localhost:6080/vnc.html

# View topics from companion (auto-discovered over network!):
docker exec -it ros2_dev_station bash
ros2 topic list | grep fmu  # See PX4 topics from companion
ros2 topic echo /fmu/out/vehicle_status
```

**Network Requirements:**
- Both machines on same WiFi/Network
- Same ROS_DOMAIN_ID in both .env files
- UDP multicast enabled (ports 7400-7500)

## 🔧 Configuration

### PX4 Setup

#### On PX4 Side (Flight Controller)

Set these parameters in PX4:

```bash
# Enable micro-ROS/DDS
XRCE_DDS_0_CFG = 2  # UDP mode

# Set UDP port (must match XRCE_DDS_PORT in .env)
XRCE_DDS_PRT = 8888

# Optionally set IP (if not using broadcast)
# XRCE_DDS_AG_IP = <companion-ip>
```

Reboot PX4 for changes to take effect.

#### On Companion Container

```bash
# Access companion container
docker exec -it ros2_companion bash

# Start micro-ROS agent
source /home/rosuser/ros2_ws/src/fastdds_ws/install/setup.bash
source /home/rosuser/ros2_ws/install/setup.bash
micro-ros-agent udp4 -p 8888

# You should see: "Micro XRCE-DDS Agent running..."
```

#### Verify PX4 Connection

In ground station or companion container:

```bash
# List available topics (should see /fmu/out/* topics)
ros2 topic list

# View vehicle status
ros2 topic echo /fmu/out/vehicle_status

# View position
ros2 topic echo /fmu/out/vehicle_local_position

# Send offboard control (example)
ros2 topic pub /fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode '{...}'
```

### ArduPilot Setup

#### On ArduPilot Side

Configure MAVLink output:
- **Serial:** Set SERIALx_PROTOCOL = 2 (MAVLink2)
- **UDP:** ArduPilot broadcasts to :14550 by default
- **TCP:** Configure via GCS parameters

#### On Companion Container

```bash
# Access companion container
docker exec -it ros2_companion bash

# For UDP (default)
ros2 launch mavros apm.launch fcu_url:=udp://:14550@

# For serial
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600

# For TCP
ros2 launch mavros apm.launch fcu_url:=tcp://192.168.1.100:5760
```

#### Verify ArduPilot Connection

```bash
# List MAVROS topics
ros2 topic list | grep mavros

# View battery status
ros2 topic echo /mavros/battery

# View GPS
ros2 topic echo /mavros/global_position/global

# Arm/disarm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "value: true"
```

## 📡 ROS 2 Communication

### Topic Flow

**PX4 → Companion → Ground Station:**
```
[PX4] --DDS--> [micro-ROS Agent] --ROS2--> [Ground Station]
                    ↓
              /fmu/out/vehicle_status
              /fmu/out/vehicle_local_position
              /fmu/out/sensor_combined
                    ...
```

**ArduPilot → Companion → Ground Station:**
```
[ArduPilot] --MAVLink--> [MAVROS2] --ROS2--> [Ground Station]
                             ↓
                       /mavros/state
                       /mavros/global_position/global
                       /mavros/imu/data
                             ...
```

### Viewing Topics

In either container:

```bash
# List all topics
ros2 topic list

# List PX4 topics
ros2 topic list | grep fmu

# List MAVROS topics
ros2 topic list | grep mavros

# Check topic info
ros2 topic info /fmu/out/vehicle_status

# Echo topic data
ros2 topic echo /fmu/out/vehicle_status
```

## 🎮 Usage Examples

### Example 1: PX4 SITL + Gazebo

**Terminal 1 - Ground Station (via VNC):**
```bash
# Launch Gazebo with PX4 world
gz sim worlds/empty.sdf
```

**Terminal 2 - PX4 SITL (outside Docker):**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

**Terminal 3 - Companion:**
```bash
docker exec -it ros2_companion bash
micro-ros-agent udp4 -p 8888
```

**Terminal 4 - Ground Station:**
```bash
# Monitor in VNC terminal
ros2 topic list
ros2 topic echo /fmu/out/vehicle_status
```

### Example 2: Hardware PX4 via Serial

**Connect PX4 to serial port** (e.g., /dev/ttyUSB0)

**Companion Container:**
```bash
docker exec -it ros2_companion bash

# Verify serial device
ls -l /dev/ttyUSB*

# Start micro-ROS with serial
# Note: Update PX4 parameter XRCE_DDS_0_CFG = 1 (Serial)
micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600
```

### Example 3: ArduPilot SITL

**Terminal 1 - ArduPilot SITL (outside Docker):**
```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

**Terminal 2 - Companion:**
```bash
docker exec -it ros2_companion bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```

**Terminal 3 - Ground Station (via VNC):**
```bash
# Launch RViz2
rviz2

# Add displays for /mavros topics
# - /mavros/local_position/pose
# - /mavros/global_position/global
```

### Example 4: Custom ROS 2 Controller

Create a simple offboard controller in ground station:

```bash
# In ground station VNC terminal
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_px4_controller

# Edit package files...
# Build
cd ~/ros2_ws
colcon build --packages-select my_px4_controller

# Run
source install/setup.bash
ros2 run my_px4_controller offboard_controller
```

## 🛠️ Development Workflow

### Adding Custom Packages

**Ground Station:**
```bash
docker exec -it ros2_ground_station bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_package
# Edit files...
cd ~/ros2_ws
colcon build --packages-select my_package
```

**Companion:**
```bash
docker exec -it ros2_companion bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_fc_interface
# Edit files...
cd ~/ros2_ws
colcon build --packages-select my_fc_interface
```

### Workspace Persistence

Workspaces are mounted from host:
- Desktop: `./ros2_ws` → container `/home/rosuser/ros2_ws`
- Companion: `./companion_ws` → container `/home/rosuser/ros2_ws`

Changes persist across container restarts.

### Shared Packages

To share packages between containers:

```bash
# In desktop workspace
cd ros2_ws/src
git clone https://github.com/your-org/shared-package.git

# Copy to companion workspace
cp -r shared-package ../../companion_ws/src/

# Build in both containers
docker exec ros2_ground_station bash -c "cd ~/ros2_ws && colcon build"
docker exec ros2_companion bash -c "cd ~/ros2_ws && colcon build"
```

## 📊 Monitoring and Debugging

### Check Container Status

```bash
# List running containers
docker compose -f docker-compose-px4.yml ps

# View logs
docker compose -f docker-compose-px4.yml logs ground-station
docker compose -f docker-compose-px4.yml logs companion

# Follow logs
docker compose -f docker-compose-px4.yml logs -f companion
```

### ROS 2 Network Diagnostics

```bash
# In any container
ros2 daemon stop
ros2 daemon start

# Check node list
ros2 node list

# Check DDS discovery
ros2 multicast receive  # Terminal 1
ros2 multicast send     # Terminal 2

# Check topic bandwidth
ros2 topic bw /fmu/out/vehicle_status

# Check topic frequency
ros2 topic hz /fmu/out/vehicle_local_position
```

### Serial Port Issues

```bash
# List serial devices
ls -l /dev/tty*

# Check permissions
groups  # Should include 'dialout'

# Test serial connection
picocom /dev/ttyUSB0 -b 921600
# Ctrl+A Ctrl+X to exit
```

## 🔒 Security Considerations

### Network Exposure

Both containers use **host network mode**:
- ✅ Enables ROS 2 DDS discovery
- ⚠️ Exposes all ports directly
- 🔒 Use firewall for production

```bash
# Recommended firewall rules
sudo ufw default deny incoming
sudo ufw allow from 192.168.1.0/24  # Your trusted network
sudo ufw enable
```

### Serial Device Access

Companion container has `privileged: true`:
- Required for serial port access
- ⚠️ Has elevated permissions
- 🔒 Only run trusted code

## 🐛 Troubleshooting

### PX4 Not Connecting

**Check 1: PX4 Parameters**
```bash
# In PX4 console
param show XRCE_DDS_0_CFG  # Should be 1 (serial) or 2 (UDP)
param show XRCE_DDS_PRT     # Should match companion port
```

**Check 2: Network Connectivity**
```bash
# In companion
netstat -ulnp | grep 8888  # Should show micro-ros-agent listening
```

**Check 3: Firewall**
```bash
sudo ufw status
sudo ufw allow 8888/udp
```

### MAVROS Not Connecting

**Check 1: FCU URL**
```bash
# Verify FCU_URL in .env matches your setup
echo $FCU_URL
```

**Check 2: MAVLink Stream**
```bash
# Test raw MAVLink
mavproxy.py --master=udp:0.0.0.0:14550
```

**Check 3: Serial Permissions**
```bash
ls -l /dev/ttyUSB0
groups  # Should include dialout
```

### Topics Not Visible Across Containers

**Check 1: ROS_DOMAIN_ID**
```bash
# Must match in all containers
echo $ROS_DOMAIN_ID
```

**Check 2: RMW Implementation**
```bash
# Must match
echo $RMW_IMPLEMENTATION
```

**Check 3: DDS Discovery**
```bash
ros2 multicast receive
# From another terminal
ros2 multicast send
```

### VNC Not Working

**Check 1: Port Availability**
```bash
netstat -tulnp | grep 5901
netstat -tulnp | grep 6080
```

**Check 2: Container Running**
```bash
docker ps | grep ground_station
```

**Check 3: VNC Logs**
```bash
docker exec ros2_ground_station cat ~/.vnc/*.log
```

## 📚 Additional Resources

### Official Documentation
- [PX4 Documentation](https://docs.px4.io/)
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [ArduPilot Documentation](https://ardupilot.org/)
- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/)

### Message Definitions
- [px4_msgs Repository](https://github.com/PX4/px4_msgs)
- [MAVROS Messages](https://github.com/mavlink/mavros/tree/ros2/mavros_msgs)

### Example Projects
- [PX4 ROS 2 Examples](https://github.com/PX4/px4_ros_com)
- [MAVROS Examples](https://github.com/mavlink/mavros/tree/ros2/mavros/scripts)

## 🤝 Architecture Decisions

### Why Two Containers?

1. **Separation of Concerns:**
   - Ground station: Heavy GUI, simulation, visualization
   - Companion: Lightweight, FC communication only

2. **Resource Efficiency:**
   - Companion can run on low-power hardware
   - Ground station needs more resources

3. **Deployment Flexibility:**
   - Can deploy companion to actual onboard computer
   - Ground station stays on development machine

### Why MAVROS2 in Both?

- **Ground Station:** Monitoring, visualization, mission planning
- **Companion:** Actual FC communication, message forwarding

### Why Host Network?

- **Automatic DDS Discovery:** No manual configuration needed
- **Simplicity:** No port mapping complexity
- **Performance:** No NAT overhead

Alternative: Use ROS 2 Discovery Server for isolated networks (see ROS2_NETWORKING.md)

## 📝 License

This setup is provided as-is for development and testing purposes.

## 🙋 Support

For issues and questions:
- Check troubleshooting section above
- Review ROS2_NETWORKING.md for network issues
- Check PX4/ArduPilot documentation
- Review container logs

---

**Happy Flying! 🚁**
