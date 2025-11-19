# ROS 2 Jazzy PX4/ArduPilot Development Environment
# Ground Station + Companion Computer Setup

Complete Docker-based development environment for PX4 and ArduPilot with ROS 2 Jazzy, featuring a ground station with GUI and a companion computer for flight controller communication.

## 🏗️ Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Development Machine                  │
│  ┌──────────────────────────┐  ┌──────────────────────────┐ │
│  │   Ground Station         │  │  Companion Computer       │ │
│  │   Container              │  │  Container                │ │
│  │                          │  │                           │ │
│  │  • Gazebo Harmonic       │  │  • micro-ROS Agent (PX4)  │ │
│  │  • RViz2                 │  │  • MAVROS2 (ArduPilot)    │ │
│  │  • VNC Desktop (XFCE)    │  │  • px4_msgs               │ │
│  │  • px4_msgs              │  │  • Serial/UDP FC comm     │ │
│  │  • MAVROS2               │  │                           │ │
│  │                          │  │                           │ │
│  └──────────┬───────────────┘  └─────────┬─────────────────┘ │
│             │                            │                   │
│             └────────── ROS 2 DDS ───────┘                   │
│                    (Host Network)                            │
│                                                               │
│                           ▼                                  │
│                  ┌─────────────────┐                         │
│                  │  Flight Controller│                       │
│                  │  • PX4 Autopilot  │                       │
│                  │  • ArduPilot      │                       │
│                  └─────────────────┘                         │
└─────────────────────────────────────────────────────────────┘
```

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

## 🚀 Quick Start

### 1. Clone and Setup

```bash
cd ros2-gazebo-desktop/

# Copy environment template
cp .env.px4 .env

# Edit configuration (optional)
nano .env
```

### 2. Build Images

```bash
# Build both containers
docker compose -f docker-compose-px4.yml build

# Or build individually
docker compose -f docker-compose-px4.yml build ground-station
docker compose -f docker-compose-px4.yml build companion
```

### 3. Start Containers

```bash
# Start both containers
docker compose -f docker-compose-px4.yml up -d

# Or start individually
docker compose -f docker-compose-px4.yml up -d ground-station
docker compose -f docker-compose-px4.yml up -d companion
```

### 4. Access Ground Station

**Via Web Browser (noVNC):**
```
http://localhost:6080/vnc.html
Password: rospassword (or your VNC_PASSWORD)
```

**Via VNC Client:**
```
Host: localhost:5901
Password: rospassword (or your VNC_PASSWORD)
```

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
