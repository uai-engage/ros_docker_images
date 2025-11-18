# Quick Reference Guide

Quick commands for working with the organized directory structure.

## Directory Structure at a Glance

```
ros2-gazebo-desktop/
├── Dockerfile & docker-compose.yml    # Main desktop (ground station)
├── docs/                              # Desktop documentation
├── scripts/                           # Desktop scripts
└── companion/                         # Companion computer (drone)
    ├── Dockerfile & docker-compose.yml
    ├── docs/                          # Companion docs
    ├── scripts/                       # Companion scripts
    └── systemd/                       # Auto-start services
```

## Ground Station / Development Setup

### Build and Run Desktop Environment

```bash
cd ros2-gazebo-desktop

# Build and start
docker compose up -d

# Access VNC
# Open browser: http://localhost:6080/vnc.html

# Check logs
docker logs -f ros2_gazebo_vnc
```

### Install micro-ROS Agent (Inside Container)

```bash
# Enter container
docker exec -it ros2_gazebo_vnc bash

# Run installation script
cd ~
./ros2_ws/../scripts/install_micro_ros.sh

# Or manually
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select micro_ros_agent
```

### Stop micro-ROS Agent

```bash
# From inside container or host (if using host network)
./scripts/stop_micro_ros.sh

# Or manually
pkill -f micro_ros_agent
ros2 daemon stop && ros2 daemon start
```

### Documentation

- **ArduPilot Integration**: `docs/ARDUPILOT_MICRO_ROS_INTEGRATION.md`
- **Quick Start**: `docs/QUICK_START_ARDUPILOT.md`
- **Troubleshooting**: `docs/BUILD_FIX_NOTES.md`
- **Stop Agent**: `docs/STOP_MICRO_ROS.md`

---

## Companion Computer Setup

### For SITL Testing (UDP Mode - Default)

```bash
cd ros2-gazebo-desktop/companion

# Quick start with defaults
docker compose up -d

# Check logs
docker logs -f micro_ros_agent_companion

# Stop
docker compose down
```

### For Production Deployment

```bash
cd ros2-gazebo-desktop/companion

# Copy to companion computer
scp -r . ubuntu@companion-computer:~/companion/

# SSH to companion
ssh ubuntu@companion-computer
cd ~/companion

# Configure
cp .env.example .env
nano .env

# Edit settings:
# CONNECTION_MODE=serial
# SERIAL_DEVICE=/dev/ttyUSB0
# BAUD_RATE=921600
# GROUND_STATION_IP=192.168.1.100

# Uncomment devices section in docker-compose.yml for serial mode

# Build and run
docker compose up -d
```

### Environment Variables

Create `.env` file in `companion/` directory:

```bash
# For SITL (UDP)
CONNECTION_MODE=udp4
AGENT_PORT=2019
GROUND_STATION_IP=127.0.0.1
ROS_DOMAIN_ID=0

# For Real Hardware (Serial)
CONNECTION_MODE=serial
SERIAL_DEVICE=/dev/ttyUSB0
BAUD_RATE=921600
GROUND_STATION_IP=192.168.1.100
ROS_DOMAIN_ID=0
```

### Documentation

- **Quick Start**: `companion/README.md`
- **Detailed Setup**: `companion/docs/COMPANION_COMPUTER_SETUP.md`
- **SITL Testing**: `companion/docs/SITL_TESTING_GUIDE.md`

### Native Installation (No Docker)

```bash
cd companion/

# Use the helper script
./scripts/start_agent.sh

# Or install systemd service
sudo cp systemd/micro-ros-agent.service /etc/systemd/system/
sudo nano /etc/systemd/system/micro-ros-agent.service  # Edit paths
sudo systemctl daemon-reload
sudo systemctl enable micro-ros-agent
sudo systemctl start micro-ros-agent
```

---

## Common Tasks

### Switch Companion Mode

**UDP to Serial:**
```bash
cd companion/

# 1. Uncomment devices section in docker-compose.yml
nano docker-compose.yml  # Uncomment lines 18-22 and 25

# 2. Update environment
CONNECTION_MODE=serial SERIAL_DEVICE=/dev/ttyUSB0 BAUD_RATE=921600 docker compose up -d
```

**Serial to TCP:**
```bash
cd companion/

# Comment out devices section, then:
CONNECTION_MODE=tcp4 AGENT_PORT=2019 docker compose up -d
```

### Check ROS Topics

**On ground station:**
```bash
# Set domain (if not already set)
export ROS_DOMAIN_ID=0

# List topics
ros2 topic list

# Should see /fmu/* topics when connected
# /fmu/battery/status
# /fmu/gps/position
# /fmu/imu/data
# etc.

# Echo a topic
ros2 topic echo /fmu/battery/status

# Check rate
ros2 topic hz /fmu/imu/data
```

### Troubleshooting

**No topics appearing:**
```bash
# 1. Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Should be 0 on both systems

# 2. Clear DDS cache
ros2 daemon stop && ros2 daemon start

# 3. Check agent logs
docker logs -f micro_ros_agent_companion

# 4. Check network connectivity
ping <companion-ip>
```

**Port already in use:**
```bash
# Find what's using port
netstat -tulpn | grep 2019

# Kill process
sudo kill -9 <PID>

# Or use different port
AGENT_PORT=8888 docker compose up -d
```

**Serial device not found:**
```bash
# List devices
ls -l /dev/tty*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group
sudo usermod -aG dialout $USER
```

---

## File Locations Quick Reference

| What | Desktop | Companion |
|------|---------|-----------|
| **Dockerfile** | `./Dockerfile` | `companion/Dockerfile` |
| **docker-compose** | `./docker-compose.yml` | `companion/docker-compose.yml` |
| **Documentation** | `docs/` | `companion/docs/` |
| **Scripts** | `scripts/` | `companion/scripts/` |
| **Quick Start** | `README.md` | `companion/README.md` |
| **Config Template** | N/A | `companion/.env.example` |
| **Systemd Service** | N/A | `companion/systemd/` |

---

## Next Steps

1. **For Ground Station Setup**:
   - Read `README.md`
   - Follow `docs/QUICK_START_ARDUPILOT.md`

2. **For Companion Computer**:
   - Read `companion/README.md`
   - For SITL: Follow `companion/docs/SITL_TESTING_GUIDE.md`
   - For Hardware: Follow `companion/docs/COMPANION_COMPUTER_SETUP.md`

3. **For Complete Understanding**:
   - Review `DIRECTORY_STRUCTURE.md` for file organization
   - Check `docs/ARDUPILOT_MICRO_ROS_INTEGRATION.md` for integration details
