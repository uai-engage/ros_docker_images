# PX4 SITL + ROS 2 + Gazebo Integration Guide

Complete guide for running PX4 SITL with ROS 2 and Gazebo Harmonic in separate containers.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│  ROS2 Container (ros2_dev_station)                          │
│                                                             │
│  ┌──────────────────────────┐   ┌─────────────────────┐    │
│  │ Gazebo Harmonic          │   │ micro-ROS agent     │    │
│  │ - Visualization          │   │ UDP 8888            │    │
│  │ - GUI Rendering          │   │                     │    │
│  │ - PX4 Models/Worlds      │   │ ROS 2 Topics:       │    │
│  │   ~/px4_gazebo/          │   │ /fmu/in/*           │    │
│  └──────────────────────────┘   │ /fmu/out/*          │    │
│                                  └─────────────────────┘    │
│  Access: http://localhost:6080/vnc.html                     │
└─────────────────────────────────────────────────────────────┘
                    ▲                          ▲
                    │ Gazebo Transport         │ uXRCE-DDS
                    │ (port 11345)             │ (port 8888)
                    │                          │
                    │     Host Network Mode    │
                    ▼                          ▼
┌─────────────────────────────────────────────────────────────┐
│  PX4 SITL Container (px4_sitl)                              │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ PX4 Autopilot                                        │   │
│  │ - Flight Controller Logic                            │   │
│  │ - Physics Simulation                                 │   │
│  │ - Sensor Simulation                                  │   │
│  │                                                      │   │
│  │ Ports:                                               │   │
│  │ - MAVLink GCS: UDP 14550 (QGroundControl)           │   │
│  │ - MAVLink API: UDP 14540 (MAVROS/Offboard)          │   │
│  │ - uXRCE-DDS:   UDP 8888  (micro-ROS agent)          │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            │
                            │ MAVLink UDP 14550
                            ▼
                ┌───────────────────────────┐
                │ QGroundControl (Windows)  │
                │ Auto-discovers vehicle    │
                └───────────────────────────┘
```

---

## Prerequisites

### 1. Clone PX4-Autopilot (with submodules)

**CRITICAL:** You must clone PX4 with the `--recursive` flag to download all submodules.

```bash
cd /mnt/d/projects/uai/engage/ros_docker_images/ros2-gazebo-desktop/px4-sitl-container

# Clone PX4 v1.16.0 with all submodules
git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.16.0
```

If you already cloned without `--recursive`, initialize submodules:
```bash
cd PX4-Autopilot
git submodule update --init --recursive
```

### 2. Build Docker Images

```bash
# Build PX4 SITL container
cd /mnt/d/projects/uai/engage/ros_docker_images/ros2-gazebo-desktop/px4-sitl-container
docker compose build

# Build ROS2 container (if not already built)
cd /mnt/d/projects/uai/engage/ros_docker_images/ros2-gazebo-desktop
docker compose -f docker-compose-simple.yml build
```

---

## Starting the System

### Step 1: Start ROS2 Container

```bash
cd /mnt/d/projects/uai/engage/ros_docker_images/ros2-gazebo-desktop

docker compose -f docker-compose-simple.yml up -d
```

**Check status:**
```bash
docker compose -f docker-compose-simple.yml ps
docker compose -f docker-compose-simple.yml logs
```

### Step 2: Start PX4 SITL Container

```bash
cd /mnt/d/projects/uai/engage/ros_docker_images/ros2-gazebo-desktop/px4-sitl-container

docker compose up -d
```

**Monitor PX4 startup:**
```bash
docker compose logs -f
```

You should see:
```
============================================
  PX4 SITL Container Starting
============================================

Configuration:
  PX4_GZ_MODEL:      x500
  PX4_GZ_WORLD:      default
  PX4_SYS_AUTOSTART: 4001
  HEADLESS:          1
  EXTERNAL_GAZEBO:   1

Mode: EXTERNAL GAZEBO
Connecting to Gazebo in ROS2 container...
```

**Note:** PX4 will wait for Gazebo to start. Don't worry if you see "waiting for Gazebo" messages.

---

## Running Gazebo with PX4 Models

### Option 1: Via VNC Web Browser (Recommended)

1. **Open browser:** http://localhost:6080/vnc.html
2. **Password:** `rospassword` (default, can be changed in docker-compose)
3. **Open terminal** in the VNC desktop
4. **Launch Gazebo:**

```bash
gz sim -v4 -r ~/px4_gazebo/worlds/default.sdf
```

### Option 2: Via Docker Exec

```bash
# Connect to ROS2 container
docker exec -it ros2_dev_station bash

# Launch Gazebo
gz sim -v4 -r ~/px4_gazebo/worlds/default.sdf
```

### Available PX4 Worlds

Located in `~/px4_gazebo/worlds/`:

| World File | Description |
|------------|-------------|
| `default.sdf` | Empty world (simple, fast) |
| `baylands.sdf` | Outdoor terrain with hills and textures |
| `windy.sdf` | World with wind simulation (tests wind resistance) |
| `typhoon_h480.sdf` | Optimized for hexacopter |

### Gazebo Command Options

```bash
# Basic launch
gz sim worlds/default.sdf

# With verbose output (recommended for debugging)
gz sim -v4 worlds/default.sdf

# Run simulation immediately (auto-start)
gz sim -r worlds/default.sdf

# Combined: verbose + auto-run
gz sim -v4 -r ~/px4_gazebo/worlds/default.sdf

# Headless (no GUI, for servers)
gz sim -s ~/px4_gazebo/worlds/default.sdf
```

**What happens:**
- Gazebo starts with the selected world
- PX4 SITL detects Gazebo via Gazebo Transport
- PX4 automatically spawns the x500 quadcopter
- Vehicle appears in Gazebo at coordinates (0, 0, 0)

---

## Starting micro-ROS Agent

The micro-ROS agent bridges PX4 to ROS 2 topics.

### In a new terminal:

```bash
# Connect to ROS2 container
docker exec -it ros2_dev_station bash

# Start micro-ROS agent
micro-ros-agent udp4 -p 8888
```

**Expected output:**
```
[1234567890.123456] info     | UDPv4AgentLinux.cpp | init | running...
[1234567890.234567] info     | Root.cpp | create_client | create
[1234567890.345678] info     | SessionManager.hpp | establish_session | session established
```

### Alternative: Use ros2 run

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

**Keep this terminal running.** The agent must stay active for PX4 ↔ ROS 2 communication.

---

## Verification & Testing

### 1. Check ROS 2 Topics

```bash
# In ROS2 container (new terminal)
docker exec -it ros2_dev_station bash

# List all PX4 topics
ros2 topic list | grep fmu
```

**Expected output:**
```
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/out/battery_status
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_control_mode
/fmu/out/vehicle_global_position
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
... (and many more)
```

### 2. Monitor Vehicle Telemetry

```bash
# Vehicle position
ros2 topic echo /fmu/out/vehicle_local_position

# Vehicle attitude (roll, pitch, yaw)
ros2 topic echo /fmu/out/vehicle_attitude

# Vehicle status (armed, mode, etc.)
ros2 topic echo /fmu/out/vehicle_status

# Battery status
ros2 topic echo /fmu/out/battery_status
```

### 3. Check Gazebo Connection

```bash
# List Gazebo topics
gz topic -l

# Should see topics like:
# /clock
# /world/default/model/x500_0/...
```

### 4. PX4 Console Commands

```bash
# Enter PX4 SITL container
docker exec -it px4_sitl bash

# Access PX4 shell (if you started it interactively)
# Type commands like:
commander status
commander takeoff
commander land
```

---

## Connecting QGroundControl

### From Windows Host:

1. **Download QGroundControl:** https://qgroundcontrol.com
2. **Open QGC**
3. **Auto-discovery:** QGC should automatically find the vehicle on UDP 14550

### Manual Connection (if auto-discovery fails):

1. Go to: **Application Settings → Comm Links**
2. Click **Add**
3. Configure:
   - **Type:** UDP
   - **Port:** 14550
   - **Server Address:** `<Ubuntu-Host-IP>` (your WSL2 IP or server IP)
4. Click **OK** and **Connect**

### Check Firewall:

If QGC can't connect from Windows:

```bash
# On Ubuntu host
sudo ufw allow 14550/udp
sudo ufw allow 14540/udp
sudo ufw status
```

---

## Common Operations

### Restart Everything

```bash
# Stop all containers
cd /mnt/d/projects/uai/engage/ros_docker_images/ros2-gazebo-desktop/px4-sitl-container
docker compose down

cd /mnt/d/projects/uai/engage/ros_docker_images/ros2-gazebo-desktop
docker compose -f docker-compose-simple.yml down

# Start ROS2 first
docker compose -f docker-compose-simple.yml up -d

# Then start PX4
cd px4-sitl-container
docker compose up -d
```

### Rebuild PX4 (after code changes)

```bash
# PX4 builds automatically at container startup
# To force rebuild, delete the build directory:

cd px4-sitl-container/PX4-Autopilot
rm -rf build/

# Restart container (will rebuild)
docker compose restart
```

### Change PX4 Vehicle Model

Edit `px4-sitl-container/.env`:

```bash
PX4_GZ_MODEL=rc_cessna  # Change from x500 to plane
```

Available models in `~/px4_gazebo/models/`:
- `x500` - Quadcopter (default)
- `x500_depth` - Quadcopter with depth camera
- `rc_cessna` - Fixed-wing RC plane
- `standard_vtol` - VTOL aircraft
- `typhoon_h480` - Hexacopter with gimbal

Restart PX4 container after changing.

### View Container Logs

```bash
# ROS2 container logs
docker compose -f docker-compose-simple.yml logs -f

# PX4 SITL logs
cd px4-sitl-container
docker compose logs -f

# Just the last 50 lines
docker compose logs --tail=50
```

---

## Troubleshooting

### Problem: Gazebo can't find PX4 models

**Symptoms:** Error loading models, vehicle doesn't spawn

**Solution:**
```bash
# Check if PX4 Gazebo files are mounted
docker exec -it ros2_dev_station bash
ls ~/px4_gazebo/models/  # Should list x500, rc_cessna, etc.
ls ~/px4_gazebo/worlds/  # Should list default.sdf, baylands.sdf, etc.

# Check environment variable
echo $GZ_SIM_RESOURCE_PATH  # Should include /home/rosuser/px4_gazebo/models
```

If missing, ensure `PX4-Autopilot` is cloned in `px4-sitl-container/` directory.

### Problem: micro-ROS agent can't connect

**Symptoms:** No `/fmu/*` topics appear

**Solution:**
```bash
# Check if agent is running
docker exec -it ros2_dev_station bash
ps aux | grep micro_ros_agent

# Check if PX4 is broadcasting
docker exec -it px4_sitl bash
netstat -ulnp | grep 8888

# Restart micro-ROS agent
# Kill existing: pkill -f micro_ros_agent
micro-ros-agent udp4 -p 8888
```

### Problem: PX4 build fails (heatshrink error)

**Symptoms:** "Cannot find source file: heatshrink/heatshrink_decoder.c"

**Solution:**
```bash
# PX4 submodules not initialized
cd px4-sitl-container/PX4-Autopilot
git submodule update --init --recursive

# Restart container
docker compose restart
```

### Problem: QGroundControl can't connect

**Symptoms:** No vehicle in QGC

**Solution:**
```bash
# 1. Check PX4 is broadcasting
docker exec -it px4_sitl bash
netstat -ulnp | grep 14550

# 2. Check firewall
sudo ufw allow 14550/udp

# 3. Check Windows firewall allows QGC

# 4. Try manual connection in QGC with host IP
```

### Problem: Gazebo crashes or freezes

**Symptoms:** Gazebo GUI unresponsive, high CPU

**Solution:**
```bash
# Increase shared memory
# Edit docker-compose-simple.yml:
shm_size: '4gb'  # Increase from 2gb

# Use software rendering (slower but more stable)
# In container:
export LIBGL_ALWAYS_SOFTWARE=1
gz sim -v4 -r ~/px4_gazebo/worlds/default.sdf
```

### Problem: Container won't start

**Symptoms:** Exit code 2, build errors

**Solution:**
```bash
# Check logs for specific error
docker compose logs

# Common fixes:
# 1. Rebuild image
docker compose build --no-cache

# 2. Check if PX4 source is properly mounted
ls px4-sitl-container/PX4-Autopilot/  # Should show PX4 files

# 3. Check disk space
df -h
```

---

## Port Reference

| Port | Protocol | Purpose | Container |
|------|----------|---------|-----------|
| 5901 | TCP | VNC Direct | ROS2 |
| 6080 | TCP | noVNC Web | ROS2 |
| 8888 | UDP | uXRCE-DDS (micro-ROS) | Both |
| 14550 | UDP | MAVLink GCS (QGC) | PX4 |
| 14540 | UDP | MAVLink API (MAVROS) | PX4 |
| 11345 | TCP | Gazebo Transport | Both |

All containers use **host network mode**, so ports are directly accessible on the host.

---

## Additional Resources

### PX4 Documentation
- Official: https://docs.px4.io/
- Gazebo Simulation: https://docs.px4.io/main/en/sim_gazebo_gz/

### ROS 2 Documentation
- micro-ROS: https://micro.ros.org/
- MAVROS2: https://github.com/mavlink/mavros

### Gazebo Documentation
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic

### Project Files
- PX4 SITL Container: `px4-sitl-container/README.md`
- Development Guide: `README-DEVELOPMENT.md`

---

## Quick Command Reference

```bash
# Start everything
docker compose -f docker-compose-simple.yml up -d
cd px4-sitl-container && docker compose up -d

# Launch Gazebo (in ROS2 container)
docker exec -it ros2_dev_station bash
gz sim -v4 -r ~/px4_gazebo/worlds/default.sdf

# Start micro-ROS agent (in ROS2 container, new terminal)
docker exec -it ros2_dev_station bash
micro-ros-agent udp4 -p 8888

# Check ROS 2 topics
docker exec -it ros2_dev_station bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_local_position

# Stop everything
docker compose down
cd .. && docker compose -f docker-compose-simple.yml down
```

---

## Summary

✅ **Two separate containers** for modularity
✅ **PX4 Gazebo models** shared via volume mount
✅ **ROS 2 topics** via micro-ROS agent (uXRCE-DDS)
✅ **QGroundControl** via MAVLink (UDP 14550)
✅ **Host networking** for seamless communication
✅ **VNC access** for GUI visualization

**Workflow:**
1. Start ROS2 container → 2. Start PX4 container → 3. Launch Gazebo → 4. Start micro-ROS agent → 5. Connect QGC

Enjoy your PX4 SITL + ROS 2 + Gazebo setup! 🚁
