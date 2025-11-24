# PX4 SITL Container with Gazebo Harmonic

Docker container for running PX4 SITL (Software-In-The-Loop) simulation with Gazebo Harmonic. Designed to work alongside the **ros2-gazebo-desktop** container for complete ROS 2 integration.

## Architecture

```
┌─────────────────────────────────┐     ┌─────────────────────────────────┐
│   PX4 SITL Container            │     │  ROS2 Gazebo Desktop Container  │
│   (this container)              │     │  (ros2-gazebo-desktop)          │
│                                 │     │                                 │
│  ┌───────────────────────────┐  │     │  ┌───────────────────────────┐  │
│  │  PX4 Autopilot (SITL)     │  │     │  │  micro-ROS Agent          │  │
│  │  └── uXRCE-DDS client ────────────────► UDP 8888                  │  │
│  │                           │  │     │  │                           │  │
│  │  MAVLink ─────────────────────────────► MAVROS2 (optional)        │  │
│  │  └── UDP 14550/14540      │  │     │  │                           │  │
│  └───────────────────────────┘  │     │  │  ROS 2 Topics:            │  │
│                                 │     │  │  /fmu/in/*  (commands)    │  │
│  ┌───────────────────────────┐  │     │  │  /fmu/out/* (telemetry)   │  │
│  │  Gazebo Harmonic          │  │     │  └───────────────────────────┘  │
│  │  (headless simulation)    │  │     │                                 │
│  └───────────────────────────┘  │     │  ┌───────────────────────────┐  │
│                                 │     │  │  Gazebo Harmonic (GUI)    │  │
│  network_mode: host             │     │  │  (visualization only)     │  │
└─────────────────────────────────┘     │  └───────────────────────────┘  │
              │                         │  network_mode: host             │
              │ UDP 14550               └─────────────────────────────────┘
              ▼
┌─────────────────────────────────┐
│  QGroundControl (Windows)       │
│  Auto-discovers on UDP 14550    │
└─────────────────────────────────┘
```

## Quick Start

### 1. Clone PX4 Source

```bash
cd px4-sitl-container

# Clone PX4-Autopilot (v1.16.0 recommended)
git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.16.0

# Or for latest development:
# git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b main
```

### 2. Configure Environment

```bash
# Copy environment template
cp .env.example .env

# Edit settings if needed
nano .env
```

### 3. Build Container

```bash
docker compose build
```

This takes ~15-20 minutes on first build.

### 4. Start PX4 SITL

```bash
docker compose up -d
```

### 5. View Logs

```bash
docker compose logs -f
```

You should see PX4 starting and waiting for connections.

---

## Connecting ROS 2 (ros2-gazebo-desktop)

In your **ros2-gazebo-desktop** container, start the micro-ROS agent:

```bash
# Connect to ROS 2 container
docker exec -it ros2_gazebo_vnc bash

# Start micro-ROS agent to receive PX4 topics
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

Or using the pre-built command:
```bash
micro-ros-agent udp4 -p 8888
```

### Verify ROS 2 Topics

```bash
# List PX4 topics
ros2 topic list | grep fmu

# Expected output:
# /fmu/in/obstacle_distance
# /fmu/in/offboard_control_mode
# /fmu/in/trajectory_setpoint
# /fmu/in/vehicle_command
# /fmu/out/failsafe_flags
# /fmu/out/sensor_combined
# /fmu/out/timesync_status
# /fmu/out/vehicle_attitude
# /fmu/out/vehicle_control_mode
# /fmu/out/vehicle_global_position
# /fmu/out/vehicle_local_position
# /fmu/out/vehicle_odometry
# /fmu/out/vehicle_status
# ... and more
```

### Echo Telemetry

```bash
# Vehicle position
ros2 topic echo /fmu/out/vehicle_local_position

# Vehicle attitude
ros2 topic echo /fmu/out/vehicle_attitude

# Vehicle status
ros2 topic echo /fmu/out/vehicle_status
```

---

## Connecting QGroundControl (Windows)

1. **Open QGroundControl** on your Windows machine

2. **Auto-discovery**: QGC should automatically discover the vehicle on UDP 14550

3. **Manual Connection** (if auto-discovery fails):
   - Go to: Application Settings → Comm Links
   - Add new link:
     - Type: UDP
     - Port: 14550
     - Server Address: `<Ubuntu-host-IP>`
   - Connect

4. **Verify Connection**:
   - Vehicle should appear in QGC
   - Telemetry data should be streaming
   - Map should show vehicle position

---

## Configuration Options

### Vehicle Models

Set `PX4_GZ_MODEL` in `.env`:

| Model | Description |
|-------|-------------|
| `x500` | Standard quadcopter (default) |
| `x500_depth` | Quadcopter with depth camera |
| `x500_lidar` | Quadcopter with 2D lidar |
| `rc_cessna` | Fixed-wing RC plane |
| `standard_vtol` | Standard VTOL aircraft |
| `typhoon_h480` | Hexacopter with camera gimbal |

### Airframes

Set `PX4_SYS_AUTOSTART` in `.env`:

| ID | Airframe |
|----|----------|
| 4001 | Generic x500 Quadcopter |
| 4011 | DJI F450 |
| 4014 | S500 Quadcopter |
| 2106 | Cessna 172 |
| 1100 | Standard Plane |
| 13000 | Generic Standard VTOL |

### Headless Mode

```bash
# Headless (default, for servers)
HEADLESS=1

# With Gazebo GUI (requires X11 forwarding)
HEADLESS=0
```

---

## Multi-Vehicle Simulation

### Option 1: Multiple Instances in Same Container

```bash
# Start 3 vehicles
docker exec -it px4_sitl /home/px4user/scripts/start-multi-instance.sh 3
```

Ports for each instance:
- Instance 0: UDP 14550, 14540, 8888
- Instance 1: UDP 14551, 14541, 8889
- Instance 2: UDP 14552, 14542, 8890

### Option 2: Multiple Containers

Create `docker-compose-multi.yml`:

```yaml
services:
  px4-sitl-0:
    extends:
      file: docker-compose.yml
      service: px4-sitl
    container_name: px4_sitl_0
    environment:
      - PX4_SIM_INSTANCE=0

  px4-sitl-1:
    extends:
      file: docker-compose.yml
      service: px4-sitl
    container_name: px4_sitl_1
    environment:
      - PX4_SIM_INSTANCE=1
```

---

## Communication Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 14550 | UDP | MAVLink - QGroundControl |
| 14540 | UDP | MAVLink - MAVROS/Offboard |
| 8888 | UDP | uXRCE-DDS - micro-ROS agent |
| 11345 | TCP | Gazebo transport |

---

## Troubleshooting

### PX4 Not Starting

```bash
# Check logs
docker compose logs px4-sitl

# Enter container for debugging
docker exec -it px4_sitl bash

# Try manual start
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

### Build Errors

```bash
# Clean build
docker exec -it px4_sitl bash
cd ~/PX4-Autopilot
make clean
make px4_sitl
```

### QGC Not Connecting

1. Check firewall on Ubuntu host:
   ```bash
   sudo ufw allow 14550/udp
   sudo ufw allow 14540/udp
   ```

2. Verify PX4 is broadcasting:
   ```bash
   # On Ubuntu host
   netstat -ulnp | grep 14550
   ```

3. Check Windows firewall allows QGC

### ROS 2 Topics Not Appearing

1. Ensure micro-ROS agent is running:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
   ```

2. Check PX4 uXRCE-DDS is enabled (it is by default in SITL)

3. Verify same network (both containers use `network_mode: host`)

### Gazebo Crashes

```bash
# Increase shared memory
# In docker-compose.yml:
shm_size: '4gb'

# Or run with software rendering
export LIBGL_ALWAYS_SOFTWARE=1
```

---

## Integration with Existing Setup

Your current setup:
```
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ ArduPilot SITL   │  │ PX4 SITL         │  │ ROS2 Desktop     │
│ (existing)       │  │ (NEW)            │  │ (existing)       │
│                  │  │                  │  │                  │
│ network: host    │  │ network: host    │  │ network: host    │
│ MAVROS ◄─────────────────────────────────► MAVROS           │
│                  │  │ uXRCE-DDS ◄────────► micro-ROS agent  │
└──────────────────┘  └──────────────────┘  └──────────────────┘
```

All three containers communicate seamlessly via host networking.

---

## File Structure

```
px4-sitl-container/
├── Dockerfile              # Container definition
├── docker-compose.yml      # Main compose file
├── .env.example            # Environment template
├── .env                    # Your configuration (gitignored)
├── README.md               # This file
├── scripts/
│   ├── start-px4-sitl.sh   # Main startup script
│   └── start-multi-instance.sh  # Multi-vehicle script
└── PX4-Autopilot/          # PX4 source (git clone)
```

---

## Summary

1. **Clone PX4**: `git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.16.0`
2. **Configure**: `cp .env.example .env`
3. **Build**: `docker compose build`
4. **Start**: `docker compose up -d`
5. **Connect ROS 2**: `ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888`
6. **Connect QGC**: Auto-discovers on UDP 14550
