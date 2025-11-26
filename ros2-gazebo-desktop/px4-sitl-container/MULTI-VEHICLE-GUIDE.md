# Multi-Vehicle PX4 SITL Simulation Guide

This guide explains how to run multiple PX4 vehicles in simulation using Docker containers with Gazebo Harmonic.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Quick Start](#quick-start)
- [Docker Compose Generator](#docker-compose-generator)
- [Vehicle Types](#vehicle-types)
- [Port Reference](#port-reference)
- [Single PX4 Source (FAQ)](#single-px4-source-faq)
- [Running the Simulation](#running-the-simulation)
- [QGroundControl Setup](#qgroundcontrol-setup)
- [Troubleshooting](#troubleshooting)

---

## Overview

The multi-vehicle simulation system allows you to run multiple PX4 SITL instances simultaneously, each controlling a different vehicle in the same Gazebo world. This is useful for:

- Swarm simulations
- Multi-robot coordination
- Testing fleet management systems
- Mixed vehicle type scenarios (drones + rovers)

### Key Features

- ✅ **Single PX4 source code** - No need for multiple git clones
- ✅ **Automatic port allocation** - Each vehicle gets unique ports
- ✅ **Mixed vehicle types** - Combine copters, planes, rovers, boats
- ✅ **Docker Compose generator** - Create custom configurations easily
- ✅ **Shared Gazebo world** - All vehicles in the same simulation

---

## Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                         HOST MACHINE                                │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │                    ROS2 Container                             │ │
│  │                                                               │ │
│  │  ┌─────────────────────────────────────────────────────────┐ │ │
│  │  │                 Gazebo Harmonic                          │ │ │
│  │  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐    │ │ │
│  │  │  │ x500_0  │  │ x500_1  │  │ rover_2 │  │ vtol_3  │    │ │ │
│  │  │  │ Copter  │  │ Copter  │  │  Rover  │  │  VTOL   │    │ │ │
│  │  │  └─────────┘  └─────────┘  └─────────┘  └─────────┘    │ │ │
│  │  └─────────────────────────────────────────────────────────┘ │ │
│  │                                                               │ │
│  │  ┌─────────────────────────────────────────────────────────┐ │ │
│  │  │              micro-ROS Agents (one per vehicle)          │ │ │
│  │  │  Port 8888    Port 8889    Port 8890    Port 8891       │ │ │
│  │  └─────────────────────────────────────────────────────────┘ │ │
│  └──────────────────────────────────────────────────────────────┘ │
│           │             │             │             │              │
│           ▼             ▼             ▼             ▼              │
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐     │
│  │ PX4 SITL   │ │ PX4 SITL   │ │ PX4 SITL   │ │ PX4 SITL   │     │
│  │ Instance 0 │ │ Instance 1 │ │ Instance 2 │ │ Instance 3 │     │
│  │            │ │            │ │            │ │            │     │
│  │ MAV:14550  │ │ MAV:14551  │ │ MAV:14552  │ │ MAV:14553  │     │
│  │ DDS:8888   │ │ DDS:8889   │ │ DDS:8890   │ │ DDS:8891   │     │
│  └────────────┘ └────────────┘ └────────────┘ └────────────┘     │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  ./PX4-Autopilot (Shared Volume - Single Git Clone)          │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────────────┐
│                    Windows Host (QGroundControl)                    │
│                                                                    │
│  QGroundControl connects to each vehicle via:                      │
│    - UDP 14550 (Vehicle 0)                                         │
│    - UDP 14551 (Vehicle 1)                                         │
│    - UDP 14552 (Vehicle 2)                                         │
│    - UDP 14553 (Vehicle 3)                                         │
└────────────────────────────────────────────────────────────────────┘
```

---

## Quick Start

### 1. Generate Docker Compose File

```bash
cd px4-sitl-container

# Generate for 2 copters
./generate-compose.sh --copters 2

# Or use interactive mode
./generate-compose.sh --interactive
```

### 2. Start Gazebo (in ROS2 Container)

```bash
# Start ROS2 container
cd ../ros2-gazebo-desktop
docker compose up -d

# Access VNC and start Gazebo
# Browser: http://localhost:6901/vnc.html
gz sim -r default.sdf
```

### 3. Start micro-ROS Agents

```bash
# In ROS2 container (one agent per vehicle)
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 &
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8889 &
```

### 4. Start PX4 Vehicles

```bash
cd px4-sitl-container
docker compose -f docker-compose-generated.yml up -d
```

### 5. Connect QGroundControl

Open QGroundControl on Windows and add UDP connections for each vehicle port.

---

## Docker Compose Generator

The `generate-compose.sh` script creates custom Docker Compose configurations for any combination of vehicles.

### Usage

```bash
./generate-compose.sh [OPTIONS]
```

### Vehicle Options

| Option | Description | Model | Autostart ID |
|--------|-------------|-------|--------------|
| `--copters N` | Number of quadcopters | x500 | 4001 |
| `--vtols N` | Number of VTOLs | standard_vtol | 13000 |
| `--planes N` | Number of fixed-wing planes | rc_cessna | 2106 |
| `--rovers N` | Number of ground rovers | r1_rover | 50000 |
| `--boats N` | Number of boats/surface vessels | boat | 60000 |

### Configuration Options

| Option | Description | Default |
|--------|-------------|---------|
| `--output FILE` | Output filename | docker-compose-generated.yml |
| `--host-ip IP` | Windows host IP for QGroundControl | Auto-detect |
| `--interactive` | Interactive mode with prompts | - |
| `--list-models` | Show all available vehicle models | - |
| `--help` | Show help message | - |

### Examples

```bash
# 2 quadcopters
./generate-compose.sh --copters 2

# 2 copters + 1 VTOL
./generate-compose.sh --copters 2 --vtols 1

# Mixed fleet: 2 copters + 1 rover
./generate-compose.sh --copters 2 --rovers 1

# Full mixed fleet
./generate-compose.sh --copters 2 --vtols 1 --planes 1 --rovers 2 --boats 1

# With specific Windows host IP
./generate-compose.sh --copters 3 --host-ip 192.168.18.93

# Custom output filename
./generate-compose.sh --copters 2 --output my-fleet.yml
```

### Interactive Mode

Run without arguments or with `--interactive` for guided setup:

```bash
./generate-compose.sh --interactive
```

Output:
```
============================================
  PX4 Multi-Vehicle Docker Compose Generator
============================================

How many vehicles of each type?

Air Vehicles:
-------------
  Quadcopters (x500) [0]: 2
  VTOLs (standard_vtol) [0]: 1
  Fixed-wing Planes (rc_cessna) [0]: 0

Ground/Water Vehicles:
----------------------
  Ground Rovers (r1_rover) [0]: 1
  Boats/Surface Vessels [0]: 0

Configuration:
--------------
  Windows Host IP (leave blank for auto-detect): 192.168.18.93
  Output file [docker-compose-generated.yml]:
```

### List Available Models

```bash
./generate-compose.sh --list-models
```

Output:
```
Available Vehicle Models:
=========================

  Type      Model           Autostart   Description
  ----      -----           ---------   -----------
  copter    x500            4001        Standard quadcopter
  vtol      standard_vtol   13000       Quad + fixed wing VTOL
  plane     rc_cessna       2106        Fixed-wing RC plane
  rover     r1_rover        50000       Differential drive rover
  boat      boat            60000       Surface vessel / boat
```

---

## Vehicle Types

### Air Vehicles

#### Quadcopter (x500)
- **Model:** x500
- **Autostart:** 4001
- **Description:** Standard quadcopter with 4 motors in X configuration
- **Use case:** General purpose aerial operations, photography, inspection

#### VTOL (standard_vtol)
- **Model:** standard_vtol
- **Autostart:** 13000
- **Description:** Vertical takeoff and landing aircraft with quad rotors + pusher motor
- **Use case:** Long-range missions requiring vertical takeoff capability

#### Fixed-Wing Plane (rc_cessna)
- **Model:** rc_cessna
- **Autostart:** 2106
- **Description:** Traditional fixed-wing aircraft
- **Use case:** Long-endurance missions, aerial mapping

### Ground/Water Vehicles

#### Ground Rover (r1_rover)
- **Model:** r1_rover
- **Autostart:** 50000
- **Description:** Differential drive ground robot
- **Use case:** Ground-based operations, delivery, inspection

#### Boat (boat)
- **Model:** boat
- **Autostart:** 60000
- **Description:** Surface vessel / boat
- **Use case:** Water-based operations, maritime applications

---

## Port Reference

### Port Allocation Formula

Each vehicle instance gets unique ports based on its instance number:

| Port Type | Formula | Instance 0 | Instance 1 | Instance 2 |
|-----------|---------|------------|------------|------------|
| MAVLink (QGC) | 14550 + N | 14550 | 14551 | 14552 |
| MAVLink (Offboard) | 14540 + N | 14540 | 14541 | 14542 |
| uXRCE-DDS (ROS 2) | 8888 + N | 8888 | 8889 | 8890 |

### Port Summary Table

| Service | Port | Protocol | Description |
|---------|------|----------|-------------|
| **Gazebo Transport** | 11345 | TCP | Gazebo discovery and communication |
| **MAVLink QGC** | 14550+ | UDP | QGroundControl communication |
| **MAVLink Offboard** | 14540+ | UDP | MAVROS / offboard control |
| **uXRCE-DDS** | 8888+ | UDP | micro-ROS agent (ROS 2 topics) |
| **VNC** | 5901 | TCP | VNC server (ROS2 container) |
| **noVNC** | 6901 | HTTP | Web-based VNC access |

### Example: 3 Vehicle Fleet

| Vehicle | Type | Model | MAVLink | Offboard | DDS |
|---------|------|-------|---------|----------|-----|
| Vehicle 0 | Copter | x500_0 | 14550 | 14540 | 8888 |
| Vehicle 1 | Copter | x500_1 | 14551 | 14541 | 8889 |
| Vehicle 2 | Rover | r1_rover_2 | 14552 | 14542 | 8890 |

---

## Single PX4 Source (FAQ)

### Q: Do I need multiple PX4-Autopilot git clones for multiple vehicles?

**A: No!** You only need **one PX4-Autopilot git clone**.

### How It Works

1. **Shared Source Code:** All PX4 SITL containers mount the same `./PX4-Autopilot` directory
2. **Instance Isolation:** Each container uses a different `PX4_SIM_INSTANCE` number
3. **Separate State:** Instance numbers create separate:
   - Parameter files
   - Log files
   - MAVLink ports
   - Model names (x500_0, x500_1, etc.)

### Directory Structure

```
px4-sitl-container/
├── PX4-Autopilot/           # Single git clone (shared by all containers)
│   ├── build/
│   │   └── px4_sitl_default/
│   │       ├── instance_0/  # State for vehicle 0
│   │       ├── instance_1/  # State for vehicle 1
│   │       └── instance_2/  # State for vehicle 2
│   ├── Tools/
│   │   └── simulation/
│   │       └── gz/
│   │           └── models/  # Gazebo models (shared)
│   └── ...
├── docker-compose.yml
├── docker-compose-generated.yml
└── generate-compose.sh
```

### Important Notes

- ✅ **One git clone** for unlimited vehicles
- ✅ **Shared model files** - no duplication
- ✅ **Separate runtime state** - each instance is isolated
- ⚠️ **Rebuild once** - compile PX4 once, use for all instances
- ⚠️ **Same PX4 version** - all vehicles use the same firmware version

### Cloning PX4-Autopilot

If you haven't cloned yet:

```bash
cd px4-sitl-container

# Clone with submodules (REQUIRED!)
git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.16.0

# The --recursive flag downloads all Gazebo models and dependencies
```

---

## Running the Simulation

### Complete Workflow

#### Step 1: Prepare Environment

```bash
# Ensure PX4-Autopilot is cloned with submodules
cd px4-sitl-container
ls PX4-Autopilot/Tools/simulation/gz/models/  # Should show vehicle models
```

#### Step 2: Generate Docker Compose

```bash
# Example: 2 copters + 1 rover
./generate-compose.sh --copters 2 --rovers 1
```

#### Step 3: Start ROS2 Container with Gazebo

```bash
# Terminal 1
cd ../ros2-gazebo-desktop
docker compose up -d

# Access VNC: http://localhost:6901/vnc.html
# In VNC terminal, start Gazebo:
gz sim -r default.sdf
```

#### Step 4: Start micro-ROS Agents

```bash
# Terminal 2 (in ROS2 container)
docker exec -it ros2_gazebo bash

# Start one agent per vehicle
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 &
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8889 &
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8890 &
```

#### Step 5: Start PX4 Vehicles

```bash
# Terminal 3
cd px4-sitl-container
docker compose -f docker-compose-generated.yml up -d

# View logs
docker compose -f docker-compose-generated.yml logs -f
```

#### Step 6: Connect QGroundControl

On Windows host:
1. Open QGroundControl
2. Go to: **Q icon → Application Settings → Comm Links**
3. Add UDP connections for each vehicle:
   - Vehicle 0: UDP port 14550
   - Vehicle 1: UDP port 14551
   - Vehicle 2: UDP port 14552

### Managing the Simulation

```bash
# View all container logs
docker compose -f docker-compose-generated.yml logs -f

# View specific vehicle logs
docker compose -f docker-compose-generated.yml logs -f px4-copter-0

# Stop all vehicles
docker compose -f docker-compose-generated.yml down

# Restart a specific vehicle
docker compose -f docker-compose-generated.yml restart px4-copter-0

# Scale (not recommended - use generator instead)
docker compose -f docker-compose-generated.yml up -d --scale px4-copter-0=0
```

---

## QGroundControl Setup

### Connecting Multiple Vehicles

QGroundControl can connect to multiple vehicles simultaneously.

#### Add Vehicle Connections

1. Open QGroundControl
2. Click **Q icon** (top-left) → **Application Settings**
3. Select **Comm Links**
4. For each vehicle, click **Add** and configure:

| Setting | Vehicle 0 | Vehicle 1 | Vehicle 2 |
|---------|-----------|-----------|-----------|
| Name | Copter 0 | Copter 1 | Rover 0 |
| Type | UDP | UDP | UDP |
| Listening Port | 14550 | 14551 | 14552 |
| Target Host | (leave blank) | (leave blank) | (leave blank) |

5. Click **OK** for each
6. Select each connection and click **Connect**

### Vehicle Selection in QGC

- Use the **vehicle selector** (top toolbar) to switch between vehicles
- Each vehicle shows its own:
  - Flight mode
  - Battery status
  - GPS position
  - Sensor status

### Multi-Vehicle Mission Planning

1. Select target vehicle from vehicle selector
2. Plan mission as normal
3. Upload mission to selected vehicle
4. Repeat for other vehicles

---

## Troubleshooting

### Common Issues

#### Vehicles not spawning in Gazebo

**Symptoms:** PX4 starts but no vehicle appears in Gazebo

**Solutions:**
1. Ensure Gazebo is running before starting PX4
2. Check GZ_PARTITION matches in both containers
3. Verify Gazebo transport: `gz topic -l | grep clock`

#### Sensor errors (gyro, barometer missing)

**Symptoms:**
```
WARN [health_and_arming_checks] Preflight Fail: Gyro Sensor 0 missing
```

**Solutions:**
1. Ensure micro-ROS agent is running BEFORE PX4 starts
2. Check Gazebo has spawned the vehicle model with sensors
3. Verify model name matches: `gz topic -l | grep x500_0`

#### QGroundControl not connecting

**Symptoms:** QGC shows "Waiting for Vehicle"

**Solutions:**
1. Verify Windows host IP is correct in `.env`
2. Check Windows firewall allows UDP on ports 14550+
3. Verify MAVLink is configured: At PX4 `pxh>` prompt: `mavlink status`

#### Port conflicts

**Symptoms:** "Address already in use" errors

**Solutions:**
1. Stop all containers: `docker compose down`
2. Check for running processes: `netstat -tulpn | grep 14550`
3. Kill conflicting processes or use different ports

#### Duplicate log messages

**Symptoms:** Each log line appears twice

**Solutions:**
1. The `rc.autostart.post` hook stops default MAVLink
2. If still seeing duplicates, check for multiple PX4 processes

### Diagnostic Commands

```bash
# Check Gazebo topics
gz topic -l | grep -E "(clock|model)"

# Check PX4 MAVLink status (in PX4 container)
docker attach px4_copter_0
# At pxh> prompt:
mavlink status

# Check sensor data
listener sensor_combined

# Check micro-ROS agent connections
ros2 topic list | grep fmu

# Check container networking
docker network inspect bridge
```

### Getting Help

- PX4 Documentation: https://docs.px4.io/
- Gazebo Documentation: https://gazebosim.org/docs
- QGroundControl: https://docs.qgroundcontrol.com/

---

## Summary

| Component | Single Vehicle | Multiple Vehicles |
|-----------|---------------|-------------------|
| PX4-Autopilot Clone | 1 | 1 (shared) |
| Docker Containers | 1 | N (one per vehicle) |
| Gazebo Instance | 1 | 1 (shared) |
| micro-ROS Agents | 1 | N (one per vehicle) |
| MAVLink Ports | 14550 | 14550, 14551, ... |
| uXRCE-DDS Ports | 8888 | 8888, 8889, ... |

**Key Points:**
- ✅ One PX4 git clone for all vehicles
- ✅ One Gazebo instance for all vehicles
- ✅ One Docker container per vehicle
- ✅ One micro-ROS agent per vehicle
- ✅ Unique ports per vehicle (auto-calculated)
