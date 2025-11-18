# Directory Structure

This document explains the organization of files in the `ros2-gazebo-desktop` directory.

## Overview

```
ros2-gazebo-desktop/
├── Dockerfile                          # Main ROS 2 Desktop + Gazebo image
├── docker-compose.yml                  # Main desktop Docker Compose configuration
├── README.md                           # Main project documentation
├── ROS2_NETWORKING.md                  # ROS 2 networking guide (general)
├── startup-vnc.sh                      # VNC startup script (used by Dockerfile)
├── .env.vnc                           # VNC environment variables
│
├── docs/                              # Documentation for main desktop setup
│   ├── ARDUPILOT_MICRO_ROS_INTEGRATION.md  # ArduPilot integration guide
│   ├── QUICK_START_ARDUPILOT.md            # Quick start for ArduPilot
│   ├── MICRO_ROS_MANUAL_INSTALL.md         # Manual micro-ROS installation
│   ├── BUILD_FIX_NOTES.md                  # Build troubleshooting notes
│   └── STOP_MICRO_ROS.md                   # How to stop micro-ROS agent
│
├── scripts/                           # Scripts for main desktop setup
│   ├── install_micro_ros.sh               # Install micro-ROS agent
│   └── stop_micro_ros.sh                  # Stop agent and clear cache
│
└── companion/                         # Companion computer setup (minimal)
    ├── Dockerfile                         # Minimal companion image
    ├── docker-compose.yml                 # Companion Docker Compose
    ├── .env.example                       # Environment configuration template
    ├── README.md                          # Companion quick start guide
    │
    ├── docs/                              # Companion documentation
    │   ├── COMPANION_COMPUTER_SETUP.md    # Complete setup guide
    │   └── SITL_TESTING_GUIDE.md          # SITL testing guide
    │
    ├── systemd/                           # Systemd service files
    │   └── micro-ros-agent.service        # Auto-start service
    │
    └── scripts/                           # Companion helper scripts
        └── start_agent.sh                 # Flexible agent startup script
```

## Directory Descriptions

### Root Level

**Main Desktop Files:**
- `Dockerfile` - Full ROS 2 Jazzy Desktop + Gazebo Harmonic image (~5.5 GB)
- `docker-compose.yml` - Docker Compose for ground station / development
- `startup-vnc.sh` - VNC server initialization (referenced by Dockerfile)

**General Documentation:**
- `README.md` - Project overview and main documentation
- `ROS2_NETWORKING.md` - General ROS 2 networking information
- `DIRECTORY_STRUCTURE.md` - This file

### docs/

Documentation specific to the main ROS 2 desktop setup and ArduPilot integration:

- **ARDUPILOT_MICRO_ROS_INTEGRATION.md** - Complete guide for ArduPilot ↔ ROS 2 integration
- **QUICK_START_ARDUPILOT.md** - Quick start commands and examples
- **MICRO_ROS_MANUAL_INSTALL.md** - Manual installation testing guide
- **BUILD_FIX_NOTES.md** - Build issues and solutions (Fast DDS conflicts, etc.)
- **STOP_MICRO_ROS.md** - How to stop agent and clear ghost topics

### scripts/

Helper scripts for the main desktop environment:

- **install_micro_ros.sh** - Install micro-ROS agent from source (for desktop)
- **stop_micro_ros.sh** - Stop agent gracefully and clear DDS cache

### companion/

Everything needed for minimal companion computer deployment on drone:

**Root Files:**
- `Dockerfile` - Minimal ROS 2 Base + micro-ROS agent (~800 MB)
- `docker-compose.yml` - Flexible compose with UDP/TCP/Serial modes
- `.env.example` - Configuration template for easy customization
- `README.md` - Quick start guide for companion computer

### companion/docs/

Detailed companion computer documentation:

- **COMPANION_COMPUTER_SETUP.md** - Complete setup guide with 3 installation methods
- **SITL_TESTING_GUIDE.md** - ArduPilot SITL testing with UDP mode

### companion/systemd/

System service files for native installations:

- **micro-ros-agent.service** - Systemd unit for auto-start on boot

### companion/scripts/

Helper scripts for companion computer:

- **start_agent.sh** - Intelligent startup script with mode selection

## Usage

### Ground Station / Development Setup

```bash
cd ros2-gazebo-desktop

# Build and run main desktop image
docker compose up -d

# Access VNC at http://localhost:6080/vnc.html

# Install micro-ROS agent inside container
docker exec -it ros2_gazebo_vnc bash
cd ~/ros2_ws
../scripts/install_micro_ros.sh
```

### Companion Computer Setup

```bash
cd ros2-gazebo-desktop/companion

# Quick start (UDP mode for SITL)
docker compose up -d

# Custom configuration
cp .env.example .env
nano .env  # Edit settings
docker compose up -d

# Check logs
docker logs -f micro_ros_agent_companion
```

### Documentation Access

**For main desktop setup:**
- Read `README.md` first
- Check `docs/QUICK_START_ARDUPILOT.md` for getting started
- See `docs/ARDUPILOT_MICRO_ROS_INTEGRATION.md` for complete integration guide

**For companion computer:**
- Read `companion/README.md` for quick start
- Check `companion/docs/SITL_TESTING_GUIDE.md` for SITL testing
- See `companion/docs/COMPANION_COMPUTER_SETUP.md` for detailed deployment options

## File Relationships

### Docker Build Context

**Main Desktop:**
- `docker-compose.yml` → builds `Dockerfile` (same directory)
- `Dockerfile` → copies `startup-vnc.sh` during build

**Companion:**
- `companion/docker-compose.yml` → builds `companion/Dockerfile`
- Both use `host` network mode for ROS 2 DDS discovery

### Documentation Cross-References

- `README.md` → references `docs/QUICK_START_ARDUPILOT.md`
- `companion/README.md` → references `companion/docs/COMPANION_COMPUTER_SETUP.md`
- `docs/ARDUPILOT_MICRO_ROS_INTEGRATION.md` → references `docs/STOP_MICRO_ROS.md`

### Script Dependencies

- `scripts/install_micro_ros.sh` → used in main desktop container
- `companion/scripts/start_agent.sh` → used with native companion installation
- `scripts/stop_micro_ros.sh` → clears ghost topics (both desktop and companion)

## Migration from Old Structure

If you have files from the old flat structure, here's the mapping:

| Old Location | New Location |
|-------------|-------------|
| `Dockerfile.companion` | `companion/Dockerfile` |
| `docker-compose.companion.yml` | `companion/docker-compose.yml` |
| `.env.companion.example` | `companion/.env.example` |
| `COMPANION_COMPUTER_*.md` | `companion/docs/*.md` or `companion/README.md` |
| `SITL_TESTING_GUIDE.md` | `companion/docs/SITL_TESTING_GUIDE.md` |
| `ARDUPILOT_*.md` | `docs/ARDUPILOT_*.md` |
| `BUILD_FIX_NOTES.md` | `docs/BUILD_FIX_NOTES.md` |
| `systemd/*` | `companion/systemd/*` |
| `install_micro_ros.sh` | `scripts/install_micro_ros.sh` |
| `stop_micro_ros.sh` | `scripts/stop_micro_ros.sh` |
| `start_agent.sh` | `companion/scripts/start_agent.sh` |

## Design Rationale

**Separation of Concerns:**
- Main desktop (5.5 GB, full features) vs Companion (800 MB, minimal)
- Each has its own Dockerfile and docker-compose.yml
- Clear separation prevents confusion

**Documentation Organization:**
- Root-level docs are general or main-desktop specific
- Companion-specific docs are in `companion/docs/`
- Quick start guides are at `README.md` level for each

**Scripts Organization:**
- Root `scripts/` for desktop environment
- `companion/scripts/` for companion-specific tools
- Prevents confusion about which script is for which environment

**Self-Contained Companion Directory:**
- Can be copied to companion computer independently
- All necessary files in one place
- Clear `.env.example` for configuration
