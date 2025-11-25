# Ubuntu Desktop Container for ROS 2 + Gazebo + PX4

**Run a complete Ubuntu 24.04 Desktop environment in Docker with ROS 2, Gazebo, and PX4 installed natively inside.**

## Concept

This is a **hybrid approach** combining:
- ✅ **Docker's isolation** - Clean, reproducible environment
- ✅ **Native installation benefits** - No multi-container networking issues
- ✅ **VNC desktop access** - Full GUI environment in your browser
- ✅ **Persistent workspace** - Your work survives container restarts

Everything (ROS 2, Gazebo, PX4) runs **natively inside a single Ubuntu Desktop container**.

## Quick Start

### 1. Build and Start Container

```bash
cd docker

# Copy environment configuration
cp .env.example .env

# Build the container (first time only, ~5-10 minutes)
docker compose build

# Start the container
docker compose up -d
```

### 2. Access Ubuntu Desktop

Open your web browser:
```
http://localhost:6901/vnc.html
```

**Password**: `password` (can be changed in `.env`)

You'll see a full Ubuntu XFCE desktop!

### 3. Install ROS 2 + Gazebo + PX4

Inside the VNC desktop, open a terminal and run:

**Option A: Automated Installation** (recommended)
```bash
cd ~
./install-all.sh
```

This runs all installation scripts automatically (~1-1.5 hours).

**Option B: Manual Step-by-Step**
```bash
cd ~/installation-scripts

./00-check-system.sh            # Check requirements
./01-install-ros2-jazzy.sh      # 15-20 min
./02-install-gazebo-harmonic.sh # 10-15 min
./03-install-px4-deps.sh        # 5-10 min
./04-setup-px4.sh               # 20-30 min
./05-install-microros.sh        # 5 min

cd ~
./configure-environment.sh      # Configure environment
source ~/.bashrc
```

### 4. Use the System

After installation, open multiple terminals in the VNC desktop:

**Terminal 1: Gazebo**
```bash
gz-default
```

**Terminal 2: PX4 SITL**
```bash
px4-sitl
```

**Terminal 3: micro-ROS Agent**
```bash
microros
```

**Terminal 4: Monitor ROS 2**
```bash
px4-topics
px4-status
```

### 5. Connect QGroundControl

From your **host machine** (Windows/Mac/Linux):

**TCP Connection (Recommended):**
- Type: **TCP**
- Server: **localhost**
- Port: **5760**

**UDP Connection:**
- Type: **UDP**
- Port: **14550**

## Architecture

```
┌─────────────────────────────────────────────────┐
│  Docker Container: ubuntu-desktop               │
│                                                  │
│  ┌────────────────────────────────────────────┐ │
│  │  Ubuntu 24.04 Desktop (XFCE)               │ │
│  │  Access via VNC: http://localhost:6901     │ │
│  │                                            │ │
│  │  ┌──────────────────────────────────────┐ │ │
│  │  │  ROS 2 Jazzy (/opt/ros/jazzy)        │ │ │
│  │  └──────────────────────────────────────┘ │ │
│  │                                            │ │
│  │  ┌──────────────────────────────────────┐ │ │
│  │  │  Gazebo Harmonic (system-wide)       │ │ │
│  │  └──────────────────────────────────────┘ │ │
│  │                                            │ │
│  │  ┌──────────────────────────────────────┐ │ │
│  │  │  PX4 Autopilot v1.16.0               │ │ │
│  │  │  ~/px4_workspace/PX4-Autopilot       │ │ │
│  │  │  (mounted from ./workspace volume)   │ │ │
│  │  └──────────────────────────────────────┘ │ │
│  │                                            │ │
│  │  ┌──────────────────────────────────────┐ │ │
│  │  │  micro-ROS Agent                     │ │ │
│  │  │  ~/px4_workspace/micro_ros_ws        │ │ │
│  │  └──────────────────────────────────────┘ │ │
│  └────────────────────────────────────────────┘ │
│                                                  │
│  Network: host mode (direct access to ports)    │
└─────────────────────────────────────────────────┘
        ↓ ports: 5901, 6901, 14550, 5760
┌─────────────────────────────────────────────────┐
│  Host Machine (Windows/Mac/Linux)               │
│  - Web Browser: http://localhost:6901           │
│  - QGroundControl: localhost:5760               │
└─────────────────────────────────────────────────┘
```

## Advantages of This Approach

| Feature | Multi-Container Docker | This Approach | Native Host |
|---------|----------------------|---------------|-------------|
| Setup Complexity | ⚠️ Complex networking | ✅ Simple | ✅ Simple |
| Sensor Bridge | ❌ Timing issues | ✅ Works perfectly | ✅ Works perfectly |
| MAVLink TCP | ❌ Binding problems | ✅ Works perfectly | ✅ Works perfectly |
| GUI Access | ⚠️ X11 forwarding | ✅ Built-in VNC | ✅ Direct |
| Isolation | ✅ Isolated | ✅ Isolated | ❌ Affects host |
| Portability | ⚠️ Complex | ✅ Single container | ❌ Host-specific |
| Persistence | ⚠️ Multi-volume | ✅ One volume | ✅ Direct |

## File Persistence

### What Persists Across Container Restarts:

✅ **`~/px4_workspace/`** - Mounted from `./workspace` on host
  - PX4-Autopilot source
  - micro-ROS workspace
  - Your ROS 2 projects
  - Build outputs

✅ **Container filesystem** - As long as you use `docker compose stop/start`
  - ROS 2 installation (`/opt/ros/jazzy`)
  - Gazebo installation
  - All system packages
  - User home directory (except px4_workspace)

### What's Lost:

❌ **If you run `docker compose down`**
  - Container filesystem (ROS 2, Gazebo, installed packages)
  - User configurations outside `~/px4_workspace`

**Solution**: Commit the container after installation (see below)

## Committing Configured Container

After installing everything, save the container state:

```bash
# Stop the container
docker compose stop

# Commit it to an image
docker commit ubuntu_desktop_dev ubuntu-desktop-ros2-px4:configured

# Now you can recreate containers from this pre-configured image
# Update docker-compose.yml to use: image: ubuntu-desktop-ros2-px4:configured
```

## Common Commands

### Container Management

```bash
# Start container
docker compose up -d

# Stop container (keeps data)
docker compose stop

# Restart container
docker compose restart

# View logs
docker compose logs -f

# Remove container (WARNING: loses non-volume data)
docker compose down

# Rebuild container
docker compose build --no-cache
```

### Access Methods

**Web Browser (noVNC):**
```
http://localhost:6901/vnc.html
```

**VNC Client:**
```
Server: localhost:5901
Password: password
```

**Terminal Access (no GUI):**
```bash
docker exec -it ubuntu_desktop_dev bash
```

## Configuration

Edit `.env` file:

```env
# User settings
USERNAME=developer
USER_UID=1000
USER_GID=1000

# Container name
CONTAINER_NAME=ubuntu_desktop_dev

# Workspace (persistent)
WORKSPACE_PATH=./workspace

# VNC password
VNC_PASSWORD=password

# Resources
MEMORY_LIMIT=16G
CPU_LIMIT=8
```

## GPU Support (NVIDIA)

To enable GPU acceleration for Gazebo:

1. Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

2. Uncomment GPU sections in `docker-compose.yml`:
```yaml
environment:
  - NVIDIA_VISIBLE_DEVICES=all
  - NVIDIA_DRIVER_CAPABILITIES=all

deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [gpu]
```

3. Rebuild and start:
```bash
docker compose build
docker compose up -d
```

## Troubleshooting

### Can't access VNC

Check if container is running:
```bash
docker ps | grep ubuntu_desktop
```

Check logs:
```bash
docker compose logs
```

Restart container:
```bash
docker compose restart
```

### Installation fails

Check available disk space:
```bash
df -h
```

Check container resources:
```bash
docker stats ubuntu_desktop_dev
```

Increase memory limit in `.env`:
```env
MEMORY_LIMIT=20G
```

### QGroundControl can't connect

Check if PX4 is running inside container:
```bash
docker exec ubuntu_desktop_dev ps aux | grep px4
```

Check port mapping (if not using host mode):
```bash
docker port ubuntu_desktop_dev
```

### Performance is slow

1. Increase allocated resources in `.env`
2. Enable GPU support (if NVIDIA GPU available)
3. Close unnecessary applications on host

## Backup and Restore

### Backup Workspace

The `./workspace` directory contains all your important data:

```bash
# Backup
tar czf px4-workspace-backup-$(date +%Y%m%d).tar.gz workspace/

# Restore
tar xzf px4-workspace-backup-YYYYMMDD.tar.gz
```

### Export Configured Container

```bash
# Save container to tar file
docker save ubuntu-desktop-ros2-px4:configured -o ubuntu-desktop-configured.tar

# Load on another machine
docker load -i ubuntu-desktop-configured.tar
```

## System Requirements

- **Docker** installed and running
- **8GB RAM minimum** (16GB recommended)
- **50GB free disk space** (30GB for container + 20GB for workspace)
- **Modern CPU** (4+ cores recommended)
- **Web browser** (for noVNC access)

## Next Steps

After installation:
- See `~/installation-docs/USAGE-GUIDE.md` for detailed usage
- Try example missions and scripts
- Develop your own ROS 2 nodes
- Connect external sensors or hardware

## Support

For issues:
1. Check container logs: `docker compose logs`
2. Check VNC connection
3. Verify resource allocation
4. Review `~/installation.log` inside container
