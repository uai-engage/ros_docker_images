# Native Ubuntu Setup for ROS 2 + Gazebo + PX4

**Complete native installation of ROS 2 Jazzy, Gazebo Harmonic, and PX4 Autopilot v1.16.0 on Ubuntu 24.04 Desktop.**

This eliminates all Docker-related networking, timing, and sensor bridge issues by running everything directly on the host.

## Why Native Installation?

✅ **No Docker networking issues** - Direct host networking
✅ **No sensor timing problems** - Native Gazebo-PX4 communication
✅ **Faster performance** - No containerization overhead
✅ **Easier debugging** - All logs accessible directly
✅ **Simpler QGroundControl connection** - Direct network access
✅ **GPU acceleration** - Native graphics card access

## Quick Installation

```bash
cd native-ubuntu-setup

# 1. Check system requirements
./scripts/00-check-system.sh

# 2. Run installation scripts (1-1.5 hours total)
cd scripts
./01-install-ros2-jazzy.sh        # 15-20 min
./02-install-gazebo-harmonic.sh    # 10-15 min
./03-install-px4-deps.sh           # 5-10 min
./04-setup-px4.sh                  # 20-30 min
./05-install-microros.sh           # 5 min

# 3. Configure environment
cd ..
./configure-environment.sh

# 4. Test installation
source ~/.bashrc
./test-installation.sh
```

## Quick Start (After Installation)

Open 4 terminals:

**Terminal 1 - Gazebo:**
```bash
gz-default
```

**Terminal 2 - PX4 SITL:**
```bash
px4-sitl
```

**Terminal 3 - micro-ROS Agent:**
```bash
microros
```

**Terminal 4 - Monitor ROS 2 Topics:**
```bash
px4-topics
ros2 topic echo /fmu/out/vehicle_status
```

## Connect QGroundControl

### From Same Ubuntu Machine:
- Auto-discovers on UDP 14550

### From Remote Windows Machine:
1. Get Ubuntu IP: `ip addr show | grep inet`
2. In QGroundControl:
   - Type: **TCP** (more reliable)
   - Server: **<ubuntu-ip>**
   - Port: **5760**

## What Gets Installed

| Component | Version | Location |
|-----------|---------|----------|
| ROS 2 | Jazzy | `/opt/ros/jazzy` |
| Gazebo | Harmonic | System-wide |
| PX4 Autopilot | v1.16.0 | `~/px4_workspace/PX4-Autopilot` |
| micro-ROS Agent | Jazzy | `~/px4_workspace/micro_ros_ws` |

## Documentation

- **[docs/INSTALLATION-GUIDE.md](docs/INSTALLATION-GUIDE.md)** - Detailed installation guide
- **[docs/USAGE-GUIDE.md](docs/USAGE-GUIDE.md)** - How to use the system
- **[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)** - Common issues and fixes

## System Requirements

- Ubuntu 24.04 LTS Desktop
- 8GB RAM minimum (16GB recommended)
- 30GB free disk space
- Internet connection
- sudo privileges

## Useful Aliases (Auto-configured)

After running `configure-environment.sh`, these aliases are available:

| Alias | Command |
|-------|---------|
| `px4-sitl` | Start PX4 with Gazebo x500 quadcopter |
| `px4-console` | Start PX4 console without Gazebo |
| `gz-default` | Start Gazebo with default world |
| `microros` | Start micro-ROS agent for ROS 2 bridge |
| `px4-topics` | List all PX4 ROS 2 topics |
| `px4-status` | Echo vehicle status topic |

## Directory Structure

```
~/px4_workspace/
├── PX4-Autopilot/              # PX4 source and build
│   ├── build/px4_sitl_default/
│   ├── Tools/simulation/gz/
│   └── ...
├── micro_ros_ws/               # micro-ROS agent workspace
│   ├── src/
│   └── install/
└── ros2_ws/                    # Your ROS 2 workspace (optional)
```

## Advantages Over Docker Setup

| Feature | Docker | Native |
|---------|--------|--------|
| Sensor Bridge | ❌ Timing issues | ✅ Works perfectly |
| MAVLink TCP | ❌ Binding issues | ✅ Direct access |
| Performance | ⚠️ Slower | ✅ Full speed |
| GPU Access | ⚠️ Limited | ✅ Full acceleration |
| Debugging | ⚠️ Complex | ✅ Simple |
| QGC Connection | ⚠️ Tunneling needed | ✅ Direct connection |

## Support

For issues:
1. Check `docs/TROUBLESHOOTING.md`
2. Run `./test-installation.sh` to diagnose
3. Review installation logs
4. Open an issue on GitHub

## Uninstallation

To remove everything:
```bash
./scripts/99-uninstall-all.sh
```

**Warning**: This removes ROS 2, Gazebo, and all PX4 files!

## License

Follows the licenses of individual components:
- ROS 2: Apache 2.0
- Gazebo: Apache 2.0
- PX4 Autopilot: BSD 3-Clause
