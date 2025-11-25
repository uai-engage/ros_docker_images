# Quick Reference Card

## Installation (One-Time)

```bash
cd native-ubuntu-setup/scripts
./00-check-system.sh            # Check requirements
./01-install-ros2-jazzy.sh      # 15-20 min
./02-install-gazebo-harmonic.sh # 10-15 min
./03-install-px4-deps.sh        # 5-10 min
./04-setup-px4.sh               # 20-30 min
./05-install-microros.sh        # 5 min
cd ..
./configure-environment.sh      # Add to ~/.bashrc
source ~/.bashrc
./test-installation.sh          # Verify
```

## Daily Usage

### Start System (4 Terminals)

```bash
# Terminal 1: Gazebo
gz-default

# Terminal 2: PX4 SITL
px4-sitl

# Terminal 3: micro-ROS
microros

# Terminal 4: Monitor
px4-topics
px4-status
```

### Quick Aliases

| Alias | Action |
|-------|--------|
| `gz-default` | Start Gazebo with default world |
| `px4-sitl` | Start PX4 with x500 quadcopter |
| `microros` | Start micro-ROS agent |
| `px4-topics` | List PX4 ROS 2 topics |
| `px4-status` | Show vehicle status |

## PX4 Console Commands

```
commander arm          # Arm vehicle
commander takeoff      # Takeoff
commander land         # Land
commander disarm       # Disarm
mavlink status         # Show connections
shutdown               # Stop PX4
```

## QGroundControl

**From Windows (TCP):**
- Type: TCP
- Server: `<ubuntu-ip>`
- Port: 5760

**From Same Machine:**
- Auto-discovers on UDP 14550

## Troubleshooting

```bash
# Test installation
./test-installation.sh

# Check if running
ps aux | grep -E "gz|px4|micro_ros"

# Check MAVLink ports
netstat -tulpn | grep -E "14550|5760|8888"

# View PX4 logs
ls -lt ~/px4_workspace/PX4-Autopilot/build/px4_sitl_default/logs/
```

## File Locations

```
/opt/ros/jazzy/                           # ROS 2
~/px4_workspace/PX4-Autopilot/            # PX4 source
~/px4_workspace/micro_ros_ws/             # micro-ROS
~/px4_workspace/PX4-Autopilot/Tools/simulation/gz/models/  # Models
~/px4_workspace/PX4-Autopilot/Tools/simulation/gz/worlds/  # Worlds
```

## Key Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 14550 | UDP | MAVLink (QGC) |
| 14540 | UDP | MAVLink (offboard) |
| 5760 | TCP | MAVLink (QGC remote) |
| 8888 | UDP | uXRCE-DDS (micro-ROS) |

## Common Issues

**Sensors missing:**
- Make sure Gazebo started first
- Check gz topics: `gz topic -l`

**QGC won't connect:**
- Open firewall: `sudo ufw allow 5760/tcp`
- Check PX4 running: `ps aux | grep px4`

**No ROS 2 topics:**
- micro-ROS agent running?
- Check: `ros2 topic list`

## Documentation

- `README.md` - Overview
- `docs/INSTALLATION-GUIDE.md` - Full install guide
- `docs/USAGE-GUIDE.md` - Detailed usage
- `docs/TROUBLESHOOTING.md` - Fix problems
