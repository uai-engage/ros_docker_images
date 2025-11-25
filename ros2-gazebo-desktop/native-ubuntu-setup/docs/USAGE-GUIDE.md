# Usage Guide - Native Ubuntu Setup

Comprehensive guide for using ROS 2 Jazzy + Gazebo Harmonic + PX4 Autopilot on Ubuntu 24.04.

## Daily Workflow

### Starting the System

**1. Start Gazebo (Terminal 1):**
```bash
# Start Gazebo with default world
gz-default

# Or manually:
gz sim ~/px4_workspace/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

# Or with GUI controls:
gz sim -r ~/px4_workspace/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
```

**2. Start PX4 SITL (Terminal 2):**
```bash
# Start PX4 with x500 quadcopter
px4-sitl

# Or manually:
cd ~/px4_workspace/PX4-Autopilot
make px4_sitl gz_x500

# Different vehicle models:
make px4_sitl gz_x500_depth    # Quadcopter with depth camera
make px4_sitl gz_rc_cessna     # Fixed-wing plane
make px4_sitl gz_standard_vtol # VTOL aircraft
```

**3. Start micro-ROS Agent (Terminal 3):**
```bash
# Start micro-ROS agent for ROS 2 communication
microros

# Or manually:
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

**4. Monitor ROS 2 Topics (Terminal 4):**
```bash
# List PX4 topics
px4-topics

# Echo vehicle status
px4-status

# Or manually:
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_status
ros2 topic echo /fmu/out/vehicle_local_position
```

### Stopping the System

Press `Ctrl+C` in each terminal in reverse order:
1. Stop micro-ROS agent (Terminal 3)
2. Stop PX4 (Terminal 2) - type `shutdown` in PX4 console
3. Stop Gazebo (Terminal 1)

## Vehicle Models

### Available Models

| Model | Command | Description |
|-------|---------|-------------|
| x500 | `make px4_sitl gz_x500` | Standard quadcopter |
| x500_depth | `make px4_sitl gz_x500_depth` | Quad with depth camera |
| x500_lidar | `make px4_sitl gz_x500_lidar` | Quad with 2D lidar |
| rc_cessna | `make px4_sitl gz_rc_cessna` | Fixed-wing plane |
| standard_vtol | `make px4_sitl gz_standard_vtol` | VTOL aircraft |

### Change Vehicle Model

```bash
cd ~/px4_workspace/PX4-Autopilot

# Stop current PX4 (Ctrl+C)
# Start with different model:
make px4_sitl gz_rc_cessna
```

## Gazebo Worlds

### Available Worlds

Located at: `~/px4_workspace/PX4-Autopilot/Tools/simulation/gz/worlds/`

- `default.sdf` - Empty world with ground plane
- `baylands.sdf` - Outdoor environment with terrain
- `warehouse.sdf` - Indoor warehouse environment
- `windy.sdf` - World with wind simulation

### Load Different World

```bash
# In Gazebo terminal:
gz sim ~/px4_workspace/PX4-Autopilot/Tools/simulation/gz/worlds/baylands.sdf
```

## ROS 2 Integration

### PX4 Topics in ROS 2

After starting micro-ROS agent, PX4 publishes to `/fmu/out/*` and subscribes to `/fmu/in/*`:

**Telemetry (Published by PX4):**
- `/fmu/out/vehicle_status` - Vehicle state, arming, flight mode
- `/fmu/out/vehicle_local_position` - Local position (NED frame)
- `/fmu/out/vehicle_global_position` - GPS position
- `/fmu/out/vehicle_attitude` - Orientation (quaternion)
- `/fmu/out/sensor_combined` - IMU data
- `/fmu/out/battery_status` - Battery info

**Commands (Received by PX4):**
- `/fmu/in/vehicle_command` - Vehicle commands
- `/fmu/in/offboard_control_mode` - Offboard control mode
- `/fmu/in/trajectory_setpoint` - Position/velocity setpoints
- `/fmu/in/vehicle_rates_setpoint` - Rate setpoints

### Example: Read Vehicle Position

```bash
# Terminal with ROS 2 sourced:
ros2 topic echo /fmu/out/vehicle_local_position
```

### Example: Send Command to PX4

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand

class PX4Commander(Node):
    def __init__(self):
        super().__init__('px4_commander')
        self.publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # 1 = arm, 0 = disarm
        self.publisher.publish(msg)
        self.get_logger().info('Arm command sent')

def main():
    rclpy.init()
    commander = PX4Commander()
    commander.arm()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## QGroundControl Connection

### Local Connection (Same Machine)

QGroundControl auto-discovers PX4 on UDP port 14550.

1. Install QGroundControl
2. Start PX4
3. Open QGC - vehicle appears automatically

### Remote Connection (Windows to Ubuntu)

**Method 1: TCP (Recommended)**

1. Get Ubuntu IP:
   ```bash
   ip addr show | grep inet
   # Example: 192.168.1.100
   ```

2. In QGroundControl:
   - Go to: Application Settings → Comm Links
   - Click "Add"
   - Name: `PX4 Ubuntu`
   - Type: **TCP**
   - Server Address: `192.168.1.100`
   - Port: `5760`
   - Click "OK" and "Connect"

**Method 2: UDP**

- Type: **UDP**
- Port: `14550`
- Server Address: `192.168.1.100`

### Firewall Configuration

If connection fails, open ports:

```bash
sudo ufw allow 14550/udp  # MAVLink UDP
sudo ufw allow 5760/tcp   # MAVLink TCP
```

## PX4 Console Commands

When PX4 is running, you can type commands:

### Common Commands

| Command | Description |
|---------|-------------|
| `commander status` | Show vehicle status |
| `commander arm` | Arm the vehicle |
| `commander disarm` | Disarm the vehicle |
| `commander takeoff` | Takeoff (if armed) |
| `commander land` | Land |
| `listener sensor_combined` | Listen to sensor data |
| `listener vehicle_local_position` | Listen to position |
| `mavlink status` | Show MAVLink connections |
| `ulog_status` | Show logging status |
| `shutdown` | Shutdown PX4 cleanly |

### Example Session

```
pxh> commander status
INFO  [commander] status:
  home: 47.397742, 8.545594, 488.00
  armed: yes
  ...

pxh> listener vehicle_local_position
TOPIC: vehicle_local_position
  timestamp: 12345678
  x: 0.00
  y: 0.00
  z: -1.50
  ...

pxh> commander land
```

## Debugging

### Check Gazebo Topics

```bash
gz topic -l                 # List all topics
gz topic -e -t /clock       # Echo clock topic
gz model -l                 # List spawned models
```

### Check PX4 Logs

```bash
cd ~/px4_workspace/PX4-Autopilot
ls -lt build/px4_sitl_default/logs/  # Latest logs
```

### Monitor System Resources

```bash
# CPU/RAM usage
htop

# GPU usage (if NVIDIA)
nvidia-smi
```

### Network Connections

```bash
# Check MAVLink ports
netstat -tulpn | grep -E "14550|14540|5760|8888"
```

## Advanced Usage

### Multiple Vehicles

Run multiple PX4 instances:

**Vehicle 1 (Terminal pair 1):**
```bash
# Terminal 1A:
gz sim default.sdf

# Terminal 1B:
cd ~/px4_workspace/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_INSTANCE=0 make px4_sitl gz_x500
```

**Vehicle 2 (Terminal pair 2):**
```bash
# Terminal 2 (PX4 only, shares Gazebo):
cd ~/px4_workspace/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_INSTANCE=1 make px4_sitl gz_x500
```

Ports for instance 1:
- MAVLink: 14551 (UDP), 14541 (offboard)
- uXRCE-DDS: 8889

### Custom ROS 2 Workspace

```bash
mkdir -p ~/px4_workspace/ros2_ws/src
cd ~/px4_workspace/ros2_ws
colcon build
source install/setup.bash
```

### Recording Data

**ROS 2 bag:**
```bash
ros2 bag record /fmu/out/vehicle_status /fmu/out/vehicle_local_position
```

**PX4 ulog:**
PX4 automatically logs to `build/px4_sitl_default/logs/`

## Performance Tips

1. **Close unused applications** - Free RAM for simulation
2. **Use dedicated GPU** - Better Gazebo performance
3. **Reduce sensor rates** - If simulation is slow
4. **Lower Gazebo physics rate** - Edit world SDF file

## Next Steps

- Try example missions in `docs/EXAMPLES.md`
- Write custom ROS 2 nodes to control PX4
- Develop autonomous flight algorithms
- Test computer vision with camera feeds
