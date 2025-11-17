# ROS 2 Jazzy Desktop with Gazebo for Windows

This setup provides a complete ROS 2 Jazzy (Ubuntu 24.04 Noble) development environment with Gazebo Harmonic simulation, configured to display GUI applications on a Windows host.

## Features

- ROS 2 Jazzy (Latest LTS-like release)
- Ubuntu 24.04 Noble base
- Gazebo Harmonic (compatible with ROS 2 Jazzy)
- Full ROS 2 Desktop installation (RViz2, RQt, etc.)
- X11 forwarding for GUI applications
- Non-root user with sudo privileges
- Persistent workspace mounting

## Prerequisites

### On Windows Host

1. **Docker Desktop for Windows** (with WSL2 backend)
   - Download from: https://www.docker.com/products/docker-desktop/
   - Enable WSL2 integration in Docker Desktop settings

2. **X Server for Windows** (Choose one):

   #### Option A: VcXsrv (Free, Recommended)
   1. Download from: https://sourceforge.net/projects/vcxsrv/
   2. Install and launch XLaunch
   3. Configuration:
      - Multiple windows
      - Display number: 0
      - Start no client
      - **Important**: Check "Disable access control"
      - Save configuration for future use

   #### Option B: X410 (Paid, from Microsoft Store)
   1. Install from Microsoft Store
   2. Enable "Allow Public Access" in settings

   #### Option C: WSLg (Windows 11 with WSL2)
   - If using Windows 11 with recent WSL2, GUI support is built-in
   - No additional X server needed

3. **Windows Firewall Configuration**
   - Allow VcXsrv/X410 through Windows Firewall
   - Both private and public networks

## Quick Start

### 1. Clone and Navigate

```powershell
cd ros2-gazebo-desktop
```

### 2. Configure Environment

Edit the `.env` file to match your setup:

```bash
# Get your Windows IP address (run in PowerShell):
ipconfig

# Update .env file with your IP:
DISPLAY=<YOUR_WINDOWS_IP>:0.0
# Example: DISPLAY=192.168.1.100:0.0

# For WSLg (Windows 11):
DISPLAY=:0
```

### 3. Create Local Workspace

```powershell
mkdir ros2_ws\src
```

### 4. Build the Docker Image

```powershell
docker-compose build
```

### 5. Start the Container

```powershell
docker-compose up -d
```

### 6. Connect to Container

```powershell
docker exec -it ros2_gazebo_desktop bash
```

## Usage

### Testing X11 Connection

Inside the container:

```bash
# Test X11 with simple app
xeyes

# Test OpenGL
glxgears
```

### Running Gazebo

```bash
# Launch Gazebo with empty world
gz sim

# Launch Gazebo with a specific world
gz sim shapes.sdf

# Launch via ROS 2
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"
```

### Running RViz2

```bash
ros2 run rviz2 rviz2
```

### Running RQt

```bash
rqt
```

### Example: TurtleSim

```bash
# In terminal 1
ros2 run turtlesim turtlesim_node

# In terminal 2
ros2 run turtlesim turtle_teleop_key
```

## Configuration Details

### Environment Variables (`.env`)

| Variable | Description | Default |
|----------|-------------|---------|
| `DISPLAY` | X11 display for GUI | `host.docker.internal:0.0` |
| `USER_UID` | User ID (match host for permissions) | `1000` |
| `USER_GID` | Group ID | `1000` |
| `USERNAME` | Container username | `rosuser` |
| `ROS_DOMAIN_ID` | ROS 2 domain for isolation | `0` |
| `HOST_WORKSPACE` | Path to workspace on host | `./ros2_ws` |
| `NETWORK_MODE` | Docker network mode | `host` |

### Volume Mounts

- **Workspace**: Your local `ros2_ws` is mounted for persistent development
- **X11 Socket**: Enables GUI forwarding
- **Shared Memory**: `/dev/shm` for better performance

## Troubleshooting

### GUI Not Displaying

1. **Check X Server is running** on Windows
2. **Verify DISPLAY variable**:
   ```bash
   echo $DISPLAY
   ```
3. **Check firewall** allows X server connections
4. **Test connectivity**:
   ```bash
   xdpyinfo
   ```

### Permission Denied Errors

Match `USER_UID` and `USER_GID` in `.env` with your host user:
```bash
# On Linux/WSL:
id -u  # for UID
id -g  # for GID
```

### Gazebo Performance Issues

1. **Enable GPU passthrough** (if NVIDIA):
   - Uncomment the GPU service in `docker-compose.yml`
   - Install `nvidia-container-toolkit`

2. **Increase shared memory**:
   ```yaml
   shm_size: '2gb'
   ```

3. **Use software rendering**:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gz sim
   ```

### ROS 2 Discovery Issues

When using bridge network mode instead of host:
```bash
# Ensure all containers use same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# Check multicast (in container)
ros2 daemon stop
ros2 daemon start
```

## Advanced Usage

### Building ROS 2 Packages

```bash
# Inside container, in workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Multiple Terminals

```powershell
# Open additional terminals
docker exec -it ros2_gazebo_desktop bash
```

Or use `tmux`/`terminator` inside the container:
```bash
terminator
```

### Custom Gazebo Models

1. Place models in `ros2_ws/src/models/`
2. They're automatically available via `GZ_SIM_RESOURCE_PATH`

### Running Specific ROS 2 Commands

```powershell
docker exec ros2_gazebo_desktop ros2 topic list
docker exec ros2_gazebo_desktop ros2 node list
```

## Stopping and Cleanup

```powershell
# Stop container
docker-compose down

# Remove image
docker rmi ros2-gazebo-desktop:jazzy-noble

# Clean up volumes
docker volume prune
```

## Security Notes

- Container runs with non-root user
- X11 forwarding requires disabling access control (use in trusted networks)
- Host network mode exposes all container ports
- Consider using bridge network with specific port mappings for production

## Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [ROS 2 and Gazebo Integration](https://github.com/gazebosim/ros_gz)
- [Docker Desktop for Windows](https://docs.docker.com/desktop/windows/)
- [VcXsrv Tutorial](https://sourceforge.net/p/vcxsrv/wiki/Home/)

## License

This configuration is provided as-is for development purposes.
