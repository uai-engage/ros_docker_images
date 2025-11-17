# ROS 2 Gazebo Desktop with VNC - Setup Guide

This guide explains how to run ROS 2 Jazzy with Gazebo Harmonic using VNC for remote access. VNC provides **much better performance** than X11 forwarding, especially for 3D applications like Gazebo.

## Why Use VNC Instead of X11?

| Feature | X11 Forwarding | VNC |
|---------|----------------|-----|
| Performance | Slow (raw pixels over network) | Fast (compressed images) |
| 3D Graphics | Very laggy | Smooth |
| Network Usage | High | Low (compressed) |
| Setup | Requires X server on Windows | Just a web browser |
| Security | Less secure | Password protected |

---

## Quick Start

### 1. Configure Environment

```bash
cd ros2-gazebo-desktop

# Copy and customize VNC configuration
cp .env.vnc .env

# Edit settings (optional)
nano .env
```

### 2. Build the VNC Image

```bash
docker compose -f docker-compose-vnc.yml build
```

This may take 10-15 minutes on first build.

### 3. Start the Container

```bash
docker compose -f docker-compose-vnc.yml up -d
```

### 4. Connect to VNC

**Option A: Web Browser (Easiest)**
```
http://<your-server-ip>:6080/vnc.html
```
- Click "Connect"
- Enter password: `rospassword` (or your custom password)

**Option B: VNC Client**
- Download: TightVNC, RealVNC, or TigerVNC Viewer
- Connect to: `<your-server-ip>:5901`
- Password: `rospassword`

### 5. Use ROS 2 and Gazebo

Inside the VNC desktop:
1. Right-click → Terminal (or use existing terminal)
2. Run:
```bash
source /opt/ros/jazzy/setup.bash
gz sim shapes.sdf
```

---

## Configuration

### Environment Variables (`.env.vnc`)

| Variable | Description | Default | Example |
|----------|-------------|---------|---------|
| `VNC_PASSWORD` | VNC connection password | `rospassword` | `mySecurePass123` |
| `VNC_RESOLUTION` | Screen resolution | `1920x1080` | `1680x1050` |
| `VNC_DEPTH` | Color depth (16, 24, 32) | `24` | `24` |
| `USERNAME` | Container username | `rosuser` | `developer` |
| `USER_UID` | User ID | `1000` | `1000` |
| `USER_GID` | Group ID | `1000` | `1000` |
| `ROS_DOMAIN_ID` | ROS 2 domain isolation | `0` | `42` |
| `RMW_IMPLEMENTATION` | DDS implementation | `rmw_fastrtps_cpp` | `rmw_cyclonedds_cpp` |
| `HOST_WORKSPACE` | Workspace mount path | `./ros2_ws` | `/home/user/my_ws` |
| `CONTAINER_NAME` | Docker container name | `ros2_gazebo_vnc` | `my_ros_vnc` |

### Recommended Resolutions

| Resolution | Use Case |
|------------|----------|
| `1920x1080` | Full HD, large displays |
| `1680x1050` | Balanced size |
| `1440x900` | Medium displays |
| `1280x720` | Lower bandwidth/smaller displays |
| `2560x1440` | High DPI / 4K monitors |

---

## Docker Compose Configuration

### `docker-compose-vnc.yml` Structure

```yaml
services:
  ros2-gazebo-vnc:
    build:
      context: .
      dockerfile: Dockerfile.vnc        # VNC-specific Dockerfile

    environment:
      - VNC_PASSWORD=${VNC_PASSWORD}     # From .env file
      - VNC_RESOLUTION=${VNC_RESOLUTION}
      - RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
      # ... other env vars

    ports:
      - "5901:5901"    # VNC server port
      - "6080:6080"    # noVNC web interface

    volumes:
      - ${HOST_WORKSPACE}:/home/rosuser/ros2_ws:rw
      - /dev/shm:/dev/shm

    command: /startup-vnc.sh   # Starts VNC server
```

### Key Components

1. **Dockerfile.vnc**: Builds image with VNC server and lightweight desktop
2. **startup-vnc.sh**: Initializes VNC server and noVNC proxy
3. **Openbox**: Lightweight window manager (fast, minimal overhead)
4. **noVNC**: Web-based VNC client (no client software needed)

---

## Detailed Setup Steps

### Step 1: Prepare Workspace

```bash
# On your Ubuntu server
cd /path/to/ros_docker_images/ros2-gazebo-desktop

# Create ROS 2 workspace directory
mkdir -p ros2_ws/src

# Copy environment template
cp .env.vnc .env
```

### Step 2: Customize Configuration

Edit `.env` file:

```bash
# Change VNC password (IMPORTANT for security!)
VNC_PASSWORD=YourSecurePassword123

# Adjust resolution if needed
VNC_RESOLUTION=1920x1080

# Set ROS 2 domain (optional, for multi-robot setups)
ROS_DOMAIN_ID=0
```

### Step 3: Build Docker Image

```bash
# Build with no cache (clean build)
docker compose -f docker-compose-vnc.yml build --no-cache

# Or quick rebuild
docker compose -f docker-compose-vnc.yml build
```

**Build includes:**
- ROS 2 Jazzy Desktop Full
- Gazebo Harmonic
- TigerVNC Server
- noVNC (web interface)
- Openbox window manager
- Development tools

### Step 4: Start Container

```bash
# Start in background
docker compose -f docker-compose-vnc.yml up -d

# View logs
docker compose -f docker-compose-vnc.yml logs -f

# You should see:
# ============================================
# VNC Server started!
# Connect via:
#   - VNC Client: <server-ip>:5901
#   - Web Browser: http://<server-ip>:6080/vnc.html
# ============================================
```

### Step 5: Connect from Windows

#### Option A: Web Browser (No Software Required)

1. Open your web browser (Chrome, Firefox, Edge)
2. Navigate to: `http://<ubuntu-server-ip>:6080/vnc.html`
3. Click "Connect"
4. Enter password when prompted
5. You'll see the Openbox desktop

#### Option B: VNC Client (Better Performance)

1. Download a VNC client:
   - [TightVNC Viewer](https://www.tightvnc.com/download.php) (Free)
   - [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/) (Free)
   - [TigerVNC Viewer](https://tigervnc.org/)

2. Connect:
   - Server: `<ubuntu-server-ip>:5901`
   - Or: `<ubuntu-server-ip>::5901`
   - Password: Your configured password

3. Adjust client settings:
   - Quality: High/Ultra
   - Encoding: Tight or ZRLE
   - Compression: 6-9

---

## Using ROS 2 and Gazebo in VNC

### Opening Applications

**Right-click on desktop** to see menu:
- **Terminal**: Open xfce4-terminal
- **File Manager**: PCManFM file browser
- **Window Options**: Move, resize, close

### Running Gazebo

```bash
# Open terminal (right-click → Terminal)
# Or use existing terminal window

# Source ROS 2 (already in .bashrc)
source /opt/ros/jazzy/setup.bash

# Launch Gazebo with example world
gz sim shapes.sdf

# Launch with specific world
gz sim /opt/ros/jazzy/share/ros_gz_sim/worlds/empty.sdf

# Launch via ROS 2
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"
```

### Running RViz2

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2
```

### Running RQt

```bash
source /opt/ros/jazzy/setup.bash
rqt
```

### TurtleSim Example

```bash
# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
ros2 run turtlesim turtle_teleop_key
```

### Multiple Terminals

Right-click → Terminal to open new windows, or use:
```bash
xfce4-terminal --tab  # New tab
xfce4-terminal &      # New window
```

---

## Advanced Configuration

### Custom VNC Password

```bash
# In .env file
VNC_PASSWORD=MySecurePassword123!

# Restart container to apply
docker compose -f docker-compose-vnc.yml down
docker compose -f docker-compose-vnc.yml up -d
```

### Change Resolution on the Fly

```bash
# Inside container
docker exec -it ros2_gazebo_vnc bash

# Change resolution
xrandr --size 1680x1050

# Or restart with new resolution
# Edit .env, then:
docker compose -f docker-compose-vnc.yml down
docker compose -f docker-compose-vnc.yml up -d
```

### Custom Port Mapping

Edit `docker-compose-vnc.yml`:

```yaml
ports:
  - "5902:5901"    # VNC on 5902
  - "8080:6080"    # noVNC on 8080
```

Then connect to:
- VNC: `server:5902`
- Web: `http://server:8080/vnc.html`

### Persist VNC Configuration

```bash
# Mount VNC config directory
volumes:
  - ./vnc_config:/home/rosuser/.vnc
```

### GPU Support (NVIDIA)

If your server has an NVIDIA GPU:

1. Install nvidia-container-toolkit
2. Modify `docker-compose-vnc.yml`:

```yaml
environment:
  - NVIDIA_VISIBLE_DEVICES=all
  - NVIDIA_DRIVER_CAPABILITIES=graphics
  # Remove: LIBGL_ALWAYS_SOFTWARE=1

deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [gpu]
```

---

## Container Management

### Start/Stop Commands

```bash
# Start
docker compose -f docker-compose-vnc.yml up -d

# Stop
docker compose -f docker-compose-vnc.yml down

# Restart
docker compose -f docker-compose-vnc.yml restart

# View status
docker compose -f docker-compose-vnc.yml ps

# View logs
docker compose -f docker-compose-vnc.yml logs -f
```

### Access Container Shell

```bash
# Interactive bash session
docker exec -it ros2_gazebo_vnc bash

# Run single command
docker exec ros2_gazebo_vnc ros2 topic list
```

### Rebuild After Changes

```bash
# Rebuild and restart
docker compose -f docker-compose-vnc.yml build
docker compose -f docker-compose-vnc.yml up -d --force-recreate
```

### Clean Up

```bash
# Remove container and network
docker compose -f docker-compose-vnc.yml down

# Remove image too
docker compose -f docker-compose-vnc.yml down --rmi all

# Remove volumes
docker compose -f docker-compose-vnc.yml down -v

# Full cleanup
docker system prune -a
```

---

## Troubleshooting

### Cannot Connect to VNC

1. **Check container is running:**
   ```bash
   docker compose -f docker-compose-vnc.yml ps
   ```

2. **Check ports are open:**
   ```bash
   netstat -tlnp | grep -E "5901|6080"
   ```

3. **Check firewall:**
   ```bash
   sudo ufw allow 5901
   sudo ufw allow 6080
   ```

4. **Check logs:**
   ```bash
   docker compose -f docker-compose-vnc.yml logs
   ```

### Black Screen in VNC

1. **Check window manager:**
   ```bash
   docker exec ros2_gazebo_vnc pgrep openbox
   ```

2. **Restart VNC server:**
   ```bash
   docker exec ros2_gazebo_vnc vncserver -kill :1
   docker exec ros2_gazebo_vnc /startup-vnc.sh
   ```

### Gazebo Not Starting

1. **Check memory:**
   ```bash
   docker stats ros2_gazebo_vnc
   ```
   Increase `MEMORY_LIMIT` in `.env` if needed.

2. **Check shared memory:**
   ```bash
   df -h /dev/shm
   ```
   Should have at least 2GB.

3. **Try software rendering:**
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gz sim shapes.sdf
   ```

### Slow Performance

1. **Reduce resolution:**
   ```bash
   VNC_RESOLUTION=1280x720
   ```

2. **Lower color depth:**
   ```bash
   VNC_DEPTH=16
   ```

3. **Use VNC client** instead of browser (better compression)

4. **Adjust VNC client settings:**
   - Lower quality
   - Enable compression
   - Use Tight encoding

### Password Issues

```bash
# Reset password
docker exec -it ros2_gazebo_vnc bash
echo "newpassword" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# Restart VNC
vncserver -kill :1
/startup-vnc.sh
```

---

## Security Recommendations

### 1. Change Default Password

**Always change the default password!**

```bash
# In .env
VNC_PASSWORD=YourStrongPassword123!
```

### 2. Use SSH Tunnel (Recommended)

Instead of exposing ports directly:

```bash
# From Windows (PowerShell)
ssh -L 5901:localhost:5901 -L 6080:localhost:6080 user@ubuntu-server

# Then connect to localhost
# VNC: localhost:5901
# Web: http://localhost:6080/vnc.html
```

### 3. Firewall Rules

Only allow specific IPs:

```bash
# On Ubuntu server
sudo ufw default deny incoming
sudo ufw allow from <your-windows-ip> to any port 5901
sudo ufw allow from <your-windows-ip> to any port 6080
```

### 4. Use VPN

For remote access over internet, use VPN rather than direct port exposure.

---

## ROS 2 Communication Setup

### Fast DDS (Default)

Already configured in `.env`:
```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Multi-Container Communication

If running multiple ROS 2 containers:

```yaml
# Same ROS_DOMAIN_ID
environment:
  - ROS_DOMAIN_ID=42

# Use bridge network with shared network
networks:
  ros_network:
    driver: bridge
```

### External ROS 2 Nodes

To communicate with ROS 2 nodes outside container:

1. Use `network_mode: host` (loses port mapping)
2. Or configure multicast properly
3. Set same `ROS_DOMAIN_ID`

---

## File Structure

```
ros2-gazebo-desktop/
├── Dockerfile.vnc           # VNC image definition
├── docker-compose-vnc.yml   # VNC compose configuration
├── startup-vnc.sh           # VNC startup script
├── .env.vnc                 # VNC environment template
├── .env                     # Your configuration (copied from template)
├── ros2_ws/                 # Your ROS 2 workspace (mounted)
│   └── src/
│       └── your_packages/
└── VNC_SETUP.md            # This documentation
```

---

## Summary

1. **Copy config**: `cp .env.vnc .env`
2. **Edit password**: Change `VNC_PASSWORD` in `.env`
3. **Build**: `docker compose -f docker-compose-vnc.yml build`
4. **Start**: `docker compose -f docker-compose-vnc.yml up -d`
5. **Connect**: `http://<server>:6080/vnc.html`
6. **Use**: Run Gazebo and ROS 2 tools in VNC desktop

VNC provides a much smoother experience than X11 forwarding, especially for 3D graphics like Gazebo. The web-based noVNC option means you don't need to install any software on Windows - just use your browser!
