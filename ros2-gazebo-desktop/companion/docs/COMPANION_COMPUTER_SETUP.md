# Companion Computer Setup for Drone

Minimal micro-ROS agent installation for companion computer on drone.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    DRONE (In Flight)                        │
│                                                             │
│  ┌──────────────────┐        ┌────────────────────────┐   │
│  │ Flight Controller│◄──────►│ Companion Computer     │   │
│  │   (ArduPilot)    │  USB/  │ (Raspberry Pi/Jetson)  │   │
│  │                  │ Serial │                        │   │
│  │  DDS_ENABLE=1    │        │  micro-ROS Agent       │   │
│  │  SERIAL_PROTOCOL │        │  (Minimal Install)     │   │
│  │  = 45 (DDS)      │        │                        │   │
│  └──────────────────┘        │  Port: /dev/ttyUSB0    │   │
│                              │  Baud: 921600          │   │
│                              └───────────┬────────────┘   │
│                                          │                │
└──────────────────────────────────────────┼────────────────┘
                                           │
                                  Radio Link (WiFi/LTE/Telemetry)
                                           │
┌──────────────────────────────────────────┼────────────────┐
│                  GROUND STATION          │                │
│                                          │                │
│  ┌───────────────────────────────────────▼─────────────┐ │
│  │  ROS 2 Desktop (Full Installation)                  │ │
│  │                                                      │ │
│  │  - Gazebo (simulation)                              │ │
│  │  - RViz2 (visualization)                            │ │
│  │  - RQt (monitoring)                                 │ │
│  │  - Custom ROS 2 nodes                               │ │
│  │  - Data logging                                     │ │
│  │  - Mission planning                                 │ │
│  └──────────────────────────────────────────────────────┘ │
│                                                           │
└───────────────────────────────────────────────────────────┘
```

---

## System Comparison

### Companion Computer (Drone) - MINIMAL

**What You NEED:**
- ✅ ROS 2 Base (minimal, ~200 MB)
- ✅ micro-ROS agent (binary only)
- ✅ Serial port access (/dev/ttyUSB0 or similar)
- ✅ Network connectivity (for radio link)
- ✅ Lightweight OS (Ubuntu Server, Raspberry Pi OS Lite)

**What You DON'T NEED:**
- ❌ ROS Desktop packages (RViz, RQt)
- ❌ Gazebo simulator
- ❌ VNC/X11/Desktop environment
- ❌ GUI applications (Firefox, gedit, etc.)
- ❌ Development tools (unless debugging)

**Resources:**
- Disk: ~500 MB (vs 5+ GB for desktop)
- RAM: ~200 MB (vs 2+ GB for desktop)
- CPU: Minimal overhead
- Boot time: Faster

---

### Ground Station - FULL DESKTOP

**What You NEED:**
- ✅ ROS 2 Desktop Full
- ✅ Gazebo (for simulation/replay)
- ✅ RViz2 (3D visualization)
- ✅ RQt (monitoring tools)
- ✅ Data logging & analysis tools
- ✅ Mission planning software
- ✅ VNC/Desktop (for remote access if needed)

**This is the image we built!** Use the full `ros2-gazebo-desktop` setup.

---

## Installation Options for Companion Computer

### Option 1: Docker (Minimal Image) - RECOMMENDED

Best for portability, easy updates, and consistent environment.

**Create minimal Dockerfile:**

```dockerfile
# Minimal ROS 2 Base + micro-ROS agent for companion computer
FROM ros:jazzy-ros-base-noble

# Install only essential packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # micro-ROS dependencies
    ros-jazzy-micro-ros-msgs \
    ros-jazzy-micro-ros-diagnostic-msgs \
    ros-jazzy-micro-ros-diagnostic-bridge \
    # Network tools
    net-tools \
    iputils-ping \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /opt/micro_ros_agent_ws/src

# Clone and build micro-ROS agent
RUN cd /opt/micro_ros_agent_ws/src \
    && git clone https://github.com/micro-ROS/micro-ROS-Agent.git \
    && cd micro-ROS-Agent \
    && git checkout jazzy

# Build micro-ROS agent
RUN cd /opt/micro_ros_agent_ws \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /opt/micro_ros_agent_ws/install/setup.bash" >> /root/.bashrc

# Entrypoint
CMD ["/bin/bash"]
```

**docker-compose.yml for companion:**

```yaml
version: '3.8'

services:
  micro-ros-agent:
    build:
      context: .
      dockerfile: Dockerfile.companion
    image: micro-ros-agent:minimal
    container_name: micro_ros_agent_companion

    # Host network for radio link communication
    network_mode: host

    # Serial device access (flight controller)
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
      # Or: - /dev/ttyACM0:/dev/ttyACM0

    privileged: true  # For serial port access

    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp

    # Auto-start micro-ROS agent
    command: >
      bash -c "source /opt/ros/jazzy/setup.bash &&
               source /opt/micro_ros_agent_ws/install/setup.bash &&
               ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600"

    restart: unless-stopped
```

**Image size comparison:**
- Full desktop image: ~5.5 GB
- Minimal companion: ~800 MB
- **Savings: 85% smaller!**

---

### Option 2: Native Installation (No Docker)

For maximum performance on resource-constrained hardware.

**Install on Ubuntu Server 24.04 (Noble):**

```bash
# 1. Add ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Install ROS 2 Base (minimal)
sudo apt update
sudo apt install -y ros-jazzy-ros-base

# 3. Install micro-ROS dependencies
sudo apt install -y \
    ros-jazzy-micro-ros-msgs \
    ros-jazzy-micro-ros-diagnostic-msgs \
    ros-jazzy-micro-ros-diagnostic-bridge

# 4. Create workspace and build micro-ROS agent
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent.git
cd micro-ROS-Agent
git checkout jazzy

cd ~/micro_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. Setup environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/micro_ros_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 6. Test
ros2 run micro_ros_agent micro_ros_agent --help
```

**Installed size: ~600 MB** (vs 5+ GB for desktop)

---

### Option 3: Binary-Only Installation (Lightest)

For extremely resource-constrained devices.

```bash
# Build micro-ROS agent on development machine
# Copy only the binary to companion computer

# On development machine:
cd ~/micro_ros_ws
colcon build --packages-select micro_ros_agent
tar -czf micro_ros_agent.tar.gz install/

# On companion computer:
# 1. Install only ROS 2 runtime libraries
sudo apt install -y ros-jazzy-ros-base-dev

# 2. Copy and extract binary
scp user@dev-machine:~/micro_ros_ws/micro_ros_agent.tar.gz .
tar -xzf micro_ros_agent.tar.gz

# 3. Run directly
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
```

**Installed size: ~400 MB**

---

## Serial Connection Configuration

### Flight Controller to Companion Computer

**Hardware:**
- USB cable: Flight controller USB → Companion computer USB
- Or UART: Flight controller TELEM port → Companion GPIO UART

**ArduPilot Parameters:**
```
SERIAL3_PROTOCOL = 45      # DDS (micro-ROS)
SERIAL3_BAUD = 921600      # High speed
```

**Find Serial Device:**
```bash
# List serial devices
ls -l /dev/tty*

# Common devices:
# /dev/ttyUSB0  - USB serial adapter
# /dev/ttyACM0  - USB CDC (most ArduPilot)
# /dev/ttyAMA0  - Raspberry Pi UART

# Check which one appears when you plug in
dmesg | grep tty
# Should show: "usb 1-1: FTDI USB Serial Device converter now attached to ttyUSB0"
```

**Permissions:**
```bash
# Add user to dialout group (for serial access)
sudo usermod -aG dialout $USER

# Or set permissions directly
sudo chmod 666 /dev/ttyUSB0

# Make persistent (udev rule)
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-serial.rules
sudo udevadm control --reload-rules
```

---

## Running micro-ROS Agent on Companion

### Start Agent (Serial Connection)

```bash
# Connect to flight controller via serial
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600

# Verbose mode (debugging)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6
```

### Auto-Start on Boot (systemd)

Create service file:

```bash
sudo nano /etc/systemd/system/micro-ros-agent.service
```

```ini
[Unit]
Description=micro-ROS Agent for ArduPilot
After=network.target

[Service]
Type=simple
User=ubuntu  # Or your username
WorkingDirectory=/home/ubuntu
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source /home/ubuntu/micro_ros_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable micro-ros-agent.service
sudo systemctl start micro-ros-agent.service

# Check status
sudo systemctl status micro-ros-agent.service

# View logs
sudo journalctl -u micro-ros-agent.service -f
```

---

## Network Configuration (Drone ↔ Ground)

### For Radio Link Communication

**Both systems must:**
1. Be on same network (even if via radio/WiFi link)
2. Use same `ROS_DOMAIN_ID`
3. Allow multicast UDP traffic
4. Have reachable IP addresses

**Example Network Setup:**

**Companion Computer (Drone):**
```bash
# WiFi or LTE connection
# IP: 192.168.1.100 (example)

# Set ROS domain
export ROS_DOMAIN_ID=0
```

**Ground Station:**
```bash
# Same network
# IP: 192.168.1.50 (example)

# Same domain
export ROS_DOMAIN_ID=0
```

**Verify connectivity:**
```bash
# From ground station, ping companion
ping 192.168.1.100

# Check ROS discovery
ros2 topic list
# Should see /fmu/* topics from drone
```

### Fast DDS Discovery Configuration

For unreliable radio links, configure discovery server mode:

**On Companion (Discovery Server):**
```bash
# Run agent as discovery server
fastdds discovery -i 0 -l 192.168.1.100 -p 11811
```

**On Ground Station (Discovery Client):**
```bash
# Point to companion's discovery server
export ROS_DISCOVERY_SERVER=192.168.1.100:11811
```

---

## Resource Optimization

### Minimal Companion Computer Specs

**Minimum:**
- CPU: Raspberry Pi 4 (2GB RAM)
- Storage: 8 GB SD card
- Network: WiFi or LTE module

**Recommended:**
- CPU: Raspberry Pi 4 (4GB) or NVIDIA Jetson Nano
- Storage: 16+ GB SD card or eMMC
- Network: 5GHz WiFi or LTE Cat-4+

### Performance Tips

**1. Use ROS Base (not Desktop):**
```bash
# Install only:
ros-jazzy-ros-base  # ~200 MB

# NOT:
ros-jazzy-desktop-full  # ~5 GB
```

**2. Disable unnecessary services:**
```bash
sudo systemctl disable bluetooth
sudo systemctl disable avahi-daemon
```

**3. Use lightweight OS:**
- Ubuntu Server 24.04 (no GUI)
- Raspberry Pi OS Lite
- Yocto Linux (advanced)

**4. Optimize boot time:**
```bash
# Disable splash screen
sudo nano /boot/cmdline.txt
# Add: quiet splash=0

# Reduce DHCP timeout
sudo nano /etc/systemd/system/dhcpcd.service.d/wait.conf
# Add: TimeoutStartSec=10sec
```

---

## Comparison Table

| Feature | Ground Station | Companion Computer |
|---------|---------------|-------------------|
| **ROS Install** | Desktop Full | Base (minimal) |
| **micro-ROS** | Optional | Required |
| **Gazebo** | Yes | No |
| **RViz2** | Yes | No |
| **Desktop/GUI** | Yes (VNC) | No |
| **Connection** | Radio/Network | Serial + Network |
| **Disk Usage** | 5-10 GB | 0.5-1 GB |
| **RAM Usage** | 2-4 GB | 200-500 MB |
| **Purpose** | Monitoring/Control | Data Bridge |

---

## Recommended Setup

### For Your Use Case:

**Companion Computer (On Drone):**
```bash
# Option 1: Docker (easier updates)
# Use minimal Dockerfile above
docker compose up -d

# Option 2: Native (better performance)
# Install ROS Base + micro-ROS agent
# Auto-start with systemd
```

**Ground Station:**
```bash
# Use the full ros2-gazebo-desktop image we built!
cd ros2-gazebo-desktop
docker compose up -d

# Access via VNC
# http://ground-station-ip:6080/vnc.html
```

### Data Flow:

```
Flight Controller → Serial → Companion Computer → Radio Link → Ground Station
   (ArduPilot)     USB/UART   (micro-ROS agent)   WiFi/LTE    (ROS 2 Desktop)
                                                               - RViz2
                                                               - Gazebo
                                                               - Logging
                                                               - Analysis
```

---

## Next Steps

1. **Choose installation method** for companion computer:
   - Docker (recommended for ease)
   - Native (for performance)
   - Binary-only (for minimal size)

2. **Configure serial connection:**
   - Identify device (/dev/ttyUSB0, etc.)
   - Set permissions
   - Test connection

3. **Setup auto-start:**
   - systemd service
   - Docker restart policy

4. **Configure radio link:**
   - Network setup
   - ROS domain matching
   - Discovery configuration

5. **Test end-to-end:**
   - Start agent on companion
   - Connect to ground station
   - Verify topics appear

---

## Files to Create

I can create these for you:

1. `Dockerfile.companion` - Minimal image for companion computer
2. `docker-compose.companion.yml` - Compose file with serial access
3. `systemd/micro-ros-agent.service` - Auto-start service
4. `scripts/start_agent.sh` - Helper script for starting agent

Would you like me to create these files?
