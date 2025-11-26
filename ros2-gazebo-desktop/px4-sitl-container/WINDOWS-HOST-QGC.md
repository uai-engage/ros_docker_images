# QGroundControl on Windows Host - px4-sitl-container Setup

## Quick Start

The `px4-sitl-container` now **automatically configures MAVLink** to target your Windows host IP!

### Setup (One-Time)

**1. Copy .env file:**
```bash
cd px4-sitl-container
cp .env.example .env
```

**2. (Optional) Set your Windows IP in `.env`:**
```bash
# Find your Windows IP:
# Windows PowerShell: ipconfig

# Edit .env and set:
WINDOWS_HOST_IP=192.168.18.93  # Your Windows IP here
```

**Or leave it empty - it will auto-detect!**

### Usage

**1. Start the PX4 container:**
```bash
docker compose up -d
```

**2. Check the logs:**
```bash
docker compose logs -f
```

You should see:
```
Auto-detected Windows host IP: 192.168.18.93
Modified rcS to add MAVLink configurations
  - UDP 14550 → 192.168.18.93 (QGroundControl)
  - TCP 5760  → 0.0.0.0 (Remote QGC)
```

**3. Start QGroundControl on Windows:**
- Launch QGroundControl
- It will auto-connect via UDP 14550!

### How It Works

The startup script:
1. ✅ Auto-detects Windows host IP (Docker gateway)
2. ✅ Modifies PX4's startup file (rcS) to add:
   ```bash
   mavlink start -x -u 14550 -t <your_windows_ip> -r 4000000
   ```
3. ✅ This runs automatically when PX4 starts
4. ✅ No manual configuration needed!

### Configuration Options

#### Option 1: Auto-Detection (Default)
Leave `WINDOWS_HOST_IP` empty in `.env`:
```bash
WINDOWS_HOST_IP=
```

The script will use `ip route | grep default` to find the Docker gateway (your Windows host).

#### Option 2: Manual IP
Set your Windows IP in `.env`:
```bash
WINDOWS_HOST_IP=192.168.18.93
```

#### Option 3: Override at Runtime
```bash
WINDOWS_HOST_IP=192.168.1.100 docker compose up -d
```

### Troubleshooting

**QGroundControl doesn't connect:**

1. **Check detected IP in logs:**
   ```bash
   docker compose logs | grep "Windows host IP"
   ```

2. **Verify Windows firewall allows UDP 14550:**
   - Windows Firewall → Advanced Settings
   - Inbound Rules → New Rule
   - Port: UDP 14550
   - Allow connection

3. **Test connectivity:**
   ```bash
   # From inside container:
   docker exec -it px4_sitl bash
   ping <your_windows_ip>
   ```

4. **Check QGC connection settings:**
   - Q icon → Application Settings → Comm Links
   - Should have UDP connection on port 14550
   - Add manually if needed

**Wrong IP detected:**

Set it manually in `.env`:
```bash
WINDOWS_HOST_IP=192.168.18.93
docker compose restart
```

### Multi-Container Setup (with ROS 2)

**Terminal 1 - ROS 2 + Gazebo:**
```bash
cd ros2-gazebo-desktop
docker compose up -d
# Access GUI and start Gazebo:
gz sim -r default.sdf
```

**Terminal 2 - PX4 SITL:**
```bash
cd px4-sitl-container
docker compose up -d
# MAVLink automatically configured!
```

**Terminal 3 - micro-ROS Agent (in ROS 2 container):**
```bash
docker exec -it ros2_gazebo bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

**Windows Host - QGroundControl:**
- Launch QGC
- Auto-connects to PX4!

### Verification

**Check MAVLink status** (inside PX4 container):
```bash
docker exec -it px4_sitl bash
# At PX4 pxh> prompt:
mavlink status
```

Should show:
```
instance #0:
    partner IP: 192.168.18.93
    transport protocol: UDP (14550, remote port: 14550)
```

## Summary

✅ **Automatic MAVLink configuration** - No manual commands needed
✅ **Auto-detects Windows IP** - Uses Docker gateway
✅ **Works on container restart** - Configuration persists
✅ **Override anytime** - Set WINDOWS_HOST_IP in .env

**Just start the container and launch QGC on Windows - it works!**
