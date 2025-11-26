# Using QGroundControl on Windows Host with Docker Container

This guide explains how to run QGroundControl on your Windows host machine while PX4 and Gazebo run inside the Docker container.

## Architecture

```
┌─────────────────────────────────────────┐
│   Windows Host (192.168.x.x)            │
│                                         │
│   ┌─────────────────────────────────┐  │
│   │  QGroundControl                  │  │
│   │  Listening on UDP :14550         │  │
│   └─────────────────────────────────┘  │
│              ▲                          │
│              │ MAVLink UDP              │
│              │                          │
└──────────────┼──────────────────────────┘
               │
┌──────────────┼──────────────────────────┐
│   Docker Container                      │
│              │                          │
│   ┌──────────┴──────────────────┐      │
│   │  PX4 SITL                    │      │
│   │  Sending to host:14550       │      │
│   └──────────────────────────────┘      │
│                                         │
│   ┌─────────────────────────────────┐  │
│   │  Gazebo Harmonic                 │  │
│   └─────────────────────────────────┘  │
│                                         │
│   ┌─────────────────────────────────┐  │
│   │  micro-ROS Agent                 │  │
│   └─────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

## Prerequisites

1. ✅ Docker container running with `network_mode: host` in docker-compose.yml
2. ✅ Windows host can access container network
3. ✅ QGroundControl installed on Windows host
4. ✅ Windows firewall allows UDP port 14550

## Startup Procedure

### Step 1: Find Your Windows Host IP

On your **Windows host**, open PowerShell or CMD:

```powershell
ipconfig
```

Look for your network adapter (WiFi or Ethernet) and note the **IPv4 Address**.

Example: `192.168.18.93`

### Step 2: Start Container Services

Inside the **Docker container VNC desktop**, open 3 terminals:

**Terminal 1 - Start Gazebo:**
```bash
gz-default
```

Wait for Gazebo to fully load (shows empty world).

**Terminal 2 - Start micro-ROS Agent (BEFORE PX4):**
```bash
microros
```

You should see:
```
[info] UDPv4AgentLinux.cpp | init | running... | port: 8888
```

**Terminal 3 - Start PX4 SITL:**
```bash
px4-sitl
```

Wait for PX4 to start. You'll see the `pxh>` prompt.

### Step 3: Configure MAVLink for Windows Host

**At the `pxh>` prompt in Terminal 3:**

```bash
mavlink start -x -u 14550 -t 192.168.18.93 -r 4000000
```

**Replace `192.168.18.93` with YOUR Windows host IP address!**

You should see:
```
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 14550 remote port 14550
INFO  [mavlink] partner IP: 192.168.18.93
```

Verify with:
```bash
mavlink status
```

Look for:
```
instance #0:
    partner IP: 192.168.18.93
    transport protocol: UDP (14550, remote port: 14550)
```

### Step 4: Start QGroundControl on Windows

**On your Windows host:**

1. Start QGroundControl
2. It should **auto-connect** via UDP port 14550
3. You'll see the vehicle appear and parameters download
4. Vehicle status should show "Ready" (after sensor calibration)

## Troubleshooting

### QGroundControl Doesn't Connect

**Check 1: Verify PX4 MAVLink is running**

In PX4 terminal:
```bash
mavlink status
```

Look for correct `partner IP` matching your Windows host.

**Check 2: Windows Firewall**

Open Windows Firewall and allow **UDP port 14550** for QGroundControl.

**Check 3: Network Connectivity**

From inside the Docker container:
```bash
ping 192.168.18.93
```

Should show replies. If not, check Docker network configuration.

**Check 4: QGroundControl Connection Settings**

In QGroundControl:
- **Q icon** → **Application Settings** → **Comm Links**
- Should have UDP connection on port **14550**
- If not, add manually:
  - Type: UDP
  - Listening Port: 14550
  - Click Connect

### "Vehicle Did Not Respond to Parameters" Error

**This means MAVLink is not configured correctly.**

1. Stop PX4 (Ctrl+C in terminal 3)
2. Restart: `px4-sitl`
3. **Immediately** at `pxh>` prompt:
   ```bash
   mavlink start -x -u 14550 -t YOUR_HOST_IP -r 4000000
   ```
4. Restart QGroundControl on Windows

### High Packet Loss

If `mavlink status` shows high packet loss (>10%):

1. Check network congestion
2. Reduce data rate:
   ```bash
   mavlink stop-all
   mavlink start -x -u 14550 -t YOUR_HOST_IP -r 2000000
   ```
3. Ensure micro-ROS was started **before** PX4

## Important Notes

### ⚠️ Startup Order Matters!

**ALWAYS start in this order:**
1. Gazebo (`gz-default`)
2. micro-ROS Agent (`microros`) ← **MUST be before PX4**
3. PX4 SITL (`px4-sitl`)
4. Configure MAVLink (at `pxh>` prompt)
5. QGroundControl (on Windows)

Starting PX4 before micro-ROS will cause communication issues!

### 💡 Auto-Detect Host IP

Use the helper script to auto-detect your Windows host IP:

```bash
~/installation-scripts/start-mavlink-host.sh
```

This will show you the correct MAVLink command to run.

### 🔄 Making Configuration Permanent

To avoid running the MAVLink command every time, create a startup override:

```bash
mkdir -p ~/px4_workspace/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix

cat > ~/px4_workspace/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rc.mavlink_local << 'EOF'
#!/bin/sh
# Auto-start MAVLink for Windows host

# Get Windows host IP (Docker gateway)
HOST_IP=$(ip route | grep default | awk '{print $3}')

# Start MAVLink targeting Windows host
mavlink start -x -u 14550 -t $HOST_IP -r 4000000

echo "[custom] MAVLink started targeting Windows host: $HOST_IP"
EOF

chmod +x ~/px4_workspace/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rc.mavlink_local
```

Then rebuild PX4:
```bash
cd ~/px4_workspace/PX4-Autopilot
make clean
make px4_sitl gz_x500
```

Now MAVLink will automatically connect to your Windows host on every startup!

## Quick Reference

### Complete Startup Commands

```bash
# Terminal 1
gz-default

# Terminal 2
microros

# Terminal 3
px4-sitl
# At pxh> prompt:
mavlink start -x -u 14550 -t <YOUR_WINDOWS_IP> -r 4000000

# Windows Host
# Start QGroundControl
```

### Verify Everything is Working

**PX4 Terminal:**
```bash
mavlink status        # Check partner IP is correct
listener vehicle_status  # See vehicle data
```

**QGroundControl:**
- Vehicle icon appears
- Parameters download successfully
- Flight mode displays
- Can arm/disarm vehicle

## Summary

✅ **QGroundControl on Windows host** = Use `-t <host_ip>` in MAVLink command
✅ **Start micro-ROS BEFORE PX4** = Ensures proper communication
✅ **Use `network_mode: host`** = Container can reach Windows host
✅ **UDP port 14550** = Standard QGC port

Your setup is now configured correctly for running QGroundControl on the Windows host while simulation runs in Docker!
