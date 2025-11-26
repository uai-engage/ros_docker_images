# QGroundControl Windows Host - Quick Start

## Your Working Configuration ✅

**Setup:** QGroundControl on Windows Host + PX4/Gazebo in Docker Container

## Startup Commands

### 1. Find Windows Host IP (One-Time)

**Windows PowerShell:**
```powershell
ipconfig
```
Note your IPv4 address (e.g., `192.168.18.93`)

### 2. Start Container Services

**Container Terminal 1:**
```bash
gz-default
```

**Container Terminal 2:**
```bash
microros
```

**Container Terminal 3:**
```bash
px4-sitl
```

**At `pxh>` prompt:**
```bash
mavlink start -x -u 14550 -t 192.168.18.93 -r 4000000
```
*(Replace with YOUR Windows IP)*

### 3. Start QGroundControl on Windows

Launch QGroundControl on Windows - it will auto-connect!

## ⚠️ Critical Points

1. **Startup Order:**
   - Gazebo FIRST
   - micro-ROS SECOND (before PX4!)
   - PX4 THIRD
   - Configure MAVLink FOURTH

2. **MAVLink Target:**
   - Use Windows host IP, NOT `127.0.0.1`
   - Example: `-t 192.168.18.93`

3. **Verify Connection:**
   ```bash
   mavlink status
   ```
   Should show: `partner IP: 192.168.18.93`

## Helper Script

Auto-detect your Windows IP:
```bash
~/installation-scripts/start-mavlink-host.sh
```

## Full Documentation

See: `USAGE-WINDOWS-HOST-QGC.md` for complete guide.

---

**This configuration is verified working! No further changes needed.**
