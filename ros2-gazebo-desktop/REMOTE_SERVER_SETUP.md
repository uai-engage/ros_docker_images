# Running ROS 2 Gazebo on Remote Ubuntu Server with Windows X11 Display

This guide explains how to run the ROS 2 Gazebo Docker container on a remote Ubuntu server while displaying the GUI on your Windows machine.

## Architecture

```
[Windows Host]              [Ubuntu Server]
  VcXsrv/X410    <---X11---   Docker Container
  (X Server)        TCP/SSH    (ROS 2 + Gazebo)
```

## Prerequisites

### On Ubuntu Server
- Ubuntu 20.04/22.04/24.04
- Docker and Docker Compose installed
- Network connectivity to Windows host
- (Optional) SSH server for secure X11 forwarding

### On Windows Host
- X Server (VcXsrv, X410, or similar)
- Network connectivity to Ubuntu server
- (Optional) SSH client for secure forwarding

---

## Method 1: Direct X11 Connection (Simple)

### Step 1: Configure Windows X Server

1. **Install VcXsrv** on Windows
2. Launch XLaunch with these settings:
   - Multiple windows
   - Display number: **0**
   - Start no client
   - ✅ **Disable access control** (Important!)
   - ✅ **Clipboard** (optional)
3. Allow VcXsrv through Windows Firewall (both private and public)

### Step 2: Open Firewall on Windows

X11 uses port 6000 + display number. For display :0, open port 6000:

```powershell
# PowerShell (Administrator)
New-NetFirewallRule -DisplayName "X11 Server" -Direction Inbound -Protocol TCP -LocalPort 6000-6010 -Action Allow
```

### Step 3: Get Your Windows IP

```powershell
ipconfig
# Note your IPv4 address, e.g., 192.168.1.100
```

### Step 4: Configure on Ubuntu Server

```bash
# Clone/copy the ros2-gazebo-desktop folder to your Ubuntu server
cd ros2-gazebo-desktop

# Copy and edit remote configuration
cp .env.remote .env

# Edit .env - set DISPLAY to your Windows IP
nano .env
# Change: DISPLAY=192.168.1.100:0.0
```

### Step 5: Build and Run

```bash
# On Ubuntu server
docker-compose build
docker-compose up -d
docker exec -it ros2_gazebo_remote bash
```

### Step 6: Test GUI

```bash
# Inside container
xeyes  # Should display on your Windows screen!
gz sim shapes.sdf  # Launch Gazebo
```

---

## Method 2: SSH X11 Forwarding (Secure)

This method encrypts X11 traffic through SSH.

### Step 1: Configure Ubuntu Server SSH

```bash
# On Ubuntu server, edit SSH config
sudo nano /etc/ssh/sshd_config

# Ensure these lines are present:
X11Forwarding yes
X11DisplayOffset 10
X11UseLocalhost no

# Restart SSH
sudo systemctl restart sshd
```

### Step 2: Install X11 Auth on Server

```bash
sudo apt-get install xauth
```

### Step 3: Configure Windows X Server

1. Start VcXsrv with same settings as Method 1
2. **Important**: Still disable access control

### Step 4: SSH from Windows with X11 Forwarding

Using PowerShell or Windows Terminal:

```powershell
# Basic X11 forwarding
ssh -X user@ubuntu-server-ip

# Trusted forwarding (faster, required for some apps)
ssh -Y user@ubuntu-server-ip
```

Using PuTTY:
1. Connection → SSH → X11
2. ✅ Enable X11 forwarding
3. X display location: `localhost:0`

### Step 5: Run Docker with SSH DISPLAY

Once SSH'd into the server:

```bash
# Check DISPLAY is set by SSH
echo $DISPLAY
# Should show something like: localhost:10.0

# Run container with SSH's DISPLAY
cd ros2-gazebo-desktop

# Override DISPLAY in docker run
docker exec -e DISPLAY=$DISPLAY -it ros2_gazebo_remote bash
```

Or modify docker-compose to pass through host DISPLAY:

```yaml
environment:
  - DISPLAY=${DISPLAY}
```

### Step 6: Fix X11 Authentication

SSH X11 forwarding uses Xauthority. You need to share it with the container:

```bash
# On server, after SSH login
xauth list  # Shows your X11 cookie

# Run container with proper auth
docker run -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/home/rosuser/.Xauthority:ro \
  --network host \
  ros2-gazebo-desktop:jazzy-noble bash
```

---

## Method 3: Using docker-compose-remote.yml

I've provided a separate compose file for remote deployment:

```bash
# On Ubuntu server
docker-compose -f docker-compose-remote.yml up -d
```

---

## Troubleshooting

### "Cannot open display"

1. **Check X server is running** on Windows
2. **Verify Windows IP** is correct in `.env`
3. **Test connectivity**:
   ```bash
   # From Ubuntu server
   ping <windows-ip>
   nc -zv <windows-ip> 6000
   ```
4. **Check Windows firewall** allows port 6000

### "No protocol specified" or "Authorization required"

For direct connection:
```bash
# On Windows, ensure VcXsrv has "Disable access control" checked
```

For SSH:
```bash
# Ensure xauth is configured
xauth list
# Copy the cookie to container if needed
```

### Slow GUI Performance

1. **Use compression** with SSH:
   ```bash
   ssh -X -C user@server  # -C enables compression
   ```

2. **Reduce color depth**:
   ```bash
   export GDK_NATIVE_WINDOWS=1
   export QT_X11_NO_MITSHM=1
   ```

3. **Use VNC/NoMachine instead** for better performance (alternative approach)

### Gazebo Specific Issues

1. **Black screen in Gazebo**:
   ```bash
   # Force software rendering
   export LIBGL_ALWAYS_SOFTWARE=1
   gz sim
   ```

2. **Gazebo GUI crashes**:
   ```bash
   # Increase shared memory
   docker run --shm-size=2g ...
   ```

---

## Performance Optimization

### For Better X11 Performance

1. **Use LAN connection** (not WiFi)
2. **Reduce Gazebo visual complexity**:
   ```bash
   gz sim --render-engine ogre  # Instead of ogre2
   ```
3. **Lower resolution** in Gazebo settings
4. **Disable shadows and complex shaders**

### Alternative: VNC/Remote Desktop

For better performance, consider running VNC server inside container:

```bash
# Install TigerVNC in Dockerfile
apt-get install -y tigervnc-standalone-server

# Start VNC server
vncserver :1 -geometry 1920x1080 -depth 24

# Connect from Windows using VNC viewer
```

---

## Security Considerations

### Direct X11 (Method 1)
- ⚠️ **Not encrypted** - X11 traffic is plaintext
- ⚠️ **Access control disabled** - anyone on network can connect
- ✅ Use only on trusted/private networks
- ✅ Consider VPN for remote access

### SSH X11 (Method 2)
- ✅ **Encrypted** through SSH tunnel
- ✅ **Authenticated** using SSH keys/passwords
- ✅ Recommended for remote access
- ⚠️ Slightly slower due to encryption overhead

### Firewall Recommendations

```bash
# On Ubuntu server - only allow your Windows IP
sudo ufw allow from <windows-ip> to any port 6000
```

---

## Network Diagrams

### Direct Connection
```
Windows (192.168.1.100)          Ubuntu Server (192.168.1.200)
┌─────────────────┐              ┌──────────────────────────┐
│   VcXsrv        │◄─────X11─────│   Docker Container       │
│   Port 6000     │   Port 6000  │   DISPLAY=192.168.1.100:0│
└─────────────────┘              └──────────────────────────┘
```

### SSH X11 Forwarding
```
Windows                           Ubuntu Server
┌─────────────────┐              ┌──────────────────────────┐
│   VcXsrv        │              │   SSH Server             │
│   Port 6000     │◄───SSH───────│   X11 Forwarding         │
│                 │   Encrypted  │   ┌──────────────────┐   │
│   SSH Client    │──────────────│   │ Docker Container │   │
│                 │              │   │ DISPLAY=:10.0    │   │
└─────────────────┘              │   └──────────────────┘   │
                                 └──────────────────────────┘
```

---

## Quick Reference Commands

```bash
# Build on server
docker-compose -f docker-compose-remote.yml build

# Start container
docker-compose -f docker-compose-remote.yml up -d

# Connect to container
docker exec -it ros2_gazebo_remote bash

# Test X11
xeyes
glxgears

# Launch Gazebo
gz sim shapes.sdf

# Launch RViz
ros2 run rviz2 rviz2

# Stop
docker-compose -f docker-compose-remote.yml down
```

## Summary

Both methods work well for running ROS 2 Gazebo on a remote Ubuntu server with Windows X11 display:

- **Direct X11**: Simpler setup, better for LAN, less secure
- **SSH X11**: More secure, works over internet, slightly more complex

For production/remote access, use SSH X11 forwarding. For local network development, direct X11 is faster and simpler.
