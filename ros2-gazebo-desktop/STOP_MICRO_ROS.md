# How to Stop micro-ROS Agent

## If Running in Foreground (Terminal)

Simply press:
```
Ctrl + C
```

The agent will cleanly shut down and you'll get your terminal back.

---

## If Running in Background

### Method 1: Find and Kill by Process Name

```bash
# Find the process
ps aux | grep micro_ros_agent

# Output example:
# rosuser  12345  5.0  0.3  123456  78910 ?  Sl  10:30  0:05 /opt/micro_ros_agent_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent udp4 --port 2019

# Kill by PID
kill 12345

# Or force kill if not responding
kill -9 12345
```

### Method 2: Kill All micro-ROS Agent Processes

```bash
# Kill all instances
pkill -f micro_ros_agent

# Force kill if not responding
pkill -9 -f micro_ros_agent
```

### Method 3: Using ROS 2 Node List

```bash
# List all ROS 2 nodes
ros2 node list

# If micro-ROS agent shows up, find its PID
ps aux | grep micro_ros_agent

# Kill the PID
kill <PID>
```

---

## If Running in tmux Session

### List tmux Sessions
```bash
tmux list-sessions

# Output example:
# micro_ros_agent: 1 windows (created Mon Nov 18 10:30:00 2025)
```

### Attach and Stop
```bash
# Attach to session
tmux attach -t micro_ros_agent

# Press Ctrl+C to stop agent

# Detach from tmux (leave session running)
# Press: Ctrl+B, then D
```

### Kill tmux Session
```bash
# Kill the entire session (stops agent)
tmux kill-session -t micro_ros_agent
```

---

## If Running in screen Session

### List screen Sessions
```bash
screen -ls

# Output example:
# There is a screen on:
#     12345.micro_ros  (Detached)
```

### Attach and Stop
```bash
# Attach to session
screen -r micro_ros

# Press Ctrl+C to stop agent

# Detach from screen
# Press: Ctrl+A, then D
```

### Kill screen Session
```bash
# Kill the session
screen -S micro_ros -X quit
```

---

## If Running as systemd Service

```bash
# Stop the service
sudo systemctl stop micro-ros-agent

# Check status
sudo systemctl status micro-ros-agent

# Disable auto-start on boot
sudo systemctl disable micro-ros-agent
```

---

## Quick Commands Cheat Sheet

| Scenario | Command |
|----------|---------|
| **Foreground** | `Ctrl+C` then `ros2 daemon stop && ros2 daemon start` |
| **Background (any)** | `pkill -f micro_ros_agent` then clear cache |
| **By PID** | `kill <PID>` then clear cache |
| **Force kill** | `pkill -9 -f micro_ros_agent` then clear cache |
| **tmux session** | `tmux kill-session -t micro_ros_agent` then clear cache |
| **screen session** | `screen -S micro_ros -X quit` then clear cache |
| **systemd service** | `sudo systemctl stop micro-ros-agent` then clear cache |
| **Clear DDS cache** | `ros2 daemon stop && ros2 daemon start` |
| **Complete cleanup** | `pkill -f micro_ros_agent && ros2 daemon stop && ros2 daemon start` |

---

## Verify Agent is Stopped

```bash
# Check if still running
ps aux | grep micro_ros_agent

# Should show only the grep command itself, not the agent

# Check port is free
netstat -tulpn | grep 2019

# Should show nothing if agent stopped
```

---

## Clear Ghost Topics (IMPORTANT!)

After stopping micro-ROS agent, topics may still appear in `ros2 topic list` even though the agent is stopped. This is due to **ROS 2 DDS discovery cache**.

### Problem: Ghost Topics Remain

```bash
# Agent stopped but topics still show
ros2 topic list
# /fmu/battery/status  ← Still here!
# /fmu/gps/position    ← Ghost topics!
# /fmu/imu/data

# Try to echo - no data comes through
ros2 topic echo /fmu/battery/status
# (nothing - no messages)
```

### Solution: Restart ROS 2 Daemon

```bash
# Stop the ROS 2 daemon (clears DDS cache)
ros2 daemon stop

# Start it again
ros2 daemon start

# Verify topics are gone
ros2 topic list
# Should only show /parameter_events and /rosout if no other nodes running
```

### Why This Happens

1. ROS 2 uses DDS (Data Distribution Service) for discovery
2. When micro-ROS agent publishes topics, they're registered in DDS
3. Ctrl+C stops the process but doesn't always clean up DDS registration
4. DDS daemon caches topic information
5. Topics appear even though publisher is gone (ghost topics)

### Complete Stop Procedure

```bash
# 1. Stop the agent
pkill -f micro_ros_agent

# 2. Clear DDS cache
ros2 daemon stop
ros2 daemon start

# 3. Verify everything clean
ps aux | grep micro_ros_agent  # Should show nothing
ros2 topic list                 # Should show no /fmu/* topics
netstat -tulpn | grep 2019      # Should show nothing
```

---

## Common Issues

### "Process won't stop with Ctrl+C"

```bash
# Find PID
ps aux | grep micro_ros_agent

# Force kill
kill -9 <PID>
```

### "Port still in use after stopping"

```bash
# Find what's using the port
sudo lsof -i :2019

# Or
sudo netstat -tulpn | grep 2019

# Kill the process
sudo kill -9 <PID>

# Wait 30 seconds for port to be released
```

### "Multiple instances running"

```bash
# Kill all instances
pkill -9 -f micro_ros_agent

# Verify all stopped
ps aux | grep micro_ros_agent
```

---

## Graceful Shutdown Script

Create a helper script that stops the agent AND clears DDS cache:

```bash
#!/bin/bash
# stop_micro_ros.sh

echo "==========================================="
echo "  Stopping micro-ROS Agent & Clearing Cache"
echo "==========================================="
echo ""

# Find PID
PID=$(pgrep -f micro_ros_agent)

if [ -z "$PID" ]; then
    echo "✓ micro-ROS agent is not running"
else
    echo "Found micro-ROS agent PID: $PID"

    # Try graceful shutdown
    echo "Stopping agent gracefully..."
    kill $PID
    sleep 2

    # Check if still running
    if pgrep -f micro_ros_agent > /dev/null; then
        echo "Process still running, forcing shutdown..."
        pkill -9 -f micro_ros_agent
        sleep 1
    fi

    # Verify stopped
    if pgrep -f micro_ros_agent > /dev/null; then
        echo "✗ ERROR: Failed to stop micro-ROS agent"
        exit 1
    else
        echo "✓ micro-ROS agent stopped"
    fi
fi

# Clear ROS 2 DDS cache (removes ghost topics)
echo ""
echo "Clearing ROS 2 DDS discovery cache..."
ros2 daemon stop
sleep 1
ros2 daemon start
sleep 1

echo ""
echo "Verifying cleanup..."

# Check topics
TOPICS=$(ros2 topic list 2>/dev/null | grep -c "/fmu/")
if [ "$TOPICS" -eq 0 ]; then
    echo "✓ Ghost topics cleared"
else
    echo "✗ Warning: $TOPICS /fmu/ topics still present"
fi

# Check port
if netstat -tuln 2>/dev/null | grep -q ":2019"; then
    echo "✗ Warning: Port 2019 still in use"
else
    echo "✓ Port 2019 is free"
fi

echo ""
echo "==========================================="
echo "  Cleanup Complete!"
echo "==========================================="
exit 0
```

Make it executable and use it:
```bash
chmod +x stop_micro_ros.sh
./stop_micro_ros.sh
```

This script:
- ✅ Stops micro-ROS agent (graceful then force if needed)
- ✅ Clears ROS 2 DDS cache (removes ghost topics)
- ✅ Verifies cleanup
- ✅ Reports status

---

## Auto-Restart Prevention

If using systemd and you want to prevent auto-restart:

```bash
# Disable service
sudo systemctl disable micro-ros-agent

# Stop service
sudo systemctl stop micro-ros-agent

# Mask service (prevent any start)
sudo systemctl mask micro-ros-agent
```

To re-enable later:
```bash
sudo systemctl unmask micro-ros-agent
sudo systemctl enable micro-ros-agent
sudo systemctl start micro-ros-agent
```
