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
| **Foreground** | `Ctrl+C` |
| **Background (any)** | `pkill -f micro_ros_agent` |
| **By PID** | `kill <PID>` |
| **Force kill** | `pkill -9 -f micro_ros_agent` |
| **tmux session** | `tmux kill-session -t micro_ros_agent` |
| **screen session** | `screen -S micro_ros -X quit` |
| **systemd service** | `sudo systemctl stop micro-ros-agent` |

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

Create a helper script:

```bash
#!/bin/bash
# stop_micro_ros.sh

echo "Stopping micro-ROS agent..."

# Find PID
PID=$(pgrep -f micro_ros_agent)

if [ -z "$PID" ]; then
    echo "micro-ROS agent is not running"
    exit 0
fi

echo "Found PID: $PID"

# Try graceful shutdown
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
    echo "ERROR: Failed to stop micro-ROS agent"
    exit 1
else
    echo "micro-ROS agent stopped successfully"
    exit 0
fi
```

Make it executable:
```bash
chmod +x stop_micro_ros.sh
./stop_micro_ros.sh
```

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
