# ROS 2 Networking in Docker - Complete Guide

This guide explains how ROS 2 DDS (Data Distribution Service) communication works in Docker containers and how to connect with external ROS 2 nodes (like your R2 Pilot).

## Table of Contents
- [Understanding ROS 2 DDS Communication](#understanding-ros-2-dds-communication)
- [Docker Network Modes](#docker-network-modes)
- [Connecting to External ROS 2 Nodes](#connecting-to-external-ros-2-nodes)
- [Multi-Container ROS 2 Setup](#multi-container-ros-2-setup)
- [Troubleshooting Communication Issues](#troubleshooting-communication-issues)

---

## Understanding ROS 2 DDS Communication

### How Fast DDS Works

ROS 2 uses DDS (Data Distribution Service) for communication. Fast DDS (eProsima) is the default implementation:

```
[Node A]  <--DDS Multicast Discovery-->  [Node B]
    |                                        |
    +-------- UDP/TCP Data Transfer ---------+
```

**Key Components:**
1. **Discovery**: Nodes announce themselves via multicast (224.0.0.0/4)
2. **Data Exchange**: Topics are published/subscribed via UDP/TCP
3. **Domain ID**: Isolates ROS 2 networks (default: 0)

### Ports Used by Fast DDS

| Port Range | Purpose |
|------------|---------|
| 7400 | PDP (Participant Discovery Protocol) |
| 7401+ | User traffic ports |
| 11811 | Fast DDS Discovery Server (optional) |

---

## Docker Network Modes

### Bridge Network (Isolated)

```yaml
network_mode: bridge  # Default
ports:
  - "5901:5901"
```

**Characteristics:**
- Container has its own network namespace
- NAT translation between container and host
- Multicast doesn't cross network boundaries
- **ROS 2 nodes CANNOT discover external nodes automatically**

```
[External ROS 2]    [Docker Container]
      |                    |
   Host Network    ----X---- Container Network
      |                    |
  Can't discover each other!
```

### Host Network (Shared)

```yaml
network_mode: host
# No ports: mapping needed
```

**Characteristics:**
- Container shares host's network stack
- All ports directly accessible
- Multicast works normally
- **ROS 2 nodes CAN discover external nodes**

```
[External ROS 2]    [Docker Container]
      |                    |
   Host Network ======== Host Network
      |                    |
   Same network - Discovery works!
```

---

## Connecting to External ROS 2 Nodes

### Option 1: Host Network (Recommended)

Use the host network compose file:

```bash
# Build with XFCE desktop
docker compose -f docker-compose-vnc-xfce.yml build

# Or use simple version
docker compose -f docker-compose-vnc-host.yml build

# Start
docker compose -f docker-compose-vnc-xfce.yml up -d
```

**Communication Test:**

```bash
# On Ubuntu Server (outside container)
ros2 topic pub /test std_msgs/String "data: 'from host'"

# Inside container (via VNC terminal)
ros2 topic echo /test
# Should see: data: 'from host'
```

**Connecting R2 Pilot:**

```bash
# Ensure same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# On your R2 Pilot system
ros2 run your_pilot your_node

# Inside container
ros2 topic list  # Should see pilot's topics
ros2 node list   # Should see pilot's nodes
```

### Option 2: Bridge Network with Discovery Server

If you must use bridge network (for port control), use Fast DDS Discovery Server:

**Inside Container (.env.vnc):**
```bash
ROS_DISCOVERY_SERVER=<host-ip>:11811
```

**On Host:**
```bash
# Start discovery server
fastdds discovery -i 0 -p 11811
```

**Docker Compose:**
```yaml
environment:
  - ROS_DISCOVERY_SERVER=host.docker.internal:11811
```

### Option 3: Specific UDP Port Forwarding

For bridge network, manually expose DDS ports:

```yaml
network_mode: bridge
ports:
  - "5901:5901"       # VNC
  - "6080:6080"       # noVNC
  - "7400:7400/udp"   # DDS discovery
  - "7401-7420:7401-7420/udp"  # DDS data
```

---

## Practical Examples

### Example 1: Container + Host Node Communication

```bash
# Terminal 1: Start container with host network
docker compose -f docker-compose-vnc-xfce.yml up -d

# Terminal 2: Inside container (via VNC or exec)
docker exec -it ros2_gazebo_vnc bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py talker

# Terminal 3: On host (Ubuntu server)
source /opt/ros/jazzy/setup.bash
ros2 topic echo /chatter
# Should receive messages from container
```

### Example 2: Container + Remote R2 Pilot

**Architecture:**
```
[Windows Client]
      |
      | VNC (view only)
      v
[Ubuntu Server]
  +-----------------+
  | Docker Container|  <-- ROS 2 Topics --> [R2 Pilot]
  | (Gazebo + ROS2) |                        (Same network)
  +-----------------+
```

**Setup:**

1. **Start Container:**
```bash
docker compose -f docker-compose-vnc-xfce.yml up -d
```

2. **Configure R2 Pilot** (must be on same network):
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run r2_pilot pilot_node
```

3. **Inside Container** (via VNC):
```bash
# Check connectivity
ros2 node list  # Should show pilot_node

# Subscribe to pilot data
ros2 topic echo /pilot/status

# Send commands to pilot
ros2 topic pub /pilot/command ...
```

### Example 3: Gazebo Simulation with External Controller

```bash
# In container VNC
gz sim worlds/empty.sdf

# Launch ROS-Gazebo bridge
ros2 launch ros_gz_sim gz_sim.launch.py

# On external system (same network)
ros2 topic pub /cmd_vel geometry_msgs/Twist ...
# Control simulated robot from outside!
```

---

## Network Configuration Files

### Fast DDS XML Configuration

Located at `~/.ros/fastdds.xml` in container:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="participant_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <discoveryProtocol>SIMPLE</discoveryProtocol>
          <use_WriterLivelinessProtocol>AUTOMATIC</use_WriterLivelinessProtocol>
          <leaseDuration>
            <sec>DURATION_INFINITY</sec>
          </leaseDuration>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `ROS_DOMAIN_ID` | DDS domain isolation | `0` |
| `RMW_IMPLEMENTATION` | DDS implementation | `rmw_fastrtps_cpp` |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | Fast DDS config path | - |
| `ROS_DISCOVERY_SERVER` | Discovery server address | - |
| `ROS_LOCALHOST_ONLY` | Restrict to localhost | `0` |

---

## Multi-Container ROS 2 Setup

### Same Host, Multiple Containers

```yaml
# docker-compose-multi.yml
services:
  gazebo-sim:
    image: ros2-gazebo-desktop:jazzy-vnc-xfce
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0

  controller:
    image: ros2-controller:latest
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    depends_on:
      - gazebo-sim
```

### Custom Docker Network

```yaml
networks:
  ros2_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16

services:
  gazebo:
    networks:
      ros2_network:
        ipv4_address: 172.20.0.2
    environment:
      - ROS_DOMAIN_ID=42
```

---

## Troubleshooting Communication Issues

### 1. Nodes Can't Discover Each Other

**Check Network Mode:**
```bash
docker inspect ros2_gazebo_vnc | grep NetworkMode
# Should be "host" for external communication
```

**Check Domain ID:**
```bash
# Both systems must match
echo $ROS_DOMAIN_ID
```

**Check Firewall:**
```bash
# On Ubuntu server
sudo ufw status
sudo ufw allow 7400:7500/udp  # DDS ports
```

### 2. Topics Visible But No Data

**Check RMW Implementation:**
```bash
# Must match on all systems
echo $RMW_IMPLEMENTATION
# Should be: rmw_fastrtps_cpp
```

**Check Multicast:**
```bash
# Test multicast connectivity
ros2 multicast receive
ros2 multicast send  # From another terminal
```

### 3. Slow Topic Discovery

**Restart DDS Daemon:**
```bash
ros2 daemon stop
ros2 daemon start
```

**Check Network Interface:**
```bash
# Ensure correct interface is used
ip addr show
```

### 4. Connection Works Then Drops

**Increase Lease Duration** in fastdds.xml:
```xml
<leaseDuration>
  <sec>DURATION_INFINITY</sec>
</leaseDuration>
```

---

## Security Considerations

### Network Exposure

Using `network_mode: host` exposes all container ports directly:

```bash
# All these are accessible from network:
5901  # VNC
6080  # noVNC
7400+ # DDS ports
Any ROS 2 service ports
```

### Recommendations

1. **Use Firewall:**
```bash
sudo ufw default deny incoming
sudo ufw allow from <trusted-ip> to any
```

2. **Use VPN** for remote access

3. **Set Secure VNC Password:**
```bash
VNC_PASSWORD=YourStrongPassword123!
```

4. **Monitor Traffic:**
```bash
ros2 topic list  # Check for unexpected topics
ros2 node list   # Check for unexpected nodes
```

---

## Common Architectures

### Architecture 1: Simulation + Real Robot

```
[VNC Desktop Container]     [Physical Robot]
      Gazebo Sim       <-->    ROS 2 Nodes
      Digital Twin              Sensors/Actuators
          |                          |
          +---- Same Topics ---------+
```

### Architecture 2: Development Setup

```
[Your Windows PC]
       |
       | VNC Access
       v
[Ubuntu Server Container]
       |
       | ROS 2 DDS
       v
[R2 Pilot / PX4 / ArduPilot]
       |
       | MAVLink/DDS
       v
[Autopilot Hardware/SITL]
```

### Architecture 3: Multi-Robot Simulation

```
[Container 1: Robot A]  <-->  [Container 2: Robot B]
        |                              |
        +------ Shared ROS 2 Domain ---+
                     |
                     v
            [Central Controller]
```

---

## Summary

**For ROS 2 communication with external systems:**

1. **Use `network_mode: host`** - This is the simplest and most reliable
2. **Match `ROS_DOMAIN_ID`** across all systems
3. **Use same `RMW_IMPLEMENTATION`** (rmw_fastrtps_cpp)
4. **Ensure network connectivity** (same subnet or routable)

**Recommended Compose File:**
```bash
docker compose -f docker-compose-vnc-xfce.yml build
docker compose -f docker-compose-vnc-xfce.yml up -d
```

This gives you:
- Full XFCE desktop GUI
- Host network for ROS 2 discovery
- VNC access on port 5901
- noVNC web access on port 6080
- Automatic discovery with external ROS 2 nodes
