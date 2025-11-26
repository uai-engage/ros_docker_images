# ROS2 Gazebo Desktop - Quick Start Guide

Complete setup guide from cloning to running PX4 SITL simulation.

---

## Step 1: Clone the Repository

```bash
git clone <repo-url> ros_docker_images
cd ros_docker_images/ros2-gazebo-desktop
```

---

## Step 2: Build ROS2 Container

```bash
cd ros2-gazebo-desktop
sudo docker compose -f docker-compose-simple.yml --env-file .env.vnc build
```

---

## Step 3: Clone PX4-Autopilot

```bash
cd px4-sitl-container
git clone --branch v1.16.0 --depth 1 https://github.com/PX4/PX4-Autopilot.git
```

---

## Step 4: Build PX4 SITL Container

```bash
cd px4-sitl-container
sudo docker compose build
```

---

## Step 5: Generate PX4 Docker Compose (Optional - Multi-Vehicle)

```bash
cd px4-sitl-container

# For single vehicle, skip this step and use docker-compose.yml

# For multiple vehicles:
./generate-compose.sh --copters 2

# Or interactive mode:
./generate-compose.sh --interactive
```

> See [px4-sitl-container/MULTI-VEHICLE-GUIDE.md](px4-sitl-container/MULTI-VEHICLE-GUIDE.md) for details.

---

## Step 6: Create Patched World File (First Time Only)

PX4's default world file is missing sensor plugins. Create a patched version.

**After starting ROS2 container (Step 7), run this in the container:**

```bash
cat > ~/px4_gazebo/worlds/default_with_plugins.sdf << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">
    <!-- copy from here>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- copy to here>
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>0.001 0.625 -0.78</direction>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
    </spherical_coordinates>
  </world>
</sdf>
EOF
```

> Why? See [px4-sitl-container/GAZEBO-WORLD-SETUP.md](px4-sitl-container/GAZEBO-WORLD-SETUP.md)

---

## Step 7: Start ROS2 Container

```bash
cd ros2-gazebo-desktop
sudo docker compose -f docker-compose-simple.yml --env-file .env.vnc up -d
```

---

## Step 8: Connect to VNC

Open browser and connect to VNC:

```
http://localhost:6080/vnc.html
```
Or from VNC viewer
```
IP: System IP,  Port: 5901
```

Password: `rospassword` (or as configured in .env)

---

## Step 9: Start Gazebo (In VNC Terminal)

Open a terminal in VNC and run:

```bash
gz sim -r ~/px4_gazebo/worlds/default_with_plugins.sdf
```

> The `-r` flag starts simulation immediately (not paused).

---

## Step 10: Start micro-ROS Agent (In VNC - New Terminal)

Open another terminal in VNC and run:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

For multiple vehicles, run one agent per vehicle:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 &
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8889 &
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8890 &
```

---

## Step 11: Start PX4 SITL Container (From Host)

```bash
cd px4-sitl-container

# Single vehicle:
sudo docker compose up -d

# Or with generated compose (multi-vehicle):
sudo docker compose -f docker-compose-generated.yml up -d
```

---

## Step 12: Configure MAVLink for QGroundControl (In PX4 Container)

Connect to PX4 container and forward MAVLink to Windows host:

```bash
# Connect to PX4 container
sudo docker exec -it px4_sitl_0 bash

# In PX4 shell (pxh>), run:
mavlink start -x -u 14550 -t <WINDOWS_HOST_IP> -r 4000000
```

Replace `<WINDOWS_HOST_IP>` with your Windows machine IP (e.g., `192.168.18.93`).

> Note: If auto-detection is configured in start-px4-sitl.sh, this may happen automatically.

---

## Step 13: Connect QGroundControl (Windows Host)

1. Open QGroundControl on Windows
2. It should auto-connect via UDP 14550
3. If not, manually add: **Application Settings → Comm Links → Add → UDP → Port 14550**

> See [px4-sitl-container/WINDOWS-HOST-QGC.md](px4-sitl-container/WINDOWS-HOST-QGC.md) for details.

---

## Startup Order Summary

```
1. Build ROS2 Container
2. Clone & Build PX4 SITL Container
3. Create patched world file (first time)
4. Start ROS2 Container
5. Connect VNC
6. Start Gazebo (in VNC)
7. Start micro-ROS Agent (in VNC)
8. Start PX4 SITL Container (from host)
9. Configure MAVLink (in PX4 container)
10. Connect QGroundControl (Windows)
```

---

## Quick Reference Commands

| Action | Command |
|--------|---------|
| Build ROS2 | `sudo docker compose -f docker-compose-simple.yml --env-file .env build` |
| Start ROS2 | `sudo docker compose -f docker-compose-simple.yml --env-file .env up -d` |
| Stop ROS2 | `sudo docker compose -f docker-compose-simple.yml down` |
| Build PX4 | `cd px4-sitl-container && sudo docker compose build` |
| Start PX4 | `cd px4-sitl-container && sudo docker compose up -d` |
| Stop PX4 | `cd px4-sitl-container && sudo docker compose down` |
| View PX4 logs | `sudo docker compose -f docker-compose.yml logs -f` |
| Connect to PX4 | `sudo docker exec -it px4_sitl_0 bash` |
| Start Gazebo | `gz sim -r ~/px4_gazebo/worlds/default_with_plugins.sdf` |
| Start micro-ROS | `ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888` |

---

## Detailed Documentation

| Document | Description |
|----------|-------------|
| [px4-sitl-container/QUICKSTART.md](px4-sitl-container/QUICKSTART.md) | PX4 container quick start |
| [px4-sitl-container/GAZEBO-WORLD-SETUP.md](px4-sitl-container/GAZEBO-WORLD-SETUP.md) | Gazebo sensor plugins setup |
| [px4-sitl-container/MULTI-VEHICLE-GUIDE.md](px4-sitl-container/MULTI-VEHICLE-GUIDE.md) | Multi-vehicle simulation guide |
| [px4-sitl-container/WINDOWS-HOST-QGC.md](px4-sitl-container/WINDOWS-HOST-QGC.md) | QGroundControl on Windows setup |
| [px4-sitl-container/README.md](px4-sitl-container/README.md) | Full PX4 container documentation |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "Sensor missing" errors in QGC | Use `default_with_plugins.sdf`, not `default.sdf` |
| Gazebo shows no sensor data | Simulation paused - use `-r` flag or click Play button |
| QGC won't connect | Run `mavlink start` command in PX4 container with correct IP |
| Vehicle won't spawn in Gazebo | Ensure Gazebo is running before starting PX4 container |
| micro-ROS not connecting | Start micro-ROS agent before PX4 container |
| VNC not accessible | Check port 6080 is not blocked, container is running |
