# PX4 SITL Quick Start

## Step 1: Start ROS2/Gazebo Container

```bash
cd ros2-gazebo-desktop
docker compose -f docker-compose-simple.yml up -d
```

Access VNC: http://localhost:6080/vnc.html

## Step 2: Create Patched World File (First Time Only)

In ROS2 container terminal, create the world file with sensor plugins:

```bash
cat > ~/px4_gazebo/worlds/default_with_plugins.sdf << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">
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

> Why? See [GAZEBO-WORLD-SETUP.md](GAZEBO-WORLD-SETUP.md)

## Step 3: Start Gazebo

In ROS2 container (VNC terminal):

```bash
gz sim -r ~/px4_gazebo/worlds/default_with_plugins.sdf
```

## Step 4: Start micro-ROS Agent

In ROS2 container (new terminal):

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```

## Step 5: Start PX4 SITL

```bash
cd px4-sitl-container
docker compose up -d
```

## Step 6: Connect QGroundControl (Windows)

1. Open QGroundControl on Windows
2. Auto-connects via UDP 14550
3. If not, add UDP connection manually to port 14550

> For Windows host setup, see [WINDOWS-HOST-QGC.md](WINDOWS-HOST-QGC.md)

---

## Multi-Vehicle Setup

For multiple vehicles, use the generator:

```bash
cd px4-sitl-container
./generate-compose.sh --copters 2 --rovers 1
docker compose -f docker-compose-generated.yml up -d
```

> Full details: [MULTI-VEHICLE-GUIDE.md](MULTI-VEHICLE-GUIDE.md)

---

## Startup Order

```
1. ROS2 Container  →  2. Gazebo  →  3. micro-ROS Agent  →  4. PX4 SITL  →  5. QGroundControl
```

---

## Detailed Documentation

| Document | Description |
|----------|-------------|
| [GAZEBO-WORLD-SETUP.md](GAZEBO-WORLD-SETUP.md) | Why sensor plugins are needed, troubleshooting |
| [MULTI-VEHICLE-GUIDE.md](MULTI-VEHICLE-GUIDE.md) | Running multiple vehicles, Docker Compose generator |
| [WINDOWS-HOST-QGC.md](WINDOWS-HOST-QGC.md) | QGroundControl setup on Windows host |
| [README.md](README.md) | Full container documentation |

---

## Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| "Sensor missing" errors | Use `default_with_plugins.sdf`, not `default.sdf` |
| No sensor data in `gz topic -e` | Gazebo paused - use `-r` flag or click Play |
| QGC won't connect | Check WINDOWS_HOST_IP in .env |
| Vehicle won't spawn | Start Gazebo before PX4 |
