# Gazebo World Setup for PX4 SITL

## The Problem

PX4's default world file (`default.sdf`) does not include the required Gazebo system plugins for sensor simulation. This causes:

- **No sensor data published** (IMU, barometer, magnetometer, GPS)
- QGroundControl shows "Preflight Fail: Gyro Sensor 0 missing"
- `gz topic -e` shows topics exist but no data flows

## Why This Happens

Gazebo Harmonic (and newer) requires **system plugins at the world level** to process sensor data. The sensors are defined in the vehicle model SDF (e.g., `x500/model.sdf`), but the world must have the corresponding system plugins to actually run them.

PX4's `default.sdf` assumes Gazebo will auto-load these plugins, but when running Gazebo externally (separate container), they must be explicitly declared.

## The Fix

After cloning PX4-Autopilot, create a patched world file with the required plugins.

### Step 1: Create the Patched World File

In your **ROS2/Gazebo container**, run:

```bash
cat > ~/px4_gazebo/worlds/default_with_plugins.sdf << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">

    <!-- ============================================ -->
    <!-- REQUIRED SYSTEM PLUGINS FOR SENSORS         -->
    <!-- These must be at world level for Gazebo     -->
    <!-- Harmonic to process sensor data             -->
    <!-- ============================================ -->
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

    <!-- ============================================ -->
    <!-- Original PX4 default.sdf content below      -->
    <!-- ============================================ -->
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
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
EOF
```

### Step 2: Use the Patched World

Start Gazebo with the patched world file:

```bash
# The -r flag starts simulation immediately (not paused)
gz sim -r ~/px4_gazebo/worlds/default_with_plugins.sdf
```

### Step 3: Verify Sensors Work

```bash
# Check if sensor data is flowing (should see continuous output)
gz topic -e -t /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu

# Press Ctrl+C after confirming data flows
```

## Required Plugins Reference

| Plugin | Purpose |
|--------|---------|
| `gz-sim-physics-system` | Physics simulation |
| `gz-sim-user-commands-system` | User commands (spawn, delete models) |
| `gz-sim-scene-broadcaster-system` | Scene state broadcasting |
| `gz-sim-imu-system` | IMU sensor processing |
| `gz-sim-air-pressure-system` | Barometer sensor processing |
| `gz-sim-magnetometer-system` | Compass sensor processing |
| `gz-sim-navsat-system` | GPS sensor processing |
| `gz-sim-sensors-system` | Master sensor system (cameras, lidar) |

## Optional Plugins

Add these if you need additional features:

```xml
<!-- Contact detection -->
<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

<!-- Airspeed sensor (for fixed-wing) -->
<plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>

<!-- Optical flow sensor -->
<plugin filename="gz-sim-optical-flow-system" name="gz::sim::systems::OpticalFlow"/>

<!-- Lidar -->
<plugin filename="gz-sim-gpu-lidar-system" name="gz::sim::systems::GpuLidar"/>
```

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     ROS2/Gazebo Container                       │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Gazebo Harmonic Simulation                  │   │
│  │                                                          │   │
│  │  World (default_with_plugins.sdf)                       │   │
│  │  ├── System Plugins (Physics, Sensors, IMU, etc.)       │   │
│  │  └── Vehicle Model (x500_0)                             │   │
│  │      └── Sensor definitions (IMU, baro, mag, GPS)       │   │
│  │                                                          │   │
│  │  Plugins process sensor data → Publish to GZ topics     │   │
│  └─────────────────────────────────────────────────────────┘   │
│                            │                                    │
│                   Gazebo Transport                              │
│                            │                                    │
└────────────────────────────┼────────────────────────────────────┘
                             │
              ┌──────────────┴──────────────┐
              │      host network mode      │
              └──────────────┬──────────────┘
                             │
┌────────────────────────────┼────────────────────────────────────┐
│                            │                                    │
│               PX4 SITL Container                                │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    PX4 Autopilot                         │   │
│  │                                                          │   │
│  │  GZ Bridge ← Receives sensor data from Gazebo topics    │   │
│  │  ├── IMU → EKF2                                         │   │
│  │  ├── Barometer → Altitude estimation                    │   │
│  │  ├── Magnetometer → Heading                             │   │
│  │  └── GPS → Position                                     │   │
│  │                                                          │   │
│  │  MAVLink → QGroundControl (Windows host)                │   │
│  │  uXRCE-DDS → micro-ROS Agent → ROS 2 topics             │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Troubleshooting

### Sensors Not Publishing

1. **Check simulation is running** (not paused):
   ```bash
   gz topic -e -t /clock
   # Should see timestamps updating
   ```

2. **Check plugins are loaded**:
   ```bash
   grep "plugin" ~/px4_gazebo/worlds/default_with_plugins.sdf
   ```

3. **Verify model spawned**:
   ```bash
   gz model --list
   # Should show: x500_0
   ```

### QGroundControl Shows Sensor Errors

- Ensure you started Gazebo with `-r` flag (runs immediately)
- Ensure you're using `default_with_plugins.sdf`, not `default.sdf`
- Restart PX4 container after Gazebo is running

### "render_engine" Errors

If you see errors about ogre2, try changing to ogre:
```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre</render_engine>
</plugin>
```

## Quick Start Checklist

1. [ ] Clone PX4-Autopilot (if not done)
2. [ ] Create `default_with_plugins.sdf` with system plugins
3. [ ] Start Gazebo: `gz sim -r ~/px4_gazebo/worlds/default_with_plugins.sdf`
4. [ ] Verify sensors: `gz topic -e -t /world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu`
5. [ ] Start micro-ROS agent: `ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888`
6. [ ] Start PX4 container: `docker compose up -d`
7. [ ] Connect QGroundControl
