# Manual micro-ROS Agent Installation Guide

This guide is for testing micro-ROS agent installation inside the running container. Once we find the working steps, they can be added back to the Dockerfile.

## Prerequisites

The Docker container must be built and running:
```bash
cd ros2-gazebo-desktop
sudo docker compose build
sudo docker compose up -d
```

The container already has:
- ✅ ROS 2 Jazzy Desktop Full
- ✅ Gazebo Harmonic
- ✅ Custom Fast DDS built from source (in `/home/rosuser/ros2_ws/src/fastdds_ws/`)
- ✅ MAVROS2 for ArduPilot

## Step 1: Access the Running Container

```bash
# Get a shell inside the container
sudo docker exec -it ros2_gazebo_vnc bash

# Or if using custom container name:
sudo docker exec -it <container_name> bash
```

## Step 2: Verify Environment

Check that the custom Fast DDS is available:

```bash
# Should show ROS and Fast DDS paths
echo $CMAKE_PREFIX_PATH

# Check Fast DDS installation
ls -la ~/ros2_ws/src/fastdds_ws/install/

# Should show: fastcdr, fastrtps, foonathan_memory_vendor
```

## Step 3: Clone micro-ROS Agent

```bash
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/micro-ROS/micro-ROS-Agent.git

# Verify
ls -la micro-ROS-Agent/
```

## Step 4: Attempt Build (Test Different Approaches)

### Approach A: Use Custom Fast DDS Only (Original Attempt)

```bash
cd ~/ros2_ws

# Source only custom Fast DDS (no ROS)
source src/fastdds_ws/install/setup.bash

# Try to build
colcon build --packages-select micro_ros_agent \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON
```

**Expected**: May fail with "Could not find rclcpp" or similar ROS package errors.

---

### Approach B: Source Both, Force Custom Fast DDS (What We Tried)

```bash
cd ~/ros2_ws

# Source ROS first, then overlay custom Fast DDS
source /opt/ros/jazzy/setup.bash
source src/fastdds_ws/install/setup.bash

# Try explicit path overrides
colcon build --packages-select micro_ros_agent \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON \
    -Dfastrtps_DIR=$HOME/ros2_ws/src/fastdds_ws/install/fastrtps/share/fastrtps/cmake \
    -Dfastcdr_DIR=$HOME/ros2_ws/src/fastdds_ws/install/fastcdr/share/fastcdr/cmake \
    -Dfoonathan_memory_DIR=$HOME/ros2_ws/src/fastdds_ws/install/foonathan_memory_vendor/share/foonathan_memory/cmake
```

**Expected**: May still get eProsima_atomic target conflict.

---

### Approach C: Override CMAKE_PREFIX_PATH

```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/fastdds_ws/install/setup.bash

# Force custom Fast DDS first in search path
CMAKE_PREFIX_PATH="$HOME/ros2_ws/src/fastdds_ws/install:$CMAKE_PREFIX_PATH" \
  colcon build --packages-select micro_ros_agent \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$HOME/ros2_ws/src/fastdds_ws/install;$CMAKE_PREFIX_PATH"
```

---

### Approach D: Unset ROS Fast DDS Paths Temporarily

```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash
source src/fastdds_ws/install/setup.bash

# Remove ROS Fast DDS from CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH=$(echo $CMAKE_PREFIX_PATH | sed 's|/opt/ros/jazzy/||g')

# Add custom Fast DDS first
export CMAKE_PREFIX_PATH="$HOME/ros2_ws/src/fastdds_ws/install:$CMAKE_PREFIX_PATH"

colcon build --packages-select micro_ros_agent \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

### Approach E: Use ROS's Fast DDS (Simplest - May Work)

**Question**: Does micro-ROS Agent actually require a specific Fast DDS version?

```bash
cd ~/ros2_ws

# Just use ROS environment, no custom Fast DDS
source /opt/ros/jazzy/setup.bash

# Try building with ROS's bundled Fast DDS
colcon build --packages-select micro_ros_agent \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**If this works**: We don't need custom Fast DDS at all! Can remove it from Dockerfile.

---

### Approach F: Patch micro-ROS CMakeLists.txt

If all else fails, modify the micro-ROS Agent's build configuration:

```bash
cd ~/ros2_ws/src/micro-ROS-Agent

# Backup original
cp CMakeLists.txt CMakeLists.txt.backup

# Check what it's looking for
grep -n "find_package.*fastrtps" CMakeLists.txt

# Example patch (line numbers may vary):
# Change: find_package(fastrtps REQUIRED)
# To: find_package(fastrtps REQUIRED PATHS /home/rosuser/ros2_ws/src/fastdds_ws/install NO_DEFAULT_PATH)
```

---

### Approach G: Install from Debian Package (If Available)

```bash
# Check if micro-ROS agent is available as a package
apt-cache search micro-ros

# If found:
sudo apt-get update
sudo apt-get install ros-jazzy-micro-ros-agent

# Then test:
ros2 run micro_ros_agent micro_ros_agent
```

## Step 5: Document What Worked

**Once you find a working approach**, document the exact commands:

```bash
# Save the successful build log
cd ~/ros2_ws
colcon build --packages-select micro_ros_agent [successful options] 2>&1 | tee /tmp/micro_ros_build_success.log

# Check linked libraries
ldd install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent | grep -E "fastrtps|fastcdr"

# Note which Fast DDS it's using:
# - If shows /home/rosuser/ros2_ws/src/fastdds_ws/install/lib/ → Using custom
# - If shows /opt/ros/jazzy/lib/ → Using ROS bundled
```

## Step 6: Test micro-ROS Agent

Once built successfully:

```bash
# Source the workspace
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/fastdds_ws/install/setup.bash  # if using custom Fast DDS
source ~/ros2_ws/install/setup.bash

# Run the agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# In another terminal, test with a micro-ROS client if available
```

## Step 7: Update Dockerfile

Once you have the working commands from Step 5, we can add them to the Dockerfile:

```dockerfile
# Add these steps before the VNC setup section:

# Clone micro-ROS agent
RUN cd /home/$USERNAME/ros2_ws/src \
    && git clone https://github.com/micro-ROS/micro-ROS-Agent.git \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws/src/micro-ROS-Agent

# Build micro-ROS agent (use the working approach from testing)
USER $USERNAME
RUN cd /home/$USERNAME/ros2_ws \
    && [INSERT WORKING SOURCE COMMANDS] \
    && [INSERT WORKING COLCON BUILD COMMAND]
USER root
```

## Common Issues and Solutions

### Issue: "Could not find package rclcpp"
**Cause**: ROS environment not sourced
**Fix**: Source `/opt/ros/jazzy/setup.bash` before building

### Issue: "eProsima_atomic target already defined"
**Cause**: CMake finding both custom and ROS Fast DDS
**Fix**: Try Approach C or D above to control search paths

### Issue: "Could not find fastrtps"
**Cause**: Custom Fast DDS not in search path
**Fix**: Source `src/fastdds_ws/install/setup.bash`

### Issue: Build succeeds but agent crashes at runtime
**Cause**: Linked against wrong Fast DDS version
**Fix**: Check `ldd` output, ensure runtime uses same Fast DDS as build

## Questions to Answer

1. **Does micro-ROS Agent work with ROS Jazzy's bundled Fast DDS?**
   - If YES: Remove custom Fast DDS build entirely
   - If NO: Document which specific version is needed

2. **What CMake options actually work?**
   - Document the exact working command

3. **Are there runtime environment requirements?**
   - Which setup.bash files must be sourced?
   - Any LD_LIBRARY_PATH overrides needed?

## Notes

- The container has 8GB memory limit and 2GB shared memory
- Custom Fast DDS is already built in `/home/rosuser/ros2_ws/src/fastdds_ws/`
- VNC is running if you need GUI tools (RQt, etc.)
- All changes inside the container are temporary unless committed to a new image

## Reporting Back

Once you've tested, share:
1. Which approach worked (A-G)
2. Exact commands used
3. Any errors encountered and how you fixed them
4. Output of `ldd` on the built binary
5. Whether the agent runs successfully

This will help create a clean, working Dockerfile!
