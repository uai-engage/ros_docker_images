# micro-ROS Agent Build Fix

## Problem Summary

The build was failing with a CMake target conflict error:
```
CMake Error: Some (but not all) targets in this export set were already defined.
  Targets Defined: eProsima_atomic
  Targets not yet defined: fastrtps
```

## Root Cause

**The Conflict:**
1. Custom Fast DDS is built and defines CMake target `eProsima_atomic`
2. micro-ROS agent needs ROS packages (`rmw_fastrtps_shared_cpp`)
3. ROS packages depend on ROS's bundled Fast DDS (`/opt/ros/jazzy/share/fastrtps/`)
4. When CMake tries to load ROS's Fast DDS, it conflicts with custom Fast DDS targets

**Why This Happens:**
- micro-ROS Agent requires specific Fast DDS versions not matching ROS Jazzy's bundled version
- But it also needs ROS 2 infrastructure (rclcpp, rmw, etc.)
- CMake cannot have two versions of the same library's targets in one build

## Solution Applied (Dockerfile:129-138)

### What Changed:

**BEFORE:**
```dockerfile
RUN cd /home/$USERNAME/ros2_ws \
    && . src/fastdds_ws/install/setup.sh \              # Only custom Fast DDS
    && colcon build --packages-select micro_ros_agent \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON
```

**AFTER:**
```dockerfile
RUN cd /home/$USERNAME/ros2_ws \
    && . /opt/ros/jazzy/setup.sh \                      # Source ROS FIRST
    && . src/fastdds_ws/install/setup.sh \              # Then overlay custom Fast DDS
    && colcon build --packages-select micro_ros_agent \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON \
         -Dfastrtps_DIR=/home/$USERNAME/ros2_ws/src/fastdds_ws/install/fastrtps/share/fastrtps/cmake \
         -Dfastcdr_DIR=/home/$USERNAME/ros2_ws/src/fastdds_ws/install/fastcdr/share/fastcdr/cmake \
         -Dfoonathan_memory_DIR=/home/$USERNAME/ros2_ws/src/fastdds_ws/install/foonathan_memory_vendor/share/foonathan_memory/cmake
```

### Why This Works:

1. **Source ROS first**: Provides access to ROS packages (rmw, rclcpp, etc.)
2. **Source custom Fast DDS second**: Overlays custom Fast DDS paths on top
3. **Explicit -D flags**: Forces CMake to use custom Fast DDS by explicitly setting paths
4. **Path precedence**: Custom Fast DDS paths override ROS paths in CMAKE_PREFIX_PATH

The key is the explicit `-Dfastrtps_DIR=` flags which tell CMake:
- "I know you'll find multiple Fast DDS configs"
- "Use THIS specific one from the custom build"
- "Ignore the one in /opt/ros/jazzy/"

## Testing the Build

Run the build:
```bash
cd ros2-gazebo-desktop
sudo docker compose build
```

### Expected Behavior:
- Fast DDS builds successfully (CACHED from previous attempt)
- micro-ROS agent build finds custom Fast DDS explicitly
- Build completes without CMake target conflicts

### If It Still Fails:

Check the error message:

#### Error 1: "Could not find fastrtps"
**Cause**: Path to custom Fast DDS is wrong
**Fix**: Verify install directory structure:
```bash
# After Fast DDS build step, add this debug line:
RUN ls -la /home/$USERNAME/ros2_ws/src/fastdds_ws/install/
```

Expected output:
```
fastcdr/
fastrtps/
foonathan_memory_vendor/
```

#### Error 2: "Could not find rclcpp" or "Could not find rmw"
**Cause**: ROS not sourced properly
**Fix**: Ensure `/opt/ros/jazzy/setup.sh` is sourced BEFORE custom Fast DDS

#### Error 3: Same eProsima_atomic error
**Cause**: CMake still finding ROS Fast DDS first
**Fix**: Add more aggressive path control:
```dockerfile
RUN cd /home/$USERNAME/ros2_ws \
    && . /opt/ros/jazzy/setup.sh \
    && . src/fastdds_ws/install/setup.sh \
    && CMAKE_PREFIX_PATH="/home/$USERNAME/ros2_ws/src/fastdds_ws/install:$CMAKE_PREFIX_PATH" \
       colcon build --packages-select micro_ros_agent \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_PREFIX_PATH="/home/$USERNAME/ros2_ws/src/fastdds_ws/install;$CMAKE_PREFIX_PATH" \
         -Dfastrtps_DIR=/home/$USERNAME/ros2_ws/src/fastdds_ws/install/fastrtps/share/fastrtps/cmake \
         -Dfastcdr_DIR=/home/$USERNAME/ros2_ws/src/fastdds_ws/install/fastcdr/share/fastcdr/cmake \
         -Dfoonathan_memory_DIR=/home/$USERNAME/ros2_ws/src/fastdds_ws/install/foonathan_memory_vendor/share/foonathan_memory/cmake
```

## Alternative Solutions

### Option 1: Use --cmake-clean-first
Force CMake to reconfigure cleanly:
```dockerfile
&& colcon build --packages-select micro_ros_agent --cmake-clean-first \
```

### Option 2: Build Without ROS's Fast DDS Libraries
Remove Fast DDS from ROS path temporarily:
```dockerfile
RUN cd /home/$USERNAME/ros2_ws \
    && . /opt/ros/jazzy/setup.sh \
    && . src/fastdds_ws/install/setup.sh \
    && mv /opt/ros/jazzy/lib/libfastrtps.so /opt/ros/jazzy/lib/libfastrtps.so.bak \
    && mv /opt/ros/jazzy/lib/libfastcdr.so /opt/ros/jazzy/lib/libfastcdr.so.bak \
    && colcon build --packages-select micro_ros_agent ... \
    && mv /opt/ros/jazzy/lib/libfastrtps.so.bak /opt/ros/jazzy/lib/libfastrtps.so \
    && mv /opt/ros/jazzy/lib/libfastcdr.so.bak /opt/ros/jazzy/lib/libfastcdr.so
```
⚠️ **Risky**: May break other ROS packages during build

### Option 3: Use LD_LIBRARY_PATH Override
Control runtime library loading:
```dockerfile
RUN cd /home/$USERNAME/ros2_ws \
    && . /opt/ros/jazzy/setup.sh \
    && . src/fastdds_ws/install/setup.sh \
    && LD_LIBRARY_PATH="/home/$USERNAME/ros2_ws/src/fastdds_ws/install/lib:$LD_LIBRARY_PATH" \
       colcon build --packages-select micro_ros_agent ...
```

### Option 4: Patch micro-ROS CMakeLists.txt
Modify micro-ROS to not use find_package for Fast DDS:
```dockerfile
RUN cd /home/$USERNAME/ros2_ws/src/micro-ROS-Agent \
    && sed -i 's/find_package(fastrtps REQUIRED)/# find_package(fastrtps REQUIRED)/' CMakeLists.txt \
    && echo 'set(fastrtps_DIR /home/rosuser/ros2_ws/src/fastdds_ws/install/fastrtps/share/fastrtps/cmake)' >> CMakeLists.txt
```
⚠️ **Fragile**: Requires maintaining patches

## Verification

After successful build, verify micro-ROS agent is using correct Fast DDS:

```bash
# Inside container
docker exec -it ros2_gazebo_vnc bash

# Check linked libraries
ldd /home/rosuser/ros2_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent

# Should show:
# libfastrtps.so.2.13 => /home/rosuser/ros2_ws/src/fastdds_ws/install/lib/libfastrtps.so.2.13
# NOT: /opt/ros/jazzy/lib/libfastrtps.so
```

## Runtime Considerations

The `.bashrc` now sources in this order:
1. `/opt/ros/jazzy/setup.bash` - ROS packages
2. `/home/rosuser/ros2_ws/src/fastdds_ws/install/setup.bash` - Custom Fast DDS (overlays)
3. `/home/rosuser/ros2_ws/install/setup.bash` - micro-ROS agent workspace

This ensures:
- ✅ ROS commands work (`ros2`, `rviz2`, etc.)
- ✅ Custom Fast DDS libraries are used (path precedence)
- ✅ micro-ROS agent is available
- ✅ Gazebo integration works

## Related Git History

This issue has been worked on multiple times:
- `46d9f39` - "Fix Fast DDS conflict: build micro-ROS without sourcing ROS"
- `aba2e3b` - "Add complete Fast DDS and micro-ROS agent build to Dockerfile"
- `b3c4a02` - "Remove micro-ROS agent build from Dockerfile"
- `0ab97eb` - "Re-add micro-ROS agent build with proper Fast DDS packages"

The challenge has always been: **How to use custom Fast DDS while still having ROS packages?**

The solution: **Source both, but force CMake to use custom Fast DDS explicitly.**
