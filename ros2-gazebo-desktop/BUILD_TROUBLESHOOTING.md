# Build Troubleshooting Guide

## Recent Build Issues (November 2025)

If you're experiencing build errors with micro-ROS agent, this guide provides solutions.

### Issue 1: CMAKE Target Conflicts
**Error:**
```
CMake Error: Some (but not all) targets were already defined.
  Targets Defined: eProsima_atomic
  Targets not yet defined: fastrtps
```

**Cause:** Conflict between custom-built Fast DDS and system Fast DDS from ROS 2.

**Solution:** Fixed in latest Dockerfile - sources ROS 2 first, then overlays custom Fast DDS.

---

### Issue 2: Missing ament_cmake
**Error:**
```
CMake Error: Could not find a package configuration file provided by "ament_cmake"
```

**Cause:** Build process couldn't find ROS 2 build tools.

**Solution:** Fixed by explicitly setting CMAKE_PREFIX_PATH with both custom Fast DDS and ROS 2.

---

## Current Build Strategy (Dockerfile lines 127-135)

```dockerfile
# Build micro-ROS agent as user (use custom Fast DDS, overlaying ROS 2)
USER $USERNAME
RUN cd /home/$USERNAME/ros2_ws \
    && . /opt/ros/jazzy/setup.sh \
    && . src/fastdds_ws/install/setup.sh \
    && colcon build --packages-select micro_ros_agent \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_PREFIX_PATH="/home/$USERNAME/ros2_ws/src/fastdds_ws/install;/opt/ros/jazzy"
```

**How it works:**
1. Sources ROS 2 Jazzy (provides ament_cmake and build tools)
2. Sources custom Fast DDS (overlays with custom build)
3. Explicitly sets CMAKE_PREFIX_PATH to prioritize custom Fast DDS
4. ROS 2 is still accessible for ament_cmake, but Fast DDS comes from custom build

---

## Alternative Solution: Pin to Specific Commit

If the current build still fails, you can pin micro-ROS-Agent to a known-good commit.

### Option A: Pin to a specific commit (recommended for production)

**Edit Dockerfile line 124:**

```dockerfile
# Current (uses latest commit)
RUN cd /home/$USERNAME/ros2_ws/src \
    && git clone https://github.com/micro-ROS/micro-ROS-Agent.git \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws/src/micro-ROS-Agent

# Alternative (pin to specific commit)
RUN cd /home/$USERNAME/ros2_ws/src \
    && git clone https://github.com/micro-ROS/micro-ROS-Agent.git \
    && cd micro-ROS-Agent \
    && git checkout 155cfaa  \
    && cd .. \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws/src/micro-ROS-Agent
```

Replace `155cfaa` with a specific commit hash that you know works.

### Option B: Use a specific branch for ROS 2 Jazzy

```dockerfile
RUN cd /home/$USERNAME/ros2_ws/src \
    && git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws/src/micro-ROS-Agent
```

(Note: Check if a `jazzy` branch exists first)

---

## Debugging Build Issues

### 1. Check what changed upstream

```bash
# Check micro-ROS-Agent latest commit
git ls-remote https://github.com/micro-ROS/micro-ROS-Agent.git HEAD

# Check base image update date
docker pull osrf/ros:jazzy-desktop-full-noble
docker inspect osrf/ros:jazzy-desktop-full-noble | grep Created
```

### 2. Build with verbose output

```bash
cd ros2-gazebo-desktop
docker compose --env-file .env.vnc build --progress=plain --no-cache
```

### 3. Test build manually inside container

```bash
# Start a temporary container with base image
docker run -it --rm osrf/ros:jazzy-desktop-full-noble bash

# Inside container, try building Fast DDS and micro-ROS agent manually
# This helps isolate the issue
```

### 4. Check environment variables during build

Add debug output to Dockerfile before the failing step:

```dockerfile
RUN cd /home/$USERNAME/ros2_ws \
    && . /opt/ros/jazzy/setup.sh \
    && . src/fastdds_ws/install/setup.sh \
    && echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH" \
    && echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH" \
    && colcon build --packages-select micro_ros_agent ...
```

---

## Known Working Configurations

### Configuration A (November 20, 2025)
- Base: `osrf/ros:jazzy-desktop-full-noble`
- micro-ROS-Agent: commit `155cfaa` (Release v3.0.1)
- Fast DDS: Built from source (latest main branch)
- Build: Successful

### Configuration B (November 21, 2025 - Updated)
- Base: `osrf/ros:jazzy-desktop-full-noble`
- micro-ROS-Agent: latest main branch
- Fast DDS: Built from source
- Build: Using explicit CMAKE_PREFIX_PATH

---

## If All Else Fails

### Nuclear Option: Use a pre-built image

If builds continue to fail, consider:

1. **Build once successfully** on a machine
2. **Save the image**: `docker save ros2-gazebo-desktop:jazzy-vnc | gzip > ros2-jazebo-jazzy.tar.gz`
3. **Load on target machines**: `docker load < ros2-gazebo-jazzy.tar.gz`

### Report the issue

If none of these solutions work, open an issue:
- [micro-ROS-Agent issues](https://github.com/micro-ROS/micro-ROS-Agent/issues)
- Include: Docker build log, base image version, date of build attempt

---

## Quick Build Test

Try building with verbose output to see exactly where it fails:

```bash
cd ros2-gazebo-desktop

# Clean build with full output
docker compose --env-file .env.vnc build --no-cache --progress=plain 2>&1 | tee build.log

# Check the log for the exact error
grep -A 20 "Error" build.log
```

---

## Prevention: Pin Versions

For production deployments, always pin versions:

```dockerfile
# Pin base image to specific digest
FROM osrf/ros:jazzy-desktop-full-noble@sha256:abc123...

# Pin git repos to specific commits
RUN git clone https://github.com/eProsima/Fast-DDS.git \
    && cd Fast-DDS \
    && git checkout v2.10.0

# Pin micro-ROS-Agent
RUN git clone https://github.com/micro-ROS/micro-ROS-Agent.git \
    && cd micro-ROS-Agent \
    && git checkout 155cfaa
```

This ensures reproducible builds even if upstream repositories change.
