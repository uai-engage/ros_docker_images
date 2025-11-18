# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is the **OSRF Docker Images** repository that hosts official Docker images for ROS (1 and 2) and Gazebo simulator. The repository provides:
- Official Library images for Docker Hub (`ros`, `gazebo`)
- OSRF Organization images with extended features (`osrf/ros`, `osrf/ros2`, `osrf/gazebo`)
- Automated Dockerfile generation from YAML templates
- CI/CD workflows for building and maintaining images

## Project Structure

### Key Directories

- **`ros/`** - ROS 1 and ROS 2 Dockerfiles organized by: `{distro}/{os}/{os_version}/`
  - Contains `platform.yaml` files defining OS and ROS version metadata
  - Generated Dockerfiles for each image variant (ros-core, ros-base, robot, perception, desktop, desktop-full)

- **`gazebo/`** - Gazebo simulator Dockerfiles organized by: `{version}/{os}/{os_version}/`
  - Contains libraries and simulation server/client split configurations

- **`ros2/`** - Specialized ROS 2 development images
  - `testing/` - Testing apt repo images for CI
  - `nightly/` - Nightly builds from ci.ros2.org
  - `source/` - Source build images with dependencies

- **`ros2-gazebo-desktop/`** - Custom ROS 2 Jazzy + Gazebo Harmonic environment with VNC
  - Full XFCE desktop environment for GUI applications
  - Includes micro-ROS agent built with custom Fast DDS to avoid conflicts
  - Uses host network mode for ROS 2 DDS discovery

- **`docker/`** - Dockerfile generation tooling container
  - Python image with dependencies for running generation scripts

- **`.ci/`** - CI scripts and requirements
- **`doc/`** - Documentation for maintenance and generation processes

### Configuration Files

- **`manifest.yaml`** - Defines all release names, OS combinations, architectures, and templates for bulk generation
- **`platform.yaml`** - Per-distro metadata (OS name/version, ROS version, architecture, package versions)
- **`images.yaml.em`** - EmPy template defining image hierarchy and ROS packages to install
- **`.config/`** - Template files for different ROS/Gazebo configurations

## Dockerfile Generation System

This repository uses a **template-based generation system** rather than manually maintaining Dockerfiles. Understanding this is critical for modifications.

### Generation Workflow

1. **Templates** are stored in the external `osrf/docker_templates` repository
2. **Configuration** comes from local `platform.yaml` and `images.yaml.em` files
3. **Python scripts** process templates with EmPy to generate final Dockerfiles

### Core Scripts

- **`create_dockerfiles.py`** - Generates Dockerfiles from platform + images config
  ```bash
  # Explicit mode
  ./create_dockerfiles.py explicit -p platform.yaml -i images.yaml.em -o output/

  # Directory mode (auto-detects platform.yaml and images.yaml.em)
  ./create_dockerfiles.py dir -d humble/ubuntu/jammy/
  ```

- **`create_dockerfolders.py`** - Bulk generation from manifest for entire distro
  ```bash
  ./create_dockerfolders.py -m manifest.yaml -o .
  ```

- **`create_dockerlibrary.py`** - Generates library manifest for Docker Official Images

### Using the Generation Container

The `docker/` directory provides a container with all generation dependencies:

```bash
cd docker
make build
./run.sh bash

# Inside container
cd ros
./create_dockerfiles.py dir -d humble/ubuntu/jammy/
exit
```

## Common Development Commands

### Building Official Images

```bash
# Build specific ROS 2 distro
cd ros/humble/ubuntu/jammy/ros-core
docker build -t ros:humble-ros-core .

# Build with compose in ros2-gazebo-desktop
cd ros2-gazebo-desktop
docker compose build
docker compose up -d
```

### Regenerating Dockerfiles

**Important:** Always regenerate Dockerfiles using scripts, not manual edits.

```bash
# For a specific ROS distro
cd ros
./create_dockerfiles.py dir -d humble/ubuntu/jammy/

# For Gazebo
cd gazebo
./create_dockerfiles.py dir -d 11/ubuntu/focal/

# For ROS 2 development images
cd ros2
./create_dockerfiles.py dir -d testing/
```

### Testing Locally

```bash
# Test a specific image builds
cd ros/humble/ubuntu/jammy/ros-base
docker build -t test:humble-ros-base .

# Test with VNC setup
cd ros2-gazebo-desktop
cp .env.vnc .env
# Edit .env to set VNC_PASSWORD
docker compose build
docker compose up -d
# Connect to http://localhost:6080/vnc.html
```

### Running CI Checks

```bash
# Install requirements
pip install -r docker/requirements.txt
pip install -r .ci/requirements.txt

# Run CI script (example)
python .ci/ci_script.py
```

## Architecture Details

### Image Hierarchy

Official ROS images follow a layered approach for efficient caching:

1. **ros-core** - Minimal ROS installation (communication libraries only)
2. **ros-base** - ros-core + basic robot tools (action/service clients)
3. **robot** - ros-base + robot-specific packages
4. **perception** - ros-base + perception/sensors
5. **desktop** - robot + perception + visualization tools (RViz, rqt)
6. **desktop-full** - desktop + simulators and additional development tools

Each layer builds `FROM` the previous, creating a Russian-doll structure.

### Package Version Pinning

- ROS package versions are **explicitly pinned** to ensure reproducible builds
- The `ros_buildfarm.common` package fetches the latest package index
- Gazebo versions are manually specified in `platform.yaml`
- This follows Docker Official Images repeatability guidelines

### Multi-Architecture Support

Images support multiple architectures:
- **amd64** (x86_64) - Primary
- **arm32v7** (ARMv7) - For ARM 32-bit systems
- **arm64v8** (AArch64) - For ARM 64-bit systems

### ROS 2 Networking in ros2-gazebo-desktop

The custom setup uses **host network mode** to enable DDS discovery:
- Fast DDS built from source (in separate workspace at `src/fastdds_ws/`)
- micro-ROS agent built using the custom Fast DDS to avoid conflicts with ROS 2's bundled Fast DDS
- Configuration in `/home/rosuser/.ros/fastdds.xml` sets discovery protocol
- `ROS_DOMAIN_ID` environment variable isolates multiple robot systems
- Supports communication with external ROS 2 nodes on the same network

## Important Constraints

### When Modifying Dockerfiles

1. **Never edit generated Dockerfiles directly** - Changes will be overwritten
2. **Edit the templates** in `osrf/docker_templates` repository instead
3. **Or modify** the `platform.yaml` / `images.yaml.em` config files
4. **Regenerate** using `create_dockerfiles.py` after template/config changes

### Package Version Updates

- Official images track the **latest** available package versions from apt
- Update procedure for ROS:
  1. Regenerate Dockerfiles (scripts auto-fetch latest versions)
  2. Test builds locally
  3. Commit to master branch
  4. Create PR to `docker-library/official-images` with new commit SHA
- Gazebo versions must be manually updated in `platform.yaml`

### Official Library Requirements

When updating images for Docker Official Library:
- Must follow [Official Images guidelines](https://github.com/docker-library/official-images)
- Maintain repeatability (pinned versions)
- Update both `osrf/docker_images` and `docker-library/official-images` repos
- Documentation in `docker-library/docs` if text changes needed

## ROS 2 Gazebo Desktop Setup

The `ros2-gazebo-desktop/` directory provides a complete development environment:

### Key Components
- **Base Image:** `osrf/ros:jazzy-desktop-full-noble`
- **Desktop:** XFCE with pre-configured shortcuts
- **VNC Server:** TigerVNC on port 5901
- **Web Access:** noVNC on port 6080
- **Simulation:** Gazebo Harmonic + MAVROS2 for ArduPilot
- **Workspace:** `/home/rosuser/ros2_ws` (mounted from host)

### Environment Variables
Configure via `.env` file (copy from `.env.vnc`):
- `VNC_PASSWORD` - VNC access password (change default!)
- `VNC_RESOLUTION` - Screen resolution (default 1920x1080)
- `ROS_DOMAIN_ID` - DDS domain for node isolation (default 0)
- `RMW_IMPLEMENTATION` - DDS implementation (default rmw_fastrtps_cpp)

### Startup Process
The `startup-vnc.sh` script:
1. Configures VNC password
2. Starts X server with specified resolution
3. Launches XFCE window manager
4. Starts noVNC websocket proxy
5. Sources ROS 2 environment

## Testing and CI

### GitHub Actions Workflows

- **`ros_ci.yaml`** - Builds ROS rolling, kilted, jazzy, humble images daily
- **`ros2_ci.yaml`** - Tests ROS 2 development images
- **`_gazebo_ci.yaml`** - Validates Gazebo images
- **`trigger_*.yaml`** - Automated rebuild triggers on package updates

### Local Testing Strategy

1. Generate Dockerfiles for target distro
2. Build base image first (ros-core)
3. Build dependent images in order
4. Test with sample ROS packages/launch files
5. Verify no broken dependencies or missing packages

## Key Git Workflow Notes

- **Main branch:** `master`
- Recent commits show micro-ROS agent integration with custom Fast DDS builds
- Feature branches follow pattern: `claude/feature-name-{id}`
- CI runs on PR and push to validate image builds
