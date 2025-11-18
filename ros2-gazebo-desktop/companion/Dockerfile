# Minimal ROS 2 Base + micro-ROS agent for companion computer
# This is a lightweight image for drone companion computers
# Size: ~800 MB (vs 5.5 GB for full desktop)
#
# Use this for:
# - Raspberry Pi / Jetson on drone
# - Serial connection to flight controller
# - Radio link to ground station
#
# Build: docker build -f Dockerfile.companion -t micro-ros-agent:minimal .
# Run: See docker-compose.companion.yml

FROM ros:jazzy-ros-base-noble

# Install only essential packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # micro-ROS dependencies
    ros-jazzy-micro-ros-msgs \
    ros-jazzy-micro-ros-diagnostic-msgs \
    ros-jazzy-micro-ros-diagnostic-bridge \
    # Network tools for debugging
    net-tools \
    iputils-ping \
    # Serial port tools
    udev \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
RUN mkdir -p /opt/micro_ros_agent_ws/src

# Clone micro-ROS agent
RUN cd /opt/micro_ros_agent_ws/src \
    && git clone https://github.com/micro-ROS/micro-ROS-Agent.git \
    && cd micro-ROS-Agent \
    && git checkout jazzy

# Build micro-ROS agent
RUN cd /opt/micro_ros_agent_ws \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select micro_ros_agent --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /opt/micro_ros_agent_ws/install/setup.bash" >> /root/.bashrc

# Expose default micro-ROS agent port (for UDP mode)
EXPOSE 2019/udp

# Default command
CMD ["/bin/bash"]
