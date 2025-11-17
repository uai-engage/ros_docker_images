# ROS 2 Jazzy Desktop with Gazebo Harmonic for GUI Development
# Based on osrf/ros:jazzy-desktop-full-noble (Ubuntu 24.04)

FROM osrf/ros:jazzy-desktop-full-noble

# Arguments for user configuration
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Harmonic (compatible with ROS 2 Jazzy) and GUI dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo Harmonic packages
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces \
    # X11 and GUI support
    x11-apps \
    mesa-utils \
    libgl1 \
    libegl1 \
    libgl1-mesa-dri \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxkbcommon-x11-0 \
    libxcb-randr0 \
    libxcb-shape0 \
    # Additional development tools
    python3-pip \
    python3-argcomplete \
    git \
    wget \
    curl \
    vim \
    nano \
    htop \
    net-tools \
    iputils-ping \
    terminator \
    dbus-x11 \
    # Audio support (optional for Gazebo)
    pulseaudio \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user with sudo privileges
# Handle case where GID/UID already exists in base image (e.g., ubuntu user)
RUN apt-get update \
    && apt-get install -y sudo \
    && if getent passwd $USER_UID > /dev/null 2>&1; then \
         userdel -f $(getent passwd $USER_UID | cut -d: -f1); \
       fi \
    && if getent group $USER_GID > /dev/null 2>&1; then \
         groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1); \
       else \
         groupadd --gid $USER_GID $USERNAME; \
       fi \
    && useradd --no-log-init --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS 2 environment for the user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc

# Create workspace directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# Set working directory
WORKDIR /home/$USERNAME/ros2_ws

# Switch to non-root user
USER $USERNAME

# Set environment variables
ENV USER=$USERNAME
ENV HOME=/home/$USERNAME
ENV ROS_DISTRO=jazzy
ENV SHELL=/bin/bash

# Copy custom entrypoint
COPY --chown=$USERNAME:$USERNAME ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
