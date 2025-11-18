# ROS 2 Jazzy Desktop with VNC and XFCE Desktop
# Full desktop environment for better GUI experience

FROM osrf/ros:jazzy-desktop-full-noble

ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo, XFCE Desktop, and VNC server
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo Harmonic packages
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces \
    # MAVROS2 for ArduPilot communication
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    # XFCE Desktop Environment (full)
    xfce4 \
    xfce4-goodies \
    xfce4-terminal \
    thunar \
    mousepad \
    xfce4-taskmanager \
    # VNC Server
    tigervnc-standalone-server \
    tigervnc-common \
    tigervnc-tools \
    novnc \
    websockify \
    # Additional GUI applications
    firefox \
    gedit \
    nautilus \
    gnome-calculator \
    evince \
    # X11 and rendering
    x11-apps \
    mesa-utils \
    libgl1 \
    libegl1 \
    libgl1-mesa-dri \
    # Development tools
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
    dbus-x11 \
    at-spi2-core \
    # Fast DDS dependencies for micro-ROS agent
    libfastrtps-dev \
    libfastcdr-dev \
    # Fonts
    fonts-dejavu \
    fonts-liberation \
    && rm -rf /var/lib/apt/lists/*

# Create user
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

# Add user to video group
RUN usermod -aG video $USERNAME 2>/dev/null || true

# Install MAVROS geographiclib datasets (required for GPS)
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh || true

# Set up ROS 2 environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/$USERNAME/.bashrc \
    && echo "export XDG_RUNTIME_DIR=/tmp/runtime-\$USER" >> /home/$USERNAME/.bashrc \
    && mkdir -p /tmp/runtime-$USERNAME \
    && chown $USERNAME:$USERNAME /tmp/runtime-$USERNAME \
    && chmod 700 /tmp/runtime-$USERNAME

# Create workspace
RUN mkdir -p /home/$USERNAME/ros2_ws/src \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# Build micro-ROS agent from source for ArduPilot DDS support
RUN cd /home/$USERNAME/ros2_ws \
    && git clone https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws/src/micro-ROS-Agent

USER $USERNAME
RUN cd /home/$USERNAME/ros2_ws \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select micro_ros_agent \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
USER root

# Setup VNC directory
RUN mkdir -p /home/$USERNAME/.vnc /home/$USERNAME/.config \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/.vnc /home/$USERNAME/.config

# Create desktop shortcuts for ROS 2 tools
RUN mkdir -p /home/$USERNAME/Desktop \
    && echo '[Desktop Entry]\nVersion=1.0\nType=Application\nName=Terminal\nExec=xfce4-terminal\nIcon=utilities-terminal\nTerminal=false\nCategories=System;' > /home/$USERNAME/Desktop/terminal.desktop \
    && echo '[Desktop Entry]\nVersion=1.0\nType=Application\nName=Gazebo\nExec=bash -c "source /opt/ros/jazzy/setup.bash && gz sim"\nIcon=gazebo\nTerminal=false\nCategories=Development;' > /home/$USERNAME/Desktop/gazebo.desktop \
    && echo '[Desktop Entry]\nVersion=1.0\nType=Application\nName=RViz2\nExec=bash -c "source /opt/ros/jazzy/setup.bash && rviz2"\nIcon=rviz\nTerminal=false\nCategories=Development;' > /home/$USERNAME/Desktop/rviz2.desktop \
    && echo '[Desktop Entry]\nVersion=1.0\nType=Application\nName=RQt\nExec=bash -c "source /opt/ros/jazzy/setup.bash && rqt"\nIcon=rqt\nTerminal=false\nCategories=Development;' > /home/$USERNAME/Desktop/rqt.desktop \
    && chmod +x /home/$USERNAME/Desktop/*.desktop \
    && chmod 755 /home/$USERNAME/Desktop/*.desktop \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/Desktop

# Create Fast DDS configuration for better network discovery
RUN mkdir -p /home/$USERNAME/.ros \
    && echo '<?xml version="1.0" encoding="UTF-8"?>\n\
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">\n\
  <participant profile_name="participant_profile" is_default_profile="true">\n\
    <rtps>\n\
      <builtin>\n\
        <discovery_config>\n\
          <discoveryProtocol>SIMPLE</discoveryProtocol>\n\
          <leaseDuration>\n\
            <sec>DURATION_INFINITY</sec>\n\
          </leaseDuration>\n\
        </discovery_config>\n\
      </builtin>\n\
    </rtps>\n\
  </participant>\n\
</profiles>' > /home/$USERNAME/.ros/fastdds.xml \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/.ros

# VNC startup script
COPY --chown=$USERNAME:$USERNAME startup-vnc.sh /startup-vnc.sh
RUN chmod +x /startup-vnc.sh

WORKDIR /home/$USERNAME/ros2_ws

USER $USERNAME

ENV USER=$USERNAME
ENV HOME=/home/$USERNAME
ENV ROS_DISTRO=jazzy
ENV SHELL=/bin/bash
ENV DISPLAY=:1

EXPOSE 5901 6080

CMD ["/startup-vnc.sh"]
