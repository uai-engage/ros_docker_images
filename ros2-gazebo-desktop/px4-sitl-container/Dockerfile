# PX4 SITL Container with Gazebo Harmonic
# Designed to run alongside ROS 2 Gazebo Desktop container
# Communication: uXRCE-DDS (UDP 8888) + MAVLink (UDP 14550)

FROM ubuntu:24.04

LABEL maintainer="Uraan AI"
LABEL description="PX4 SITL with Gazebo Harmonic for ROS 2 integration"

ARG USERNAME=px4user
ARG USER_UID=1000
ARG USER_GID=1000
ARG PX4_VERSION=v1.16.0

ENV DEBIAN_FRONTEND=noninteractive
ENV PX4_HOME=/home/${USERNAME}/PX4-Autopilot

# Install base dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build essentials
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    curl \
    ca-certificates \
    gnupg \
    lsb-release \
    sudo \
    # Python
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-setuptools \
    python3-wheel \
    python3-packaging \
    python3-toml \
    python3-numpy \
    python3-empy \
    python3-jinja2 \
    python3-yaml \
    python3-jsonschema \
    python3-pyulog \
    # PX4 build dependencies
    astyle \
    exiftool \
    genromfs \
    libxml2-dev \
    libxslt1-dev \
    unzip \
    zip \
    # Networking & tools
    net-tools \
    iputils-ping \
    iproute2 \
    netcat-openbsd \
    socat \
    # For headless Gazebo
    xvfb \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libegl1-mesa \
    libgles2-mesa \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Create user
RUN if getent passwd $USER_UID > /dev/null 2>&1; then \
        userdel -f $(getent passwd $USER_UID | cut -d: -f1); \
    fi \
    && if getent group $USER_GID > /dev/null 2>&1; then \
        groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1); \
    else \
        groupadd --gid $USER_GID $USERNAME; \
    fi \
    && useradd --no-log-init --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install Python packages for PX4
RUN pip3 install --break-system-packages --no-cache-dir \
    kconfiglib \
    pyserial \
    future \
    cerberus \
    pymavlink \
    MAVProxy

# Switch to user
USER $USERNAME
WORKDIR /home/$USERNAME

# Clone PX4-Autopilot (will be overwritten if volume mounted)
# This provides a default if no volume is mounted
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b ${PX4_VERSION} --depth 1 \
    && cd PX4-Autopilot \
    && git submodule update --init --recursive

# Build PX4 SITL with Gazebo Harmonic
# This pre-builds to speed up first run (will rebuild if source is mounted)
RUN cd PX4-Autopilot \
    && make px4_sitl

# Set up environment
RUN echo "# PX4 Environment" >> ~/.bashrc \
    && echo "export PX4_HOME=\$HOME/PX4-Autopilot" >> ~/.bashrc \
    && echo "export PATH=\$PX4_HOME/Tools:\$PATH" >> ~/.bashrc \
    && echo "# Gazebo Harmonic paths" >> ~/.bashrc \
    && echo "export GZ_SIM_RESOURCE_PATH=\$PX4_HOME/Tools/simulation/gz/models:\$PX4_HOME/Tools/simulation/gz/worlds" >> ~/.bashrc \
    && echo "# MAVLink broadcast settings" >> ~/.bashrc \
    && echo "export PX4_SIM_HOST_ADDR=0.0.0.0" >> ~/.bashrc

# Copy startup scripts
USER root
COPY --chown=$USERNAME:$USERNAME scripts/ /home/$USERNAME/scripts/
RUN chmod +x /home/$USERNAME/scripts/*.sh

USER $USERNAME
WORKDIR /home/$USERNAME/PX4-Autopilot

# Default environment variables
ENV PX4_HOME=/home/${USERNAME}/PX4-Autopilot
ENV PX4_SIM_HOST_ADDR=0.0.0.0
ENV PX4_GZ_MODEL=x500
ENV PX4_GZ_WORLD=default
ENV PX4_SYS_AUTOSTART=4001
ENV HEADLESS=1

# Expose ports
# 14550: MAVLink (QGC)
# 14540: MAVLink (MAVROS/offboard)
# 8888: uXRCE-DDS (micro-ROS agent)
# 18570: Gazebo transport (optional)
EXPOSE 14550/udp 14540/udp 8888/udp 18570

# Default command
CMD ["/home/px4user/scripts/start-px4-sitl.sh"]
