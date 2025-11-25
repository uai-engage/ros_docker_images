# Ubuntu 24.04 Desktop in Docker with VNC
# For running ROS 2 + Gazebo + PX4 natively inside a container

FROM ubuntu:24.04

LABEL maintainer="Uraan AI"
LABEL description="Ubuntu 24.04 Desktop with VNC for native ROS 2 + Gazebo + PX4"

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Create user (same UID/GID as host for volume permissions)
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

# Set timezone
ENV TZ=UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install Ubuntu Desktop essentials + VNC
RUN apt-get update && apt-get install -y \
    # Desktop environment (lightweight XFCE)
    xfce4 \
    xfce4-goodies \
    # VNC server
    tigervnc-standalone-server \
    tigervnc-common \
    # noVNC (web-based VNC client)
    novnc \
    websockify \
    # X11 utilities
    x11-utils \
    x11-xserver-utils \
    dbus-x11 \
    # Fonts
    fonts-liberation \
    fonts-dejavu \
    # Terminal
    xfce4-terminal \
    # File manager
    thunar \
    # Basic tools
    sudo \
    curl \
    wget \
    git \
    nano \
    vim \
    net-tools \
    iputils-ping \
    ca-certificates \
    # Browser (optional, for viewing docs)
    firefox \
    && rm -rf /var/lib/apt/lists/*

# Create user with sudo privileges
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up VNC
USER $USERNAME
WORKDIR /home/$USERNAME

# Create VNC password file
RUN mkdir -p ~/.vnc && \
    echo "password" | vncpasswd -f > ~/.vnc/passwd && \
    chmod 600 ~/.vnc/passwd

# VNC startup script
RUN echo '#!/bin/bash' > ~/.vnc/xstartup && \
    echo 'unset SESSION_MANAGER' >> ~/.vnc/xstartup && \
    echo 'unset DBUS_SESSION_BUS_ADDRESS' >> ~/.vnc/xstartup && \
    echo 'exec startxfce4' >> ~/.vnc/xstartup && \
    chmod +x ~/.vnc/xstartup

# Create startup script
USER root
COPY --chown=$USERNAME:$USERNAME docker/start-desktop.sh /home/$USERNAME/
RUN chmod +x /home/$USERNAME/start-desktop.sh

# Copy installation scripts
COPY --chown=$USERNAME:$USERNAME scripts/ /home/$USERNAME/installation-scripts/
COPY --chown=$USERNAME:$USERNAME docs/ /home/$USERNAME/installation-docs/
COPY --chown=$USERNAME:$USERNAME *.sh /home/$USERNAME/
COPY --chown=$USERNAME:$USERNAME *.md /home/$USERNAME/

RUN chmod +x /home/$USERNAME/installation-scripts/*.sh \
    && chmod +x /home/$USERNAME/*.sh

# Expose VNC and noVNC ports
EXPOSE 5901 6901

# Switch to user
USER $USERNAME
WORKDIR /home/$USERNAME

# Default command
CMD ["/home/developer/start-desktop.sh"]
