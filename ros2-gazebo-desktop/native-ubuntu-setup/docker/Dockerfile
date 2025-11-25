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

# Copy ALL installation scripts and supporting files
COPY --chown=$USERNAME:$USERNAME scripts/ /home/$USERNAME/installation-scripts/
COPY --chown=$USERNAME:$USERNAME docs/ /home/$USERNAME/installation-docs/
COPY --chown=$USERNAME:$USERNAME *.sh /home/$USERNAME/
COPY --chown=$USERNAME:$USERNAME *.md /home/$USERNAME/
COPY --chown=$USERNAME:$USERNAME CONTAINER-README.md /home/$USERNAME/README-START-HERE.txt

# Make all scripts executable
RUN chmod +x /home/$USERNAME/installation-scripts/*.sh \
    && chmod +x /home/$USERNAME/*.sh \
    && find /home/$USERNAME -name "*.sh" -type f -exec chmod +x {} \;

# Create Desktop directory
RUN mkdir -p /home/$USERNAME/Desktop \
    && chown $USERNAME:$USERNAME /home/$USERNAME/Desktop

# Create desktop shortcuts as regular files (symlinks work better for scripts)
RUN ln -s /home/$USERNAME/install-all.sh /home/$USERNAME/Desktop/install-all.sh \
    && ln -s /home/$USERNAME/README-START-HERE.txt /home/$USERNAME/Desktop/README-START-HERE.txt \
    && ln -s /home/$USERNAME/installation-scripts /home/$USERNAME/Desktop/installation-scripts \
    && ln -s /home/$USERNAME/installation-docs /home/$USERNAME/Desktop/docs

# Create a desktop file for automated installation
RUN echo '[Desktop Entry]' > /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && echo 'Version=1.0' >> /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && echo 'Type=Application' >> /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && echo 'Name=Install ROS 2 + Gazebo + PX4' >> /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && echo 'Comment=Automated installation of all components' >> /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && echo 'Exec=xfce4-terminal -e "/home/'$USERNAME'/install-all.sh"' >> /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && echo 'Icon=system-software-install' >> /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && echo 'Terminal=true' >> /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && echo 'Categories=Development;' >> /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && chmod +x /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop \
    && chown $USERNAME:$USERNAME /home/$USERNAME/Desktop/Install-ROS2-Gazebo-PX4.desktop

# Verify files are present
RUN echo "Verifying copied files..." \
    && ls -la /home/$USERNAME/installation-scripts/ \
    && ls -la /home/$USERNAME/*.sh \
    && echo "All scripts copied successfully!"

# Expose VNC and noVNC ports
EXPOSE 5901 6901

# Switch to user
USER $USERNAME
WORKDIR /home/$USERNAME

# Default command
CMD ["/home/developer/start-desktop.sh"]
