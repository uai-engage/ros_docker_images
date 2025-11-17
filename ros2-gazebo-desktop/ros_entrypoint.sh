#!/bin/bash
set -e

# Setup ROS 2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source workspace if it exists
if [ -f "/home/$USER/ros2_ws/install/setup.bash" ]; then
    source "/home/$USER/ros2_ws/install/setup.bash"
fi

# Fix for GUI applications in Docker
export QT_X11_NO_MITSHM=1
export NO_AT_BRIDGE=1

# Ensure dbus is available for GUI apps
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    eval $(dbus-launch --sh-syntax)
fi

exec "$@"
