#!/bin/bash
# Start VNC and noVNC for Ubuntu Desktop container

set -e

echo "============================================"
echo "  Starting Ubuntu Desktop Container"
echo "============================================"
echo ""

# Start D-Bus
sudo service dbus start

# Start VNC server
echo "Starting VNC server on :1 (port 5901)..."
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no

# Start noVNC
echo "Starting noVNC on port 6901..."
/usr/share/novnc/utils/novnc_proxy --vnc localhost:5901 --listen 6901 &

echo ""
echo "============================================"
echo "  Ubuntu Desktop Ready!"
echo "============================================"
echo ""
echo "Access methods:"
echo "  - noVNC (web): http://localhost:6901/vnc.html"
echo "  - VNC client:  localhost:5901"
echo "  - Password:    password"
echo ""
echo "To change VNC password:"
echo "  vncpasswd"
echo ""
echo "Installation scripts available at:"
echo "  ~/installation-scripts/"
echo ""
echo "To install ROS 2 + Gazebo + PX4:"
echo "  ~/installation-scripts/00-check-system.sh"
echo "  ~/installation-scripts/01-install-ros2-jazzy.sh"
echo "  # ... and so on"
echo ""
echo "Or run automated installation:"
echo "  ~/install-all.sh"
echo "============================================"

# Keep container running
tail -f /dev/null
