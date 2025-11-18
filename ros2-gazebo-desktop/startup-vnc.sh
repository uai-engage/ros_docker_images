#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash

# Set VNC password
mkdir -p ~/.vnc
echo "${VNC_PASSWORD:-rospassword}" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# VNC configuration for XFCE
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS

# Set environment
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
export XDG_SESSION_TYPE=x11
export QT_X11_NO_MITSHM=1
export NO_AT_BRIDGE=1

# Start dbus
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    eval $(dbus-launch --sh-syntax)
    export DBUS_SESSION_BUS_ADDRESS
fi

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Start XFCE desktop
startxfce4 &

# Wait for desktop to initialize properly
sleep 5

# Trust desktop shortcuts (mark as executable and trusted)
if [ -d ~/Desktop ]; then
    for file in ~/Desktop/*.desktop; do
        if [ -f "$file" ]; then
            chmod +x "$file" 2>/dev/null || true
            gio set "$file" metadata::trusted true 2>/dev/null || true
            # Alternative method for older systems
            xattr -w user.xdg.origin.url "file://$file" "$file" 2>/dev/null || true
        fi
    done
fi

# Keep session alive
wait
EOF
chmod +x ~/.vnc/xstartup

# Kill any existing VNC sessions
vncserver -kill :1 2>/dev/null || true

# Clean up old locks
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true

# Start VNC server
echo "Starting VNC server on :1 (port 5901)..."
vncserver :1 \
    -geometry ${VNC_RESOLUTION:-1920x1080} \
    -depth ${VNC_DEPTH:-24} \
    -localhost no \
    -SecurityTypes VncAuth \
    -xstartup ~/.vnc/xstartup

# Wait for VNC to start
sleep 2

# Start noVNC (web-based VNC client)
echo "Starting noVNC on port 6080..."
/usr/share/novnc/utils/novnc_proxy \
    --vnc localhost:5901 \
    --listen 6080 &

echo ""
echo "============================================"
echo "  ROS 2 Gazebo VNC Desktop Started!"
echo "============================================"
echo ""
echo "Connect via:"
echo "  - Web Browser: http://<server-ip>:6080/vnc.html"
echo "  - VNC Client:  <server-ip>:5901"
echo "  - Password:    ${VNC_PASSWORD:-rospassword}"
echo ""
echo "Desktop Features:"
echo "  - Full XFCE desktop environment"
echo "  - Desktop shortcuts for Gazebo, RViz2, RQt"
echo "  - File manager, text editor, browser"
echo ""
echo "ROS 2 Configuration:"
echo "  - ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo "  - RMW: ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo ""
echo "============================================"
echo ""

# Keep container running and show logs
tail -f ~/.vnc/*:1.log
