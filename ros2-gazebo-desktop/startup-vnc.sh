#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash

# Set VNC password
mkdir -p ~/.vnc
echo "${VNC_PASSWORD:-rospassword}" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# VNC configuration
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
export QT_X11_NO_MITSHM=1
export NO_AT_BRIDGE=1

# Start dbus
eval $(dbus-launch --sh-syntax)

# Start window manager
openbox-session &

# Wait for window manager
sleep 2

# Keep session alive
wait
EOF
chmod +x ~/.vnc/xstartup

# Kill any existing VNC sessions
vncserver -kill :1 2>/dev/null || true

# Start VNC server
echo "Starting VNC server on :1 (port 5901)..."
vncserver :1 \
    -geometry ${VNC_RESOLUTION:-1920x1080} \
    -depth ${VNC_DEPTH:-24} \
    -localhost no \
    -SecurityTypes VncAuth

# Start noVNC (web-based VNC client)
echo "Starting noVNC on port 6080..."
/usr/share/novnc/utils/novnc_proxy \
    --vnc localhost:5901 \
    --listen 6080 &

echo ""
echo "============================================"
echo "VNC Server started!"
echo "============================================"
echo "Connect via:"
echo "  - VNC Client: <server-ip>:5901"
echo "  - Web Browser: http://<server-ip>:6080/vnc.html"
echo "  - Password: ${VNC_PASSWORD:-rospassword}"
echo ""
echo "Inside VNC, open terminal and run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  gz sim shapes.sdf"
echo "============================================"
echo ""

# Keep container running
tail -f /home/$USER/.vnc/*:1.log
