#!/bin/bash
# Start VNC and noVNC for Ubuntu Desktop container

# Don't exit on errors - we want to continue even if some services fail
set +e

echo "============================================"
echo "  Starting Ubuntu Desktop Container"
echo "============================================"
echo ""

# Fix hostname resolution
HOSTNAME=$(hostname)
if ! grep -q "$HOSTNAME" /etc/hosts; then
    echo "127.0.0.1 $HOSTNAME" | sudo tee -a /etc/hosts > /dev/null
fi

# Fix px4_workspace permissions (mounted volume may be owned by root)
if [ -d "$HOME/px4_workspace" ]; then
    WORKSPACE_OWNER=$(stat -c '%U' "$HOME/px4_workspace")
    if [ "$WORKSPACE_OWNER" != "$(whoami)" ]; then
        echo "Fixing px4_workspace permissions..."
        sudo chown -R $(whoami):$(whoami) "$HOME/px4_workspace"
    fi
fi

# Start D-Bus
sudo service dbus start

# Clean up any existing VNC sessions
vncserver -kill :1 2>/dev/null || true

# Start VNC server
echo "Starting VNC server on :1 (port 5901)..."
if vncserver :1 -geometry 1920x1080 -depth 24 -localhost no; then
    echo "✅ VNC server started successfully"
else
    echo "❌ Failed to start VNC server"
    echo "Checking for errors..."
    cat ~/.vnc/*.log 2>/dev/null | tail -20 || echo "No VNC logs found"
    exit 1
fi

# Wait a moment for VNC to be ready
sleep 2

# Start noVNC
echo "Starting noVNC on port 6901..."
/usr/share/novnc/utils/novnc_proxy --vnc localhost:5901 --listen 6901 &

if [ $? -eq 0 ]; then
    echo "✅ noVNC started successfully"
else
    echo "⚠️  noVNC may have issues, but continuing..."
fi

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
echo "============================================"
echo "  Installation Scripts Available"
echo "============================================"
echo ""
echo "All scripts are pre-loaded and ready to use!"
echo ""
echo "Option 1: AUTOMATED INSTALLATION (Recommended)"
echo "  ~/install-all.sh"
echo "  OR double-click 'install-all.sh' on Desktop"
echo ""
echo "Option 2: MANUAL STEP-BY-STEP"
echo "  ~/installation-scripts/00-check-system.sh"
echo "  ~/installation-scripts/01-install-ros2-jazzy.sh"
echo "  ~/installation-scripts/02-install-gazebo-harmonic.sh"
echo "  ~/installation-scripts/03-install-px4-deps.sh"
echo "  ~/installation-scripts/04-setup-px4.sh"
echo "  ~/installation-scripts/05-install-microros.sh"
echo ""
echo "Then configure environment:"
echo "  ~/configure-environment.sh"
echo "  source ~/.bashrc"
echo ""
echo "Desktop shortcuts created for easy access!"
echo ""
echo "Documentation:"
echo "  ~/installation-docs/INSTALLATION-GUIDE.md"
echo "  ~/installation-docs/USAGE-GUIDE.md"
echo "  ~/QUICK-REFERENCE.md"
echo "============================================"

# Keep container running
tail -f /dev/null
