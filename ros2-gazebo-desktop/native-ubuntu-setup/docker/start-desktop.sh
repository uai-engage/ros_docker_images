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
