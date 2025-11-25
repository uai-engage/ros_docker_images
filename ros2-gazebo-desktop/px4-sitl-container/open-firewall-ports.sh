#!/bin/bash
# Open firewall ports for PX4 MAVLink connections
# Run this on the server to allow remote QGroundControl connections

echo "=========================================="
echo "Opening Firewall Ports for PX4 SITL"
echo "=========================================="
echo ""

# Detect firewall type
if command -v ufw &> /dev/null; then
    echo "Detected: UFW firewall"
    echo ""

    echo "Opening ports:"
    echo "  - TCP 5760  (MAVLink TCP - QGroundControl)"
    echo "  - UDP 14550 (MAVLink UDP - QGroundControl)"
    echo "  - UDP 14540 (MAVLink UDP - MAVROS)"
    echo ""

    sudo ufw allow 5760/tcp comment "PX4 MAVLink TCP"
    sudo ufw allow 14550/udp comment "PX4 MAVLink UDP GCS"
    sudo ufw allow 14540/udp comment "PX4 MAVLink UDP Offboard"

    echo ""
    echo "Firewall rules added successfully!"
    echo ""
    echo "Current UFW status:"
    sudo ufw status | grep -E "(5760|14550|14540|Status)"

elif command -v firewall-cmd &> /dev/null; then
    echo "Detected: firewalld"
    echo ""

    echo "Opening ports:"
    echo "  - TCP 5760  (MAVLink TCP - QGroundControl)"
    echo "  - UDP 14550 (MAVLink UDP - QGroundControl)"
    echo "  - UDP 14540 (MAVLink UDP - MAVROS)"
    echo ""

    sudo firewall-cmd --permanent --add-port=5760/tcp
    sudo firewall-cmd --permanent --add-port=14550/udp
    sudo firewall-cmd --permanent --add-port=14540/udp
    sudo firewall-cmd --reload

    echo ""
    echo "Firewall rules added successfully!"
    echo ""
    echo "Current firewall status:"
    sudo firewall-cmd --list-ports

else
    echo "No firewall detected (ufw/firewalld)"
    echo ""
    echo "If you're using iptables or another firewall, manually open:"
    echo "  - TCP 5760"
    echo "  - UDP 14550"
    echo "  - UDP 14540"
fi

echo ""
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Restart PX4 container: docker compose --env-file .env.example restart"
echo "2. In QGroundControl (Windows):"
echo "   - Add TCP connection"
echo "   - Server: 10.200.10.66"
echo "   - Port: 5760"
echo ""
echo "=========================================="
