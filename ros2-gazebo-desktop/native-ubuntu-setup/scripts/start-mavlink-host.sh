#!/bin/bash
# Start MAVLink connection to QGroundControl on Windows Host
# Auto-detects host IP address

echo "==========================================="
echo "  MAVLink Connection to Windows Host"
echo "==========================================="
echo ""

# Try to detect Windows host IP
# Method 1: Check default gateway (usually the host in Docker)
HOST_IP=$(ip route | grep default | awk '{print $3}')

if [ -z "$HOST_IP" ]; then
    # Method 2: Try to get from /etc/resolv.conf
    HOST_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}' | head -1)
fi

if [ -z "$HOST_IP" ]; then
    # Method 3: Manual entry
    echo "❌ Could not auto-detect Windows host IP"
    echo ""
    read -p "Enter Windows host IP address: " HOST_IP
fi

echo "Detected/Using Windows host IP: $HOST_IP"
echo ""
echo "This will configure MAVLink to send data to QGroundControl"
echo "running on your Windows host machine at: $HOST_IP:14550"
echo ""
read -p "Is this correct? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    read -p "Enter correct IP address: " HOST_IP
fi

echo ""
echo "Configuring MAVLink to target: $HOST_IP:14550"
echo ""

# Create command file for easy copy-paste
cat > /tmp/mavlink_command.txt << EOF
mavlink start -x -u 14550 -t $HOST_IP -r 4000000
EOF

echo "==========================================="
echo "  MAVLink Command Ready"
echo "==========================================="
echo ""
echo "Copy and paste this command at the PX4 pxh> prompt:"
echo ""
echo "  mavlink start -x -u 14550 -t $HOST_IP -r 4000000"
echo ""
echo "Or simply run:"
echo "  cat /tmp/mavlink_command.txt"
echo ""
echo "Then start QGroundControl on your Windows host."
echo "It should auto-connect via UDP port 14550."
echo ""
echo "==========================================="
