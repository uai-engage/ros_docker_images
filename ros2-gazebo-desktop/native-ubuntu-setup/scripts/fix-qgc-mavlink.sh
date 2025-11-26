#!/bin/bash
# Fix QGroundControl MAVLink Communication with PX4 SITL
# Resolves "Vehicle did not respond to request for parameters" error

set -e

echo "==========================================="
echo "  QGroundControl MAVLink Communication Fix"
echo "==========================================="
echo ""
echo "This script will:"
echo "  1. Configure PX4 MAVLink broadcasting"
echo "  2. Create proper startup configuration"
echo "  3. Set up connection parameters"
echo "  4. Test the configuration"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Fix cancelled"
    exit 1
fi

PX4_DIR=~/px4_workspace/PX4-Autopilot

if [ ! -d "$PX4_DIR" ]; then
    echo "❌ PX4 directory not found: $PX4_DIR"
    exit 1
fi

echo ""
echo "Step 1/5: Stopping any running PX4 instances..."
pkill -f px4 || true
sleep 2

echo ""
echo "Step 2/5: Creating PX4 MAVLink configuration..."

# Create PX4 ROMFS override directory
mkdir -p "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"

# Create custom airframe configuration with MAVLink broadcasting enabled
cat > "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/1047_gz_x500" << 'EOF'
#!/bin/sh
#
# @name Gazebo x500 Quadcopter (with MAVLink Broadcasting)
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

# Enable MAVLink broadcasting for QGroundControl
param set-default MAV_0_BROADCAST 1
param set-default MAV_1_BROADCAST 1

# Set MAVLink modes
param set-default MAV_0_MODE 0       # Normal mode
param set-default MAV_1_MODE 2       # Onboard mode

# Set MAVLink rates (increase for better responsiveness)
param set-default MAV_0_RATE 1200000
param set-default MAV_1_RATE 1200000

# Disable forwarding to avoid loops
param set-default MAV_0_FORWARD 0
param set-default MAV_1_FORWARD 0

# Configure control allocation for quadcopter
param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

# Motor positions for X configuration
param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.22
param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.05
param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.22
param set-default CA_ROTOR3_KM -0.05

# Enable Gazebo simulation
param set-default SIM_GZ_EN 1

# MAVLink system ID
param set-default MAV_SYS_ID 1
param set-default MAV_COMP_ID 1
EOF

chmod +x "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/1047_gz_x500"

echo "   ✅ Custom airframe configuration created"

echo ""
echo "Step 3/5: Creating MAVLink parameter file..."

# Create parameters file
PARAMS_FILE="$HOME/.ros/etc/init.d-posix/px4-rc.params"
mkdir -p "$(dirname "$PARAMS_FILE")"

cat > "$PARAMS_FILE" << 'EOF'
# MAVLink Configuration for QGroundControl
MAV_0_BROADCAST 1
MAV_1_BROADCAST 1
MAV_0_MODE 0
MAV_1_MODE 2
MAV_0_RATE 1200000
MAV_1_RATE 1200000
MAV_0_FORWARD 0
MAV_1_FORWARD 0
MAV_SYS_ID 1
MAV_COMP_ID 1
EOF

echo "   ✅ Parameter file created at: $PARAMS_FILE"

echo ""
echo "Step 4/5: Updating PX4 alias with proper configuration..."

# Update the px4-sitl alias in bashrc
if grep -q "alias px4-sitl=" ~/.bashrc; then
    # Replace existing alias
    sed -i '/alias px4-sitl=/c\alias px4-sitl="cd ~/px4_workspace/PX4-Autopilot && make px4_sitl gz_x500"' ~/.bashrc
    echo "   ✅ Updated px4-sitl alias"
else
    # Add new alias
    echo 'alias px4-sitl="cd ~/px4_workspace/PX4-Autopilot && make px4_sitl gz_x500"' >> ~/.bashrc
    echo "   ✅ Added px4-sitl alias"
fi

echo ""
echo "Step 5/5: Creating PX4 startup helper script..."

# Create a startup script with proper parameters
cat > "$HOME/start-px4-qgc.sh" << 'EOF'
#!/bin/bash
# Start PX4 SITL with QGroundControl-compatible configuration

echo "Starting PX4 SITL with MAVLink broadcasting enabled..."
echo ""

cd ~/px4_workspace/PX4-Autopilot

# Set environment variables
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_MODEL=x500

# Start PX4 with custom airframe
make px4_sitl gz_x500

# When PX4 starts, it will automatically load the configuration
# from ROMFS/px4fmu_common/init.d-posix/airframes/1047_gz_x500
EOF

chmod +x "$HOME/start-px4-qgc.sh"

echo "   ✅ Startup helper created at: ~/start-px4-qgc.sh"

echo ""
echo "==========================================="
echo "  ✅ MAVLink Configuration Complete!"
echo "==========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Start Gazebo (in Terminal 1):"
echo "   gz-default"
echo ""
echo "2. Start PX4 SITL (in Terminal 2):"
echo "   ~/start-px4-qgc.sh"
echo "   OR"
echo "   px4-sitl"
echo ""
echo "3. Wait for PX4 to fully start (you'll see 'pxh>' prompt)"
echo ""
echo "4. In QGroundControl:"
echo "   - Click Q icon (top-left)"
echo "   - Go to: Application Settings → Comm Links"
echo "   - Check for UDP connection on port 14550"
echo "   - If not present, add new connection:"
echo "     * Type: UDP"
echo "     * Listening Port: 14550"
echo "     * Click OK and Connect"
echo ""
echo "5. (Optional) Start micro-ROS agent (in Terminal 3):"
echo "   microros"
echo ""
echo "Expected result:"
echo "  - PX4 console shows: 'INFO [mavlink] partner IP: 127.0.0.1'"
echo "  - No 'localhost only' or 'command_ack lost' errors"
echo "  - QGroundControl displays vehicle parameters"
echo "  - Vehicle status shows 'Ready to Fly' (after calibration)"
echo ""
echo "If still having issues:"
echo "  - At PX4 pxh> prompt, run: mavlink status"
echo "  - Check QGC Comm Links settings"
echo "  - Ensure no firewall blocking port 14550"
echo ""
echo "==========================================="
