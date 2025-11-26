#!/bin/bash
# Generate Docker Compose for Multi-Vehicle PX4 SITL Simulation
#
# Usage:
#   ./generate-compose.sh                    # Interactive mode
#   ./generate-compose.sh --copters 2        # 2 x500 quadcopters
#   ./generate-compose.sh --copters 2 --vtols 1  # 2 copters + 1 VTOL
#   ./generate-compose.sh --config vehicles.yaml  # From config file
#
# Generated file: docker-compose-generated.yml

set -e

# Default values
COPTERS=0
VTOLS=0
PLANES=0
ROVERS=0
BOATS=0
OUTPUT_FILE="docker-compose-generated.yml"
WINDOWS_HOST_IP=""

# Vehicle configurations
# Models available in PX4 v1.16.0 with Gazebo Harmonic
declare -A VEHICLE_MODELS
VEHICLE_MODELS[copter]="x500"           # Standard quadcopter
VEHICLE_MODELS[vtol]="standard_vtol"    # Standard VTOL (quad + fixed wing)
VEHICLE_MODELS[plane]="rc_cessna"       # Fixed-wing RC plane
VEHICLE_MODELS[rover]="r1_rover"        # Differential drive rover
VEHICLE_MODELS[boat]="boat"             # Surface vessel / boat

declare -A VEHICLE_AUTOSTART
VEHICLE_AUTOSTART[copter]="4001"        # Generic Quadcopter
VEHICLE_AUTOSTART[vtol]="13000"         # Generic Standard VTOL
VEHICLE_AUTOSTART[plane]="2106"         # Generic Fixed Wing
VEHICLE_AUTOSTART[rover]="50000"        # Generic Rover (Ackermann)
VEHICLE_AUTOSTART[boat]="60000"         # Boat (Surface Vessel)

declare -A VEHICLE_DESCRIPTION
VEHICLE_DESCRIPTION[copter]="Quadcopter (x500)"
VEHICLE_DESCRIPTION[vtol]="VTOL (standard_vtol)"
VEHICLE_DESCRIPTION[plane]="Fixed-wing Plane (rc_cessna)"
VEHICLE_DESCRIPTION[rover]="Ground Rover (r1_rover)"
VEHICLE_DESCRIPTION[boat]="Boat / Surface Vessel"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}"
    echo "============================================"
    echo "  PX4 Multi-Vehicle Docker Compose Generator"
    echo "============================================"
    echo -e "${NC}"
}

print_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Vehicle Options:"
    echo "  --copters N       Number of quadcopters (x500)"
    echo "  --vtols N         Number of VTOLs (standard_vtol)"
    echo "  --planes N        Number of fixed-wing planes (rc_cessna)"
    echo "  --rovers N        Number of ground rovers (r1_rover)"
    echo "  --boats N         Number of boats/surface vessels"
    echo ""
    echo "Configuration Options:"
    echo "  --output FILE     Output file (default: docker-compose-generated.yml)"
    echo "  --host-ip IP      Windows host IP for QGroundControl"
    echo "  --interactive     Interactive mode (prompts for input)"
    echo "  --list-models     List all available vehicle models"
    echo "  --help            Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 --copters 2                        # 2 quadcopters"
    echo "  $0 --copters 2 --vtols 1              # 2 copters + 1 VTOL"
    echo "  $0 --copters 2 --rovers 1             # 2 copters + 1 rover"
    echo "  $0 --copters 1 --planes 1 --rovers 1  # Mixed fleet"
    echo "  $0 --copters 3 --host-ip 192.168.1.100"
    echo "  $0 --interactive                      # Prompted input"
    echo ""
}

list_models() {
    echo ""
    echo "Available Vehicle Models:"
    echo "========================="
    echo ""
    echo "  Type      Model           Autostart   Description"
    echo "  ----      -----           ---------   -----------"
    echo "  copter    x500            4001        Standard quadcopter"
    echo "  vtol      standard_vtol   13000       Quad + fixed wing VTOL"
    echo "  plane     rc_cessna       2106        Fixed-wing RC plane"
    echo "  rover     r1_rover        50000       Differential drive rover"
    echo "  boat      boat            60000       Surface vessel / boat"
    echo ""
    echo "Note: Model availability depends on your PX4-Autopilot version"
    echo "      and Gazebo model files."
    echo ""
}

interactive_mode() {
    print_header

    echo "How many vehicles of each type?"
    echo ""
    echo "Air Vehicles:"
    echo "-------------"

    read -p "  Quadcopters (x500) [0]: " input
    COPTERS=${input:-0}

    read -p "  VTOLs (standard_vtol) [0]: " input
    VTOLS=${input:-0}

    read -p "  Fixed-wing Planes (rc_cessna) [0]: " input
    PLANES=${input:-0}

    echo ""
    echo "Ground/Water Vehicles:"
    echo "----------------------"

    read -p "  Ground Rovers (r1_rover) [0]: " input
    ROVERS=${input:-0}

    read -p "  Boats/Surface Vessels [0]: " input
    BOATS=${input:-0}

    echo ""
    echo "Configuration:"
    echo "--------------"

    read -p "  Windows Host IP (leave blank for auto-detect): " input
    WINDOWS_HOST_IP=${input:-}

    read -p "  Output file [docker-compose-generated.yml]: " input
    OUTPUT_FILE=${input:-docker-compose-generated.yml}
}

# Parse command line arguments
INTERACTIVE=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --copters)
            COPTERS="$2"
            shift 2
            ;;
        --vtols)
            VTOLS="$2"
            shift 2
            ;;
        --planes)
            PLANES="$2"
            shift 2
            ;;
        --rovers)
            ROVERS="$2"
            shift 2
            ;;
        --boats)
            BOATS="$2"
            shift 2
            ;;
        --output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --host-ip)
            WINDOWS_HOST_IP="$2"
            shift 2
            ;;
        --interactive)
            INTERACTIVE=true
            shift
            ;;
        --list-models)
            list_models
            exit 0
            ;;
        --help)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

# Interactive mode if no arguments or --interactive flag
if [ $INTERACTIVE = true ] || [ $((COPTERS + VTOLS + PLANES + ROVERS + BOATS)) -eq 0 ]; then
    interactive_mode
fi

TOTAL_VEHICLES=$((COPTERS + VTOLS + PLANES + ROVERS + BOATS))

if [ $TOTAL_VEHICLES -eq 0 ]; then
    echo -e "${RED}Error: At least one vehicle is required!${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Generating Docker Compose for:${NC}"
echo ""
echo "  Air Vehicles:"
echo "    - Quadcopters: $COPTERS"
echo "    - VTOLs:       $VTOLS"
echo "    - Planes:      $PLANES"
echo ""
echo "  Ground/Water:"
echo "    - Rovers:      $ROVERS"
echo "    - Boats:       $BOATS"
echo ""
echo "  Total:           $TOTAL_VEHICLES vehicles"
echo ""

# Start generating the compose file
cat > "$OUTPUT_FILE" << 'HEADER'
# Auto-generated Multi-Vehicle PX4 SITL Docker Compose
# Generated by generate-compose.sh
#
# Usage:
#   docker compose -f docker-compose-generated.yml up -d
#   docker compose -f docker-compose-generated.yml logs -f
#   docker compose -f docker-compose-generated.yml down

x-px4-common: &px4-common
  build:
    context: .
    dockerfile: ${DOCKERFILE:-Dockerfile.light}
    args:
      USERNAME: ${USERNAME:-px4user}
      USER_UID: ${USER_UID:-1000}
      USER_GID: ${USER_GID:-1000}
      PX4_VERSION: ${PX4_VERSION:-v1.16.0}
  stdin_open: true
  tty: true
  volumes:
    - ${PX4_SOURCE_PATH:-./PX4-Autopilot}:/home/${USERNAME:-px4user}/PX4-Autopilot:rw
    - /dev/shm:/dev/shm
  network_mode: host
  shm_size: '2gb'
  ipc: host
  deploy:
    resources:
      limits:
        memory: ${MEMORY_LIMIT:-8G}
      reservations:
        memory: ${MEMORY_RESERVATION:-4G}
  working_dir: /home/${USERNAME:-px4user}/PX4-Autopilot

services:
HEADER

# Track instance numbers
INSTANCE=0
PREV_SERVICE=""

# Function to add a vehicle service
add_vehicle() {
    local TYPE=$1
    local INDEX=$2
    local MODEL=${VEHICLE_MODELS[$TYPE]}
    local AUTOSTART=${VEHICLE_AUTOSTART[$TYPE]}
    local MAVLINK_PORT=$((14550 + INSTANCE))
    local UXRCE_PORT=$((8888 + INSTANCE))
    local SERVICE_NAME="px4-${TYPE}-${INDEX}"

    # Calculate spawn position (spread vehicles apart)
    local X_POS=$((INSTANCE * 3))
    local Y_POS=0

    cat >> "$OUTPUT_FILE" << EOF
  # ==========================================
  # ${TYPE^} ${INDEX} (Instance ${INSTANCE})
  # Model: ${MODEL}_${INSTANCE}
  # MAVLink: ${MAVLINK_PORT}, DDS: ${UXRCE_PORT}
  # ==========================================
  ${SERVICE_NAME}:
    <<: *px4-common
    image: px4-sitl:\${PX4_VERSION:-v1.16.0}-${TYPE}-${INDEX}
    container_name: px4_${TYPE}_${INDEX}
    environment:
      - PX4_GZ_MODEL=${MODEL}
      - PX4_GZ_WORLD=\${PX4_GZ_WORLD:-default}
      - PX4_SYS_AUTOSTART=${AUTOSTART}
      - PX4_SIM_INSTANCE=${INSTANCE}
      - PX4_SIM_HOST_ADDR=\${PX4_SIM_HOST_ADDR:-0.0.0.0}
      - EXTERNAL_GAZEBO=1
      - PX4_GZ_STANDALONE=1
      - HEADLESS=1
      - GZ_PARTITION=\${GZ_PARTITION:-gazebo}
      - GZ_IP=\${GZ_IP:-127.0.0.1}
      - GZ_RELAY=1
      - UXRCE_DDS_AG_IP=\${UXRCE_DDS_AG_IP:-127.0.0.1}
      - UXRCE_DDS_PRT=${UXRCE_PORT}
      - WINDOWS_HOST_IP=\${WINDOWS_HOST_IP:-${WINDOWS_HOST_IP}}
      - MAVLINK_UDP_PORT=${MAVLINK_PORT}
      - PX4_GZ_MODEL_POSE="${X_POS},${Y_POS},0,0,0,0"
    command: /home/\${USERNAME:-px4user}/scripts/start-px4-sitl.sh
    healthcheck:
      test: ["CMD", "pgrep", "-f", "px4"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 60s
EOF

    # Add dependency on previous service (for sequential startup)
    if [ -n "$PREV_SERVICE" ]; then
        cat >> "$OUTPUT_FILE" << EOF
    depends_on:
      ${PREV_SERVICE}:
        condition: service_healthy
EOF
    fi

    echo "" >> "$OUTPUT_FILE"

    PREV_SERVICE=$SERVICE_NAME
    INSTANCE=$((INSTANCE + 1))
}

# Generate copter services
for i in $(seq 0 $((COPTERS - 1))); do
    add_vehicle "copter" $i
done

# Generate VTOL services
for i in $(seq 0 $((VTOLS - 1))); do
    add_vehicle "vtol" $i
done

# Generate plane services
for i in $(seq 0 $((PLANES - 1))); do
    add_vehicle "plane" $i
done

# Generate rover services
for i in $(seq 0 $((ROVERS - 1))); do
    add_vehicle "rover" $i
done

# Generate boat services
for i in $(seq 0 $((BOATS - 1))); do
    add_vehicle "boat" $i
done

# Add usage comments at the end
cat >> "$OUTPUT_FILE" << EOF
# ==========================================
# Vehicle Summary
# ==========================================
#
EOF

# Reset instance counter for summary
INSTANCE=0

# Air vehicles
if [ $COPTERS -gt 0 ] || [ $VTOLS -gt 0 ] || [ $PLANES -gt 0 ]; then
    echo "#   Air Vehicles:" >> "$OUTPUT_FILE"
fi

for i in $(seq 0 $((COPTERS - 1))); do
    MAVLINK_PORT=$((14550 + INSTANCE))
    UXRCE_PORT=$((8888 + INSTANCE))
    echo "#     Copter $i:  MAVLink ${MAVLINK_PORT}, DDS ${UXRCE_PORT}, Model x500_${INSTANCE}" >> "$OUTPUT_FILE"
    INSTANCE=$((INSTANCE + 1))
done

for i in $(seq 0 $((VTOLS - 1))); do
    MAVLINK_PORT=$((14550 + INSTANCE))
    UXRCE_PORT=$((8888 + INSTANCE))
    echo "#     VTOL $i:    MAVLink ${MAVLINK_PORT}, DDS ${UXRCE_PORT}, Model standard_vtol_${INSTANCE}" >> "$OUTPUT_FILE"
    INSTANCE=$((INSTANCE + 1))
done

for i in $(seq 0 $((PLANES - 1))); do
    MAVLINK_PORT=$((14550 + INSTANCE))
    UXRCE_PORT=$((8888 + INSTANCE))
    echo "#     Plane $i:   MAVLink ${MAVLINK_PORT}, DDS ${UXRCE_PORT}, Model rc_cessna_${INSTANCE}" >> "$OUTPUT_FILE"
    INSTANCE=$((INSTANCE + 1))
done

# Ground/Water vehicles
if [ $ROVERS -gt 0 ] || [ $BOATS -gt 0 ]; then
    echo "#   Ground/Water Vehicles:" >> "$OUTPUT_FILE"
fi

for i in $(seq 0 $((ROVERS - 1))); do
    MAVLINK_PORT=$((14550 + INSTANCE))
    UXRCE_PORT=$((8888 + INSTANCE))
    echo "#     Rover $i:   MAVLink ${MAVLINK_PORT}, DDS ${UXRCE_PORT}, Model r1_rover_${INSTANCE}" >> "$OUTPUT_FILE"
    INSTANCE=$((INSTANCE + 1))
done

for i in $(seq 0 $((BOATS - 1))); do
    MAVLINK_PORT=$((14550 + INSTANCE))
    UXRCE_PORT=$((8888 + INSTANCE))
    echo "#     Boat $i:    MAVLink ${MAVLINK_PORT}, DDS ${UXRCE_PORT}, Model boat_${INSTANCE}" >> "$OUTPUT_FILE"
    INSTANCE=$((INSTANCE + 1))
done

cat >> "$OUTPUT_FILE" << 'FOOTER'
#
# ==========================================
# Quick Start
# ==========================================
#
# 1. Start ROS2 container with Gazebo:
#    cd ../ros2-gazebo-desktop && docker compose up -d
#    # In VNC: gz sim -r default.sdf
#
# 2. Start micro-ROS agents (one per vehicle):
FOOTER

# Add micro-ROS commands
INSTANCE=0
for i in $(seq 0 $((TOTAL_VEHICLES - 1))); do
    UXRCE_PORT=$((8888 + i))
    echo "#    ros2 run micro_ros_agent micro_ros_agent udp4 -p ${UXRCE_PORT} &" >> "$OUTPUT_FILE"
done

cat >> "$OUTPUT_FILE" << 'FOOTER2'
#
# 3. Start all vehicles:
#    docker compose -f docker-compose-generated.yml up -d
#
# 4. Connect QGroundControl:
#    Add UDP connections for each vehicle's MAVLink port
#
# ==========================================
FOOTER2

echo -e "${GREEN}✅ Generated: ${OUTPUT_FILE}${NC}"
echo ""
echo "To start the simulation:"
echo ""
echo "  1. Start Gazebo:"
echo "     gz sim -r default.sdf"
echo ""
echo "  2. Start micro-ROS agents:"
INSTANCE=0
for i in $(seq 0 $((TOTAL_VEHICLES - 1))); do
    UXRCE_PORT=$((8888 + i))
    echo "     ros2 run micro_ros_agent micro_ros_agent udp4 -p ${UXRCE_PORT} &"
done
echo ""
echo "  3. Start PX4 vehicles:"
echo "     docker compose -f ${OUTPUT_FILE} up -d"
echo ""
echo "  4. View logs:"
echo "     docker compose -f ${OUTPUT_FILE} logs -f"
echo ""
