#!/bin/bash
# Run multiple PX4 SITL instances
# Usage: ./start-multi-instance.sh [num_instances]

NUM_INSTANCES=${1:-1}
BASE_INSTANCE=${PX4_SIM_INSTANCE:-0}

echo "Starting ${NUM_INSTANCES} PX4 SITL instance(s)..."

cd ${PX4_HOME:-/home/px4user/PX4-Autopilot}

# Start Xvfb if headless
if [ "${HEADLESS:-1}" = "1" ]; then
    Xvfb :99 -screen 0 1920x1080x24 &
    export DISPLAY=:99
    sleep 2
fi

for i in $(seq 0 $((NUM_INSTANCES - 1))); do
    INSTANCE=$((BASE_INSTANCE + i))
    
    # Calculate ports for this instance
    # MAVLink: 14550 + instance, 14540 + instance
    # uXRCE-DDS: 8888 + instance
    
    echo "Starting instance ${INSTANCE}..."
    echo "  MAVLink GCS: UDP $((14550 + INSTANCE))"
    echo "  MAVLink API: UDP $((14540 + INSTANCE))"
    echo "  uXRCE-DDS:   UDP $((8888 + INSTANCE))"
    
    PX4_SIM_INSTANCE=${INSTANCE} \
    PX4_GZ_MODEL_POSE="${i},${i},0,0,0,0" \
    make px4_sitl gz_${PX4_GZ_MODEL:-x500} &
    
    sleep 5  # Wait between instances
done

echo ""
echo "All instances started. Press Ctrl+C to stop."
wait
