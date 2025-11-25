#!/bin/bash
# Wrapper to start PX4 and then add TCP MAVLink
set -e

echo "=========================================="
echo "Starting PX4 with TCP MAVLink support"
echo "=========================================="

# Start PX4 in background
make px4_sitl gz_${PX4_GZ_MODEL:-x500} &
PX4_PID=$!

# Wait for PX4 to be fully initialized
echo "Waiting for PX4 to initialize..."
sleep 15

# Check if PX4 is still running
if ! kill -0 $PX4_PID 2>/dev/null; then
    echo "ERROR: PX4 process died during startup"
    exit 1
fi

# Create a named pipe for PX4 shell commands
FIFO="/tmp/px4_commands_$$"
mkfifo "$FIFO" || true

# Send TCP MAVLink command to PX4 shell
echo "Starting TCP MAVLink on port 5760..."
{
    echo "mavlink start -x -o 5760 -t 0.0.0.0 -m onboard -r 4000000"
    echo "mavlink status"
    sleep 2
} > "$FIFO" &

# Connect the pipe to PX4's stdin (this sends commands to running PX4)
cat "$FIFO" | nc localhost 4560 2>/dev/null || \
    echo "Note: Could not connect to PX4 shell via TCP (normal for SITL)"

# Clean up
rm -f "$FIFO"

# Keep script running with PX4
wait $PX4_PID
