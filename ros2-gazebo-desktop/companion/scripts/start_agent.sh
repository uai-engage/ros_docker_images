#!/bin/bash
# start_agent.sh
# Helper script for starting micro-ROS agent with various configurations

set -e

echo "==========================================="
echo "  micro-ROS Agent Startup Script"
echo "==========================================="
echo ""

# Configuration
WORKSPACE_PATH="${WORKSPACE_PATH:-/opt/micro_ros_agent_ws}"
SERIAL_DEVICE="${SERIAL_DEVICE:-/dev/ttyUSB0}"
BAUD_RATE="${BAUD_RATE:-921600}"
UDP_PORT="${UDP_PORT:-2019}"
CONNECTION_MODE="${CONNECTION_MODE:-serial}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
VERBOSE="${VERBOSE:-false}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -m, --mode MODE           Connection mode: serial, udp4, udp6, tcp4, tcp6 (default: serial)"
    echo "  -d, --device DEVICE       Serial device path (default: /dev/ttyUSB0)"
    echo "  -b, --baud RATE           Serial baud rate (default: 921600)"
    echo "  -p, --port PORT           UDP/TCP port (default: 2019)"
    echo "  -w, --workspace PATH      Workspace path (default: /opt/micro_ros_agent_ws)"
    echo "  -v, --verbose             Enable verbose logging"
    echo "  -h, --help                Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Serial mode with defaults"
    echo "  $0 -m udp4 -p 2019                   # UDP mode on port 2019"
    echo "  $0 -d /dev/ttyACM0 -b 115200         # Serial with custom device/baud"
    echo "  $0 -m serial -d /dev/ttyUSB0 -v      # Serial with verbose logging"
    echo ""
    echo "Environment Variables:"
    echo "  CONNECTION_MODE    Connection mode (serial, udp4, etc.)"
    echo "  SERIAL_DEVICE      Serial device path"
    echo "  BAUD_RATE          Serial baud rate"
    echo "  UDP_PORT           UDP/TCP port"
    echo "  WORKSPACE_PATH     micro-ROS workspace path"
    echo "  ROS_DOMAIN_ID      ROS 2 Domain ID (default: 0)"
    echo "  VERBOSE            Enable verbose mode (true/false)"
    exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -m|--mode)
            CONNECTION_MODE="$2"
            shift 2
            ;;
        -d|--device)
            SERIAL_DEVICE="$2"
            shift 2
            ;;
        -b|--baud)
            BAUD_RATE="$2"
            shift 2
            ;;
        -p|--port)
            UDP_PORT="$2"
            shift 2
            ;;
        -w|--workspace)
            WORKSPACE_PATH="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            show_usage
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            ;;
    esac
done

# Check if micro-ROS agent is already running
if pgrep -f micro_ros_agent > /dev/null; then
    print_warn "micro-ROS agent is already running!"
    echo ""
    ps aux | grep micro_ros_agent | grep -v grep
    echo ""
    read -p "Do you want to stop it and restart? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Stopping existing micro-ROS agent..."
        pkill -f micro_ros_agent
        sleep 2

        # Clear DDS cache
        print_info "Clearing ROS 2 DDS cache..."
        ros2 daemon stop 2>/dev/null || true
        sleep 1
        ros2 daemon start 2>/dev/null || true
        sleep 1
    else
        print_info "Exiting without changes"
        exit 0
    fi
fi

# Source ROS environment
print_info "Sourcing ROS 2 environment..."
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    print_error "ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    exit 1
fi
source /opt/ros/jazzy/setup.bash

# Source workspace
print_info "Sourcing micro-ROS workspace: $WORKSPACE_PATH"
if [ ! -f "$WORKSPACE_PATH/install/setup.bash" ]; then
    print_error "Workspace not found: $WORKSPACE_PATH/install/setup.bash"
    print_error "Have you built the micro-ROS agent?"
    exit 1
fi
source "$WORKSPACE_PATH/install/setup.bash"

# Verify micro_ros_agent is available
if ! command -v ros2 &> /dev/null; then
    print_error "ros2 command not found"
    exit 1
fi

# Build command based on connection mode
print_info "Configuration:"
echo "  Connection mode: $CONNECTION_MODE"
echo "  ROS Domain ID:   $ROS_DOMAIN_ID"

AGENT_CMD="ros2 run micro_ros_agent micro_ros_agent"

case $CONNECTION_MODE in
    serial)
        # Check if serial device exists
        if [ ! -e "$SERIAL_DEVICE" ]; then
            print_error "Serial device not found: $SERIAL_DEVICE"
            echo ""
            echo "Available serial devices:"
            ls -l /dev/tty* 2>/dev/null | grep -E "ttyUSB|ttyACM|ttyAMA" || echo "  (none found)"
            exit 1
        fi

        # Check permissions
        if [ ! -r "$SERIAL_DEVICE" ] || [ ! -w "$SERIAL_DEVICE" ]; then
            print_warn "Serial device permissions may be insufficient"
            print_info "Try: sudo chmod 666 $SERIAL_DEVICE"
            print_info "Or:  sudo usermod -aG dialout $USER"
        fi

        echo "  Serial device:   $SERIAL_DEVICE"
        echo "  Baud rate:       $BAUD_RATE"
        AGENT_CMD="$AGENT_CMD serial --dev $SERIAL_DEVICE -b $BAUD_RATE"
        ;;

    udp4|udp6|tcp4|tcp6)
        echo "  Port:            $UDP_PORT"
        AGENT_CMD="$AGENT_CMD $CONNECTION_MODE --port $UDP_PORT"
        ;;

    *)
        print_error "Invalid connection mode: $CONNECTION_MODE"
        print_error "Valid modes: serial, udp4, udp6, tcp4, tcp6"
        exit 1
        ;;
esac

# Add verbose flag if requested
if [ "$VERBOSE" = true ]; then
    AGENT_CMD="$AGENT_CMD -v6"
    echo "  Verbose logging: enabled"
fi

echo ""
print_info "Starting micro-ROS agent..."
echo ""
echo "Command: $AGENT_CMD"
echo ""
echo "Press Ctrl+C to stop"
echo ""
echo "==========================================="
echo ""

# Start the agent
exec $AGENT_CMD
