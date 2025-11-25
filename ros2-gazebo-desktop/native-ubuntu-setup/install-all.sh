#!/bin/bash
# Automated Installation of ROS 2 + Gazebo + PX4
# Runs all installation scripts in sequence

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
INSTALL_SCRIPTS_DIR="${SCRIPT_DIR}/installation-scripts"

# Check if running in container
if [ -f "/.dockerenv" ]; then
    IN_CONTAINER=true
    echo "Detected running inside Docker container"
else
    IN_CONTAINER=false
    INSTALL_SCRIPTS_DIR="${SCRIPT_DIR}/scripts"
fi

echo "============================================"
echo "  Automated ROS 2 + Gazebo + PX4 Installation"
echo "============================================"
echo ""
echo "This will install:"
echo "  1. ROS 2 Jazzy (15-20 min)"
echo "  2. Gazebo Harmonic (10-15 min)"
echo "  3. PX4 Dependencies (5-10 min)"
echo "  4. PX4 Autopilot v1.16.0 (20-30 min)"
echo "  5. micro-ROS Agent (5 min)"
echo ""
echo "Total time: ~1-1.5 hours"
echo ""
echo "Installation will proceed automatically."
echo "You can monitor progress, but no interaction needed."
echo ""

# Check if running in automated mode (from Docker or with AUTO_INSTALL env var)
if [ -z "$AUTO_INSTALL" ] && [ ! -f "/.dockerenv" ]; then
    read -p "Continue? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Installation cancelled"
        exit 1
    fi
else
    echo "Running in automated mode..."
    echo "Waiting 5 seconds before starting (Ctrl+C to cancel)..."
    sleep 5
fi

# Log file
LOG_FILE="${SCRIPT_DIR}/installation.log"
echo "Logging to: $LOG_FILE"
echo ""

# Function to run script with logging
run_script() {
    local script=$1
    local name=$2

    echo ""
    echo "========================================"
    echo "  $name"
    echo "========================================"
    echo ""

    if [ -f "$script" ]; then
        # Run script, accepting all prompts automatically
        echo "Running: $script"

        # If in container or auto mode, pipe 'y' to all prompts
        if [ -f "/.dockerenv" ] || [ -n "$AUTO_INSTALL" ]; then
            yes 'y' | bash "$script" 2>&1 | tee -a "$LOG_FILE"
        else
            bash "$script" 2>&1 | tee -a "$LOG_FILE"
        fi

        if [ ${PIPESTATUS[0]} -eq 0 ]; then
            echo "✅ $name completed successfully"
        else
            echo "❌ $name failed"
            echo "Check log: $LOG_FILE"
            exit 1
        fi
    else
        echo "❌ Script not found: $script"
        exit 1
    fi
}

# Create log file
echo "Installation started: $(date)" > "$LOG_FILE"
echo "========================================" >> "$LOG_FILE"

# Run installation scripts
run_script "${INSTALL_SCRIPTS_DIR}/00-check-system.sh" "System Requirements Check"
run_script "${INSTALL_SCRIPTS_DIR}/01-install-ros2-jazzy.sh" "ROS 2 Jazzy Installation"
run_script "${INSTALL_SCRIPTS_DIR}/02-install-gazebo-harmonic.sh" "Gazebo Harmonic Installation"
run_script "${INSTALL_SCRIPTS_DIR}/03-install-px4-deps.sh" "PX4 Dependencies Installation"
run_script "${INSTALL_SCRIPTS_DIR}/04-setup-px4.sh" "PX4 Autopilot Setup"
run_script "${INSTALL_SCRIPTS_DIR}/05-install-microros.sh" "micro-ROS Agent Installation"

# Configure environment
echo ""
echo "========================================"
echo "  Configuring Environment"
echo "========================================"
echo ""

if [ -f "${SCRIPT_DIR}/configure-environment.sh" ]; then
    bash "${SCRIPT_DIR}/configure-environment.sh" 2>&1 | tee -a "$LOG_FILE"
fi

# Source new environment
if [ -f "$HOME/.bashrc" ]; then
    echo "Sourcing ~/.bashrc..."
    source "$HOME/.bashrc"
fi

# Run tests
echo ""
echo "========================================"
echo "  Testing Installation"
echo "========================================"
echo ""

if [ -f "${SCRIPT_DIR}/test-installation.sh" ]; then
    bash "${SCRIPT_DIR}/test-installation.sh" 2>&1 | tee -a "$LOG_FILE"
fi

# Summary
echo ""
echo "============================================"
echo "  🎉 Installation Complete!"
echo "============================================"
echo ""
echo "Installation log saved to: $LOG_FILE"
echo ""
echo "Next steps:"
echo "  1. Open new terminal (or source ~/.bashrc)"
echo "  2. Start using the system:"
echo ""
echo "     Terminal 1: gz-default    # Start Gazebo"
echo "     Terminal 2: px4-sitl      # Start PX4"
echo "     Terminal 3: microros      # Start micro-ROS agent"
echo "     Terminal 4: px4-topics    # Monitor ROS 2 topics"
echo ""
echo "  3. Connect QGroundControl:"
if [ "$IN_CONTAINER" = true ]; then
    echo "     Type: TCP"
    echo "     Server: localhost (from host machine)"
    echo "     Port: 5760"
else
    echo "     Should auto-discover on UDP 14550"
fi
echo ""
echo "See usage guide:"
if [ "$IN_CONTAINER" = true ]; then
    echo "  cat ~/installation-docs/USAGE-GUIDE.md"
else
    echo "  cat docs/USAGE-GUIDE.md"
fi
echo ""
echo "============================================"
