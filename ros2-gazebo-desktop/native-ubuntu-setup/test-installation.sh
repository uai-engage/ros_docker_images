#!/bin/bash
# Test Installation
# Verifies all components are installed and working

set -e

echo "==========================================="
echo "  Testing Installation"
echo "==========================================="
echo ""

ERRORS=0

# Test ROS 2
echo "1. Testing ROS 2 Jazzy..."
if command -v ros2 &> /dev/null; then
    ROS_VERSION=$(ros2 --version 2>&1)
    echo "   ✅ $ROS_VERSION"
else
    echo "   ❌ ROS 2 not found in PATH"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Test Gazebo
echo "2. Testing Gazebo Harmonic..."
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version 2>&1 | head -1)
    echo "   ✅ $GZ_VERSION"
else
    echo "   ❌ Gazebo not found in PATH"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Test PX4
echo "3. Testing PX4 Autopilot..."
PX4_HOME=~/px4_workspace/PX4-Autopilot
if [ -f "$PX4_HOME/build/px4_sitl_default/bin/px4" ]; then
    PX4_VERSION=$(cd $PX4_HOME && git describe --tags 2>/dev/null || echo "unknown")
    echo "   ✅ PX4 built at $PX4_HOME"
    echo "   ✅ Version: $PX4_VERSION"
else
    echo "   ❌ PX4 not built at $PX4_HOME"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Test micro-ROS
echo "4. Testing micro-ROS Agent..."
MICROROS_WS=~/px4_workspace/micro_ros_ws
if [ -f "$MICROROS_WS/install/setup.bash" ]; then
    echo "   ✅ micro-ROS Agent built"
    source $MICROROS_WS/install/setup.bash
    if ros2 pkg list | grep -q micro_ros_agent; then
        echo "   ✅ micro_ros_agent package found"
    else
        echo "   ⚠️  micro_ros_agent package not found in ROS 2"
    fi
else
    echo "   ❌ micro-ROS Agent not built"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Test Python packages
echo "5. Testing Python packages..."
python3 -c "import pymavlink" 2>/dev/null && \
    echo "   ✅ pymavlink installed" || \
    echo "   ❌ pymavlink missing"
python3 -c "import numpy" 2>/dev/null && \
    echo "   ✅ numpy installed" || \
    echo "   ❌ numpy missing"
python3 -c "import jinja2" 2>/dev/null && \
    echo "   ✅ jinja2 installed" || \
    echo "   ❌ jinja2 missing"
echo ""

# Test Gazebo models
echo "6. Testing PX4 Gazebo models..."
if [ -d "$PX4_HOME/Tools/simulation/gz/models" ]; then
    MODEL_COUNT=$(ls -1 $PX4_HOME/Tools/simulation/gz/models | wc -l)
    echo "   ✅ Found $MODEL_COUNT Gazebo models"
else
    echo "   ❌ Gazebo models directory not found"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Summary
echo "==========================================="
echo "  Test Summary"
echo "==========================================="
if [ $ERRORS -eq 0 ]; then
    echo "✅ All tests passed!"
    echo ""
    echo "Your installation is complete and ready to use."
    echo ""
    echo "Quick start:"
    echo "  Terminal 1: gz-default     (Start Gazebo)"
    echo "  Terminal 2: px4-sitl       (Start PX4)"
    echo "  Terminal 3: microros       (Start micro-ROS agent)"
    echo "  Terminal 4: px4-topics     (List ROS 2 topics)"
    echo ""
    echo "See full usage guide:"
    echo "  cat docs/USAGE-GUIDE.md"
else
    echo "❌ $ERRORS test(s) failed"
    echo ""
    echo "Please review the errors above and:"
    echo "  - Re-run the failed installation scripts"
    echo "  - Check docs/TROUBLESHOOTING.md"
fi
echo "==========================================="
