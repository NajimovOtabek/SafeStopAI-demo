#!/bin/bash
# run_demo.sh

# Function to cleanup background processes on exit
cleanup() {
    echo ""
    echo "Stopping all processes..."
    kill $(jobs -p) 2>/dev/null
    exit
}

# Trap Ctrl+C (SIGINT) and call cleanup
trap cleanup SIGINT

echo "=================================================="
echo "  Vehicle SafeStop AI - Demo Launcher"
echo "=================================================="

# 1. Check for ROS2 and Auto-Source
# Allow passing path as argument: ./run_demo.sh /path/to/setup.zsh
if [ ! -z "$1" ]; then
    echo "Sourcing provided setup file: $1"
    source "$1"
fi

# Check for RoboStack 'ros_env' and activate it if we are not already in it
if [[ "$CONDA_DEFAULT_ENV" != "ros_env" ]]; then
    # Try to find conda
    if command -v conda &> /dev/null; then
        if conda env list | grep -q "ros_env"; then
            echo "Found 'ros_env' Conda environment. Activating..."
            # Need to source conda.sh to use 'conda activate' in script
            # Try common locations
            CONDA_BASE=$(conda info --base)
            if [ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
                source "$CONDA_BASE/etc/profile.d/conda.sh"
                conda activate ros_env
            fi
        fi
    fi
fi

if ! command -v ros2 &> /dev/null; then
    echo "WARN: 'ros2' command not found. Attempting to source..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "Found ROS2 Humble. Sourcing..."
        source /opt/ros/humble/setup.bash
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        echo "Found ROS2 Foxy. Sourcing..."
        source /opt/ros/foxy/setup.bash
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        echo "Found ROS2 Iron. Sourcing..."
        source /opt/ros/iron/setup.bash
    else
        echo "ERROR: Could not find ROS2 installation automatically."
        echo "----------------------------------------------------------------"
        echo "It seems you do not have ROS2 installed in your current environment."
        echo "Since you are on macOS with Conda, we recommend installing via RoboStack:"
        echo ""
        echo "  conda create -n ros_env python=3.10 -y"
        echo "  conda activate ros_env"
        echo "  conda config --env --add channels conda-forge"
        echo "  conda config --env --add channels robostack-staging"
        echo "  conda config --env --remove channels defaults"
        echo "  conda install ros-humble-desktop ros-humble-rosbridge-suite -y"
        echo ""
        echo "After installing, run this script again!"
        echo "----------------------------------------------------------------"
        exit 1
    fi
fi

# 1.5 Check for Colcon (Build Tool)
if ! command -v colcon &> /dev/null; then
    echo "WARN: 'colcon' command not found."
    echo "Attempting to install colcon-common-extensions via Conda..."
    if command -v conda &> /dev/null; then
        conda install colcon-common-extensions -y
    else
        echo "ERROR: colcon not found and Conda not available to install it."
        echo "Please install it manually: pip install colcon-common-extensions"
        exit 1
    fi
fi

# 2. Build Workspace
echo "[1/4] Building ROS2 packages..."
colcon build --packages-select safestop_ai
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# 3. Source Workspace
echo "[2/4] Sourcing workspace..."
source install/setup.bash

# 4. Launch ROS2 Backend
echo "[3/4] Starting ROS2 Backend (Verification + Bridge)..."
ros2 launch safestop_ai verification.launch.py &
ROS_PID=$!

# Wait a bit for ROS to start
sleep 5

# 5. Launch Dashboard
echo "[4/4] Starting Investor Dashboard..."
cd safestop_dashboard
# Ensure dependencies are installed (quietly)
npm install > /dev/null 2>&1
# Start Vite server
npm run dev &
DASH_PID=$!

echo "=================================================="
echo "  🚀 DEMO IS LIVE!"
echo "=================================================="
echo "  > Dashboard: http://localhost:5173"
echo "  > Backend PID: $ROS_PID"
echo "=================================================="
echo "  Press Ctrl+C to stop the demo."
echo "=================================================="

# Wait for background processes
wait
