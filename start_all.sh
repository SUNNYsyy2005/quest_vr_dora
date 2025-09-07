#!/bin/bash

# Quest VR Dora Control System - Complete Startup Script
# This script handles all initialization steps for the VR-controlled robot arm system

echo "========================================="
echo "Quest VR Robot Control System (Dora)"
echo "Complete System Startup"
echo "========================================="

# Function to check command status
check_status() {
    if [ $? -eq 0 ]; then
        echo "✅ $1 successful"
    else
        echo "❌ $1 failed"
        echo "Please check the error and try again."
        exit 1
    fi
}

# Step 1: Activate CAN interface
echo ""
echo "Step 1: Activating CAN interface..."
echo "----------------------------------------"
if [ -f "can_activate.sh" ]; then
    bash can_activate.sh can0 1000000 3-8.4.3:1.0
    check_status "CAN activation"
elif [ -f "../can_activate.sh" ]; then
    bash ../can_activate.sh can0 1000000 3-8.4.3:1.0
    check_status "CAN activation"
elif [ -f "../../can_activate.sh" ]; then
    bash ../../can_activate.sh can0 1000000 3-8.4.3:1.0
    check_status "CAN activation"
else
    echo "⚠️  can_activate.sh not found, trying manual setup..."
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up
    check_status "Manual CAN setup"
fi

# Verify CAN is up
ip link show can0 | grep -q "UP"
if [ $? -eq 0 ]; then
    echo "✅ CAN interface can0 is UP"
else
    echo "❌ CAN interface can0 is not UP"
    echo "Please check your CAN connection and drivers."
    exit 1
fi

# Step 2: Activate conda environment
echo ""
echo "Step 2: Activating conda environment..."
echo "----------------------------------------"
source /home/dora/miniconda3/etc/profile.d/conda.sh
conda activate vt
check_status "Conda environment activation"

# Check and install ppadb if needed
echo ""
echo "Checking Python dependencies..."
python -c "import ppadb" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing ppadb..."
    pip install ppadb
    check_status "ppadb installation"
else
    echo "✅ ppadb already installed"
fi

# Step 3: Set library path
echo ""
echo "Step 3: Setting library paths..."
echo "----------------------------------------"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"
echo "LD_LIBRARY_PATH set to: $LD_LIBRARY_PATH"

# Step 4: Start Dora coordinator
echo ""
echo "Step 4: Starting Dora coordinator..."
echo "----------------------------------------"
# Check if dora is already running
if pgrep -x "dora-coordinator" > /dev/null; then
    echo "⚠️  Dora coordinator already running, skipping 'dora up'"
else
    echo "Starting Dora coordinator..."
    dora up &
    sleep 2  # Give coordinator time to start
fi

# Step 5: Check Quest connection
echo ""
echo "Step 5: Checking Quest VR connection..."
echo "----------------------------------------"
if command -v adb &> /dev/null; then
    if adb devices | grep -q "device$"; then
        echo "✅ Quest device found via USB"
        QUEST_MODEL=$(adb shell getprop ro.product.model 2>/dev/null | tr -d '\r\n')
        echo "   Device: $QUEST_MODEL"
    else
        echo "⚠️  No Quest device found via USB"
        echo "   Please ensure:"
        echo "   1. Quest is connected via USB"
        echo "   2. USB debugging is enabled in Quest settings"
        echo "   3. You've authorized this computer on the Quest"
        read -p "Press Enter to continue anyway or Ctrl+C to exit..."
    fi
else
    echo "⚠️  adb command not found. Cannot check Quest connection."
    echo "   Install adb with: sudo apt-get install adb"
    read -p "Press Enter to continue anyway or Ctrl+C to exit..."
fi

# Step 6: Run the dataflow
echo ""
echo "Step 6: Starting VR control system..."
echo "========================================="
echo ""
echo "Control Instructions:"
echo "  🔴 A Button: Reset to initial position"
echo "  🟢 B Button: Enable control (hold to move)"
echo "  🟡 Right Trigger: Control gripper"
echo "  📍 Right Controller: Move robot end-effector"
echo ""
echo "System Status:"
echo "  CAN: can0 @ 1Mbps"
echo "  Environment: vt (conda)"
echo "  Coordinator: Running"
echo ""
echo "Starting dataflow in 3 seconds..."
echo "Press Ctrl+C to stop the system"
echo "----------------------------------------"
sleep 3

# Run the main dataflow
dora run dataflow.yml

# Cleanup on exit
echo ""
echo "========================================="
echo "Shutting down system..."
echo "----------------------------------------"
echo "✅ System stopped"
echo "To restart, run: ./start_all.sh"
