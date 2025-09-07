#!/bin/bash

# Quest VR Dora Control System - Clean Version
# This script runs the VR-controlled robot arm system

echo "========================================="
echo "Quest VR Robot Control System (Dora)"
echo "========================================="

# Check if virtual environment exists
if [ ! -d "/home/dora/miniconda3/envs/vt" ]; then
    echo "Error: Virtual environment /home/dora/miniconda3/envs/vt not found!"
    echo "Please create it first with required dependencies."
    echo ""
    echo "To create the environment:"
    echo "  conda create -n vt python=3.9"
    echo "  conda activate vt"
    echo "  pip install -r requirements.txt"
    exit 1
fi

# Activate virtual environment
source /home/dora/miniconda3/etc/profile.d/conda.sh
conda activate vt

# Check if Quest is connected via USB
echo ""
echo "Checking Quest connection..."
if adb devices | grep -q "device$"; then
    echo "✅ Quest device found via USB"
else
    echo "⚠️  No Quest device found. Please connect your Quest via USB."
    echo "   Make sure USB debugging is enabled in Quest settings."
    read -p "Press Enter to continue anyway or Ctrl+C to exit..."
fi

# Run the dora dataflow
echo ""
echo "Starting Dora dataflow..."
echo "----------------------------------------"
echo "Controls:"
echo "  A Button: Reset to initial position"
echo "  B Button: Enable control (hold to move)"
echo "  Right Trigger: Control gripper"
echo "----------------------------------------"
echo ""

# Run with proper environment
export PYTHONPATH=/home/dora/dora-pipers/quest_vr_dora_clean:$PYTHONPATH
dora run dataflow.yml