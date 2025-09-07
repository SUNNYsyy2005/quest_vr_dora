#!/bin/bash

# Quest VR Dora Control System - Stop Script
# This script safely stops all components

echo "========================================="
echo "Stopping Quest VR Robot Control System"
echo "========================================="

# Stop dora processes
echo "Stopping Dora processes..."
pkill -f "dora run"
sleep 1

# Optionally stop dora coordinator
read -p "Stop Dora coordinator? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Stopping Dora coordinator..."
    dora down
fi

# Optionally disable CAN interface
read -p "Disable CAN interface? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Disabling CAN interface..."
    sudo ip link set can0 down
fi

echo ""
echo "✅ System stopped"
echo "To restart, run: ./start_all.sh"