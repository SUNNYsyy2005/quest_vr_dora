#!/bin/bash

# CAN Bus Activation Script
# Usage: ./can_activate.sh <interface> <bitrate> <device>
# Example: ./can_activate.sh can0 1000000 3-8.4.3:1.0

CAN_INTERFACE=${1:-can0}
BITRATE=${2:-1000000}
USB_DEVICE=${3:-"3-8.4.3:1.0"}

echo "Activating CAN interface..."
echo "  Interface: $CAN_INTERFACE"
echo "  Bitrate: $BITRATE"
echo "  USB Device: $USB_DEVICE"

# Unbind and rebind USB device if specified
if [ ! -z "$USB_DEVICE" ]; then
    echo "Resetting USB device $USB_DEVICE..."
    echo "$USB_DEVICE" | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null 2>&1
    sleep 0.5
    echo "$USB_DEVICE" | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null 2>&1
    sleep 1
fi

# Check if CAN interface already exists and is up
if ip link show $CAN_INTERFACE 2>/dev/null | grep -q "UP"; then
    echo "Interface $CAN_INTERFACE already UP, bringing it down first..."
    sudo ip link set $CAN_INTERFACE down
    sleep 0.5
fi

# Configure CAN interface
echo "Configuring $CAN_INTERFACE..."
sudo ip link set $CAN_INTERFACE type can bitrate $BITRATE 2>/dev/null

# Bring up the interface
echo "Bringing up $CAN_INTERFACE..."
sudo ip link set $CAN_INTERFACE up

# Verify the interface is up
if ip link show $CAN_INTERFACE | grep -q "UP"; then
    echo "✅ CAN interface $CAN_INTERFACE is UP at ${BITRATE}bps"
    
    # Show interface statistics
    echo ""
    echo "Interface details:"
    ip -details link show $CAN_INTERFACE | grep -E "can|bitrate|state"
else
    echo "❌ Failed to bring up $CAN_INTERFACE"
    exit 1
fi