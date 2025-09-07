#!/usr/bin/env python3

import os
from dora import Node
import pyarrow as pa
import numpy as np
from oculus_reader import OculusReader
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [VR_READER] [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# Simplified PyArrow approach - let PyArrow infer schemas
# Remove complex schema definitions to avoid conversion issues

def _safe_get_joystick_axis(joystick_data, axis_index):
    """Safely extract joystick axis value (handles both list and float formats)"""
    if joystick_data is None:
        return 0.0
    
    # If it's already a float (single value), return it for axis 0, or 0.0 for axis 1
    if isinstance(joystick_data, (int, float)):
        return float(joystick_data) if axis_index == 0 else 0.0
    
    # If it's a list, extract the requested axis
    if isinstance(joystick_data, (list, tuple)) and len(joystick_data) > axis_index:
        return float(joystick_data[axis_index])
    
    # Default fallback
    return 0.0

def extract_button_data(buttons):
    """Extract button data in EXACT questVR_ws ROS format"""
    if not buttons:
        return {
            'leftTrig': [0.0],  # Array format like ROS
            'rightTrig': [0.0], # Array format like ROS  
            'leftGrip': [0.0],
            'rightGrip': [0.0],
            'A': False,         # ROS naming
            'B': False,         # ROS naming
            'X': False,
            'Y': False,
            'leftJS': [0.0, 0.0],
            'rightJS': [0.0, 0.0]
        }
    
    try:
        # Preserve EXACT ROS button format - no conversion!
        # This matches questVR_ws exactly: buttons['rightTrig'][0]
        button_data = {
            'leftTrig': [float(buttons.get('leftTrig', (0.0,))[0] if isinstance(buttons.get('leftTrig', 0.0), tuple) else buttons.get('leftTrig', 0.0))],
            'rightTrig': [float(buttons.get('rightTrig', (0.0,))[0] if isinstance(buttons.get('rightTrig', 0.0), tuple) else buttons.get('rightTrig', 0.0))],
            'leftGrip': [float(buttons.get('leftGrip', (0.0,))[0] if isinstance(buttons.get('leftGrip', 0.0), tuple) else buttons.get('leftGrip', 0.0))],
            'rightGrip': [float(buttons.get('rightGrip', (0.0,))[0] if isinstance(buttons.get('rightGrip', 0.0), tuple) else buttons.get('rightGrip', 0.0))],
            'A': bool(buttons.get('A', False)),         # ROS naming
            'B': bool(buttons.get('B', False)),         # ROS naming  
            'X': bool(buttons.get('X', False)),
            'Y': bool(buttons.get('Y', False)),
            'leftJS': [float(_safe_get_joystick_axis(buttons.get('leftJS', 0.0), 0)), float(_safe_get_joystick_axis(buttons.get('leftJS', 0.0), 1))],
            'rightJS': [float(_safe_get_joystick_axis(buttons.get('rightJS', 0.0), 0)), float(_safe_get_joystick_axis(buttons.get('rightJS', 0.0), 1))]
        }
    except Exception as e:
        logger.error(f"Error in extract_button_data: {e}")
        logger.error(f"Button data: {buttons}")
        raise e
    
    # Individual button logging with clear descriptions (ROS format)
    if button_data.get('A', False):
        logger.warning(f"🔴 BUTTON A DETECTED - Reset Command!")
    if button_data.get('B', False):
        logger.warning(f"🟢 BUTTON B DETECTED - Enable Control!")
    if button_data.get('X', False):
        logger.warning(f"🔵 BUTTON X DETECTED!")
    if button_data.get('Y', False):
        logger.warning(f"🟡 BUTTON Y DETECTED!")
    if button_data.get('rightTrig', [0.0])[0] > 0.1:
        logger.warning(f"🟡 RIGHT TRIGGER: {button_data['rightTrig'][0]:.2f} - Gripper Control!")
    if button_data.get('leftTrig', [0.0])[0] > 0.1:
        logger.warning(f"🟠 LEFT TRIGGER: {button_data['leftTrig'][0]:.2f}")
    if button_data.get('rightGrip', [0.0])[0] > 0.1:
        logger.warning(f"🤏 RIGHT GRIP: {button_data['rightGrip'][0]:.2f}")
    if button_data.get('leftGrip', [0.0])[0] > 0.1:
        logger.warning(f"🤏 LEFT GRIP: {button_data['leftGrip'][0]:.2f}")
    
    return button_data

def process_and_send_vr_data(node, transforms, buttons):
    """Process and send VR transforms and buttons data"""
    try:
        # Process transforms
        transform_data = {}
        
        for hand in ['left', 'right']:
            hand_key = hand[0]  # 'l' or 'r'
            if transforms and hand_key in transforms:
                matrix = transforms[hand_key]
                if isinstance(matrix, np.ndarray) and matrix.shape == (4, 4):
                    transform_data[hand] = matrix.flatten().tolist()
                    logger.debug(f"  {hand.capitalize()} controller: position=[{matrix[0,3]:.3f}, {matrix[1,3]:.3f}, {matrix[2,3]:.3f}]")
                else:
                    transform_data[hand] = [0.0] * 16
                    logger.debug(f"  {hand.capitalize()} controller: invalid matrix format")
            else:
                transform_data[hand] = [0.0] * 16
                logger.debug(f"  {hand.capitalize()} controller: no data")
        
        # Send transforms as simple PyArrow array 
        transform_array = pa.array([transform_data])
        node.send_output("transforms", transform_array, {"timestamp": time.time()})
        
        # Process buttons
        button_data = extract_button_data(buttons)
        
        # Send buttons as simple PyArrow array
        button_array = pa.array([button_data])
        node.send_output("buttons", button_array, {"timestamp": time.time()})
        
        return True
        
    except Exception as e:
        logger.error(f"Error processing VR data: {e}")
        return False

def send_default_data(node):
    """Send default/empty data to keep the pipeline alive"""
    # Send empty transforms
    transform_data = {
        'left': [0.0] * 16,
        'right': [0.0] * 16
    }
    transform_array = pa.array([transform_data])
    node.send_output("transforms", transform_array, {"timestamp": time.time()})
    
    # Send empty buttons using ROS format
    button_data = extract_button_data(None)
    button_array = pa.array([button_data])
    node.send_output("buttons", button_array, {"timestamp": time.time()})

def main():
    logger.info("VR Reader Node started")
    
    # Get IP address from environment or use default
    ip_address = os.environ.get('QUEST_IP', None)
    logger.info(f"Quest IP configuration: {ip_address or 'USB connection'}")
    
    # Initialize Oculus Reader with run=False to manually control the loop
    try:
        logger.info("Initializing Oculus Reader...")
        oculus_reader = OculusReader(ip_address=ip_address, run=False)
        logger.info("Starting Oculus Reader logcat monitoring...")
        oculus_reader.run()
        logger.info("✅ Oculus Reader initialized successfully")
        
        # Give logcat thread time to start up and capture data
        logger.info("Waiting for logcat thread to stabilize...")
        time.sleep(2)
        
    except Exception as e:
        logger.error(f"❌ Failed to initialize Oculus Reader: {e}")
        logger.error("Make sure Quest is connected via USB or WiFi and ADB is available")
        return
    
    # Test initial data reading
    logger.info("Testing initial VR data reading...")
    test_transforms, test_buttons = oculus_reader.get_transformations_and_buttons()
    if test_transforms or test_buttons:
        logger.info("✅ Initial VR data test successful")
    else:
        logger.warning("⚠️ No initial VR data received - this is normal if VR app isn't started yet")
    
    # Get node ID from environment variable or use default
    node_id = os.getenv("DORA_NODE_ID", "vr_reader")
    node = Node(node_id)
    tick_count = 0
    last_log_time = time.time()
    data_received_count = 0
    
    try:
        while True:
            event = node.next()
            
            if event is None:
                continue
                
            if event["type"] == "STOP":
                logger.info("🛑 Received STOP signal")
                break
                
            if event["type"] == "INPUT" and event["id"] == "tick":
                tick_count += 1
                
                # Less frequent logging for cleaner output
                current_time = time.time()
                should_log = (tick_count % 100 == 0 or 
                            current_time - last_log_time >= 3 or 
                            tick_count <= 5)
                
                if should_log:
                    logger.info(f"Processing tick #{tick_count} (received at {current_time:.3f})")
                    last_log_time = current_time
                
                # Get latest transforms and buttons from VR
                try:
                    transforms, buttons = oculus_reader.get_transformations_and_buttons()
                    
                    if transforms is not None or buttons is not None:
                        data_received_count += 1
                        
                        if should_log or data_received_count <= 5:
                            logger.info(f"✅ Got VR data #{data_received_count} - Transforms: {len(transforms) if transforms else 0} controllers, "
                                      f"Buttons: {'Yes' if buttons else 'No'}")
                        
                        # Process and send the data
                        success = process_and_send_vr_data(node, transforms, buttons)
                        if not success:
                            logger.warning("Failed to process VR data")
                    else:
                        if should_log:
                            logger.info("⚠️ No VR data received (Quest app may not be running or controllers idle)")
                        
                        # Send empty/default data to keep the pipeline alive
                        send_default_data(node)
                        
                except Exception as e:
                    logger.error(f"❌ Failed to get VR data: {e}")
                    # Send default data to keep pipeline alive
                    send_default_data(node)
                
    except KeyboardInterrupt:
        logger.info("🛑 VR Reader stopped by user")
    except Exception as e:
        logger.error(f"❌ VR Reader error: {e}")
    finally:
        logger.info(f"⏹️ VR Reader stopped (processed {tick_count} ticks, received {data_received_count} data frames)")
        if 'oculus_reader' in locals():
            logger.info("Shutting down Oculus Reader...")
            oculus_reader.stop()

if __name__ == "__main__":
    main()
