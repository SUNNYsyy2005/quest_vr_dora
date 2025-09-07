#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../questVR_ws/src/Piper_ros/src/piper/scripts'))

from dora import Node, DoraStatus
import pyarrow as pa
import time
import logging
import threading
import numpy as np

# Import Piper SDK
try:
    from piper_sdk import *
    from piper_sdk import C_PiperInterface
    PIPER_SDK_AVAILABLE = True
except ImportError as e:
    PIPER_SDK_AVAILABLE = False
    print(f"Warning: Piper SDK not available: {e}")

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [ROBOT_CTRL] [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

class DoraRobotController:
    """
    Dora-compatible wrapper for PIPER robot controller
    Integrates real Piper SDK for hardware control
    """
    
    def __init__(self, can_port="can0", auto_enable=True, gripper_exist=True):
        logger.info("Initializing Dora Robot Controller...")
        
        # Configuration
        self.can_port = can_port
        self.auto_enable = auto_enable
        self.gripper_exist = gripper_exist
        self.gripper_val_multiple = 1.0
        
        # State variables
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.is_initialized = False
        self.__enable_flag = False
        logger.debug(f"Initial joint positions: {self.joint_positions}")
        
        # Initialize hardware connection with enhanced validation
        if PIPER_SDK_AVAILABLE:
            self.piper = self._initialize_hardware_connection()
        else:
            logger.warning("⚠️ Piper SDK not available - running in simulation mode")
            logger.info("To install Piper SDK: Check questVR_ws/src/Piper_ros/src/piper for installation instructions")
            self.piper = None
        
        # Safety limits (from PIPER controller) 
        self.joint_limits = [
            (-3.14, 3.14),   # joint1
            (-1.57, 1.57),   # joint2  
            (-1.57, 1.57),   # joint3
            (-3.14, 3.14),   # joint4
            (-1.57, 1.57),   # joint5
            (-3.14, 3.14),   # joint6
            (0.0, 0.08)      # gripper (in meters, converted to SDK units later)
        ]
        
        logger.info("Robot Controller initialized successfully")
        
        # Auto-enable if requested and hardware is available
        if self.auto_enable and self.piper:
            logger.info("🚀 Starting post-initialization auto-enable process...")
            self._auto_enable_arm()
    
    def _verify_can_interface(self):
        """Verify CAN interface is available and properly configured"""
        logger.debug(f"🔍 Verifying CAN interface: {self.can_port}")
        
        try:
            # Check if CAN interface exists
            import subprocess
            result = subprocess.run(['ip', 'link', 'show', self.can_port], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                logger.info(f"✅ CAN interface {self.can_port} exists")
                
                # Check if interface is UP
                if 'UP' in result.stdout:
                    logger.info(f"✅ CAN interface {self.can_port} is UP")
                    
                    # Extract bitrate if available
                    if 'bitrate' in result.stdout:
                        import re
                        bitrate_match = re.search(r'bitrate (\d+)', result.stdout)
                        if bitrate_match:
                            bitrate = bitrate_match.group(1)
                            logger.info(f"✅ CAN bitrate: {bitrate} bps")
                        else:
                            logger.warning("⚠️ Could not determine CAN bitrate")
                    
                    return True
                else:
                    logger.error(f"❌ CAN interface {self.can_port} is DOWN")
                    logger.error("💡 Try: sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0")
                    return False
            else:
                logger.error(f"❌ CAN interface {self.can_port} not found")
                logger.error("💡 Available interfaces:")
                list_result = subprocess.run(['ip', 'link', 'show'], capture_output=True, text=True, timeout=5)
                if list_result.returncode == 0:
                    can_interfaces = [line for line in list_result.stdout.split('\n') if 'can' in line.lower()]
                    for interface in can_interfaces:
                        logger.error(f"   {interface.strip()}")
                return False
                
        except subprocess.TimeoutExpired:
            logger.error("❌ Timeout checking CAN interface status")
            return False
        except Exception as e:
            logger.error(f"❌ Error checking CAN interface: {e}")
            return False
    
    def _initialize_hardware_connection(self):
        """Initialize hardware connection with comprehensive validation"""
        logger.info(f"🔌 Initializing Piper hardware connection...")
        logger.info(f"CAN Configuration: port={self.can_port}, auto_enable={self.auto_enable}")
        
        # Step 1: Verify CAN interface
        if not self._verify_can_interface():
            logger.error("❌ CAN interface verification failed")
            return None
        
        try:
            # Step 2: Create Piper interface
            logger.debug("🔧 Creating C_PiperInterface instance...")
            piper = C_PiperInterface(can_name=self.can_port)
            logger.info("✅ C_PiperInterface created successfully")
            
            # Step 3: Connect to port with retry mechanism
            logger.debug("🔌 Connecting to hardware port...")
            max_connect_attempts = 3
            for attempt in range(max_connect_attempts):
                try:
                    connect_result = piper.ConnectPort()
                    logger.info(f"✅ ConnectPort() successful (attempt {attempt+1})")
                    logger.debug(f"ConnectPort result: {connect_result}")
                    break
                except Exception as connect_e:
                    logger.warning(f"⚠️ ConnectPort attempt {attempt+1} failed: {connect_e}")
                    if attempt < max_connect_attempts - 1:
                        time.sleep(0.5)
                    else:
                        raise connect_e
            
            # Step 4: Initialize motion control
            logger.debug("🎮 Initializing motion control...")
            try:
                motion_result = piper.MotionCtrl_2(0x01, 0x01, 30, 0xad)
                logger.info("✅ Motion control initialized successfully")
                logger.debug(f"MotionCtrl_2 result: {motion_result}")
            except Exception as motion_e:
                logger.warning(f"⚠️ Motion control initialization warning: {motion_e}")
                logger.warning("Continuing with basic connection...")
            
            # Step 5: Comprehensive communication test
            logger.debug("📡 Testing hardware communication...")
            communication_ok = False
            test_attempts = 3
            
            for test_attempt in range(test_attempts):
                try:
                    test_status = piper.GetArmLowSpdInfoMsgs()
                    logger.info(f"✅ Hardware communication test successful (attempt {test_attempt+1})")
                    
                    # Detailed initial motor state analysis
                    initial_motor_status = {
                        'motor_1': test_status.motor_1.foc_status.driver_enable_status,
                        'motor_2': test_status.motor_2.foc_status.driver_enable_status,
                        'motor_3': test_status.motor_3.foc_status.driver_enable_status,
                        'motor_4': test_status.motor_4.foc_status.driver_enable_status,
                        'motor_5': test_status.motor_5.foc_status.driver_enable_status,
                        'motor_6': test_status.motor_6.foc_status.driver_enable_status
                    }
                    enabled_count = sum(initial_motor_status.values())
                    
                    logger.info(f"📊 Initial motor state: {enabled_count}/6 motors enabled")
                    
                    # Log individual motor states
                    for motor, status in initial_motor_status.items():
                        state_emoji = "✅" if status else "❌"
                        logger.debug(f"{state_emoji} {motor}: {'enabled' if status else 'disabled'}")
                    
                    # Additional hardware diagnostics
                    try:
                        logger.debug("🔍 Reading extended hardware diagnostics...")
                        
                        # Test motor currents and voltages (temperature may not be available)
                        motor_data = [test_status.motor_1, test_status.motor_2, test_status.motor_3, 
                                     test_status.motor_4, test_status.motor_5, test_status.motor_6]
                        
                        # Check if temperature data is available
                        try:
                            temps = [motor.foc_status.temp for motor in motor_data]
                            avg_temp = sum(temps) / len(temps)
                            max_temp = max(temps)
                            logger.debug(f"🌡️ Motor temperatures - Avg: {avg_temp:.1f}°C, Max: {max_temp:.1f}°C")
                            
                            if max_temp > 70:
                                logger.warning(f"⚠️ High motor temperature detected: {max_temp}°C")
                            elif max_temp > 50:
                                logger.info(f"ℹ️ Motor temperatures normal (max: {max_temp:.1f}°C)")
                        except AttributeError:
                            logger.debug("🌡️ Temperature data not available from hardware")
                        
                        # Check current and voltage data
                        try:
                            currents = [abs(motor.foc_status.current) for motor in motor_data]
                            voltages = [motor.foc_status.voltage for motor in motor_data]
                            
                            max_current = max(currents)
                            avg_voltage = sum(voltages) / len(voltages)
                            
                            logger.debug(f"🔋 Motor status - Max current: {max_current:.0f}mA, Avg voltage: {avg_voltage:.1f}V")
                            
                            if max_current > 3000:
                                logger.info(f"⚡ High motor activity detected: {max_current:.0f}mA")
                                
                        except AttributeError:
                            logger.debug("🔋 Current/voltage data not available from hardware")
                        
                    except Exception as diag_e:
                        logger.debug(f"Extended diagnostics failed: {diag_e}")
                    
                    communication_ok = True
                    break
                    
                except Exception as test_e:
                    logger.warning(f"⚠️ Communication test attempt {test_attempt+1} failed: {test_e}")
                    if test_attempt < test_attempts - 1:
                        time.sleep(0.5)
                    else:
                        logger.error(f"❌ All communication tests failed")
                        raise test_e
            
            if communication_ok:
                logger.info("🎉 Hardware connection established successfully!")
                
                # Return the piper interface first, then auto-enable
                return piper
                
        except Exception as e:
            logger.error(f"❌ Failed to initialize Piper hardware: {e}")
            logger.error(f"Exception details: {type(e).__name__}: {str(e)}")
            
            # Provide detailed troubleshooting information
            logger.error("🔧 Hardware initialization failed - Troubleshooting guide:")
            logger.error("  1. ✓ Check CAN interface: sudo ip link show can0")
            logger.error("  2. ✓ Setup CAN interface: sudo ip link set can0 type can bitrate 1000000")
            logger.error("  3. ✓ Bring up interface: sudo ip link set up can0") 
            logger.error("  4. ✓ Check hardware power and connections")
            logger.error("  5. ✓ Verify user permissions for CAN access")
            logger.error("  6. ✓ Check emergency stops are released")
            logger.error("  7. ✓ Ensure Piper SDK is properly installed")
            
            return None
        
    def _check_individual_motor_diagnostics(self, motor_num, motor_data):
        """Enhanced individual motor diagnostics"""
        try:
            enable_status = motor_data.foc_status.driver_enable_status
            logger.debug(f"Motor {motor_num} detailed status:")
            logger.debug(f"  - Enable Status: {enable_status}")
            
            # Safely check other attributes that may not exist
            try:
                logger.debug(f"  - Angle (raw): {motor_data.angle}")
            except AttributeError:
                logger.debug(f"  - Angle: Not available")
                
            try:
                logger.debug(f"  - Velocity: {motor_data.velocity}")
            except AttributeError:
                logger.debug(f"  - Velocity: Not available")
                
            try:
                logger.debug(f"  - Current: {motor_data.foc_status.current}")
                logger.debug(f"  - Voltage: {motor_data.foc_status.voltage}")
                logger.debug(f"  - Error Code: {motor_data.foc_status.foc_status}")
                
                # Check for error conditions
                if abs(motor_data.foc_status.current) > 5000:  # High current warning  
                    logger.warning(f"⚠️ Motor {motor_num} high current: {motor_data.foc_status.current}mA")
                    
                if motor_data.foc_status.foc_status != 0:  # Error status
                    logger.error(f"❌ Motor {motor_num} error status: {motor_data.foc_status.foc_status}")
                    
            except AttributeError:
                logger.debug(f"  - Current/Voltage/FOC status: Not available")
            
            # Check for error conditions (with safe attribute access)
            try:
                if hasattr(motor_data.foc_status, 'temp') and motor_data.foc_status.temp > 70:
                    logger.warning(f"⚠️ Motor {motor_num} temperature high: {motor_data.foc_status.temp}°C")
            except AttributeError:
                logger.debug(f"Motor {motor_num} temperature data not available")
                
            return enable_status
            
        except Exception as e:
            logger.error(f"Failed to check motor {motor_num} enable status: {e}")
            # Even if diagnostic details fail, try to get basic enable status
            try:
                basic_enable_status = motor_data.foc_status.driver_enable_status
                logger.warning(f"Motor {motor_num} enable status (basic): {basic_enable_status}")
                return basic_enable_status
            except:
                logger.error(f"Cannot determine enable status for motor {motor_num}")
                return False

    def _auto_enable_arm(self):
        """Auto enable arm with enhanced diagnostics"""
        logger.info("🔧 Starting enhanced auto-enable process...")
        enable_flag = False
        timeout = 15  # Increase timeout to 15 seconds for thorough diagnostics
        start_time = time.time()
        attempt_count = 0
        
        logger.info(f"Auto-enable configuration: timeout={timeout}s, CAN_port={self.can_port}")
        
        # Check if hardware is available
        if not hasattr(self, 'piper') or not self.piper:
            logger.error("❌ Hardware interface not available - cannot enable arm")
            logger.error("This indicates Piper SDK is not available or hardware initialization failed")
            return
            
        # Initial hardware communication test
        logger.debug("🔍 Testing initial hardware communication...")
        try:
            test_msg = self.piper.GetArmLowSpdInfoMsgs()
            logger.info("✅ Initial hardware communication successful")
        except Exception as comm_e:
            logger.error(f"❌ Initial hardware communication failed: {comm_e}")
            logger.error("This indicates a fundamental CAN bus or hardware connectivity issue")
            return
        
        while not enable_flag and (time.time() - start_time) < timeout:
            attempt_count += 1
            elapsed_time = time.time() - start_time
            
            try:
                logger.info(f"🔄 Auto-enable attempt #{attempt_count} (elapsed: {elapsed_time:.1f}s)")
                
                # Get comprehensive hardware status
                logger.debug("📊 Reading comprehensive hardware status...")
                status = self.piper.GetArmLowSpdInfoMsgs()
                
                # Enhanced individual motor diagnostics
                logger.debug("🔍 Performing detailed motor diagnostics...")
                motor_states = {}
                motor_states['motor_1'] = self._check_individual_motor_diagnostics(1, status.motor_1)
                motor_states['motor_2'] = self._check_individual_motor_diagnostics(2, status.motor_2)
                motor_states['motor_3'] = self._check_individual_motor_diagnostics(3, status.motor_3)
                motor_states['motor_4'] = self._check_individual_motor_diagnostics(4, status.motor_4)
                motor_states['motor_5'] = self._check_individual_motor_diagnostics(5, status.motor_5)
                motor_states['motor_6'] = self._check_individual_motor_diagnostics(6, status.motor_6)
                
                # Analyze motor status patterns
                enabled_motors = [name for name, enabled in motor_states.items() if enabled]
                disabled_motors = [name for name, enabled in motor_states.items() if not enabled]
                enabled_count = len(enabled_motors)
                
                logger.info(f"📈 Motor Status Summary: {enabled_count}/6 enabled")
                if enabled_motors:
                    logger.info(f"✅ Enabled motors: {', '.join(enabled_motors)}")
                if disabled_motors:
                    logger.warning(f"❌ Disabled motors: {', '.join(disabled_motors)}")
                    # Log which specific motor number is disabled (1-6 mapping)
                    disabled_nums = [name.split('_')[1] for name in disabled_motors]
                    logger.warning(f"❌ Motor numbers disabled: {', '.join(disabled_nums)} (of 1-6)")
                    if '6' in disabled_nums:
                        logger.error(f"🔥 MOTOR 6 (gripper joint) is disabled - this affects gripper control")
                    if any(num in disabled_nums for num in ['1', '2', '3']):
                        logger.error(f"🔥 Primary joint motors disabled - affects vertical motion")
                
                # Check if all motors are enabled
                enable_flag = all(motor_states.values())
                
                if enable_flag:
                    logger.info("🎉 All motors are enabled!")
                    
                    # Final verification with additional hardware checks
                    logger.debug("🔍 Performing final enable verification...")
                    try:
                        arm_status = self.piper.GetArmStatus()
                        logger.debug("✅ Final arm status check successful")
                        logger.debug(f"System ready - Joint limits OK, Communication stable")
                    except Exception as final_e:
                        logger.warning(f"⚠️ Final status check failed but motors enabled: {final_e}")
                    break
                else:
                    logger.warning(f"⚠️ {6-enabled_count} motors still need enabling...")
                    
                    # Analyze failure patterns  
                    if enabled_count == 0:
                        logger.error("❌ NO motors enabled - likely CAN bus or power issue")
                    elif enabled_count < 3:
                        logger.error("❌ Few motors enabled - possible hardware fault")  
                    else:
                        logger.warning(f"⚠️ Partial enable ({enabled_count}/6) - retrying...")
                    
                    # Send enable command with enhanced logging  
                    # Try both EnableArm(6) for joints only and EnableArm(7) for joints+gripper
                    logger.debug("📤 Sending EnableArm(6) command for joints only...")
                    enable_result_joints = self.piper.EnableArm(6)
                    logger.debug(f"EnableArm(6) result: {enable_result_joints}")
                    time.sleep(0.2)
                    
                    logger.debug("📤 Sending EnableArm(7) command for joints+gripper...")  
                    enable_result_all = self.piper.EnableArm(7)
                    logger.debug(f"EnableArm(7) result: {enable_result_all}")
                    
                    # Enhanced wait time based on number of disabled motors
                    wait_time = 0.3 + (len(disabled_motors) * 0.1)  # More time for more motors
                    logger.debug(f"⏱️ Waiting {wait_time:.1f}s for motors to respond...")
                    time.sleep(wait_time)
                    
                    # Additional comprehensive status check
                    logger.debug("🔍 Checking extended hardware status...")
                    try:
                        arm_status = self.piper.GetArmStatus()
                        
                        # Check communication status for each joint
                        comm_status = {
                            'joint_1_comm': arm_status.arm_status.err_status.communication_status_joint_1,
                            'joint_2_comm': arm_status.arm_status.err_status.communication_status_joint_2,
                            'joint_3_comm': arm_status.arm_status.err_status.communication_status_joint_3,
                            'joint_4_comm': arm_status.arm_status.err_status.communication_status_joint_4,
                            'joint_5_comm': arm_status.arm_status.err_status.communication_status_joint_5,
                            'joint_6_comm': arm_status.arm_status.err_status.communication_status_joint_6
                        }
                        
                        failed_comm = [joint for joint, status in comm_status.items() if not status]
                        if failed_comm:
                            logger.error(f"❌ Communication failures: {', '.join(failed_comm)}")
                        else:
                            logger.debug("✅ All joint communications OK")
                            
                        # Check joint limit status
                        limit_status = {
                            'joint_1_limit': arm_status.arm_status.err_status.joint_1_angle_limit,
                            'joint_2_limit': arm_status.arm_status.err_status.joint_2_angle_limit,
                            'joint_3_limit': arm_status.arm_status.err_status.joint_3_angle_limit,
                            'joint_4_limit': arm_status.arm_status.err_status.joint_4_angle_limit,
                            'joint_5_limit': arm_status.arm_status.err_status.joint_5_angle_limit,
                            'joint_6_limit': arm_status.arm_status.err_status.joint_6_angle_limit
                        }
                        
                        limit_errors = [joint for joint, has_limit_error in limit_status.items() if has_limit_error]
                        if limit_errors:
                            logger.warning(f"⚠️ Joint limit warnings: {', '.join(limit_errors)}")
                            
                    except Exception as status_e:
                        logger.debug(f"Could not get extended status: {status_e}")
                    
            except Exception as e:
                logger.error(f"❌ Error during auto-enable attempt #{attempt_count}: {e}")
                logger.error(f"Exception details: {type(e).__name__}: {str(e)}")
                logger.debug("Possible causes: CAN driver issues, hardware faults, power problems")
                time.sleep(0.5)
                
        final_elapsed = time.time() - start_time
        
        if enable_flag:
            self.__enable_flag = True
            logger.info(f"🎉 ARM ENABLED SUCCESSFULLY!")
            logger.info(f"📊 Enable Statistics: {attempt_count} attempts in {final_elapsed:.1f}s")
            logger.info("🚀 Robot is ready for operation")
        else:
            self.__enable_flag = False
            logger.error(f"❌ FAILED TO AUTO-ENABLE ARM")
            logger.error(f"📊 Failed after {attempt_count} attempts in {final_elapsed:.1f}s (timeout: {timeout}s)")
            logger.error("🔧 Diagnostic checklist:")
            logger.error("  1. ✓ CAN interface available? (ip link show can0)")
            logger.error("  2. ✓ Motor power connected?") 
            logger.error("  3. ✓ Hardware emergency stops released?")
            logger.error("  4. ✓ User permissions for CAN access?")
            logger.error("  5. ✓ Piper SDK properly installed?")
            logger.error("💡 Try: sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0")
    
    def init_pose(self, target_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        """
        Move robot to initial pose with smooth trajectory
        """
        logger.info(f"Moving to initial pose: {[f'{p:.3f}' for p in target_positions]}")
        
        # Implement smooth trajectory like the original PIPER code
        current_positions = self.joint_positions.copy()
        duration = 0.5  # seconds
        rate = 50  # Hz
        steps = int(duration * rate)
        
        logger.debug(f"Trajectory: {steps} steps over {duration}s at {rate}Hz")
        
        # Calculate increments for each joint
        increments = [(target - current) / steps 
                     for current, target in zip(current_positions, target_positions)]
        
        # Execute smooth trajectory
        for step in range(steps + 1):
            interpolated_positions = [
                current + increment * step 
                for current, increment in zip(current_positions, increments)
            ]
            
            # Apply safety limits
            safe_positions = self.apply_joint_limits(interpolated_positions)
            
            # Send to hardware (would be actual hardware call)
            self.send_joint_commands(safe_positions)
            
            # Control loop timing
            time.sleep(1.0 / rate)
        
        self.joint_positions = target_positions.copy()
        self.is_initialized = True
        logger.info("Initial pose reached successfully")
    
    def apply_joint_limits(self, joint_positions):
        """
        Apply safety limits to joint positions
        """
        safe_positions = []
        limited = False
        for i, pos in enumerate(joint_positions):
            if i < len(self.joint_limits):
                min_limit, max_limit = self.joint_limits[i]
                safe_pos = max(min_limit, min(max_limit, pos))
                if safe_pos != pos:
                    limited = True
                    logger.warning(f"Joint {i+1} limited: {pos:.3f} -> {safe_pos:.3f} (limits: [{min_limit:.3f}, {max_limit:.3f}])")
                safe_positions.append(safe_pos)
            else:
                safe_positions.append(pos)
        
        if limited:
            logger.debug(f"Applied limits - Result: {[f'{p:.3f}' for p in safe_positions]}")
        
        return safe_positions
    
    def _log_real_time_status(self):
        """Log real-time hardware status for monitoring"""
        if not self.piper:
            logger.debug("💭 Status monitoring: Hardware not connected (simulation mode)")
            return
            
        try:
            logger.debug("📊 Performing real-time status check...")
            
            # Get current hardware status
            status = self.piper.GetArmLowSpdInfoMsgs()
            
            # Check motor enable states
            motor_states = {
                'motor_1': status.motor_1.foc_status.driver_enable_status,
                'motor_2': status.motor_2.foc_status.driver_enable_status,
                'motor_3': status.motor_3.foc_status.driver_enable_status,
                'motor_4': status.motor_4.foc_status.driver_enable_status,
                'motor_5': status.motor_5.foc_status.driver_enable_status,
                'motor_6': status.motor_6.foc_status.driver_enable_status
            }
            
            enabled_count = sum(motor_states.values())
            
            # Create status summary
            enable_status_emoji = "✅" if enabled_count == 6 else "⚠️" if enabled_count > 0 else "❌"
            logger.info(f"📈 Status Monitor: {enable_status_emoji} {enabled_count}/6 motors enabled, Enable flag: {self.__enable_flag}")
            
            # Hardware monitoring (temperature, current, voltage)
            try:
                motor_data = [status.motor_1, status.motor_2, status.motor_3, 
                             status.motor_4, status.motor_5, status.motor_6]
                
                currents = [abs(motor.foc_status.current) for motor in motor_data]
                voltages = [motor.foc_status.voltage for motor in motor_data]
                
                max_current = max(currents)
                avg_voltage = sum(voltages) / len(voltages)
                
                # Try to get temperature data if available
                try:
                    temps = [motor.foc_status.temp for motor in motor_data]
                    avg_temp = sum(temps) / len(temps)
                    max_temp = max(temps)
                    
                    temp_emoji = "🔥" if max_temp > 70 else "🌡️" if max_temp > 50 else "❄️"
                    temp_info = f"{temp_emoji} Temp avg:{avg_temp:.1f}°C max:{max_temp:.1f}°C, "
                    
                    # Warn about concerning temperatures
                    if max_temp > 65:
                        logger.warning(f"⚠️ Motor temperature elevated: {max_temp:.1f}°C")
                        
                except AttributeError:
                    temp_info = "🌡️ Temp data N/A, "
                
                current_emoji = "⚡" if max_current > 3000 else "🔋"
                
                logger.info(f"🔍 Hardware Monitor: {temp_info}"
                           f"{current_emoji} Max current:{max_current:.0f}mA, Voltage:{avg_voltage:.1f}V")
                
                # Warn about high current
                if max_current > 4000:
                    logger.warning(f"⚠️ High motor current detected: {max_current:.0f}mA")
                    
            except Exception as monitor_e:
                logger.debug(f"Hardware monitoring failed: {monitor_e}")
            
            # Check for any disabled motors and report which ones
            disabled_motors = [name for name, enabled in motor_states.items() if not enabled]
            if disabled_motors:
                logger.warning(f"❌ Disabled motors detected: {', '.join(disabled_motors)}")
                
            # Additional system status
            try:
                arm_status = self.piper.GetArmStatus()
                
                # Check communication status
                comm_failures = []
                if not arm_status.arm_status.err_status.communication_status_joint_1:
                    comm_failures.append("J1")
                if not arm_status.arm_status.err_status.communication_status_joint_2:
                    comm_failures.append("J2")
                if not arm_status.arm_status.err_status.communication_status_joint_3:
                    comm_failures.append("J3")
                if not arm_status.arm_status.err_status.communication_status_joint_4:
                    comm_failures.append("J4")
                if not arm_status.arm_status.err_status.communication_status_joint_5:
                    comm_failures.append("J5")
                if not arm_status.arm_status.err_status.communication_status_joint_6:
                    comm_failures.append("J6")
                
                if comm_failures:
                    logger.error(f"📡❌ Communication failures: {', '.join(comm_failures)}")
                else:
                    logger.debug("📡✅ All joint communications OK")
                    
            except Exception as status_e:
                logger.debug(f"Extended status check failed: {status_e}")
                
        except Exception as e:
            logger.warning(f"⚠️ Real-time status monitoring failed: {e}")
            logger.debug(f"Status monitoring error details: {type(e).__name__}: {str(e)}")
    
    def send_joint_commands(self, joint_positions):
        """
        Send joint commands to hardware using Piper SDK
        """
        # Apply safety limits
        safe_positions = self.apply_joint_limits(joint_positions)
        
        # Log the command for debugging
        logger.debug(f"Sending joint commands: J1={safe_positions[0]:.3f}, J2={safe_positions[1]:.3f}, "
                    f"J3={safe_positions[2]:.3f}, J4={safe_positions[3]:.3f}, J5={safe_positions[4]:.3f}, "
                    f"J6={safe_positions[5]:.3f}, Gripper={safe_positions[6]:.3f}")
        
        if self.piper and self.__enable_flag:
            try:
                # Verify robot is still enabled before sending commands
                status = self.piper.GetArmLowSpdInfoMsgs()
                current_enable_status = [
                    status.motor_1.foc_status.driver_enable_status,
                    status.motor_2.foc_status.driver_enable_status,
                    status.motor_3.foc_status.driver_enable_status,
                    status.motor_4.foc_status.driver_enable_status,
                    status.motor_5.foc_status.driver_enable_status,
                    status.motor_6.foc_status.driver_enable_status
                ]
                
                enabled_count = sum(current_enable_status)
                if enabled_count < 6:
                    logger.warning(f"⚠️ Only {enabled_count}/6 motors enabled - attempting re-enable")
                    # Use same dual-enable strategy as in auto_enable
                    self.piper.EnableArm(6)
                    time.sleep(0.1)
                    self.piper.EnableArm(7)
                    time.sleep(0.1)
                
                # Convert to Piper SDK format (similar to ROS version)
                factor = 1000 * 180 / np.pi  # Convert radians to millidegrees
                
                joint_0 = round(safe_positions[0] * factor)
                joint_1 = round(safe_positions[1] * factor) 
                joint_2 = round(safe_positions[2] * factor)
                joint_3 = round(safe_positions[3] * factor)
                joint_4 = round(safe_positions[4] * factor)
                joint_5 = round(safe_positions[5] * factor)
                
                logger.debug(f"SDK commands: J0={joint_0}, J1={joint_1}, J2={joint_2}, J3={joint_3}, J4={joint_4}, J5={joint_5}")
                
                # Send joint commands to hardware
                result = self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
                logger.debug(f"JointCtrl result: {result}")
                
                # Handle gripper if it exists
                if self.gripper_exist and len(safe_positions) >= 7:
                    # Convert gripper position from meters to SDK units
                    gripper_raw = safe_positions[6]  # Original value from IK solver
                    joint_6 = int(round(gripper_raw * 1000 * 1000))  # m to micrometers (as int)
                    joint_6 = int(joint_6 * self.gripper_val_multiple)
                    joint_6 = int(max(0, min(80000, abs(joint_6))))  # Apply gripper limits (as int)
                    
                    # Enhanced gripper debugging
                    logger.debug(f"🤏 Gripper control: raw={gripper_raw:.4f}, converted={joint_6}, threshold_check={joint_6 >= 50}")
                    
                    # Always send gripper command (both open and close)
                    if joint_6 >= 50:  # Opening gripper (trigger pressed)
                        gripper_result = self.piper.GripperCtrl(joint_6, 1000, 0x01, 0)  # 1N default force
                        logger.warning(f"🔓 GRIPPER OPENING: {joint_6} micrometers, result: {gripper_result}")
                    else:  # Closing gripper (trigger released or below threshold)
                        # Force close gripper when trigger is released
                        gripper_result = self.piper.GripperCtrl(0, 1000, 0x01, 0)
                        logger.warning(f"🔒 GRIPPER CLOSING: 0 micrometers (trigger released), result: {gripper_result}")
                        
                    if gripper_raw > 0.001 and joint_6 < 50:  # Show when input detected but below threshold
                        logger.info(f"🤏 Gripper input detected but below threshold: raw={gripper_raw:.4f}, converted={joint_6}")
                
                logger.debug("✅ Joint commands sent to hardware successfully")
                
            except Exception as e:
                logger.error(f"❌ Failed to send joint commands to hardware: {e}")
                logger.debug(f"Exception details: {type(e).__name__}: {str(e)}")
                
        elif not self.piper:
            logger.warning("HARDWARE INTERFACE NOT CONNECTED - Commands not sent to actual robot")
        elif not self.__enable_flag:
            logger.warning("Robot not enabled - Commands not sent (use Button B to enable control)")
            
        # Update internal state
        self.joint_positions = safe_positions.copy()
        
    def enable_robot(self):
        """Enable the robot"""
        if self.piper:
            try:
                logger.info("Attempting to enable robot...")
                
                # Send enable commands with both joint-only and full enable
                logger.debug("Sending EnableArm(6) for joints only...")
                result_joints = self.piper.EnableArm(6)
                logger.debug(f"EnableArm(6) result: {result_joints}")
                time.sleep(0.2)
                
                logger.debug("Sending EnableArm(7) for joints+gripper...")
                result_all = self.piper.EnableArm(7)
                logger.debug(f"EnableArm(7) result: {result_all}")
                
                # Wait and verify
                time.sleep(0.5)
                
                # Verify enable status
                status = self.piper.GetArmLowSpdInfoMsgs()
                motor_status = {
                    'motor_1': status.motor_1.foc_status.driver_enable_status,
                    'motor_2': status.motor_2.foc_status.driver_enable_status,
                    'motor_3': status.motor_3.foc_status.driver_enable_status,
                    'motor_4': status.motor_4.foc_status.driver_enable_status,
                    'motor_5': status.motor_5.foc_status.driver_enable_status,
                    'motor_6': status.motor_6.foc_status.driver_enable_status
                }
                
                all_enabled = all(motor_status.values())
                enabled_count = sum(motor_status.values())
                
                logger.info(f"Enable verification: {enabled_count}/6 motors enabled")
                
                if all_enabled:
                    self.__enable_flag = True
                    logger.info("✅ Robot enabled successfully")
                    return True
                else:
                    disabled_motors = [name for name, enabled in motor_status.items() if not enabled]
                    logger.warning(f"⚠️ Robot partially enabled. Disabled motors: {disabled_motors}")
                    self.__enable_flag = False
                    return False
                    
            except Exception as e:
                logger.error(f"Failed to enable robot: {e}")
                logger.debug(f"Exception details: {type(e).__name__}: {str(e)}")
                return False
        else:
            logger.error("Cannot enable robot - Piper interface not available")
            return False
        
    def disable_robot(self):
        """Disable the robot"""
        if self.piper:
            try:
                # Send zero commands to stop motion
                self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
                self.__enable_flag = False
                logger.info("Robot disabled")
                return True
            except Exception as e:
                logger.error(f"Failed to disable robot: {e}")
                return False
        return False
        
    def get_enable_flag(self):
        """Get current enable status"""
        return self.__enable_flag

def main():
    logger.info("Robot Controller Node starting...")
    
    # Configuration - can be modified based on environment variables or config file
    can_port = os.environ.get('CAN_PORT', 'can0')
    auto_enable = os.environ.get('AUTO_ENABLE', 'true').lower() == 'true'
    gripper_exist = os.environ.get('GRIPPER_EXIST', 'true').lower() == 'true'
    
    logger.info(f"Configuration: CAN_PORT={can_port}, AUTO_ENABLE={auto_enable}, GRIPPER_EXIST={gripper_exist}")
    
    # Initialize robot controller with configuration
    try:
        robot_controller = DoraRobotController(
            can_port=can_port,
            auto_enable=auto_enable, 
            gripper_exist=gripper_exist
        )
        logger.info("Robot controller initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize robot controller: {e}", exc_info=True)
        return
    
    # State tracking
    last_command_time = time.time()
    safety_timeout = 1.0  # seconds
    command_count = 0
    last_status_log = time.time()
    status_log_interval = 10.0  # Log status every 10 seconds
    logger.info(f"Safety timeout set to {safety_timeout}s")
    logger.info(f"Status monitoring interval: {status_log_interval}s")
    
    # Get node ID from environment variable or use default
    node_id = os.getenv("DORA_NODE_ID", "robot_controller")
    node = Node(node_id)
    
    try:
        while True:
            event = node.next()
            
            if event is None:
                continue
                
            if event["type"] == "STOP":
                logger.info("🛑 Received STOP signal")
                logger.info("Disabling robot and moving to safe position before shutdown...")
                # Disable robot and move to safe position before stopping
                robot_controller.disable_robot()
                robot_controller.init_pose()
                logger.info(f"Robot Controller Node shutdown complete (processed {command_count} commands)")
                break
                
            if event["type"] == "INPUT":
                if event["id"] == "joint_commands":
                    command_count += 1
                    
                    # Convert Arrow array data back to joint commands
                    array = event["value"]
                    
                    if len(array) > 0:
                        joint_dict = array[0].as_py()  # Get first (and only) element
                        logger.debug(f"Received joint command #{command_count}")
                        
                        # Enhanced debugging for gripper reception
                        gripper_value = joint_dict.get('gripper', 0.0)
                        if gripper_value != 0.0:
                            logger.warning(f"📥 RECEIVED GRIPPER: {gripper_value:.4f} (full dict: {joint_dict})")
                        else:
                            logger.debug(f"📥 Gripper value is zero in received dict: {joint_dict}")
                        
                        # Extract joint positions
                        joint_positions = [
                            joint_dict.get('joint1', 0.0),
                            joint_dict.get('joint2', 0.0), 
                            joint_dict.get('joint3', 0.0),
                            joint_dict.get('joint4', 0.0),
                            joint_dict.get('joint5', 0.0),
                            joint_dict.get('joint6', 0.0),
                            joint_dict.get('gripper', 0.0)
                        ]
                        
                        control_enabled = joint_dict.get('enabled', False)
                        reset_requested = joint_dict.get('reset_requested', False)
                        
                        logger.debug(f"Command state - Control: {control_enabled}, Reset: {reset_requested}")
                        
                        # Handle reset request
                        if reset_requested:
                            logger.warning("RESET REQUESTED - Moving to safe position")
                            robot_controller.init_pose()
                        
                        # Handle normal control commands
                        elif control_enabled:
                            logger.info(f"Executing command #{command_count} - Control enabled")
                            # Enable robot if not already enabled
                            if not robot_controller.get_enable_flag():
                                robot_controller.enable_robot()
                            
                            robot_controller.send_joint_commands(joint_positions)
                            last_command_time = time.time()
                        else:
                            logger.debug("Control not enabled - holding position")
                            # Keep robot enabled to maintain position (like ROS version)
                            # Only handle gripper if needed
                            gripper_value = joint_dict.get('gripper', 0.0)
                            if gripper_value > 0.001:
                                # Still allow gripper control when B is released
                                current_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gripper_value]
                                robot_controller.send_joint_commands(current_positions)
                            # Do NOT disable robot - let it hold position
                        
                        # Safety timeout check
                        time_since_last = time.time() - last_command_time
                        if time_since_last > safety_timeout:
                            logger.error(f"SAFETY TIMEOUT ({time_since_last:.1f}s > {safety_timeout}s) - Robot should stop!")
                            # Hold current position or move to safe pose
                            # robot_controller.init_pose()  # Uncomment to auto-reset on timeout
                    
                    # Periodic real-time status monitoring
                    current_time = time.time()
                    if current_time - last_status_log >= status_log_interval:
                        robot_controller._log_real_time_status()
                        last_status_log = current_time
                
    except KeyboardInterrupt:
        logger.info("🛑 Robot Controller stopped by user")
    except Exception as e:
        logger.error(f"❌ Robot Controller error: {e}")
    finally:
        logger.info("Shutting down robot controller...")
        if 'robot_controller' in locals():
            robot_controller.disable_robot()
        logger.info("⏹️ Robot Controller stopped")

if __name__ == "__main__":
    main()

