#!/usr/bin/env python3
"""
Exact replication of questVR_ws IK solver for dora
Uses Pinocchio + CasADi for accurate IK solving, identical to ROS version
"""

from dora import Node
import pyarrow as pa
import numpy as np
import time
import logging
import math
import os

import pinocchio as pin
from pinocchio import casadi as cpin
import casadi

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [IK_EXACT] [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

def matrix_to_xyzrpy(matrix):
    """Convert transformation matrix to position and Euler angles (from questVR_ws)"""
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]

def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    """Create transformation matrix from position and Euler angles (from questVR_ws)"""
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    return transformation_matrix

def calc_pose_incre(base_pose, pose_data):
    """Calculate incremental pose (from questVR_ws teleop_single_piper.py)"""
    begin_matrix = create_transformation_matrix(base_pose[0], base_pose[1], base_pose[2],
                                                base_pose[3], base_pose[4], base_pose[5])
    zero_matrix = create_transformation_matrix(0.19, 0.0, 0.2, 0, 0, 0)
    end_matrix = create_transformation_matrix(pose_data[0], pose_data[1], pose_data[2],
                                            pose_data[3], pose_data[4], pose_data[5])
    result_matrix = np.dot(zero_matrix, np.dot(np.linalg.inv(begin_matrix), end_matrix))
    xyzrpy = matrix_to_xyzrpy(result_matrix)
    return xyzrpy

def adjustment_matrix(transform):
    """Adjust VR coordinate frame to robot frame (from questVR_ws VR class)"""
    if transform.shape != (4, 4):
        if len(transform) == 16:
            transform = np.array(transform).reshape(4, 4)
        else:
            raise ValueError("Input transform must be a 4x4 numpy array or 16-element array.")
    
    adj_mat = np.array([
        [0,0,-1,0],
        [-1,0,0,0],
        [0,1,0,0],
        [0,0,0,1]
    ])
    
    r_adj = create_transformation_matrix(0,0,0, -np.pi, 0, -np.pi/2)
    
    transform = adj_mat @ transform  
    transform = np.dot(transform, r_adj)  
    
    return transform

def quaternion_from_matrix(matrix):
    """Convert rotation matrix to quaternion"""
    w = np.sqrt(1.0 + matrix[0,0] + matrix[1,1] + matrix[2,2]) / 2.0
    w4 = (4.0 * w)
    x = (matrix[2,1] - matrix[1,2]) / w4
    y = (matrix[0,2] - matrix[2,0]) / w4
    z = (matrix[1,0] - matrix[0,1]) / w4
    return [x, y, z, w]

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion"""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [x, y, z, w]

class Arm_IK:
    """Exact replication of questVR_ws Arm_IK class using Pinocchio + CasADi"""
    
    def __init__(self):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        
        # Load URDF - try to use the exact same URDF as questVR_ws
        urdf_paths = [
            "/home/dora/dora-pipers/questVR_ws/src/Piper_ros/src/piper_description/urdf/piper_description.urdf",
            os.path.join(os.path.dirname(__file__), "../urdf/piper_description.urdf"),
            "/home/dora/dora-pipers/quest_vr_dora/urdf/piper_accurate.urdf"
        ]
        
        urdf_path = None
        for path in urdf_paths:
            if os.path.exists(path):
                urdf_path = path
                logger.info(f"Found URDF at: {path}")
                break
        
        if urdf_path is None:
            logger.error("No URDF file found! Creating simplified model...")
            self.robot = self._create_simple_robot()
        else:
            # Build without visual/collision models to avoid mesh path issues
            import pinocchio as pin
            model = pin.buildModelFromUrdf(urdf_path)
            self.robot = pin.RobotWrapper(model)
        
        # Lock gripper joints (joint7, joint8) just like in questVR_ws
        self.mixed_jointsToLockIDs = ["joint7", "joint8"]
        
        try:
            self.reduced_robot = self.robot.buildReducedRobot(
                list_of_joints_to_lock=self.mixed_jointsToLockIDs,
                reference_configuration=np.array([0] * self.robot.model.nq),
            )
        except:
            # If joint names don't match, use the robot as-is
            logger.warning("Could not lock gripper joints, using full robot model")
            self.reduced_robot = self.robot
        
        # Add end effector frame exactly as in questVR_ws
        self.first_matrix = create_transformation_matrix(0, 0, 0, 0, -1.57, 0)
        self.second_matrix = create_transformation_matrix(0.13, 0.0, 0.0, 0, 0, 0)
        self.last_matrix = np.dot(self.first_matrix, self.second_matrix)
        q = quaternion_from_matrix(self.last_matrix)
        
        try:
            joint6_id = self.reduced_robot.model.getJointId('joint6')
        except:
            joint6_id = min(5, self.reduced_robot.model.njoints - 1)
        
        self.reduced_robot.model.addFrame(
            pin.Frame('ee',
                      joint6_id,
                      pin.SE3(
                          pin.Quaternion(q[3], q[0], q[1], q[2]),
                          np.array([self.last_matrix[0, 3], self.last_matrix[1, 3], self.last_matrix[2, 3]]),
                      ),
                      pin.FrameType.OP_FRAME)
        )
        
        # Initialize data
        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)
        
        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        
        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        
        # Get the hand joint ID and define the error function
        self.gripper_id = self.reduced_robot.model.getFrameId("ee")
        self.error = casadi.Function(
            "error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log6(
                        self.cdata.oMf[self.gripper_id].inverse() * cpin.SE3(self.cTf)
                    ).vector,
                )
            ],
        )
        
        # Defining the optimization problem (exact same as questVR_ws)
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.param_tf = self.opti.parameter(4, 4)
        
        # Error weighting (exact same as questVR_ws)
        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = error_vec[:3]  
        ori_error = error_vec[3:]  
        weight_position = 1.0  
        weight_orientation = 0.1  
        self.totalcost = casadi.sumsqr(weight_position * pos_error) + casadi.sumsqr(weight_orientation * ori_error)
        self.regularization = casadi.sumsqr(self.var_q)
        
        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        
        self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization)
        
        # Solver options (exact same as questVR_ws)
        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 50,
                'tol': 1e-4
            },
            'print_time': False
        }
        self.opti.solver("ipopt", opts)
        
        logger.info("✅ Exact Pinocchio+CasADi IK solver initialized (questVR_ws replication)")
    
    def _create_simple_robot(self):
        """Create a simple robot model as fallback"""
        model = pin.Model()
        
        # Create a simple 6-DOF serial chain
        parent = 0
        for i in range(6):
            joint_placement = pin.SE3(np.eye(3), np.array([0, 0, 0.1 if i == 0 else 0]))
            if i % 2 == 0:
                joint = pin.JointModelRZ()  # Revolute around Z
            else:
                joint = pin.JointModelRY()  # Revolute around Y
            parent = model.addJoint(parent, joint, joint_placement, f"joint{i+1}")
        
        # Add gripper joints (will be locked)
        for i in range(6, 8):
            joint_placement = pin.SE3(np.eye(3), np.array([0.05, 0, 0]))
            joint = pin.JointModelRY()
            parent = model.addJoint(parent, joint, joint_placement, f"joint{i+1}")
        
        robot = pin.RobotWrapper(model)
        return robot
    
    def ik_fun(self, target_pose, gripper=0, motorstate=None):
        """IK solving function - exact replication of questVR_ws"""
        gripper_joints = np.array([gripper/2.0, -gripper/2.0])
        
        if motorstate is not None:
            self.init_data = motorstate[:self.reduced_robot.model.nq]
        
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_tf, target_pose)
        
        try:
            sol = self.opti.solve_limited()
            sol_q = self.opti.value(self.var_q)
            
            if self.init_data is not None:
                max_diff = max(abs(self.history_data - sol_q))
                self.init_data = sol_q
                if max_diff > 30.0/180.0*3.1415:
                    logger.debug(f"Large joint change detected: {max_diff}")
                    self.init_data = np.zeros(self.reduced_robot.model.nq)
            else:
                self.init_data = sol_q
            self.history_data = sol_q
            
            return sol_q, True
            
        except Exception as e:
            logger.error(f"ERROR in convergence: {e}")
            return None, False

def process_exact_ik(ik_solver, transforms, buttons, base_RR):
    """Process IK using exact questVR_ws algorithm"""
    try:
        # Check buttons (exact same logic as questVR_ws)
        control_enabled = buttons.get('b', False)  # Button B enables control
        reset_requested = buttons.get('a', False)  # Button A resets position
        
        # Get right controller transform
        if 'r' not in transforms:
            logger.warning("No right hand transform available")
            return None, base_RR
        
        right_transform = transforms['r']
        
        # Convert to 4x4 matrix if needed
        if len(right_transform) == 16:
            right_transform = np.array(right_transform).reshape(4, 4)
        
        # Apply adjustment matrix (exact same as questVR_ws VR class)
        right_transform = adjustment_matrix(right_transform)
        
        # Extract pose
        RR = matrix_to_xyzrpy(right_transform)
        
        if reset_requested:
            # Reset position and update base reference (exact same as questVR_ws)
            logger.info("Reset requested - returning to init pose")
            base_RR = RR.copy()  # Update base reference
            
            # Return zero position with gripper control
            gripper_value = buttons.get('right_trigger', 0.0) * 0.07
            return {
                'joint1': 0.0,
                'joint2': 0.0,
                'joint3': 0.0,
                'joint4': 0.0,
                'joint5': 0.0,
                'joint6': 0.0,
                'gripper': gripper_value,
                'enabled': False,
                'reset_requested': True
            }, base_RR
        
        if control_enabled:
            # Calculate incremental pose (exact same as questVR_ws)
            RR_ = calc_pose_incre(base_RR, RR)
            
            # Create target pose
            q = quaternion_from_euler(RR_[3], RR_[4], RR_[5])
            target = pin.SE3(
                pin.Quaternion(q[3], q[0], q[1], q[2]),
                np.array([RR_[0], RR_[1], RR_[2]]),
            )
            
            # Get gripper value (exact same scaling as questVR_ws)
            gripper_value = float(buttons.get('right_trigger', 0.0)) * 0.07
            
            # Solve IK
            sol_q, success = ik_solver.ik_fun(target.homogeneous, gripper_value)
            
            if success and sol_q is not None:
                # Return joint commands
                return {
                    'joint1': float(sol_q[0]) if len(sol_q) > 0 else 0.0,
                    'joint2': float(sol_q[1]) if len(sol_q) > 1 else 0.0,
                    'joint3': float(sol_q[2]) if len(sol_q) > 2 else 0.0,
                    'joint4': float(sol_q[3]) if len(sol_q) > 3 else 0.0,
                    'joint5': float(sol_q[4]) if len(sol_q) > 4 else 0.0,
                    'joint6': float(sol_q[5]) if len(sol_q) > 5 else 0.0,
                    'gripper': float(gripper_value),
                    'enabled': True,
                    'reset_requested': False
                }, base_RR
            else:
                logger.warning("IK solution failed")
                return None, base_RR
        
        else:
            # Not enabled - just control gripper
            gripper_value = buttons.get('right_trigger', 0.0) * 0.07
            return {
                'joint1': 0.0,
                'joint2': 0.0,
                'joint3': 0.0,
                'joint4': 0.0,
                'joint5': 0.0,
                'joint6': 0.0,
                'gripper': gripper_value,
                'enabled': False,
                'reset_requested': False
            }, base_RR
            
    except Exception as e:
        logger.error(f"Error in exact IK processing: {e}", exc_info=True)
        return None, base_RR

def main():
    logger.info("🚀 Starting Exact IK Solver Node (questVR_ws replication)")
    
    # Initialize IK solver
    ik_solver = Arm_IK()
    
    # Base reference pose (exact same as questVR_ws)
    base_RR = [0.19, 0.0, 0.2, 0, 0, 0]
    
    command_count = 0
    last_transforms = None
    last_buttons = None
    
    # Get node ID from environment variable or use default
    # node_id = os.getenv("DORA_NODE_ID", "ik_solver_bak")
    # node = Node(node_id)
    node = Node("ik_solver_bak")
    
    try:
        while True:
            event = node.next()
            
            if event is None:
                continue
            
            if event["type"] == "STOP":
                logger.info(f"🛑 Received STOP signal (sent {command_count} commands)")
                break
            
            if event["type"] == "INPUT":
                if event["id"] == "transforms":
                    logger.debug("Received transforms data")
                    array = event["value"]
                    transforms = {}
                    
                    try:
                        # Handle PyArrow data
                        if hasattr(array, '__len__') and len(array) > 0:
                            transform_dict = array[0].as_py()
                        elif hasattr(array, 'to_pylist'):
                            data_list = array.to_pylist()
                            if data_list and len(data_list) > 0:
                                transform_dict = data_list[0]
                            else:
                                continue
                        else:
                            continue
                        
                        if 'left' in transform_dict and transform_dict['left'] != [0.0] * 16:
                            transforms['l'] = transform_dict['left']
                        if 'right' in transform_dict and transform_dict['right'] != [0.0] * 16:
                            transforms['r'] = transform_dict['right']
                            
                    except Exception as e:
                        logger.error(f"Error processing transforms: {e}")
                        continue
                    
                    last_transforms = transforms
                    
                    # Process IK immediately after receiving transforms if we have buttons
                    if last_transforms is not None and last_buttons is not None:
                        joint_commands, base_RR = process_exact_ik(
                            ik_solver,
                            last_transforms,
                            last_buttons,
                            base_RR
                        )
                        
                        if joint_commands is not None:
                            command_count += 1
                            
                            # Debug: Log gripper values
                            if joint_commands.get('gripper', 0.0) > 0.01:
                                logger.debug(f"📤 Sending gripper value: {joint_commands['gripper']:.3f}")
                            
                            # Debug: Log full command dict before sending
                            logger.debug(f"📤 Full joint_commands dict being sent: {joint_commands}")
                            
                            # Send joint commands
                            command_array = pa.array([joint_commands])
                            node.send_output(
                                "joint_commands",
                                command_array,
                                {"timestamp": time.time()}
                            )
                            
                            if joint_commands['enabled']:
                                if command_count % 10 == 0:
                                    logger.info(f"🤖 Control active #{command_count}")
                            elif joint_commands.get('gripper', 0.0) > 0.01:
                                logger.debug(f"🔧 Gripper-only mode: {joint_commands['gripper']:.3f}")
                    
                elif event["id"] == "buttons":
                    logger.debug("Received buttons data")
                    array = event["value"]
                    
                    try:
                        # Handle PyArrow data
                        if hasattr(array, '__len__') and len(array) > 0:
                            button_dict = array[0].as_py()
                        elif hasattr(array, 'to_pylist'):
                            data_list = array.to_pylist()
                            if data_list and len(data_list) > 0:
                                button_dict = data_list[0]
                            else:
                                continue
                        else:
                            continue
                        
                        last_buttons = {
                            'left_trigger': button_dict.get('leftTrig', [0.0])[0] if isinstance(button_dict.get('leftTrig', 0.0), list) else button_dict.get('leftTrig', 0.0),
                            'right_trigger': button_dict.get('rightTrig', [0.0])[0] if isinstance(button_dict.get('rightTrig', 0.0), list) else button_dict.get('rightTrig', 0.0),
                            'left_grip': button_dict.get('leftGrip', [0.0])[0] if isinstance(button_dict.get('leftGrip', 0.0), list) else button_dict.get('leftGrip', 0.0),
                            'right_grip': button_dict.get('rightGrip', [0.0])[0] if isinstance(button_dict.get('rightGrip', 0.0), list) else button_dict.get('rightGrip', 0.0),
                            'a': button_dict.get('A', False),
                            'b': button_dict.get('B', False),
                            'x': button_dict.get('X', False),
                            'y': button_dict.get('Y', False),
                        }
                        
                        # Log button presses
                        if last_buttons['a']:
                            logger.warning("🔴 BUTTON A PRESSED - RESET!")
                        if last_buttons['b']:
                            logger.warning("🟢 BUTTON B PRESSED - CONTROL ENABLED!")
                        if last_buttons['right_trigger'] > 0.1:
                            logger.debug(f"🟡 RIGHT TRIGGER: {last_buttons['right_trigger']:.2f}")
                        
                    except Exception as e:
                        logger.error(f"Error processing buttons: {e}")
                        continue
                    
                    # Process IK immediately after receiving buttons if we have transforms
                    if last_transforms is not None and last_buttons is not None:
                        joint_commands, base_RR = process_exact_ik(
                            ik_solver,
                            last_transforms,
                            last_buttons,
                            base_RR
                        )
                        
                        if joint_commands is not None:
                            command_count += 1
                            
                            # Debug: Log gripper values
                            if joint_commands.get('gripper', 0.0) > 0.01:
                                logger.debug(f"📤 Sending gripper value: {joint_commands['gripper']:.3f}")
                            
                            # Debug: Log full command dict before sending
                            logger.debug(f"📤 Full joint_commands dict being sent: {joint_commands}")
                            
                            # Send joint commands
                            command_array = pa.array([joint_commands])
                            node.send_output(
                                "joint_commands",
                                command_array,
                                {"timestamp": time.time()}
                            )
                            
                            if joint_commands['enabled']:
                                if command_count % 10 == 0:
                                    logger.info(f"🤖 Control active #{command_count}")
                            elif joint_commands.get('gripper', 0.0) > 0.01:
                                logger.debug(f"🔧 Gripper-only mode: {joint_commands['gripper']:.3f}")
                            
    except KeyboardInterrupt:
        logger.info("🛑 IK Solver stopped by user")
    except Exception as e:
        logger.error(f"❌ IK Solver error: {e}", exc_info=True)
    finally:
        logger.info(f"⏹️ IK Solver stopped (sent {command_count} commands)")

if __name__ == "__main__":
    main()
