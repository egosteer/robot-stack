#!/usr/bin/env python3

import logging
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

try:
    from hand.ry_hand_controller import RuiyanHandController, RuiyanInstructionType
    from hand.ry_hand_interface import RuiyanFingerStatusMessage, SerialInterface
    from hand.Ruckig_Interpolator import SmoothJointInterpolator
except ImportError:
    from .ry_hand_controller import RuiyanHandController, RuiyanInstructionType
    from .ry_hand_interface import RuiyanFingerStatusMessage, SerialInterface
    from .Ruckig_Interpolator import SmoothJointInterpolator


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger(__name__)


class HandControlNode(Node):
    """ROS2 Node for controlling a single Ruiyan hand via RS485"""
    
    def __init__(self):
        super().__init__('hand_control_node')
        
        # Declare parameters
        self.declare_parameter("hand_side", "left")  # 'left' or 'right'
        self.declare_parameter("frequency", 80.0)
        self.declare_parameter("serial_port", "/dev/hand_left")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("enable_interpolation", True)  # Enable Ruckig interpolation
        
        # Get parameters
        self.hand_side = (
            self.get_parameter("hand_side")
            .get_parameter_value()
            .string_value
        )
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        self.enable_interpolation = (
            self.get_parameter("enable_interpolation")
            .get_parameter_value()
            .bool_value
        )
        self.dof = 6  # Degrees of freedom
        
        # Initialize hand controller
        hand_interface = SerialInterface(
            port=serial_port,
            baudrate=baudrate,
            mock=False,
            auto_connect=True,
        )
        self.hand = RuiyanHandController(
            hand_interface,
            motors_id=[1, 2, 3, 4, 5, 6],
            instruction=RuiyanInstructionType(int('0xAA', 16)),
        )
        
        self.hand.set(
            velocity_list=[3000] * self.dof,
            current_list=[1000] * self.dof,
        )
        
        # Create publishers and subscribers
        self.joint_states_publisher = self.create_publisher(
            JointState, 
            f"/state/{self.hand_side}_hand/joints", 
            10
        )
        self.set_angles_subscription = self.create_subscription(
            JointState,
            f"/action/{self.hand_side}_hand/joints",
            self.set_angles_callback,
            10,
        )
        
        self.get_logger().info(
            f"Ruiyan {self.hand_side} hand node started with frequency: {self.frequency} Hz"
        )
        self.get_logger().info(f"Serial port: {serial_port}, baudrate: {baudrate}")
        self.get_logger().info(f"Interpolation: {'enabled' if self.enable_interpolation else 'disabled'}")
        
        # Initialize interpolator if enabled
        if self.enable_interpolation:
            self._init_interpolator()
        else:
            self.interpolator = None
        
        # Initialize target command storage
        self.target_angles = None
        
        # Create timer
        self.create_timer(1.0 / self.frequency, self.loop)
    
    def _init_interpolator(self):
        """Initialize Ruckig smooth joint interpolator"""
        dt = 1.0 / self.frequency
        
        self.interpolator = SmoothJointInterpolator(
            dof=self.dof,
            step=dt,
            alpha=0.5
        )
        
        # Set kinematic limits (tuned for the lower RS485 update rate).
        max_velocity = [200,] * self.dof
        max_acceleration = [400,] * self.dof
        max_jerk = [400,] * self.dof
        
        self.interpolator.set_kinematic_limits(
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            max_jerk=max_jerk
        )
        
        # Initialize interpolator with zero position
        self.interpolator.set_input_param(
            current_position=np.zeros(self.dof),
            current_velocity=np.zeros(self.dof),
            current_acceleration=np.zeros(self.dof)
        )
        
        self.get_logger().info(
            f"Ruckig interpolator initialized for {self.dof} DOF "
            f"with {self.frequency} Hz control frequency"
        )
    
    def _status_list_to_joint_state(
        self, status_list: List[RuiyanFingerStatusMessage]
    ) -> JointState:
        """Convert motor status list to JointState message"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = f"{self.hand_side}_hand_base_link"
        
        # Filter None values and sort by motor_id
        valid_status_list = [status for status in status_list if status is not None]
        sorted_status = sorted(valid_status_list, key=lambda x: x.motor_id)
        
        # Define motor_id to joint name mapping
        joint_name_mapping = {
            1: "thumb_rotate",
            2: "thumb_bend",
            3: "index_bend",
            4: "middle_bend",
            5: "ring_bend",
            6: "pinky_bend"
        }
        
        joint_names = []
        positions = []
        velocities = []
        efforts = []
        
        for status in sorted_status:
            joint_name = joint_name_mapping.get(
                status.motor_id, f"joint_{status.motor_id}"
            )
            joint_names.append(joint_name)
            positions.append(float(status.position or 0) / 4095)
            velocities.append(float(status.velocity or 0))
            efforts.append(float(status.current or 0))
        
        joint_state.name = joint_names
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        
        return joint_state
    
    def loop(self):
        """Main control loop"""
        try:
            # Apply interpolation and send commands if we have a target
            if self.target_angles is not None:
                if self.enable_interpolation and self.interpolator is not None:
                    # Use Ruckig interpolator for smooth motion
                    smoothed_angles, _, _, _ = self.interpolator.update(target_pos=self.target_angles)
                    if isinstance(smoothed_angles, np.ndarray):
                        smoothed_angles = smoothed_angles.tolist()
                else:
                    # Direct control without interpolation
                    smoothed_angles = self.target_angles.tolist()
                
                # Convert to motor positions (0-4095) and send commands
                self.hand.set(position_list=[int(p * 4095) for p in smoothed_angles])
            
            # Read status and publish
            status_list = self.hand.loop()
            if status_list:  # Only publish if we have valid data
                joint_state = self._status_list_to_joint_state(status_list)
                self.joint_states_publisher.publish(joint_state)
        except Exception as e:
            logger.warning(
                f"{self.hand_side.capitalize()} hand loop failed: {e}. "
                "Skipping this publish cycle."
            )
    
    def set_angles_callback(self, msg: JointState):
        """Callback for setting target hand angles"""
        # Save target angles (interpolation will be done in loop)
        self.target_angles = np.array(msg.position)


def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Node manually stopped")
    except Exception:
        if rclpy.ok():
            raise
    finally:
        # Cleanup interpolator
        if hasattr(node, 'interpolator') and node.interpolator is not None:
            node.interpolator.close()
            logger.info("Interpolator closed")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
