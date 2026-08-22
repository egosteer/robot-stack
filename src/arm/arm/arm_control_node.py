#!/usr/bin/env python3

import os
import sys
import time
import signal
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from ruckig import InputParameter, OutputParameter, Result, Ruckig

# Add the RealMan SDK (assets/realman_arm_sdk) to sys.path.
_dir = os.path.dirname(os.path.abspath(__file__))
assets_dir = '/root/workspace/robot-stack/assets'
while _dir != os.path.dirname(_dir):
    candidate = os.path.join(_dir, 'assets')
    if os.path.isdir(candidate):
        assets_dir = candidate
        break
    _dir = os.path.dirname(_dir)
sys.path.insert(0, assets_dir)

try:
    from realman_arm_sdk import *
    print(f"✅ Imported RealMan SDK from {assets_dir}/realman_arm_sdk")
except ImportError as e:
    print(f"❌ Failed to import RealMan SDK: {e}")
    sys.exit(1)


@dataclass
class RealmanConfig:
    """Realman arm configuration."""
    ip: str = ""
    port: int = 8080
    thread_mode: int = 2  # RM_TRIPLE_MODE_E
    run_mode: int = 1     # 1=real robot, 0=simulation
    timeout_ms: int = 30000  # 30s timeout
    v_percent: float = 80.0
    a_percent: float = 100.0
    blend_r_deg: float = 0.0
    blocking: int = 0


class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')

        self.declare_parameter('arm_side', "")
        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('arm_state', 'api_gt')
        self.arm_side = self.get_parameter('arm_side').value
        self.frequency = self.get_parameter('frequency').value
        self.arm_state = self.get_parameter('arm_state').value
        self.dt = 1.0 / self.frequency

        if self.arm_state not in ['api_gt', 'ruckig']:
            raise ValueError(f"Unsupported arm_state: {self.arm_state}, must be one of ['api_gt', 'ruckig']")

        self.rm_config = RealmanConfig()
        if self.arm_side == "left":
            self.rm_config.ip = "192.168.100.100"
            self.arm_name = "Left arm"
            self.frame_id = "left_arm_base"
        elif self.arm_side == "right":
            self.rm_config.ip = "192.168.100.101"
            self.arm_name = "Right arm"
            self.frame_id = "right_arm_base"
        self.dof = 7
        arm_prefix = 'arm1' if self.arm_side == 'left' else 'arm2'
        self.joint_names = [f'{arm_prefix}_joint_link{i+1}' for i in range(self.dof)]

        if not self._init_robotic_arm():
            self.get_logger().error(f"❌ {self.arm_name} initialization failed")
            return

        self._init_ruckig()

        self.arm_command_sub = self.create_subscription(
            JointState,
            f'/action/{self.arm_side}_arm/joints',
            self.arm_command_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.joint_state_pub = None
        self.wrist_pose_pub = None
        if self.arm_state == 'ruckig':
            self.joint_state_pub = self.create_publisher(
                JointState,
                f'/state/{self.arm_side}_arm/joints',
                10,
                callback_group=MutuallyExclusiveCallbackGroup()
            )
        else:
            self.wrist_pose_pub = self.create_publisher(
                PoseStamped,
                f'/state/{self.arm_side}_arm/wrist_pose',
                10,
                callback_group=MutuallyExclusiveCallbackGroup()
            )

        self.control_timer = self.create_timer(
            self.dt,
            self.control_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.status_timer = self.create_timer(
            self.dt,
            self.publish_status,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        signal.signal(signal.SIGINT, self._signal_handler)

        self.get_logger().info(f"🤖 {self.arm_name} control node started (arm_state={self.arm_state})")

    def _init_robotic_arm(self) -> bool:
        """Initialize the robotic arm."""
        try:
            self.get_logger().info(f"🔌 Connecting to {self.arm_name}: {self.rm_config.ip}:{self.rm_config.port}")

            self.rm_arm = RoboticArm(self.rm_config.thread_mode)

            self.rm_arm.rm_set_timeout(self.rm_config.timeout_ms)

            self.rm_handle = self.rm_arm.rm_create_robot_arm(
                self.rm_config.ip,
                self.rm_config.port,
                level=2  # warning level
            )

            if self.rm_handle.id <= 0:
                self.get_logger().error(f"❌ {self.arm_name} connection failed, handle ID: {self.rm_handle.id}")
                return False

            self.get_logger().info(f"✅ {self.arm_name} connected, handle ID: {self.rm_handle.id}")

            self.rm_arm.rm_set_arm_run_mode(self.rm_config.run_mode)

            power_state = self.rm_arm.rm_get_arm_power_state()
            if isinstance(power_state, tuple) and len(power_state) >= 2:
                if power_state[1] == 0:
                    self.get_logger().info(f"🔌 Powering on {self.arm_name}...")
                    self.rm_arm.rm_set_arm_power(1)
                    time.sleep(1)
                else:
                    self.get_logger().info(f"✅ {self.arm_name} already powered on")

            current_state = self.rm_arm.rm_get_current_arm_state()
            if isinstance(current_state, tuple) and current_state[0] == 0:
                joint_degrees = current_state[1]['joint'][:7]
                self.current_joint_positions = np.radians(joint_degrees)
                self.get_logger().info(f"✅ {self.arm_name} current position: {np.degrees(self.current_joint_positions)}")
            else:
                self.get_logger().warn(f"⚠️  Cannot read {self.arm_name} current position, using default")
                self.current_joint_positions = np.array([0.0813, -1.0521, 0.0675, -1.6824, -1.4881, 1.4717, -0.1232])
            self.target_joint_positions = None

            self.rm_connected = True
            return True

        except Exception as e:
            self.get_logger().error(f"❌ {self.arm_name} initialization error: {e}")
            import traceback
            self.get_logger().error(f"{self.arm_name} traceback: {traceback.format_exc()}")
            return False

    def _init_ruckig(self):
        """Initialize the Ruckig interpolator."""
        try:
            self.interpolator = Ruckig(self.dof, self.dt)
            self.input_param = InputParameter(self.dof)
            self.output_param = OutputParameter(self.dof)

            max_velocity = np.array([4, 4, 4, 4, 4, 4, 4])
            max_acceleration = np.array([10, 10, 10, 10, 10, 10, 10])
            max_jerk = np.array([100, 100, 100, 100, 100, 100, 100])

            self.input_param.max_velocity = max_velocity.tolist()
            self.input_param.max_acceleration = max_acceleration.tolist()
            self.input_param.max_jerk = max_jerk.tolist()

            if self.current_joint_positions is not None:
                self.input_param.current_position = self.current_joint_positions.tolist()
                self.input_param.current_velocity = [0.0] * self.dof
                self.input_param.current_acceleration = [0.0] * self.dof
                self.input_param.target_position = self.current_joint_positions.tolist()
                self.input_param.target_velocity = [0.0] * self.dof
                self.input_param.target_acceleration = [0.0] * self.dof

            self.get_logger().info(f"✅ Ruckig interpolator initialized ({self.dof}DOF, {self.frequency}Hz)")

        except Exception as e:
            self.get_logger().error(f"❌ Ruckig initialization error: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")

    def arm_command_callback(self, msg: JointState):
        """Handle joint commands."""
        self.target_joint_positions = np.array(msg.position)

    def control_callback(self):
        if self.target_joint_positions is not None and self.interpolator is not None:

            self.input_param.target_position = self.target_joint_positions.tolist()

            result = self.interpolator.update(self.input_param, self.output_param)

            if result == Result.Working or result == Result.Finished:
                interpolated_positions = np.array(self.output_param.new_position)

                success = self._send_to_arm(interpolated_positions)

                if success:
                    self.current_joint_positions = interpolated_positions

                    # Feed interpolator output back as input for the next step
                    self.input_param.current_position = self.output_param.new_position
                    self.input_param.current_velocity = self.output_param.new_velocity
                    self.input_param.current_acceleration = self.output_param.new_acceleration

    def _send_to_arm(self, joint_positions: np.ndarray) -> bool:
        """Send joint positions to the arm."""
        try:
            if not self.rm_connected or self.rm_arm is None:
                return False

            joint_degrees = np.degrees(joint_positions).tolist()

            result = self.rm_arm.rm_movej_canfd(
                joint_degrees,
                True,  # follow=True, high-follow mode
                0,     # trajectory_mode=0, full pass-through
                0      # radio disabled
            )

            if result == 0:
                return True
            else:
                if result == -1:  # communication failure
                    self.get_logger().warn("⚠️  Arm communication failed, retrying...")
                    time.sleep(0.01)
                    retry_result = self.rm_arm.rm_movej_canfd(joint_degrees, True, 0, 0)
                    return retry_result == 0
                else:
                    self.get_logger().warn(f"⚠️  {self.arm_name} command error: {result}")
                    return False

        except Exception as e:
            self.get_logger().error(f"❌ Error sending {self.arm_name} command: {e}")
            return False

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.name = list(self.joint_names)
        msg.position = self.current_joint_positions.tolist()
        msg.velocity = [0.0] * self.dof
        msg.effort = [0.0] * self.dof
        self.joint_state_pub.publish(msg)

    def _publish_wrist_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        _, state = self.rm_arm.rm_get_current_arm_state()
        pose = state['pose']
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        quat = R.from_euler('xyz', [pose[3], pose[4], pose[5]]).as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.wrist_pose_pub.publish(msg)

    def publish_status(self):
        """Publish arm state."""
        if self.arm_state == 'ruckig':
            self._publish_joint_state()
        else:
            self._publish_wrist_pose()

    def _signal_handler(self, signum, frame):
        self.get_logger().info("🛑 Interrupt received, cleaning up...")
        self._cleanup()
        rclpy.shutdown()

    def _cleanup(self):
        """Release resources."""
        try:
            if hasattr(self, 'control_timer'):
                self.control_timer.cancel()
            if hasattr(self, 'status_timer'):
                self.status_timer.cancel()

            if self.rm_arm is not None:
                try:
                    self.rm_arm.rm_delete_robot_arm()
                    self.get_logger().info("✅ Arm connection cleaned up")
                except:
                    pass

            try:
                RoboticArm.rm_destory()
                self.get_logger().info("✅ SDK state cleaned up")
            except:
                pass

            self.get_logger().info("✅ Cleanup complete")

        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ArmControlNode()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
