#!/usr/bin/env python3

import os
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState


def _default_robot_xml_path() -> str:
    current = Path(__file__).resolve()
    for parent in current.parents:
        candidate = parent / 'assets' / 'robot_mjcf' / 'robot_scene.xml'
        if candidate.exists():
            return str(candidate)
    return '/root/workspace/robot-stack/assets/robot_mjcf/robot_scene.xml'


class ArmFKNode(Node):
    def __init__(self):
        super().__init__('arm_fk_node')

        self.declare_parameter('arm_side', 'left')
        self.declare_parameter('robot_xml_path', _default_robot_xml_path())

        self.arm_side = self.get_parameter('arm_side').value
        self.robot_xml_path = self.get_parameter('robot_xml_path').value

        if self.arm_side not in ['left', 'right']:
            raise ValueError(f'Unsupported arm_side: {self.arm_side}')
        if not os.path.exists(self.robot_xml_path):
            raise FileNotFoundError(f'Arm FK XML not found: {self.robot_xml_path}')

        self.model = mujoco.MjModel.from_xml_path(self.robot_xml_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)

        if self.arm_side == 'left':
            self.joint_slice = slice(0, 7)
            self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'arm1_link0')
            self.wrist_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'left_wrist')
            self.frame_id = 'left_arm_base'
        else:
            self.joint_slice = slice(7, 14)
            self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'arm2_link0')
            self.wrist_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'right_wrist')
            self.frame_id = 'right_arm_base'

        self.pose_pub = self.create_publisher(PoseStamped, f'/state/{self.arm_side}_arm/wrist_pose', 10)
        self.create_subscription(JointState, f'/state/{self.arm_side}_arm/joints', self.joint_state_callback, 10)

        self.get_logger().info(f'ArmFKNode ready for {self.arm_side} arm, xml={self.robot_xml_path}')

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) != 7:
            self.get_logger().warn(f'Expected 7 joint angles, got {len(msg.position)}, ignoring.')
            return

        self.data.qpos[self.joint_slice] = np.asarray(msg.position, dtype=np.float64)
        mujoco.mj_forward(self.model, self.data)

        base_pos = self.data.body(self.base_id).xpos.copy()
        base_mat = self.data.body(self.base_id).xmat.copy().reshape(3, 3)
        wrist_pos_world = self.data.site(self.wrist_site_id).xpos.copy()
        wrist_mat_world = self.data.site(self.wrist_site_id).xmat.copy().reshape(3, 3)

        wrist_pos_in_base = base_mat.T @ (wrist_pos_world - base_pos)
        wrist_mat_in_base = base_mat.T @ wrist_mat_world
        wrist_quat = R.from_matrix(wrist_mat_in_base).as_quat()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = float(wrist_pos_in_base[0])
        pose_msg.pose.position.y = float(wrist_pos_in_base[1])
        pose_msg.pose.position.z = float(wrist_pos_in_base[2])
        pose_msg.pose.orientation.x = float(wrist_quat[0])
        pose_msg.pose.orientation.y = float(wrist_quat[1])
        pose_msg.pose.orientation.z = float(wrist_quat[2])
        pose_msg.pose.orientation.w = float(wrist_quat[3])
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmFKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
