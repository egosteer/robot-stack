#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import mujoco
from pathlib import Path
import threading
from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


# Inverse of the multiplier in hand_ik_node.publish_joints — undoes it so q_FK == q_IK.
# Must stay in sync with hand_ik_node.MOTOR_POSITION_MULTIPLIER.
MOTOR_POSITION_MULTIPLIER = [0.6, 1.0, 1.0, 1.0, 1.0, 1.0]


def _assets_dir() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / 'assets').is_dir():
            return parent / 'assets'
    return Path('/root/workspace/robot-stack/assets')


class HandFKSolver:
    """Dexterous hand forward-kinematics solver, optimized for 80Hz real-time feedback."""
    def __init__(self, hand_type='left'):
        self.hand_type = hand_type
        self.data_lock = threading.Lock()

        mjcf_file = _assets_dir() / 'ruiyan_hand_mjcf' / hand_type / 'hand.xml'

        self.model = mujoco.MjModel.from_xml_path(str(mjcf_file))
        self.data = mujoco.MjData(self.model)

        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']

        # Cache core info up front to avoid string lookups in the loop.
        self._setup_joint_info()
        self._record_joint_metadata()
        self._setup_site_info()

        with self.data_lock:
            mujoco.mj_forward(self.model, self.data)

    def _setup_joint_info(self):
        all_joints = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)]
        prefix = 'hand1' if self.hand_type == 'left' else 'hand2'
        active_candidates = [
            f'{prefix}_joint_link_1_1', f'{prefix}_joint_link_1_2',
            f'{prefix}_joint_link_2_1', f'{prefix}_joint_link_3_1',
            f'{prefix}_joint_link_4_1', f'{prefix}_joint_link_5_1',
        ]
        self.joint_names = [j for j in active_candidates if j in all_joints]

    def _record_joint_metadata(self):
        # Pre-cache joint qpos addresses and limit ranges for performance.
        self.joint_metadata = []
        for name in self.joint_names:
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qpos_adr = self.model.jnt_qposadr[jnt_id]
            lower, upper = self.model.jnt_range[jnt_id]
            
            self.joint_metadata.append({
                'name': name,
                'adr': qpos_adr,
                'low': lower,
                'range': (upper - lower) if (upper - lower) != 0 else 1.0
            })
        print(f"[FK Solver] {self.hand_type.upper()} cached {len(self.joint_metadata)} active joints")

    def _setup_site_info(self):
        side_prefix = self.hand_type
        self.fingertip_sites = [f'{side_prefix}_{name}_tip' for name in self.finger_names]
        self.site_ids_list = []
        for site_name in self.fingertip_sites:
            try:
                self.site_ids_list.append(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name))
            except:
                pass

    def _get_mimic_relations(self):
        # (leader, follower, multiplier, offset) coupled-joint relations.
        p = 'hand1' if self.hand_type == 'left' else 'hand2'
        return [
            (f'{p}_joint_link_1_2', f'{p}_joint_link_1_3', 1.675, 0.0),
            (f'{p}_joint_link_2_1', f'{p}_joint_link_2_2', 1.0, 0.0),
            (f'{p}_joint_link_3_1', f'{p}_joint_link_3_2', 1.0, 0.0),
            (f'{p}_joint_link_4_1', f'{p}_joint_link_4_2', 1.0, 0.0),
            (f'{p}_joint_link_5_1', f'{p}_joint_link_5_2', 1.0, 0.0),
        ]

    def _apply_mimic_joints(self):
        """Manually apply coupling constraints to qpos."""
        with self.data_lock:
            for leader_name, follower_name, multiplier, offset in self._get_mimic_relations():
                try:
                    l_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, leader_name)
                    f_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, follower_name)
                    l_adr = self.model.jnt_qposadr[l_id]
                    f_adr = self.model.jnt_qposadr[f_id]
                    self.data.qpos[f_adr] = offset + multiplier * self.data.qpos[l_adr]
                except:
                    pass

    def compute_fk(self, normalized_positions):
        """
        Compute FK from normalized joint angles.
        normalized_positions: length-6 array in [0.0, 1.0].
        """
        if len(normalized_positions) != len(self.joint_metadata):
            return None

        adjusted = np.clip(
            np.asarray(normalized_positions, dtype=np.float64) / np.asarray(MOTOR_POSITION_MULTIPLIER),
            0.0,
            1.0,
        )

        with self.data_lock:
            # Map normalized values to physical radians.
            for i, m in enumerate(self.joint_metadata):
                val_rad = m['low'] + adjusted[i] * m['range']
                self.data.qpos[m['adr']] = val_rad

        self._apply_mimic_joints()

        with self.data_lock:
            mujoco.mj_forward(self.model, self.data)

            # Extract fingertip site poses.
            results = []
            for s_id in self.site_ids_list:
                pos = self.data.site(s_id).xpos.copy()
                xmat = self.data.site(s_id).xmat.copy().reshape(3, 3)
                quat = R.from_matrix(xmat).as_quat() # [x, y, z, w]
                results.append((pos, quat))
        return results


class HandFKNode(Node):
    def __init__(self):
        super().__init__('hand_fk_node')

        self.declare_parameter('hand_side', 'left')
        self.declare_parameter('frequency', 80.0)
        self.hand_side = self.get_parameter('hand_side').value
        self.frequency = self.get_parameter('frequency').value

        self.get_logger().info(f"Initializing {self.hand_side.upper()} Hand FK Node (Timer-driven {self.frequency}Hz)")

        self.fk_solver = HandFKSolver(hand_type=self.hand_side)

        self.latest_normalized_joints = None  # most recent measured joint angles
        self.data_lock = threading.Lock()     # guards the joint-angle cache

        # Subscribe to measured feedback from Hand Control Node (position is 0.0~1.0).
        self.sub_joint_states = self.create_subscription(
            JointState,
            f'/state/{self.hand_side}_hand/joints',
            self.joint_state_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Publish fingertip poses to the Interface / Model.
        self.pub_keypoints = self.create_publisher(
            PoseArray,
            f'/state/{self.hand_side}_hand/keypoints',
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Timer: run FK and publish at a fixed rate.
        self.timer = self.create_timer(
            1.0 / self.frequency, 
            self.control_loop, 
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def joint_state_callback(self, msg: JointState):
        """Only caches the latest hardware feedback; no heavy computation here."""
        if msg.position and len(msg.position) == 6:
            with self.data_lock:
                self.latest_normalized_joints = np.array(msg.position)

    def control_loop(self):
        """Fixed-rate FK computation loop."""
        with self.data_lock:
            current_joints = self.latest_normalized_joints

        # Skip publishing keypoints until the first hardware feedback arrives.
        if current_joints is None:
            return

        # compute_fk handles denormalization, mimic coupling, and mj_forward internally.
        fingertip_poses = self.fk_solver.compute_fk(current_joints)

        if fingertip_poses:
            self.publish_keypoints(fingertip_poses)

    def publish_keypoints(self, poses_data):
        """Convert the pose array to a PoseArray and publish it."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.hand_side}_hand_base"

        for pos, quat in poses_data:
            p = Pose()
            p.position.x = float(pos[0])
            p.position.y = float(pos[1])
            p.position.z = float(pos[2])
            # quat order [x, y, z, w]
            p.orientation.x = float(quat[0])
            p.orientation.y = float(quat[1])
            p.orientation.z = float(quat[2])
            p.orientation.w = float(quat[3])
            msg.poses.append(p)

        self.pub_keypoints.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = HandFKNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()