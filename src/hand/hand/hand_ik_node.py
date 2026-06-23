#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
import mujoco
import mink
from pathlib import Path
import threading
from scipy.spatial.transform import Rotation as R


RUIYAN_HAND_JOINT_NAMES = [
    'thumb_rotate',
    'thumb_bend',
    'index_bend',
    'middle_bend',
    'ring_bend',
    'pinky_bend',
]

# motor_cmd = normalized_qpos * MOTOR_POSITION_MULTIPLIER.
# Must stay in sync with the same constant in hand_fk_node.py (which applies the inverse).
MOTOR_POSITION_MULTIPLIER = [0.6, 1.0, 1.0, 1.0, 1.0, 1.0]


def _assets_dir() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / 'assets').is_dir():
            return parent / 'assets'
    return Path('/root/workspace/robot-stack/assets')


class HandIKSolver:
    """Dexterous hand inverse-kinematics solver; mocap-stable variant tuned for 80Hz real-time control."""
    def __init__(self, hand_type='left', solver='daqp', frequency=80.0):
        self.hand_type = hand_type
        self.solver = solver
        self.dt = 1.0 / frequency
        self.data_lock = threading.RLock()

        mjcf_file = _assets_dir() / 'ruiyan_hand_mjcf' / hand_type / 'hand.xml'

        self.model = mujoco.MjModel.from_xml_path(str(mjcf_file))
        self.configuration = mink.Configuration(self.model)

        self.finger_names =['thumb', 'index', 'middle', 'ring', 'pinky']

        # Cache all core info up front.
        self._setup_joint_info()
        self._record_joint_metadata()
        self._setup_site_info()
        self._setup_mocap_targets()
        self._setup_tasks()

        self.limits = [mink.ConfigurationLimit(self.model)]

        self.reset_to_home()

    def _setup_joint_info(self):
        all_joints =[mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)]
        prefix = 'hand1' if self.hand_type == 'left' else 'hand2'
        active_candidates =[
            f'{prefix}_joint_link_1_1', f'{prefix}_joint_link_1_2',
            f'{prefix}_joint_link_2_1', f'{prefix}_joint_link_3_1',
            f'{prefix}_joint_link_4_1', f'{prefix}_joint_link_5_1',
        ]
        self.joint_names = [j for j in active_candidates if j in all_joints]

    def _record_joint_metadata(self):
        self.joint_metadata =[]
        for name in self.joint_names:
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qpos_adr = self.model.jnt_qposadr[jnt_id]
            lower, upper = self.model.jnt_range[jnt_id]
            
            self.joint_metadata.append({
                'name': name,
                'id': jnt_id,
                'adr': qpos_adr,
                'low': lower,
                'high': upper,
                'range': (upper - lower) if (upper - lower) != 0 else 1.0
            })
        
        self.limited_joint_adrs = [self.model.jnt_qposadr[i] for i in range(self.model.njnt) if self.model.jnt_limited[i]]
        self.limited_joint_ranges = [self.model.jnt_range[i] for i in range(self.model.njnt) if self.model.jnt_limited[i]]
        print(f"[IK Solver] {self.hand_type.upper()} hand initialized, cached limits for {len(self.joint_metadata)} active joints")

    def _setup_site_info(self):
        side_prefix = self.hand_type
        self.fingertip_sites =[f'{side_prefix}_{name}_tip' for name in self.finger_names]
        self.site_ids_list =[] 
        for site_name in self.fingertip_sites:
            try:
                self.site_ids_list.append(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name))
            except:
                pass

    def _setup_mocap_targets(self):
        """Maintain both a dict (lookup by name) and a list (fast iteration)."""
        mocap_body_names =[f'{name}_target' for name in self.finger_names]
        self.mocap_ids_list =[]
        self.mocap_ids = {}
        
        for finger_name, body_name in zip(self.finger_names, mocap_body_names):
            try:
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                mocap_id = self.model.body_mocapid[body_id]
                if mocap_id >= 0:
                    self.mocap_ids_list.append(mocap_id)
                    self.mocap_ids[finger_name] = mocap_id
            except:
                pass

    def _setup_tasks(self):
        self.tasks = []
        self.hand_tasks =[]
        for site_name in self.fingertip_sites:
            task = mink.FrameTask(
                frame_name=site_name,
                frame_type="site",
                position_cost=1000.0,
                orientation_cost=0.001,
                lm_damping=1.0,
            )
            self.hand_tasks.append(task)
        self.tasks.extend(self.hand_tasks)

    def _get_mimic_relations(self):
        p = 'hand1' if self.hand_type == 'left' else 'hand2'
        return[
            (f'{p}_joint_link_1_2', f'{p}_joint_link_1_3', 1.675, 0.0),
            (f'{p}_joint_link_2_1', f'{p}_joint_link_2_2', 1.0, 0.0),
            (f'{p}_joint_link_3_1', f'{p}_joint_link_3_2', 1.0, 0.0),
            (f'{p}_joint_link_4_1', f'{p}_joint_link_4_2', 1.0, 0.0),
            (f'{p}_joint_link_5_1', f'{p}_joint_link_5_2', 1.0, 0.0),
        ]

    def _couple_mimic_velocities(self, vel):
        vel = vel.copy()
        for leader_name, follower_name, multiplier, offset in self._get_mimic_relations():
            try:
                l_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, leader_name)
                f_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, follower_name)
                l_dof = self.model.jnt_dofadr[l_id]
                f_dof = self.model.jnt_dofadr[f_id]
                
                combined_vel = (vel[l_dof] + vel[f_dof]) / (1.0 + multiplier)
                vel[l_dof] = combined_vel
                vel[f_dof] = multiplier * combined_vel
            except:
                pass
        return vel

    def _apply_mimic_joints(self):
        with self.data_lock:
            for leader_name, follower_name, multiplier, offset in self._get_mimic_relations():
                try:
                    l_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, leader_name)
                    f_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, follower_name)
                    l_adr = self.model.jnt_qposadr[l_id]
                    f_adr = self.model.jnt_qposadr[f_id]
                    self.configuration.data.qpos[f_adr] = offset + multiplier * self.configuration.data.qpos[l_adr]
                except:
                    pass

    def _clamp_joint_limits(self):
        with self.data_lock:
            qpos = self.configuration.data.qpos
            for adr, (low, high) in zip(self.limited_joint_adrs, self.limited_joint_ranges):
                qpos[adr] = np.clip(qpos[adr], low, high)

    def reset_to_home(self):
        with self.data_lock:
            try:
                self.configuration.update_from_keyframe("home")
            except:
                self.configuration.q = np.zeros(self.model.nq)
            
            mujoco.mj_forward(self.model, self.configuration.data)
            self.sync_mocap_targets_from_current_fk()

    def update_mocap_targets(self, target_dict):
        with self.data_lock:
            for finger_name, pos in target_dict.items():
                if finger_name in self.mocap_ids:
                    mid = self.mocap_ids[finger_name]
                    self.configuration.data.mocap_pos[mid] = pos

    def sync_mocap_targets_from_current_fk(self):
        with self.data_lock:
            mujoco.mj_forward(self.model, self.configuration.data)
            for m_id, s_id in zip(self.mocap_ids_list, self.site_ids_list):
                site_pos = self.configuration.data.site(s_id).xpos.copy()
                site_quat = R.from_matrix(
                    self.configuration.data.site(s_id).xmat.copy().reshape(3, 3)
                ).as_quat(scalar_first=True)
                self.configuration.data.mocap_pos[m_id] = site_pos
                self.configuration.data.mocap_quat[m_id] = site_quat

    def set_active_joints_normalized(self, normalized_positions):
        normalized_positions = np.asarray(normalized_positions, dtype=np.float64)
        if len(normalized_positions) != len(self.joint_metadata):
            return None

        normalized_positions = np.clip(normalized_positions, 0.0, 1.0)

        with self.data_lock:
            for i, m in enumerate(self.joint_metadata):
                val_rad = m['low'] + normalized_positions[i] * m['range']
                self.configuration.data.qpos[m['adr']] = val_rad

        self._apply_mimic_joints()
        self._clamp_joint_limits()

        with self.data_lock:
            mujoco.mj_forward(self.model, self.configuration.data)

        self.sync_mocap_targets_from_current_fk()
        return self.get_active_joints_normalized()

    def step_ik_from_mocap(self, max_iterations=10):
        for i, (m_id, s_id) in enumerate(zip(self.mocap_ids_list, self.site_ids_list)):
            target_pos = self.configuration.data.mocap_pos[m_id].copy()
            with self.data_lock:
                rot_mat = self.configuration.data.site(s_id).xmat.reshape(3, 3).copy()

            transform = np.eye(4)
            transform[:3, :3] = rot_mat
            transform[:3, 3] = target_pos
            self.hand_tasks[i].set_target(mink.SE3.from_matrix(transform))

        for _ in range(max_iterations):
            with self.data_lock:
                mujoco.mj_forward(self.model, self.configuration.data)

            vel = mink.solve_ik(
                configuration=self.configuration,
                tasks=self.tasks,
                dt=self.dt,
                solver=self.solver,
                damping=1e-3,
                safety_break=False,
                limits=self.limits,
            )

            vel = self._couple_mimic_velocities(vel)
            self.configuration.integrate_inplace(vel, self.dt)
            self._apply_mimic_joints()
            self._clamp_joint_limits()

            if np.linalg.norm(vel) < 1e-8:
                break
                
        with self.data_lock:
            mujoco.mj_forward(self.model, self.configuration.data)

    def get_active_joints_normalized(self):
        with self.data_lock:
            qpos = self.configuration.data.qpos
            normalized = [
                np.clip((qpos[m['adr']] - m['low']) / m['range'], 0.0, 1.0)
                for m in self.joint_metadata
            ]
        return np.array(normalized)


class HandIKNode(Node):
    def __init__(self):
        super().__init__('hand_ik_node')

        self.declare_parameter('hand_side', 'left')
        self.declare_parameter('frequency', 80.0)
        # Standalone teleop data collection: the glove maps absolutely onto the hand (hand_joints = glove_joints).
        # Default False -> keep the relative mapping used by inference/HITL.
        self.declare_parameter('glove_absolute', False)

        self.hand_side = self.get_parameter('hand_side').value
        self.frequency = self.get_parameter('frequency').value
        self.glove_absolute = self.get_parameter('glove_absolute').value

        self.get_logger().info(f"Initializing {self.hand_side.upper()} Hand IK Node at {self.frequency}Hz")

        try:
            self.ik_solver = HandIKSolver(
                hand_type=self.hand_side,
                frequency=self.frequency
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load IK Solver: {e}")
            raise e

        self.current_mode = "execution"
        self.commander = "model"
        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
        self.current_model_keypoints = None
        self.current_hand_joints = self.ik_solver.get_active_joints_normalized()
        self.current_glove_joints = None
        self.reference_glove_joints = None
        self.reference_hand_joints = None
        
        self.pub_joints = self.create_publisher(
            JointState, 
            f'/action/{self.hand_side}_hand/joints', 
            10
        )

        self.sub_model_keypoints = self.create_subscription(
            PoseArray,
            f'/action/{self.hand_side}_hand/keypoints',
            self.keypoints_callback,
            10
        )

        self.sub_glove = self.create_subscription(
            JointState,
            f'/action/{self.hand_side}_glove/joints',
            self.glove_callback,
            10
        )

        self.sub_commander = self.create_subscription(
            String,
            '/commander',
            self.commander_callback,
            10
        )
        
        self.sub_mode = self.create_subscription(
            String,
            '/system/mode',
            self.mode_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / self.frequency, 
            self.control_loop
        )
        
        self.get_logger().info(f"{self.hand_side.upper()} Hand IK Node ready. Waiting for action chunk...")

    def mode_callback(self, msg: String):
        new_mode = msg.data.lower()
        if new_mode not in ["execution", "reset"]:
            return

        if self.current_mode != new_mode:
            self.get_logger().info(f"[{self.hand_side.upper()}] Mode Switched: {self.current_mode} -> {new_mode}")
        self.current_mode = new_mode

        if self.current_mode == "reset":
            self._reset_to_home()
            self.get_logger().info(f"[{self.hand_side.upper()}] Hand completely reset to Home posture.")

    def commander_callback(self, msg: String):
        commander = msg.data.strip().lower()
        if commander not in ["human", "model"]:
            self.get_logger().warn(f"[{self.hand_side.upper()}] Unknown commander: {msg.data}")
            return

        if self.commander == commander:
            return

        if commander == "human":
            assert self.current_glove_joints is not None, f"[{self.hand_side.upper()}] Glove joints are not available"
            self._capture_human_reference()
            self.commander = "human"
            self.get_logger().info(f"[{self.hand_side.upper()}] Commander switched to human glove control")
        else:
            self.commander = "model"
            self.get_logger().info(f"[{self.hand_side.upper()}] Commander switched to model control")
            self.current_model_keypoints = None
            self._clear_human_reference()

    def _clear_human_reference(self):
        self.reference_glove_joints = None
        self.reference_hand_joints = None

    def _capture_human_reference(self):
        self.reference_glove_joints = self.current_glove_joints.copy()
        self.current_hand_joints = self.ik_solver.get_active_joints_normalized()
        self.reference_hand_joints = self.current_hand_joints.copy()
        self.current_model_keypoints = None
        self.ik_solver.sync_mocap_targets_from_current_fk()
        self.get_logger().info(
            f"[{self.hand_side.upper()}] Human reference captured: glove={self.reference_glove_joints}, "
            f"joints={self.reference_hand_joints}"
        )

    def _reset_to_home(self):
        self.ik_solver.reset_to_home()
        self.current_hand_joints = self.ik_solver.get_active_joints_normalized()
        self.current_model_keypoints = None
        self.commander = "model"
        self._clear_human_reference()

    def keypoints_callback(self, msg: PoseArray):
        if self.current_mode != "execution":
            return

        if len(msg.poses) != 5:
            self.get_logger().warn(f"[{self.hand_side.upper()}] Expected 5 keypoints, got {len(msg.poses)}. Ignored.")
            return

        target_dict = {}
        for i, finger_name in enumerate(self.finger_names):
            pos = msg.poses[i].position
            target_dict[finger_name] = np.array([pos.x, pos.y, pos.z])

        self.current_model_keypoints = target_dict

    def glove_callback(self, msg: JointState):
        if not msg.position or len(msg.position) != len(self.current_hand_joints):
            return

        self.current_glove_joints = np.asarray(msg.position, dtype=np.float64)

    def publish_joints(self, joint_angles):
        assert len(joint_angles) == len(RUIYAN_HAND_JOINT_NAMES), (
            f"[{self.hand_side.upper()}] Expected {len(RUIYAN_HAND_JOINT_NAMES)} hand joints, "
            f"got {len(joint_angles)}"
        )
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(RUIYAN_HAND_JOINT_NAMES)
        position = [float(angle) * multiplier for angle, multiplier in zip(joint_angles, MOTOR_POSITION_MULTIPLIER)]
        msg.position = position
        self.pub_joints.publish(msg)

    def control_loop(self):
        if self.current_mode == "reset":
            self.publish_joints(self.current_hand_joints)
            return

        if self.commander == "human":
            assert self.current_glove_joints is not None, f"[{self.hand_side.upper()}] Glove joints are not available"

            if self.glove_absolute:
                # Standalone teleop: absolute mapping, hand joints = glove normalized values (no delta).
                target_joints = np.clip(self.current_glove_joints, 0.0, 1.0)
            else:
                # inference/HITL: relative (delta) mapping.
                assert self.reference_glove_joints is not None, f"[{self.hand_side.upper()}] Reference glove joints are not available"
                assert self.reference_hand_joints is not None, f"[{self.hand_side.upper()}] Reference hand joints are not available"
                target_joints = np.clip(
                    self.reference_hand_joints + (self.current_glove_joints - self.reference_glove_joints),
                    0.0,
                    1.0,
                )
            current_joints = self.ik_solver.set_active_joints_normalized(target_joints)
            if current_joints is not None:
                self.current_hand_joints = current_joints
            self.publish_joints(self.current_hand_joints)
            return

        if self.current_mode == "execution":
            if self.current_model_keypoints is not None:
                self.ik_solver.update_mocap_targets(self.current_model_keypoints)
                self.ik_solver.step_ik_from_mocap(max_iterations=10)
            self.current_hand_joints = self.ik_solver.get_active_joints_normalized()
            self.publish_joints(self.current_hand_joints)


def main(args=None):
    rclpy.init(args=args)
    node = HandIKNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
