#!/usr/bin/env python3

import os
import time
import threading

# Use the GLX rendering backend for MuJoCo (headless rendering)
os.environ['MUJOCO_GL'] = 'glx'

import mink
import numpy as np
from scipy.spatial.transform import Rotation as R
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class ArmIKNode(Node):
    def __init__(self):
        super().__init__('arm_ik_node')
        
        self.declare_parameter('enable_viewer', True)
        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('solver', 'daqp')
        self.declare_parameter('robot_xml_path', '')

        self.enable_viewer = self.get_parameter('enable_viewer').value
        self.frequency = self.get_parameter('frequency').value
        self.dt = 1.0 / self.frequency
        self.solver = self.get_parameter('solver').value

        # robot dual-arm config: wrist sites used as end-effectors
        self.hands = ['left_wrist', 'right_wrist']  # left/right wrist sites
        self.targets = ['left_wrist_target', 'right_wrist_target']
        self.bases = ['arm1_link0', 'arm2_link0']
        self.xml_path = self.get_parameter('robot_xml_path').value
        if self.xml_path == '':
            self.xml_path = "/root/workspace/robot-stack/assets/robot_mjcf/robot_scene.xml"

        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.configuration = mink.Configuration(self.model)
        self.data_lock = threading.Lock()

        self.left_arm_joint_names = [f'arm1_joint_link{i+1}' for i in range(7)]  # ARM1 = left arm
        self.right_arm_joint_names = [f'arm2_joint_link{i+1}' for i in range(7)]  # ARM2 = right arm

        self.mink_setup()

        self.viewer = None
        if self.enable_viewer:
            self._start_viewer()

        self._warm_up_sim()

        self._publish_count = 0
        self.mode = 'execution'
        self.commander = 'model'
        self.commander_lock = threading.Lock()
        self.current_tracker_poses = [None, None]
        self.reference_tracker_poses = [None, None]
        self.current_arm_poses = [pose.copy() for pose in self.initial_mocap_poses_base]
        self.reference_arm_poses = [None, None]

        self.wrist_poses_sub = self.create_subscription(
            PoseArray,
            '/action/both_arms/wrist_poses',
            self.wrist_poses_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.left_tracker_pose_sub = self.create_subscription(
            PoseStamped,
            '/action/left_tracker/pose',
            self.left_tracker_pose_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.right_tracker_pose_sub = self.create_subscription(
            PoseStamped,
            '/action/right_tracker/pose',
            self.right_tracker_pose_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.system_mode_sub = self.create_subscription(
            String,
            '/system/mode',
            self.system_mode_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.commander_sub = self.create_subscription(
            String,
            '/commander',
            self.commander_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.left_arm_command_pub = self.create_publisher(
            JointState, 
            '/action/left_arm/joints',
            10
        )

        self.right_arm_command_pub = self.create_publisher(
            JointState, 
            '/action/right_arm/joints',
            10
        )
        
        self.timer = self.create_timer(self.dt, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(f'ArmIKNode initialized with frequency: {self.frequency}Hz')
        self.get_logger().info(f'robot sites: {self.hands}')
        self.get_logger().info(f'robot targets: {self.targets}')
        self.get_logger().info(f'Viewer enabled: {self.enable_viewer}')

    def mink_setup(self):
        """Set up IK tasks for the robot dual-arm configuration."""
        # Joint damping cost, 14 DOF (both arms), equal for all joints
        damping_joints_cost = np.ones(self.model.nv) * 1.0
        self.damping_task = mink.DampingTask(self.model, cost=damping_joints_cost)

        self.kinetic_energy_task = mink.KineticEnergyRegularizationTask(cost=1e-3)
        self.kinetic_energy_task.set_dt(self.dt)

        self.posture_task = mink.PostureTask(self.model, cost=0.1)

        self.tasks = [
            self.damping_task,
            self.kinetic_energy_task,
            self.posture_task,
        ]

        # One frame task per hand
        self.hand_tasks = []
        for hand_frame_name in self.hands:
            task = mink.FrameTask(
                frame_name=hand_frame_name,
                frame_type="site",
                position_cost=5.0,
                orientation_cost=1.0,
                lm_damping=1.0,
            )
            self.hand_tasks.append(task)
        self.tasks.extend(self.hand_tasks)

        self.configuration.update_from_keyframe("home")  # update config from the "home" keyframe
        self.posture_task.set_target(self.configuration.q)
        self.initial_mocap_pos = {}
        self.initial_mocap_quat = {}
        self.initial_mocap_poses_base = []
        self.T_base2world_list = []
        with self.data_lock:
            mujoco.mj_forward(self.model, self.configuration.data)
            for i in range(len(self.hands)):
                hand = self.hands[i]
                site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, hand)
                site_xpos = self.configuration.data.site(site_id).xpos.copy()
                site_xmat = self.configuration.data.site(site_id).xmat.copy()
                arm_name = "Left arm" if i == 0 else "Right arm"
                self.get_logger().info(f"🎈 {arm_name} ({hand}) site initial position: {site_xpos}, rotation matrix: {site_xmat}")

                target = self.targets[i]
                target_mocap_id = self.model.body(target).mocapid[0]
                xml_pos = self.configuration.data.mocap_pos[target_mocap_id].copy()
                xml_quat = self.configuration.data.mocap_quat[target_mocap_id].copy()
                self.initial_mocap_pos[target] = xml_pos
                self.initial_mocap_quat[target] = xml_quat
                T_target2world = np.eye(4)
                T_target2world[:3, :3] = R.from_quat(xml_quat, scalar_first=True).as_matrix()
                T_target2world[:3, 3] = xml_pos
                self.get_logger().info(f"🟢 {arm_name} ({target}) keeping XML-defined mocap position: {xml_pos}")
                self.get_logger().info(f"🟢 {arm_name} ({target}) keeping XML-defined mocap orientation: {xml_quat}")

                base = self.bases[i]
                base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, base)
                base_pos = self.configuration.data.xpos[base_id].copy()
                base_quat_mujoco = self.configuration.data.xquat[base_id].copy()
                T_base2world = np.eye(4)
                T_base2world[:3, :3] = R.from_quat(base_quat_mujoco, scalar_first=True).as_matrix()
                T_base2world[:3, 3] = base_pos
                self.T_base2world_list.append(T_base2world)
                self.initial_mocap_poses_base.append(np.linalg.inv(T_base2world) @ T_target2world)
                self.get_logger().info(f"💾 Cached {arm_name} base ({base}) global 4x4 transform")

        _mink_joint_limit = mink.ConfigurationLimit(self.model)
        self.limits = [_mink_joint_limit]

    def _start_viewer(self):
        """Launch the MuJoCo viewer in a separate thread."""
        try:
            self.viewer = mujoco.viewer.launch_passive(
                    model=self.model, 
                    data=self.configuration.data, 
                    show_left_ui=True,
                    show_right_ui=True
                )
            mujoco.mjv_defaultFreeCamera(self.model, self.viewer.cam)
        except Exception as e:
            self.get_logger().warning(f"Failed to start viewer (headless environment?): {e}")
            self.viewer = None
            self.enable_viewer = False

    def _warm_up_sim(self):
        """Warm up the simulation."""
        for _ in range(10):
            with self.data_lock:
                mujoco.mj_forward(self.model, self.configuration.data)
                if self.enable_viewer and self.viewer:
                    self.viewer.sync()
                time.sleep(0.01)

    def _pose_to_matrix(self, pose):
        T = np.eye(4)
        T[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
        T[:3, :3] = R.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]).as_matrix()
        return T

    def _get_site_pose_base(self, index):
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.hands[index])
        with self.data_lock:
            mujoco.mj_forward(self.model, self.configuration.data)
            T_site2world = np.eye(4)
            T_site2world[:3, 3] = self.configuration.data.site(site_id).xpos.copy()
            T_site2world[:3, :3] = self.configuration.data.site(site_id).xmat.copy().reshape(3, 3)
        return np.linalg.inv(self.T_base2world_list[index]) @ T_site2world

    def _clear_human_references(self):
        self.reference_tracker_poses = [None, None]
        self.reference_arm_poses = [None, None]

    def _reset_to_home(self):
        with self.commander_lock:
            self.commander = 'model'
            self._clear_human_references()
            self.current_arm_poses = [pose.copy() for pose in self.initial_mocap_poses_base]

        with self.data_lock:
            self.configuration.update_from_keyframe("home")
            for target in self.targets:
                target_mocap_id = self.model.body(target).mocapid[0]
                self.configuration.data.mocap_pos[target_mocap_id] = self.initial_mocap_pos[target]
                self.configuration.data.mocap_quat[target_mocap_id] = self.initial_mocap_quat[target]
            mujoco.mj_forward(self.model, self.configuration.data)

    def wrist_poses_callback(self, msg: PoseArray):
        """Callback for incoming wrist poses."""
        for i in range(len(msg.poses)):
            if i >= len(self.current_arm_poses):
                break
            pose = msg.poses[i]
            target_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            target_quat_scipy =[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            T_target2base = np.eye(4)
            T_target2base[:3, :3] = R.from_quat(target_quat_scipy).as_matrix()
            T_target2base[:3, 3] = target_pos
            with self.commander_lock:
                self.current_arm_poses[i] = T_target2base

    def left_tracker_pose_callback(self, msg: PoseStamped):
        """Callback for the left tracker pose."""
        with self.commander_lock:
            self.current_tracker_poses[0] = self._pose_to_matrix(msg.pose)

    def right_tracker_pose_callback(self, msg: PoseStamped):
        """Callback for the right tracker pose."""
        with self.commander_lock:
            self.current_tracker_poses[1] = self._pose_to_matrix(msg.pose)

    def system_mode_callback(self, msg: String):
        """Callback for system mode changes."""
        self.mode = msg.data

    def commander_callback(self, msg: String):
        """Callback for commander changes."""
        commander = msg.data.strip().lower()
        if commander == 'model':
            with self.commander_lock:
                self.commander = 'model'
                self._clear_human_references()
            self.get_logger().info("Switched commander to MODEL - IK will follow model targets")
        elif commander == 'human':
            with self.commander_lock:
                current_tracker_poses = [
                    pose.copy() if pose is not None else None
                    for pose in self.current_tracker_poses
                ]
            if current_tracker_poses[0] is None or current_tracker_poses[1] is None:
                self.get_logger().warning("Tracker poses must be available for HUMAN commander")
                return

            reference_arm_poses = [self._get_site_pose_base(0), self._get_site_pose_base(1)]
            with self.commander_lock:
                self.reference_tracker_poses = current_tracker_poses
                self.reference_arm_poses = reference_arm_poses
                self.commander = 'human'
            self.get_logger().info("Switched commander to HUMAN - IK will follow human targets")
        else:
            self.get_logger().warning(f"Unknown commander: {msg.data}")

    def timer_callback(self):
        """Timer callback: solve IK and publish joint states."""
        t0 = time.time()

        if self.mode == 'reset':
            self.mode = 'execution'
            self._reset_to_home()
        else:
            with self.commander_lock:
                commander = self.commander
                current_arm_poses = [
                    pose.copy() if pose is not None else None
                    for pose in self.current_arm_poses
                ]
                current_tracker_poses = [
                    pose.copy() if pose is not None else None
                    for pose in self.current_tracker_poses
                ]
                reference_tracker_poses = [
                    pose.copy() if pose is not None else None
                    for pose in self.reference_tracker_poses
                ]
                reference_arm_poses = [
                    pose.copy() if pose is not None else None
                    for pose in self.reference_arm_poses
                ]

            for i in range(len(self.targets)):
                if commander == 'model':
                    T_target2base = current_arm_poses[i]
                    T_target2world = self.T_base2world_list[i] @ T_target2base
                elif commander == 'human':
                    assert (
                        reference_tracker_poses[i] is not None
                        and current_tracker_poses[i] is not None
                        and reference_arm_poses[i] is not None
                    ), f"Required poses for human commander are not available for arm {i}"
                    delta = current_tracker_poses[i] @ np.linalg.inv(reference_tracker_poses[i])
                    T_target2world = delta @ self.T_base2world_list[i] @ reference_arm_poses[i]

                target_pos_world = T_target2world[:3, 3]
                target_rot_world = T_target2world[:3, :3]
                target_quat_world_mujoco = R.from_matrix(target_rot_world).as_quat(scalar_first=True)

                target_mocap_id = self.model.body(self.targets[i]).mocapid[0]
                with self.data_lock:
                    self.configuration.data.mocap_pos[target_mocap_id] = target_pos_world
                    self.configuration.data.mocap_quat[target_mocap_id] = target_quat_world_mujoco

        with self.data_lock:
            mujoco.mj_forward(self.model, self.configuration.data)
            if self.enable_viewer and self.viewer:
                self.viewer.sync()

        # Update task targets
        for i in range(len(self.hand_tasks)):
            hand_task = self.hand_tasks[i]
            target = self.targets[i]
            with self.data_lock:
                _hand_target = mink.SE3.from_mocap_name(self.model, self.configuration.data, target)
                hand_task.set_target(_hand_target)

        vel = mink.solve_ik(
            configuration=self.configuration, 
            tasks=self.tasks, 
            dt=self.dt, 
            solver=self.solver, 
            damping=1e-1,
            safety_break=False,
            limits=self.limits,
        )
        self.configuration.integrate_inplace(vel, self.dt)
        
        with self.data_lock:
            _solved_joints = self.configuration.data.qpos.copy()  # robot: [arm1_joints, arm2_joints]
        
        self.get_logger().debug(f'Solved joints: {_solved_joints}')

        self._publish_arms_joints_cmd(target_arm_joints=_solved_joints)

        t1 = time.time()
        self.get_logger().debug(f'IK solve loop time: {(t1-t0)*1000:.2f}ms')
    
    def _publish_arms_joints_cmd(self, target_arm_joints):
        """Publish robot dual-arm joint angles (matching the control node format)."""
        if isinstance(target_arm_joints, np.ndarray):
            joint_positions = target_arm_joints.tolist()
        else:
            joint_positions = target_arm_joints

        # Verify joint count (14 DOF)
        expected_joints = self.left_arm_joint_names + self.right_arm_joint_names
        assert len(joint_positions) == len(expected_joints), f"joint_positions length mismatch: {len(joint_positions)} != {len(expected_joints)}"

        timestamp = self.get_clock().now().to_msg()
        left_arm_joint_state_msg = JointState()
        left_arm_joint_state_msg.header.stamp = timestamp
        left_arm_joint_state_msg.header.frame_id = "base_link"
        left_arm_joint_state_msg.name = self.left_arm_joint_names
        left_arm_joint_state_msg.position = [float(pos) for pos in joint_positions[:7]]
        
        right_arm_joint_state_msg = JointState()
        right_arm_joint_state_msg.header.stamp = timestamp
        right_arm_joint_state_msg.header.frame_id = "base_link"
        right_arm_joint_state_msg.name = self.right_arm_joint_names
        right_arm_joint_state_msg.position = [float(pos) for pos in joint_positions[7:]]

        self.left_arm_command_pub.publish(left_arm_joint_state_msg)
        self.right_arm_command_pub.publish(right_arm_joint_state_msg)

        self._publish_count += 1

    def destroy_node(self):
        """Cleanup on node destruction."""
        if self.viewer:
            self.viewer.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ArmIKNode()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        

if __name__ == '__main__':
    main()
