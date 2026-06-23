#!/usr/bin/env python3
import os
import cv2
import time
import bisect
import shlex
import signal
import subprocess
import datetime
from contextlib import ExitStack
from enum import Enum
from collections import deque
from pynput import keyboard
import requests
import threading
import numpy as np
from itertools import islice
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from cv_bridge import CvBridge

from .geometry import (
    homo_matrix_from_trans_6drot,
    homo_matrix_to_trans_6drot
)
try:
    from .utils.websocket_client import WebsocketClientPolicy
except ImportError:
    from utils.websocket_client import WebsocketClientPolicy

class SystemState(Enum):
    IDLE = 0
    READY = 1
    FIRST_OBS = 2
    RUNNING = 3
    RESETTING = 4

# --- Math utilities ---
def matrix_from_pose_msg(pose):
    t = [pose.position.x, pose.position.y, pose.position.z]
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    T = np.eye(4)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T

def pose_from_matrix(T):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = map(float, T[:3, 3])
    quat = R.from_matrix(T[:3, :3]).as_quat()
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, quat)
    return msg

def get_6d_rot(T):
    return np.concatenate([T[:3, 0], T[:3, 1]])

def matrix_from_6d_rot(trans, rot6d):
    v1, v2 = rot6d[:3], rot6d[3:]
    e1 = v1 / (np.linalg.norm(v1) + 1e-6)
    e2 = v2 - np.dot(e1, v2) * e1
    e2 = e2 / (np.linalg.norm(e2) + 1e-6)
    e3 = np.cross(e1, e2)
    T = np.eye(4)
    T[:3, :3] = np.stack([e1, e2, e3], axis=1)
    T[:3, 3] = trans
    return T

class ModelInterfaceNode(Node):
    def __init__(self):
        super().__init__('model_interface_node')

        # --- 1. Locks and signals ---
        self.l_pose_cb_lock = threading.Lock()
        self.r_pose_cb_lock = threading.Lock()
        self.l_kp_cb_lock = threading.Lock()
        self.r_kp_cb_lock = threading.Lock()
        self.action_timer_lock = threading.Lock()
        self.inference_lock = threading.Lock()
        self.commander_lock = threading.Lock()
        self.infer_event = threading.Event()

        # --- 2. Parameter loading ---
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('model_server_host', '0.0.0.0')
        self.declare_parameter('model_server_port', 8000)
        self.declare_parameter('calibration_path', '')
        self.declare_parameter('camera_setup', 'both')   # head, chest, both
        self.declare_parameter('image_mode', 'rgbd')     # rgb, rgbd
        self.declare_parameter('ui_service_host', 'localhost')
        self.declare_parameter('ui_service_port', 8081)
        self.declare_parameter('audio_language', 'Chinese')  # voice prompt language: Chinese / English
        self.declare_parameter('state_frequency', 3.0)
        self.declare_parameter('state_horizon', 18)
        self.declare_parameter('image_frequency', 1.0)
        self.declare_parameter('image_horizon', 6)
        self.declare_parameter('action_execution_len', 6)
        self.declare_parameter('action_rtc_len', 4)
        self.declare_parameter('action_mode', 'relative')
        self.declare_parameter('buffer_size', 1000)
        self.declare_parameter('debug_code', False)
        self.declare_parameter('do_resize', False)
        self.declare_parameter('original_wh', [640, 480])
        self.declare_parameter('target_wh', [384, 384])
        self.declare_parameter('image_compression', 'jpeg')  # none, jpeg
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('human_in_the_loop', False)
        self.declare_parameter('tracker_launch_command', ['ros2', 'launch', 'tracker', 'dual_tracker.launch.py'])
        self.declare_parameter('glove_launch_command', ['ros2', 'launch', 'glove', 'dual_glove.launch.py'])
        self.declare_parameter('human_input_shutdown_timeout', 5.0)
        self.declare_parameter('recording_service_timeout', 3.0)

        self.ctrl_freq = self.get_parameter('control_frequency').value
        self.calib_root = self.get_parameter('calibration_path').value
        self.camera_setup = self.get_parameter('camera_setup').value
        self.image_mode = self.get_parameter('image_mode').value
        self.camera_names = self._camera_names_from_setup(self.camera_setup)
        self.cam_name = self.camera_names[0]
        self.ui_host = self.get_parameter('ui_service_host').value
        self.ui_port = self.get_parameter('ui_service_port').value
        self.audio_language = str(self.get_parameter('audio_language').value).strip().capitalize()  # -> Chinese / English
        self.s_freq = self.get_parameter('state_frequency').value
        self.s_hor = self.get_parameter('state_horizon').value
        self.i_freq = self.get_parameter('image_frequency').value
        self.i_hor = self.get_parameter('image_horizon').value
        self.use_depth = self._use_depth_from_image_mode(self.image_mode)
        self.depth_camera_names = list(self.camera_names) if self.use_depth else []
        self.act_len = self.get_parameter('action_execution_len').value
        self.act_rtc_len = self.get_parameter('action_rtc_len').value
        self.act_mode = self.get_parameter('action_mode').value
        self.max_buf = self.get_parameter('buffer_size').value
        self.debug_code = self.get_parameter('debug_code').value
        self.do_resize = self.get_parameter('do_resize').value
        self.original_wh = tuple(self.get_parameter('original_wh').value)
        self.target_wh = tuple(self.get_parameter('target_wh').value)
        self.image_compression = self.get_parameter('image_compression').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.human_in_the_loop = self._bool_from_parameter('human_in_the_loop')
        self.tracker_launch_command = self._launch_command_from_parameter('tracker_launch_command')
        self.glove_launch_command = self._launch_command_from_parameter('glove_launch_command')
        self.human_input_shutdown_timeout = float(self.get_parameter('human_input_shutdown_timeout').value)
        self.recording_service_timeout = float(self.get_parameter('recording_service_timeout').value)
        self.human_input_processes = {}
        self.human_input_process_lock = threading.Lock()
        self.get_logger().info(f"📋 Config: control_freq={self.ctrl_freq}Hz, state_freq={self.s_freq}Hz, state_horizon={self.s_hor}, image_freq={self.i_freq}, image_horizon={self.i_hor}, camera_setup={self.camera_setup}, cameras={self.camera_names}, image_mode={self.image_mode}, action_len={self.act_len}, rtc_action_len={self.act_rtc_len}, action_mode={self.act_mode}, buffer_size={self.max_buf}, do_resize={self.do_resize}, original_wh={self.original_wh}, target_wh={self.target_wh}, image_compression={self.image_compression}, jpeg_quality={self.jpeg_quality}, human_in_the_loop={self.human_in_the_loop}")

        # --- 3. Timing parameters ---
        self.state_dt_ns = int(1e9 / self.s_freq)
        self.image_dt_ns = int(1e9 / self.i_freq)

        # --- 4. Calibration ---
        self.get_logger().info(f"📐 Loading calibration: cameras={self.camera_names}, path={self.calib_root}")
        self.camera_calibrations = self._load_camera_calibrations()
        primary_calibration = self.camera_calibrations[self.cam_name]
        self.K_mat = primary_calibration['camera_matrix']
        self.T_cam2base_l = primary_calibration['left']['T_cam2base']
        self.T_cam2base_r = primary_calibration['right']['T_cam2base']
        self.T_base2cam_l = primary_calibration['left']['T_base2cam']
        self.T_base2cam_r = primary_calibration['right']['T_base2cam']
        self.camera_intrinsics_by_name = {
            name: calibration['camera_matrix']
            for name, calibration in self.camera_calibrations.items()
        }
        self.camera_world2cam_by_name = self._build_camera_world2cam_by_name()
        self.get_logger().info(
            f"✅ Calibration loaded: cameras={list(self.camera_calibrations.keys())}, "
            f"primary_camera={self.cam_name}, primary_intrinsics_shape={self.K_mat.shape}, primary_intrinsics=\n{self.K_mat}"
        )

        self.T_wrist2tcp_l = np.array([
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [1, 0, 0, 0.0345],
            [0, 0, 0, 1]
        ])
        self.T_wrist2tcp_r = np.array([
            [0, 1, 0, 0],
            [0, 0, -1, 0],
            [-1, 0, 0, 0.0345],
            [0, 0, 0, 1]
        ])
        self.T_tcp2wrist_l = np.linalg.inv(self.T_wrist2tcp_l)
        self.T_tcp2wrist_r = np.linalg.inv(self.T_wrist2tcp_r)
        self.T_wrist2handbase_l = np.array([
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [1, 0, 0, 0.01665],
            [0, 0, 0, 1]
        ])
        self.T_wrist2handbase_r = np.array([
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [-1, 0, 0, 0.01665],
            [0, 0, 0, 1]
        ])
        self.T_handbase2wrist_l = np.linalg.inv(self.T_wrist2handbase_l)
        self.T_handbase2wrist_r = np.linalg.inv(self.T_wrist2handbase_r)

        # --- 5. Data structures ---
        self.state = SystemState.IDLE
        self.current_instr = ""

        self.action_queue = deque(maxlen=200)
        self.action_queue_copy = list(self.action_queue)
        
        # Sensor buffers: deque.append is atomic in CPython, so no lock is needed
        self.buf_rgb = {name: deque(maxlen=self.max_buf) for name in self.camera_names}
        self.buf_depth = {name: deque(maxlen=self.max_buf) for name in self.depth_camera_names}
        self.rgb_cb_locks = {name: threading.Lock() for name in self.camera_names}
        self.depth_cb_locks = {name: threading.Lock() for name in self.depth_camera_names}
        self.buf_l_wrist = deque(maxlen=self.max_buf)
        self.buf_r_wrist = deque(maxlen=self.max_buf)
        self.buf_l_kps = deque(maxlen=self.max_buf)
        self.buf_r_kps = deque(maxlen=self.max_buf)
        
        self.cv_bridge = CvBridge()
        
        self.ui_session = requests.Session()
        adapter = HTTPAdapter(
            max_retries=Retry(total=3, backoff_factor=1, status_forcelist=[500, 502, 503, 504])
        )
        self.ui_session.mount('http://', adapter)
        self.ui_session.mount('https://', adapter)

        # --- 6. Communication interfaces (each with its own callback group) ---
        self.pub_system_mode = self.create_publisher(String, '/system/mode', 10)
        self.pub_commander = self.create_publisher(String, '/commander', 10)
        self.pub_instruction = self.create_publisher(String, '/interface/instruction', 10)
        self.pub_action_poses = self.create_publisher(PoseArray, '/action/both_arms/wrist_poses', 1)
        self.pub_action_hand_l = self.create_publisher(PoseArray, '/action/left_hand/keypoints', 1)
        self.pub_action_hand_r = self.create_publisher(PoseArray, '/action/right_hand/keypoints', 1)
        self.recording_client = self.create_client(SetBool, '/toggle_recording')
        self.discard_recording_client = self.create_client(Trigger, '/discard_recording')
        self.recording_state = 'idle'
        self.recording_state_lock = threading.Lock()

        self.commander_mode = 'model'
        self.commander_generation = 0
        self.bootstrap_inference = False

        self.camera_subscriptions = []
        rgb_callbacks = {
            'head': self.head_rgb_cb,
            'chest': self.chest_rgb_cb,
        }
        depth_callbacks = {
            'head': self.head_depth_cb,
            'chest': self.chest_depth_cb,
        }
        for camera_name in self.camera_names:
            self.camera_subscriptions.append(
                self.create_subscription(
                    Image,
                    f'/camera/{camera_name}/rgb',
                    rgb_callbacks[camera_name],
                    qos_profile_sensor_data,
                    callback_group=MutuallyExclusiveCallbackGroup()
                )
            )
        for camera_name in self.depth_camera_names:
            self.camera_subscriptions.append(
                self.create_subscription(
                    Image,
                    f'/camera/{camera_name}/depth',
                    depth_callbacks[camera_name],
                    qos_profile_sensor_data,
                    callback_group=MutuallyExclusiveCallbackGroup()
                )
            )
        self.create_subscription(PoseStamped, '/state/left_arm/wrist_pose', self.l_pose_cb, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(PoseStamped, '/state/right_arm/wrist_pose', self.r_pose_cb, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(PoseArray, '/state/left_hand/keypoints', self.l_kp_cb, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(PoseArray, '/state/right_hand/keypoints', self.r_kp_cb, 1, callback_group=MutuallyExclusiveCallbackGroup())
        rgb_topics = [f'/camera/{name}/rgb' for name in self.camera_names]
        depth_topics = [f'/camera/{name}/depth' for name in self.depth_camera_names]
        self.get_logger().info(f"📡 Subscribed topics: RGB={rgb_topics}, Depth={depth_topics}, "
                              f"left_wrist={'/state/left_arm/wrist_pose'}, right_wrist={'/state/right_arm/wrist_pose'}")

        # --- 7. Startup ---
        try:
            self.policy_client = WebsocketClientPolicy(
                host=self.get_parameter('model_server_host').value,
                port=self.get_parameter('model_server_port').value,
                image_compression=self.image_compression,
                jpeg_quality=self.jpeg_quality,
            )
            self.get_logger().info("✅ WebSocket connected")
        except Exception as e:
            self.get_logger().error(f"❌ WebSocket connection failed: {e}")

        self.kbd_listener = keyboard.Listener(on_press=self.on_key_press)
        self.kbd_listener.start()
        
        threading.Thread(target=self.remote_input_loop, daemon=True).start()
        threading.Thread(target=self.inference_worker, daemon=True).start()
        self.get_logger().info("🔄 Background threads started: remote input loop, inference worker")

        self.control_timer = self.create_timer(1.0 / self.ctrl_freq, self.control_timer_cb, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(f"🚀 Model Interface started (Multi-threaded & Lock-Optimized)")

    def destroy_node(self):
        self._stop_human_input_nodes()
        if hasattr(self, 'kbd_listener'):
            self.kbd_listener.stop()
        return super().destroy_node()

    def _launch_command_from_parameter(self, parameter_name):
        value = self.get_parameter(parameter_name).value
        if isinstance(value, str):
            command = shlex.split(value)
        else:
            command = [str(part) for part in value]

        if not command:
            raise ValueError(f"{parameter_name} must not be empty")

        return command

    def _bool_from_parameter(self, parameter_name):
        value = self.get_parameter(parameter_name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _start_human_input_nodes(self):
        if not self.human_in_the_loop:
            return

        launch_commands = {
            'tracker': self.tracker_launch_command,
            'glove': self.glove_launch_command,
        }
        with self.human_input_process_lock:
            for name, command in launch_commands.items():
                process = self.human_input_processes.get(name)
                if process is not None and process.poll() is None:
                    self.get_logger().info(f"{name} launch already running: pid={process.pid}")
                    continue

                try:
                    process = subprocess.Popen(command, start_new_session=True)
                    self.human_input_processes[name] = process
                    self.get_logger().info(f"Started {name} launch: {' '.join(command)} (pid={process.pid})")
                except Exception as exc:
                    self.get_logger().error(f"Failed to start {name} launch: {exc}")

    def _stop_human_input_nodes(self):
        if not hasattr(self, 'human_input_process_lock'):
            return

        with self.human_input_process_lock:
            processes = list(self.human_input_processes.items())
            self.human_input_processes.clear()

        for name, process in processes:
            if process.poll() is not None:
                self.get_logger().info(f"{name} launch already exited: returncode={process.returncode}")
                continue

            self.get_logger().info(f"Shutting down {name} launch: pid={process.pid}")
            self._signal_launch_process(name, process, signal.SIGINT)

        deadline = time.monotonic() + self.human_input_shutdown_timeout
        for name, process in processes:
            if process.poll() is not None:
                continue

            remaining = max(0.0, deadline - time.monotonic())
            try:
                process.wait(timeout=remaining)
                self.get_logger().info(f"{name} launch closed")
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"{name} launch did not exit in time, sending SIGTERM")
                self._terminate_process_group(name, process, signal.SIGTERM)

        for name, process in processes:
            if process.poll() is not None:
                continue

            try:
                process.wait(timeout=1.0)
                self.get_logger().info(f"{name} launch closed")
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"{name} launch still not exited, sending SIGKILL")
                self._terminate_process_group(name, process, signal.SIGKILL)

    def _terminate_process_group(self, name, process, sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass
        except Exception as exc:
            self.get_logger().warn(f"Failed to terminate {name} launch: {exc}")

    def _signal_launch_process(self, name, process, sig):
        try:
            process.send_signal(sig)
        except ProcessLookupError:
            pass
        except Exception as exc:
            self.get_logger().warn(f"Failed to signal {name} launch to shut down: {exc}")

    def _camera_names_from_setup(self, camera_setup):
        valid_setups = {
            'head': ['head'],
            'chest': ['chest'],
            'both': ['head', 'chest'],
        }
        if camera_setup in valid_setups:
            return valid_setups[camera_setup]

        raise ValueError("camera_setup must be 'head', 'chest', or 'both'")

    def _use_depth_from_image_mode(self, image_mode):
        if image_mode == 'rgb':
            return False
        if image_mode == 'rgbd':
            return True
        raise ValueError("image_mode must be 'rgb' or 'rgbd'")

    def _load_camera_calibrations(self):
        calibrations = {}
        for camera_name in self.camera_names:
            T_cam2base_l, camera_matrix = self.find_and_load_calibration(camera_name, 'left')
            T_cam2base_r, _ = self.find_and_load_calibration(camera_name, 'right')
            if self.do_resize:
                camera_matrix = self.resize_intrinsics(camera_matrix, old_wh=self.original_wh, new_wh=self.target_wh)
            calibrations[camera_name] = {
                'camera_matrix': camera_matrix,
                'left': {
                    'T_cam2base': T_cam2base_l,
                    'T_base2cam': np.linalg.inv(T_cam2base_l),
                },
                'right': {
                    'T_cam2base': T_cam2base_r,
                    'T_base2cam': np.linalg.inv(T_cam2base_r),
                },
            }
        return calibrations

    def _build_camera_world2cam_by_name(self):
        if len(self.camera_names) == 1:
            return {}

        head_calibration = self.camera_calibrations['head']
        T_world2base = head_calibration['left']['T_cam2base']
        world2cam = {}
        for camera_name, calibration in self.camera_calibrations.items():
            world2cam[camera_name] = calibration['left']['T_base2cam'] @ T_world2base
        return world2cam

    def calc_wrist_to_cam(self, T_tcp2base, side):
        if side == 'left':
            T_wrist2tcp = self.T_wrist2tcp_l
            T_base2cam = self.T_base2cam_l
        elif side == 'right':
            T_wrist2tcp = self.T_wrist2tcp_r
            T_base2cam = self.T_base2cam_r
        T_wrist2cam = T_base2cam @ T_tcp2base @ T_wrist2tcp
        return T_wrist2cam

    def calc_keypoints_to_wrist(self, keypoints, side):
        assert len(keypoints) == 5, "incorrect number of keypoints"
        keypoints_transformed = []
        for keypoint in keypoints:
            keypoint_homo = np.ones((4, 1))
            keypoint_homo[:3, 0] = keypoint
            if side == 'left':
                T_handbase2wrist = self.T_handbase2wrist_l
            elif side == 'right':
                T_handbase2wrist = self.T_handbase2wrist_r
            keypoint_homo = T_handbase2wrist @ keypoint_homo
            keypoints_transformed.append(keypoint_homo[:3, 0])
        return np.array(keypoints_transformed)

    def calc_tcp_to_arm_base(self, T_wrist2cam, side):
        if side == 'left':
            T_tcp2wrist = self.T_tcp2wrist_l
            T_cam2base = self.T_cam2base_l
        elif side == 'right':
            T_tcp2wrist = self.T_tcp2wrist_r
            T_cam2base = self.T_cam2base_r
        T_tcp2base = T_cam2base @ T_wrist2cam @ T_tcp2wrist
        return T_tcp2base

    def calc_keypoints_to_hand_base(self, keypoints, side):
        assert len(keypoints) == 5, "incorrect number of keypoints"
        keypoints_transformed = []
        for keypoint in keypoints:
            keypoint_homo = np.ones((4, 1))
            keypoint_homo[:3, 0] = keypoint
            if side == 'left':
                T_wrist2handbase = self.T_wrist2handbase_l
            elif side == 'right':
                T_wrist2handbase = self.T_wrist2handbase_r
            keypoint_homo = T_wrist2handbase @ keypoint_homo
            keypoints_transformed.append(keypoint_homo[:3, 0])
        return np.array(keypoints_transformed)

    # --- State and timing logic ---
    def _is_active_recording(self, state):
        """Atomic read: only record data in these states."""
        return state in [SystemState.FIRST_OBS, SystemState.RUNNING]

    def _execute_switch_state(self, new_state):
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f"🔄 State transition: {old_state.name} -> {new_state.name} ")

    def _image_locks(self):
        return list(self.rgb_cb_locks.values()) + list(self.depth_cb_locks.values())

    def _switch_state(self, new_state):
        old_state = self.state
        if old_state == new_state: return
        if new_state == SystemState.RESETTING:
            with ExitStack() as stack:
                # inference_lock → action_timer_lock → buffer locks (must match inference_worker).
                stack.enter_context(self.inference_lock)
                stack.enter_context(self.action_timer_lock)
                for lock in self._image_locks():
                    stack.enter_context(lock)
                stack.enter_context(self.l_pose_cb_lock)
                stack.enter_context(self.r_pose_cb_lock)
                stack.enter_context(self.l_kp_cb_lock)
                stack.enter_context(self.r_kp_cb_lock)
                self._execute_switch_state(new_state)
                self.action_queue.clear()
                self._clear_image_buffers()
                self.buf_l_wrist.clear()
                self.buf_r_wrist.clear(); self.buf_l_kps.clear(); self.buf_r_kps.clear()
            self.get_logger().info(f"🧹 Reset complete: cleared action queue and buffers")
        elif new_state == SystemState.FIRST_OBS:
            assert not self.action_queue, "action queue should be empty in FIRST_OBS state"
            assert self._image_buffers_empty(), "image buffers should be empty in FIRST_OBS state"
            assert not self.buf_l_wrist, "left wrist buffer should be empty in FIRST_OBS state"
            assert not self.buf_r_wrist, "right wrist buffer should be empty in FIRST_OBS state"
            assert not self.buf_l_kps, "left hand keypoint buffer should be empty in FIRST_OBS state"
            assert not self.buf_r_kps, "right hand keypoint buffer should be empty in FIRST_OBS state"
            self._execute_switch_state(new_state)
        else:
            self._execute_switch_state(new_state)

    def _transition_commander(self, mode, bootstrap=False):
        if mode not in ('human', 'model'):
            raise ValueError("commander must be 'human' or 'model'")

        with self.action_timer_lock:
            with self.commander_lock:
                self.commander_mode = mode
                self.commander_generation += 1
                self.bootstrap_inference = bool(mode == 'model' and bootstrap)
                generation = self.commander_generation
            self.action_queue.clear()
            self.action_queue_copy = []
            self.infer_event.clear()

        if mode == 'model' and bootstrap:
            self._publish_hold_action_from_latest_state()

        self.pub_commander.publish(String(data=mode))
        self.get_logger().info(
            f"🧭 Control handover: commander={mode}, generation={generation}, bootstrap={self.bootstrap_inference}"
        )

        if mode == 'model' and bootstrap and self.state in (SystemState.FIRST_OBS, SystemState.RUNNING):
            self.infer_event.set()

    def _call_recording_service(self, client, request, service_name, action_name):
        if not client.wait_for_service(timeout_sec=self.recording_service_timeout):
            self.get_logger().warn(f"{service_name} unavailable, skipping {action_name}")
            return False

        future = client.call_async(request)
        deadline = time.monotonic() + self.recording_service_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            self.get_logger().warn(f"{service_name} response timed out; {action_name} may still be running in background")
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"{service_name} call failed: {exc}")
            return False

        if response.success:
            self.get_logger().info(f"{action_name} succeeded: {response.message}")
            return True

        self.get_logger().warn(f"{action_name} failed: {response.message}")
        return False

    def _start_recording_episode(self):
        request = SetBool.Request()
        request.data = True
        success = self._call_recording_service(
            self.recording_client,
            request,
            '/toggle_recording',
            'start recording'
        )
        if success:
            with self.recording_state_lock:
                self.recording_state = 'recording'
            self.pub_instruction.publish(String(data=self.current_instr))
        return success

    def _stop_recording_episode(self):
        request = SetBool.Request()
        request.data = False
        success = self._call_recording_service(
            self.recording_client,
            request,
            '/toggle_recording',
            'stop recording'
        )
        if success:
            with self.recording_state_lock:
                self.recording_state = 'stopped'
        return success

    def _discard_recording_episode(self):
        success = self._call_recording_service(
            self.discard_recording_client,
            Trigger.Request(),
            '/discard_recording',
            'discard recording'
        )
        if success:
            with self.recording_state_lock:
                self.recording_state = 'discarded'
        return success

    def _recording_state(self):
        with self.recording_state_lock:
            return self.recording_state

    def _stop_recording_if_running(self):
        if self._recording_state() == 'recording':
            self._stop_recording_episode()

    def _handle_recording_pedal(self):
        recording_state = self._recording_state()

        if recording_state == 'recording':
            self.get_logger().info("User pressed '3': stop current recording")
            if self._stop_recording_episode():
                self.play_sound("stop_recording")
        elif recording_state == 'stopped':
            self.get_logger().info("User pressed '3': discard the just-finished recording")
            if self._discard_recording_episode():
                self.play_sound("delete")
        else:
            self.get_logger().info("User pressed '3': no recording to stop or discard, ignoring")

    def _reset_after_episode_end(self):
        self._switch_state(SystemState.RESETTING)
        self.play_sound("stop_and_reset")
        self.pub_system_mode.publish(String(data="reset"))
        self._stop_human_input_nodes()
        time.sleep(3.0)
        self._switch_state(SystemState.IDLE)

    def _publish_keypoints_action(self, publisher, keypoints, frame_id, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        for pt in keypoints:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, pt)
            msg.poses.append(pose)
        publisher.publish(msg)

    def _publish_hold_action_from_latest_state(self):
        with self.l_pose_cb_lock:
            left_wrist = self.buf_l_wrist[-1][1] if self.buf_l_wrist else None
        with self.r_pose_cb_lock:
            right_wrist = self.buf_r_wrist[-1][1] if self.buf_r_wrist else None
        with self.l_kp_cb_lock:
            left_keypoints = self.buf_l_kps[-1][1].copy() if self.buf_l_kps else None
        with self.r_kp_cb_lock:
            right_keypoints = self.buf_r_kps[-1][1].copy() if self.buf_r_kps else None

        assert (
            left_wrist is not None
            and right_wrist is not None
            and left_keypoints is not None
            and right_keypoints is not None
        ), "missing latest state when switching back to model"

        now = self.get_clock().now().to_msg()

        wrist_msg = PoseArray()
        wrist_msg.header.stamp = now
        wrist_msg.header.frame_id = "base_link"
        wrist_msg.poses.append(left_wrist)
        wrist_msg.poses.append(right_wrist)
        self.pub_action_poses.publish(wrist_msg)

        self._publish_keypoints_action(self.pub_action_hand_l, left_keypoints, "left_wrist_link", now)
        self._publish_keypoints_action(self.pub_action_hand_r, right_keypoints, "right_wrist_link", now)

    def _clear_image_buffers(self):
        for buf in self.buf_rgb.values():
            buf.clear()
        for buf in self.buf_depth.values():
            buf.clear()

    def _image_buffers_empty(self):
        return (
            all(len(buf) == 0 for buf in self.buf_rgb.values())
            and all(len(buf) == 0 for buf in self.buf_depth.values())
        )

    def _get_msg_ns(self, header):
        return header.stamp.sec * 10**9 + header.stamp.nanosec

    def _update_buffer(self, buf, header, data, buf_name=""):
        """Lock-free write, relying on deque thread safety and state isolation."""
        if not self._is_active_recording(self.state):
            return

        ts = self._get_msg_ns(header)

        if len(buf) > 0 and buf[-1][0] >= ts:
            self.get_logger().warn(f"⚠️  Timestamp error: current={self.get_formatted_time(buf[-1][0])}, new={self.get_formatted_time(ts)}")
            return

        buf.append((ts, data))

        if len(buf) == 1 and buf_name:
            self.get_logger().info(f"📥 First {buf_name} data received: time={self.get_formatted_time(ts)}")

        # Cold-start check only matters during state transitions
        if self.state == SystemState.FIRST_OBS:
            self._check_first_obs_complete()

    def _check_first_obs_complete(self):
        buf_sizes = {}
        for camera_name, buf in self.buf_rgb.items():
            buf_sizes[f'{camera_name}/RGB'] = len(buf)
        for camera_name, buf in self.buf_depth.items():
            buf_sizes[f'{camera_name}/Depth'] = len(buf)
        buf_sizes.update({
            'left_wrist': len(self.buf_l_wrist),
            'right_wrist': len(self.buf_r_wrist),
            'left_hand_kps': len(self.buf_l_kps),
            'right_hand_kps': len(self.buf_r_kps)
        })
        if all(l > 5 for l in buf_sizes.values()):
            self.infer_event.set()
        else:
            missing = [k for k, v in buf_sizes.items() if v <= 5]
            if len(missing) <= 2 and self.debug_code:  # only log near-complete cases to limit log volume
                self.get_logger().info(f"⏳ Waiting for data: missing={missing}, current={buf_sizes}")

    # --- Sensor callbacks ---
    def head_rgb_cb(self, m):
        self._rgb_cb(m, 'head')

    def chest_rgb_cb(self, m):
        self._rgb_cb(m, 'chest')

    def head_depth_cb(self, m):
        self._depth_cb(m, 'head')

    def chest_depth_cb(self, m):
        self._depth_cb(m, 'chest')

    def _rgb_cb(self, m, camera_name): 
        data = self.cv_bridge.imgmsg_to_cv2(m, 'rgb8')
        if self.do_resize:
            data = self.resize_image(data, self.target_wh, is_depth=False)
        with self.rgb_cb_locks[camera_name]:
            self._update_buffer(self.buf_rgb[camera_name], m.header, data, f"{camera_name}/RGB")

    def _depth_cb(self, m, camera_name): 
        data = self.cv_bridge.imgmsg_to_cv2(m, 'passthrough')
        if self.do_resize:
            data = self.resize_image(data, self.target_wh, is_depth=True)
        with self.depth_cb_locks[camera_name]:
            self._update_buffer(self.buf_depth[camera_name], m.header, data, f"{camera_name}/Depth")

    def l_pose_cb(self, m): 
        with self.l_pose_cb_lock:
            self._update_buffer(self.buf_l_wrist, m.header, m.pose, "left wrist pose")

    def r_pose_cb(self, m): 
        with self.r_pose_cb_lock:
            self._update_buffer(self.buf_r_wrist, m.header, m.pose, "right wrist pose")

    def l_kp_cb(self, m):
        pts = np.array([[p.position.x, p.position.y, p.position.z] for p in m.poses])
        with self.l_kp_cb_lock:
            self._update_buffer(self.buf_l_kps, m.header, pts, "left hand keypoints")

    def r_kp_cb(self, m):
        pts = np.array([[p.position.x, p.position.y, p.position.z] for p in m.poses])
        with self.r_kp_cb_lock:
            self._update_buffer(self.buf_r_kps, m.header, pts, "right hand keypoints")

    def resize_image(self, img, target_wh, is_depth=False):
        width, height = target_wh
        if is_depth:
            resized_img = cv2.resize(img, (width, height), interpolation=cv2.INTER_NEAREST)
        else:
            resized_img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
        return resized_img

    def resize_intrinsics(self, K, old_wh, new_wh):
        """
        Adjust camera intrinsics for a new image size.

        Args:
            K (np.array): original 3x3 intrinsics matrix
            old_wh (tuple): (Width, Height) original size
            new_wh (tuple): (Width, Height) new size

        Returns:
            np.array: new intrinsics matrix
        """
        old_w, old_h = old_wh
        new_w, new_h = new_wh

        scale_x = new_w / old_w
        scale_y = new_h / old_h

        K_new = K.copy()

        K_new[0, 0] *= scale_x  # fx
        K_new[1, 1] *= scale_y  # fy

        K_new[0, 2] *= scale_x  # cx
        K_new[1, 2] *= scale_y  # cy

        return K_new
    
    # --- Sampling and inference ---
    def _find_nearest(self, sorted_buf, target_ns):
        idx = bisect.bisect_left(sorted_buf, target_ns, key=lambda x: x[0])
        
        if idx == 0: return sorted_buf[0]
        if idx == len(sorted_buf): return sorted_buf[-1]
        
        if (target_ns - sorted_buf[idx-1][0]) < (sorted_buf[idx][0] - target_ns):
            return sorted_buf[idx-1]
        else:
            return sorted_buf[idx]

    def _is_in_range(self, sorted_buf, target_ts):
        return sorted_buf[0][0] <= target_ts <= sorted_buf[-1][0]

    def _sample_buffer_sequence(self, snap, t_ref, horizon, dt_ns, label):
        seq, ts_seq = [], []
        for h in range(horizon):
            t = t_ref - (h * dt_ns)
            if not self._is_in_range(snap, t):
                break
            nearest = self._find_nearest(snap, t)
            seq.append(nearest[1])
            ts_seq.append(nearest[0])
        assert seq, f"{label} buffer time range is insufficient"
        return np.stack(seq)[::-1], list(reversed(ts_seq))

    def get_relative_action(self, state, raw_action):
        rel_action = raw_action.copy()
        for idx in range(2):
            wrist_raw_action_homo_mat = homo_matrix_from_trans_6drot(raw_action[..., idx*3:idx*3+3], raw_action[..., 6+idx*6:6+idx*6+6])
            wrist_state_homo_mat = homo_matrix_from_trans_6drot(state[idx*3:idx*3+3], state[6+idx*6:6+idx*6+6])
            wrist_rel_action_homo_mat = np.linalg.pinv(wrist_state_homo_mat) @ wrist_raw_action_homo_mat
            rel_trans, rel_rot6d = homo_matrix_to_trans_6drot(wrist_rel_action_homo_mat)
            rel_action[..., idx*3:idx*3+3] = rel_trans
            rel_action[..., 6+idx*6:6+idx*6+6] = rel_rot6d
        rel_action[..., 18:] = raw_action[..., 18:] - state[18:]
        return rel_action

    def get_absolute_action(self, state, rel_action):
        raw_action = rel_action.copy()
        for idx in range(2):
            wrist_rel_action_homo_mat = homo_matrix_from_trans_6drot(rel_action[..., idx*3:idx*3+3], rel_action[..., 6+idx*6:6+idx*6+6])
            wrist_state_homo_mat = homo_matrix_from_trans_6drot(state[idx*3:idx*3+3], state[6+idx*6:6+idx*6+6])
            wrist_raw_action_homo_mat = wrist_state_homo_mat @ wrist_rel_action_homo_mat
            raw_trans, raw_rot6d = homo_matrix_to_trans_6drot(wrist_raw_action_homo_mat)
            raw_action[..., idx*3:idx*3+3] = raw_trans
            raw_action[..., 6+idx*6:6+idx*6+6] = raw_rot6d
        raw_action[..., 18:] = rel_action[..., 18:] + state[18:]
        return raw_action

    def prepare_inference_payload(self):
        with self.commander_lock:
            bootstrap_inference = self.bootstrap_inference

        # Snapshot all buffers under their locks
        snap_rgb = {}
        for name, buf in self.buf_rgb.items():
            with self.rgb_cb_locks[name]:
                snap_rgb[name] = list(buf)
        snap_depth = {}
        for name, buf in self.buf_depth.items():
            with self.depth_cb_locks[name]:
                snap_depth[name] = list(buf)
        with self.l_pose_cb_lock:
            snap_lw = list(self.buf_l_wrist)
        with self.r_pose_cb_lock:
            snap_rw = list(self.buf_r_wrist)
        with self.l_kp_cb_lock:
            snap_lk = list(self.buf_l_kps)
        with self.r_kp_cb_lock:
            snap_rk = list(self.buf_r_kps)
        image_snaps = list(snap_rgb.values()) + list(snap_depth.values())
        state_snaps = [snap_lw, snap_rw, snap_lk, snap_rk]
        all_snaps = image_snaps + state_snaps

        assert all(len(s) > 0 for s in all_snaps), "buffer data is incomplete"
        
        if self.debug_code:
            buf_sizes = {
                **{f'{name}/RGB': len(s) for name, s in snap_rgb.items()},
                **{f'{name}/Depth': len(s) for name, s in snap_depth.items()},
                'Left Wrist': len(snap_lw),
                'Right Wrist': len(snap_rw),
                'Left Kps': len(snap_lk),
                'Right Kps': len(snap_rk),
            }
            self.get_logger().info(f"📊 Preparing inference data: buffer_sizes={buf_sizes}")

        # check the timestamp of all_snaps
        if self.debug_code:
            for camera_name, snap in snap_rgb.items():
                self._print_buffer_info(f"{camera_name}/RGB", snap)
            for camera_name, snap in snap_depth.items():
                self._print_buffer_info(f"{camera_name}/Depth", snap)
            self._print_buffer_info("Left Wrist", snap_lw)
            self._print_buffer_info("Right Wrist", snap_rw)
            self._print_buffer_info("Left Kps", snap_lk)
            self._print_buffer_info("Right Kps", snap_rk)

        if self.state == SystemState.FIRST_OBS:
            rgb_inputs = {
                name: np.stack([snap[-1][1]])
                for name, snap in snap_rgb.items()
            }
            depth_inputs = {
                name: np.stack([snap[-1][1]])
                for name, snap in snap_depth.items()
            }
            for name, depth_input in depth_inputs.items():
                if depth_input.ndim == 3:
                    depth_inputs[name] = np.expand_dims(depth_input, axis=-1)
            lw = self.calc_wrist_to_cam(matrix_from_pose_msg(snap_lw[-1][1]), 'left')
            rw = self.calc_wrist_to_cam(matrix_from_pose_msg(snap_rw[-1][1]), 'right')
            lk = self.calc_keypoints_to_wrist(snap_lk[-1][1], 'left')
            rk = self.calc_keypoints_to_wrist(snap_rk[-1][1], 'right')
            vec = np.concatenate([
                lw[:3, 3], rw[:3, 3],
                get_6d_rot(lw), get_6d_rot(rw),
                lk.flatten(), rk.flatten()])
            states_in = vec[None, :].astype(np.float32)
            action_rtc = None
        else:
            # Aligned sampling
            t_ref = min(s[-1][0] for s in all_snaps)
            if self.debug_code:
                self.get_logger().info(f"⏱️  Time alignment: t_ref={self.get_formatted_time(t_ref)}, image_grid={self.image_dt_ns/1e6:.1f}ms, state_grid={self.state_dt_ns/1e6:.1f}ms")

            # Image
            rgb_inputs, rgb_ts_by_name = {}, {}
            for camera_name, snap in snap_rgb.items():
                rgb_inputs[camera_name], rgb_ts_by_name[camera_name] = self._sample_buffer_sequence(
                    snap, t_ref, self.i_hor, self.image_dt_ns, f"{camera_name}/RGB"
                )

            depth_inputs, depth_ts_by_name = {}, {}
            for camera_name, snap in snap_depth.items():
                depth_input, depth_ts_by_name[camera_name] = self._sample_buffer_sequence(
                    snap, t_ref, self.i_hor, self.image_dt_ns, f"{camera_name}/Depth"
                )
                if depth_input.ndim == 3:
                    depth_input = np.expand_dims(depth_input, axis=-1)
                depth_inputs[camera_name] = depth_input
            if self.debug_code:
                for camera_name, ts_seq in rgb_ts_by_name.items():
                    self.get_logger().info(f"Sampled {camera_name}/RGB timestamps: {', '.join(self.get_formatted_time(ts) for ts in ts_seq)}")
                for camera_name, ts_seq in depth_ts_by_name.items():
                    self.get_logger().info(f"Sampled {camera_name}/Depth timestamps: {', '.join(self.get_formatted_time(ts) for ts in ts_seq)}")

            # State
            states_list = []
            if self.debug_code:
                lw_ts_seq, rw_ts_seq, lk_ts_seq, rk_ts_seq = [], [], [], []
            for h in range(self.s_hor):
                t = t_ref - (h * self.state_dt_ns)
                state_snaps = [snap_lw, snap_rw, snap_lk, snap_rk]
                if not all(self._is_in_range(b, t) for b in state_snaps): break

                nearest_lw = self._find_nearest(snap_lw, t)
                nearest_rw = self._find_nearest(snap_rw, t)
                nearest_lk = self._find_nearest(snap_lk, t)
                nearest_rk = self._find_nearest(snap_rk, t)
                if self.debug_code:
                    lw_ts_seq.append(nearest_lw[0])
                    rw_ts_seq.append(nearest_rw[0])
                    lk_ts_seq.append(nearest_lk[0])
                    rk_ts_seq.append(nearest_rk[0])
                lw = self.calc_wrist_to_cam(matrix_from_pose_msg(nearest_lw[1]), 'left')
                rw = self.calc_wrist_to_cam(matrix_from_pose_msg(nearest_rw[1]), 'right')
                lk = self.calc_keypoints_to_wrist(nearest_lk[1], 'left')
                rk = self.calc_keypoints_to_wrist(nearest_rk[1], 'right')

                vec = np.concatenate([
                    lw[:3, 3], rw[:3, 3],
                    get_6d_rot(lw), get_6d_rot(rw),
                    lk.flatten(), rk.flatten()
                ])
                states_list.append(vec)
            states_in = np.array(states_list)[::-1].astype(np.float32)
            if self.debug_code:
                self.get_logger().info(f"Sampled left wrist timestamps: {', '.join(self.get_formatted_time(ts) for ts in reversed(lw_ts_seq))}")
                self.get_logger().info(f"Sampled right wrist timestamps: {', '.join(self.get_formatted_time(ts) for ts in reversed(rw_ts_seq))}")
                self.get_logger().info(f"Sampled left hand keypoint timestamps: {', '.join(self.get_formatted_time(ts) for ts in reversed(lk_ts_seq))}")
                self.get_logger().info(f"Sampled right hand keypoint timestamps: {', '.join(self.get_formatted_time(ts) for ts in reversed(rk_ts_seq))}")

            if self.act_rtc_len > 0 and not bootstrap_inference:
                if self.act_mode == 'absolute':
                    action_rtc = np.array([item['raw'] for item in islice(self.action_queue_copy, 0, self.act_rtc_len)])
                elif self.act_mode == 'relative':
                    action_rtc = np.array([self.get_relative_action(states_in[-1], item['raw']) for item in islice(self.action_queue_copy, 0, self.act_rtc_len)])
            else:
                action_rtc = None

        instr = self.current_instr
        rgb_in = rgb_inputs[self.cam_name]
        depth_in = depth_inputs.get(self.cam_name)
        
        if self.debug_code:
            depth_shape = depth_in.shape if depth_in is not None else 'N/A'
            rgb_shapes = {name: data.shape for name, data in rgb_inputs.items()}
            depth_shapes = {name: data.shape for name, data in depth_inputs.items()}
            self.get_logger().info(f"📦 Inference data ready: primary_rgb_shape={rgb_in.shape}, primary_depth_shape={depth_shape}, rgb_set={rgb_shapes}, depth_set={depth_shapes}, states_shape={states_in.shape}, action_shape={action_rtc.shape if action_rtc is not None else 'N/A'}")

        if len(self.camera_names) == 1:
            if self.use_depth:
                return {
                    "image": rgb_in, "depth_image": depth_in, "camera_intrinsics": self.K_mat,
                    "instruction": instr, "states": states_in, "action_rtc": action_rtc
                }
            else:
                return {
                    "image": rgb_in, "camera_intrinsics": self.K_mat,
                    "instruction": instr, "states": states_in, "action_rtc": action_rtc
                }

        payload = {
            "image": rgb_inputs,
            "camera_intrinsics": self.camera_intrinsics_by_name,
            "camera_extrinsics": self.camera_world2cam_by_name,
            "instruction": instr,
            "states": states_in,
            "action_rtc": action_rtc
        }
        if self.use_depth:
            payload["depth_image"] = depth_inputs
        return payload

    # --- Inference worker ---
    def inference_worker(self):
        while rclpy.ok():
            self.infer_event.wait()
            with self.commander_lock:
                infer_generation = self.commander_generation
                bootstrap_inference = self.bootstrap_inference
            try:
                with self.inference_lock:
                    if self.debug_code:
                        self.get_logger().info(f"🧠 Starting inference: instruction='{self.current_instr}'")
                    start_time = time.time()

                    try:
                        payload = self.prepare_inference_payload()
                    except AssertionError as e:
                        self.get_logger().warn(f"⚠️  Data preparation failed: {e}, waiting for more data...")
                        time.sleep(0.02)
                        continue
                    self.get_logger().info(f"🧠 Inference data ready: elapsed={time.time() - start_time:.3f}s")

                    try:
                        res = self.policy_client.infer(payload)
                        pred = res["pred_actions"]

                        inference_time = time.time() - start_time
                        self.get_logger().info(f"✅ Inference complete: elapsed={inference_time:.3f}s, num_predicted_actions={pred.shape[0]}")

                        with self.commander_lock:
                            current_generation = self.commander_generation
                            current_commander = self.commander_mode

                        if infer_generation != current_generation or current_commander != 'model':
                            self.get_logger().info("🧹 Inference result is stale, discarding this action queue update")
                            continue

                        if self.state == SystemState.FIRST_OBS or bootstrap_inference:
                            action_start = 0
                            action_end = self.act_len + self.act_rtc_len
                        else:
                            action_start = self.act_rtc_len
                            action_end = self.act_rtc_len + self.act_len + self.act_rtc_len
                        new_actions = []
                        last_state = payload["states"][-1]
                        for i in range(action_start, action_end):
                            if self.act_mode == 'absolute':
                                v = pred[i]
                            elif self.act_mode == 'relative':
                                v = self.get_absolute_action(last_state, pred[i])
                            new_actions.append({
                                'raw': v,
                                'l': self.calc_tcp_to_arm_base(matrix_from_6d_rot(v[0:3], v[6:12]), 'left'),
                                'r': self.calc_tcp_to_arm_base(matrix_from_6d_rot(v[3:6], v[12:18]), 'right'),
                                'lk': self.calc_keypoints_to_hand_base(v[18:33].reshape(-1, 3), 'left'),
                                'rk': self.calc_keypoints_to_hand_base(v[33:48].reshape(-1, 3), 'right')
                            })
                        with self.action_timer_lock:
                            with self.commander_lock:
                                current_generation = self.commander_generation
                                current_commander = self.commander_mode
                                if infer_generation != current_generation or current_commander != 'model':
                                    self.get_logger().info("🧹 Inference result is stale, discarding this action queue update")
                                    continue
                                self.bootstrap_inference = False
                            self.action_queue.extend(new_actions)
                            self.get_logger().info(f"📥 Action queue updated: current length={len(self.action_queue)}")

                        if self.state == SystemState.FIRST_OBS:
                            self._switch_state(SystemState.RUNNING)
                            with ExitStack() as stack:
                                for lock in self._image_locks():
                                    stack.enter_context(lock)
                                self._clear_image_buffers()
                            with self.l_pose_cb_lock:
                                self.buf_l_wrist.clear()
                            with self.r_pose_cb_lock:
                                self.buf_r_wrist.clear()
                            with self.l_kp_cb_lock:
                                self.buf_l_kps.clear()
                            with self.r_kp_cb_lock:
                                self.buf_r_kps.clear()
                    except Exception as e:
                        self.get_logger().error(f"❌ Inference error: {e}", exc_info=True)
                        time.sleep(0.1)
            finally:
                with self.action_timer_lock:
                    self.infer_event.clear()

    # --- Control timer ---
    def control_timer_cb(self):
        with self.action_timer_lock:
            if self.state != SystemState.RUNNING: return
            with self.commander_lock:
                commander_mode = self.commander_mode
                bootstrap_inference = self.bootstrap_inference

            if commander_mode != 'model':
                return

            if bootstrap_inference and not self.infer_event.is_set():
                self.get_logger().info("Triggering inference: rebuilding action queue after model control resumed")
                self.action_queue_copy = list(self.action_queue)
                self.infer_event.set()
                return

            if len(self.action_queue) == self.act_rtc_len and not self.infer_event.is_set():
                self.get_logger().info(f"Triggering inference")
                self.action_queue_copy = list(self.action_queue)
                self.infer_event.set()
            
            if not self.action_queue:
                if self.infer_event.is_set() or bootstrap_inference:
                    return
                self.get_logger().warn(f"⚠️  Action queue is empty, cannot execute control")
                return

            data = self.action_queue.popleft()
            
            now = self.get_clock().now().to_msg()
            try:
                # 1. Arm
                pa = PoseArray(); pa.header.stamp, pa.header.frame_id = now, "base_link"
                pa.poses.append(pose_from_matrix(data['l']))
                pa.poses.append(pose_from_matrix(data['r']))
                self.pub_action_poses.publish(pa)

                # 2. Hand
                self._publish_keypoints_action(self.pub_action_hand_l, data['lk'], "left_wrist_link", now)
                self._publish_keypoints_action(self.pub_action_hand_r, data['rk'], "right_wrist_link", now)
            except Exception as e:
                self.get_logger().warn(f"⚠️  Action publish failed: {e}")

    def remote_input_loop(self):
        """Blocking instruction input fetched from the host service via HTTP GET."""
        self.get_logger().info(f"🌐 Remote input loop started: waiting to connect to {self.ui_host}:{self.ui_port}")
        while rclpy.ok():
            if self.state == SystemState.IDLE:
                try:
                    url = f"http://{self.ui_host}:{self.ui_port}/get_input"
                    self.get_logger().info(f"📡 Requesting remote input: {url} (no timeout, waiting for user input...)")
                    resp = self.ui_session.get(url, timeout=None).json()
                    
                    self.current_instr = resp["instruction"]
                    self._switch_state(SystemState.READY)

                    self.get_logger().info(f"✅ Instruction received: '{self.current_instr}'")
                except requests.exceptions.ConnectionError as e:
                    self.get_logger().warn(f"⚠️  Cannot connect to host service: {e}")
                    time.sleep(2.0)
                except Exception as e:
                    self.get_logger().warn(f"⚠️  Error fetching remote input: {e}")
                    time.sleep(1.0)
            else:
                time.sleep(0.5)

    def on_key_press(self, key):
        try:
            self._handle_key_press(key)
        except Exception as e:
            self.get_logger().error(f"Key handler error: {e}", exc_info=True)

    def _handle_key_press(self, key):
        try: k = key.char
        except: k = None
        if k != '1':
            if k == '3':
                if not self.human_in_the_loop:
                    self.get_logger().info("User pressed '3', but human_in_the_loop=false, ignoring recording control")
                    return
                self._handle_recording_pedal()
                return

            if k == '2' and self.state in (SystemState.FIRST_OBS, SystemState.RUNNING):
                if not self.human_in_the_loop:
                    self.get_logger().info("User pressed '2', but human_in_the_loop=false, ignoring human control switch")
                    return
                with self.commander_lock:
                    commander_mode = self.commander_mode
                if commander_mode == 'model':
                    self.get_logger().info("🧑‍🔧 User pressed '2': switching to human control")
                    self._transition_commander('human')
                    self.play_sound("expert")
                else:
                    self.get_logger().info("🤖 User pressed '2': switching back to model control")
                    self._transition_commander('model', bootstrap=True)
                    self.play_sound("model")
                return
            return

        if self.state == SystemState.READY:
            self.get_logger().info(f"▶️  User pressed '1': starting first observation (instruction='{self.current_instr}')")
            self._start_human_input_nodes()
            if self.human_in_the_loop:
                self._start_recording_episode()
            self._transition_commander('model')
            self.play_sound("start")
            self.pub_system_mode.publish(String(data="execution"))
            self._switch_state(SystemState.FIRST_OBS)
        elif self.state in (SystemState.FIRST_OBS, SystemState.RUNNING):
            self.get_logger().info("🛑 User pressed '1': stopping execution and resetting system")
            self._transition_commander('model')
            self._stop_recording_if_running()
            self._reset_after_episode_end()

    def find_and_load_calibration(self, cam, arm):
        subdirs = [d for d in os.listdir(self.calib_root) if os.path.isdir(os.path.join(self.calib_root, d))]
        matches = [d for d in subdirs if cam in d and arm in d]
        if not matches:
            self.get_logger().error(f"❌ Calibration data not found: camera={cam}, arm={arm}, path={self.calib_root}")
            raise FileNotFoundError(f"calibration data not found: {cam}/{arm}")
        calib_path = os.path.join(self.calib_root, sorted(matches)[-1], 'calibration_results', 'result.npz')
        self.get_logger().info(f"📂 Loading calibration file: {calib_path}")
        d = np.load(calib_path)
        return d['T_cam2base'], d['camera_matrix']

    def play_sound(self, name):
        url = f"http://{self.ui_host}:{self.ui_port}/play/{self.audio_language}/{name}"
        threading.Thread(target=lambda: self.ui_session.post(url, timeout=0.5), daemon=True).start()

    def _print_buffer_info(self, buf_name, buf):
        self.get_logger().info(f"Buffer Name: {buf_name}")
        self.get_logger().info(f"Buffer Size: {len(buf)}")
        self.get_logger().info(f"Buffer Timestamp: {', '.join([self.get_formatted_time(x[0]) for x in buf])}")

    def get_formatted_time(self, ts_ns):
        seconds = ts_ns / 1e9
        dt = datetime.datetime.fromtimestamp(seconds)
        formatted_time = dt.strftime('%Mm%Ss') + f'{dt.microsecond // 1000:03d}ms'
        return formatted_time


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ModelInterfaceNode()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
