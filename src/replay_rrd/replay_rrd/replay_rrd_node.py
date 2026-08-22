#!/usr/bin/env python3

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence

import mujoco
import numpy as np
import rclpy
import rerun as rr

try:
    import rerun_bindings as rrb
except Exception:
    rrb = None
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String


# Mirror of hand_ik_node.MOTOR_POSITION_MULTIPLIER. Hand action/state topics
# carry motor-cmd-scaled values; divide them back to 0..1 normalized angles
# before feeding the FK model (same as hand_fk_node.compute_fk).
HAND_MOTOR_POSITION_MULTIPLIER = [0.6, 1.0, 1.0, 1.0, 1.0, 1.0]


def _repo_root_from_this_file() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        candidate = parent / 'assets' / 'robot_mjcf' / 'robot_scene.xml'
        if candidate.exists():
            return parent
    return Path('/root/workspace/robot-stack')


def _default_robot_xml_path() -> str:
    repo_root = _repo_root_from_this_file()
    return str(repo_root / 'assets' / 'robot_mjcf' / 'robot_scene.xml')


def _load_rrd_recording(rrd_path: str):
    if hasattr(rr, 'recording') and hasattr(rr.recording, 'load_recording'):
        return rr.recording.load_recording(rrd_path)

    if hasattr(rr, 'dataframe') and hasattr(rr.dataframe, 'load_recording'):
        return rr.dataframe.load_recording(rrd_path)

    if hasattr(rr, 'load_recording'):
        return rr.load_recording(rrd_path)

    if rrb is not None and hasattr(rrb, 'load_recording'):
        return rrb.load_recording(rrd_path)

    raise RuntimeError('The current rerun version does not support load_recording; install a rerun-sdk that includes the dataframe/recording API.')


def _resolve_hand_mjcf_file(hand_type: str) -> Path:
    repo_root = _repo_root_from_this_file()
    candidate = repo_root / 'assets' / 'ruiyan_hand_mjcf' / hand_type / 'hand.xml'
    if candidate.exists():
        return candidate

    raise FileNotFoundError(f'MJCF file for {hand_type} hand not found at {candidate}')


@dataclass
class ReplayFrame:
    left_arm: np.ndarray
    right_arm: np.ndarray
    left_hand: np.ndarray
    right_hand: np.ndarray
    timestamp_sec: float


class ArmFKSolver:
    def __init__(self, xml_path: str):
        self.xml_path = xml_path
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.left_wrist_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'left_wrist')
        self.right_wrist_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'right_wrist')
        self.left_arm_base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'arm1_link0')
        self.right_arm_base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'arm2_link0')

    def compute_fk(self, left_arm_joints: np.ndarray, right_arm_joints: np.ndarray) -> Dict[str, Dict[str, np.ndarray]]:
        joint_angles = np.concatenate([left_arm_joints, right_arm_joints], axis=0).astype(np.float64)
        if joint_angles.shape[0] != self.model.nq:
            raise ValueError(f'Arm joint dimension mismatch: expected {self.model.nq}, got {joint_angles.shape[0]}')

        self.data.qpos[:] = joint_angles
        mujoco.mj_forward(self.model, self.data)

        left_base_pos = self.data.body(self.left_arm_base_id).xpos.copy()
        left_base_mat = self.data.body(self.left_arm_base_id).xmat.copy().reshape(3, 3)
        right_base_pos = self.data.body(self.right_arm_base_id).xpos.copy()
        right_base_mat = self.data.body(self.right_arm_base_id).xmat.copy().reshape(3, 3)

        left_tcp_pos = self.data.site(self.left_wrist_site_id).xpos.copy()
        left_tcp_mat = self.data.site(self.left_wrist_site_id).xmat.copy().reshape(3, 3)
        right_tcp_pos = self.data.site(self.right_wrist_site_id).xpos.copy()
        right_tcp_mat = self.data.site(self.right_wrist_site_id).xmat.copy().reshape(3, 3)

        left_tcp_pos_in_base = left_base_mat.T @ (left_tcp_pos - left_base_pos)
        left_tcp_mat_in_base = left_base_mat.T @ left_tcp_mat
        right_tcp_pos_in_base = right_base_mat.T @ (right_tcp_pos - right_base_pos)
        right_tcp_mat_in_base = right_base_mat.T @ right_tcp_mat

        return {
            'left': {
                'position': left_tcp_pos_in_base,
                'orientation': R.from_matrix(left_tcp_mat_in_base).as_quat(),
            },
            'right': {
                'position': right_tcp_pos_in_base,
                'orientation': R.from_matrix(right_tcp_mat_in_base).as_quat(),
            },
        }


class HandFKSolver:
    def __init__(self, hand_type='left'):
        self.hand_type = hand_type
        self.model = mujoco.MjModel.from_xml_path(str(_resolve_hand_mjcf_file(hand_type)))
        self.data = mujoco.MjData(self.model)
        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']

        self._setup_joint_info()
        self._record_joint_metadata()
        self._setup_site_info()
        mujoco.mj_forward(self.model, self.data)

    def _setup_joint_info(self):
        all_joints = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)]
        prefix = 'hand1' if self.hand_type == 'left' else 'hand2'
        active_candidates = [
            f'{prefix}_joint_link_1_1', f'{prefix}_joint_link_1_2',
            f'{prefix}_joint_link_2_1', f'{prefix}_joint_link_3_1',
            f'{prefix}_joint_link_4_1', f'{prefix}_joint_link_5_1',
        ]
        self.joint_names = [joint_name for joint_name in active_candidates if joint_name in all_joints]

    def _record_joint_metadata(self):
        self.joint_metadata = []
        for name in self.joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qpos_adr = self.model.jnt_qposadr[joint_id]
            lower, upper = self.model.jnt_range[joint_id]
            self.joint_metadata.append({
                'name': name,
                'adr': qpos_adr,
                'low': lower,
                'range': (upper - lower) if (upper - lower) != 0 else 1.0,
            })

    def _setup_site_info(self):
        side_prefix = self.hand_type
        self.site_ids_list = []
        for finger_name in self.finger_names:
            site_name = f'{side_prefix}_{finger_name}_tip'
            try:
                self.site_ids_list.append(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name))
            except Exception:
                pass

    def _get_mimic_relations(self):
        prefix = 'hand1' if self.hand_type == 'left' else 'hand2'
        return [
            (f'{prefix}_joint_link_1_2', f'{prefix}_joint_link_1_3', 1.675, 0.0),
            (f'{prefix}_joint_link_2_1', f'{prefix}_joint_link_2_2', 1.0, 0.0),
            (f'{prefix}_joint_link_3_1', f'{prefix}_joint_link_3_2', 1.0, 0.0),
            (f'{prefix}_joint_link_4_1', f'{prefix}_joint_link_4_2', 1.0, 0.0),
            (f'{prefix}_joint_link_5_1', f'{prefix}_joint_link_5_2', 1.0, 0.0),
        ]

    def _apply_mimic_joints(self):
        for leader_name, follower_name, multiplier, offset in self._get_mimic_relations():
            try:
                leader_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, leader_name)
                follower_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, follower_name)
                leader_adr = self.model.jnt_qposadr[leader_id]
                follower_adr = self.model.jnt_qposadr[follower_id]
                self.data.qpos[follower_adr] = offset + multiplier * self.data.qpos[leader_adr]
            except Exception:
                pass

    def compute_fk(self, normalized_positions: np.ndarray):
        if len(normalized_positions) != len(self.joint_metadata):
            return None

        adjusted = np.clip(
            np.asarray(normalized_positions, dtype=np.float64) / np.asarray(HAND_MOTOR_POSITION_MULTIPLIER),
            0.0,
            1.0,
        )

        for index, metadata in enumerate(self.joint_metadata):
            value_rad = metadata['low'] + adjusted[index] * metadata['range']
            self.data.qpos[metadata['adr']] = value_rad

        self._apply_mimic_joints()
        mujoco.mj_forward(self.model, self.data)

        results = []
        for site_id in self.site_ids_list:
            pos = self.data.site(site_id).xpos.copy()
            quat = R.from_matrix(self.data.site(site_id).xmat.copy().reshape(3, 3)).as_quat()
            results.append((pos, quat))
        return results


class ReplayRRDNode(Node):
    def __init__(self):
        super().__init__('replay_rrd')

        self.declare_parameter('rrd_file_path', '')
        self.declare_parameter('target_hz', 30.0)

        self.rrd_file_path = self.get_parameter('rrd_file_path').value
        self.target_hz = float(self.get_parameter('target_hz').value)
        self.robot_xml_path = _default_robot_xml_path()

        if not self.rrd_file_path:
            raise RuntimeError('Parameter rrd_file_path must not be empty; specify the .rrd file to replay in the config.')
        if not os.path.exists(self.rrd_file_path):
            raise RuntimeError(f'RRD file does not exist: {self.rrd_file_path}')
        if not os.path.exists(self.robot_xml_path):
            raise RuntimeError(f'Arm FK XML does not exist: {self.robot_xml_path}')

        self.arm_action_pub = self.create_publisher(PoseArray, '/action/both_arms/wrist_poses', 10)
        self.left_hand_action_pub = self.create_publisher(PoseArray, '/action/left_hand/keypoints', 10)
        self.right_hand_action_pub = self.create_publisher(PoseArray, '/action/right_hand/keypoints', 10)
        self.system_mode_pub = self.create_publisher(String, '/system/mode', 10)
        self.commander_pub = self.create_publisher(String, '/commander', 10)

        self.arm_fk_solver = ArmFKSolver(self.robot_xml_path)
        self.left_hand_fk_solver = HandFKSolver(hand_type='left')
        self.right_hand_fk_solver = HandFKSolver(hand_type='right')

        self.frames = self._load_replay_frames(self.rrd_file_path, self.target_hz)
        self.frame_index = 0
        self._done = False
        self._finishing = False
        self._finish_ticks = 0
        # arm_ik / hand_ik default to ('execution', 'model') at startup, but if
        # model_interface ran earlier it may have left mode='reset'. Republish
        # both topics so replay is self-sufficient; do it a few times to ride
        # past any publisher-discovery race.
        self._mode_publish_remaining = 5
        self._publish_execution_state()

        self.timer = self.create_timer(1.0 / self.target_hz, self._timer_callback)

        duration_sec = self.frames[-1].timestamp_sec - self.frames[0].timestamp_sec if len(self.frames) > 1 else 0.0
        self.get_logger().info(
            f'ReplayRRDNode ready: {len(self.frames)} frames, {self.target_hz:.1f} Hz, duration {duration_sec:.2f}s, file={self.rrd_file_path}'
        )

    def _load_replay_frames(self, rrd_path: str, target_hz: float) -> List[ReplayFrame]:
        if target_hz <= 0:
            raise RuntimeError('target_hz must be positive.')

        recording = _load_rrd_recording(rrd_path)
        view = recording.view(index='timestamp', contents='/**')
        schema_column_names = list(view.select().schema.names)

        left_arm_columns = self._choose_arm_columns(schema_column_names, 'left')
        right_arm_columns = self._choose_arm_columns(schema_column_names, 'right')
        left_hand_columns = self._choose_hand_columns(schema_column_names, 'left')
        right_hand_columns = self._choose_hand_columns(schema_column_names, 'right')

        selected_columns = self._dedupe_columns([
            'timestamp',
            *left_arm_columns,
            *right_arm_columns,
            *left_hand_columns,
            *right_hand_columns,
        ])
        table = view.select(columns=selected_columns).read_all()
        timestamps_sec = self._timestamps_to_seconds(table['timestamp'])

        left_arm = self._extract_joint_stream(table, timestamps_sec, left_arm_columns, 'left arm')
        right_arm = self._extract_joint_stream(table, timestamps_sec, right_arm_columns, 'right arm')
        left_hand = self._extract_joint_stream(table, timestamps_sec, left_hand_columns, 'left hand')
        right_hand = self._extract_joint_stream(table, timestamps_sec, right_hand_columns, 'right hand')

        common_start = max(left_arm['timestamp'][0], right_arm['timestamp'][0], left_hand['timestamp'][0], right_hand['timestamp'][0])
        common_end = min(left_arm['timestamp'][-1], right_arm['timestamp'][-1], left_hand['timestamp'][-1], right_hand['timestamp'][-1])
        if common_end <= common_start:
            raise RuntimeError('Cannot build a common replay timeline: the four streams have no valid overlapping interval.')

        dt = 1.0 / target_hz
        target_timestamps = np.arange(common_start, common_end + 0.5 * dt, dt, dtype=np.float64)
        if target_timestamps.size == 0:
            raise RuntimeError('Downsampled timeline is empty; check the RRD data and the target_hz parameter.')

        left_arm_samples = self._resample_stream(left_arm, target_timestamps)
        right_arm_samples = self._resample_stream(right_arm, target_timestamps)
        left_hand_samples = self._resample_stream(left_hand, target_timestamps)
        right_hand_samples = self._resample_stream(right_hand, target_timestamps)

        frames: List[ReplayFrame] = []
        for ts, l_arm, r_arm, l_hand, r_hand in zip(
            target_timestamps,
            left_arm_samples,
            right_arm_samples,
            left_hand_samples,
            right_hand_samples,
        ):
            frames.append(
                ReplayFrame(
                    left_arm=np.asarray(l_arm, dtype=np.float64),
                    right_arm=np.asarray(r_arm, dtype=np.float64),
                    left_hand=np.asarray(l_hand, dtype=np.float64),
                    right_hand=np.asarray(r_hand, dtype=np.float64),
                    timestamp_sec=float(ts),
                )
            )

        return frames

    def _timestamps_to_seconds(self, timestamp_column) -> np.ndarray:
        values = timestamp_column.to_numpy(zero_copy_only=False)
        if np.issubdtype(values.dtype, np.datetime64):
            return values.astype('datetime64[ns]').astype(np.int64).astype(np.float64) / 1e9

        if np.issubdtype(values.dtype, np.number):
            numeric_values = values.astype(np.float64)
            if numeric_values.size > 0 and np.nanmax(np.abs(numeric_values)) > 1e12:
                return numeric_values / 1e9
            return numeric_values

        converted = []
        for value in timestamp_column.to_pylist():
            if hasattr(value, 'timestamp'):
                converted.append(value.timestamp())
            else:
                converted.append(float(value))
        return np.asarray(converted, dtype=np.float64)

    def _dedupe_columns(self, columns: Sequence[str]) -> List[str]:
        result = []
        seen = set()
        for column in columns:
            if column in seen:
                continue
            seen.add(column)
            result.append(column)
        return result

    def _scalar_column_groups(self, path_prefix: str, joint_names: Sequence[str]) -> List[List[str]]:
        return [
            [f'{path_prefix}/{joint_name}:Scalar' for joint_name in joint_names],
            [f'{path_prefix}/{joint_name}:Scalars:scalars' for joint_name in joint_names],
        ]

    def _extract_scalar_value(self, value):
        while isinstance(value, (list, tuple, np.ndarray)):
            if len(value) == 0:
                return np.nan
            value = value[0]

        if value is None:
            return np.nan

        try:
            return float(value)
        except Exception:
            return np.nan

    def _find_columns_by_tokens(self, column_names, joint_tokens, prefix_candidates):
        matched_columns = []
        for token in joint_tokens:
            token_matches = []
            for column_name in column_names:
                if not any(column_name.startswith(prefix) for prefix in prefix_candidates):
                    continue

                entity_path = column_name.split(':', 1)[0]
                joint_name = entity_path.rsplit('/', 1)[-1]
                if joint_name != token:
                    continue

                if ':Scalars:' not in column_name and ':Scalar' not in column_name:
                    continue
                token_matches.append(column_name)

            token_matches = sorted(token_matches, key=len)
            if not token_matches:
                return None
            matched_columns.append(token_matches[0])

        return matched_columns

    def _select_joint_columns(self, column_names, candidate_specs, stream_name: str) -> List[str]:
        for spec in candidate_specs:
            for exact_columns in spec.get('exact_column_groups', []):
                if all(column in column_names for column in exact_columns):
                    return exact_columns

            token_columns = self._find_columns_by_tokens(
                column_names=column_names,
                joint_tokens=spec.get('joint_tokens', []),
                prefix_candidates=spec.get('prefixes', []),
            )
            if token_columns is not None:
                return token_columns

        related_columns = [
            name for name in column_names
            if any(prefix in name for spec in candidate_specs for prefix in spec.get('prefixes', []))
        ]
        raise RuntimeError(
            f'RRD is missing {stream_name} columns. No candidate group matched: {candidate_specs}\n'
            f'{stream_name} related available columns: {related_columns[:80]}'
        )

    def _extract_joint_stream(self, table, timestamps_sec: np.ndarray, chosen_columns, stream_name: str) -> Dict[str, np.ndarray]:
        missing_columns = [column for column in chosen_columns if column not in table.column_names]
        if missing_columns:
            raise RuntimeError(f'RRD is missing {stream_name} columns: {missing_columns}')

        data_list = []
        for column in chosen_columns:
            processed = [self._extract_scalar_value(value) for value in table[column].to_pylist()]
            data_list.append(np.asarray(processed, dtype=np.float64))

        joint_data = np.column_stack(data_list)
        valid_mask = ~np.isnan(joint_data).any(axis=1)

        if valid_mask.sum() == 0:
            raise RuntimeError(f'{stream_name} data is empty.')

        self.get_logger().info(f'{stream_name} using columns: {chosen_columns}')
        return {
            'timestamp': timestamps_sec[valid_mask],
            'data': joint_data[valid_mask],
        }

    def _joint_candidate_specs(self, path_prefixes: Sequence[str], joint_names: Sequence[str]):
        return [
            {
                'exact_column_groups': self._scalar_column_groups(prefix, joint_names),
                'prefixes': [prefix],
                'joint_tokens': list(joint_names),
            }
            for prefix in path_prefixes
        ]

    def _arm_candidate_specs(self, side: str):
        # ARM1 = left arm, ARM2 = right arm.
        arm_prefix = 'arm1' if side == 'left' else 'arm2'
        joint_names = [f'{arm_prefix}_joint_link{i+1}' for i in range(7)]
        # Prefer the action stream; fall back to the state stream.
        return self._joint_candidate_specs(
            [f'/action/{side}_arm/joints/position', f'/state/{side}_arm/joints/position'],
            joint_names,
        )

    def _choose_arm_columns(self, column_names, side: str) -> List[str]:
        return self._select_joint_columns(column_names, self._arm_candidate_specs(side), f'{side} arm')

    def _hand_candidate_specs(self, side: str):
        joint_names = ['thumb_rotate', 'thumb_bend', 'index_bend', 'middle_bend', 'ring_bend', 'pinky_bend']
        # Prefer the action stream; fall back to the state stream.
        return self._joint_candidate_specs(
            [f'/action/{side}_hand/joints/position', f'/state/{side}_hand/joints/position'],
            joint_names,
        )

    def _choose_hand_columns(self, column_names, side: str) -> List[str]:
        return self._select_joint_columns(column_names, self._hand_candidate_specs(side), f'{side} hand')

    def _resample_stream(self, stream: Dict[str, np.ndarray], target_timestamps: np.ndarray) -> List[np.ndarray]:
        source_timestamps = stream['timestamp']
        source_data = stream['data']
        samples = []
        for target_ts in target_timestamps:
            idx = np.searchsorted(source_timestamps, target_ts, side='right') - 1
            idx = max(0, min(idx, len(source_timestamps) - 1))
            samples.append(source_data[idx])
        return samples

    def _timer_callback(self):
        if self._finishing:
            self._publish_reset_state()
            self._finish_ticks -= 1
            if self._finish_ticks <= 0:
                self._done = True
            return

        if self._mode_publish_remaining > 0:
            self._publish_execution_state()
            self._mode_publish_remaining -= 1

        if self.frame_index >= len(self.frames):
            self.get_logger().info('Replay completed, homing and exiting.')
            self._finishing = True
            self._finish_ticks = max(1, int(round(0.5 * self.target_hz)))
            self._publish_reset_state()
            return

        frame = self.frames[self.frame_index]
        now = self.get_clock().now().to_msg()

        arm_fk = self.arm_fk_solver.compute_fk(frame.left_arm, frame.right_arm)
        left_hand_fk = self.left_hand_fk_solver.compute_fk(frame.left_hand)
        right_hand_fk = self.right_hand_fk_solver.compute_fk(frame.right_hand)

        self.arm_action_pub.publish(self._build_arm_pose_array(now, arm_fk))
        self.left_hand_action_pub.publish(self._build_hand_pose_array(now, 'left', left_hand_fk))
        self.right_hand_action_pub.publish(self._build_hand_pose_array(now, 'right', right_hand_fk))

        self.frame_index += 1

    def _build_arm_pose_array(self, stamp, arm_fk: Dict[str, Dict[str, np.ndarray]]) -> PoseArray:
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'

        for side in ('left', 'right'):
            pose = Pose()
            pose.position.x = float(arm_fk[side]['position'][0])
            pose.position.y = float(arm_fk[side]['position'][1])
            pose.position.z = float(arm_fk[side]['position'][2])
            pose.orientation.x = float(arm_fk[side]['orientation'][0])
            pose.orientation.y = float(arm_fk[side]['orientation'][1])
            pose.orientation.z = float(arm_fk[side]['orientation'][2])
            pose.orientation.w = float(arm_fk[side]['orientation'][3])
            msg.poses.append(pose)

        return msg

    def _publish_execution_state(self):
        self.system_mode_pub.publish(String(data='execution'))
        self.commander_pub.publish(String(data='model'))

    def _publish_reset_state(self):
        try:
            self.system_mode_pub.publish(String(data='reset'))
            self.commander_pub.publish(String(data='model'))
        except Exception:
            pass

    def destroy_node(self):
        self._publish_reset_state()
        return super().destroy_node()

    def _build_hand_pose_array(self, stamp, side: str, pose_list) -> PoseArray:
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = f'{side}_wrist_link'

        if pose_list is None:
            return msg

        for pos, _ in pose_list:
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            msg.poses.append(pose)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReplayRRDNode()
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
