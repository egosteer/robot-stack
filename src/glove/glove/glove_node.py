import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from .glove_reader import SimpleGloveReader


ANGLE_INDICES = {
    'left': [4, 2, 7, 11, 15, 19],
    'right': [4, 2, 7, 11, 15, 19],
}

JOINT_NAMES = [
    'thumb_rotate',
    'thumb_bend',
    'index_bend',
    'middle_bend',
    'ring_bend',
    'pinky_bend',
]

GLOVE_ANGLE_MIN = {
    'left': [305, 260, 235, 235, 235, 270],
    'right': [50, 257, 240, 245, 240, 260],
}
GLOVE_ANGLE_MAX = {
    'left': [299, 210, 130, 125, 125, 150],
    'right': [60, 220, 155, 130, 130, 150],
}
CLIP_MIN = {
    'left': [0, 0, 0, 0, 0, 0],
    'right': [0, 0, 0, 0, 0, 0],
}
CLIP_MAX = {
    'left': [1, 1, 1, 1, 1, 1],
    'right': [1, 1, 1, 1, 1, 1],
}


def precise_wait_until(time_end, dt=0.001):
    while True:
        if time.time() >= time_end:
            break
        time.sleep(dt)


class LowPassFilter:
    def __init__(self, delta=0.1):
        self.delta = delta
        self.filtered_values = None

    def filter(self, values):
        if self.filtered_values is None:
            self.filtered_values = np.array(values, dtype=np.float64)
        else:
            _delta = np.array(values, dtype=np.float64) - self.filtered_values
            _delta = np.clip(_delta, -self.delta, self.delta)
            self.filtered_values = self.filtered_values + _delta
        return self.filtered_values.tolist()

    def reset(self):
        self.filtered_values = None


class GloveNode(Node):
    def __init__(self):
        super().__init__('glove_node')

        self.declare_parameter('glove_port', '/dev/glove_left')
        self.declare_parameter('hand_name', 'left')
        self.declare_parameter('publish_rate', 80.0)
        self.declare_parameter('glove_read_rate', 200.0)
        self.declare_parameter('enable_smooth', True)
        self.declare_parameter('low_pass_delta', 0.1)
        self.declare_parameter('data_expired_duration', 0.5)

        self.glove_port = self.get_parameter('glove_port').value
        self.hand_name = self.get_parameter('hand_name').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.glove_read_rate = float(self.get_parameter('glove_read_rate').value)
        self.enable_smooth = bool(self.get_parameter('enable_smooth').value)
        self.low_pass_delta = float(self.get_parameter('low_pass_delta').value)
        self.data_expired_duration = float(self.get_parameter('data_expired_duration').value)

        if self.hand_name not in ('left', 'right'):
            raise ValueError('hand_name must be left or right')

        self.action_pub = self.create_publisher(
            JointState,
            f'/action/{self.hand_name}_glove/joints',
            10,
        )

        self.glove_controller = SimpleGloveReader(
            port=self.glove_port,
            baudrate=500000,
            timeout=0.02,
        )

        self.low_pass_filter = LowPassFilter(delta=self.low_pass_delta)
        self.action_angles_queue = deque(maxlen=max(1, int(self.glove_read_rate * 0.1)))
        self.action_angles = None
        self.action_angles_stamp = None
        self._lock = threading.Lock()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(f'{self.hand_name} glove node started on {self.glove_port}')

    def _reader_loop(self):
        dt = 1.0 / self.glove_read_rate
        while rclpy.ok():
            t0 = time.time()
            raw_angles = self.glove_controller.read_angles()
            if raw_angles and len(raw_angles) == 21:
                processed = self.process_action_angles(raw_angles)
                with self._lock:
                    self.action_angles = processed
                    self.action_angles_stamp = self.get_clock().now()
            precise_wait_until(t0 + dt)

    def process_action_angles(self, action_angles):
        action_angles = np.array(action_angles, dtype=np.float64)[ANGLE_INDICES[self.hand_name]].tolist()

        for i, angle in enumerate(action_angles):
            if abs(angle) > 600:
                action_angles[i] = 0.0

        action_angles = [
            (angle - GLOVE_ANGLE_MIN[self.hand_name][i]) /
            (GLOVE_ANGLE_MAX[self.hand_name][i] - GLOVE_ANGLE_MIN[self.hand_name][i])
            for i, angle in enumerate(action_angles)
        ]
        action_angles = np.clip(action_angles, CLIP_MIN[self.hand_name], CLIP_MAX[self.hand_name]).tolist()

        if self.enable_smooth:
            action_angles = self.low_pass_filter.filter(action_angles)
            self.action_angles_queue.append(action_angles)
            action_angles = np.mean(np.array(self.action_angles_queue), axis=0).tolist()

        return np.clip(action_angles, 0.0, 1.0).tolist()

    def timer_callback(self):
        current_time = self.get_clock().now()
        with self._lock:
            action_angles = None if self.action_angles is None else list(self.action_angles)
            stamp = self.action_angles_stamp

        if action_angles is None or stamp is None:
            return

        if (current_time - stamp) > Duration(seconds=self.data_expired_duration):
            self.get_logger().warn(
                f'{self.hand_name} glove data expired after {self.data_expired_duration:.2f}s'
            )
            return

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = stamp.to_msg()
        msg.name = JOINT_NAMES
        msg.position = action_angles
        self.action_pub.publish(msg)

    def destroy_node(self):
        try:
            self.glove_controller.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GloveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
