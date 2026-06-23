import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

try:
    from .vive_tracker import ViveTrackerModule
except Exception:
    import pathlib

    sys.path.append(str(pathlib.Path(__file__).parent))
    from vive_tracker import ViveTrackerModule


def get_init_tcp_pose_mat(hand_name):
    tcp_mat = np.eye(4)
    if hand_name == 'right':
        tcp_mat[:3, 3] = np.array([0.2, -0.45, 1.0])
        tcp_mat[:3, :3] = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]).reshape(3, 3)
        return tcp_mat
    if hand_name == 'left':
        tcp_mat[:3, 3] = np.array([0.2, 0.45, 1.0])
        tcp_mat[:3, :3] = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]).reshape(3, 3)
        return tcp_mat
    raise ValueError(f"Invalid hand name: {hand_name}")


class TrackerNode(Node):
    def __init__(self):
        super().__init__('robot_vive_tracker_node')

        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('base_frame', 'map')
        self.declare_parameter('serial_number', '')
        self.declare_parameter('hand_name', 'left')

        self.publish_rate = self.get_parameter('publish_rate').value
        self.base_frame = self.get_parameter('base_frame').value
        self.hand_name = self.get_parameter('hand_name').value
        self.serial_number = self.get_parameter('serial_number').value
        self.device_key = 'tracker'

        self.vive_tracker_module = ViveTrackerModule()
        self.vive_tracker_module.print_discovered_objects()

        all_tracking_devices = self.vive_tracker_module.return_selected_devices(self.device_key)
        self.tracking_device = self._get_device_by_serial(all_tracking_devices)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'/action/{self.hand_name}_tracker/pose',
            10,
        )

        self.rot_cali_mat = np.eye(4)
        self.rot_cali_mat[:3, :3] = R.from_euler('xyz', [np.pi, 0.0, -np.pi / 2]).as_matrix()
        self.world_rot_cali_mat = np.eye(4)
        self.world_rot_cali_mat[:3, :3] = R.from_euler('xyz', [np.pi / 2, 0.0, np.pi]).as_matrix()

        self.tracker2wrist_mat = np.eye(4)
        self.tracker2wrist_mat[:3, 3] = np.array([0.0, 0.0, -0.08])

        self.init_tcp_pose_mat = get_init_tcp_pose_mat(self.hand_name)
        self.init_tracker_pose = self.calibrate_tracker_system(self.tracking_device.get_T().copy())
        self.prev_position = self.init_tcp_pose_mat[:3, 3]

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def calibrate_tracker_system(self, T):
        return self.world_rot_cali_mat @ T @ self.rot_cali_mat @ self.tracker2wrist_mat

    def _get_device_by_serial(self, all_devices):
        serial_to_device = {}
        for _, device in all_devices.items():
            serial_to_device[device.get_serial()] = device

        if self.serial_number not in serial_to_device:
            available_serials = list(serial_to_device.keys())
            self.get_logger().error(f'❌ Serial number {self.serial_number} not found')
            self.get_logger().error(f'Available serial numbers: {available_serials}')
            raise KeyError(f'Serial number {self.serial_number} not present among discovered devices')

        device = serial_to_device[self.serial_number]
        self.get_logger().info(f'✅ Device with serial {self.serial_number} maps to the {self.hand_name} hand')
        return device

    def timer_callback(self):
        current_time = self.get_clock().now()

        T = self.tracking_device.get_T().copy()
        T = self.calibrate_tracker_system(T)
        delta_T = np.linalg.inv(self.init_tracker_pose) @ T

        robot_tcp_target_pose = self.init_tcp_pose_mat @ delta_T
        T = robot_tcp_target_pose

        position = T[:3, 3]
        rotation_matrix = T[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()

        _change = np.linalg.norm(position - self.prev_position)
        if _change > 0.1:
            self.get_logger().warn(f"position jump: {_change} m")
            return
        self.prev_position = position

        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = self.base_frame

        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])

        pose_msg.pose.orientation.x = float(quaternion[0])
        pose_msg.pose.orientation.y = float(quaternion[1])
        pose_msg.pose.orientation.z = float(quaternion[2])
        pose_msg.pose.orientation.w = float(quaternion[3])

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TrackerNode()
        node.get_logger().info("Vive Tracker started publishing data...")
        node.get_logger().info("Press Ctrl+C to stop the node")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Interrupt signal received")
    except Exception as e:
        if rclpy.ok():
            print(f"❌ Tracker node exception: {e}")
            import traceback
            traceback.print_exc()
    finally:
        print("🧹 Cleaning up tracker node...")
        if node:
            try:
                node.destroy_node()
                print("📴 Tracker node destroyed")
            except KeyboardInterrupt:
                print("📴 Tracker node cleanup interrupted")
            except Exception as e:
                print(f"Tracker node destroy exception: {e}")
        if rclpy.ok():
            try:
                rclpy.shutdown()
                print("📴 ROS2 shut down")
            except KeyboardInterrupt:
                print("📴 ROS2 shutdown interrupted")
            except Exception as e:
                print(f"ROS2 shutdown exception: {e}")
        print("✅ Tracker node fully exited")


if __name__ == "__main__":
    main()
