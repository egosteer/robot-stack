#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from cv_bridge import CvBridge


class MockRobotDataNode(Node):
    def __init__(self):
        super().__init__('mock_robot_data')
        self.cv_bridge = CvBridge()

        self.declare_parameter('camera_frequency', 30.0)
        self.camera_frequency = self.get_parameter('camera_frequency').value
        self.camera_names = ['head', 'chest']
        if self.camera_frequency <= 0.0:
            raise ValueError("camera_frequency must be greater than 0")

        # Separate mutex groups so high-frequency timers don't block each other
        self.group_cam = MutuallyExclusiveCallbackGroup()
        self.group_hand = MutuallyExclusiveCallbackGroup()
        self.group_arm = MutuallyExclusiveCallbackGroup()

        self.pub_rgb = {
            camera_name: self.create_publisher(Image, f'/camera/{camera_name}/rgb', 10)
            for camera_name in self.camera_names
        }
        self.pub_depth = {
            camera_name: self.create_publisher(Image, f'/camera/{camera_name}/depth', 10)
            for camera_name in self.camera_names
        }
        
        self.pub_hand_l = self.create_publisher(PoseArray, '/state/left_hand/keypoints', 10)
        self.pub_hand_r = self.create_publisher(PoseArray, '/state/right_hand/keypoints', 10)

        self.pub_arm_l = self.create_publisher(PoseStamped, '/state/left_arm/wrist_pose', 10)
        self.pub_arm_r = self.create_publisher(PoseStamped, '/state/right_arm/wrist_pose', 10)

        # Pre-generate image data once; later only the timestamp is updated,
        # avoiding the cost of generating random data on every callback
        self.rgb_msgs = {}
        self.depth_msgs = {}
        
        for camera_name in self.camera_names:
            rgb_data = np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8)
            rgb_msg = self.cv_bridge.cv2_to_imgmsg(rgb_data, encoding='rgb8')
            rgb_msg.header.frame_id = f"{camera_name}_camera_optical_frame"
            self.rgb_msgs[camera_name] = rgb_msg

            depth_data = np.random.randint(500, 1500, size=(480, 640), dtype=np.uint16)
            depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_data, encoding='16UC1')
            depth_msg.header.frame_id = f"{camera_name}_camera_optical_frame"
            self.depth_msgs[camera_name] = depth_msg

        # Camera timer: 30Hz (~33.3ms)
        self.create_timer(1.0 / self.camera_frequency, self.timer_cam_cb, callback_group=self.group_cam)

        # Hand timer: 80Hz (12.5ms)
        self.create_timer(1.0/80.0, self.timer_hand_cb, callback_group=self.group_hand)

        # Arm timer: 100Hz (10ms)
        self.create_timer(1.0/100.0, self.timer_arm_cb, callback_group=self.group_arm)

        self.get_logger().info(
            f"Mock data generator started | cameras={self.camera_names} | image mode=RGB-D | "
            f"camera: {self.camera_frequency}Hz | hand: 80Hz | arm: 100Hz"
        )

    def timer_cam_cb(self):
        """Simulate 30Hz camera data."""
        now = self.get_clock().now().to_msg()

        for camera_name in self.camera_names:
            self.rgb_msgs[camera_name].header.stamp = now
            self.pub_rgb[camera_name].publish(self.rgb_msgs[camera_name])

            self.depth_msgs[camera_name].header.stamp = now
            self.pub_depth[camera_name].publish(self.depth_msgs[camera_name])

    def timer_hand_cb(self):
        """Simulate 80Hz 5-finger hand keypoint data."""
        now = self.get_clock().now().to_msg()

        def create_mock_kps(side):
            pa = PoseArray()
            pa.header.stamp = now
            pa.header.frame_id = f"{side}_wrist_link"
            for i in range(5): # 5 fingers
                p = Pose()
                p.position.x = 0.05 + 0.01 * i
                p.position.y = 0.02 * np.sin(time.time() * 2) # small sinusoidal wobble
                p.position.z = 0.1
                pa.poses.append(p)
            return pa

        self.pub_hand_l.publish(create_mock_kps("left"))
        self.pub_hand_r.publish(create_mock_kps("right"))

    def timer_arm_cb(self):
        """Simulate 100Hz arm end-effector pose."""
        now = self.get_clock().now().to_msg()

        def create_mock_pose(side):
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = "base_link"
            # Baseline positions for the robot's left/right hands
            ps.pose.position.x = 0.4
            ps.pose.position.y = 0.2 if side == "left" else -0.2
            ps.pose.position.z = 0.3 + 0.05 * np.cos(time.time())
            # Identity quaternion
            ps.pose.orientation.w = 1.0
            return ps

        self.pub_arm_l.publish(create_mock_pose("left"))
        self.pub_arm_r.publish(create_mock_pose("right"))

def main(args=None):
    rclpy.init(args=args)
    node = MockRobotDataNode()
    # Multi-threaded executor so high-frequency timers are not blocked
    executor = MultiThreadedExecutor(num_threads=4)
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
