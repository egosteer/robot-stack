#!/usr/bin/env python3
"""
Camera Node - publishes RGB and depth images at a fixed rate for the inference system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camera.realsense_image_module import RealSenseImage
from rclpy.qos import qos_profile_sensor_data


class CameraNode(Node):
    """
    Reads images from a RealSense camera and publishes them to ROS topics.

    - Rate: 30 Hz
    - Output: Image
    - Topics: /camera/{head,chest}/depth and /camera/{head,chest}/rgb
    """

    def __init__(self):
        super().__init__('camera_node')
        self.frame_count = 0

        self.declare_parameter('camera_name', 'head')  # 'head' or 'chest'
        self.declare_parameter('serial_number', '')
        self.declare_parameter('frequency', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        serial_number = self.get_parameter('serial_number').get_parameter_value().string_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        
        if self.frequency <= 0.0:
            raise ValueError('frequency must be greater than 0')

        # Empty serial string means use the default device.
        self.serial_number = serial_number if serial_number else None

        self.rgb_publisher = None
        self.depth_publisher = None

        self.rgb_publisher = self.create_publisher(
            Image,
            f'/camera/{self.camera_name}/rgb',
            qos_profile_sensor_data
        )
        self.depth_publisher = self.create_publisher(
            Image,
            f'/camera/{self.camera_name}/depth',
            qos_profile_sensor_data
        )
        
        self.bridge = CvBridge()

        try:
            self.realsense_camera = RealSenseImage(
                SN_number=self.serial_number,
                width=self.width,
                height=self.height,
                fps=int(self.frequency),
                enable_depth=True
            )
            self.get_logger().info('RealSense camera initialized successfully')
            if self.serial_number:
                self.get_logger().info(f'Using serial number: {self.serial_number}')
            else:
                self.get_logger().info('Using default RealSense device')
        except Exception as e:
            self.get_logger().error(f"Failed to initialize RealSense camera: {str(e)}")
            raise

        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.capture_and_publish)

        self.get_logger().info('Camera Node started')
        self.get_logger().info(f'Camera name: {self.camera_name}')
        self.get_logger().info(f'Publish rate: {self.frequency} Hz')
        self.get_logger().info(f'Image size: {self.width}x{self.height}')
        self.get_logger().info('Publish RGB: True')
        self.get_logger().info('Publish Depth: True')
        self.get_logger().info('Topics:')
        self.get_logger().info(f'  - /camera/{self.camera_name}/rgb (Image)')
        self.get_logger().info(f'  - /camera/{self.camera_name}/depth (Image)')

    def capture_and_publish(self):
        try:
            rgb_frame, depth_frame = self.realsense_camera.capture_rgb_depth_frames()

            rgb_ready = rgb_frame is not None
            depth_ready = depth_frame is not None

            if rgb_ready and depth_ready:
                current_time = self.get_clock().now().to_msg()
                frame_id = f'{self.camera_name}_camera_frame'

                # RealSense outputs RGB format.
                ros_rgb = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='rgb8')
                ros_rgb.header.stamp = current_time
                ros_rgb.header.frame_id = frame_id
                self.rgb_publisher.publish(ros_rgb)

                # Depth is 16-bit single channel.
                ros_depth = self.bridge.cv2_to_imgmsg(depth_frame, encoding='16UC1')
                ros_depth.header.stamp = current_time
                ros_depth.header.frame_id = frame_id
                self.depth_publisher.publish(ros_depth)

                self.frame_count += 1

                if self.frame_count % 100 == 0:
                    self.get_logger().info(
                        f'Published {self.frame_count} frames '
                        f'(RGB: {rgb_frame.shape}, Depth: {depth_frame.shape})'
                    )
            else:
                self.get_logger().warn('Failed to read RGB-D frames from RealSense camera')

        except Exception as e:
            self.get_logger().error(f'Image capture or conversion error: {str(e)}')

    def destroy_node(self):
        self.get_logger().info(f'Shutting down Camera Node, published {self.frame_count} frames total')
        if hasattr(self, 'realsense_camera'):
            self.realsense_camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if rclpy.ok():
            if node:
                node.get_logger().error(f'Node runtime error: {str(e)}')
            else:
                print(f'Node initialization error: {str(e)}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
