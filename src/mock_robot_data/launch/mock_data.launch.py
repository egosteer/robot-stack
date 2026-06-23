from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share = get_package_share_directory('mock_robot_data')
    default_params_file = os.path.join(package_share, 'config', 'mock_robot_data.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the mock_robot_data ROS2 parameter file.',
        ),
        Node(
            package='mock_robot_data',
            executable='mock_publisher_node',
            name='mock_robot_data',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
