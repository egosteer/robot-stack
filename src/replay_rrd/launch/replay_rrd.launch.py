from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('replay_rrd')
    default_params_file = os.path.join(package_share, 'config', 'replay_rrd.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the replay_rrd ROS2 parameter file.',
        ),
        Node(
            package='replay_rrd',
            executable='replay_rrd_node.py',
            name='replay_rrd',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
