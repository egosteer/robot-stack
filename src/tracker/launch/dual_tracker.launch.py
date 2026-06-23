from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        config = {}

    common = {
        'publish_rate': float(config.get('publish_rate', 100.0)),
        'base_frame': str(config.get('base_frame', 'map')),
    }

    return [
        Node(
            package='tracker',
            executable='tracker_node',
            name='robot_vive_tracker_left',
            output='screen',
            parameters=[{
                **common,
                'serial_number': str(config.get('left_serial_number', '')),
                'hand_name': 'left',
            }],
        ),
        Node(
            package='tracker',
            executable='tracker_node',
            name='robot_vive_tracker_right',
            output='screen',
            parameters=[{
                **common,
                'serial_number': str(config.get('right_serial_number', '')),
                'hand_name': 'right',
            }],
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('tracker'), 'config', 'dual_tracker.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to the tracker config (serial numbers, publish rate, base frame).'),
        OpaqueFunction(function=launch_setup),
    ])
