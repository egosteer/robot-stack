#!/usr/bin/env python3
"""
Camera Node Launch File
Launches camera nodes based on the config file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def launch_configured_cameras(context, *args, **kwargs):
    """Create one camera node for each entry in the YAML config."""
    params_file = LaunchConfiguration('params_file').perform(context)

    with open(params_file, 'r', encoding='utf-8') as file:
        camera_config = yaml.safe_load(file) or {}

    camera_nodes = []
    for node_name, _ in camera_config.items():
        camera_nodes.append(
            Node(
                package='camera',
                executable='camera_node',
                name=node_name,
                output='screen',
                parameters=[params_file],
            )
        )

    return camera_nodes


def generate_launch_description():
    """Build the launch description; starts head/chest RealSense camera nodes per config."""
    package_share = get_package_share_directory('camera')
    default_params_file = os.path.join(package_share, 'config', 'cameras.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the camera ROS2 parameter file.',
    )

    return LaunchDescription([
        params_file_arg,
        OpaqueFunction(function=launch_configured_cameras),
    ])
