#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _src(package_name, launch_file):
    share = get_package_share_directory(package_name)
    return PythonLaunchDescriptionSource(os.path.join(share, 'launch', launch_file))


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        config = {}

    task_name = str(config.get('task_name', ''))
    audio_language = str(config.get('audio_language', 'English'))
    home_settle_sec = LaunchConfiguration('home_settle_sec')

    return [
        IncludeLaunchDescription(
            _src('record', 'record.launch.py'),
            launch_arguments={'task_name': task_name}.items(),
        ),
        Node(
            package='teleop_collection',
            executable='teleop_collection_node',
            name='teleop_collection_node',
            output='screen',
            parameters=[{
                'home_settle_sec': ParameterValue(home_settle_sec, value_type=float),
                'audio_language': audio_language,
            }],
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('teleop_collection'), 'config', 'teleop_collection.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to the teleop_collection config (task_name, audio_language).'),
        DeclareLaunchArgument(
            'home_settle_sec', default_value='3.0',
            description='Seconds to wait after pedal-2 fork for tracker+arm to come up and home before publishing /commander=human.'),
        OpaqueFunction(function=launch_setup),
    ])
