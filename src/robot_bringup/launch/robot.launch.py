#!/usr/bin/env python3
"""Unified hardware bring-up.

  mode:=inference   cameras + dexterous hands (model-driven) + arms
  mode:=teleop      cameras + dexterous hands (absolute glove mapping) + data gloves
                    (arms + trackers are forked on demand by the collect node's pedal 2)
  mode:=replay      dexterous hands (model-driven) + arms, no cameras (actuators for replay_rrd)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def package_launch(package_name, launch_file):
    package_share = get_package_share_directory(package_name)
    return PythonLaunchDescriptionSource(
        os.path.join(package_share, 'launch', launch_file)
    )


def generate_launch_description():
    camera_share = get_package_share_directory('camera')
    default_camera_params = os.path.join(camera_share, 'config', 'cameras.yaml')

    mode = LaunchConfiguration('mode')
    params_file = LaunchConfiguration('params_file')
    arm_state = LaunchConfiguration('arm_state')

    is_teleop = IfCondition(PythonExpression(["'", mode, "' == 'teleop'"]))
    is_model = IfCondition(PythonExpression(["'", mode, "' in ['inference', 'replay']"]))
    has_camera = IfCondition(PythonExpression(["'", mode, "' != 'replay'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='inference',
            description="'inference' (cameras + arms + hands), 'teleop' (cameras + gloves + hands), or 'replay' (arms + hands, no cameras).",
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_camera_params,
            description='Path to the camera ROS2 parameter file.',
        ),
        DeclareLaunchArgument(
            'arm_state',
            default_value='ruckig',
            description="Arm state source: 'api_gt' or 'ruckig'.",
        ),

        # Cameras — inference + teleop (not replay)
        IncludeLaunchDescription(
            package_launch('camera', 'camera_node.launch.py'),
            launch_arguments={'params_file': params_file}.items(),
            condition=has_camera,
        ),

        # Inference / replay: model-driven dexterous hands + arms
        IncludeLaunchDescription(
            package_launch('hand', 'dual_hands.launch.py'),
            condition=is_model,
        ),
        IncludeLaunchDescription(
            package_launch('arm', 'dual_arms.launch.py'),
            launch_arguments={'arm_state': arm_state}.items(),
            condition=is_model,
        ),

        # Teleop: dexterous hands in absolute glove mapping + data gloves (arms forked by collect)
        IncludeLaunchDescription(
            package_launch('hand', 'dual_hands.launch.py'),
            launch_arguments={'glove_absolute': 'true'}.items(),
            condition=is_teleop,
        ),
        IncludeLaunchDescription(
            package_launch('glove', 'dual_glove.launch.py'),
            condition=is_teleop,
        ),
    ])
