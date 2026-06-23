#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Default false: keep the relative mapping for inference/HITL; teleop (robot --teleop) passes true for absolute mapping.
    glove_absolute = LaunchConfiguration('glove_absolute')
    glove_absolute_arg = DeclareLaunchArgument(
        'glove_absolute',
        default_value='false',
        description='Hand follows glove with absolute mapping (teleop). false = relative (inference/HITL).',
    )

    left_hand_ik_node = Node(
        package='hand',
        executable='hand_ik_node.py',
        name='left_hand_ik_node',
        output='screen',
        parameters=[{
            'hand_side': 'left',
            'frequency': 80.0,
            'glove_absolute': ParameterValue(glove_absolute, value_type=bool),
        }]
    )

    right_hand_ik_node = Node(
        package='hand',
        executable='hand_ik_node.py',
        name='right_hand_ik_node',
        output='screen',
        parameters=[{
            'hand_side': 'right',
            'frequency': 80.0,
            'glove_absolute': ParameterValue(glove_absolute, value_type=bool),
        }]
    )

    left_hand_control_node = Node(
        package='hand',
        executable='hand_control_node.py',
        name='left_hand_control_node',
        output='screen',
        parameters=[{
            'hand_side': 'left',
            'frequency': 80.0,
            'serial_port': '/dev/hand_left',
            'baudrate': 460800,
            'enable_interpolation': True,
        }]
    )

    right_hand_control_node = Node(
        package='hand',
        executable='hand_control_node.py',
        name='right_hand_control_node',
        output='screen',
        parameters=[{
            'hand_side': 'right',
            'frequency': 80.0,
            'serial_port': '/dev/hand_right',
            'baudrate': 460800,
            'enable_interpolation': True,
        }]
    )

    left_hand_fk_node = Node(
        package='hand',
        executable='hand_fk_node.py',
        name='left_hand_fk_node',
        output='screen',
        parameters=[{
            'hand_side': 'left',
            'frequency': 80.0,
        }]
    )

    right_hand_fk_node = Node(
        package='hand',
        executable='hand_fk_node.py',
        name='right_hand_fk_node',
        output='screen',
        parameters=[{
            'hand_side': 'right',
            'frequency': 80.0,
        }]
    )

    return LaunchDescription([
        glove_absolute_arg,
        left_hand_ik_node,
        right_hand_ik_node,
        left_hand_control_node,
        right_hand_control_node,
        left_hand_fk_node,
        right_hand_fk_node,
    ])
