#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_state = LaunchConfiguration('arm_state')
    enable_viewer = LaunchConfiguration('enable_viewer')

    arm_ik_node = Node(
        package='arm',
        executable='arm_ik_node.py',
        name='arm_ik_node',
        output='screen',
        parameters=[{
            'frequency': 100.0,
            'enable_viewer': ParameterValue(enable_viewer, value_type=bool),
        }]
    )

    left_arm_control_node = Node(
        package='arm',
        executable='arm_control_node.py',
        name='left_arm_control_node',
        output='screen',
        parameters=[{
            'arm_side': 'left',
            'frequency': 100.0,
            'arm_state': arm_state,
        }]
    )

    right_arm_control_node = Node(
        package='arm',
        executable='arm_control_node.py',
        name='right_arm_control_node',
        output='screen',
        parameters=[{
            'arm_side': 'right',
            'frequency': 100.0,
            'arm_state': arm_state,
        }]
    )

    left_arm_fk_node = Node(
        package='arm',
        executable='arm_fk_node.py',
        name='left_arm_fk_node',
        output='screen',
        parameters=[{
            'arm_side': 'left',
        }],
        condition=IfCondition(PythonExpression(["'", arm_state, "' == 'ruckig'"]))
    )

    right_arm_fk_node = Node(
        package='arm',
        executable='arm_fk_node.py',
        name='right_arm_fk_node',
        output='screen',
        parameters=[{
            'arm_side': 'right',
        }],
        condition=IfCondition(PythonExpression(["'", arm_state, "' == 'ruckig'"]))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_state',
            default_value='ruckig',
            description="Arm state source: 'api_gt' uses RM API wrist pose, 'ruckig' uses interpolated joints + FK node.",
        ),
        DeclareLaunchArgument(
            'enable_viewer',
            default_value='true',
            description='Show the MuJoCo IK viewer. Set false when forked from teleop to avoid a popup on every press of 2.',
        ),
        arm_ik_node,
        left_arm_control_node,
        right_arm_control_node,
        left_arm_fk_node,
        right_arm_fk_node,
    ])
