from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def package_launch(package_name, launch_file):
    package_share = get_package_share_directory(package_name)
    return PythonLaunchDescriptionSource(
        os.path.join(package_share, 'launch', launch_file)
    )


def read_task_name_from_params(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    try:
        with open(params_file, 'r') as f:
            data = yaml.safe_load(f) or {}
        node_params = (data.get('model_interface_node', {})
                           .get('ros__parameters', {})) or {}
        return str(node_params.get('task_name', '') or '').strip()
    except (OSError, yaml.YAMLError):
        return ''


def include_record(context, *args, **kwargs):
    task_name = read_task_name_from_params(context)
    return [
        IncludeLaunchDescription(
            package_launch('record', 'record.launch.py'),
            launch_arguments={'task_name': task_name}.items(),
        )
    ]


def launch_model_interface(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file')
    human_in_the_loop = LaunchConfiguration('human_in_the_loop').perform(context).strip()
    human_in_the_loop_dash = LaunchConfiguration('human-in-the-loop').perform(context).strip()
    human_in_the_loop_override = human_in_the_loop_dash or human_in_the_loop

    parameters = [params_file]
    if human_in_the_loop_override:
        parameters.append({
            'human_in_the_loop': human_in_the_loop_override.lower() in ('1', 'true', 'yes', 'on'),
        })

    return [
        Node(
            package='model_interface',
            executable='model_interface_node',
            name='model_interface_node',
            output='screen',
            parameters=parameters,
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory('model_interface')
    default_params_file = os.path.join(package_share, 'config', 'model_interface.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the model_interface ROS2 parameter file.',
        ),
        DeclareLaunchArgument(
            'human_in_the_loop',
            default_value='',
            description='Optional override for the human_in_the_loop parameter: true or false.',
        ),
        DeclareLaunchArgument(
            'human-in-the-loop',
            default_value='',
            description='Alias for human_in_the_loop.',
        ),
        OpaqueFunction(function=include_record),
        OpaqueFunction(function=launch_model_interface),
    ])
