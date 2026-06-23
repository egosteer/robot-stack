from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def launch_record(context, *args, **kwargs):
    task_name = LaunchConfiguration('task_name').perform(context).strip()

    base = os.path.abspath('recordings')
    data_folder = os.path.join(base, task_name) if task_name else base

    return [
        Node(
            package='record',
            executable='record_node',
            name='record_node',
            output='screen',
            parameters=[{
                'data_folder': data_folder,
                'enable_compression': ParameterValue(LaunchConfiguration('enable_compression'), value_type=bool),
                'compression_quality': ParameterValue(LaunchConfiguration('compression_quality'), value_type=int),
                'queue_size': ParameterValue(LaunchConfiguration('queue_size'), value_type=int),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'task_name',
            default_value='',
            description='Optional task name. Episodes go to recordings/<task_name>/; empty → recordings/.',
        ),
        DeclareLaunchArgument(
            'enable_compression',
            default_value='true',
            description='Whether RGB images are saved as JPEG encoded images.',
        ),
        DeclareLaunchArgument(
            'compression_quality',
            default_value='80',
            description='JPEG compression quality for RGB images.',
        ),
        DeclareLaunchArgument(
            'queue_size',
            default_value='1000',
            description='Recorder subscription queue size.',
        ),
        OpaqueFunction(function=launch_record),
    ])
