from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    glove_port_left = DeclareLaunchArgument(
        'glove_port_left',
        default_value='/dev/glove_left',
        description='Left glove serial port',
    )
    glove_port_right = DeclareLaunchArgument(
        'glove_port_right',
        default_value='/dev/glove_right',
        description='Right glove serial port',
    )
    publish_rate_left = DeclareLaunchArgument(
        'publish_rate_left',
        default_value='80.0',
        description='Left glove publish rate',
    )
    publish_rate_right = DeclareLaunchArgument(
        'publish_rate_right',
        default_value='80.0',
        description='Right glove publish rate',
    )
    glove_read_rate_left = DeclareLaunchArgument(
        'glove_read_rate_left',
        default_value='200.0',
        description='Left glove read rate',
    )
    glove_read_rate_right = DeclareLaunchArgument(
        'glove_read_rate_right',
        default_value='200.0',
        description='Right glove read rate',
    )
    enable_smooth_left = DeclareLaunchArgument(
        'enable_smooth_left',
        default_value='true',
        description='Enable smoothing for left glove',
    )
    enable_smooth_right = DeclareLaunchArgument(
        'enable_smooth_right',
        default_value='true',
        description='Enable smoothing for right glove',
    )

    left_node = Node(
        package='glove',
        executable='glove_node',
        name='left_glove_node',
        output='screen',
        parameters=[{
            'glove_port': LaunchConfiguration('glove_port_left'),
            'hand_name': 'left',
            'publish_rate': ParameterValue(LaunchConfiguration('publish_rate_left'), value_type=float),
            'glove_read_rate': ParameterValue(LaunchConfiguration('glove_read_rate_left'), value_type=float),
            'enable_smooth': ParameterValue(LaunchConfiguration('enable_smooth_left'), value_type=bool),
        }],
    )

    right_node = Node(
        package='glove',
        executable='glove_node',
        name='right_glove_node',
        output='screen',
        parameters=[{
            'glove_port': LaunchConfiguration('glove_port_right'),
            'hand_name': 'right',
            'publish_rate': ParameterValue(LaunchConfiguration('publish_rate_right'), value_type=float),
            'glove_read_rate': ParameterValue(LaunchConfiguration('glove_read_rate_right'), value_type=float),
            'enable_smooth': ParameterValue(LaunchConfiguration('enable_smooth_right'), value_type=bool),
        }],
    )

    return LaunchDescription([
        glove_port_left,
        glove_port_right,
        publish_rate_left,
        publish_rate_right,
        glove_read_rate_left,
        glove_read_rate_right,
        enable_smooth_left,
        enable_smooth_right,
        left_node,
        right_node,
    ])
