from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    params_file = LaunchConfiguration('params')
    declare_params = DeclareLaunchArgument(
        'params',
        default_value=['', ''],  # 実際は -p/--params-file で与える想定
        description='YAML parameters file for gnss node'
    )

    node = LifecycleNode(
        package='gnss',
        executable='gnss_node',
        name='gnss',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        declare_params,
        node
    ])
