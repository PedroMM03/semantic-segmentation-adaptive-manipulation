from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('xarm6_controller'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='xarm6_controller',
            executable='xarm6_controller_node',
            name='xarm6_controller_node',
            output='screen',
            parameters=[config_file]
        )
    ])
