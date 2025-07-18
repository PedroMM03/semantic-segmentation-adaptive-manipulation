from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('zed_vision'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='zed_vision',
            executable='yolo_zed_vision_node',
            name='yolo_zed_vision_node',
            output='screen',
            parameters=[config_file]
        )
    ])
