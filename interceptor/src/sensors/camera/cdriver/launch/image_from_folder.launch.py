from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cdriver',
            executable='image_offline_node',    # entry_point from setup.py
            name='image_from_folder_node',      # node name
            output='screen',
            parameters=[
                os.path.join(
                    os.path.dirname(__file__),  # path to launch/ folder
                    '../config/image_from_folder_node.yaml'
                )
            ]
        )
    ])