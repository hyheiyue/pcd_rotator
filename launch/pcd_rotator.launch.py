import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('pcd_rotator'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='pcd_rotator',
            executable='pcd_rotator',
            name='pcd_rotator',
            parameters=[config_path],
            output='screen'
        )
    ])