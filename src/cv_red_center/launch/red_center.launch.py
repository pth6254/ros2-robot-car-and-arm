import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('cv_red_center'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='cv_red_center',
            executable='red_center_publisher',
            name='red_center_publisher',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='cv_red_center',
            executable='red_center_subscriber',
            name='red_center_subscriber',
            output='screen',
        ),
    ])
