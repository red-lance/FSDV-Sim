import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("fs_autonomy"), "config", "accel_params.yaml"
    )
    return LaunchDescription([
        Node(
            package="fs_autonomy",
            executable="accel_driver",
            name="accel_driver",
            parameters=[params],
            output="screen",
        ),
    ])
