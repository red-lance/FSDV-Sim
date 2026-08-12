"""State-estimation stack: wheel-odometry adapter + robot_localization EKF.

Runs alongside the sim (or the real car's sensor feeds):

    ros2 launch fs_autonomy estimation.launch.py

Publishes /odometry/filtered -- dead-reckoned pose/velocity fused from the
IMU and wheel speeds. Point a controller at it with odom_topic to run on
estimated state instead of ground truth.

Requires: ros-humble-robot-localization (apt).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_params = os.path.join(
        get_package_share_directory("fs_autonomy"), "config", "ekf_params.yaml")

    return LaunchDescription([
        # the sim's URDF has no imu link; the sim IMU is body-mounted, so an
        # identity transform ties its frame into the tree
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf",
            arguments=["--frame-id", "base_footprint", "--child-frame-id", "imu"],
        ),
        Node(
            package="fs_autonomy",
            executable="wheel_odometry",
            name="wheel_odometry",
        ),
        Node(
            package="fs_autonomy",
            executable="imu_frontend",
            name="imu_frontend",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[ekf_params],
        ),
    ])
