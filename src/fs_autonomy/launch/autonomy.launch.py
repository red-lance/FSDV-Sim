"""Full autonomy stack: foxglove_bridge + cone_viz + all mission controllers.

The bridge snapshots the sim's topics/services at startup, so it is held back
until /eufs_sim2/get_map is advertised -- launch order relative to
`eufs sim run` no longer matters. If the sim RESTARTS while this is running,
the bridge's snapshot is stale again: restart this launch and reconnect
Foxglove.

Every mission controller runs all the time; each one gates on its own
AMIState in /sim/ros_can/state_str, so whichever mission is selected in
Foxglove is the one that engages once the state machine reaches DRIVING.

The EKF estimation stack (estimation.launch.py) is included when
robot_localization is installed -- it publishes /odometry/filtered but the
controllers keep driving on ground truth /odom unless their config sets
odom_topic. Disable it with estimation:=false.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    get_package_share_directory("robot_localization")
    HAVE_EKF = True
except Exception:
    HAVE_EKF = False


def generate_launch_description():
    share = get_package_share_directory("fs_autonomy")
    accel_params = os.path.join(share, "config", "accel_params.yaml")
    skidpad_params = os.path.join(share, "config", "skidpad_params.yaml")
    trackdrive_params = os.path.join(share, "config", "trackdrive_params.yaml")
    autocross_params = os.path.join(share, "config", "autocross_params.yaml")
    wait_for_sim = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "echo 'waiting for the sim (/eufs_sim2/get_map)...'; "
            "until ros2 service type /eufs_sim2/get_map >/dev/null 2>&1; "
            "do sleep 1; done; "
            "echo 'sim is up -- starting foxglove_bridge.'",
        ],
        name="wait_for_sim",
        output="screen",
    )
    bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        )
    )
    actions = [
        DeclareLaunchArgument(
            "estimation", default_value="true",
            description="run the EKF estimation stack (needs robot_localization)"),
        wait_for_sim,
        RegisterEventHandler(
            OnProcessExit(target_action=wait_for_sim, on_exit=[bridge])
        ),
    ]
    if HAVE_EKF:
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(share, "launch", "estimation.launch.py")),
            condition=IfCondition(LaunchConfiguration("estimation")),
        ))
    actions += [
        Node(
            package="fs_autonomy",
            executable="cone_viz",
            name="cone_viz",
            output="screen",
        ),
        Node(
            package="fs_autonomy",
            executable="accel_driver",
            name="accel_driver",
            parameters=[accel_params],
            output="screen",
        ),
        Node(
            package="fs_autonomy",
            executable="skidpad_driver",
            name="skidpad_driver",
            parameters=[skidpad_params],
            output="screen",
        ),
        Node(
            package="fs_autonomy",
            executable="trackdrive_driver",
            name="trackdrive_driver",
            parameters=[trackdrive_params],
            output="screen",
        ),
        # autocross = trackdrive with laps: 1, gated on its own mission
        Node(
            package="fs_autonomy",
            executable="trackdrive_driver",
            name="autocross_driver",
            parameters=[autocross_params],
            output="screen",
        ),
    ]
    return LaunchDescription(actions)
