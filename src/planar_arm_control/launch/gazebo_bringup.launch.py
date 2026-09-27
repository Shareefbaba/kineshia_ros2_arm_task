"""
gazebo_bringup.launch.py

Launches the Gazebo world, spawns the planar arm, and starts the ROS<->Gazebo bridge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("planar_arm_control")
    world_path = os.path.join(pkg_share, "world", "planar_arm_world.sdf")
    urdf_path = os.path.join(pkg_share, "urdf", "planar_arm.sdf")
    bridge_config_path = os.path.join(pkg_share, "config", "planar_arm_bridge.yaml")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    spawn_arm = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "planar_arm", "-file", urdf_path],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config_path}"],
        output="screen",
    )

    return LaunchDescription([
        gz_sim,
        spawn_arm,
        bridge,
    ])