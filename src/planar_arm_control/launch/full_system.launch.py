"""
full_system.launch.py

Launches everything, and automatically drives controller_node through
configure -> activate, so no manual `ros2 lifecycle set` commands are needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg


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

    controller_node = LifecycleNode(
        package="planar_arm_control",
        # Gazebo mode needs controller_node_gazebo (real /joint_states
        # feedback + safety gate + clamp fix) -- the plain "controller_node"
        # executable maps to controller_node.py, which has no real physics
        # to diverge from and is meant for bringup.launch.py instead.
        executable="controller_node_gazebo",
        name="controller_node",
        namespace="",
        output="screen",
        parameters=[{"publish_rate_hz": 50.0, "trajectory_duration": 2.0, "control_mode": "position"}],
    )

    gui_node = Node(
        package="planar_arm_control",
        executable="gui_node",
        name="gui_node",
        output="screen",
    )

    # --- Automatic lifecycle transitions: configure, then activate ---
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(controller_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(controller_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=controller_node,
            goal_state='inactive',
            entities=[activate_event],
        )
    )

    return LaunchDescription([
        gz_sim,
        spawn_arm,
        bridge,
        controller_node,
        gui_node,
        activate_on_configured,
        configure_event,
    ])