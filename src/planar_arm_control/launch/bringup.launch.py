"""
bringup.launch.py

Launches the controller node and the GUI node together, and automatically
drives controller_node through configure -> activate, so no manual
`ros2 lifecycle set` commands are needed.
"""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg


def generate_launch_description():
    controller_node = LifecycleNode(
        package="planar_arm_control",
        executable="controller_node",
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
        controller_node,
        gui_node,
        activate_on_configured,
        configure_event,
    ])
