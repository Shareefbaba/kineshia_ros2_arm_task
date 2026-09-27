#!/usr/bin/env python3
"""
test_controller_node.py

Unit tests for the controller_node stretch goal. This file did not exist in
the submitted package (despite being listed as a completed stretch goal) --
added while verifying the take-home end to end. Scope is deliberately kept to
fast, launch-free unit tests of the pure logic (trajectory generation, mode
dispatch, kinematics); the action server, lifecycle transitions, and Gazebo
integration were verified manually against a live system instead, since they
need a running rclpy graph / simulator that a plain pytest run doesn't have.
"""

import math

import pytest
import rclpy

from planar_arm_control.controller_node import ControllerNode
from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = ControllerNode()
    yield n
    n.destroy_node()


def test_build_trajectory_endpoints_and_length(node):
    start = [0.0, 0.0, 0.0]
    goal = [0.5, -0.3, 0.2]
    duration = 2.0
    trajectory = node.build_trajectory(start, goal, duration)

    assert len(trajectory) == int(duration * node.publish_rate_hz)
    assert trajectory[0] == pytest.approx(start, abs=1e-9)
    assert trajectory[-1] == pytest.approx(goal, abs=1e-9)


def test_build_trajectory_is_monotonic_per_joint(node):
    start = [0.0, 0.0, 0.0]
    goal = [1.0, -1.0, 0.5]
    trajectory = node.build_trajectory(start, goal, node.trajectory_duration)

    for j in range(node.arm.n_joints):
        values = [wp[j] for wp in trajectory]
        diffs = [b - a for a, b in zip(values, values[1:])]
        sign = 1 if goal[j] >= start[j] else -1
        assert all(sign * d >= -1e-9 for d in diffs), (
            f"joint {j} trajectory is not monotonic toward the goal"
        )


def test_move_to_position_mode_builds_trajectory_to_ik_solution(node):
    node.control_mode = "position"
    target_x, target_y = 4.0, 1.0

    node.move_to(target_x, target_y)

    assert node.trajectory is not None
    assert node.trajectory_index == 0
    expected_goal = node.arm.inverse_kinematics((target_x, target_y), initial_guess=node.current_joint_angles)
    assert node.trajectory[-1] == pytest.approx(expected_goal, abs=1e-6)


def test_move_to_velocity_mode_sets_goal_without_trajectory(node):
    node.control_mode = "velocity"
    target_x, target_y = 5.0, 0.5

    node.move_to(target_x, target_y)

    assert node.goal_joint_angles is not None
    assert node.trajectory is None
    expected_goal = node.arm.inverse_kinematics((target_x, target_y), initial_guess=node.current_joint_angles)
    assert list(node.goal_joint_angles) == pytest.approx(expected_goal, abs=1e-6)


def test_move_to_rejects_target_with_no_safe_ik_solution(node):
    # (1.0, 1.0) has no analytical IK solution within this arm's joint limits;
    # the Jacobian fallback in planar_arm.py returns a pose that violates
    # joint1/joint2 limits. move_to() must refuse it rather than silently
    # committing to a trajectory the safety gate will block command-by-command
    # (which previously let PickPlace report false success -- see controller_node.py).
    node.control_mode = "position"
    previous_trajectory = node.trajectory

    result = node.move_to(1.0, 1.0)

    assert result is False
    assert node.trajectory is previous_trajectory


def test_distance_to_matches_forward_kinematics(node):
    node.current_joint_angles = [0.3, -0.2, 0.1]
    ee_x, ee_y = node.arm.forward_kinematics(node.current_joint_angles)[-1]

    reported = node.distance_to(ee_x, ee_y)

    assert reported == pytest.approx(0.0, abs=1e-9)

    far_x, far_y = ee_x + 1.0, ee_y + 1.0
    expected = math.sqrt((ee_x - far_x) ** 2 + (ee_y - far_y) ** 2)
    assert node.distance_to(far_x, far_y) == pytest.approx(expected, abs=1e-9)


def test_default_control_mode_is_position(node):
    assert node.control_mode == "position"


@pytest.mark.parametrize("target", [(4.0, 1.0), (3.0, 2.0), (5.0, 0.5)])
def test_planar_arm_ik_round_trips_to_target(target):
    arm = PlanarArm(LINK_LENGTHS)
    q = arm.inverse_kinematics(target)

    assert arm.within_joint_limits(q)
    assert arm.arm_above_base(q)
    achieved = arm.end_effector(q)
    assert achieved == pytest.approx(target, abs=1e-3)
