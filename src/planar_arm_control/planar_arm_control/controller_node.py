#!/usr/bin/env python3
"""2D planar arm controller."""
import time
import math
import threading
from collections import deque
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionServer
from my_custom_interfaces.action import PickPlace
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from planar_arm_control.planar_arm import PlanarArm
LINK_LENGTHS = [3.0, 2.0, 1.5]
PICK_PLACE_PAUSE_SECONDS = 1.0
VELOCITY_GAIN = 2.0
MAX_JOINT_VELOCITY = 3.0
MOVE_TIMEOUT_SECONDS = 10.0
ARRIVAL_TOLERANCE = 0.1

class ControllerNode(LifecycleNode):

    def __init__(self):
        super().__init__('controller_node')
        self.arm = PlanarArm(LINK_LENGTHS)
        self.current_joint_angles = [0.0, 0.0, 0.0]
        self.trajectory = None
        self.trajectory_index = 0
        self.goal_joint_angles = None
        self._action_lock = threading.Lock()
        self.declare_parameter('publish_rate_hz', 50.0)
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.declare_parameter('trajectory_duration', 2.0)
        self.trajectory_duration = self.get_parameter('trajectory_duration').get_parameter_value().double_value
        self.declare_parameter('control_mode', 'position')
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value
        self.joint_state_publisher = None
        self.target_pose_subscriber = None
        self.timer = None
        self.jitter_report_timer = None
        self.pick_place_action_server = None
        self.pick_place_callback_group = ReentrantCallbackGroup()
        self._is_active = False
        self.last_tick_time = None
        self.tick_gaps = deque(maxlen=200)
        self.get_logger().info(f'controller_node constructed (control_mode={self.control_mode!r}). Waiting to be configured.')

    def on_configure(self, state):
        self.joint_state_publisher = self.create_lifecycle_publisher(JointState, '/joint_states', 10)
        self.target_pose_subscriber = self.create_subscription(PointStamped, '/target_pose', self.controller_callback, 10)
        self.pick_place_action_server = ActionServer(self, PickPlace, 'pick_place', self.execute_pick_place_callback, callback_group=self.pick_place_callback_group)
        self.get_logger().info('on_configure: publisher, subscriber, action server created.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._is_active = True
        self.last_tick_time = None
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.jitter_report_timer = self.create_timer(1.0, self.jitter_report_callback)
        self.get_logger().info('on_activate: timers started, now accepting targets.')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self._is_active = False
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        if self.jitter_report_timer is not None:
            self.jitter_report_timer.cancel()
            self.jitter_report_timer = None
        self.get_logger().info('on_deactivate: timers stopped, arm will hold its last position.')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        if self.target_pose_subscriber is not None:
            self.destroy_subscription(self.target_pose_subscriber)
            self.target_pose_subscriber = None
        if self.joint_state_publisher is not None:
            self.destroy_lifecycle_publisher(self.joint_state_publisher)
            self.joint_state_publisher = None
        if self.pick_place_action_server is not None:
            self.pick_place_action_server.destroy()
            self.pick_place_action_server = None
        self.trajectory = None
        self.trajectory_index = 0
        self.goal_joint_angles = None
        self.tick_gaps.clear()
        self.get_logger().info('on_cleanup: all resources released.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('on_shutdown: controller_node shutting down.')
        return TransitionCallbackReturn.SUCCESS

    def move_to(self, x, y):
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        if math.hypot(x, y) > sum(self.arm.link_lengths) + 1e-06:
            return False
        goal_joint_angles = self.arm.inverse_kinematics((x, y), initial_guess=self.current_joint_angles)
        if not (self.arm.within_joint_limits(goal_joint_angles) and self.arm.arm_above_base(goal_joint_angles)):
            return False
        achieved = self.arm.end_effector(goal_joint_angles)
        if math.dist(achieved, (x, y)) > ARRIVAL_TOLERANCE:
            return False
        if self.control_mode == 'velocity':
            self.goal_joint_angles = goal_joint_angles
        else:
            candidate = self.build_trajectory(self.current_joint_angles, goal_joint_angles, self.trajectory_duration)
            if not candidate or any((not (self.arm.within_joint_limits(q) and self.arm.arm_above_base(q)) for q in candidate)):
                return False
            self.trajectory = candidate
            self.trajectory_index = 0
        return True

    def distance_to(self, x, y):
        end_effector = self.arm.forward_kinematics(self.current_joint_angles)[-1]
        return math.sqrt((end_effector[0] - x) ** 2 + (end_effector[1] - y) ** 2)

    def controller_callback(self, msg):
        if not self._is_active:
            self.get_logger().warn('Ignoring target — controller is not active.')
            return
        if self._action_lock.locked():
            self.get_logger().warn('Ignoring target while pick-and-place is running.')
            return
        if not self.move_to(msg.point.x, msg.point.y):
            self.get_logger().warn('Rejecting unreachable or unsafe target.')

    def execute_pick_place_callback(self, goal_handle):
        if not self._is_active:
            goal_handle.abort()
            result = PickPlace.Result()
            result.success = False
            result.message = 'Controller is not active — activate it before requesting pick-and-place.'
            return result
        if not self._action_lock.acquire(blocking=False):
            goal_handle.abort()
            result = PickPlace.Result()
            result.success = False
            result.message = 'Another pick-and-place action is running.'
            return result
        try:
            return self._run_pick_place(goal_handle)
        finally:
            self._action_lock.release()

    def _run_pick_place(self, goal_handle):
        pick_x = goal_handle.request.pick_x
        pick_y = goal_handle.request.pick_y
        place_x = goal_handle.request.place_x
        place_y = goal_handle.request.place_y
        feedback_msg = PickPlace.Feedback()
        for label, x, y in (('pick', pick_x, pick_y), ('place', place_x, place_y)):
            if not self.move_to(x, y):
                goal_handle.abort()
                result = PickPlace.Result()
                result.success = False
                result.message = f'Unreachable or unsafe {label} target ({x}, {y}).'
                return result
            feedback_msg.status = f'moving_to_{label}'
            goal_handle.publish_feedback(feedback_msg)
            deadline = time.monotonic() + MOVE_TIMEOUT_SECONDS
            while self.distance_to(x, y) > ARRIVAL_TOLERANCE:
                if goal_handle.is_cancel_requested:
                    self.trajectory = None
                    self.goal_joint_angles = None
                    goal_handle.canceled()
                    result = PickPlace.Result()
                    result.success = False
                    result.message = 'Pick-and-place canceled.'
                    return result
                if not self._is_active or time.monotonic() >= deadline:
                    self.trajectory = None
                    self.goal_joint_angles = None
                    goal_handle.abort()
                    result = PickPlace.Result()
                    result.success = False
                    result.message = f'Timed out or controller inactive while moving to {label}.'
                    return result
                time.sleep(0.1)
            feedback_msg.status = 'picking' if label == 'pick' else 'placing'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(PICK_PLACE_PAUSE_SECONDS)
        goal_handle.succeed()
        result = PickPlace.Result()
        result.success = True
        result.message = 'Pick and place complete'
        return result

    def build_trajectory(self, start_angles, goal_angles, duration):
        num_waypoints = max(2, int(duration * self.publish_rate_hz))
        trajectory = []
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)
            progress = 6 * t ** 5 - 15 * t ** 4 + 10 * t ** 3
            waypoint = []
            for j in range(self.arm.n_joints):
                delta = goal_angles[j] - start_angles[j]
                current_angle = start_angles[j] + delta * progress
                waypoint.append(current_angle)
            trajectory.append(waypoint)
        return trajectory

    def send_command(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = q
        self.joint_state_publisher.publish(msg)

    def timer_callback(self):
        now = time.time()
        if self.last_tick_time is not None:
            self.tick_gaps.append(now - self.last_tick_time)
        self.last_tick_time = now
        if self.control_mode == 'velocity':
            if self.goal_joint_angles is not None:
                dt = 1.0 / self.publish_rate_hz
                new_angles = []
                for j in range(self.arm.n_joints):
                    error = self.goal_joint_angles[j] - self.current_joint_angles[j]
                    velocity = VELOCITY_GAIN * error
                    velocity = max(-MAX_JOINT_VELOCITY, min(MAX_JOINT_VELOCITY, velocity))
                    new_angles.append(self.current_joint_angles[j] + velocity * dt)
                self.current_joint_angles = new_angles
        elif self.trajectory is not None and self.trajectory_index < len(self.trajectory):
            self.current_joint_angles = self.trajectory[self.trajectory_index]
            self.trajectory_index += 1
        if self.arm.within_joint_limits(self.current_joint_angles) and self.arm.arm_above_base(self.current_joint_angles):
            self.send_command(self.current_joint_angles)
        else:
            self.get_logger().warn('Unsafe pose detected -- not publishing')

    def jitter_report_callback(self):
        if not self.tick_gaps:
            return
        gaps_ms = [gap * 1000 for gap in self.tick_gaps]
        avg_ms = sum(gaps_ms) / len(gaps_ms)
        target_ms = 1000.0 / self.publish_rate_hz
        self.get_logger().info(f'Timer jitter: avg={avg_ms:.2f}ms min={min(gaps_ms):.2f}ms max={max(gaps_ms):.2f}ms (target={target_ms:.2f}ms)')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()