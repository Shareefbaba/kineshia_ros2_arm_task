#!/usr/bin/env python3
"""Gazebo controller with joint-state feedback."""
import time
import math
import threading
from collections import deque
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
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
GROUND_TOLERANCE = 0.16
JOINT_LIMIT_EPSILON = 1e-06
REAL_POSE_MARGIN_LOG_PERIOD_SEC = 3.0
MOVE_TIMEOUT_SECONDS = 20.0
FEEDBACK_TIMEOUT_SECONDS = 1.0

class ControllerNode(LifecycleNode):

    def __init__(self):
        super().__init__('controller_node')
        self.arm = PlanarArm(LINK_LENGTHS)
        self.current_joint_angles = [0.0, 0.0, 0.0]
        self.real_joint_angles = None
        self.last_joint_state_time = None
        self._action_lock = threading.Lock()
        self.trajectory = None
        self.trajectory_index = 0
        self.goal_joint_angles = None
        self.last_commanded_target = None
        self.declare_parameter('publish_rate_hz', 50.0)
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.declare_parameter('trajectory_duration', 2.0)
        self.trajectory_duration = self.get_parameter('trajectory_duration').get_parameter_value().double_value
        self.declare_parameter('control_mode', 'position')
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value
        self.joint_state_publisher = None
        self.joint1_cmd_publisher = None
        self.joint2_cmd_publisher = None
        self.joint3_cmd_publisher = None
        self.target_pose_subscriber = None
        self.joint_state_subscriber = None
        self.timer = None
        self.jitter_report_timer = None
        self.pick_place_action_server = None
        self.pick_place_callback_group = ReentrantCallbackGroup()
        self._is_active = False
        self.last_tick_time = None
        self.tick_gaps = deque(maxlen=200)
        self.get_logger().info(f'controller_node constructed (control_mode={self.control_mode!r}). Waiting to be configured.')

    def on_configure(self, state):
        self.joint1_cmd_publisher = self.create_lifecycle_publisher(Float64, '/joint1_cmd', 10)
        self.joint2_cmd_publisher = self.create_lifecycle_publisher(Float64, '/joint2_cmd', 10)
        self.joint3_cmd_publisher = self.create_lifecycle_publisher(Float64, '/joint3_cmd', 10)
        self.target_pose_subscriber = self.create_subscription(PointStamped, '/target_pose', self.controller_callback, 10)
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
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
        if self.joint_state_subscriber is not None:
            self.destroy_subscription(self.joint_state_subscriber)
            self.joint_state_subscriber = None
        for pub_attr in ['joint1_cmd_publisher', 'joint2_cmd_publisher', 'joint3_cmd_publisher']:
            pub = getattr(self, pub_attr)
            if pub is not None:
                self.destroy_lifecycle_publisher(pub)
                setattr(self, pub_attr, None)
        if self.pick_place_action_server is not None:
            self.pick_place_action_server.destroy()
            self.pick_place_action_server = None
        self.trajectory = None
        self.trajectory_index = 0
        self.goal_joint_angles = None
        self.last_commanded_target = None
        self.real_joint_angles = None
        self.last_joint_state_time = None
        self.tick_gaps.clear()
        self.get_logger().info('on_cleanup: all resources released.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('on_shutdown: controller_node shutting down.')
        return TransitionCallbackReturn.SUCCESS

    def move_to(self, x, y):
        if not self.feedback_is_fresh():
            self.get_logger().warn('Rejecting move: no recent /joint_states feedback.')
            return False
        start_angles = list(self.real_joint_angles)
        target_xy = tuple(self.arm.reachable_target((x, y)))
        goal_joint_angles = self.arm.inverse_kinematics(target_xy, initial_guess=start_angles)
        if not (self.arm.within_joint_limits(goal_joint_angles) and self.arm.arm_above_base(goal_joint_angles)):
            self.get_logger().error(f'No safe IK solution for target ({x}, {y}) -- ignoring move request.')
            return False
        self.last_commanded_target = target_xy
        if self.control_mode == 'velocity':
            self.current_joint_angles = start_angles
            self.goal_joint_angles = goal_joint_angles
        else:
            candidate = self.build_trajectory(start_angles, goal_joint_angles, self.trajectory_duration)
            if any((not (self._within_real_joint_limits(q, JOINT_LIMIT_EPSILON) and self.arm.arm_above_base(q, tolerance=GROUND_TOLERANCE)) for q in candidate)):
                self.get_logger().warn('Rejecting move: unsafe intermediate trajectory pose.')
                return False
            self.current_joint_angles = start_angles
            self.trajectory = candidate
            self.trajectory_index = 0
        return True

    def feedback_is_fresh(self):
        return self.real_joint_angles is not None and self.last_joint_state_time is not None and (time.monotonic() - self.last_joint_state_time < FEEDBACK_TIMEOUT_SECONDS)

    def stop_at_measured_pose(self):
        self.trajectory = None
        self.goal_joint_angles = None
        if self.feedback_is_fresh():
            self.current_joint_angles = list(self.real_joint_angles)

    def distance_to(self, x, y):
        joint_angles = self.real_joint_angles if self.real_joint_angles is not None else self.current_joint_angles
        end_effector = self.arm.forward_kinematics(joint_angles)[-1]
        return math.sqrt((end_effector[0] - x) ** 2 + (end_effector[1] - y) ** 2)

    def _within_real_joint_limits(self, q, epsilon):
        for i in range(self.arm.n_joints):
            lower = self.arm.joint_limits[i][0]
            upper = self.arm.joint_limits[i][1]
            if q[i] < lower - epsilon or q[i] > upper + epsilon:
                return False
        return True

    def joint_state_callback(self, msg):
        try:
            order = {name: i for i, name in enumerate(msg.name)}
            self.real_joint_angles = [msg.position[order['joint1']], msg.position[order['joint2']], msg.position[order['joint3']]]
            if all((math.isfinite(q) for q in self.real_joint_angles)):
                self.last_joint_state_time = time.monotonic()
            else:
                self.real_joint_angles = None
        except (KeyError, IndexError):
            self.get_logger().warn('Received /joint_states without joint1/2/3 -- ignoring.')

    def controller_callback(self, msg):
        if not self._is_active:
            self.get_logger().warn('Ignoring target — controller is not active.')
            return
        if self._action_lock.locked():
            self.get_logger().warn('Ignoring target while pick-and-place is running.')
            return
        self.move_to(msg.point.x, msg.point.y)

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

    def _wait_for_target(self, goal_handle, target, label):
        deadline = time.monotonic() + MOVE_TIMEOUT_SECONDS
        while self.distance_to(*target) > 0.1:
            if goal_handle.is_cancel_requested:
                self.stop_at_measured_pose()
                goal_handle.canceled()
                return 'Pick-and-place canceled.'
            if not self._is_active or not self.feedback_is_fresh():
                self.stop_at_measured_pose()
                goal_handle.abort()
                return f'Controller inactive or joint feedback lost while moving to {label}.'
            if time.monotonic() >= deadline:
                self.stop_at_measured_pose()
                goal_handle.abort()
                return f'Timed out while moving to {label}.'
            time.sleep(0.1)
        return None

    def _run_pick_place(self, goal_handle):
        pick_x = goal_handle.request.pick_x
        pick_y = goal_handle.request.pick_y
        place_x = goal_handle.request.place_x
        place_y = goal_handle.request.place_y
        feedback_msg = PickPlace.Feedback()
        if not self.move_to(pick_x, pick_y):
            goal_handle.abort()
            result = PickPlace.Result()
            result.success = False
            result.message = f'No safe IK solution for pick location ({pick_x}, {pick_y}).'
            return result
        pick_target = self.last_commanded_target
        feedback_msg.status = 'moving_to_pick'
        goal_handle.publish_feedback(feedback_msg)
        failure = self._wait_for_target(goal_handle, pick_target, 'pick')
        if failure:
            result = PickPlace.Result()
            result.success = False
            result.message = failure
            return result
        feedback_msg.status = 'picking'
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(PICK_PLACE_PAUSE_SECONDS)
        if not self.move_to(place_x, place_y):
            goal_handle.abort()
            result = PickPlace.Result()
            result.success = False
            result.message = f'No safe IK solution for place location ({place_x}, {place_y}).'
            return result
        place_target = self.last_commanded_target
        feedback_msg.status = 'moving_to_place'
        goal_handle.publish_feedback(feedback_msg)
        failure = self._wait_for_target(goal_handle, place_target, 'place')
        if failure:
            result = PickPlace.Result()
            result.success = False
            result.message = failure
            return result
        feedback_msg.status = 'placing'
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
        msg1 = Float64()
        msg1.data = q[0]
        self.joint1_cmd_publisher.publish(msg1)
        msg2 = Float64()
        msg2.data = q[1]
        self.joint2_cmd_publisher.publish(msg2)
        msg3 = Float64()
        msg3.data = q[2]
        self.joint3_cmd_publisher.publish(msg3)

    def timer_callback(self):
        if self.real_joint_angles is not None and (not self.feedback_is_fresh()):
            self.get_logger().warn('Joint feedback is stale; stopping commands.', throttle_duration_sec=5.0)
            self.trajectory = None
            self.goal_joint_angles = None
            return
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
        if self.real_joint_angles is None:
            self.get_logger().warn('No /joint_states feedback yet -- publishing without a real safety check.', throttle_duration_sec=5.0)
            gate_angles = self.current_joint_angles
            ground_tolerance = 1e-06
            limits_ok = self.arm.within_joint_limits(gate_angles)
        else:
            gate_angles = self.real_joint_angles
            ground_tolerance = GROUND_TOLERANCE
            limits_ok = self._within_real_joint_limits(gate_angles, JOINT_LIMIT_EPSILON)
        above_base_ok = self.arm.arm_above_base(gate_angles, tolerance=ground_tolerance)
        min_y = min((y for _, y in self.arm.forward_kinematics(gate_angles)))
        if limits_ok and above_base_ok:
            self.send_command(self.current_joint_angles)
            self.get_logger().info(f'Real-pose ground margin: min_y={min_y:.4f} (ground_limit={-ground_tolerance:.4f}, margin={min_y + ground_tolerance:.4f})', throttle_duration_sec=REAL_POSE_MARGIN_LOG_PERIOD_SEC)
        else:
            real_ee = self.arm.forward_kinematics(gate_angles)[-1]
            self.get_logger().warn(f'Unsafe REAL pose detected -- not publishing. real_joint_angles={gate_angles}, real_ee=({real_ee[0]:.4f}, {real_ee[1]:.4f}), min_y={min_y:.4f} (ground_limit={-ground_tolerance:.4f}), within_joint_limits={limits_ok}, arm_above_base={above_base_ok}')

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