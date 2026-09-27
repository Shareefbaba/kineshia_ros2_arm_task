#!/usr/bin/env python3
"""
gui_node.py  —  STARTER STUB. This is YOUR work to implement.

Goal: a PyQt5 + PyQtGraph node that is a live client of the controller.
"""

import sys
import time
import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

from rclpy.action import ActionClient
from my_custom_interfaces.action import PickPlace

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]


class GuiNode(Node):
    def __init__(self):
        super().__init__("gui_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        self.current_joint_angles = [0.0, 0.0, 0.0]
        self.needs_redraw = False
        self.last_target = None

        self.start_time = time.time()
        self.angle_history = deque(maxlen=200)
        self.error_history = deque(maxlen=200)  # (time, distance-to-target) for the live error plot

        self.target_pose_publisher = self.create_publisher(PointStamped, "/target_pose", 10)
        self.joint_state_subscriber = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        self.pick_place_action_client = ActionClient(self, PickPlace, 'pick_place')
        self.pick_place_status = None

        self.get_logger().info("gui_node started (stub — implement me).")

    def joint_state_callback(self, msg):
        self.current_joint_angles = msg.position
        self.needs_redraw = True
        elapsed = time.time() - self.start_time
        self.angle_history.append((elapsed, msg.position))


class RobotWindow(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)

        self.plot_widget = pg.PlotWidget()
        self.layout.addWidget(self.plot_widget)
        self.arm_line = self.plot_widget.plot([], [])

        self.angle_plot_widget = pg.PlotWidget()
        self.layout.addWidget(self.angle_plot_widget)
        self.joint1_line = self.angle_plot_widget.plot([], [], pen='r')
        self.joint2_line = self.angle_plot_widget.plot([], [], pen='g')
        self.joint3_line = self.angle_plot_widget.plot([], [], pen='b')

        self.error_plot_widget = pg.PlotWidget()
        self.layout.addWidget(self.error_plot_widget)
        self.error_line = self.error_plot_widget.plot([], [], pen='m')

        self.x_input = QtWidgets.QLineEdit()
        self.y_input = QtWidgets.QLineEdit()
        self.send_button = QtWidgets.QPushButton("Send Target")
        self.target_row = QtWidgets.QHBoxLayout()
        self.target_row.addWidget(self.x_input)
        self.target_row.addWidget(self.y_input)
        self.target_row.addWidget(self.send_button)

        self.pick_place_x_input = QtWidgets.QLineEdit()
        self.pick_place_y_input = QtWidgets.QLineEdit()
        self.send_pick_place_button = QtWidgets.QPushButton("Start Pick & Place (Action)")
        self.pick_place_row = QtWidgets.QHBoxLayout()
        self.pick_place_row.addWidget(self.pick_place_x_input)
        self.pick_place_row.addWidget(self.pick_place_y_input)
        self.pick_place_row.addWidget(self.send_pick_place_button)

        self.layout.addLayout(self.target_row)
        self.layout.addLayout(self.pick_place_row)

        self.send_button.clicked.connect(self.on_send_clicked)
        self.send_pick_place_button.clicked.connect(self.on_pick_place_clicked)

        self.position_label = QtWidgets.QLabel("End-Effector: (-,-)")
        self.status_label = QtWidgets.QLabel("Status: Idle")
        self.layout.addWidget(self.position_label)
        self.layout.addWidget(self.status_label)

        self.ros_timer = QtCore.QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        self.ros_timer.start(20)

        self.draw_timer = QtCore.QTimer()
        self.draw_timer.timeout.connect(self.redraw)
        self.draw_timer.start(30)

    def redraw(self):
        if not self.node.needs_redraw:
            return
        self.node.needs_redraw = False
        points = self.node.arm.forward_kinematics(self.node.current_joint_angles)
        end_effector = points[-1]

        x_values = []
        y_values = []
        for point in points:
            x_values.append(point[0])
            y_values.append(point[1])
        self.arm_line.setData(x_values, y_values)

        time_values = []
        joint1_value = []
        joint2_value = []
        joint3_value = []
        for entry_time, entry_angle in self.node.angle_history:
            time_values.append(entry_time)
            joint1_value.append(entry_angle[0])
            joint2_value.append(entry_angle[1])
            joint3_value.append(entry_angle[2])
        self.joint1_line.setData(time_values, joint1_value)
        self.joint2_line.setData(time_values, joint2_value)
        self.joint3_line.setData(time_values, joint3_value)

        self.position_label.setText(f"End-Effector: ({end_effector[0]:.2f}, {end_effector[1]:.2f})")

        elapsed = time.time() - self.node.start_time
        if self.node.last_target is not None:
            target_x, target_y = self.node.last_target
            distance = math.sqrt((end_effector[0] - target_x) ** 2 + (end_effector[1] - target_y) ** 2)
        else:
            distance = 0.0
        self.node.error_history.append((elapsed, distance))

        error_time_values = []
        error_values = []
        for entry_time, entry_error in self.node.error_history:
            error_time_values.append(entry_time)
            error_values.append(entry_error)
        self.error_line.setData(error_time_values, error_values)

        if self.node.pick_place_status is not None:
            self.status_label.setText(f"Status: {self.node.pick_place_status}")
        elif self.node.last_target is None:
            self.status_label.setText("Status: Idle (no target sent)")
        else:
            if distance < 0.1:
                self.status_label.setText("Status: Idle")
            else:
                self.status_label.setText("Status: Moving")

    def on_send_clicked(self):
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
        except ValueError:
            self.node.get_logger().warn("Invalid target - enter numbers of x and y.")
            return
        target_msg = PointStamped()
        target_msg.point.x = x
        target_msg.point.y = y
        self.node.target_pose_publisher.publish(target_msg)
        self.node.last_target = (x, y)

    def on_pick_place_clicked(self):
        try:
            pick_x = float(self.x_input.text())
            pick_y = float(self.y_input.text())
            place_x = float(self.pick_place_x_input.text())
            place_y = float(self.pick_place_y_input.text())
        except ValueError:
            self.node.get_logger().warn("Invalid pick/place target - enter numbers for all 4 fields.")
            return

        goal_msg = PickPlace.Goal()
        goal_msg.pick_x = pick_x
        goal_msg.pick_y = pick_y
        goal_msg.place_x = place_x
        goal_msg.place_y = place_y

        self.node.pick_place_status = "sending_goal"
        send_goal_future = self.node.pick_place_action_client.send_goal_async(
            goal_msg, feedback_callback=self.pick_place_feedback_callback
        )
        send_goal_future.add_done_callback(self.pick_place_goal_response_callback)

    def pick_place_feedback_callback(self, feedback_msg):
        self.node.pick_place_status = feedback_msg.feedback.status

    def pick_place_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.pick_place_status = "goal_rejected"
            return
        self.node.pick_place_status = "goal_accepted"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.pick_place_result_callback)

    def pick_place_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.node.pick_place_status = f"complete: {result.message}"
        else:
            self.node.pick_place_status = f"failed: {result.message}"


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    node = GuiNode()
    window = RobotWindow(node)
    # Positioned clear of Gazebo's default window rect so the two windows
    # (started independently by the launch file, with no guaranteed
    # stacking order) don't land on top of each other at startup.
    window.move(1150, 50)
    window.show()
    window.raise_()
    window.activateWindow()
    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()