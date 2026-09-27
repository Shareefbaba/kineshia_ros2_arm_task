# Design note — 3-DoF planar arm

## What I built

The main demonstration is a ROS 2 Humble controller and PyQtGraph operator GUI for the supplied three-link arm. `bringup.launch.py` starts the controller and GUI and activates the lifecycle controller. The supplied `planar_arm.py` handles forward and inverse kinematics; I kept it unchanged. `my_custom_interfaces` defines the `PickPlace` action.

The GUI sends a single Cartesian target on `/target_pose` or a pick/place pair to the `/pick_place` action. The controller checks the requested pose, runs IK using the current joint angles as its initial guess, then checks joint limits and the ground constraint. In position mode it samples a quintic joint-space move over two seconds and publishes `/joint_states` at 50 Hz. Velocity mode is selectable with the `control_mode` parameter and moves toward the same IK result with a bounded proportional velocity. The GUI subscribes to `/joint_states`, draws the arm from those angles, plots joint angles and target error, and displays action status. A Qt timer calls nonblocking `rclpy.spin_once`, so ROS callbacks and drawing can run alongside the Qt event loop. The controller uses a multithreaded executor so the action can wait for motion while the control timer keeps running.

For pick and place, the action moves to the pick point, pauses to represent a pick, moves to the place point, then pauses again. It reports each phase and supports timeout and cancellation. There is no simulated gripper or object attachment, so the action demonstrates sequencing rather than physical grasping. The controller also logs its timer intervals to make timing variation visible.

## Decisions and limits

I chose joint-space quintic interpolation because it gives smooth starts and stops with a small planner. Each sampled pose is checked against the joint and ground limits; an unsafe route is rejected. The 2D controller publishes the commanded joint state as simulated telemetry, so its plots show the model following the command, not measured tracking error. For the fixed demonstration, pick `(4, 2)` and place `(-3, 3)` complete in the 2D flow. The `(7, 3)` point is beyond the 6.5-unit maximum reach; the 2D controller rejects it and logs the reason. The GUI does not yet show a clear projection or rejection message for a single target.

I also added an optional `full_system.launch.py` path with an SDF arm, Gazebo bridges, and a controller that reads joint feedback before issuing commands. This is separate from the reliable 2D demonstration. I observed a small downward initial deflection and slow settling in Gazebo, so I would present it as a work in progress rather than claim physical accuracy.

The command publication is in `send_command`, which is the point I would replace with a hardware driver while retaining the target handling and planning rules. With more time, I would share the planner between the two controller variants, show rejected or projected targets in the GUI, add a GUI cancel control and a real gripper interface, then tune and measure Gazebo tracking against joint feedback.
