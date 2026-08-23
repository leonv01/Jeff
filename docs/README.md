# Jeff Hexapod Project Documentation

Welcome to the documentation for **Jeff**, a 6-legged (hexapod) robotic platform built on **ROS 2**.

## Documentation Index

- [System Architecture Overview](ARCHITECTURE.md) - Comprehensive package-by-package architecture, class diagrams, design patterns, and hardware interface flow.
- [Outstanding Implementation TODO List](TODO.md) - Comprehensive checklist of un-implemented features, stubs, and pending tasks per package.

## Workspace Packages

| Package | Type | Description |
| :--- | :--- | :--- |
| [`hexapod_description`](../src/hexapod_description) | ROS 2 Package | Robot URDF 3D kinematic model, meshes, and RViz display launch files. |
| [`hexapod_gait`](../src/hexapod_gait) | ROS 2 Package | Gait pattern generation (Tripod, Wave, Ripple), 3D trajectory generator, 3-DOF analytical IK solver, and Doxygen API docs. |
| [`hexapod_hardware`](../src/hexapod_hardware) | ROS 2 Control Plugin | `ros2_control` `SystemInterface` plugin with HAL strategies for I2C PCA9685 PWM drivers & mock hardware. |
| [`hexapod_teleop`](../src/hexapod_teleop) | ROS 2 Package | Teleoperation controller input strategies and factory node. |

