# Jeff Hexapod Project - Outstanding Implementation TODO List

This document lists all features, stubs, and components in the **Jeff Hexapod** workspace that are currently un-implemented, stubbed out, or incomplete.

---

## 1. `hexapod_gait` Package

- [x] **Gait Strategy Implementation (`TripodGaitConcrete`, `WaveGaitConcrete` & `RipleGaitConcrete`)**
  - [x] Implement `TripodGaitConcrete` for 3-leg stance / 3-leg swing alternating tripod gait trajectory propagation.
  - [x] Implement `WaveGaitConcrete` for 5-leg stance / 1-leg swing sequential wave gait calculation.
  - [x] Implement `RipleGaitConcrete` for 4-leg stance / 2-leg swing ripple gait calculation.
  - [x] Populate and manage `leg_map_` inside `GaitStrategy`.
  - [x] Add trajectory interpolation (`TrajectoryGenerator` parabolic swing curve and linear stance return).

- [x] **ROS 2 Node Execution & Control Loop (`hexapod_gait.cpp`)**
  - [x] Implement `main()` with ROS 2 node lifecycle and `rclcpp::spin(node)`.
  - [x] Add subscriber for velocity commands (`geometry_msgs/msg/Twist` on `/cmd_vel`).
  - [x] Add subscriber for dynamic gait mode selection (`std_msgs/msg/String` on `/gait_mode`).
  - [x] Add subscriber for body pose adjustments (`geometry_msgs/msg/Pose` on `/body_pose`).
  - [x] Add publisher for leg joint commands (`sensor_msgs/msg/JointState` on `/joint_commands`).
  - [x] Implement periodic control timer loop execution (50 Hz / 20ms period).

- [x] **Doxygen API Documentation**
  - [x] Add comprehensive Doxygen documentation (`/** ... */`) across all headers and source files in `hexapod_gait`.

- [ ] **ROS Parameters Integration**
  - [ ] Replace hardcoded default `"tripod_gait"` strategy string with dynamic ROS parameters.
  - [ ] Declare ROS parameters for step frequency, stride length bounds, body height offset, and ground clearance height.


---

## 2. `hexapod_hardware` Package

- [ ] **Hardware Strategy Selection & Config (`hexapod_system.cpp`)**
  - [ ] Replace hardcoded `servo_strategy_ = std::make_unique<MockConcrete>()` with dynamic instantiation based on ROS parameters or URDF hardware attributes (e.g. `use_mock_hardware`).

- [ ] **Hardware Feedback & Joint Reading (`HexapodSystem::read`)**
  - [ ] Replace command-to-state dummy loopback (`hw_states_[i] = hw_commands_[i]`) with actual servo state reading via `servo_strategy_->get_angle_rad(...)`.

- [ ] **PCA9685 Driver Stubs (`PCA9685Concrete`)**
  - [ ] Implement `PCA9685Concrete::set_angle_degree(ServoCommand &servo_command)` (currently `(void)servo_command`).
  - [ ] Implement `PCA9685Concrete::get_angle_rad(ServoCommand &servo_command)` for reading back state/PWM values.
  - [ ] Add per-joint calibration settings: individual zero-degree pulse width offsets, direction reversal flags, and joint angle min/max limits.
  - [ ] Make I2C device path (`/dev/i2c-1`), I2C addresses (`0x40`, `0x41`), and channel mapping configurable via YAML/URDF params.

---

## 3. `hexapod_teleop` Package

- [x] **Node Execution (`hexapod_teleop.cpp`)**
  - [x] Simplify architecture by removing obsolete `TeleopStrategy` and `TeleopFactory` boilerplate.
  - [x] Update `main()` to initialize ROS 2 and call `rclcpp::spin(node)`.
  - [x] Add subscriber for joystick inputs (`sensor_msgs/msg/Joy`) on `/joy`.

- [ ] **Command Publisher & State Processing**
  - [ ] Implement processing logic in `JoySubscriber` callback for PS4 controller inputs.
  - [ ] Implement publisher for velocity commands (`/cmd_vel`) or gait control parameters sent to `hexapod_gait_node`.

---

## 4. `hexapod_description` & System Integration

- [ ] **`ros2_control` Controllers Configuration**
  - [ ] Create `config/controllers.yaml` defining `joint_state_broadcaster` and joint command controllers (`forward_command_controller/ForwardCommandController` or `joint_trajectory_controller/JointTrajectoryController`) for all 18 joints.

- [ ] **Bringup & System Launch Files**
  - [ ] Create a combined bringup launch file (`hexapod_bringup.launch.py`) to launch:
    - `robot_state_publisher`
    - `ros2_control` `controller_manager` with `hexapod_hardware`
    - `joint_state_broadcaster` & joint controller
    - `hexapod_gait_node`
    - `hexapod_teleop_node`

---

## 5. Package Metadata & Testing Quality

- [ ] **Package Declarations (`package.xml`)**
  - [ ] Replace `TODO` placeholders in `<description>` and `<license>` tags across `hexapod_gait`, `hexapod_hardware`, and `hexapod_teleop`.

- [ ] **Automated Testing**
  - [ ] Add unit tests for `HexapodIk::solve_ik` verifying forward/inverse kinematic solutions.
  - [ ] Add unit tests for `GaitFactory` and `TeleopFactory`.
  - [ ] Add integration test for `HexapodSystem` mock hardware interface lifecycle (`on_init`, `on_activate`, `read`, `write`, `on_deactivate`).
