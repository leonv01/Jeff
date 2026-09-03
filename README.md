# Jeff Hexapod Robot

ROS 2 workspace for the Jeff Hexapod robot, containing robot description models, gait generation, hardware control interfaces, and teleoperation nodes.

## Documentation

For a detailed breakdown of package architecture, design patterns, Doxygen API documentation, and hardware interface flow, see:
- [Documentation Index](docs/README.md)
- [System & Package Architecture Overview](docs/ARCHITECTURE.md)

## Packages

- **hexapod_description**: URDF robot models, meshes, and RViz visualization launch files.
- **hexapod_gait**: Gait generation strategies (Tripod Gait, Wave Gait, Ripple Gait), trajectory generation, and gait factory with full Doxygen API documentation.
- **hexapod_hardware**: ros2_control Hardware Interface plugin for PCA9685 servo control and hardware simulation.
- **hexapod_teleop**: Teleoperation control node.


## Prerequisites

- ROS 2 (Humble recommended)
- colcon build tool
- System packages and dependencies:

```bash
sudo apt update
sudo apt install ros-humble-joint-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-hardware-interface \
                 ros-humble-rclcpp \
                 ros-humble-rclcpp-lifecycle \
                 ros-humble-pluginlib
```

## Building the Workspace

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To build a single package:

```bash
colcon build --packages-select <package_name>
```

## How to Run

### 1. Single-Command Master Bringup (Recommended)

To launch the complete Hexapod system (Servo Driver, Gait/IK Generator, Teleop, and Gamepad driver) with a single command:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Real hardware execution on Raspberry Pi:
ros2 launch hexapod_servo_driver bringup.launch.py

# Simulation / Mock hardware dry-run:
ros2 launch hexapod_servo_driver bringup.launch.py hardware_type:=mock
```

---

### 2. Visualize Robot Model in RViz

To launch RViz with joint slider GUI controls:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hexapod_description display.launch.py
```

To launch without the joint slider GUI:

```bash
ros2 launch hexapod_description display.launch.py use_gui:=false
```

If you encounter OpenGL or DRI driver errors (e.g. in VM/Docker/Nouveau), force software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch hexapod_description display.launch.py
```

### 2. Run Gait Node

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run hexapod_gait hexapod_gait
```

### 3. Run Teleoperation Node

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run hexapod_teleop hexapod_teleop --ros-args --params-file src/hexapod_teleop/config/ps4.yaml
```

## Teleoperation & Controller Setup

### 1. Gamepad Passthrough (Docker / Dev Container)
The container environment (`docker-compose.yml` and `.devcontainer/devcontainer.json`) is configured to pass through host input devices via `--device=/dev/input`.

If you encounter permission issues reading `/dev/input/js*` or `/dev/input/event*` inside the container, grant read/write permissions on the host:

```bash
sudo chmod a+rw /dev/input/js* /dev/input/event*
```

### 2. Testing Gamepad & Teleop Topics
Install the joystick driver packages inside the container:

```bash
sudo apt update
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```

* **Step 1: Start Joystick Driver (`/joy`)**:
  ```bash
  ros2 run joy joy_node
  ```

* **Step 2: Run Hexapod Teleop Node (`/gait_mode`)**:
  Publishes gait selection strings (`tripod_gait`, `wave_gait`, `ripple_gait`) when pressing controller buttons (Cross, Square, Circle):
  ```bash
  ros2 run hexapod_teleop hexapod_teleop --ros-args --params-file src/hexapod_teleop/config/ps4.yaml
  # In a separate terminal:
  ros2 topic echo /gait_mode
  ```

* **Step 3: Run Velocity Teleop (`/cmd_vel`)**:
  Publishes velocity commands when holding down the **L1 Bumper** and moving analog sticks:
  ```bash
  ros2 launch teleop_twist_joy teleop-launch.py config_filepath:=src/hexapod_teleop/config/ps4.yaml
  # In a separate terminal:
  ros2 topic echo /cmd_vel
  ```

