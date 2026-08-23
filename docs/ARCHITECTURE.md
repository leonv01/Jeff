# Jeff Hexapod System Architecture

This document provides a detailed architectural breakdown of the **Jeff Hexapod** codebase, structured per package.

---

## 1. High-Level System Architecture

The project consists of four ROS 2 packages organized following modular design principles (Strategy Pattern, Factory Pattern, Hardware Abstraction Layer, and `ros2_control` integration).

```mermaid
graph TD
    subgraph Teleoperation["Package: hexapod_teleop"]
        TeleopNode["HexapodTeleop Node"]
        TeleopFactory["TeleopFactory"]
        TeleopStrat["TeleopStrategy (Interface)"]
        TeleopController["TeleopControllerConcrete"]
    end

    subgraph GaitAndIK["Package: hexapod_gait"]
        GaitNode["HexapodGait Node"]
        GaitFactory["GaitFactory"]
        GaitStrat["GaitStrategy (Interface)"]
        TripodGait["TripodGaitConcrete"]
        WaveGait["WaveGaitConcrete"]
        RippleGait["RipleGaitConcrete"]
        TrajGen["TrajectoryGenerator"]
        IkSolver["HexapodIk (IK Solver)"]
    end


    subgraph Description["Package: hexapod_description"]
        URDF["assembly_1.urdf (URDF Model)"]
        RSP["robot_state_publisher"]
        JSP["joint_state_publisher / gui"]
        RViz["RViz2 Visualization"]
    end

    subgraph ControlAndHardware["Package: hexapod_hardware (ros2_control Plugin)"]
        HexSys["HexapodSystem (SystemInterface)"]
        ServoStrat["ServoStrategy (HAL Interface)"]
        PCA9685["PCA9685Concrete (I2C Driver)"]
        MockHW["MockConcrete (Simulator)"]
    end

    subgraph PhysicalHardware["Hardware Layer"]
        I2CBus["I2C Bus (/dev/i2c-1)"]
        PCA_Board1["PCA9685 #1 (0x40) - 9 Servos"]
        PCA_Board2["PCA9685 #2 (0x41) - 9 Servos"]
        Servos["18x Servo Motors (6 Legs x 3 Joints)"]
    end

    %% Interactions
    TeleopNode -->|Commands| GaitNode
    GaitNode -->|Calculates Leg Trajectories| GaitStrat
    GaitStrat -->|Cartesian Targets| IkSolver
    IkSolver -->|Joint Angles rad| HexSys
    HexSys -->|Command Interface| ServoStrat
    ServoStrat --> PCA9685
    ServoStrat --> MockHW
    PCA9685 -->|PWM Ticks over I2C| I2CBus
    I2CBus --> PCA_Board1
    I2CBus --> PCA_Board2
    PCA_Board1 --> Servos
    PCA_Board2 --> Servos
    URDF --> RSP
    RSP --> RViz
    JSP --> RSP
```

---

## 2. Package Breakdowns & Internal Architecture

### 2.1 [`hexapod_description`](../src/hexapod_description)

#### Overview
Defines the physical and kinematic model of the hexapod robot using Unified Robot Description Format (URDF) and provides launch configurations for state publishing and RViz visualization.

#### Architectural Diagram

```mermaid
graph LR
    subgraph Files["Package Assets"]
        URDF_File["urdf/assembly_1.urdf"]
        RViz_File["rviz/display.rviz"]
        Launch_Py["launch/display.launch.py"]
    end

    subgraph Nodes["ROS 2 Nodes"]
        RSP_Node["robot_state_publisher"]
        JSP_Node["joint_state_publisher / _gui"]
        RViz_Node["rviz2"]
    end

    Launch_Py -->|Loads URDF| RSP_Node
    Launch_Py -->|Publishes /joint_commands| JSP_Node
    Launch_Py -->|Loads Config| RViz_Node
    RSP_Node -->|Publishes /tf & /tf_static| RViz_Node
    JSP_Node -->|Publishes /joint_states| RSP_Node
```

#### Key Components:
- **URDF Model (`urdf/assembly_1.urdf`)**: Describes 18 revolute joints across 6 legs (`LF`, `LM`, `LR`, `RF`, `RM`, `RR`) with three segments per leg: `coxa`, `femur`, and `tibia`.
- **Display Launch (`launch/display.launch.py`)**: Configures `robot_state_publisher` and `joint_state_publisher_gui` to visualize joint states in `rviz2`.

---

### 2.2 [`hexapod_gait`](../src/hexapod_gait)

#### Overview
Responsible for gait generation, step sequencing, and analytical inverse kinematics (IK) calculation for all 6 legs.

#### Class Diagram & Architecture

```mermaid
classDiagram
    class rclcpp_Node {
        <<ROS 2 Node>>
    }

    class HexapodGait {
        -int step_counter_
        -unique_ptr~GaitStrategy~ gait_strategy_
        -Twist current_velocity_
        -TimerBase control_timer_
        +HexapodGait()
        -cmd_vel_callback(Twist msg)
        -gait_mode_callback(String msg)
        -body_pose_callback(Pose msg)
        -control_timer_callback()
    }

    class HexapodIk {
        -double COXA_LENGTH = 0.052
        -double FEMUR_LENGTH = 0.075208
        -double TIBIA_LENGTH = 0.098265
        +solve_ik(Vector3d target)$ Vector3d
    }

    class TrajectoryGenerator {
        +calculate_swing_trajectory(double stride_length, double swing_height, double gait_phase)$ Vector3d
        +calculate_stance_trajectory(double stride_length, double stance_phase)$ Vector3d
    }

    class GaitStrategy {
        <<Abstract Interface>>
        #double duty_cycle_
        #int total_cycle_steps_
        #int current_cycle_steps_
        #unordered_map~LEG, LegData~ leg_map_
        #unordered_map~LEG, double~ leg_offsets_
        +propagate_gait(int step, double stride, double height, double alpha)* unordered_map
        +propagate_leg(LEG leg, int step, double stride, double height, double alpha)* LegData
        +get_leg(LEG leg_id) LegData
        +update_current_steps(int steps)
    }

    class TripodGaitConcrete {
        +TripodGaitConcrete(double duty_cycle, double total_cycle_steps)
        +propagate_gait(int step, double stride, double height, double alpha) unordered_map
        +propagate_leg(LEG leg, int step, double stride, double height, double alpha) LegData
    }

    class WaveGaitConcrete {
        +WaveGaitConcrete(double duty_cycle, double total_cycle_steps)
        +propagate_gait(int step, double stride, double height, double alpha) unordered_map
        +propagate_leg(LEG leg, int step, double stride, double height, double alpha) LegData
    }

    class RipleGaitConcrete {
        +RipleGaitConcrete(double duty_cycle, double total_cycle_steps)
        +propagate_gait(int step, double stride, double height, double alpha) unordered_map
        +propagate_leg(LEG leg, int step, double stride, double height, double alpha) LegData
    }

    class GaitFactory {
        +create_gait(string type, double duty_cycle)$ unique_ptr~GaitStrategy~
    }

    class LegData {
        <<struct>>
        +double coxa_joint_
        +double tibia_joint_
        +double femur_joint_
        +double leg_phase_
        +double leg_stance_offset_
        +double leg_mount_angle_
        +Vector3d base_point_
        +LEG_SIDE leg_side_
    }

    rclcpp_Node <|-- HexapodGait
    HexapodGait --> GaitStrategy : uses active strategy
    HexapodGait ..> GaitFactory : instantiates via
    GaitStrategy <|-- TripodGaitConcrete : implements
    GaitStrategy <|-- WaveGaitConcrete : implements
    GaitStrategy <|-- RipleGaitConcrete : implements
    GaitStrategy ..> LegData : manages
    GaitStrategy ..> TrajectoryGenerator : calculates trajectories via
    GaitStrategy ..> HexapodIk : calculates IK joint angles via
```

#### Key Design Patterns & Modules:
- **Strategy Pattern**: `GaitStrategy` defines the abstract interface for leg gait propagation. `TripodGaitConcrete` (Duty Cycle: 0.5), `WaveGaitConcrete` (Duty Cycle: 0.833), and `RipleGaitConcrete` (Duty Cycle: 0.667) implement specific leg phase offset configurations and timing profiles.
- **Factory Pattern**: `GaitFactory::create_gait()` instantiates requested strategies dynamically by string name (`"tripod_gait"`, `"wave_gait"`, `"ripple_gait"` / `"riple_gait"`).
- **Trajectory Generator (`TrajectoryGenerator`)**: Computes 3D parabolic swing trajectories ($z = -H \sin(\phi)$) and linear stance ground return trajectories for leg propagation.
- **Analytical IK Solver (`HexapodIk`)**: Solves 3-DOF inverse kinematics for each leg given target Cartesian coordinates $(x, y, z)$:
  $$\theta_1 = \text{atan2}(y, x)$$
  $$\theta_2 = \alpha + \beta$$
  $$\theta_3 = \text{acos}\left(\frac{L_F^2 + L_T^2 - c^2}{2 L_F L_T}\right) - \pi$$
- **Doxygen API Documentation**: Every header and source file in `hexapod_gait` is fully documented in standard Doxygen format (`/** ... */`) covering file headers, classes, methods, parameters, return values, macros, structs, and enums.


---

### 2.3 [`hexapod_hardware`](../src/hexapod_hardware)

#### Overview
Integrates physical servo drivers with the `ros2_control` framework using a custom `hardware_interface::SystemInterface` plugin.

#### Class Diagram & Architecture

```mermaid
classDiagram
    class SystemInterface {
        <<ros2_control Interface>>
    }

    class HexapodSystem {
        -vector~double~ hw_commands_
        -vector~double~ hw_states_
        -unique_ptr~ServoStrategy~ servo_strategy_
        +on_init(HardwareInfo info) CallbackReturn
        +export_state_interfaces() vector~StateInterface~
        +export_command_interfaces() vector~CommandInterface~
        +on_activate(State previous_state) CallbackReturn
        +on_deactivate(State previous_state) CallbackReturn
        +read(Time time, Duration period) return_type
        +write(Time time, Duration period) return_type
    }

    class ServoStrategy {
        <<Abstract HAL Interface>>
        +initialize()* bool
        +set_angle_rad(ServoCommand cmd)*
        +set_angle_degree(ServoCommand cmd)*
        +get_angle_rad(ServoCommand cmd)* bool
        +shutdown()*
    }

    class PCA9685Concrete {
        -unordered_map~uint, unique_ptr~PCA9685~~ pcas_
        -unordered_map~uint, tuple~uint,uint~~ channel_mapping_
        -double FREQUENCY_ = 50.0
        +initialize() bool
        +set_angle_rad(ServoCommand cmd)
        +set_angle_degree(ServoCommand cmd)
        +get_angle_rad(ServoCommand cmd) bool
        +shutdown()
        -rad_to_pwm(double angle) uint
    }

    class MockConcrete {
        -Logger logger_
        -unordered_map~uint, double~ mock_angles_
        +initialize() bool
        +set_angle_rad(ServoCommand cmd)
        +set_angle_degree(ServoCommand cmd)
        +get_angle_rad(ServoCommand cmd) bool
        +shutdown()
    }

    class ServoCommand {
        <<struct>>
        +unsigned int leg
        +double angle
    }

    SystemInterface <|-- HexapodSystem
    HexapodSystem --> ServoStrategy : delegates write/read to
    ServoStrategy <|-- PCA9685Concrete : implements (Real HW)
    ServoStrategy <|-- MockConcrete : implements (Simulation)
    ServoStrategy ..> ServoCommand : accepts
```

#### Key Features:
- **`ros2_control` Plugin**: Exported via `PLUGINLIB_EXPORT_CLASS(hexapod_hardware::HexapodSystem, hardware_interface::SystemInterface)` and declared in `hexapod_hardware.xml`.
- **PCA9685 I2C Mapping**: Drives 18 servos across two PCA9685 I2C boards at `0x40` (channels 0..8) and `0x41` (channels 9..17) at 50 Hz PWM frequency.
- **Hardware Abstraction Layer (HAL)**: `ServoStrategy` enables seamless switching between physical PCA9685 execution (`PCA9685Concrete`) and dry-run mock testing (`MockConcrete`).

---

### 2.4 [`hexapod_teleop`](../src/hexapod_teleop)

#### Overview
Provides the teleoperation interface for receiving remote control commands and delegating motion directives to the gait system.

#### Class Diagram & Architecture

```mermaid
classDiagram
    class rclcpp_Node {
        <<ROS 2 Node>>
    }

    class HexapodTeleop {
        -Subscription~Joy~::SharedPtr joy_subscriber_
        +HexapodTeleop()
        -JoySubscriber(SharedPtr msg)
    }

    rclcpp_Node <|-- HexapodTeleop
```

---

## 3. Data Flow Sequence

The end-to-end execution loop for joint movement follows the sequence below:

```mermaid
sequenceDiagram
    autonumber
    actor User as User / Gamepad
    participant Teleop as hexapod_teleop
    participant Gait as hexapod_gait
    participant IK as HexapodIk
    participant Controller as ros2_control (Manager)
    participant HW as HexapodSystem
    participant HAL as PCA9685Concrete
    participant Servos as I2C Servo Hardware

    User->>Teleop: Motion Input (Joystick/Cmd)
    Teleop->>Gait: Direction / Velocity Command
    Gait->>Gait: Select Gait Strategy (Wave/Ripple)
    Gait->>IK: Pass Foot Position (x, y, z)
    IK-->>Gait: Return Joint Angles (theta1, theta2, theta3)
    Gait->>Controller: Publish Joint Angles / Commands
    Controller->>HW: write(time, period)
    HW->>HAL: set_angle_rad(ServoCommand)
    HAL->>HAL: Convert Radians to PWM Ticks (100..500)
    HAL->>Servos: I2C PWM Command to 0x40 / 0x41
```
