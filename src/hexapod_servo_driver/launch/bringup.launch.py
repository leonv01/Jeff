"""
Master unified launch file for the complete Hexapod system.
Starts servo_driver_node, hexapod_gait_node, hexapod_teleop_node, and joy_node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    # --- Launch Arguments ---
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='pca9685',
        description='Hardware strategy: pca9685 (physical servos) or mock (simulation)'
    )

    total_steps_arg = DeclareLaunchArgument(
        'total_steps',
        default_value='30',
        description='Total discrete steps per gait cycle'
    )

    launch_joy_arg = DeclareLaunchArgument(
        'launch_joy',
        default_value='true',
        description='Whether to launch the joy_node for gamepad input'
    )

    # --- Node Definitions ---
    
    # 1. Standalone PCA9685 Servo Driver Node
    servo_driver_node = Node(
        package='hexapod_servo_driver',
        executable='servo_driver_node',
        name='hexapod_servo_driver_node',
        parameters=[{
            'hardware_type': LaunchConfiguration('hardware_type'),
            'joint_commands_topic': '/joint_commands',
        }],
        output='screen'
    )

    # 2. Hexapod Gait & Inverse Kinematics Node
    gait_node = Node(
        package='hexapod_gait',
        executable='hexapod_gait',
        name='hexapod_gait_node',
        parameters=[{
            'total_steps': LaunchConfiguration('total_steps')
        }],
        output='screen'
    )

    # 3. Teleoperation Controller Node
    teleop_node = Node(
        package='hexapod_teleop',
        executable='hexapod_teleop',
        name='hexapod_teleop_node',
        output='screen'
    )

    # 4. Standard ROS 2 Gamepad / Joystick Node (Optional)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_joy'))
    )

    return LaunchDescription([
        hardware_type_arg,
        total_steps_arg,
        launch_joy_arg,
        servo_driver_node,
        gait_node,
        teleop_node,
        joy_node
    ])
