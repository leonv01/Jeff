from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='pca9685',
        description='Hardware strategy: pca9685 or mock'
    )

    topic_arg = DeclareLaunchArgument(
        'joint_commands_topic',
        default_value='/joint_commands',
        description='Topic to subscribe for joint position commands'
    )

    servo_driver_node = Node(
        package='hexapod_servo_driver',
        executable='servo_driver_node',
        name='hexapod_servo_driver_node',
        parameters=[{
            'hardware_type': LaunchConfiguration('hardware_type'),
            'joint_commands_topic': LaunchConfiguration('joint_commands_topic'),
        }],
        output='screen'
    )

    return LaunchDescription([
        hardware_type_arg,
        topic_arg,
        servo_driver_node
    ])
