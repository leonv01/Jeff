from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    teleop_node: Node = Node(
        package='hexapod_teleop',
        executable='hexapod_teleop',
        name='hexapod_teleop_node',
        output='screen'
    )
    
    return LaunchDescription([teleop_node])