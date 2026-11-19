from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    camera_node: Node = Node(
        package='hexapod_vision',
        executable='camera_node',
        name='camera_node',
        output='screen'
    )
    
    return LaunchDescription([camera_node])