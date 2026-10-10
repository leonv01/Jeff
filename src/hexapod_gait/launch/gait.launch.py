import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    ld: LaunchDescription = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('hexapod_gait'),
        'config',
        'params.yaml'
    )
    
    gait_node: Node = Node(
        package='hexapod_gait',
        executable='hexapod_gait',
        name='hexapod_gait_node',
        parameters=[config],
        output='screen'
    )
    
    ld.add_action(gait_node)
    return ld