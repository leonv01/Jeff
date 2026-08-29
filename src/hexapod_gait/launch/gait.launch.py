from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    total_steps_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        'total_steps',
        default_value='30',
        description='Total steps per cycle gait'
    )
    
    gait_node: Node = Node(
        package='hexapod_gait',
        executable='hexapod_gait',
        name='hexapod_gait_node',
        parameters=[{ 'total_steps': LaunchConfiguration('total_steps') }],
        output='screen'
    )
    
    return LaunchDescription([
        total_steps_arg,
        gait_node
    ])