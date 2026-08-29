from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    
    urdf_path: PathJoinSubstitution = PathJoinSubstitution([
        FindPackageShare('hexapod_description'), 'urdf', 'assembly_1.urdf'   
    ])
    
    robot_description: dict[str, Command] = { 'robot_description': Command(['cat ', urdf_path]) }
    
    ros2_control_node: Node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[robot_description],
        output='screen'
    )
    
    return LaunchDescription([ros2_control_node])