import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_imu')

    # Launch Arguments
    bus_number_arg = DeclareLaunchArgument(
        'bus_number',
        default_value='1',
        description='I2C Bus number on Raspberry Pi',
    )
    device_address_arg = DeclareLaunchArgument(
        'device_address',
        default_value='104',  # 0x68 in decimal
        description='MPU-6500 I2C device address (0x68 = 104)',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='TF Frame ID for IMU messages',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='IMU publish frequency in Hz',
    )

    # Driver Node
    imu_node = Node(
        package='hexapod_imu',
        executable='hexapod_imu_node',
        name='hexapod_imu_node',
        output='screen',
        parameters=[
            {
                'bus_number': LaunchConfiguration('bus_number'),
                'device_address': LaunchConfiguration('device_address'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
    )

    # Madgwick Orientation Filter Node
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[
            {
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'fixed_frame': 'base_link',
            }
        ],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data', '/imu/data'),
        ],
    )

    return LaunchDescription(
        [
            bus_number_arg,
            device_address_arg,
            frame_id_arg,
            publish_rate_arg,
            imu_node,
            imu_filter_node,
        ]
    )
