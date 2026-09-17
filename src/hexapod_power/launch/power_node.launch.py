import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare Launch Arguments
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='0x42',
        description='I2C Address for the INA3221 sensor board'
    )

    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Whether to run in hardware mock mode'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate_hz',
        default_value='1.0',
        description='Telemetry update rate in Hz'
    )

    # Power Node Action
    power_node = Node(
        package='hexapod_power',
        executable='hexapod_power_node',
        name='hexapod_power_node',
        output='screen',
        parameters=[{
            'i2c_address': LaunchConfiguration('i2c_address'),
            'use_mock': LaunchConfiguration('use_mock'),
            'update_rate_hz': LaunchConfiguration('update_rate_hz'),
        }]
    )

    return LaunchDescription([
        i2c_address_arg,
        use_mock_arg,
        update_rate_arg,
        power_node
    ])
