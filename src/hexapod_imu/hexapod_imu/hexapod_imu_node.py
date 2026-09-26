import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from hexapod_imu.mpu6500 import (
    ACCEL_FS_SEL_2G,
    GYRO_FS_SEL_250DPS,
    DEFAULT_ADDRESS,
    MPU6500,
)


class HexapodIMUNode(Node):
    """ROS 2 Node for reading MPU-6500 IMU data over I2C and publishing to /imu/data_raw."""

    def __init__(self) -> None:
        super().__init__('hexapod_imu_node')

        # Declare ROS 2 Parameters
        self.declare_parameter('bus_number', 1)
        self.declare_parameter('device_address', DEFAULT_ADDRESS)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz

        bus_num = self.get_parameter('bus_number').get_parameter_value().integer_value
        address = self.get_parameter('device_address').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Initialize Hardware
        self.get_logger().info(
            f'Initializing MPU-6500 on /dev/i2c-{bus_num} at address {hex(address)}...'
        )
        try:
            self.sensor = MPU6500(
                i2c_bus=bus_num,
                address=address,
                accel_fs=ACCEL_FS_SEL_2G,
                gyro_fs=GYRO_FS_SEL_250DPS,
            )
            self.get_logger().info('MPU-6500 IMU hardware initialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU-6500 hardware: {e}')
            self.sensor = None

        # Publisher
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Timer Callback
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        if self.sensor is None:
            return

        try:
            ax, ay, az = self.sensor.acceleration
            gx, gy, gz = self.sensor.gyro
        except Exception as e:
            self.get_logger().warn(f'I2C read failure: {e}', throttle_duration_sec=2.0)
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Linear Acceleration (m/s^2)
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        # Angular Velocity (rad/s)
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        # Publish Message
        self.publisher_.publish(msg)

    def destroy_node(self) -> None:
        if hasattr(self, 'sensor') and self.sensor is not None:
            self.sensor.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HexapodIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
