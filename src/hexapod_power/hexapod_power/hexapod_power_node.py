import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from .power_strategy.power_strategy import PowerStrategy
from .power_strategy.ina3221_concrete import INA3221Concrete

from .battery_strategy.battery_strategy import BatteryStrategy
from .battery_strategy.lifepo4_concrete import LiFePO4Concrete

from .power_strategy.mock_power_concrete import MockPowerConcrete

class HexapodPowerNode(Node):
    def __init__(self):
        super().__init__('hexapod_power_node')
    
        # Declare parameters with default values
        self.declare_parameter('i2c_address', 0x42)
        self.declare_parameter('use_mock', False)
        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('num_cells', 2)

        i2c_address = self.get_parameter('i2c_address').value
        use_mock = self.get_parameter('use_mock').value
        update_rate = self.get_parameter('update_rate_hz').value
        num_cells = self.get_parameter('num_cells').value

        # Convert hex string if passed as string parameter from launch
        if isinstance(i2c_address, str):
            i2c_address = int(i2c_address, 16)

        if use_mock:
            self.strategy: PowerStrategy = MockPowerConcrete()
        else:
            self.strategy: PowerStrategy = INA3221Concrete(i2c_address=i2c_address)

        self.battery_type: BatteryStrategy = LiFePO4Concrete(num_cells=num_cells, low_battery_threshold=0.2)
        
        if not self.strategy.initialize():
            self.get_logger().warn("Failed to initialize INA3221 strategy, falling back to Mock Strategy")
            self.strategy = MockPowerConcrete()
            self.strategy.initialize()
        else:
            self.get_logger().info(f"Successfully initialized INA3221 strategy (I2C: 0x{i2c_address:02X})")
            
        self.batt_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        
        timer_period = 1.0 / update_rate
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        telemetry = self.strategy.read_power_data()
        
        if not telemetry.is_healthy or not telemetry.channels:
            self.get_logger().warn("Power telemetry not available or unhealthy")
            return
        
        ch1 = telemetry.channels[0]
        timestamp = self.get_clock().now().to_msg()
        
        batt_msg = BatteryState()
        batt_msg.header.stamp = timestamp
        batt_msg.header.frame_id = 'base_link'
        
        batt_msg.voltage = ch1.bus_voltage
        batt_msg.current = ch1.current
        batt_msg.percentage = self.battery_type.calculate_percentage(ch1.bus_voltage, ch1.current)
        batt_msg.power_supply_technology = self.battery_type.get_technology()
        batt_msg.power_supply_health = self.battery_type.get_health(ch1.bus_voltage)
        batt_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        batt_msg.present = True
        
        self.batt_pub.publish(batt_msg)
        self.get_logger().info(f"Battery: {ch1.bus_voltage:.2f}V, Current: {ch1.current:.2f}A, Soc: {batt_msg.percentage*100:.1f}%")

def main(args=None):
    rclpy.init(args=args)
    node = HexapodPowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
