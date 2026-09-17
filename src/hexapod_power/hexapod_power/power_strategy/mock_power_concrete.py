from .power_strategy import PowerStrategy, PowerTelemetry, ChannelReading

class MockPowerConcrete(PowerStrategy):
    """Mock strategy for desktop testing without INA3221 hardware."""

    def initialize(self) -> bool:
        return True

    def read_power_data(self) -> PowerTelemetry:
        readings = [
            ChannelReading(bus_voltage=6.6, shunt_voltage=0.01, current=1.2, power=7.92),  # 2S LiFePO4 (~6.6V)
            ChannelReading(bus_voltage=5.0, shunt_voltage=0.005, current=0.5, power=2.5),  # 5V Logic
            ChannelReading(bus_voltage=6.6, shunt_voltage=0.001, current=0.1, power=0.66), # Aux
        ]
        return PowerTelemetry(channels=readings, is_healthy=True, status_message="Mock Hardware OK")

    def shutdown(self) -> None:
        pass
