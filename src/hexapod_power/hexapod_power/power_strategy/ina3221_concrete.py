import board  # type: ignore
import adafruit_ina3221  # type: ignore

from .power_strategy import (
    PowerStrategy, 
    PowerTelemetry, 
    ChannelReading
)

class INA3221Concrete(PowerStrategy):
    def __init__(self, 
                 i2c_address: int = 0x40, 
                 shunt_resistor_ohms: float = 0.1) -> None:
        self.address = i2c_address
        self.shunt_resistor = shunt_resistor_ohms
        self.sensor = None
        
    def initialize(self) -> bool:
        try:
            i2c = board.I2C()
            self.sensor = adafruit_ina3221.INA3221(i2c, address=self.address)
            
            for ch in range(1, 4):
                self.sensor[ch].shunt_resistance = self.shunt_resistor
                
            return True
        
        except Exception as e:
        
            return False    
    
    def read_power_data(self) -> PowerTelemetry:
        
        if not self.sensor:
            return PowerTelemetry(channels=[], is_healthy=False, status_message="Hardware not initialized")
        
        channels = []
        
        try:
            
            for ch in range(1, 4):
                v_bus = float(self.sensor[ch].bus_voltage)
                v_shunt = float(self.sensor[ch].shunt_voltage)
                current = float(self.sensor[ch].current)
                power = v_bus * current
                channels.append(ChannelReading(v_bus, v_shunt, current, power))
                
            return PowerTelemetry(channels=channels, is_healthy=True, status_message="OK")
        
        except Exception as e:
            return PowerTelemetry(channels=[], is_healthy=False, status_message="I2C Read Error: {e}")
        
        
    def shutdown(self) -> None:
        self.sensor = None