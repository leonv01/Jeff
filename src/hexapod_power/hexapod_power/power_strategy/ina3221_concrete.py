import board  # type: ignore
import traceback
import adafruit_ina3221  # type: ignore
from adafruit_extended_bus import ExtendedI2C
import time

from .power_strategy import (
    PowerStrategy, 
    PowerTelemetry, 
    ChannelReading
)

class INA3221Concrete(PowerStrategy):
    def __init__(self, 
                 i2c_bus: int = 0x04,
                 i2c_address: int = 0x40, 
                 shunt_resistor_ohms: float = 0.1) -> None:
        self.address = i2c_address
        self.i2c_bus = i2c_bus
        self.shunt_resistor = shunt_resistor_ohms
        self.sensor = None
        
    def initialize(self) -> bool:
        try:
            i2c = ExtendedI2C(self.i2c_bus)
            self.sensor = adafruit_ina3221.INA3221(i2c, address=self.address)
            
            # Setup only. No reads here.
            for ch in range(3):
                self.sensor[ch].shunt_resistance = self.shunt_resistor

            time.sleep(0.2) 
            return True
        
        except Exception as e:
            self.last_error = traceback.format_exc()
            self.sensor = None
            return False
    
    def read_power_data(self) -> PowerTelemetry:
        
        if not self.sensor:
            return PowerTelemetry(channels=[], is_healthy=False, status_message="Hardware not initialized")
        
        channels = []
        
        try:
            
            for ch in range(3):
                c = self.sensor[ch]
                v_bus = float(c.bus_voltage)
                v_shunt = float(c.shunt_voltage)
                current = float(c.current)
                channels.append(ChannelReading(v_bus, v_shunt, current, v_bus * current))
            
            if all(c.bus_voltage < 0.5 for c in channels):
                return PowerTelemetry(channels=[], is_healthy=False,
                                      status_message="No voltage on any channel")

            return PowerTelemetry(channels=channels, is_healthy=True, status_message="OK")

        except Exception as e:
            return PowerTelemetry(channels=[], is_healthy=False,
                                  status_message=f"I2C Read Error: {e}")
        
    def shutdown(self) -> None:
        self.sensor = None
