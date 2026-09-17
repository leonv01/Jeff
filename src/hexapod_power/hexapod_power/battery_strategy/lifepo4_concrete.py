from .battery_strategy import BatteryStrategy

from sensor_msgs.msg import BatteryState

import math

class LiFePO4Concrete(BatteryStrategy):
    
    def __init__(self, num_cells: int = 2, low_battery_threshold: float = 0.10):
        self.num_cells: int = num_cells
        self.cell_min: float = 2.50
        self.cell_max: float = 3.65
        
        self.resting_voltages: list[float] = [
            3.65,   # 100%   
            3.40,   # 90%
            3.35,   # 80%
            3.32,   # 70%
            3.30,   # 60%
            3.28,   # 50%
            3.26,   # 40%
            3.22,   # 30%
            3.10,   # 20%
            2.90,   # 10%
            2.50,   # 00%
        ]
        
        self.low_battery_threshold: float = low_battery_threshold
        
    def calculate_percentage(self, voltage: float, current: float = 0) -> float:
        if voltage < 2.0:
            return 0.0
        
        cell_v: float = voltage / self.num_cells
        
        percentage: float = 1.0
        
        for v in self.resting_voltages:
            if cell_v > v:
                break
            percentage -= 1.0 / len(self.resting_voltages)
        
        return max(0, min(percentage, 1.0))
    
    def is_low_voltage(self, voltage: float) -> bool:
        percentage: float = self.calculate_percentage(voltage=voltage)
        
        return percentage <= self.low_battery_threshold
    
    def get_technology(self) -> int:
        return BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
    
    def get_health(self, voltage: float) -> int:
        if self.is_low_voltage(voltage=voltage) and voltage > 2.0:
            return BatteryState.POWER_SUPPLY_HEALTH_DEAD
        return BatteryState.POWER_SUPPLY_HEALTH_GOOD