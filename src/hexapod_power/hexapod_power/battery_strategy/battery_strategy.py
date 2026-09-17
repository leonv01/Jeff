from abc import ABC, abstractmethod
from enum import Enum

class BATTERY_STATE(Enum):
    NORMAL = 0
    WARNING = 1
    CRITICAL = 2

class BatteryStrategy(ABC):
    
    @abstractmethod
    def calculate_percentage(self, voltage: float, current: float = 0.0) -> float:
        pass
    
    @abstractmethod
    def is_low_voltage(self, voltage: float) -> bool:
        pass
    
    @abstractmethod
    def get_technology(self) -> int:
        pass
    
    @abstractmethod
    def get_health(self, voltage: float) -> int:
        pass