from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List

@dataclass
class ChannelReading:
    bus_voltage: float
    shunt_voltage: float
    current: float
    power: float
    
@dataclass
class PowerTelemetry:
    channels: List[ChannelReading]
    is_healthy: bool
    status_message: str
    
class PowerStrategy(ABC):
    
    @abstractmethod
    def initialize(self) -> bool:
        pass
    
    @abstractmethod
    def read_power_data(self) -> PowerTelemetry:
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        pass