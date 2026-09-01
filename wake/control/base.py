from __future__ import annotations
from abc import ABC,abstractmethod
from dataclasses import dataclass

@dataclass(frozen=True)
class VelocityCommand:
    vx:float;vy:float;vz:float;yaw_rate:float

class FlightController(ABC):
    @abstractmethod
    def hold(self)->None:...
    @abstractmethod
    def set_velocity_world(self,vx:float,vy:float,vz:float,yaw_rate:float)->None:...
    @abstractmethod
    def land(self)->None:...

class MockFlightController(FlightController):
    def __init__(self)->None:self.commands:list[object]=[]
    def hold(self)->None:self.commands.append("HOLD")
    def set_velocity_world(self,vx:float,vy:float,vz:float,yaw_rate:float)->None:self.commands.append(VelocityCommand(vx,vy,vz,yaw_rate))
    def land(self)->None:self.commands.append("LAND")
