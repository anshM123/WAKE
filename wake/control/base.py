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

class SimulatorFlightController(FlightController):
    """Kinematic high-level controller for software-only planner tests."""
    def __init__(self)->None:self.velocity=VelocityCommand(0,0,0,0);self.position=[0.,0.,0.];self.landed=False
    def hold(self)->None:self.velocity=VelocityCommand(0,0,0,0)
    def set_velocity_world(self,vx:float,vy:float,vz:float,yaw_rate:float)->None:
        if self.landed:raise RuntimeError("simulated vehicle is landed")
        self.velocity=VelocityCommand(vx,vy,vz,yaw_rate)
    def land(self)->None:self.hold();self.landed=True
    def step(self,dt_s:float)->None:self.position[0]+=self.velocity.vx*dt_s;self.position[1]+=self.velocity.vy*dt_s;self.position[2]+=self.velocity.vz*dt_s
