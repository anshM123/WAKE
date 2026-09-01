from wake.control.base import FlightController

class MatrixAdapter(FlightController):
    """Fail-closed placeholder until Matrix high-level setpoints are documented.

    MSP_SET_MOTOR is explicitly not a flight-control implementation.
    """
    def _disabled(self)->None:raise RuntimeError("Matrix autonomous control disabled: verify and implement a documented high-level setpoint interface")
    def hold(self)->None:self._disabled()
    def set_velocity_world(self,vx:float,vy:float,vz:float,yaw_rate:float)->None:self._disabled()
    def land(self)->None:self._disabled()
