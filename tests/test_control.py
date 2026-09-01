import pytest
from wake.control.base import SimulatorFlightController
from wake.control.matrix_adapter import MatrixAdapter
from wake.control.watchdog import CommandWatchdog
def test_simulator_high_level_motion():
    controller=SimulatorFlightController();controller.set_velocity_world(1,0,0,0);controller.step(.5);assert controller.position[0]==.5
def test_watchdog_holds_and_rejects_replay():
    controller=SimulatorFlightController();watchdog=CommandWatchdog(controller,100);watchdog.accept(1,1000);controller.set_velocity_world(1,0,0,0);assert not watchdog.check(101_001_000);assert controller.velocity.vx==0
    with pytest.raises(ValueError):watchdog.accept(1)
def test_matrix_remains_disabled():
    adapter=MatrixAdapter()
    with pytest.raises(RuntimeError):adapter.hold()
    assert adapter.failure_mode=="FLIGHT_INTERFACE_DISABLED"
