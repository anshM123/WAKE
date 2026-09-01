from dataclasses import dataclass
import time
from wake.control.base import FlightController
@dataclass(frozen=True)
class CommandEnvelope:sequence:int;timestamp_ns:int
class CommandWatchdog:
    def __init__(self,controller:FlightController,timeout_ms:float=250)->None:self.controller=controller;self.timeout_ns=timeout_ms*1e6;self.last:CommandEnvelope|None=None
    def accept(self,sequence:int,timestamp_ns:int|None=None)->None:
        timestamp_ns=time.monotonic_ns() if timestamp_ns is None else timestamp_ns
        if self.last is not None and sequence<=self.last.sequence:raise ValueError("command sequence must increase")
        self.last=CommandEnvelope(sequence,timestamp_ns)
    def check(self,now_ns:int|None=None)->bool:
        now_ns=time.monotonic_ns() if now_ns is None else now_ns
        if self.last is None or now_ns-self.last.timestamp_ns>self.timeout_ns:self.controller.hold();return False
        return True
