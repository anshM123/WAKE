from __future__ import annotations
import logging, socket
from wake.protocol.messages import decode_telemetry
from wake.protocol.sequence import SequenceTracker
from wake.types import TelemetrySample

LOG=logging.getLogger(__name__)
class TelemetryReceiver:
    def __init__(self, bind: str="0.0.0.0", port: int=5005) -> None: self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.socket.bind((bind,port)); self.socket.settimeout(.25); self.sequence=SequenceTracker(); self.invalid_packets=0; self.dropped_packets=0; self.duplicates=0; self.reordered=0
    def receive_once(self) -> TelemetrySample | None:
        try: payload,_=self.socket.recvfrom(4096)
        except socket.timeout: return None
        try: sample=decode_telemetry(payload)
        except ValueError as exc: self.invalid_packets+=1; LOG.warning("Rejected telemetry packet: %s",exc); return None
        update=self.sequence.update(sample.sequence); self.dropped_packets+=update.lost
        if update.reordered or update.duplicate:
            self.reordered += int(update.reordered); self.duplicates += int(update.duplicate)
            LOG.warning("Rejected reordered/duplicate telemetry sequence %d",sample.sequence); return None
        return sample
    def close(self)->None:self.socket.close()
