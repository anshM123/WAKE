from __future__ import annotations
import json, socket, threading
from wake.pose.base import PoseProvider
from wake.protocol.messages import pose_from_mapping
from wake.types import PoseSample

class UDPPoseProvider(PoseProvider):
    def __init__(self, bind: str = "0.0.0.0", port: int = 5006) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self._socket.bind((bind, port)); self._latest: PoseSample | None = None; self._lock = threading.Lock()
    def receive_once(self) -> PoseSample:
        payload, _ = self._socket.recvfrom(4096); message = json.loads(payload.decode("utf-8")); pose = pose_from_mapping(message)
        with self._lock: self._latest = pose
        return pose
    def latest_pose(self) -> PoseSample | None:
        with self._lock: return self._latest
    def close(self) -> None: self._socket.close()
