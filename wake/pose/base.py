from __future__ import annotations
from collections import deque
from typing import Protocol
from wake.types import PoseSample

class PoseProvider(Protocol):
    def latest_pose(self) -> PoseSample | None: ...

class MockPoseProvider:
    def __init__(self, poses: list[PoseSample] | None = None) -> None: self._poses = deque(poses or [])
    def push(self, pose: PoseSample) -> None: self._poses.append(pose)
    def latest_pose(self) -> PoseSample | None: return self._poses[-1] if self._poses else None
