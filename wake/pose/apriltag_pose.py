"""Optional AprilTag provider boundary.

OpenCV/pupil-apriltags integration is deliberately isolated here. Camera
intrinsics, tag size/family, quality gates, and transforms are all mandatory
configuration. Importing WAKE does not require vision dependencies.
"""
from dataclasses import dataclass
from wake.types import PoseSample

@dataclass
class AprilTagQualityGate:
    tag_id: int
    max_reprojection_error_px: float
    min_decision_margin: float
    max_pose_jump_m: float

    def accepts(self, *, tag_id: int, reprojection_error: float, decision_margin: float) -> bool:
        return tag_id == self.tag_id and reprojection_error <= self.max_reprojection_error_px and decision_margin >= self.min_decision_margin

class AprilTagPoseProvider:
    """Prepared provider; call ``update`` from a configured detector loop."""
    def __init__(self, gate: AprilTagQualityGate) -> None: self.gate = gate; self._latest: PoseSample | None = None
    def update(self, pose: PoseSample, decision_margin: float) -> bool:
        if not self.gate.accepts(tag_id=pose.tag_id, reprojection_error=pose.reprojection_error, decision_margin=decision_margin): return False
        self._latest = pose; return True
    def latest_pose(self) -> PoseSample | None: return self._latest
