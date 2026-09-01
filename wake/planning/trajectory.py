"""Known-safe trajectory primitives used for conservative return paths."""
from wake.types import PoseSample
def safe_return_path(trajectory:list[PoseSample])->list[PoseSample]:return list(reversed(trajectory))
