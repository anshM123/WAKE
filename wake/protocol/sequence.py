from dataclasses import dataclass

UINT32_MOD = 1 << 32

@dataclass
class SequenceUpdate:
    lost: int = 0
    reordered: bool = False
    duplicate: bool = False

class SequenceTracker:
    def __init__(self) -> None: self.last: int | None = None
    def update(self, sequence: int) -> SequenceUpdate:
        if not 0 <= sequence < UINT32_MOD: raise ValueError("sequence must be uint32")
        if self.last is None: self.last = sequence; return SequenceUpdate()
        delta = (sequence - self.last) % UINT32_MOD
        if delta == 0: return SequenceUpdate(duplicate=True)
        if delta < UINT32_MOD // 2:
            self.last = sequence
            return SequenceUpdate(lost=max(0, delta - 1))
        return SequenceUpdate(reordered=True)
