from __future__ import annotations
from bisect import bisect_left
from collections import deque
from typing import Callable, Generic, TypeVar
T = TypeVar("T")

class TimestampBuffer(Generic[T]):
    def __init__(self, timestamp: Callable[[T], int], capacity: int = 2048) -> None: self.timestamp=timestamp; self.capacity=capacity; self._values: deque[T]=deque()
    def add(self, value: T) -> None:
        values=list(self._values); index=bisect_left([self.timestamp(v) for v in values], self.timestamp(value)); values.insert(index,value); self._values=deque(values[-self.capacity:])
    def bracket(self, timestamp_ns: int) -> tuple[T, T] | None:
        values=list(self._values); times=[self.timestamp(v) for v in values]; i=bisect_left(times,timestamp_ns)
        return None if i == 0 or i == len(values) else (values[i-1],values[i])
    def __len__(self) -> int: return len(self._values)
