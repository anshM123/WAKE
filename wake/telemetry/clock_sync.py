"""NTP-like conversion from the XIAO boot clock to host monotonic time."""
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import json
import math
import socket
import time
import numpy as np


@dataclass(frozen=True)
class ClockExchange:
    sequence: int
    host_t1_ns: int
    xiao_t2_us: int
    xiao_t3_us: int
    host_t4_ns: int

    @property
    def rtt_ns(self) -> float:
        processing_ns = (self.xiao_t3_us - self.xiao_t2_us) * 1000
        return max(0.0, (self.host_t4_ns - self.host_t1_ns) - processing_ns)

    @property
    def host_midpoint_ns(self) -> float:
        return (self.host_t1_ns + self.host_t4_ns) / 2.0

    @property
    def xiao_midpoint_ns(self) -> float:
        return (self.xiao_t2_us + self.xiao_t3_us) * 500.0


@dataclass(frozen=True)
class ClockStatus:
    rtt_ms: float
    offset_ns: float
    skew_ppm: float
    model_age_ms: float
    residual_ms: float
    confidence: float
    sample_count: int


class ClockModel:
    """Robust affine clock model fitted to the best rolling RTT samples."""

    def __init__(
        self,
        *,
        max_samples: int = 60,
        max_rtt_ms: float = 30.0,
        stale_after_ms: float = 3000.0,
        minimum_samples: int = 4,
    ) -> None:
        self.samples: deque[ClockExchange] = deque(maxlen=max_samples)
        self.max_rtt_ns = max_rtt_ms * 1e6
        self.stale_after_ns = stale_after_ms * 1e6
        self.minimum_samples = minimum_samples
        self.scale = 1.0
        self.offset_ns = 0.0
        self.residual_ns = math.inf
        self.last_update_ns: int | None = None
        self.best_rtt_ns = math.inf

    def add(self, exchange: ClockExchange) -> bool:
        if exchange.rtt_ns > self.max_rtt_ns:
            return False
        if exchange.xiao_t3_us < exchange.xiao_t2_us:
            return False
        self.samples.append(exchange)
        self._fit()
        self.last_update_ns = exchange.host_t4_ns
        return True

    def _fit(self) -> None:
        ordered = sorted(self.samples, key=lambda sample: sample.rtt_ns)
        keep_count = max(2, math.ceil(len(ordered) * 0.5))
        selected = ordered[:keep_count]
        x = np.asarray([sample.xiao_midpoint_ns for sample in selected])
        y = np.asarray([sample.host_midpoint_ns for sample in selected])
        weights = 1.0 / np.maximum(
            np.asarray([sample.rtt_ns for sample in selected]), 100_000.0
        )
        x_origin = float(x.mean())
        y_origin = float(y.mean())
        centered_x = x - x_origin
        denominator = float(np.sum(weights * centered_x * centered_x))
        if denominator > 0:
            self.scale = float(
                np.sum(weights * centered_x * (y - y_origin)) / denominator
            )
        self.offset_ns = y_origin - self.scale * x_origin
        residuals = y - (self.scale * x + self.offset_ns)
        self.residual_ns = float(np.sqrt(np.average(residuals**2, weights=weights)))
        self.best_rtt_ns = float(min(sample.rtt_ns for sample in selected))

    def to_host_ns(self, xiao_timestamp_us: int, now_ns: int | None = None) -> int:
        if not self.healthy(now_ns=now_ns):
            raise RuntimeError("CLOCK_UNSYNCED")
        return round(self.scale * xiao_timestamp_us * 1000 + self.offset_ns)

    def healthy(self, now_ns: int | None = None) -> bool:
        if len(self.samples) < self.minimum_samples or self.last_update_ns is None:
            return False
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        return now_ns - self.last_update_ns <= self.stale_after_ns

    def status(self, now_ns: int | None = None) -> ClockStatus:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        age_ns = math.inf if self.last_update_ns is None else now_ns - self.last_update_ns
        sample_factor = min(1.0, len(self.samples) / max(1, self.minimum_samples * 2))
        rtt_factor = max(0.0, 1.0 - self.best_rtt_ns / self.max_rtt_ns)
        residual_factor = max(0.0, 1.0 - self.residual_ns / 5e6)
        freshness = 0.0 if not math.isfinite(age_ns) else max(0.0, 1.0 - age_ns / self.stale_after_ns)
        confidence = sample_factor * rtt_factor * residual_factor * freshness
        return ClockStatus(
            rtt_ms=self.best_rtt_ns / 1e6,
            offset_ns=self.offset_ns,
            skew_ppm=(self.scale - 1.0) * 1e6,
            model_age_ms=age_ns / 1e6,
            residual_ms=self.residual_ns / 1e6,
            confidence=confidence,
            sample_count=len(self.samples),
        )


class ClockSynchronizer:
    """UDP client that periodically updates a :class:`ClockModel`."""

    def __init__(self, host: str, port: int = 5008, model: ClockModel | None = None) -> None:
        self.address = (host, port)
        self.model = model or ClockModel()
        self.sequence = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(0.25)

    def exchange_once(self) -> bool:
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        t1 = time.monotonic_ns()
        request = {
            "type": "clock_sync_request",
            "sequence": self.sequence,
            "host_t1_ns": t1,
        }
        self.socket.sendto(json.dumps(request).encode("utf-8"), self.address)
        try:
            payload, _ = self.socket.recvfrom(1024)
        except socket.timeout:
            return False
        t4 = time.monotonic_ns()
        response = json.loads(payload.decode("utf-8"))
        if response.get("type") != "clock_sync_response":
            return False
        if int(response.get("sequence", -1)) != self.sequence:
            return False
        if int(response.get("host_t1_ns", -1)) != t1:
            return False
        return self.model.add(
            ClockExchange(
                self.sequence,
                t1,
                int(response["xiao_t2_us"]),
                int(response["xiao_t3_us"]),
                t4,
            )
        )

    def close(self) -> None:
        self.socket.close()
