import pytest
from wake.telemetry.clock_sync import ClockExchange, ClockModel


def exchange(sequence, xiao_us, *, offset_ns=2_000_000_000, scale=1.0, rtt_ns=2_000_000):
    xiao_ns = xiao_us * 1000
    host_midpoint = round(scale * xiao_ns + offset_ns)
    return ClockExchange(
        sequence,
        host_midpoint - rtt_ns // 2,
        xiao_us,
        xiao_us,
        host_midpoint + rtt_ns // 2,
    )


def test_constant_offset_conversion():
    model = ClockModel(minimum_samples=3)
    for index in range(6):
        model.add(exchange(index, 1_000_000 + index * 100_000))
    assert model.to_host_ns(2_000_000, now_ns=model.last_update_ns) == pytest.approx(4_000_000_000, abs=10)


def test_known_skew():
    model = ClockModel(minimum_samples=3)
    scale = 1.0 + 75e-6
    for index in range(8):
        model.add(exchange(index, 1_000_000 + index * 200_000, scale=scale))
    assert model.status(now_ns=model.last_update_ns).skew_ppm == pytest.approx(75, abs=.1)


def test_high_rtt_rejected_and_jitter_robust():
    model = ClockModel(minimum_samples=3, max_rtt_ms=20)
    assert not model.add(exchange(0, 1_000_000, rtt_ns=50_000_000))
    for index, rtt in enumerate([2_000_000, 9_000_000, 1_000_000, 12_000_000, 2_000_000]):
        assert model.add(exchange(index + 1, 1_000_000 + index * 100_000, rtt_ns=rtt))
    assert model.status(now_ns=model.last_update_ns).residual_ms < .001


def test_stale_model_rejected():
    model = ClockModel(minimum_samples=2, stale_after_ms=100)
    model.add(exchange(0, 1_000_000))
    model.add(exchange(1, 1_100_000))
    with pytest.raises(RuntimeError, match="CLOCK_UNSYNCED"):
        model.to_host_ns(1_200_000, now_ns=model.last_update_ns + 101_000_000)
