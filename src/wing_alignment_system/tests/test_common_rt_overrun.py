import threading
import time

from wing_alignment_system.common_rt import FixedRateLoop


def test_fixed_rate_loop_reports_overrun():
    seen = []
    ready = threading.Event()

    def tick():
        time.sleep(0.01)

    def on_overrun(name, tick_sec, overrun_sec, count):
        seen.append((name, tick_sec, overrun_sec, count))
        ready.set()

    loop = FixedRateLoop(
        name="test_rt",
        hz=500.0,
        tick_fn=tick,
        on_overrun=on_overrun,
    )
    loop.start()
    try:
        assert ready.wait(0.5)
    finally:
        loop.stop()

    assert seen
    name, tick_sec, overrun_sec, count = seen[0]
    assert name == "test_rt"
    assert tick_sec > 0.0
    assert overrun_sec > 0.0
    assert count >= 1
