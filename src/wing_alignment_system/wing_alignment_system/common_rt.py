# -*- coding: utf-8 -*-

import queue
import threading
import time
from typing import Callable, Generic, List, Optional, TypeVar


T = TypeVar('T')


class FixedRateLoop:
    def __init__(
        self,
        name: str,
        hz: float,
        tick_fn: Callable[[], None],
        on_error: Optional[Callable[[BaseException], None]] = None,
        on_overrun: Optional[Callable[[str, float, float, int], None]] = None,
    ):
        self.name = str(name or 'fixed_rate_loop')
        self.hz = max(1e-3, float(hz))
        self.period_sec = 1.0 / self.hz
        self._tick_fn = tick_fn
        self._on_error = on_error
        self._on_overrun = on_overrun
        self._overrun_count = 0
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, name=self.name, daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 1.0):
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, float(timeout)))
        self._thread = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self):
        next_deadline = time.perf_counter()
        while not self._stop_evt.is_set():
            tick_start = time.perf_counter()
            try:
                self._tick_fn()
            except BaseException as exc:  # pragma: no cover - defensive thread boundary
                if self._on_error is not None:
                    self._on_error(exc)
                self._stop_evt.set()
                break

            next_deadline += self.period_sec
            now = time.perf_counter()
            tick_sec = now - tick_start
            sleep_sec = next_deadline - now
            if sleep_sec > 0.0:
                self._stop_evt.wait(sleep_sec)
            else:
                self._overrun_count += 1
                if self._on_overrun is not None:
                    try:
                        self._on_overrun(self.name, tick_sec, -sleep_sec, self._overrun_count)
                    except BaseException:
                        pass
                # When we overrun, snap to "now" to avoid unbounded catch-up.
                next_deadline = time.perf_counter()


class LatestValueBuffer(Generic[T]):
    def __init__(self, initial: Optional[T] = None):
        self._lock = threading.Lock()
        self._value: Optional[T] = initial

    def set(self, value: Optional[T]):
        with self._lock:
            self._value = value

    def get(self) -> Optional[T]:
        with self._lock:
            return self._value

    def pop(self) -> Optional[T]:
        with self._lock:
            value = self._value
            self._value = None
            return value

    def clear(self):
        with self._lock:
            self._value = None


class EventQueue(Generic[T]):
    def __init__(self, maxsize: int = 0):
        self._queue: queue.Queue[T] = queue.Queue(maxsize=max(0, int(maxsize)))

    def put(self, item: T) -> bool:
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            return False

    def drain(self) -> List[T]:
        out: List[T] = []
        while True:
            try:
                out.append(self._queue.get_nowait())
            except queue.Empty:
                return out
