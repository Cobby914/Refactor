"""Join sensor callbacks to confirmed CARLA frames within a bounded window."""

from collections import Counter
from dataclasses import dataclass
from threading import Lock
from typing import Any

from .simulation_engine import SimulationFrame


@dataclass(frozen=True)
class JoinedFrame:
    simulation: SimulationFrame
    actors: list[dict]
    measurements: dict[str, Any]
    missing: tuple[str, ...]
    status: str
    segment: int | None = None


class FrameBuffer:
    def __init__(self, first_frame: int, sensors: tuple[str, ...], *, max_lag_frames: int, segment=None):
        if max_lag_frames < 1 or not sensors or len(sensors) != len(set(sensors)):
            raise ValueError("frame buffer needs a positive window and unique sensor labels")
        self._first = self._next = first_frame
        self._latest = first_frame - 1
        self._sensor_labels = sensors
        self._sensor_set = set(sensors)
        self._max_lag = max_lag_frames
        self._segment = segment
        self._truth: dict[int, tuple[SimulationFrame, list[dict]]] = {}
        self._samples: dict[int, dict[str, Any]] = {}
        self._lock = Lock()
        self._accepting_frames = True
        self._closed = False
        self._statuses: Counter = Counter()
        self._ignored: Counter = Counter()
        self._received: Counter = Counter()
        self._max_arrival_lag: Counter = Counter()

    def offer(self, sensor_label: str, frame: int, measurement) -> None:
        """Call on a CARLA streaming thread. Keep the first sample per sensor and frame."""
        with self._lock:
            if self._closed:
                return
            if sensor_label not in self._sensor_set:
                self._ignored["unknown_sensor"] += 1
            elif frame < self._first:
                self._ignored["before_recording"] += 1
            elif frame < self._next:
                self._ignored["late"] += 1
            elif frame > self._latest + int(self._accepting_frames):
                self._ignored["outside_recording"] += 1
            else:
                samples = self._samples.setdefault(frame, {})
                if sensor_label in samples:
                    self._ignored["duplicate"] += 1
                else:
                    samples[sensor_label] = measurement
                    self._max_arrival_lag[sensor_label] = max(
                        self._max_arrival_lag[sensor_label], self._latest - frame,
                    )

    def confirm(self, frame: SimulationFrame, actors: list[dict]) -> list[JoinedFrame]:
        """Register an exact snapshot after a successful paired simulator step."""
        with self._lock:
            if not self._accepting_frames or frame.carla_frame != self._latest + 1:
                raise ValueError(f"unexpected confirmed CARLA frame {frame.carla_frame}")
            self._latest = frame.carla_frame
            self._truth[self._latest] = (frame, actors)
            return self._collect(final=False)

    @property
    def pending(self) -> bool:
        with self._lock:
            return bool(self._truth)

    def end_recording(self) -> None:
        with self._lock:
            self._accepting_frames = False
            self._samples.pop(self._latest + 1, None)

    def pop_ready(self) -> list[JoinedFrame]:
        with self._lock:
            return self._collect(final=False)

    def finish(self) -> list[JoinedFrame]:
        with self._lock:
            self._accepting_frames = False
            self._closed = True
            frames = self._collect(final=True)
            self._samples.clear()
            return frames

    def stats(self) -> dict:
        with self._lock:
            return {
                "first_frame": self._first if self._latest >= self._first else None,
                "last_frame": self._latest if self._latest >= self._first else None,
                "frames": dict(self._statuses),
                "received": {label: self._received[label] for label in self._sensor_labels},
                "ignored": dict(self._ignored),
                "max_arrival_lag_frames": dict(self._max_arrival_lag),
            }

    def _collect(self, *, final: bool) -> list[JoinedFrame]:
        frames = []
        while self._next <= self._latest:
            samples = self._samples.get(self._next, {})
            missing = tuple(label for label in self._sensor_labels if label not in samples)
            if not missing:
                status = "complete"
            elif self._latest - self._next >= self._max_lag:
                status = "expired"
            elif final:
                status = "shutdown"
            else:
                break
            frame, actors = self._truth.pop(self._next)
            self._samples.pop(self._next, None)
            ordered = {label: samples[label] for label in self._sensor_labels if label in samples}
            frames.append(JoinedFrame(frame, actors, ordered, missing, status, self._segment))
            self._statuses[status] += 1
            self._received.update(ordered.keys())
            self._next += 1
        return frames
