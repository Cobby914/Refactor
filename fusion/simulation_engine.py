"""Ordered simulator stepping for the v2 fusion path."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from math import isclose, isfinite
from threading import Lock


@dataclass(frozen=True)
class SimulationFrame:
    """Identity of a completed SUMO–CARLA step."""

    step: int
    sumo_time_s: float
    carla_frame: int


class SimulationStepError(RuntimeError):
    """The requested step could not complete."""


class SimulationEngine:
    """Advance both simulators through one coordinator.

    The session supplies baselines after initial setup. The SUMO operation
    returns its actual time after advancing once. The traffic operation returns
    when scenario and mirror updates have been applied. The CARLA operation
    advances the synchronous world once and returns the resulting frame ID.

    A failed or interrupted operation may already have changed simulator state.
    The caller must close that session and create a new engine after recovery.
    """

    def __init__(
        self,
        *,
        fixed_delta_s: float,
        initial_sumo_time_s: float,
        initial_carla_frame: int,
        advance_sumo: Callable[[], float],
        sync_traffic: Callable[[], None],
        tick_carla: Callable[[], int],
    ) -> None:
        if not isfinite(fixed_delta_s) or fixed_delta_s <= 0:
            raise ValueError("fixed_delta_s must be finite and positive")
        if not isfinite(initial_sumo_time_s):
            raise ValueError("initial_sumo_time_s must be finite")
        if type(initial_carla_frame) is not int or initial_carla_frame < 0:
            raise ValueError("initial_carla_frame must be a nonnegative integer")

        self._fixed_delta_s = fixed_delta_s
        self._initial_sumo_time_s = initial_sumo_time_s
        self._initial_carla_frame = initial_carla_frame
        self._advance_sumo = advance_sumo
        self._sync_traffic = sync_traffic
        self._tick_carla = tick_carla
        self._completed_steps = 0
        self._incomplete_stage: str | None = None
        self._step_lock = Lock()

    def step(self) -> SimulationFrame:
        """Return a paired frame after advancing SUMO, traffic, and CARLA."""
        if not self._step_lock.acquire(blocking=False):
            raise SimulationStepError("a co-simulation step is already running")
        try:
            step = self._completed_steps + 1
            if self._incomplete_stage is not None:
                raise SimulationStepError(
                    f"session stopped at step {step} during {self._incomplete_stage}; "
                    "start a new session to resume"
                )

            try:
                self._incomplete_stage = "SUMO advance"
                sumo_time_s = self._advance_sumo()
                expected_time_s = self._initial_sumo_time_s + step * self._fixed_delta_s
                tolerance_s = min(1e-6, self._fixed_delta_s / 2)
                if not isclose(sumo_time_s, expected_time_s, rel_tol=0, abs_tol=tolerance_s):
                    raise ValueError(f"expected SUMO time {expected_time_s:g} s, got {sumo_time_s:g} s")

                self._incomplete_stage = "traffic synchronization"
                self._sync_traffic()

                self._incomplete_stage = "CARLA tick"
                carla_frame = self._tick_carla()
                expected_frame = self._initial_carla_frame + step
                if type(carla_frame) is not int or carla_frame != expected_frame:
                    raise ValueError(f"expected CARLA frame {expected_frame}, got {carla_frame!r}")

                frame = SimulationFrame(step, sumo_time_s, carla_frame)
            except Exception as exc:
                raise SimulationStepError(
                    f"step {step} failed during {self._incomplete_stage}: {exc}"
                ) from exc

            self._completed_steps = step
            self._incomplete_stage = None
            return frame
        finally:
            self._step_lock.release()
