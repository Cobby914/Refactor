"""Run the v2 clocks against headless SUMO traffic and an idle CARLA world."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
from math import ceil, isclose, isfinite
from pathlib import Path
import signal
import sys
from tempfile import TemporaryDirectory

from .carla_adapter import open_carla_session
from .simulation_engine import SimulationEngine, SimulationFrame
from .sumo_adapter import open_sumo_session


@dataclass(frozen=True)
class TrafficRun:
    final_frame: SimulationFrame
    peak_sumo_vehicles: int
    sumo_pid: int
    sumo_version: str
    carla_version: str


def run_traffic(
    config_path: Path,
    *,
    steps: int,
    step_length_s: float = 0.05,
    carla_host: str = "127.0.0.1",
    carla_port: int = 2000,
    ready_timeout_s: float = 30.0,
    rpc_timeout_s: float = 5.0,
    on_frame: Callable[[SimulationFrame], None] | None = None,
) -> TrafficRun:
    """Pair SUMO traffic steps with CARLA frames and release both on exit."""
    if type(steps) is not int or steps < 1:
        raise ValueError("steps must be a positive integer")
    if not 1 <= carla_port <= 65535:
        raise ValueError("CARLA port must be between 1 and 65535")
    for name, value in (
        ("step length", step_length_s), ("ready timeout", ready_timeout_s),
        ("RPC timeout", rpc_timeout_s),
    ):
        if not isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")

    timeouts = {"ready_timeout_s": ready_timeout_s, "rpc_timeout_s": rpc_timeout_s}
    with open_sumo_session(config_path, **timeouts) as sumo:
        if not isclose(sumo.fixed_delta_s, step_length_s, rel_tol=0, abs_tol=1e-9):
            raise RuntimeError(
                f"SUMO effective step {sumo.fixed_delta_s:g}s differs from requested "
                f"{step_length_s:g}s. Use a step length representable in whole milliseconds."
            )
        with open_carla_session(carla_host, carla_port, sumo.fixed_delta_s, **timeouts) as carla:
            peak_sumo_vehicles = 0

            def observe_traffic():
                nonlocal peak_sumo_vehicles
                peak_sumo_vehicles = max(
                    peak_sumo_vehicles, sumo.connection.vehicle.getIDCount()
                )

            engine = SimulationEngine(
                fixed_delta_s=sumo.fixed_delta_s,
                initial_sumo_time_s=sumo.initial_time_s,
                initial_carla_frame=carla.initial_frame,
                advance_sumo=sumo.advance,
                sync_traffic=observe_traffic,
                tick_carla=carla.tick,
            )
            for _ in range(steps):
                frame = engine.step()
                if on_frame is not None:
                    on_frame(frame)
            return TrafficRun(frame, peak_sumo_vehicles, sumo.pid, sumo.version, carla.version)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--step-length", type=float, default=0.05)
    parser.add_argument("--ready-timeout", type=float, default=30.0)
    parser.add_argument("--rpc-timeout", type=float, default=5.0)
    args = parser.parse_args()
    try:
        from traffic.validate_scenarios import build_files

        with TemporaryDirectory(prefix="scalesense-v2-") as temp_dir:
            config, _ = build_files(
                scenario_id=1, density=60, duration=ceil(args.steps * args.step_length),
                direction="BOTH", ambient_vehicles=0, pedestrians=0, bicycles=0,
                seed=42, step_length=args.step_length, temp_dir=temp_dir,
            )
            run = run_traffic(
                Path(config), steps=args.steps, step_length_s=args.step_length,
                carla_host=args.carla_host, carla_port=args.carla_port,
                ready_timeout_s=args.ready_timeout, rpc_timeout_s=args.rpc_timeout,
            )
    except KeyboardInterrupt:
        print("Traffic session interrupted.", file=sys.stderr)
        return 130
    except (ImportError, OSError, RuntimeError, ValueError, OverflowError) as exc:
        print(f"Traffic session failed: {exc}", file=sys.stderr)
        return 1

    frame = run.final_frame
    print(f"CARLA {run.carla_version}, {run.sumo_version} (PID {run.sumo_pid})")
    print(
        f"Paired {frame.step} steps through SUMO t={frame.sumo_time_s:g}s and CARLA "
        f"frame {frame.carla_frame}. Peak SUMO vehicles: {run.peak_sumo_vehicles}."
    )
    return 0


def _interrupt(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, _interrupt)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _interrupt)
    raise SystemExit(main())
