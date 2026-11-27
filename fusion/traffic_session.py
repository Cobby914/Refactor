"""Run headless SUMO scenarios with traffic mirrored into an idle CARLA world."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
import json
from math import ceil, isclose, isfinite
from pathlib import Path
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
from threading import Event
from typing import TYPE_CHECKING

from .carla_adapter import open_carla_session
from .simulation_engine import SimulationEngine, SimulationFrame
from .sumo_adapter import open_sumo_session

if TYPE_CHECKING:
    from .capture_session import CaptureOptions


@dataclass(frozen=True)
class TrafficRun:
    final_frame: SimulationFrame
    peak_sumo_vehicles: int
    peak_carla_actors: int
    spawn_collisions: int
    sumo_pid: int
    sumo_version: str
    carla_version: str


class StopRequested(KeyboardInterrupt):
    """Stop at a simulator boundary and finish the confirmed capture."""


def check_stop(stop_event):
    if stop_event is not None and stop_event.is_set():
        raise StopRequested("stop requested")


@contextmanager
def cooperative_stop():
    """Translate CLI signals into a request checked between paired steps."""
    event = Event()
    previous = {}
    try:
        for name in ("SIGINT", "SIGTERM", "SIGBREAK"):
            signum = getattr(signal, name, None)
            if signum is not None:
                previous[signum] = signal.signal(signum, lambda *_: event.set())
        yield event
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def build_scenario_config(directory, *, scenario, density, duration, direction,
                          ambient_vehicles, pedestrians, bicycles, seed, step_length):
    from traffic.validate_scenarios import ambient_traffic, build_files, runner

    config, [route_file, *_] = build_files(
        scenario, density, duration, direction, ambient_vehicles, pedestrians,
        bicycles, seed, step_length, temp_dir=directory,
    )
    violations = ambient_traffic.check_spawn_exclusion(
        runner.NET_FILE, Path(route_file).read_text(encoding="utf-8"),
    )
    if violations:
        raise ValueError("spawn/despawn inside the radar zone: " + ", ".join(violations))
    return Path(config)


@contextmanager
def open_row_sumo(config_path, step_length_s, **kwargs):
    with open_sumo_session(config_path, **kwargs) as sumo:
        if not isclose(sumo.fixed_delta_s, step_length_s, rel_tol=0, abs_tol=1e-9):
            raise RuntimeError(
                f"SUMO effective step {sumo.fixed_delta_s:g}s differs from requested "
                f"{step_length_s:g}s. Use a step length representable in whole milliseconds."
            )
        yield sumo


class TrafficRow:
    def __init__(self, carla, sumo, controller, mirror, lights):
        self.carla = carla
        self.sumo = sumo
        self.controller = controller
        self.mirror = mirror
        self.lights = lights
        self.peak_sumo_vehicles = 0
        self.peak_carla_actors = 0

    def make_engine(self):
        return SimulationEngine(
            fixed_delta_s=self.sumo.fixed_delta_s,
            initial_sumo_time_s=self.sumo.initial_time_s,
            initial_carla_frame=self.carla.world.get_snapshot().frame,
            advance_sumo=self.sumo.advance,
            sync_traffic=self._synchronize,
            tick_carla=self._tick,
        )

    def _synchronize(self):
        self.controller.step()
        self.peak_sumo_vehicles = max(self.peak_sumo_vehicles, self.mirror.step())
        self.peak_carla_actors = max(self.peak_carla_actors, len(self.mirror.actor_ids))
        self.lights.step()

    def _tick(self):
        frame = self.carla.tick()
        self.lights.verify()
        return frame

    def record(self, capture, frame):
        capture.record(frame, self.carla.world.get_snapshot(), self.mirror.actor_ids.values())

    def result(self, frame):
        return TrafficRun(
            frame, self.peak_sumo_vehicles, self.peak_carla_actors, self.mirror.spawn_collisions,
            self.sumo.pid, self.sumo.version, self.carla.version,
        )


@contextmanager
def open_traffic_row(carla, sumo, *, scenario, duration, seed, render_radius, stretch_signals):
    from traffic.runner import NET_FILE, SCENARIOS, get_controller, spawn_buses_with_stop
    from traffic.stretch_signals import apply_corridor_priority

    from .actor_mirror import ActorMirror
    from .traffic_light_mirror import TrafficLightMirror

    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario {scenario}")
    if render_radius is not None and (not isfinite(render_radius) or render_radius <= 0):
        raise ValueError("render radius must be finite and positive")
    if stretch_signals not in ("green", "static"):
        raise ValueError("stretch signals must be green or static")

    connection = sumo.connection
    with ExitStack() as resources:
        if stretch_signals == "green":
            apply_corridor_priority(NET_FILE, connection=connection, verbose=False)
        controller = get_controller(scenario, duration, connection=connection)
        resources.callback(controller.on_stop)
        controller.on_start()
        if scenario == 5:
            spawn_buses_with_stop(duration, connection=connection)

        lights = TrafficLightMirror(carla, connection)
        resources.callback(lights.close)
        lights.start()
        mirror = ActorMirror(carla, connection, seed=seed, render_radius=render_radius)
        resources.callback(mirror.close)
        yield TrafficRow(carla, sumo, controller, mirror, lights)


def prepare_capture(carla, capture, stop_event=None):
    from .capture_session import WARMUP_TICKS

    for _ in range(WARMUP_TICKS):
        check_stop(stop_event)
        carla.tick()
    capture.prepare(carla.world.get_snapshot())


def run_offline_steps(directory, *, label, postprocess, on_state=None):
    if not label:
        return

    capture_scripts = Path(__file__).resolve().parents[1] / "dataset" / "capture"
    if on_state is not None:
        on_state("labeling")
    subprocess.run([
        sys.executable, "-B", str(capture_scripts / "LabelRadarCapture.py"),
        "--capture-dir", str(directory),
    ], check=True)
    if postprocess:
        if on_state is not None:
            on_state("postprocessing")
        meta = json.loads((directory / "run_meta.json").read_text(encoding="utf-8"))
        subprocess.run([
            sys.executable, "-B", str(capture_scripts / "PostProcessDataset.py"),
            "--capture-dir", str(directory), "--seed", str(meta["postprocess_seed"]),
        ], check=True)


def run_traffic(
    config_path: Path,
    *,
    steps: int,
    step_length_s: float = 0.05,
    carla_host: str = "127.0.0.1",
    carla_port: int = 2000,
    ready_timeout_s: float = 30.0,
    rpc_timeout_s: float = 5.0,
    scenario: int = 1,
    seed: int = 42,
    render_radius: float | None = 150.0,
    stretch_signals: str = "green",
    capture_options: CaptureOptions | None = None,
    on_frame: Callable[[SimulationFrame], None] | None = None,
    stop_event: Event | None = None,
) -> TrafficRun:
    """Advance a scenario and verify its traffic and signals before publishing frames."""
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

    def stop_check():
        check_stop(stop_event)

    timeouts = {
        "ready_timeout_s": ready_timeout_s, "rpc_timeout_s": rpc_timeout_s, "stop_check": stop_check,
    }
    with ExitStack() as resources:
        sumo = resources.enter_context(open_row_sumo(config_path, step_length_s, **timeouts))
        carla = resources.enter_context(open_carla_session(
            carla_host, carla_port, sumo.fixed_delta_s, **timeouts,
        ))
        row = resources.enter_context(open_traffic_row(
            carla, sumo, scenario=scenario, duration=ceil(steps * step_length_s),
            seed=seed, render_radius=render_radius, stretch_signals=stretch_signals,
        ))
        capture = None
        if capture_options is not None:
            from .capture_session import open_capture

            capture = resources.enter_context(open_capture(
                carla, capture_options, scenario=scenario, seed=seed, sumo_version=sumo.version,
            ))
            prepare_capture(carla, capture, stop_event)
            capture.start_window(carla.world.get_snapshot().frame + 1)
        engine = row.make_engine()
        for _ in range(steps):
            stop_check()
            frame = engine.step()
            if capture is not None:
                row.record(capture, frame)
            if on_frame is not None:
                on_frame(frame)
        if capture is not None:
            capture.drain_window(carla.tick)
            capture.finish_window()
        return row.result(frame)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--step-length", type=float, default=0.05)
    parser.add_argument("--ready-timeout", type=float, default=30.0)
    parser.add_argument("--rpc-timeout", type=float, default=5.0)
    parser.add_argument("--scenario", type=int, choices=range(1, 12), default=1)
    parser.add_argument("--density", type=int, default=60)
    parser.add_argument("--direction", choices=("WB", "EB", "BOTH"), default="BOTH")
    parser.add_argument("--ambient-vehicles", type=int, default=0)
    parser.add_argument("--pedestrians", type=int, default=0)
    parser.add_argument("--bicycles", type=int, default=0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--render-radius", type=float, default=150.0)
    parser.add_argument("--no-cull", action="store_true")
    parser.add_argument("--stretch-signals", choices=("green", "static"), default="green")
    parser.add_argument("--capture-dir", type=Path, help="write a v2 capture into a new directory")
    parser.add_argument("--radars", type=int, default=8)
    parser.add_argument("--radars-south", type=int)
    parser.add_argument("--radars-north", type=int)
    parser.add_argument("--radar-height", type=float, default=3.0)
    parser.add_argument("--radar-pps", type=int, default=0)
    parser.add_argument("--label", action="store_true", help="run legacy offline radar labeling")
    parser.add_argument("--postprocess", action="store_true", help="run labeling and legacy postprocessing")
    args = parser.parse_args()
    if not 1 <= args.density <= 100:
        parser.error("--density must be between 1 and 100")
    if min(args.ambient_vehicles, args.pedestrians, args.bicycles) < 0:
        parser.error("ambient actor counts must be nonnegative")
    if (args.label or args.postprocess) and args.capture_dir is None:
        parser.error("--label and --postprocess require --capture-dir")
    try:
        capture_options = None
        if args.capture_dir is not None:
            from .capture_session import CaptureOptions
            from .config import split_radar_count

            if args.radars < 1:
                raise ValueError("--radars must be positive")
            south, north = split_radar_count(args.radars)
            capture_options = CaptureOptions(
                args.capture_dir.expanduser().resolve(),
                south if args.radars_south is None else args.radars_south,
                north if args.radars_north is None else args.radars_north,
                args.radar_height, args.radar_pps,
            )
        with TemporaryDirectory(prefix="scalesense-v2-") as temp_dir, cooperative_stop() as stop_event:
            config = build_scenario_config(
                temp_dir, scenario=args.scenario, density=args.density,
                duration=ceil(args.steps * args.step_length), direction=args.direction,
                ambient_vehicles=args.ambient_vehicles, pedestrians=args.pedestrians,
                bicycles=args.bicycles, seed=args.seed, step_length=args.step_length,
            )
            run = run_traffic(
                Path(config), steps=args.steps, step_length_s=args.step_length,
                carla_host=args.carla_host, carla_port=args.carla_port,
                ready_timeout_s=args.ready_timeout, rpc_timeout_s=args.rpc_timeout,
                scenario=args.scenario, seed=args.seed,
                render_radius=None if args.no_cull else args.render_radius,
                stretch_signals=args.stretch_signals,
                capture_options=capture_options, stop_event=stop_event,
            )
        if capture_options is not None:
            directory = capture_options.directory
            meta = json.loads((directory / "run_meta.json").read_text(encoding="utf-8"))
            print(f"Capture {meta['capture_status']} in {directory}", flush=True)
            run_offline_steps(directory, label=args.label or args.postprocess, postprocess=args.postprocess)
    except KeyboardInterrupt:
        print("Traffic session interrupted.", file=sys.stderr)
        return 130
    except (ImportError, OSError, RuntimeError, ValueError, OverflowError, subprocess.CalledProcessError) as exc:
        print(f"Traffic session failed: {exc}", file=sys.stderr)
        return 1

    frame = run.final_frame
    print(f"CARLA {run.carla_version}, {run.sumo_version} (PID {run.sumo_pid})")
    print(
        f"Paired {frame.step} steps through SUMO t={frame.sumo_time_s:g}s and CARLA "
        f"frame {frame.carla_frame}. Peak SUMO vehicles: {run.peak_sumo_vehicles}. "
        f"Peak CARLA mirrors: {run.peak_carla_actors}."
    )
    if run.spawn_collisions:
        print(f"Deferred {run.spawn_collisions} actor spawns because of collision.")
    return 0



if __name__ == "__main__":
    raise SystemExit(main())
