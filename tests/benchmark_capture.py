"""Measure one real legacy or v2 campaign on an isolated CARLA server."""

import argparse
from collections import Counter, defaultdict
from contextlib import ExitStack
import csv
from functools import wraps
import importlib
from inspect import signature
import json
import os
from pathlib import Path
import platform
from statistics import median, quantiles
import subprocess
import sys
from time import perf_counter
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def write_json(path, obj):
    path.write_text(json.dumps(obj, indent=2), encoding="utf-8")


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


class TimedRadarLock:
    """Measure the legacy helper's CSV serialization and buffered writes once per scan."""

    def __init__(self, lock):
        self.lock = lock

    def __enter__(self):
        self.before_write = perf_counter()
        return self.lock.__enter__()

    def __exit__(self, *error):
        return self.lock.__exit__(*error)


class Trace:
    def __init__(self, stack):
        self.stack = stack
        self.samples = defaultdict(list)
        self.last_tick = 0
        self.backlogs = {}

    def wrap(self, owner, name, observe):
        measured = self.measure(getattr(owner, name), observe)
        self.stack.enter_context(patch.object(owner, name, measured))

    def profile_mirror(self, mirror):
        from traci import constants
        from traci.domain import Domain

        pose_variables = (constants.VAR_POSITION, constants.VAR_ANGLE, constants.VAR_LANE_ID)
        query_times = defaultdict(float)

        def record_pose_query(args, _, start, end):
            if args[0]._name in ("vehicle", "person") and args[1] in pose_variables:
                query_times[self.last_tick + 1] += end - start

        self.wrap(Domain, "_getUniversal", record_pose_query)
        self.wrap(mirror, "_plan_updates", lambda _, __, start, end:
                  self.samples["mirror_plan"].append((self.last_tick + 1, start, end)))
        return query_times

    @staticmethod
    def measure(original, observe):
        @wraps(original)
        def call(*args, **kwargs):
            start = perf_counter()
            result = original(*args, **kwargs)
            end = perf_counter()
            observe(args, result, start, end)
            return result

        return call

    def capture(self, carla, capture, *, legacy):
        def tick(_args, frame, start, end):
            self.last_tick = frame
            self.samples["tick"].append((frame, start, end))
            for name, pending in self.backlogs.items():
                self.samples[name].append((frame, pending()))

        listen = carla.Sensor.listen

        def observe_callbacks(sensor, callback):
            sensor_type = sensor.type_id
            sensor_id = sensor.id
            attrs = sensor.attributes
            self.samples["sensors"].append((sensor_type, sensor_id, attrs))

            def measure(data):
                self.samples["callbacks"].append((sensor_type, sensor_id, data.frame, self.last_tick))
                return callback(data)

            return listen(sensor, measure)

        self.stack.enter_context(patch.object(carla.Sensor, "listen", observe_callbacks))
        self.wrap(carla.World, "tick", tick)
        self.trace_legacy_radar(capture)
        if not legacy:
            self.wrap(capture, "_encode_carla_image_png", lambda args, _, start, end:
                      self.samples["png_encode"].append((args[0].frame, start, end)))
            self.wrap(capture, "camera_metadata_row", lambda args, _, start, end:
                      self.samples["camera_metadata"].append((args[0].frame, start, end)))
            snapshot_factory = capture.make_fast_tick_snapshot_fn

            def timed_snapshot_factory(*args, **kwargs):
                return self.measure(snapshot_factory(*args, **kwargs), lambda args, _, start, end:
                                    self.samples["ground_truth"].append((args[1].frame, start, end)))

            self.stack.enter_context(patch.object(capture, "make_fast_tick_snapshot_fn", timed_snapshot_factory))
        if legacy:
            from capture.radar_stream import PerRadarDequeBuffer

            self.wrap(PerRadarDequeBuffer, "__init__", lambda args, *_:
                      self.backlogs.update(radar_queue=args[0].pending))
            self.wrap(capture.CameraFrameWriter, "__init__", lambda args, *_:
                      self.backlogs.update(camera_queue=args[0].pending))

    def trace_legacy_radar(self, capture):
        original = capture.process_radar_measurement_fast

        @wraps(original)
        def timed_scan(measurement, sensor_id, label, actor, *, radar_writer, lock, counts):
            timed_lock = TimedRadarLock(lock)
            start = perf_counter()
            original(measurement, sensor_id, label, actor, radar_writer=radar_writer, lock=timed_lock, counts=counts)
            self.add_radar_sample(measurement, label, start, timed_lock.before_write, perf_counter())

        self.stack.enter_context(patch.object(capture, "process_radar_measurement_fast", timed_scan))

    def add_radar_sample(self, measurement, label, start, before_write, end):
        frame = measurement.frame
        self.samples["radar_written"].append((label, frame, len(measurement)))
        self.samples["radar_csv"].append((frame, start, end))
        self.samples["radar_format"].append((frame, start, before_write))
        self.samples["radar_output"].append((frame, before_write, end))

    def trace_batched_radar(self, radar_csv):
        original_format = radar_csv.format_radar_scan
        original_write = radar_csv.write_radar_scan
        before_write = None

        @wraps(original_format)
        def timed_format(*args):
            nonlocal before_write
            rows = original_format(*args)
            before_write = perf_counter()
            return rows

        @wraps(original_write)
        def timed_write(stream, measurement, sensor_id, label):
            start = perf_counter()
            original_write(stream, measurement, sensor_id, label)
            self.add_radar_sample(measurement, label, start, before_write, perf_counter())

        self.stack.enter_context(patch.object(radar_csv, "format_radar_scan", timed_format))
        self.stack.enter_context(patch.object(radar_csv, "write_radar_scan", timed_write))


def trace_legacy_child(directory, script, arguments):
    sys.path.insert(0, str(script.parent))
    sys.argv = [str(script), *arguments]
    module = importlib.import_module(script.stem)
    with ExitStack() as stack:
        trace = Trace(stack)
        if script.stem == "runner":
            trace.wrap(module.CarlaSyncManager, "step", lambda args, _, start, end:
                       trace.samples["mirror"].append((args[0].last_frame, start, end)))
        else:
            import carla

            trace.capture(carla, module, legacy=True)
        try:
            module.main()
        finally:
            write_json(directory / f"{script.stem}_trace.json", trace.samples)


def latency(samples):
    ms = [(end - start) * 1000 for _, start, end in samples]
    return {"count": len(ms), "median_ms": median(ms), "p95_ms": quantiles(ms, n=100)[94]} if len(ms) > 1 else {}


def summarize(directory, trace, run_end):
    segments = read_json(directory / "segments.json")["segments"]
    windows = [set(range(row["start_frame"], row["end_frame"] + 1)) for row in segments]
    frames = set().union(*windows)
    ticks = [event for event in trace["tick"] if event[0] in frames]
    tick_by_frame = {event[0]: event for event in ticks}
    mirrors = [event for event in trace["mirror"] if event[0] in frames]
    mirror_frames = {event[0] for event in mirrors}
    radar = defaultdict(set)
    zero_scans = 0
    for label, frame, returns in trace["radar_written"]:
        if frame in frames:
            radar[label].add(frame)
            zero_scans += returns == 0
    manifest = directory / "capture_frames.jsonl"
    with (directory / "camera_data.csv").open(encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        camera_header = reader.fieldnames
        camera = Counter(int(row["frame"]) for row in reader)
    with (directory / "actor_frames.jsonl").open(encoding="utf-8") as file:
        truth = {json.loads(line)["frame"] for line in file}
    callbacks = defaultdict(Counter)
    late = defaultdict(Counter)
    for sensor_type, sensor_id, frame, last_tick in trace["callbacks"]:
        if frame in frames:
            callbacks[sensor_type][(sensor_id, frame)] += 1
            late[sensor_type][max(0, last_tick - frame)] += 1
    points = 0
    with (directory / "radar_data.csv").open(encoding="utf-8", newline="") as file:
        radar_header = next(csv.reader(file))
        points = sum(1 for _ in file)

    rows = []
    periods = []
    for segment, window in zip(segments, windows):
        window_ticks = [tick_by_frame[frame] for frame in sorted(window) if frame in tick_by_frame]
        window_periods = [(frame, prev[1], start) for prev, (frame, start, _) in zip(window_ticks, window_ticks[1:])]
        periods.extend(window_periods)
        rows.append({
            "segment_id": segment["segment_id"], "scenario": segment["scenario"],
            "window_frames": len(window), "recording_s": window_ticks[-1][2] - window_ticks[0][1],
            "tick_period": latency(window_periods), "mirror_updates": len(mirror_frames & window),
            "camera_frames": len(camera.keys() & window), "actor_frames": len(truth & window),
            "radar_scans": {label: len(values & window) for label, values in radar.items()},
        })
    summary = {
        "window_frames": len(frames), "recording_s": sum(row["recording_s"] for row in rows),
        "segments": rows,
        "shutdown_s": run_end - ticks[-1][2], "carla_tick": latency(ticks),
        "actor_mirror": latency(mirrors), "mirror_updates_in_window": len(mirror_frames),
        "tick_period": latency(periods),
        "camera_frames": len(camera.keys() & frames), "camera_rows": sum(camera.values()),
        "camera_header": camera_header,
        "actor_frames": len(truth & frames), "radar_scans": {label: len(values & frames) for label, values in radar.items()},
        "zero_return_radar_scans": zero_scans, "radar_csv_rows": points, "radar_header": radar_header,
        "camera_frames_without_actor_log": len(camera.keys() - truth),
        "radar_frames_without_actor_log": len(set().union(*radar.values()) - truth),
        "sensor_callbacks": {kind: len(values) for kind, values in callbacks.items()},
        "duplicate_callbacks": {kind: sum(count - 1 for count in values.values()) for kind, values in callbacks.items()},
        "callback_lag_ticks": late,
        "tick_sampled_backlog_max": {name: max((pending for frame, pending in values if frame in frames), default=0)
                                     for name, values in trace.items() if name.endswith("_queue")},
        "world_frame_start": min(frames), "world_frame_end": max(frames),
    }
    if manifest.is_file():
        summary["frame_sync"] = read_json(directory / "run_meta.json")["frame_sync"]
        summary["paired_step"] = latency([event for event in trace["paired_step"] if event[0] in frames])
        with manifest.open(encoding="utf-8") as file:
            summary["sumo_recording_end_s"] = json.loads(list(file)[-1])["sumo_time_s"]
        summary["recording_stages"] = {}
        stages = ["ground_truth", "writer_submit", "png_encode", "camera_metadata", "radar_csv",
                  "radar_format", "radar_output"]
        stages.extend(stage for stage in ("mirror_plan", "traci_pose_queries") if stage in trace)
        for stage in stages:
            by_frame = defaultdict(float)
            for frame, start, end in trace[stage]:
                if frame in frames:
                    by_frame[frame] += end - start
            summary["recording_stages"][stage] = latency([(frame, 0, seconds) for frame, seconds in by_frame.items()])
    return summary


def benchmark_config(args, directory):
    from fusion.campaign import CampaignConfig, RunSpec

    if args.campaign is not None:
        cfg = CampaignConfig.load(args.campaign)
    else:
        cfg = CampaignConfig(
            name=f"benchmark_{args.engine}_{args.radars}",
            runs=[RunSpec(scenario=1, density=60, duration=args.duration, seed=42,
                          ambient_vehicles=30, pedestrians=12, bicycles=8)],
            warmup_s=args.warmup, radars_south=args.radars // 2, radars_north=args.radars // 2,
            radar_points_per_second=args.radar_pps, rate_hz=20,
            scene_cleanup=False, clear_trees=False, sync_timeout=15, runner_start_timeout_s=30,
        )
    if hasattr(cfg, "engine"):
        cfg.engine = args.engine
    elif args.engine != "legacy":
        raise RuntimeError("this revision only supports the legacy engine")
    cfg.label = cfg.postprocess = False
    cfg.carla_host, cfg.carla_port = args.host, args.port
    cfg.capture_base_dir = str(directory)
    return cfg


def run_trial(args):
    import carla
    from fusion import orchestrator
    from fusion.campaign import run_campaign
    from traffic import sumo_env

    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    client = carla.Client(args.host, args.port)
    client.set_timeout(30)
    world = client.get_world()
    if world.get_map().name.rsplit("/", 1)[-1] != "Town10HD_Opt" or world.get_settings().synchronous_mode:
        raise RuntimeError("benchmark requires an idle asynchronous Town10HD_Opt server")
    settings = world.get_settings()
    baseline = world.get_actors()
    if any(actor.type_id.startswith(("vehicle.", "walker.", "sensor.")) for actor in baseline):
        raise RuntimeError("benchmark needs a dedicated world without existing actors or sensors")
    baseline_ids = {actor.id for actor in baseline}
    cfg = benchmark_config(args, directory)
    with ExitStack() as stack:
        trace = Trace(stack)
        if args.engine == "legacy":
            launch = orchestrator._popen

            def launch_instrumented(command, cwd, env, *, new_group):
                script = Path(command[0])
                if script.stem in ("runner", "CaptureRadarCameraData"):
                    command = [str(Path(__file__).resolve()), "--child", str(directory), *command]
                return launch(command, cwd, dict(env, PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1"),
                              new_group=new_group)

            stack.enter_context(patch.object(orchestrator, "_popen", launch_instrumented))
        else:
            from fusion import capture_writer
            from fusion.actor_mirror import ActorMirror
            from fusion.capture_session import legacy
            from fusion.simulation_engine import SimulationEngine

            trace.capture(carla, legacy, legacy=False)
            if hasattr(capture_writer, "radar_csv"):
                trace.trace_batched_radar(capture_writer.radar_csv)
            trace.wrap(capture_writer.CaptureWriter, "__init__", lambda args, *_:
                       trace.backlogs.update(joined_frame_queue=args[0]._queue.qsize))
            trace.wrap(capture_writer.CaptureWriter, "submit", lambda args, _, start, end:
                       trace.samples["writer_submit"].append((args[1].simulation.carla_frame, start, end)))
            trace.wrap(SimulationEngine, "step", lambda _, frame, start, end:
                       trace.samples["paired_step"].append((frame.carla_frame, start, end)))
            trace.wrap(ActorMirror, "step", lambda _, result, start, end:
                       trace.samples["mirror"].append((trace.last_tick + 1, start, end)))
            if args.profile_mirror:
                query_times = trace.profile_mirror(ActorMirror)
        started = perf_counter()
        # This harness can also be copied into an older revision for comparison.
        if args.engine == "v2" and "carla_client" in signature(run_campaign).parameters:
            result = run_campaign(cfg, carla_client=client)
        else:
            result = run_campaign(cfg)
        run_end = perf_counter()
        if args.profile_mirror:
            trace.samples["traci_pose_queries"] = [(frame, 0, seconds) for frame, seconds in query_times.items()]

    actors = world.get_actors()
    leftovers = [actor for actor in actors if actor.id not in baseline_ids]
    summary = {
        "ok": result.ok, "error": result.error, "directory": result.capture_dir,
        "config": {**cfg.to_dict(), "engine": args.engine},
        "carla_version": client.get_server_version(),
        "sumo_version": subprocess.check_output([sumo_env.sumo_binary("sumo"), "--version"], text=True).splitlines()[0],
        "platform": platform.platform(), "python": platform.python_version(), "total_s": run_end - started,
        "settings_restored": world.get_settings() == settings,
        "leftover_dynamic_actors": [actor.type_id for actor in leftovers],
        "lights_frozen": sum(light.is_frozen() for light in actors.filter("traffic.traffic_light*")),
    }
    write_json(directory / "result.json", summary)
    # Measure legacy cleanup as delivered, then release only the rig actors that
    # the just-finished trial created on this dedicated endpoint.
    rig = [actor.id for actor in leftovers if actor.attributes.get("role_name", "").startswith(
        ("dataset_camera_", "dataset_radar_", "scalesense-v2-sensor:"),
    )]
    if rig:
        responses = client.apply_batch_sync([carla.command.DestroyActor(actor_id) for actor_id in rig], False)
        errors = [response.error for response in responses if response.error]
        if len(responses) != len(rig) or errors:
            raise RuntimeError(f"failed to release leftover benchmark rig: {errors}")
        world.wait_for_tick(5)
    if result.ok:
        if args.engine == "legacy":
            trace.samples.update(read_json(directory / "CaptureRadarCameraData_trace.json"))
            trace.samples.update(read_json(directory / "runner_trace.json"))
        else:
            write_json(directory / "v2_trace.json", trace.samples)
        summary.update(summarize(Path(result.capture_dir), trace.samples, run_end))
        write_json(directory / "result.json", summary)
    return 0 if result.ok else 1


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--child":
        _, _, directory, script, *arguments = sys.argv
        trace_legacy_child(Path(directory), Path(script), arguments)
        return 0
    os.environ["PYTHONDONTWRITEBYTECODE"] = "1"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", required=True, choices=("legacy", "v2"))
    preset = parser.add_mutually_exclusive_group(required=True)
    preset.add_argument("--radars", type=int, choices=(4, 8, 12, 14), help="use the single-row S01 benchmark")
    preset.add_argument("--campaign", type=Path, help="use settings from a campaign JSON")
    parser.add_argument("--output", required=True, type=Path, help="new directory for the capture and timing traces")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", required=True, type=int, help="dedicated asynchronous CARLA endpoint")
    parser.add_argument("--duration", type=int, default=22, help="recorded seconds for the single-row preset")
    parser.add_argument("--warmup", type=float, default=8, help="warm-up seconds for the single-row preset")
    parser.add_argument("--radar-pps", type=int, default=1200, help="rays per second for the single-row preset")
    parser.add_argument("--profile-mirror", action="store_true",
                        help="measure v2 mirror planning and TraCI pose queries separately")
    args = parser.parse_args()
    if args.profile_mirror and args.engine != "v2":
        parser.error("--profile-mirror requires --engine v2")
    return run_trial(args)


if __name__ == "__main__":
    raise SystemExit(main())
