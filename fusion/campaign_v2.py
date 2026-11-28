"""Run one or more scenarios on the v2 clock with one shared capture rig."""

from contextlib import ExitStack
from datetime import datetime
from math import ceil
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event

from .campaign import CampaignConfig, CampaignResult, RowOutcome, RunSpec, _write_json_atomic, write_campaign_record
from .carla_adapter import open_carla_session
from .config import v2_period_errors
from .orchestrator import _say
from .traffic_session import (
    StopRequested,
    build_scenario_config,
    check_stop,
    open_row_sumo,
    open_traffic_row,
    prepare_capture,
    run_offline_steps,
)


RPC_TIMEOUT_S = 5.0


def _scene_setup(world, cfg, say):
    if cfg.scene_cleanup:
        from dataset.world.ClearParkedCarsAndMotorcycles import disable_environment_objects
        from dataset.world.ClearTrashCansAndMailboxes import ENV_OBJECT_KEYWORDS, disable_matching_environment_objects

        vehicles = sum(disable_environment_objects(world, label) for label in ("Car", "Motorcycle"))
        props, _ = disable_matching_environment_objects(world, ENV_OBJECT_KEYWORDS)
        say(f"Scene cleanup hid {vehicles} parked vehicle objects and {props} street props.")
    if cfg.clear_trees:
        from dataset.world.ClearStretchTrees import _env_flag, stretch_box_from_env, vegetation_over_stretch

        ids = {obj.id for obj in vegetation_over_stretch(world, stretch_box_from_env())}
        if ids and not _env_flag("DATASET_TREE_CLEAR_DRY_RUN"):
            world.enable_environment_objects(ids, False)
            say(f"Hid {len(ids)} vegetation objects over the monitored stretch.")


def _capture_directory(cfg):
    from dataset.dataset_paths import data_output_dir

    root = Path(cfg.capture_base_dir).expanduser() if cfg.capture_base_dir else data_output_dir()
    safe_name = "".join(c if c.isalnum() or c in "-_" else "_" for c in cfg.name)[:48]
    suffix = f"_{safe_name}" if safe_name else ""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return (root / f"sensor_capture_{timestamp}{suffix}").resolve()


def _record_row(row, capture, *, warm_steps, record_steps, segment, stop_check, progress, status):
    engine = row.make_engine()
    finish_ticks = None
    last_progress = None
    try:
        for index in range(warm_steps + record_steps):
            stop_check()
            if index == warm_steps:
                capture.start_window(row.carla.world.get_snapshot().frame + 1, segment=segment)
            frame = engine.step()
            status.setdefault("first_frame", frame.carla_frame)
            status["carla_frame"] = frame.carla_frame
            status["sim_time"] = frame.sumo_time_s
            if index < warm_steps:
                phase, completed, total = "warmup", index + 1, warm_steps
            else:
                row.record(capture, frame)
                phase, completed, total = "recording", index - warm_steps + 1, record_steps
            percent = completed * 100 // total
            milestone = (phase, percent // 5)
            if milestone != last_progress:
                progress(phase, f"{percent}%")
                last_progress = milestone
    except StopRequested:
        finish_ticks = row.carla.tick
        raise
    else:
        finish_ticks = row.carla.tick
    finally:
        try:
            if finish_ticks is not None:
                capture.drain_window(finish_ticks)
        finally:
            status["frame_sync"] = capture.finish_window(flush=True)


def _segment(spec, cfg, row, status, record_steps):
    sync = status.get("frame_sync", {})
    row.start_frame = sync.get("first_frame")
    row.end_frame = sync.get("last_frame")
    row.recorded_frames = sum(sync.get("frames", {}).values())
    if row.start_frame is None or row.end_frame is None:
        return None
    segment = {
        "segment_id": row.index, "scenario": spec.scenario, "scenario_name": spec.scenario_name,
        "duration_s": spec.duration, "density": spec.density, "direction": spec.direction,
        "ambient_vehicles": spec.ambient_vehicles, "pedestrians": spec.pedestrians,
        "bicycles": spec.bicycles, "seed": spec.seed, "signals": spec.signals, "note": spec.note,
        "warmup_s": cfg.warmup_s, "rate_hz": cfg.rate_hz,
        "runner_first_frame": status.get("first_frame"),
        "start_frame": row.start_frame, "end_frame": row.end_frame,
        "planned_end_frame": row.start_frame + record_steps - 1,
        "recorded_frames": row.recorded_frames,
        "status": "complete" if row.recorded_frames == record_steps else "truncated",
        "frame_sync": sync,
    }
    if row.status in ("stopped", "failed"):
        segment["close_reason"] = row.detail
    return segment


def run_campaign_v2(cfg, log=None, on_event=None, stop_event=None, *, single=False, carla_client=None):
    result = CampaignResult(rows=[RowOutcome(i) for i in range(len(cfg.runs))])
    log = log if log is not None else []
    stop_event = stop_event if stop_event is not None else Event()

    def say(message):
        _say(log, f"[v2] {message}")

    def emit(**event):
        if on_event is not None:
            try:
                on_event(event)
            except Exception:
                pass

    def publish(row, status, detail=""):
        row.status, row.detail = status, detail
        emit(type="row", index=row.index, status=status, detail=detail,
             start_frame=row.start_frame, end_frame=row.end_frame, recorded_frames=row.recorded_frames)

    def stop_check():
        check_stop(stop_event)

    errors = v2_period_errors(cfg.fixed_delta_s) + cfg.v2_runtime_errors() if single else cfg.validate()
    if errors:
        result.error = ", ".join(errors)
        say(result.error)
        emit(type="finished", ok=False, error=result.error, run_dir="")
        return result

    runners = {}
    segments = []
    output_dir = None
    try:
        from .capture_session import CaptureOptions, open_capture

        delta = cfg.fixed_delta_s
        warm_steps = round(cfg.warmup_s / delta)
        output_dir = _capture_directory(cfg)
        options = CaptureOptions(
            output_dir, cfg.radars_south, cfg.radars_north, cfg.radar_height_m,
            cfg.radar_points_per_second,
        )
        say(f"Starting {len(cfg.runs)} run(s), {cfg.radar_count} radars at {cfg.rate_hz:g} Hz.")
        with ExitStack() as resources:
            stop_check()
            carla = resources.enter_context(open_carla_session(
                cfg.carla_host, cfg.carla_port, delta, ready_timeout_s=cfg.sync_timeout,
                rpc_timeout_s=RPC_TIMEOUT_S, stop_check=stop_check, client=carla_client,
            ))
            _scene_setup(carla.world, cfg, say)
            capture = None
            for outcome, spec in zip(result.rows, cfg.runs):
                stop_check()
                record_steps = round(spec.duration / delta)
                duration = ceil((warm_steps + record_steps) * delta)
                status = {"scenario": spec.scenario}
                runners[f"runner_{outcome.index}"] = status
                publish(outcome, "starting")
                say(f"Run {outcome.index + 1}/{len(result.rows)} S{spec.scenario:02d} {spec.scenario_name}.")

                def progress(phase, detail):
                    publish(outcome, phase, detail)

                try:
                    with TemporaryDirectory(prefix="scalesense-v2-row-") as directory:
                        config = build_scenario_config(
                            directory, scenario=spec.scenario, density=spec.density,
                            duration=duration, direction=spec.direction, ambient_vehicles=spec.ambient_vehicles,
                            pedestrians=spec.pedestrians, bicycles=spec.bicycles, seed=spec.seed,
                            step_length=delta,
                        )
                        with open_row_sumo(
                            config, delta, ready_timeout_s=cfg.runner_start_timeout_s,
                            rpc_timeout_s=RPC_TIMEOUT_S, gui=cfg.sumo_gui, stop_check=stop_check,
                        ) as sumo, open_traffic_row(
                            carla, sumo, scenario=spec.scenario, duration=duration, seed=spec.seed,
                            render_radius=None if cfg.no_cull else cfg.render_radius,
                            stretch_signals=spec.signals,
                        ) as row:
                            status.update(pid=sumo.pid, version=sumo.version)
                            if capture is None:
                                capture = resources.enter_context(open_capture(
                                    carla, options, scenario=spec.scenario if single else None,
                                    seed=spec.seed if single else None, sumo_version=sumo.version,
                                    campaign_name=None if single else cfg.name,
                                ))
                                result.capture_dir = str(output_dir)
                                prepare_capture(carla, capture, stop_event)
                                emit(type="capture", state="running", run_dir=result.capture_dir)
                            _record_row(
                                row, capture, warm_steps=warm_steps, record_steps=record_steps,
                                segment=None if single else outcome.index, stop_check=stop_check,
                                progress=progress, status=status,
                            )
                            unseen = [
                                label for label, count in status["frame_sync"]["received"].items() if not count
                            ]
                            if unseen:
                                raise RuntimeError("no captured measurements from " + ", ".join(unseen))
                except KeyboardInterrupt:
                    outcome.status, outcome.detail = "stopped", "stop requested"
                    raise
                except Exception as exc:
                    outcome.status, outcome.detail = "failed", str(exc)
                    raise
                else:
                    outcome.status = "done"
                finally:
                    segment = _segment(spec, cfg, outcome, status, record_steps)
                    status.update(state=outcome.status, recorded_frames=outcome.recorded_frames)
                    publish(outcome, outcome.status, outcome.detail or f"{outcome.recorded_frames} frames")
                    if segment is not None and not single:
                        segments.append(segment)
                        _write_json_atomic(output_dir / "segments.json", {"format": 1, "segments": segments})
                stop_check()
    except KeyboardInterrupt:
        result.stopped = True
        say("Stop requested. Kept the confirmed capture frames.")
    except Exception as exc:
        result.error = str(exc)
        say(f"Campaign failed: {exc}")

    for outcome in result.rows:
        if outcome.status == "queued":
            publish(outcome, "skipped", "campaign stopped" if result.stopped else "campaign failed")
    if result.capture_dir:
        try:
            if not single:
                _write_json_atomic(output_dir / "segments.json", {"format": 1, "segments": segments})
            if not result.error and any(row.recorded_frames for row in result.rows):
                run_offline_steps(
                    output_dir, label=cfg.label, postprocess=cfg.postprocess,
                    on_state=lambda state: emit(type="capture", state=state, run_dir=result.capture_dir),
                )
            result.ok = not result.stopped and not result.error and all(row.status == "done" for row in result.rows)
            if not single:
                write_campaign_record(cfg, result, runners)
        except Exception as exc:
            result.ok = False
            result.error = str(exc)
            say(f"Capture finalization failed: {exc}")
            if not single:
                try:
                    write_campaign_record(cfg, result, runners)
                except OSError as record_error:
                    say(f"Could not save campaign results: {record_error}")
    if result.capture_dir:
        say(f"Capture directory {result.capture_dir}")
    emit(type="finished", ok=result.ok, stopped=result.stopped, error=result.error, run_dir=result.capture_dir)
    return result


def run_single_v2(cfg, stop_event=None):
    cfg.validate()
    if not cfg.ok:
        _say(None, "[v2] " + ", ".join(cfg.errors))
        return CampaignResult(error=", ".join(cfg.errors))
    spec = RunSpec(
        scenario=cfg.scenario, density=cfg.density, duration=cfg.duration, direction=cfg.direction,
        ambient_vehicles=cfg.ambient_vehicles, pedestrians=cfg.pedestrians, bicycles=cfg.bicycles,
        seed=cfg.ambient_seed,
    )
    settings = {name: getattr(cfg, name) for name in (
        "radars_south", "radars_north", "radar_height_m", "radar_points_per_second", "rate_hz",
        "label", "postprocess", "scene_cleanup", "clear_trees", "sumo_gui", "render_radius",
        "no_cull", "carla_host", "carla_port", "capture_base_dir", "sync_timeout",
    )}
    campaign = CampaignConfig(runs=[spec], warmup_s=0, engine="v2", **settings)
    return run_campaign_v2(campaign, stop_event=stop_event, single=True)
