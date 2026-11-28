"""Own a v2 roadside rig and record only verified paired simulator frames."""

from collections import Counter
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from time import monotonic, sleep
from uuid import uuid4

import carla

from dataset.capture import CaptureRadarCameraData as legacy
from dataset.setup.RadarCameraSetup import (
    configure_dataset_camera_blueprint,
    dataset_camera_positions,
)
from capture.ExportCameraExtrinsics import write_outputs as write_camera_extrinsics
from capture.ExportRadarExtrinsics import sensor_extrinsic_row, write_outputs as write_radar_extrinsics
from capture.radar_layout import (
    apply_radar_pitch,
    radar_pitch_deg_from_env,
    rig_anchor_from_env,
    rig_heading_deg_from_env,
    stretch_radar_positions,
)

from .capture_writer import CaptureWriter
from .config import radar_pps_errors, radar_rig_errors
from .frame_buffer import FrameBuffer


WARMUP_TICKS = 3
TAIL_TICKS = 6
MAX_LAG_FRAMES = 8
ROLE_PREFIX = "scalesense-v2-sensor:"
# CARLA 0.9.16 can assert in MultiStreamState::DisconnectSession when DestroyActor
# races in-flight TCP stream teardown. Upstream examples drain after sensor.stop().
SENSOR_STREAM_DRAIN_S = 0.4


@dataclass(frozen=True)
class CaptureOptions:
    directory: Path
    radars_south: int
    radars_north: int
    height_m: float = 3.0
    radar_pps: int = 0

    def __post_init__(self):
        errors = radar_rig_errors(self.radars_south, self.radars_north, self.height_m)
        errors.extend(radar_pps_errors(self.radar_pps))
        if errors:
            raise ValueError(", ".join(errors))


@contextmanager
def open_sensor_rig(session, options: CaptureOptions):
    world = session.world
    if world.get_settings().no_rendering_mode:
        raise RuntimeError("v2 capture requires CARLA rendering for the RGB camera")
    blueprints = world.get_blueprint_library()
    radar_positions = stretch_radar_positions(
        options.radars_south, options.radars_north, height=options.height_m,
    )
    apply_radar_pitch(radar_positions)
    role_prefix = f"{ROLE_PREFIX}{uuid4().hex}:"
    camera_positions = dataset_camera_positions()
    definitions = []
    for label, transform in {**radar_positions, **camera_positions}.items():
        if label in camera_positions:
            blueprint = blueprints.find("sensor.camera.rgb")
            configure_dataset_camera_blueprint(blueprint)
            blueprint.set_attribute("sensor_tick", "0")
        else:
            blueprint = blueprints.find("sensor.other.radar")
            legacy.configure_dataset_radar_blueprint(
                blueprint, points_per_second=options.radar_pps or None, sensor_tick_s=0,
            )
        blueprint.set_attribute("role_name", role_prefix + label)
        definitions.append((label, carla.command.SpawnActor(blueprint, transform)))

    ids = {}
    spawn_uncertain = True
    sensors = {}
    try:
        responses = session.client.apply_batch_sync([cmd for _, cmd in definitions], False)
        errors = []
        for (label, _), response in zip(definitions, responses):
            if response.error:
                errors.append(f"{label}: {response.error}")
            else:
                ids[label] = response.actor_id
        spawn_uncertain = len(responses) != len(definitions)
        if spawn_uncertain or errors:
            raise RuntimeError("incomplete CARLA sensor rig: " + ", ".join(errors or ["missing batch replies"]))

        for label, actor_id in ids.items():
            sensor = world.get_actor(actor_id)
            if sensor is None:
                raise RuntimeError(f"CARLA did not expose spawned sensor {label}")
            sensors[label] = sensor
        try:
            yield sensors
        finally:
            for sensor in sensors.values():
                stop_sensor(sensor)
            if sensors:
                sleep(SENSOR_STREAM_DRAIN_S)
    finally:
        owned = set(ids.values())
        try:
            if spawn_uncertain:
                session.tick()
                owned.update(
                    actor.id for actor in world.get_actors().filter("sensor.*")
                    if actor.attributes.get("role_name", "").startswith(role_prefix)
                )
        finally:
            if owned:
                responses = session.client.apply_batch_sync(
                    [carla.command.DestroyActor(actor_id) for actor_id in sorted(owned)], False,
                )
                if len(responses) != len(owned) or any(response.error for response in responses):
                    raise RuntimeError("CARLA did not acknowledge removal of the full v2 sensor rig")


def stop_sensor(sensor):
    try:
        if sensor.is_listening:
            sensor.stop()
    except RuntimeError:
        pass


class CaptureSession:
    def __init__(self, session, directory, sensors, metadata):
        self._world = session.world
        self._directory = directory
        self._sensors = sensors
        self._metadata = metadata
        self._buffer: FrameBuffer | None = None
        self._first_frame = self._last_frame = None
        self._statuses: Counter = Counter()
        self._received: Counter = Counter(dict.fromkeys(sensors, 0))
        self._ignored: Counter = Counter()
        self._max_lag: Counter = Counter()
        self._snapshot_actors = legacy.make_fast_tick_snapshot_fn(self._world, prune_absent=True)
        self._radar_ids = {label: actor.id for label, actor in sensors.items() if label != "C10"}
        camera = sensors["C10"]
        self._writer = CaptureWriter(
            directory, self._radar_ids, camera.id, float(camera.attributes["fov"]),
        )

    def listen(self):
        for label, actor in self._sensors.items():
            actor.listen(lambda data, label=label: self._offer(label, data))

    def _offer(self, label, data):
        buffer = self._buffer
        if buffer is not None:
            buffer.offer(label, int(data.frame), data)

    def prepare(self, snapshot):
        radar_rows = []
        camera_rows = []
        for label, actor in self._sensors.items():
            sensor_snapshot = snapshot.find(actor.id)
            if sensor_snapshot is None:
                raise RuntimeError(f"sensor {label} is missing from CARLA snapshot {snapshot.frame}")
            row = sensor_extrinsic_row(actor.id, label, sensor_snapshot.get_transform())
            (camera_rows if label == "C10" else radar_rows).append(row)
        write_radar_extrinsics(radar_rows, self._directory)
        write_camera_extrinsics(camera_rows, self._directory)

    def start_window(self, first_frame, *, segment=None):
        if self._buffer is not None:
            raise RuntimeError("capture already has an active recording window")
        self._buffer = FrameBuffer(
            first_frame, tuple(self._sensors), max_lag_frames=MAX_LAG_FRAMES, segment=segment,
        )

    def record(self, frame, snapshot, required_actor_ids):
        self._writer.check()
        if snapshot.frame != frame.carla_frame:
            raise RuntimeError(f"capture expected snapshot {frame.carla_frame}, got {snapshot.frame}")
        actors = self._snapshot_actors(self._world, snapshot)
        missing = set(required_actor_ids) - {actor["id"] for actor in actors}
        if missing:
            raise RuntimeError(f"missing ground truth for CARLA actors {sorted(missing)} in frame {snapshot.frame}")
        self._submit(self._buffer.confirm(frame, actors))

    def drain_window(self, tick):
        buffer = self._buffer
        if buffer is None:
            return
        buffer.end_recording()
        for _ in range(TAIL_TICKS):
            self._submit(buffer.pop_ready())
            if not buffer.pending:
                break
            tick()
        deadline = monotonic() + 0.5
        while buffer.pending and monotonic() < deadline:
            self._submit(buffer.pop_ready())
            if buffer.pending:
                sleep(0.01)

    def finish_window(self, *, flush=False):
        buffer = self._buffer
        if buffer is None:
            return {}
        self._submit(buffer.finish())
        self._buffer = None
        stats = buffer.stats()
        if stats["first_frame"] is not None:
            if self._first_frame is None:
                self._first_frame = stats["first_frame"]
            self._last_frame = stats["last_frame"]
        self._statuses.update(stats["frames"])
        self._received.update(stats["received"])
        self._ignored.update(stats["ignored"])
        self._max_lag |= Counter(stats["max_arrival_lag_frames"])
        if flush:
            self._writer.flush()
        return stats

    def _submit(self, frames):
        for item in frames:
            self._writer.submit(item)

    def close(self, status):
        try:
            try:
                self.finish_window()
            finally:
                self._writer.close()
            missing = [label for label, received in self._received.items() if not received]
            if status == "complete" and missing:
                raise RuntimeError("no captured measurements from " + ", ".join(missing))
            if status == "complete" and any(key != "complete" for key in self._statuses):
                status = "partial"
        except BaseException:
            status = "failed"
            raise
        finally:
            self.write_meta("stop", status=status, stats={
                "first_frame": self._first_frame,
                "last_frame": self._last_frame,
                "frames": dict(self._statuses),
                "received": dict(self._received),
                "ignored": dict(self._ignored),
                "max_arrival_lag_frames": {label: self._max_lag[label] for label in self._received},
            })

    def write_meta(self, phase, *, status, stats=None):
        legacy.write_run_meta(
            str(self._directory), self._world, radar_count=len(self._radar_ids),
            camera_count=1, phase=phase,
            extra={**self._metadata, "capture_status": status, "frame_sync": stats or {}},
        )


@contextmanager
def open_capture(session, options: CaptureOptions, *, scenario, seed, sumo_version, campaign_name=None):
    directory = options.directory.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    with open_sensor_rig(session, options) as sensors:
        radar = sensors["R1"]
        pps = int(radar.attributes["points_per_second"])
        anchor_x, anchor_y = rig_anchor_from_env(options.radars_south + options.radars_north)
        metadata = {
            "engine": "cosimulation-v2",
            "scenario": scenario,
            "seed": seed,
            "carla_version": session.version,
            "sumo_version": sumo_version,
            "fixed_delta_seconds": session.fixed_delta_s,
            "rig_pose": {
                "anchor_x": anchor_x,
                "anchor_y": anchor_y,
                "height_m": options.height_m,
                "radars_south": options.radars_south,
                "radars_north": options.radars_north,
                "yaw_deg": rig_heading_deg_from_env(),
                "pitch_deg": radar_pitch_deg_from_env(),
            },
            "radar_points_per_second": pps,
            "dataset_env": {
                "DATASET_RADARS_SOUTH": str(options.radars_south),
                "DATASET_RADARS_NORTH": str(options.radars_north),
                "DATASET_RIG_HEIGHT_M": str(options.height_m),
                "DATASET_RADAR_POINTS_PER_SECOND": str(pps),
                "DATASET_RADAR_SENSOR_TICK_S": radar.attributes["sensor_tick"],
                "DATASET_SYNC_FIXED_DELTA_S": str(session.fixed_delta_s),
            },
        }
        if campaign_name is not None:
            metadata["campaign_name"] = campaign_name
        capture = CaptureSession(session, directory, sensors, metadata)
        try:
            capture.write_meta("start", status="recording")
            capture.listen()
            yield capture
        except KeyboardInterrupt:
            capture.close("interrupted")
            raise
        except BaseException:
            capture.close("failed")
            raise
        else:
            capture.close("complete")
