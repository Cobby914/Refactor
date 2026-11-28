"""Encode camera images in parallel and write joined frames in order."""

from concurrent.futures import Future
from contextlib import ExitStack
from dataclasses import dataclass
import json
from pathlib import Path
from queue import Full, Queue, SimpleQueue
from tempfile import TemporaryDirectory
from threading import BoundedSemaphore, Event, Thread
from time import monotonic

from dataset.capture import CaptureRadarCameraData as legacy
from capture.actor_frame_log import serialize_actor_frame

from . import radar_csv
from .frame_buffer import JoinedFrame


MAX_QUEUED_FRAMES = 8


@dataclass(frozen=True)
class _PendingFrame:
    joined: JoinedFrame
    png: Future[Path] | None


class CaptureWriter:
    def __init__(self, directory: Path, radar_ids: dict[str, int], camera_id: int, camera_fov: float):
        self._directory = directory
        self._radar_ids = radar_ids
        self._camera_id = camera_id
        self._camera_fov = camera_fov
        self._camera_folder = directory / "camera_frames" / f"camera_{camera_id}"
        self._camera_folder.mkdir(parents=True)
        self._queue: Queue[_PendingFrame | Event | None] = Queue(maxsize=MAX_QUEUED_FRAMES)
        # One permit covers a frame from submission through commit, including
        # work held by a PNG encoder. Account for the writer's active frame.
        self._slots = BoundedSemaphore(MAX_QUEUED_FRAMES + 1)
        self._staging = TemporaryDirectory(prefix=".pending-png-", dir=directory)
        self._png_jobs = SimpleQueue()
        self._error: Exception | None = None
        self._encoders = [
            Thread(target=self._encode_cameras, name=f"v2-camera-png-{index}", daemon=True)
            for index in range(legacy.camera_encoder_threads_from_env())
        ]
        for encoder in self._encoders:
            encoder.start()
        self._thread = Thread(target=self._run, name="v2-capture-writer", daemon=True)
        self._thread.start()

    def check(self):
        if self._error is not None:
            raise RuntimeError("v2 capture writer failed") from self._error

    def submit(self, item: JoinedFrame):
        deadline = monotonic() + 30
        while monotonic() < deadline:
            self.check()
            if self._slots.acquire(timeout=0.1):
                break
        else:
            raise TimeoutError("v2 capture writer did not free a frame slot within 30 seconds")
        try:
            self.check()
            png = None
            if "C10" in item.measurements:
                png = Future()
                name = f"frame_{item.simulation.carla_frame:08d}.png"
                self._png_jobs.put((item.measurements["C10"], Path(self._staging.name) / name, png))
            self._put(_PendingFrame(item, png))
        except BaseException:
            self._slots.release()
            raise

    def _put(self, item):
        deadline = monotonic() + 30
        while monotonic() < deadline:
            self.check()
            try:
                self._queue.put(item, timeout=0.1)
                return
            except Full:
                continue
        raise TimeoutError("v2 capture writer did not free queue space within 30 seconds")

    def close(self):
        try:
            if self._error is None:
                self._put(None)
        finally:
            self._thread.join(timeout=60)
        if self._thread.is_alive():
            raise TimeoutError("v2 capture writer did not finish within 60 seconds")
        self.check()

    def flush(self):
        """Flush row output before committing the segment range."""
        barrier = Event()
        self._put(barrier)
        deadline = monotonic() + 60
        while not barrier.wait(0.1):
            self.check()
            if monotonic() >= deadline:
                raise TimeoutError("v2 capture writer did not flush within 60 seconds")

    def _encode_cameras(self):
        while (job := self._png_jobs.get()) is not None:
            if self._error is not None:
                continue
            image, path, result = job
            try:
                legacy._encode_carla_image_png(image, str(path))
            except Exception as exc:
                result.set_exception(exc)
            else:
                result.set_result(path)

    def _run(self):
        try:
            self._write_frames()
        except Exception as exc:
            self._error = exc
        finally:
            for _ in self._encoders:
                self._png_jobs.put(None)
            for encoder in self._encoders:
                encoder.join()
            try:
                self._staging.cleanup()
            except OSError as exc:
                if self._error is None:
                    self._error = exc

    def _write_frames(self):
        with ExitStack() as files:
            radar_file, _ = legacy.setup_radar_writer(self._directory / "radar_data.csv")
            files.enter_context(radar_file)
            camera_file, camera_writer = legacy.setup_camera_writer(self._directory / "camera_data.csv")
            files.enter_context(camera_file)
            gt = files.enter_context((self._directory / "actor_frames.jsonl").open("w", encoding="utf-8"))
            manifest = files.enter_context((self._directory / "capture_frames.jsonl").open("w", encoding="utf-8"))
            while (pending := self._queue.get()) is not None:
                if isinstance(pending, Event):
                    for stream in (radar_file, camera_file, gt, manifest):
                        stream.flush()
                    pending.set()
                    continue
                try:
                    item = pending.joined
                    staged = pending.png.result() if pending.png is not None else None
                    frame = item.simulation
                    gt.write(serialize_actor_frame(frame.carla_frame, item.actors, item.segment))
                    for label, measurement in item.measurements.items():
                        if label == "C10":
                            assert staged is not None
                            path = str(staged.replace(self._camera_folder / staged.name))
                            camera_writer.writerow(legacy.camera_metadata_row(
                                measurement, self._camera_id, label, path,
                                self._camera_fov, item.actors, item.segment,
                            ))
                        else:
                            radar_csv.write_radar_scan(radar_file, measurement, self._radar_ids[label], label)
                    record = {
                        "frame": frame.carla_frame, "step": frame.step,
                        "sumo_time_s": frame.sumo_time_s, "status": item.status,
                        "received": list(item.measurements), "missing": item.missing,
                    }
                    if item.segment is not None:
                        record["segment_id"] = item.segment
                    manifest.write(json.dumps(record, separators=(",", ":")) + "\n")
                finally:
                    self._slots.release()
