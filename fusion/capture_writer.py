"""Write joined frames in the legacy dataset formats on one bounded worker."""

from contextlib import ExitStack
import json
from pathlib import Path
from queue import Full, Queue
from threading import Event, Lock, Thread
from time import monotonic

from dataset.capture import CaptureRadarCameraData as legacy
from capture.actor_frame_log import serialize_actor_frame

from .frame_buffer import JoinedFrame


class CaptureWriter:
    def __init__(self, directory: Path, radar_ids: dict[str, int], camera_id: int, camera_fov: float):
        self._directory = directory
        self._radar_ids = radar_ids
        self._camera_id = camera_id
        self._camera_fov = camera_fov
        self._camera_folder = directory / "camera_frames" / f"camera_{camera_id}"
        self._camera_folder.mkdir(parents=True)
        self._queue: Queue[JoinedFrame | Event | None] = Queue(maxsize=8)
        self._error: Exception | None = None
        self._thread = Thread(target=self._run, name="v2-capture-writer", daemon=True)
        self._thread.start()

    def check(self):
        if self._error is not None:
            raise RuntimeError("v2 capture writer failed") from self._error

    def submit(self, item: JoinedFrame):
        self._put(item)

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
        if self._error is None:
            self._put(None)
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

    def _run(self):
        try:
            with ExitStack() as files:
                radar_file, radar_writer = legacy.setup_radar_writer(self._directory / "radar_data.csv")
                files.enter_context(radar_file)
                camera_file, camera_writer = legacy.setup_camera_writer(self._directory / "camera_data.csv")
                files.enter_context(camera_file)
                gt = files.enter_context((self._directory / "actor_frames.jsonl").open("w", encoding="utf-8"))
                manifest = files.enter_context((self._directory / "capture_frames.jsonl").open("w", encoding="utf-8"))
                lock = Lock()
                counts = {"radar_messages": 0, "radar_detections": 0, "radar_scored": 0}
                while (item := self._queue.get()) is not None:
                    if isinstance(item, Event):
                        for stream in (radar_file, camera_file, gt, manifest):
                            stream.flush()
                        item.set()
                        continue
                    frame = item.simulation
                    gt.write(serialize_actor_frame(frame.carla_frame, item.actors, item.segment))
                    for label, measurement in item.measurements.items():
                        if label == "C10":
                            path = str(self._camera_folder / f"frame_{frame.carla_frame:08d}.png")
                            legacy._encode_carla_image_png(measurement, path)
                            camera_writer.writerow(legacy.camera_metadata_row(
                                measurement, self._camera_id, label, path,
                                self._camera_fov, item.actors, item.segment,
                            ))
                        else:
                            legacy.process_radar_measurement_fast(
                                measurement, self._radar_ids[label], label, None,
                                radar_writer=radar_writer, lock=lock, counts=counts,
                            )
                    record = {
                        "frame": frame.carla_frame, "step": frame.step,
                        "sumo_time_s": frame.sumo_time_s, "status": item.status,
                        "received": list(item.measurements), "missing": item.missing,
                    }
                    if item.segment is not None:
                        record["segment_id"] = item.segment
                    manifest.write(json.dumps(record, separators=(",", ":")) + "\n")
        except Exception as exc:
            self._error = exc
