import csv
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import carla

from fusion.capture_writer import CaptureWriter
from fusion.frame_buffer import FrameBuffer
from fusion.simulation_engine import SimulationFrame


class CaptureWriterTests(unittest.TestCase):
    def image(self, frame):
        return SimpleNamespace(
            frame=frame, timestamp=frame * 0.05, width=1, height=1,
            transform=carla.Transform(), raw_data=bytes(4),
        )

    @staticmethod
    def actor(actor_id):
        return {
            "id": actor_id, "kind": "vehicle", "type_id": "vehicle.tesla.model3",
            "class_label": "Car", "location": carla.Location(x=10),
            "rotation": {"roll": 0, "pitch": 0, "yaw": 0}, "bbox": None,
        }

    def test_delayed_camera_row_uses_actors_from_its_own_frame(self):
        buffer = FrameBuffer(100, ("C10",), max_lag_frames=5)
        buffer.confirm(SimulationFrame(1, 0.05, 100), [self.actor(16)])
        buffer.offer("C10", 101, self.image(101))
        buffer.confirm(SimulationFrame(2, 0.10, 101), [self.actor(23)])
        buffer.offer("C10", 100, self.image(100))

        with TemporaryDirectory() as directory:
            path = Path(directory)
            writer = CaptureWriter(path, {}, 9, 90)
            try:
                for frame in buffer.pop_ready():
                    writer.submit(frame)
            finally:
                writer.close()
            with (path / "camera_data.csv").open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(
                [(row["frame"], row["nearest_actor_id"]) for row in rows], [("100", "16"), ("101", "23")],
            )
            for row in rows:
                self.assertTrue(Path(row["image_path"]).is_file())
            manifest = (path / "capture_frames.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual([json.loads(line)["status"] for line in manifest], ["complete", "complete"])

    def test_background_writer_reports_encoding_failure(self):
        buffer = FrameBuffer(100, ("C10",), max_lag_frames=5)
        buffer.offer("C10", 100, self.image(100))
        frame, = buffer.confirm(SimulationFrame(1, 0.05, 100), [])

        with TemporaryDirectory() as directory, patch(
            "fusion.capture_writer.legacy._encode_carla_image_png", side_effect=OSError("injected PNG error"),
        ):
            writer = CaptureWriter(Path(directory), {}, 9, 90)
            writer.submit(frame)
            with self.assertRaisesRegex(RuntimeError, "capture writer failed") as error:
                writer.close()
            self.assertEqual(str(error.exception.__cause__), "injected PNG error")
