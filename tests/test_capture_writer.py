import csv
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event, Thread
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import carla

from fusion.capture_writer import CaptureWriter, MAX_QUEUED_FRAMES
from fusion.frame_buffer import FrameBuffer, JoinedFrame
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

    def test_out_of_order_encoding_flushes_images_and_metadata_in_frame_order(self):
        from fusion.capture_writer import legacy

        buffer = FrameBuffer(100, ("C10",), max_lag_frames=5)
        buffer.confirm(SimulationFrame(1, 0.05, 100), [self.actor(16)])
        buffer.offer("C10", 101, self.image(101))
        buffer.confirm(SimulationFrame(2, 0.10, 101), [self.actor(23)])
        buffer.offer("C10", 100, self.image(100))

        encode_png = legacy._encode_carla_image_png
        second_encoded = Event()
        encoded = []

        def encode(image, path):
            if image.frame == 100 and not second_encoded.wait(5):
                raise TimeoutError("frame 101 did not encode while frame 100 was waiting")
            encode_png(image, path)
            encoded.append(image.frame)
            if image.frame == 101:
                second_encoded.set()

        with (TemporaryDirectory() as directory,
              patch.object(legacy, "camera_encoder_threads_from_env", return_value=2),
              patch.object(legacy, "_encode_carla_image_png", side_effect=encode)):
            path = Path(directory)
            writer = CaptureWriter(path, {}, 9, 90)
            try:
                for frame in buffer.pop_ready():
                    writer.submit(frame)
                writer.flush()
                with (path / "camera_data.csv").open(newline="", encoding="utf-8") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(encoded, [101, 100])
                self.assertEqual(
                    [(row["frame"], row["nearest_actor_id"]) for row in rows], [("100", "16"), ("101", "23")],
                )
                for row in rows:
                    self.assertTrue(Path(row["image_path"]).is_file())
                manifest = (path / "capture_frames.jsonl").read_text(encoding="utf-8").splitlines()
                self.assertEqual([json.loads(line)["frame"] for line in manifest], [100, 101])
            finally:
                second_encoded.set()
                writer.close()
            self.assertFalse(list(path.glob(".pending-png-*")))

    def test_background_writer_reports_encoding_failure(self):
        from fusion.capture_writer import legacy

        buffer = FrameBuffer(100, ("C10",), max_lag_frames=5)
        buffer.offer("C10", 100, self.image(100))
        first, = buffer.confirm(SimulationFrame(1, 0.05, 100), [])
        buffer.offer("C10", 101, self.image(101))
        second, = buffer.confirm(SimulationFrame(2, 0.10, 101), [])

        encode_png = legacy._encode_carla_image_png
        second_encoded = Event()

        def encode(image, path):
            if image.frame == 100:
                if not second_encoded.wait(5):
                    raise TimeoutError("frame 101 did not encode before the injected failure")
                raise OSError("injected PNG error")
            encode_png(image, path)
            second_encoded.set()

        with (TemporaryDirectory() as directory,
              patch.object(legacy, "camera_encoder_threads_from_env", return_value=2),
              patch.object(legacy, "_encode_carla_image_png", side_effect=encode)):
            path = Path(directory)
            writer = CaptureWriter(path, {}, 9, 90)
            try:
                writer.submit(first)
                writer.submit(second)
                with self.assertRaisesRegex(RuntimeError, "capture writer failed"):
                    writer.flush()
            finally:
                second_encoded.set()
                with self.assertRaisesRegex(RuntimeError, "capture writer failed") as error:
                    writer.close()
            self.assertEqual(str(error.exception.__cause__), "injected PNG error")
            self.assertEqual((path / "capture_frames.jsonl").read_text(encoding="utf-8"), "")
            self.assertFalse(list(path.rglob("*.png")))
            self.assertFalse(list(path.glob(".pending-png-*")))

    def test_pending_encodes_share_the_frame_backpressure_limit(self):
        from fusion.capture_writer import legacy

        frames = [
            JoinedFrame(SimulationFrame(i, i * 0.05, i), [], {"C10": self.image(i)}, (), "complete")
            for i in range(1, MAX_QUEUED_FRAMES + 3)
        ]
        release_encoders = Event()
        submitting = Event()
        submitted = Event()
        encode_png = legacy._encode_carla_image_png

        def encode(image, path):
            if not release_encoders.wait(5):
                raise TimeoutError("test did not release the encoders")
            encode_png(image, path)

        with (TemporaryDirectory() as directory,
              patch.object(legacy, "camera_encoder_threads_from_env", return_value=2),
              patch.object(legacy, "_encode_carla_image_png", side_effect=encode)):
            path = Path(directory)
            writer = CaptureWriter(path, {}, 9, 90)

            def submit_last_frame():
                submitting.set()
                writer.submit(frames[-1])
                submitted.set()

            producer = Thread(target=submit_last_frame, daemon=True)
            try:
                for frame in frames[:-1]:
                    writer.submit(frame)
                producer.start()
                self.assertTrue(submitting.wait(5))
                self.assertFalse(submitted.wait(0.1))
                release_encoders.set()
                self.assertTrue(submitted.wait(5))
            finally:
                release_encoders.set()
                if producer.ident is not None:
                    producer.join(timeout=5)
                writer.close()
            self.assertEqual(len(list(path.rglob("*.png"))), len(frames))
