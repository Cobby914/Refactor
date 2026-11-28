from contextlib import redirect_stdout
import csv
from io import StringIO
import json
import os
from pathlib import Path
import signal
from tempfile import TemporaryDirectory
from threading import Lock
import unittest
from unittest.mock import patch

from fusion.traffic_session import run_traffic
from traffic.validate_scenarios import build_files


@unittest.skipUnless(os.environ.get("SCALESENSE_TEST_CARLA_PORT"), "requires a dedicated rendering CARLA server")
class LiveCaptureSessionTests(unittest.TestCase):
    def setUp(self):
        import carla

        self.host = os.environ.get("SCALESENSE_TEST_CARLA_HOST", "127.0.0.1")
        self.port = int(os.environ["SCALESENSE_TEST_CARLA_PORT"])
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.original = self.world.get_settings()
        self.assertFalse(self.original.synchronous_mode)
        self.root = Path(self.enterContext(TemporaryDirectory(prefix="scalesense-v2-capture-test-")))
        self.config = Path(build_files(
            scenario_id=1, density=60, duration=4, direction="BOTH", ambient_vehicles=0,
            pedestrians=0, bicycles=0, seed=42, step_length=0.05, temp_dir=self.root,
        )[0])

    def capture(self, directory, radar_count, **kwargs):
        from fusion.capture_session import CaptureOptions

        return run_traffic(
            self.config, steps=32, carla_host=self.host, carla_port=self.port,
            capture_options=CaptureOptions(directory, radar_count // 2, radar_count // 2, radar_pps=800),
            **kwargs,
        )

    def assert_released(self):
        self.assertEqual(self.world.get_settings(), self.original)
        self.assertFalse(any(
            actor.attributes.get("role_name", "").startswith(("scalesense-v2-sensor:", "scalesense-v2:"))
            for actor in self.world.get_actors()
        ))

    def test_four_radar_layouts_write_exact_frames_and_legacy_files(self):
        from PIL import Image
        from dataset.capture.CaptureRadarCameraData import process_radar_measurement_fast
        from dataset.capture.LabelRadarCapture import label_radar_capture_dir
        from fusion import radar_csv

        write_scan = radar_csv.write_radar_scan
        checked = set()

        def compare_first_scan(stream, measurement, sensor_id, label):
            if label not in checked and len(measurement):
                self.assertEqual(len(measurement.raw_data), 16 * len(measurement))
                reference = StringIO(newline="")
                process_radar_measurement_fast(
                    measurement, sensor_id, label, None,
                    radar_writer=csv.writer(reference), lock=Lock(),
                    counts=dict(radar_messages=0, radar_detections=0, radar_scored=0),
                )
                self.assertEqual(radar_csv.format_radar_scan(measurement, sensor_id, label), reference.getvalue())
                checked.add(label)
            write_scan(stream, measurement, sensor_id, label)

        for count in (4, 8, 12, 14):
            with self.subTest(radars=count):
                directory = self.root / f"sensor_capture_v2_{count}"
                checked.clear()
                with patch.object(radar_csv, "write_radar_scan", side_effect=compare_first_scan):
                    run = self.capture(directory, count)
                frames = self.read_jsonl(directory / "capture_frames.jsonl")
                ground_truth = self.read_jsonl(directory / "actor_frames.jsonl")
                expected_ids = list(range(run.final_frame.carla_frame - 31, run.final_frame.carla_frame + 1))
                self.assertEqual([row["frame"] for row in frames], expected_ids)
                self.assertEqual([row["frame"] for row in ground_truth], expected_ids)
                self.assertEqual([row["step"] for row in frames], list(range(1, 33)))
                self.assertEqual(frames[-1]["sumo_time_s"], 1.6)
                self.assertTrue(any(row["actors"] for row in ground_truth))
                for row in ground_truth:
                    for actor in row["actors"]:
                        self.assertEqual(set(actor), {
                            "id", "kind", "type_id", "class_label", "location", "rotation", "bbox",
                        })

                meta = self.read_json(directory / "run_meta.json")
                label_set = {"C10", *(f"R{index}" for index in range(1, count + 1))}
                self.assertEqual(checked, label_set - {"C10"})
                self.assertEqual(meta["radar_count"], count)
                self.assertEqual(meta["camera_count"], 1)
                self.assertEqual(meta["capture_status"], "complete")
                self.assertEqual(meta["frame_sync"]["received"], dict.fromkeys(label_set, 32))
                self.assertTrue(all(set(row["received"]) == label_set and not row["missing"] for row in frames))

                radar_poses = self.read_json(directory / "radar_extrinsics.json")
                self.assertEqual(radar_poses, self.read_json(directory / "sensor_extrinsics.json"))
                self.assertEqual(len(radar_poses), count)
                radar_ids = {row["sensor_label"]: str(row["sensor_id"]) for row in radar_poses}
                with (directory / "radar_data.csv").open(newline="", encoding="utf-8") as stream:
                    radar = list(csv.DictReader(stream))
                self.assertEqual({row["sensor_label"] for row in radar}, label_set - {"C10"})
                self.assertTrue(all(row["sensor_id"] == radar_ids[row["sensor_label"]] for row in radar))
                self.assertLessEqual({int(row["frame"]) for row in radar}, set(expected_ids))
                label_log = StringIO()
                with redirect_stdout(label_log):
                    labeled = label_radar_capture_dir(directory, progress_every=0)
                self.assertIn("missing actor frames: 0", label_log.getvalue())
                with labeled.open(newline="", encoding="utf-8") as stream:
                    self.assertEqual(sum(1 for _ in csv.DictReader(stream)), len(radar))

                camera_poses = self.read_json(directory / "camera_extrinsics.json")
                self.assertEqual([row["sensor_label"] for row in camera_poses], ["C10"])
                with (directory / "camera_data.csv").open(newline="", encoding="utf-8") as stream:
                    camera = list(csv.DictReader(stream))
                self.assertEqual([int(row["frame"]) for row in camera], expected_ids)
                actor_ids = {row["frame"]: {str(actor["id"]) for actor in row["actors"]} for row in ground_truth}
                for row, frame_id in zip(camera, expected_ids):
                    self.assertEqual(row["sensor_id"], str(camera_poses[0]["sensor_id"]))
                    path = Path(row["image_path"])
                    self.assertEqual(path.parent.parent, directory / "camera_frames")
                    self.assertEqual(path.name, f"frame_{frame_id:08d}.png")
                    if row["nearest_actor_id"]:
                        self.assertIn(row["nearest_actor_id"], actor_ids[frame_id])
                    with Image.open(path) as image:
                        self.assertEqual(image.size, (int(row["width"]), int(row["height"])))
                        image.verify()
                self.assert_released()

    def test_interrupted_capture_keeps_confirmed_frames_and_releases_rig(self):
        def stop(frame):
            if frame.step == 3:
                signal.raise_signal(signal.SIGINT)

        directory = self.root / "sensor_capture_interrupted"
        with self.assertRaises(KeyboardInterrupt):
            self.capture(directory, 4, on_frame=stop)
        self.assertEqual(self.read_json(directory / "run_meta.json")["capture_status"], "interrupted")
        self.assertEqual([row["step"] for row in self.read_jsonl(directory / "capture_frames.jsonl")], [1, 2, 3])
        self.assert_released()

    @staticmethod
    def read_json(path):
        return json.loads(path.read_text(encoding="utf-8"))

    @staticmethod
    def read_jsonl(path):
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
