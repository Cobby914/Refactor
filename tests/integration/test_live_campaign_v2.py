import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
from time import monotonic, sleep
import unittest
from unittest.mock import patch

from fusion.campaign import CampaignConfig, RunSpec, run_campaign


@unittest.skipUnless(os.environ.get("SCALESENSE_TEST_CARLA_PORT"), "requires a dedicated rendering CARLA server")
class LiveCampaignV2Tests(unittest.TestCase):
    def setUp(self):
        import carla

        self.host = os.environ.get("SCALESENSE_TEST_CARLA_HOST", "127.0.0.1")
        self.port = int(os.environ["SCALESENSE_TEST_CARLA_PORT"])
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.original = self.world.get_settings()
        self.assertFalse(self.original.synchronous_mode)
        self.root = Path(self.enterContext(TemporaryDirectory(prefix="scalesense-v2-campaign-test-")))
        self.children = []
        launch = subprocess.Popen

        def track_sumo(command, *args, **kwargs):
            child = launch(command, *args, **kwargs)
            if Path(command[0]).stem in ("sumo", "sumo-gui"):
                self.children.append(child)
            return child

        self.enterContext(patch("fusion.sumo_adapter.subprocess.Popen", side_effect=track_sumo))

    def config(self, **settings):
        cfg = CampaignConfig(
            engine="v2", name="campaign test", warmup_s=0.15, radars_south=2, radars_north=2,
            radar_points_per_second=800, carla_host=self.host, carla_port=self.port,
            label=False, postprocess=False, scene_cleanup=False, clear_trees=False,
            capture_base_dir=str(self.root), sync_timeout=10, runner_start_timeout_s=10,
            runs=[RunSpec(scenario=1, duration=5, seed=12), RunSpec(scenario=3, duration=5, seed=34)],
        )
        for key, value in settings.items():
            setattr(cfg, key, value)
        return cfg

    def assert_released(self, workers):
        self.assertEqual(self.client.get_world().id, self.world.id)
        self.assertEqual(self.world.get_settings(), self.original)
        actors = self.world.get_actors()
        self.assertFalse(any(
            actor.attributes.get("role_name", "").startswith(("scalesense-v2:", "scalesense-v2-sensor:"))
            for actor in actors
        ))
        self.assertFalse(any(light.is_frozen() for light in actors.filter("traffic.traffic_light*")))
        self.assertEqual(len(self.children), workers)
        for child in self.children:
            self.assertIsNotNone(child.returncode, f"unreaped SUMO process {child.pid}")

    def assert_recorded_segments(self, directory, counts):
        segments = self.read_json(directory / "segments.json")["segments"]
        manifest = self.read_jsonl(directory / "capture_frames.jsonl")
        actors = self.read_jsonl(directory / "actor_frames.jsonl")
        expected = []
        self.assertEqual([segment["recorded_frames"] for segment in segments], counts)
        for index, segment in enumerate(segments):
            self.assertEqual(segment["segment_id"], index)
            start, end = segment["start_frame"], segment["end_frame"]
            self.assertEqual(start, segment["runner_first_frame"] + 3)
            self.assertEqual(end - start + 1, counts[index])
            if index:
                self.assertGreater(start, segments[index - 1]["end_frame"] + 3)
            expected.extend((index, frame) for frame in range(start, end + 1))
        self.assertEqual([(row["segment_id"], row["frame"]) for row in manifest], expected)
        self.assertEqual([(row["segment"], row["frame"]) for row in actors], expected)
        return segments, manifest

    def test_two_consecutive_campaigns_keep_one_rig_and_exact_frame_ranges(self):
        directories = []
        for index in range(2):
            result = run_campaign(self.config(label=index == 0))
            self.assertTrue(result.ok, result.error)
            directory = Path(result.capture_dir)
            directories.append(directory)
            segments, manifest = self.assert_recorded_segments(directory, [100, 100])
            self.assertEqual([row["scenario"] for row in segments], [1, 3])
            self.assertEqual([row["seed"] for row in segments], [12, 34])
            self.assertEqual([row["step"] for row in manifest], list(range(4, 104)) * 2)
            record = self.read_json(directory / "campaign.json")
            self.assertEqual([row["status"] for row in record["rows"]], ["done", "done"])
            self.assertEqual(len({row["pid"] for row in record["runner_status"].values()}), 2)
            plan = CampaignConfig.load(directory / "campaign_plan.json")
            self.assertEqual(plan.engine, "v2")
            poses = self.read_json(directory / "camera_extrinsics.json")
            with (directory / "camera_data.csv").open(newline="", encoding="utf-8") as stream:
                camera = list(csv.DictReader(stream))
            self.assertEqual({row["sensor_id"] for row in camera}, {str(poses[0]["sensor_id"])})
            self.assertEqual([(int(row["segment_id"]), int(row["frame"])) for row in camera],
                             [(row["segment_id"], row["frame"]) for row in manifest])
            if plan.label:
                with (directory / "radar_data_labeled.csv").open(newline="", encoding="utf-8") as stream:
                    radar = list(csv.DictReader(stream))
                expected = {row["frame"]: row["segment_id"] for row in manifest}
                self.assertEqual({row["segment_id"] for row in radar}, {"0", "1"})
                for row in radar:
                    segment = expected[int(row["frame"])]
                    self.assertEqual((int(row["segment_id"]), int(row["scenario_id"])),
                                     (segment, segments[segment]["scenario"]))
            self.assert_released((index + 1) * 2)
        self.assertNotEqual(*directories)

    def test_gui_stop_retains_current_row_prefix(self):
        try:
            import tkinter as tk
        except ImportError:
            self.skipTest("requires Tk")

        cfg = self.config()
        cfg.runs[0].duration = 10
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(f"Tk could not initialize: {exc}")
        import fusion_gui

        root.withdraw()
        with patch.object(fusion_gui, "AUTOSAVE", self.root / "last_campaign.json"), \
                patch.object(fusion_gui.messagebox, "askyesno", return_value=True):
            app = fusion_gui.CampaignGUI(root)
            try:
                app._apply_config(cfg)
                app.start_btn.invoke()
                self.wait_for_gui(root, lambda: app.status[0].startswith("recording") or not app._running)
                self.assertTrue(app.status[0].startswith("recording"), app.status)
                app.stop_btn.invoke()
                self.wait_for_gui(root, lambda: not app._running)
                self.assertTrue(app.capture_state.cget("text").startswith("Capture: stopped"))
            finally:
                if app._stop_event is not None:
                    app._stop_event.set()
                if app._worker is not None:
                    app._worker.join(timeout=20)
                root.destroy()
        self.assert_stopped(next(self.root.glob("sensor_capture_*")), limit=200)
        self.assert_released(1)

    def test_cli_sigint_uses_saved_engine_and_returns_interrupted_exit_code(self):
        import run_fusion

        plan = self.root / "campaign.json"
        self.config().save(plan)

        def interrupt_campaign(cfg, stop_event):
            def interrupt(event):
                if event.get("status") == "recording":
                    signal.raise_signal(signal.SIGINT)

            return run_campaign(cfg, on_event=interrupt, stop_event=stop_event)

        old_handler = signal.getsignal(signal.SIGINT)
        with patch.object(sys, "argv", ["run_fusion.py", "--campaign", str(plan)]), \
                patch.object(run_fusion, "run_campaign", side_effect=interrupt_campaign):
            self.assertEqual(run_fusion.main(), 130)
        self.assertIs(signal.getsignal(signal.SIGINT), old_handler)
        self.assert_stopped(next(self.root.glob("sensor_capture_*")), limit=100)
        self.assert_released(1)

    def test_stalled_tick_preserves_paired_prefix_and_allows_retry(self):
        from fusion.traffic_session import TrafficRow

        tick = TrafficRow._tick
        calls = 0

        def stall(row):
            nonlocal calls
            calls += 1
            if calls > 8:
                raise TimeoutError("injected CARLA tick timeout")
            return tick(row)

        cfg = self.config()
        with patch.object(TrafficRow, "_tick", stall):
            failed = run_campaign(cfg)
        self.assertEqual(calls, 9)
        self.assertFalse(failed.ok)
        self.assertFalse(failed.stopped)
        self.assertIn("injected CARLA tick timeout", failed.error)
        self.assertEqual([row.status for row in failed.rows], ["failed", "skipped"])
        self.assert_recorded_segments(Path(failed.capture_dir), [5])
        self.assert_released(1)
        retry = run_campaign(cfg)
        self.assertTrue(retry.ok, retry.error)
        self.assert_recorded_segments(Path(retry.capture_dir), [100, 100])
        self.assert_released(3)

    def assert_stopped(self, directory, *, limit):
        campaign = self.read_json(directory / "campaign.json")
        self.assertTrue(campaign["stopped"])
        self.assertFalse(campaign["ok"])
        self.assertEqual([row["status"] for row in campaign["rows"]], ["stopped", "skipped"])
        count = campaign["rows"][0]["recorded_frames"]
        self.assertTrue(0 < count < limit)
        segments, _ = self.assert_recorded_segments(directory, [count])
        self.assertEqual(segments[0]["status"], "truncated")
        self.assertEqual(self.read_json(directory / "run_meta.json")["capture_status"], "interrupted")

    def wait_for_gui(self, root, condition):
        deadline = monotonic() + 30
        while not condition():
            self.assertLess(monotonic(), deadline, "timed out waiting for the campaign GUI")
            root.update()
            sleep(0.01)

    @staticmethod
    def read_json(path):
        return json.loads(path.read_text(encoding="utf-8"))

    @staticmethod
    def read_jsonl(path):
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
