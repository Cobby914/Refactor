import os
from pathlib import Path
import signal
import subprocess
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

from fusion.carla_adapter import open_carla_session
from fusion.traffic_session import run_traffic
from traffic.carla_sync import BIKE_BLUEPRINTS
from traffic.validate_scenarios import build_files


@unittest.skipUnless(os.environ.get("SCALESENSE_TEST_CARLA_PORT"), "requires a dedicated CARLA server")
class LiveTrafficSessionTests(unittest.TestCase):
    def setUp(self):
        import carla

        self.host = os.environ.get("SCALESENSE_TEST_CARLA_HOST", "127.0.0.1")
        self.port = int(os.environ["SCALESENSE_TEST_CARLA_PORT"])
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.original_settings = self.world.get_settings()
        self.assertFalse(self.original_settings.synchronous_mode)
        temp_dir = self.enterContext(TemporaryDirectory(prefix="scalesense-v2-test-"))
        self.config = Path(build_files(
            scenario_id=1, density=60, duration=10, direction="BOTH", ambient_vehicles=0,
            pedestrians=0, bicycles=0, seed=42, step_length=0.05, temp_dir=temp_dir,
        )[0])
        self.processes = []
        original_popen = subprocess.Popen

        def track_process(*args, **kwargs):
            process = original_popen(*args, **kwargs)
            self.processes.append(process)
            return process

        self.enterContext(patch("fusion.sumo_adapter.subprocess.Popen", side_effect=track_process))

    def run_session(self, *, steps=200, **kwargs):
        return run_traffic(
            self.config, steps=steps, carla_host=self.host, carla_port=self.port,
            ready_timeout_s=10, rpc_timeout_s=5, **kwargs,
        )

    def assert_session_released(self, expected_children):
        self.assertEqual(self.client.get_world().id, self.world.id)
        self.assertEqual(self.world.get_settings(), self.original_settings)
        actors = self.world.get_actors()
        self.assertFalse(any(
            actor.attributes.get("role_name", "").startswith("scalesense-v2:")
            for actor in actors
        ))
        self.assertFalse(any(light.is_frozen() for light in actors.filter("traffic.traffic_light*")))
        self.assertEqual(len(self.processes), expected_children)
        for process in self.processes:
            self.assertIsNotNone(process.returncode, f"unreaped SUMO process {process.pid}")
            self.assertIsNotNone(process.poll(), f"live SUMO process {process.pid}")

    def test_interrupted_session_releases_resources_and_repeats_on_same_world(self):
        def interrupt(frame):
            self.assertEqual(frame.step, 1)
            self.assertTrue(self.world.get_settings().synchronous_mode)
            signal.raise_signal(signal.SIGINT)

        with self.assertRaises(KeyboardInterrupt):
            self.run_session(on_frame=interrupt)
        self.assert_session_released(1)

        first_frames = []
        second_frames = []
        first = self.run_session(on_frame=first_frames.append)
        self.assert_session_released(2)
        second = self.run_session(on_frame=second_frames.append)
        self.assert_session_released(3)

        self.assertEqual(len(first_frames), 200)
        self.assertEqual(len(second_frames), 200)
        for frames in (first_frames, second_frames):
            self.assertEqual(
                [frame.carla_frame - frames[0].carla_frame for frame in frames],
                list(range(200)),
            )
            for index, frame in enumerate(frames, 1):
                self.assertAlmostEqual(frame.sumo_time_s, index * 0.05, places=6)
        self.assertGreater(first.peak_sumo_vehicles, 0)
        self.assertEqual(first.peak_sumo_vehicles, second.peak_sumo_vehicles)

    def test_carla_startup_failure_after_apply_restores_world_and_reaps_sumo(self):
        startup_world = Mock(wraps=self.world)
        get_settings = self.world.get_settings

        def check_settings():
            settings = get_settings()
            if startup_world.get_settings.call_count == 3:
                self.assertTrue(settings.synchronous_mode)
                raise RuntimeError("injected startup verification failure")
            return settings

        startup_world.get_settings.side_effect = check_settings
        with patch("fusion.carla_adapter._wait_for_world", return_value=startup_world):
            with self.assertRaisesRegex(RuntimeError, "injected startup verification failure"):
                self.run_session(steps=1)
        self.assert_session_released(1)
        self.run_session(steps=1)
        self.assert_session_released(2)

    def test_bad_sumo_config_reaps_child_before_carla_is_configured(self):
        invalid_config = self.config.with_name("invalid.sumocfg")
        invalid_config.write_text("<configuration><input>", encoding="utf-8")
        self.config = invalid_config

        with self.assertRaisesRegex(RuntimeError, "SUMO exited during startup|SUMO TraCI session failed"):
            self.run_session(steps=1)
        self.assert_session_released(1)

    def test_refuses_world_already_in_synchronous_mode(self):
        with open_carla_session(self.host, self.port, 0.05, ready_timeout_s=10, rpc_timeout_s=5):
            with self.assertRaisesRegex(RuntimeError, "CARLA is already synchronous"):
                self.run_session(steps=1)
            self.assertTrue(self.world.get_settings().synchronous_mode)
        self.assert_session_released(1)

    def test_alternating_fixed_deltas_confirm_a_consecutive_startup_tick(self):
        for delta in (0.025, 0.05) * 3:
            with self.subTest(delta=delta), open_carla_session(
                self.host, self.port, delta, ready_timeout_s=10, rpc_timeout_s=5,
            ) as session:
                self.assertEqual(session.tick(), session.initial_frame + 1)
                self.assertAlmostEqual(session.world.get_snapshot().timestamp.delta_seconds, delta, places=6)
            self.assert_session_released(0)

    def test_cars_bicycles_and_walkers_move_as_corridor_aspects_change(self):
        self.config = Path(build_files(
            scenario_id=1, density=60, duration=40, direction="BOTH", ambient_vehicles=3,
            pedestrians=12, bicycles=8, seed=42, step_length=0.05, temp_dir=self.config.parent,
        )[0])
        origins = {}
        moved = set()
        aspects = set()
        traffic_lights = sorted(
            self.world.get_actors().filter("traffic.traffic_light*"), key=lambda light: light.id,
        )

        def observe(frame):
            if frame.step % 10:
                return
            for actor in self.world.get_actors():
                role = actor.attributes.get("role_name", "")
                if not role.startswith("scalesense-v2:"):
                    continue
                category = "car"
                if actor.type_id.startswith("walker."):
                    category = "walker"
                elif actor.type_id in BIKE_BLUEPRINTS:
                    category = "bicycle"
                position = actor.get_location()
                start = origins.setdefault(actor.id, position)
                if position.distance(start) > 0.25:
                    moved.add(category)
            aspects.add(tuple(str(light.get_state()) for light in traffic_lights))

        result = self.run_session(steps=800, render_radius=None, on_frame=observe)
        self.assertGreater(result.peak_carla_actors, 3)
        self.assertEqual(moved, {"car", "bicycle", "walker"})
        self.assertGreater(len(aspects), 1)
        self.assert_session_released(1)
