from threading import Event, Thread
import unittest
from unittest.mock import Mock, call

from fusion.simulation_engine import SimulationEngine, SimulationFrame, SimulationStepError


class SimulationEngineTests(unittest.TestCase):
    def setUp(self):
        self.operations = Mock()
        self.operations.advance_sumo.side_effect = [10.05, 10.10]
        self.operations.tick_carla.side_effect = [501, 502]

    def make_engine(self, **baselines):
        settings = {
            "fixed_delta_s": 0.05,
            "initial_sumo_time_s": 10.0,
            "initial_carla_frame": 500,
            **baselines,
        }
        return SimulationEngine(
            advance_sumo=self.operations.advance_sumo,
            sync_traffic=self.operations.sync_traffic,
            tick_carla=self.operations.tick_carla,
            **settings,
        )

    def test_pairs_two_steps_in_order(self):
        engine = self.make_engine()

        self.assertEqual(engine.step(), SimulationFrame(1, 10.05, 501))
        self.assertEqual(engine.step(), SimulationFrame(2, 10.10, 502))
        self.assertEqual(self.operations.mock_calls, [
            call.advance_sumo(), call.sync_traffic(), call.tick_carla(),
            call.advance_sumo(), call.sync_traffic(), call.tick_carla(),
        ])

    def test_rejects_invalid_baselines(self):
        for settings in (
            {"fixed_delta_s": 0},
            {"fixed_delta_s": float("nan")},
            {"initial_sumo_time_s": float("inf")},
            {"initial_carla_frame": -1},
        ):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                self.make_engine(**settings)
        self.assertEqual(self.operations.mock_calls, [])

    def test_traffic_and_carla_do_not_advance_on_sumo_clock_mismatch(self):
        for actual_time_s in (10.0, 10.1, float("nan")):
            with self.subTest(actual_time_s=actual_time_s):
                self.operations.reset_mock()
                self.operations.advance_sumo.side_effect = [actual_time_s]
                engine = self.make_engine()

                with self.assertRaisesRegex(SimulationStepError, "expected SUMO time"):
                    engine.step()
                with self.assertRaisesRegex(SimulationStepError, "start a new session"):
                    engine.step()
                self.assertEqual(self.operations.mock_calls, [call.advance_sumo()])

    def test_duplicate_and_skipped_carla_frames_invalidate_the_session(self):
        for unexpected_frame in (501, 503):
            with self.subTest(unexpected_frame=unexpected_frame):
                self.operations.reset_mock()
                self.operations.advance_sumo.side_effect = [10.05, 10.10]
                self.operations.tick_carla.side_effect = [501, unexpected_frame]
                engine = self.make_engine()
                engine.step()

                with self.assertRaisesRegex(SimulationStepError, "expected CARLA frame 502"):
                    engine.step()
                with self.assertRaisesRegex(SimulationStepError, "start a new session"):
                    engine.step()
                self.assertEqual(self.operations.advance_sumo.call_count, 2)
                self.assertEqual(self.operations.tick_carla.call_count, 2)

    def test_traffic_failure_stops_before_carla_tick(self):
        self.operations.sync_traffic.side_effect = RuntimeError("mirror rejected")
        engine = self.make_engine()

        with self.assertRaisesRegex(SimulationStepError, "traffic synchronization") as error:
            engine.step()
        self.assertIsInstance(error.exception.__cause__, RuntimeError)
        self.assertEqual(self.operations.mock_calls, [call.advance_sumo(), call.sync_traffic()])

    def test_carla_timeout_does_not_repeat_a_partial_step(self):
        self.operations.tick_carla.side_effect = TimeoutError("RPC did not return")
        engine = self.make_engine()

        with self.assertRaisesRegex(SimulationStepError, "CARLA tick"):
            engine.step()
        with self.assertRaisesRegex(SimulationStepError, "start a new session"):
            engine.step()
        self.assertEqual(self.operations.mock_calls, [
            call.advance_sumo(), call.sync_traffic(), call.tick_carla(),
        ])

    def test_interrupt_propagates_and_invalidates_the_session(self):
        self.operations.advance_sumo.side_effect = KeyboardInterrupt
        engine = self.make_engine()

        with self.assertRaises(KeyboardInterrupt):
            engine.step()
        with self.assertRaisesRegex(SimulationStepError, "start a new session"):
            engine.step()
        self.assertEqual(self.operations.mock_calls, [call.advance_sumo()])

    def test_concurrent_call_is_rejected_without_advancing_either_simulator(self):
        started = Event()
        release = Event()
        results = []

        def advance_sumo():
            started.set()
            if not release.wait(timeout=5):
                raise TimeoutError("test did not release the first step")
            return 10.05

        self.operations.advance_sumo.side_effect = advance_sumo
        engine = self.make_engine()
        worker = Thread(target=lambda: results.append(engine.step()))
        worker.start()
        try:
            self.assertTrue(started.wait(timeout=5))
            with self.assertRaisesRegex(SimulationStepError, "already running"):
                engine.step()
        finally:
            release.set()
            worker.join(timeout=5)

        self.assertFalse(worker.is_alive())
        self.assertEqual(results, [SimulationFrame(1, 10.05, 501)])
        self.assertEqual(self.operations.mock_calls, [
            call.advance_sumo(), call.sync_traffic(), call.tick_carla(),
        ])
