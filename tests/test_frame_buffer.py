import unittest

from fusion.frame_buffer import FrameBuffer
from fusion.simulation_engine import SimulationFrame


class FrameBufferTests(unittest.TestCase):
    def setUp(self):
        self.buffer = FrameBuffer(100, ("R1", "C10"), max_lag_frames=2)

    def confirm(self, number):
        step = number - 99
        return self.buffer.confirm(SimulationFrame(step, step * 0.05, number), [{"id": number}])

    def test_joins_early_radar_and_out_of_order_camera_to_exact_ground_truth(self):
        self.buffer.offer("R1", 100, [])
        self.buffer.offer("R1", 100, ["duplicate"])
        self.assertEqual(self.confirm(100), [])
        self.buffer.offer("R1", 101, ["radar 101"])
        self.buffer.offer("C10", 101, "camera 101")
        self.assertEqual(self.confirm(101), [])
        self.buffer.offer("C10", 100, "camera 100")

        first, second = self.buffer.pop_ready()
        self.assertEqual(first.actors, [{"id": 100}])
        self.assertEqual(first.measurements, {"R1": [], "C10": "camera 100"})
        self.assertEqual(second.actors, [{"id": 101}])
        self.assertEqual(second.measurements["C10"], "camera 101")
        self.assertEqual(self.buffer.stats()["ignored"], {"duplicate": 1})

    def test_expired_missing_camera_never_matches_a_neighbor_or_late_sample(self):
        self.buffer.offer("R1", 100, "radar 100")
        self.confirm(100)
        self.confirm(101)
        completed, = self.confirm(102)

        self.assertEqual(completed.status, "expired")
        self.assertEqual(completed.missing, ("C10",))
        self.assertEqual(completed.measurements, {"R1": "radar 100"})
        self.buffer.offer("C10", 100, "late camera")
        self.assertEqual(self.buffer.stats()["ignored"], {"late": 1})

    def test_shutdown_keeps_delayed_last_frame_and_rejects_unconfirmed_tail(self):
        self.buffer.offer("C10", 99, "warm-up")
        self.buffer.offer("R1", 100, "radar 100")
        self.confirm(100)
        self.confirm(101)
        self.buffer.offer("C10", 102, "unconfirmed")
        self.buffer.end_recording()
        self.buffer.offer("R1", 102, "maintenance tick")
        self.buffer.offer("C10", 101, "camera 101")

        ended = self.buffer.finish()
        self.assertEqual([item.simulation.carla_frame for item in ended], [100, 101])
        self.assertEqual([item.status for item in ended], ["shutdown", "shutdown"])
        self.assertEqual(ended[1].measurements, {"C10": "camera 101"})
        self.assertEqual(self.buffer.stats()["ignored"], {
            "before_recording": 1, "outside_recording": 1,
        })
        self.assertFalse(self.buffer.pending)

    def test_rejects_discontinuous_ground_truth(self):
        with self.assertRaisesRegex(ValueError, "unexpected confirmed"):
            self.confirm(101)
