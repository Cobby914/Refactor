import unittest

import carla

from fusion.traffic_light_mirror import signal_aspect


class SignalAspectTests(unittest.TestCase):
    def test_reduces_protected_approach_and_permissive_turns(self):
        for movements, aspect in {
            "Ggr": "Green",
            "ggr": "Red",
            "ggg": "Green",
            "rry": "Yellow",
            "rru": "Red",
            "Oo": "Off",
        }.items():
            with self.subTest(movements=movements):
                self.assertEqual(signal_aspect(movements), getattr(carla.TrafficLightState, aspect))

    def test_empty_approach_is_reported(self):
        with self.assertRaises(ValueError):
            signal_aspect("")
