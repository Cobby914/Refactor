import csv
from io import StringIO
from struct import Struct
from threading import Lock
from types import SimpleNamespace
import unittest

import carla

from dataset.capture.CaptureRadarCameraData import process_radar_measurement_fast
from fusion.radar_csv import write_radar_scan


class RadarScan(SimpleNamespace):
    def __len__(self):
        return len(self.detections)

    def __iter__(self):
        return iter(self.detections)


class RadarCsvTests(unittest.TestCase):
    def assert_matches_legacy(self, values, *, raw_available):
        detection = Struct("=ffff")
        raw = b"".join(detection.pack(*point) for point in values)
        scan = RadarScan(
            frame=1024,
            timestamp=51.234567621,
            transform=carla.Transform(carla.Location(1.125, -0.0, -5.5), carla.Rotation(-2.75, 87, 0)),
            detections=[SimpleNamespace(velocity=v, azimuth=az, altitude=al, depth=d)
                        for v, az, al, d in detection.iter_unpack(raw)],
            raw_data=raw if raw_available else b"?",
        )
        label = 'R,"east"\r\nkerb'
        expected = StringIO(newline="")
        process_radar_measurement_fast(
            scan, 25, label, None, radar_writer=csv.writer(expected), lock=Lock(),
            counts=dict(radar_messages=0, radar_detections=0, radar_scored=0),
        )
        actual = StringIO(newline="")
        write_radar_scan(actual, scan, 25, label)
        self.assertEqual(actual.getvalue().encode("utf-8"), expected.getvalue().encode("utf-8"))

    def test_packed_returns_match_legacy_csv_byte_for_byte(self):
        self.assert_matches_legacy(
            [(-5.999123, 0.325, -0.125, 23.9999), (-0.0, -1.125, 0.0625, 0.00000049),
             (float("nan"), float("inf"), -float("inf"), 35.5)],
            raw_available=True,
        )

    def test_empty_and_iterator_fallback_match_legacy_csv(self):
        self.assert_matches_legacy([], raw_available=True)
        self.assert_matches_legacy([(-1.0, 0.125, -0.375, 12.0)], raw_available=False)
