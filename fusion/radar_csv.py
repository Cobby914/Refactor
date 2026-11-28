"""Format a complete CARLA radar scan in the dataset CSV schema."""

import csv
from io import StringIO
from struct import Struct


# CARLA serializes each detection as native float32 velocity, azimuth,
# altitude and depth, in that order. The dataset columns start with depth.
_RADAR_DETECTION = Struct("=ffff")
_UNLABELED_COLUMNS = ("",) * 10 + ("0", "1", "")


def format_radar_scan(measurement, sensor_id, sensor_label):
    count = len(measurement)
    if not count:
        return ""

    # Keep CSV quoting for sensor labels. These fields and the pose are shared
    # by every return in the scan.
    prefix = StringIO(newline="")
    csv.writer(prefix).writerow((sensor_id, sensor_label, measurement.frame, f"{measurement.timestamp:.6f}", ""))
    leading_fields = prefix.getvalue().removesuffix("\r\n")
    transform = measurement.transform
    loc, rot = transform.location, transform.rotation
    pose = (loc.x, loc.y, loc.z, rot.pitch, rot.yaw, rot.roll)
    pose_fields = tuple(f"{value:.6f}" for value in pose)
    trailing_fields = "," + ",".join(pose_fields + _UNLABELED_COLUMNS) + "\r\n"

    raw = getattr(measurement, "raw_data", None)
    if raw is not None and len(raw) == count * _RADAR_DETECTION.size:
        detections = _RADAR_DETECTION.iter_unpack(raw)
    else:
        detections = ((d.velocity, d.azimuth, d.altitude, d.depth) for d in measurement)
    return "".join(
        f"{leading_fields}{index},{depth:.6f},{azimuth:.6f},{altitude:.6f},{velocity:.6f}{trailing_fields}"
        for index, (velocity, azimuth, altitude, depth) in enumerate(detections)
    )


def write_radar_scan(stream, measurement, sensor_id, sensor_label):
    stream.write(format_radar_scan(measurement, sensor_id, sensor_label))
