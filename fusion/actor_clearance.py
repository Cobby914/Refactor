"""Check swept CARLA mirror bounds before applying a pose."""

from math import hypot, radians, sqrt

import carla


def _hull(points):
    def half(ordered):
        result = []
        for point in ordered:
            while len(result) >= 2:
                (ax, ay), (bx, by) = result[-2:]
                if (bx - ax) * (point[1] - ay) > (by - ay) * (point[0] - ax):
                    break
                result.pop()
            result.append(point)
        return result

    unique = sorted(set(points))
    return tuple(half(unique)[:-1] + half(reversed(unique))[:-1]) if len(unique) > 1 else tuple(unique)


class Footprint:
    def __init__(self, points, bottom, top):
        self.points = _hull(points)
        self.bottom, self.top = bottom, top
        xs, ys = zip(*self.points)
        self.x_min, self.x_max = min(xs), max(xs)
        self.y_min, self.y_max = min(ys), max(ys)

    @classmethod
    def at(cls, box, transform):
        corners = box.get_world_vertices(transform)
        return cls([(p.x, p.y) for p in corners], min(p.z for p in corners), max(p.z for p in corners))

    def swept_to(self, target):
        return Footprint(self.points + target.points, min(self.bottom, target.bottom), max(self.top, target.top))

    def conflicts(self, other, margin):
        if (self.top + margin < other.bottom or other.top + margin < self.bottom
                or self.x_max + margin < other.x_min or other.x_max + margin < self.x_min
                or self.y_max + margin < other.y_min or other.y_max + margin < self.y_min):
            return False
        # A separating axis from either polygon proves that the buffered bounds are clear.
        for points in (self.points, other.points):
            for (ax, ay), (bx, by) in zip(points, points[1:] + points[:1]):
                dx, dy = by - ay, ax - bx
                padding = margin * hypot(dx, dy)
                projected = [x * dx + y * dy for x, y in self.points]
                obstacle = [x * dx + y * dy for x, y in other.points]
                if max(projected) + padding < min(obstacle) or max(obstacle) + padding < min(projected):
                    return False
        return True


def _angle_delta(start, end):
    return (end - start + 180) % 360 - 180


def rotation_padding(box, start, end):
    """Bound the corner travel between the endpoint orientations."""
    radius = box.location.length() + box.extent.length()
    angle = sum(abs(_angle_delta(getattr(start, name), getattr(end, name))) for name in ("pitch", "yaw", "roll"))
    return radius * radians(angle) / 2


def catch_up(current, desired, max_distance):
    """Limit movement after a hold so clearing a boundary doesn't cause a jump."""
    delta = desired.location - current.location
    distance = sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z)
    if distance <= max_distance:
        return desired
    scale = max_distance / distance
    start, end = current.rotation, desired.rotation
    return carla.Transform(
        current.location + delta * scale,
        carla.Rotation(
            pitch=start.pitch + scale * _angle_delta(start.pitch, end.pitch),
            yaw=start.yaw + scale * _angle_delta(start.yaw, end.yaw),
            roll=start.roll + scale * _angle_delta(start.roll, end.roll),
        ),
    )
