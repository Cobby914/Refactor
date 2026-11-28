from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import carla
from traci.exceptions import TraCIException

from fusion.actor_mirror import ActorMirror


class ActorMirrorTests(unittest.TestCase):
    def test_crossing_actor_waits_then_catches_up_without_changing_id(self):
        connection = Mock()
        session = Mock()
        session.fixed_delta_s = 0.05
        with patch("fusion.actor_mirror.CarlaSyncManager"):
            mirror = ActorMirror(session, connection, seed=42, render_radius=None)
        car, person = ("vehicle", "car"), ("walker", "person")
        mirror._actors.update({car: 1, person: 2})
        mirror._boxes.update({
            car: carla.BoundingBox(carla.Location(), carla.Vector3D(2, 1, 1)),
            person: carla.BoundingBox(carla.Location(), carla.Vector3D(0.3, 0.3, 1)),
        })
        start = {1: carla.Transform(), 2: carla.Transform(carla.Location(y=-2))}
        target = {car: start[1], person: carla.Transform(carla.Location(y=2))}
        snapshot = Mock()
        snapshot.find.side_effect = lambda actor_id: SimpleNamespace(get_transform=lambda: start[actor_id])

        updates = mirror._plan_updates(snapshot, target, set())
        self.assertEqual([key for key, _ in updates], [car])
        self.assertEqual(mirror.actor_ids, {car: 1, person: 2})

        start[1] = carla.Transform(carla.Location(x=10))
        target[car] = start[1]
        updates = dict(mirror._plan_updates(snapshot, target, set()))
        self.assertEqual(updates[person].actor_id, 2)
        self.assertAlmostEqual(updates[person].transform.location.y, -1.9, places=5)

    def test_actor_error_destroys_only_failed_actor_and_suppresses_respawn(self):
        for failure in ("position", "transform", "command"):
            with self.subTest(failure=failure):
                self.check_actor_error(failure)

    def check_actor_error(self, failure):
        connection = Mock()
        connection.vehicle.getIDList.return_value = ("failed", "healthy")
        connection.person.getIDList.return_value = ()
        session = Mock()
        session.world.get_snapshot.return_value.has_actor.return_value = True
        snapshot = session.world.get_snapshot.return_value
        snapshot.find.side_effect = lambda actor_id: SimpleNamespace(
            get_transform=lambda: carla.Transform(carla.Location(y=10 * actor_id))
        )
        with patch("fusion.actor_mirror.CarlaSyncManager") as manager:
            mirror = ActorMirror(session, connection, seed=42, render_radius=None)
        geometry = manager.return_value
        failed = ("vehicle", "failed")
        healthy = ("vehicle", "healthy")
        mirror._actors.update({failed: 1, healthy: 2})
        mirror._boxes.update({
            key: carla.BoundingBox(carla.Location(), carla.Vector3D(2, 1, 1))
            for key in (failed, healthy)
        })
        mirror._visible.update((failed, healthy))

        def position(sumo_id):
            if failure == "position" and sumo_id == "failed":
                raise TraCIException("position unavailable")
            return (0.0, 0.0)

        def transform(sumo_id, *, position):
            x = float("nan") if failure == "transform" and sumo_id == "failed" else 0.0
            return carla.Transform(carla.Location(x=x, y=10 if sumo_id == "failed" else 20))

        def apply(commands, do_tick):
            for command in commands:
                if (failure == "command" and isinstance(command, carla.command.ApplyTransform)
                        and command.actor_id == 1):
                    yield SimpleNamespace(error="transform rejected")
                else:
                    yield SimpleNamespace(error="")

        connection.vehicle.getPosition.side_effect = position
        geometry.vehicle_transform.side_effect = transform
        session.client.apply_batch_sync.side_effect = lambda commands, do_tick: list(apply(commands, do_tick))

        with self.assertLogs("fusion.actor_mirror", level="WARNING") as captured:
            self.assertEqual(mirror.step(), 2)
        self.assertIn("vehicle failed", captured.output[0])
        self.assertEqual(mirror.actor_ids, {healthy: 2})
        geometry.forget_actor_geometry.assert_called_once_with(*failed)
        self.assertEqual(
            [
                command.actor_id
                for call in session.client.apply_batch_sync.call_args_list
                for command in call.args[0]
                if isinstance(command, carla.command.DestroyActor)
            ],
            [1],
        )

        session.client.apply_batch_sync.reset_mock()
        connection.vehicle.getPosition.reset_mock(side_effect=True)
        connection.vehicle.getPosition.return_value = (0.0, 0.0)
        self.assertEqual(mirror.step(), 2)
        connection.vehicle.getPosition.assert_called_once_with("healthy")
        commands, do_tick = session.client.apply_batch_sync.call_args.args
        self.assertFalse(do_tick)
        self.assertEqual(len(commands), 1)
        self.assertIsInstance(commands[0], carla.command.ApplyTransform)
        self.assertEqual(commands[0].actor_id, 2)
