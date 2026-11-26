from dataclasses import replace
import os
import unittest
from unittest.mock import Mock

import carla

from fusion.actor_mirror import ActorMirror
from fusion.carla_adapter import open_carla_session
from traffic.carla_sync import OFFSET_X, OFFSET_Y


@unittest.skipUnless(os.environ.get("SCALESENSE_TEST_CARLA_PORT"), "requires a dedicated CARLA server")
class LiveActorMirrorTests(unittest.TestCase):
    def open_session(self):
        host = os.environ.get("SCALESENSE_TEST_CARLA_HOST", "127.0.0.1")
        port = int(os.environ["SCALESENSE_TEST_CARLA_PORT"])
        return open_carla_session(host, port, 0.05, ready_timeout_s=10, rpc_timeout_s=5)

    def vehicle_connection(self, origin, actor_id):
        connection = Mock()
        connection.vehicle.getIDList.return_value = (actor_id,)
        connection.person.getIDList.return_value = ()
        connection.vehicle.getAngle.return_value = 13.0
        connection.vehicle.getLaneID.return_value = ":189_1_0"
        connection.vehicle.getTypeID.return_value = "car"
        connection.vehicle.getPosition.return_value = (origin.x + OFFSET_X, OFFSET_Y - origin.y)
        return connection

    def test_cull_reenter_and_arrival_preserve_unrelated_vehicle(self):
        with self.open_session() as session:
            world = session.world
            waypoint = world.get_map().get_waypoint(carla.Location(-28.8, 13.1, 1))
            origin = waypoint.transform.location
            actor_id = "test-vehicle"
            connection = self.vehicle_connection(origin, actor_id)

            blueprint = world.get_blueprint_library().find("vehicle.tesla.model3")
            blueprint.set_attribute("role_name", "actor-mirror-test-unrelated")
            unrelated = world.spawn_actor(
                blueprint, carla.Transform(carla.Location(origin.x, origin.y, origin.z + 8)),
            )
            try:
                mirror = ActorMirror(
                    session, connection, seed=42, render_radius=5, center=(origin.x, origin.y),
                )
                try:
                    key = ("vehicle", actor_id)

                    def place(offset):
                        connection.vehicle.getPosition.return_value = (
                            origin.x + offset + OFFSET_X, OFFSET_Y - origin.y,
                        )
                        mirror.step()
                        session.tick()

                    place(0)
                    first = mirror.actor_ids[key]
                    place(1)
                    self.assertEqual(mirror.actor_ids[key], first)
                    pose = world.get_snapshot().find(first).get_transform()
                    self.assertAlmostEqual(pose.location.x, origin.x + 1, places=3)
                    self.assertAlmostEqual(pose.location.y, origin.y, places=3)
                    self.assertAlmostEqual(pose.rotation.yaw, -77, places=2)

                    place(10)
                    self.assertEqual(mirror.actor_ids, {})
                    self.assertFalse(world.get_snapshot().has_actor(first))
                    place(0)
                    second = mirror.actor_ids[key]
                    self.assertNotEqual(first, second)
                    connection.vehicle.getIDList.return_value = ()
                    mirror.step()
                    session.tick()
                    self.assertFalse(world.get_snapshot().has_actor(second))

                    connection.vehicle.getIDList.return_value = (actor_id,)
                    place(0)
                    third = mirror.actor_ids[key]
                finally:
                    mirror.close()
                session.tick()
                self.assertFalse(world.get_snapshot().has_actor(third))
                self.assertTrue(world.get_snapshot().has_actor(unrelated.id))
            finally:
                unrelated.destroy()

    def test_cleanup_recovers_spawn_with_lost_batch_response(self):
        with self.open_session() as session:
            waypoint = session.world.get_map().get_waypoint(carla.Location(-28.8, 13.1, 1))
            origin = waypoint.transform.location
            connection = self.vehicle_connection(origin, "interrupted-spawn")
            client = Mock(wraps=session.client)
            spawned_ids = []

            def lose_first_response(commands, do_tick):
                self.assertFalse(do_tick)
                responses = session.client.apply_batch_sync(commands, do_tick)
                if not spawned_ids:
                    self.assertEqual(len(responses), 1)
                    self.assertFalse(responses[0].error)
                    spawned_ids.append(responses[0].actor_id)
                    raise TimeoutError("injected lost spawn response")
                return responses

            client.apply_batch_sync.side_effect = lose_first_response
            mirror = ActorMirror(replace(session, client=client), connection, seed=42, render_radius=None)
            try:
                with self.assertRaisesRegex(TimeoutError, "lost spawn response"):
                    mirror.step()
                self.assertEqual(mirror.actor_ids, {})
                self.assertIsNotNone(session.world.get_actor(spawned_ids[0]))
            finally:
                mirror.close()
            session.tick()
            self.assertFalse(session.world.get_snapshot().has_actor(spawned_ids[0]))
