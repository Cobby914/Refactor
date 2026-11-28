"""Mirror SUMO actors with acknowledged CARLA commands and scoped ownership."""

from __future__ import annotations

import logging
from math import hypot, isfinite
from uuid import uuid4

import carla
from traci.constants import INVALID_DOUBLE_VALUE
from traci.exceptions import TraCIException

from traffic.carla_sync import (
    CarlaSyncManager,
    DEFAULT_CENTER_CARLA,
    PED_DEFAULT_HALF_HEIGHT,
    SPAWN_RETRY_EXTRA_Z,
    sumo_to_carla_xy_yaw,
)

from .actor_clearance import Footprint, catch_up, rotation_padding


ActorKey = tuple[str, str]
ROLE_PREFIX = "scalesense-v2:"
SPAWN_STAGING_HEIGHT_M = 20.0
CROSSING_CLEARANCE_M = 0.4
SAME_KIND_CLEARANCE_M = 0.05
CATCH_UP_SPEED_MPS = {"vehicle": 22.0, "walker": 2.0}
logger = logging.getLogger(__name__)


class ActorMirror:
    def __init__(self, session, connection, *, seed, render_radius, center=DEFAULT_CENTER_CARLA):
        self._session = session
        self._domains = {"vehicle": connection.vehicle, "walker": connection.person}
        self._center = center
        self._render_radius = render_radius if render_radius is not None else float("inf")
        self._geometry = CarlaSyncManager(connection=connection, seed=seed)
        self._geometry.attach_world(session.world)
        self._role_prefix = f"{ROLE_PREFIX}{uuid4().hex}:"
        self._actors: dict[ActorKey, int] = {}
        self._boxes: dict[ActorKey, carla.BoundingBox] = {}
        self._held: set[ActorKey] = set()
        self._visible: set[ActorKey] = set()
        # A failed actor stays out of CARLA until its ID leaves the SUMO domain.
        self._failed_ids: dict[str, set[str]] = {kind: set() for kind in self._domains}
        self._spawn_uncertain = False
        self.spawn_collisions = 0

    @property
    def actor_ids(self) -> dict[ActorKey, int]:
        return self._actors.copy()

    def _in_radius(self, position):
        x, y, _ = sumo_to_carla_xy_yaw(*position, 0)
        return hypot(x - self._center[0], y - self._center[1]) <= self._render_radius

    def _apply(self, commands):
        if not commands:
            return []
        responses = self._session.client.apply_batch_sync(commands, False)
        if len(responses) != len(commands):
            raise RuntimeError("CARLA returned an incomplete actor batch response")
        return responses

    def _destroy(self, actor_ids):
        commands = [carla.command.DestroyActor(actor_id) for actor_id in actor_ids]
        responses = self._apply(commands)
        errors = [
            f"{actor_id}: {response.error}"
            for actor_id, response in zip(actor_ids, responses)
            if response.error and "not found" not in response.error.lower()
        ]
        if errors:
            raise RuntimeError("CARLA actor destruction failed: " + ", ".join(errors))

    def _discard(self, keys):
        self._destroy([self._actors[key] for key in sorted(keys) if key in self._actors])
        for key in keys:
            self._forget(key)
            self._visible.discard(key)

    def _forget(self, key):
        self._actors.pop(key, None)
        self._boxes.pop(key, None)
        self._held.discard(key)
        self._geometry.forget_actor_geometry(*key)

    def _discard_failed(self, failures):
        self._discard(failures)
        for (kind, sumo_id), reason in failures.items():
            self._failed_ids[kind].add(sumo_id)
            logger.warning("Dropped %s %s from the CARLA mirror: %s", kind, sumo_id, reason)

    def _spawn(self, poses):
        pending = {}
        failures = {}
        for key in sorted(poses.keys() - self._actors.keys()):
            kind, sumo_id = key
            try:
                blueprint = (
                    self._geometry.vehicle_blueprint(sumo_id) if kind == "vehicle"
                    else self._geometry.walker_blueprint(sumo_id)
                )
            except TraCIException as exc:
                failures[key] = f"could not select CARLA blueprint: {exc}"
                continue
            if not blueprint.has_attribute("role_name"):
                raise RuntimeError(f"CARLA blueprint {blueprint.id} has no ownership role")
            blueprint.set_attribute("role_name", f"{self._role_prefix}{kind}:{sumo_id}")
            pending[key] = blueprint

        spawned = []
        for extra_z in SPAWN_RETRY_EXTRA_Z:
            if not pending:
                break
            commands = []
            for key, blueprint in pending.items():
                pose = poses[key]
                location = pose.location
                # Read the real blueprint bounds out of traffic before admitting the spawn.
                transform = carla.Transform(
                    carla.Location(location.x, location.y, location.z + SPAWN_STAGING_HEIGHT_M + extra_z),
                    pose.rotation,
                )
                commands.append(carla.command.SpawnActor(blueprint, transform))
            self._spawn_uncertain = True
            for key, response in zip(list(pending), self._apply(commands)):
                if not response.error:
                    self._actors[key] = response.actor_id
                    spawned.append(key)
                    del pending[key]
                elif "collision" not in response.error.lower():
                    failures[key] = f"spawn failed: {response.error}"
                    del pending[key]
            self._spawn_uncertain = False
        self.spawn_collisions += len(pending)
        self._discard_failed(failures)
        return spawned

    @staticmethod
    def _blocked(key, footprint, occupied, rotation_margin=0.0):
        return any(
            footprint.conflicts(other, rotation_margin + (
                SAME_KIND_CLEARANCE_M if other_key[0] == key[0] else CROSSING_CLEARANCE_M
            ))
            for other_key, other in occupied.items() if other_key != key
        )

    def _plan_updates(self, snapshot, poses, spawned):
        current = {
            key: snapshot.find(actor_id).get_transform()
            for key, actor_id in self._actors.items() if key not in spawned
        }
        occupied = {key: Footprint.at(self._boxes[key], pose) for key, pose in current.items()}
        updates = []
        # Reserve both ends of an accepted move for the rest of this batch.
        for key in sorted(current):
            actor_id, box = self._actors[key], self._boxes[key]
            start, desired = current[key], poses[key]
            target = desired
            if key in self._held:
                target = catch_up(start, desired, CATCH_UP_SPEED_MPS[key[0]] * self._session.fixed_delta_s)
            footprint = Footprint.at(box, target)
            sweep = occupied[key].swept_to(footprint)
            if self._blocked(key, sweep, occupied, rotation_padding(box, start.rotation, target.rotation)):
                self._held.add(key)
                continue
            if target is desired:
                self._held.discard(key)
            updates.append((key, carla.command.ApplyTransform(actor_id, target)))
            occupied[key] = sweep

        deferred = []
        for key in sorted(spawned):
            target = poses[key]
            footprint = Footprint.at(self._boxes[key], target)
            if self._blocked(key, footprint, occupied):
                deferred.append(key)
                continue
            actor_id = self._actors[key]
            updates.append((key, carla.command.SetSimulatePhysics(actor_id, False)))
            updates.append((key, carla.command.ApplyTransform(actor_id, target)))
            occupied[key] = footprint
        self._discard(deferred)
        self.spawn_collisions += len(deferred)
        return updates

    def step(self) -> int:
        positions = {}
        failures = {}
        vehicle_count = 0
        for kind, domain in self._domains.items():
            actor_ids = set(domain.getIDList())
            failed_ids = self._failed_ids[kind]
            failed_ids.intersection_update(actor_ids)
            if kind == "vehicle":
                vehicle_count = len(actor_ids)
            for sumo_id in sorted(actor_ids - failed_ids):
                key = (kind, sumo_id)
                try:
                    position = domain.getPosition(sumo_id)
                except TraCIException as exc:
                    failures[key] = f"could not read SUMO position: {exc}"
                    continue
                if INVALID_DOUBLE_VALUE in position:
                    continue
                if not all(isfinite(value) for value in position):
                    failures[key] = "SUMO returned a non-finite position"
                    continue
                if self._in_radius(position):
                    positions[key] = position

        self._discard_failed(failures)
        visible = set(positions)
        self._discard(self._visible - visible)
        self._visible = visible

        snapshot = self._session.world.get_snapshot()
        for key, actor_id in list(self._actors.items()):
            if not snapshot.has_actor(actor_id):
                self._forget(key)

        poses = {}
        failures = {}
        for key, position in positions.items():
            kind, sumo_id = key
            try:
                pose = (
                    self._geometry.vehicle_transform(sumo_id, position=position)
                    if kind == "vehicle" else self._geometry.walker_transform(sumo_id, position=position)
                )
            except TraCIException as exc:
                failures[key] = f"could not build CARLA transform: {exc}"
                continue
            if not all(isfinite(value) for value in (
                pose.location.x, pose.location.y, pose.location.z,
                pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll,
            )):
                failures[key] = "computed a non-finite CARLA transform"
                continue
            poses[key] = pose

        self._discard_failed(failures)
        spawned = self._spawn(poses)
        failures = {}
        for key in spawned:
            kind, sumo_id = key
            actor = self._session.world.get_actor(self._actors[key])
            if actor is None:
                failures[key] = "CARLA could not find the newly spawned actor"
                continue
            self._boxes[key] = actor.bounding_box
            if kind == "walker":
                height = self._boxes[key].extent.z
                self._geometry.set_walker_height(sumo_id, height)
                poses[key].location += carla.Location(z=height - PED_DEFAULT_HALF_HEIGHT)

        self._discard_failed(failures)
        updates = self._plan_updates(snapshot, poses, set(spawned) & self._actors.keys())
        failures = {}
        for (key, command), response in zip(updates, self._apply([command for _, command in updates])):
            if response.error:
                reason = f"{type(command).__name__} failed: {response.error}"
                failures.setdefault(key, reason)
        self._discard_failed(failures)
        return vehicle_count

    def close(self):
        owned = set(self._actors.values())
        try:
            if self._spawn_uncertain:
                # A timed-out spawn batch can finish server-side before its IDs are known.
                self._session.tick()
                owned.update(
                    actor.id for actor in self._session.world.get_actors()
                    if actor.attributes.get("role_name", "").startswith(self._role_prefix)
                )
        finally:
            self._destroy(sorted(owned))
            self._actors.clear()
            self._boxes.clear()
            self._held.clear()
