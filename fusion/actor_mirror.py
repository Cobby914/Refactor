"""Mirror SUMO actors with acknowledged CARLA commands and scoped ownership."""

from __future__ import annotations

from math import hypot, isfinite
from uuid import uuid4

import carla
from traci.constants import INVALID_DOUBLE_VALUE

from traffic.carla_sync import (
    CarlaSyncManager,
    DEFAULT_CENTER_CARLA,
    PED_DEFAULT_HALF_HEIGHT,
    SPAWN_RETRY_EXTRA_Z,
    sumo_to_carla_xy_yaw,
)


ActorKey = tuple[str, str]
ROLE_PREFIX = "scalesense-v2:"


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
        self._visible: set[ActorKey] = set()
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

    def _spawn(self, poses):
        pending = {}
        for key in sorted(poses.keys() - self._actors.keys()):
            kind, sumo_id = key
            blueprint = (
                self._geometry.vehicle_blueprint(sumo_id) if kind == "vehicle"
                else self._geometry.walker_blueprint(sumo_id)
            )
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
                transform = carla.Transform(
                    carla.Location(location.x, location.y, location.z + extra_z), pose.rotation,
                )
                commands.append(carla.command.SpawnActor(blueprint, transform))
            self._spawn_uncertain = True
            errors = []
            for key, response in zip(list(pending), self._apply(commands)):
                if not response.error:
                    self._actors[key] = response.actor_id
                    spawned.append(key)
                    del pending[key]
                elif "collision" not in response.error.lower():
                    errors.append(f"{key}: {response.error}")
            self._spawn_uncertain = False
            if errors:
                raise RuntimeError("CARLA actor spawn failed: " + ", ".join(errors))
        self.spawn_collisions += len(pending)
        return spawned

    def step(self) -> int:
        positions = {}
        vehicle_count = 0
        for kind, domain in self._domains.items():
            actor_ids = sorted(domain.getIDList())
            if kind == "vehicle":
                vehicle_count = len(actor_ids)
            for sumo_id in actor_ids:
                position = domain.getPosition(sumo_id)
                if INVALID_DOUBLE_VALUE in position:
                    continue
                if not all(isfinite(value) for value in position):
                    raise RuntimeError(f"SUMO returned an invalid position for {kind} {sumo_id}")
                if self._in_radius(position):
                    positions[kind, sumo_id] = position

        visible = set(positions)
        removed = sorted(self._actors.keys() - visible)
        self._destroy([self._actors[key] for key in removed])
        for key in removed:
            del self._actors[key]
        for key in self._visible - visible:
            self._geometry.forget_actor_geometry(*key)
        self._visible = visible

        snapshot = self._session.world.get_snapshot()
        for key, actor_id in list(self._actors.items()):
            if not snapshot.has_actor(actor_id):
                del self._actors[key]
                self._geometry.forget_actor_geometry(*key)

        poses = {}
        for key, position in positions.items():
            kind, sumo_id = key
            pose = (
                self._geometry.vehicle_transform(sumo_id, position=position)
                if kind == "vehicle" else self._geometry.walker_transform(sumo_id, position=position)
            )
            if not all(isfinite(value) for value in (
                pose.location.x, pose.location.y, pose.location.z,
                pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll,
            )):
                raise RuntimeError(f"computed an invalid CARLA transform for {kind} {sumo_id}")
            poses[key] = pose

        spawned = self._spawn(poses)
        for key in spawned:
            kind, sumo_id = key
            if kind == "walker":
                actor = self._session.world.get_actor(self._actors[key])
                if actor is None:
                    raise RuntimeError(f"CARLA did not expose newly spawned walker {sumo_id}")
                height = actor.bounding_box.extent.z
                self._geometry.set_walker_height(sumo_id, height)
                poses[key].location += carla.Location(z=height - PED_DEFAULT_HALF_HEIGHT)

        commands = [carla.command.SetSimulatePhysics(self._actors[key], False) for key in spawned]
        commands.extend(
            carla.command.ApplyTransform(actor_id, poses[key])
            for key, actor_id in sorted(self._actors.items())
        )
        errors = [response.error for response in self._apply(commands) if response.error]
        if errors:
            raise RuntimeError("CARLA actor transform or physics command failed: " + ", ".join(errors))
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
