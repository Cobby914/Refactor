"""Mirror the verified Town10HD_Opt corridor signal approaches."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot

import carla

from traffic.carla_sync import sumo_to_carla_xy_yaw


# SUMO TLS ID -> CARLA OpenDRIVE signal ID -> directed incoming SUMO edge.
CORRIDOR_APPROACHES = {
    "189": {"949": "-19", "950": "-14", "951": "15", "952": "20"},
    "532": {"957": "3", "958": "-2", "959": "-21"},
    "719": {"960": "18", "961": "-5", "962": "6"},
}


@dataclass(frozen=True)
class SignalBinding:
    carla_light: carla.TrafficLight
    link_indices: tuple[int, ...]


def signal_aspect(states: str) -> carla.TrafficLightState:
    """Reduce the protected movements of one approach to a physical bulb."""
    if not states:
        raise ValueError("a signal aspect requires at least one controlled movement")
    if "G" in states:
        return carla.TrafficLightState.Green
    if any(state in "yY" for state in states):
        return carla.TrafficLightState.Yellow
    if all(state == "g" for state in states):
        return carla.TrafficLightState.Green
    if all(state in "oO" for state in states):
        return carla.TrafficLightState.Off
    return carla.TrafficLightState.Red


class TrafficLightMirror:
    def __init__(self, session, connection):
        self._session = session
        self._sumo = connection.trafficlight
        lights = session.world.get_actors().filter("traffic.traffic_light*")
        lights_by_id = {actor.get_opendrive_id(): actor for actor in lights}
        self._all_lights = list(lights)
        self._bindings = {}
        self._saved = {}
        self._expected = {}
        frozen_states = {actor.is_frozen() for actor in lights}
        if len(frozen_states) != 1:
            raise RuntimeError("CARLA traffic lights have mixed or missing freeze state")
        self._was_frozen = frozen_states.pop()
        for tls_id, approaches in CORRIDOR_APPROACHES.items():
            self._bindings[tls_id] = self._bind(tls_id, approaches, lights_by_id, connection.lane)
            for binding in self._bindings[tls_id]:
                light = binding.carla_light
                self._saved[light.id] = (light, light.get_state())

    def _bind(self, tls_id, approaches, lights_by_id, lanes):
        if not approaches.keys() <= lights_by_id.keys():
            raise RuntimeError(f"CARLA traffic-light group for SUMO junction {tls_id} is incomplete")
        light = lights_by_id[next(iter(approaches))]
        group_ids = {actor.get_opendrive_id() for actor in light.get_group_traffic_lights()}
        if group_ids != approaches.keys():
            raise RuntimeError(f"CARLA traffic-light group does not match SUMO junction {tls_id}")

        links = self._sumo.getControlledLinks(tls_id)
        program = self._sumo.getProgram(tls_id)
        logic = next((
            item for item in self._sumo.getAllProgramLogics(tls_id) if item.programID == program
        ), None)
        if logic is None:
            raise RuntimeError(f"SUMO junction {tls_id} has no phase logic for program {program}")
        bindings = []
        for landmark, edge in approaches.items():
            light = lights_by_id[landmark]
            incoming = {
                index: {link[0] for link in (group or ()) if link[0].rsplit("_", 1)[0] == edge}
                for index, group in enumerate(links)
            }
            incoming = {index: lane_ids for index, lane_ids in incoming.items() if lane_ids}
            endpoints = [
                sumo_to_carla_xy_yaw(*lanes.getShape(lane_id)[-1][:2], 0)[:2]
                for lane_id in sorted(set().union(*incoming.values()))
            ]
            stops = [waypoint.transform.location for waypoint in light.get_stop_waypoints()]
            if not any(hypot(stop.x - x, stop.y - y) <= 10 for x, y in endpoints for stop in stops):
                raise RuntimeError(f"CARLA signal {landmark} is not near SUMO approach {edge}")
            protected = tuple(
                index for index in incoming
                if any(phase.state[index] == "G" for phase in logic.phases)
            )
            bindings.append(SignalBinding(light, protected or tuple(incoming)))
        return bindings

    def start(self):
        if not self._was_frozen:
            self._session.world.freeze_all_traffic_lights(True)
            self._verify_during_maintenance(
                lambda: all(light.is_frozen() for light in self._all_lights),
                "CARLA did not freeze the traffic lights",
            )

    def step(self):
        changed = False
        for tls_id, bindings in self._bindings.items():
            states = self._sumo.getRedYellowGreenState(tls_id)
            for binding in bindings:
                light = binding.carla_light
                state = signal_aspect("".join(states[index] for index in binding.link_indices))
                if self._expected.get(light.id) != state:
                    light.set_state(state)
                    changed = True
                self._expected[light.id] = state
        if changed:
            # State setters are async on CARLA 0.9.16. Round-trip before the tick
            # and verify the aspects in the resulting snapshot.
            self._session.world.get_settings()

    def verify(self):
        for actor_id, state in self._expected.items():
            light, _ = self._saved[actor_id]
            if not light.is_frozen() or light.get_state() != state:
                raise RuntimeError(f"CARLA signal {light.get_opendrive_id()} did not reach {state}")

    def close(self):
        try:
            for light, state in self._saved.values():
                light.set_state(state)
            self._verify_during_maintenance(
                lambda: all(light.get_state() == state for light, state in self._saved.values()),
                "CARLA did not restore the original corridor signal aspects",
            )
        finally:
            if not self._was_frozen:
                self._session.world.freeze_all_traffic_lights(False)
                self._verify_during_maintenance(
                    lambda: all(not light.is_frozen() for light in self._all_lights),
                    "CARLA did not release the traffic-light freeze",
                )

    def _verify_during_maintenance(self, predicate, error):
        self._session.world.get_settings()
        for _ in range(3):
            self._session.tick()
            if predicate():
                return
        raise RuntimeError(error)
