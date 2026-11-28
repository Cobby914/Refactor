"""Own CARLA synchronous settings for one v2 session."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from math import isclose
from time import monotonic, sleep
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import carla


@dataclass(frozen=True)
class CarlaSession:
    client: carla.Client
    world: carla.World
    version: str
    initial_frame: int
    fixed_delta_s: float
    rpc_timeout_s: float

    def tick(self) -> int:
        frame = self.world.tick(self.rpc_timeout_s)
        snapshot = self.world.get_snapshot()
        if snapshot.frame != frame:
            raise RuntimeError(f"CARLA tick returned frame {frame}, snapshot was {snapshot.frame}")
        delta_s = snapshot.timestamp.delta_seconds
        if not isclose(delta_s, self.fixed_delta_s, rel_tol=0, abs_tol=1e-7):
            raise RuntimeError(f"CARLA frame {frame} used {delta_s:g}s, expected {self.fixed_delta_s:g}s")
        return frame


def _wait_for_world(client, endpoint, ready_timeout_s, rpc_timeout_s, stop_check=None):
    deadline = monotonic() + ready_timeout_s
    last_error = None
    while (remaining := deadline - monotonic()) > 0:
        if stop_check is not None:
            stop_check()
        client.set_timeout(min(remaining, rpc_timeout_s))
        try:
            return client.get_world()
        except RuntimeError as exc:
            last_error = exc
            sleep(max(0, min(0.1, deadline - monotonic())))
    raise RuntimeError(f"CARLA at {endpoint} did not become ready in {ready_timeout_s:g}s") from last_error


def _confirm_start_frame(world, fixed_delta_s, rpc_timeout_s, stop_check):
    # ApplySettings can leave a frame in flight. Confirm a clean tick before
    # any SUMO advance, and keep tick() strict once the session is handed out.
    previous = world.get_snapshot().frame
    consecutive = 0
    for _ in range(4):
        if stop_check is not None:
            stop_check()
        reply = world.tick(rpc_timeout_s)
        snapshot = world.get_snapshot()
        if (reply == snapshot.frame == previous + 1
                and isclose(snapshot.timestamp.delta_seconds, fixed_delta_s, rel_tol=0, abs_tol=1e-7)):
            consecutive += 1
            if consecutive == 2:
                return reply
        else:
            consecutive = 0
        previous = snapshot.frame
    raise RuntimeError(f"CARLA did not confirm consecutive {fixed_delta_s:g}s setup ticks. "
                       "Use 20 Hz on this server or select the legacy engine.")


@contextmanager
def open_carla_session(host, port, fixed_delta_s, *, ready_timeout_s, rpc_timeout_s, stop_check=None, client=None):
    """Connect to an idle Town10HD_Opt world and restore its settings on exit."""
    import carla

    if client is None:
        client = carla.Client(host, port)
    world = _wait_for_world(client, f"{host}:{port}", ready_timeout_s, rpc_timeout_s, stop_check)
    client.set_timeout(rpc_timeout_s)
    client_version = client.get_client_version()
    server_version = client.get_server_version()
    if server_version != client_version:
        raise RuntimeError(f"CARLA version mismatch (client {client_version}, server {server_version})")

    map_name = world.get_map().name.rsplit("/", 1)[-1]
    if map_name != "Town10HD_Opt":
        raise RuntimeError(f"expected CARLA map Town10HD_Opt, got {map_name}")
    original_settings = world.get_settings()
    if original_settings.synchronous_mode:
        raise RuntimeError("CARLA is already synchronous. Use an idle asynchronous endpoint.")

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = fixed_delta_s
    if settings.substepping:
        max_delta_s = settings.max_substep_delta_time * settings.max_substeps
        if fixed_delta_s > max_delta_s + 1e-7:
            raise RuntimeError(f"CARLA physics substeps allow at most {max_delta_s:g}s per tick")

    if stop_check is not None:
        stop_check()
    try:
        world.apply_settings(settings, rpc_timeout_s)
        effective = world.get_settings()
        if not effective.synchronous_mode or not isclose(
            effective.fixed_delta_seconds or 0, fixed_delta_s, rel_tol=0, abs_tol=1e-7
        ):
            raise RuntimeError("CARLA did not enable the requested synchronous fixed delta")
        initial_frame = _confirm_start_frame(world, fixed_delta_s, rpc_timeout_s, stop_check)
        yield CarlaSession(
            client, world, server_version, initial_frame, fixed_delta_s, rpc_timeout_s
        )
    finally:
        world.apply_settings(original_settings, rpc_timeout_s)
        if world.get_settings() != original_settings:
            raise RuntimeError("CARLA did not restore its original world settings")
