"""Start a private SUMO child and use its TraCI connection directly."""

from __future__ import annotations

from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from pathlib import Path
import subprocess
import sys
from time import monotonic, sleep
from typing import TYPE_CHECKING

from traffic import sumo_env

if TYPE_CHECKING:
    from traci.connection import Connection


@dataclass(frozen=True)
class SumoSession:
    connection: Connection
    version: str
    pid: int
    fixed_delta_s: float
    initial_time_s: float

    def advance(self) -> float:
        self.connection.simulationStep()
        return self.connection.simulation.getTime()


def _connect(traci, port, process, deadline):
    last_error = None
    while monotonic() < deadline:
        if (code := process.poll()) is not None:
            raise RuntimeError(f"SUMO exited during startup (code {code})") from last_error
        try:
            return traci.connect(port=port, host="127.0.0.1", numRetries=0)
        except traci.exceptions.FatalTraCIError as exc:
            last_error = exc
            sleep(max(0, min(0.1, deadline - monotonic())))
    raise RuntimeError("timed out waiting for the SUMO TraCI listener") from last_error


def _close_connection(connection, connection_socket, rpc_timeout_s):
    try:
        if connection_socket.fileno() != -1:
            connection_socket.settimeout(min(rpc_timeout_s, 2))
        connection.close(wait=False)
    finally:
        connection_socket.close()


def _reap(process):
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"SUMO process {process.pid} did not exit after being killed") from exc


@contextmanager
def open_sumo_session(config_path: Path, *, ready_timeout_s, rpc_timeout_s):
    """Yield only after a version handshake and clock query succeed."""
    sumo_env.ensure_traci_importable()
    import traci
    from sumolib.miscutils import getFreeSocketPort

    port = getFreeSocketPort()
    cmd = [
        sumo_env.sumo_binary(), "-c", str(config_path.resolve()),
        "--remote-port", str(port), "--num-clients", "1",
        "--no-step-log", "--duration-log.disable", "true",
    ]
    group = (
        {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if sys.platform == "win32"
        else {"start_new_session": True}
    )
    try:
        with ExitStack() as cleanup:
            process = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, **group)
            cleanup.callback(_reap, process)
            deadline = monotonic() + ready_timeout_s
            connection = _connect(traci, port, process, deadline)
            # TraCI 1.28 provides no public per-connection read timeout.
            connection_socket = connection._socket
            cleanup.callback(_close_connection, connection, connection_socket, rpc_timeout_s)

            def set_startup_timeout():
                remaining = deadline - monotonic()
                if remaining <= 0:
                    raise RuntimeError("timed out during the SUMO TraCI handshake")
                connection_socket.settimeout(min(remaining, rpc_timeout_s))

            set_startup_timeout()
            _, version = connection.getVersion()
            set_startup_timeout()
            fixed_delta_s = connection.simulation.getDeltaT()
            set_startup_timeout()
            initial_time_s = connection.simulation.getTime()
            if (code := process.poll()) is not None:
                raise RuntimeError(f"SUMO exited during the TraCI handshake (code {code})")
            connection_socket.settimeout(rpc_timeout_s)
            yield SumoSession(connection, version, process.pid, fixed_delta_s, initial_time_s)
    except (traci.exceptions.TraCIException, traci.exceptions.FatalTraCIError) as exc:
        raise RuntimeError(f"SUMO TraCI session failed: {exc}") from exc
