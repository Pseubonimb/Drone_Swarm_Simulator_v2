"""
Shared DroneController: MAVLinkWorker, NED/attitude/velocity reads, RC keepalive.

All pymavlink access goes through the worker (thread-safe). Scenarios import this
class and pass scenario-specific init_steps to initialize(). Algorithm-specific
logic (e.g. FollowerPIDPursuit in leader_forward_back, move_with_pid_braking) stays in scenarios.
"""

import os
import threading
import time
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from core.mavlink.utils import RC_NEUTRAL
from core.mavlink.worker import MAVLinkWorker

_DEFAULT_POSITION: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}
_DEFAULT_ATTITUDE: Dict[str, float] = {"rx": 0.0, "ry": 0.0, "rz": 0.0}
_DEFAULT_VELOCITY: Dict[str, float] = {"vx": 0.0, "vy": 0.0, "vz": 0.0}

if TYPE_CHECKING:
    from core.network import CoordExchangeManager


class DroneController:
    """Controller for one drone: MAVLinkWorker, state getters, RC keepalive. Thread-safe."""

    def __init__(
        self,
        config: Dict[str, Any],
        logging_enabled: bool = False,
        log_dir: str = "logs",
    ) -> None:
        """Initialize controller with config and optional file logging.

        Args:
            config: Dict with 'id', 'role', and either 'udp_port' (MAVProxy UDP out) or
                'mavlink_connection' (full pymavlink URI, e.g. tcp:127.0.0.1:5760).
            logging_enabled: If True, write position logs to log_dir/drone_<id>_log.txt.
            log_dir: Directory for log files when logging_enabled is True.
        """
        self.config: Dict[str, Any] = config
        self.worker: Optional[MAVLinkWorker] = None
        self.other_drones_positions: Dict[int, Dict[str, float]] = {}
        self.lock: threading.Lock = threading.Lock()
        self.logging_enabled: bool = logging_enabled
        self.log_dir: str = log_dir
        self.log_iter: int = 0
        self.logfile: Optional[Any] = None
        if self.logging_enabled:
            os.makedirs(self.log_dir, exist_ok=True)
            self.logfile = open(
                os.path.join(self.log_dir, f"drone_{self.config['id']}_log.txt"), "w"
            )
        self.last_rc_channels = {
            "roll": RC_NEUTRAL,
            "pitch": RC_NEUTRAL,
            "throttle": RC_NEUTRAL,
            "yaw": RC_NEUTRAL,
        }
        self.rc_channels_lock = threading.Lock()

    @classmethod
    def start_swarm_coord_exchange(
        cls,
        controllers: List["DroneController"],
        **kwargs: Any,
    ) -> "CoordExchangeManager":
        """Start multi-drone position exchange in a background thread.

        Wraps ``CoordExchangeManager`` so scenarios only call this one entry point.
        Accepts the same keyword arguments as ``CoordExchangeManager`` (rates, noise,
        ``noise_dict``, CSV paths, ``on_step``, etc.).

        Returns:
            Running manager; call ``stop()`` on shutdown if you need a clean join.
        """
        from core.network import CoordExchangeManager

        mgr = CoordExchangeManager(controllers, **kwargs)
        mgr.start()
        return mgr

    def connect(self) -> None:
        """Create MAVLinkWorker and start its dedicated I/O thread."""
        conn_str = self.config.get("mavlink_connection")
        if not conn_str:
            conn_str = f'udp:127.0.0.1:{self.config["udp_port"]}'
        self.worker = MAVLinkWorker(conn_str, self.config["id"])
        self.worker.start()

    def initialize(self, init_steps: List[Dict[str, Any]]) -> None:
        """Run init sequence (arm, takeoff, set mode, etc.) via worker.

        Args:
            init_steps: List of command dicts for worker.run_init_sequence()
                (e.g. set_mode, sleep, arm, takeoff, rc_override, request_position_stream).
        """
        if self.worker is None:
            return
        self.worker.run_init_sequence(init_steps)

    def start_rc_keepalive(self) -> None:
        """Start a daemon thread that periodically sends RC_OVERRIDE via worker."""
        def keepalive_loop() -> None:
            while self.worker is not None:
                try:
                    with self.rc_channels_lock:
                        roll = self.last_rc_channels["roll"]
                        pitch = self.last_rc_channels["pitch"]
                        throttle = self.last_rc_channels["throttle"]
                        yaw = self.last_rc_channels["yaw"]
                    if self.worker:
                        self.worker.send_rc_override(
                            roll, pitch, throttle, yaw, controller=self
                        )
                    time.sleep(0.2)
                except Exception:
                    break

        threading.Thread(target=keepalive_loop, daemon=True).start()

    def get_position(self) -> Dict[str, float]:
        """Return this drone's last known NED position (x, y, z).

        Returns:
            Dict with keys 'x', 'y', 'z' (meters in NED). Defaults to origin if unavailable.
        """
        if self.worker is None:
            return dict(_DEFAULT_POSITION)
        pos = self.worker.get_position()
        if pos is None:
            return dict(_DEFAULT_POSITION)
        return {"x": pos["x"], "y": pos["y"], "z": pos["z"]}

    def get_attitude(self) -> Dict[str, float]:
        """Return current Euler angles (roll, pitch, yaw) in radians as rx, ry, rz."""
        if self.worker is None:
            return dict(_DEFAULT_ATTITUDE)
        att = self.worker.get_attitude()
        if att is None:
            return dict(_DEFAULT_ATTITUDE)
        return {"rx": att["rx"], "ry": att["ry"], "rz": att["rz"]}

    def get_velocity(self) -> Dict[str, float]:
        """Return current NED velocity (vx, vy, vz) in m/s from LOCAL_POSITION_NED cache."""
        if self.worker is None:
            return dict(_DEFAULT_VELOCITY)
        pos = self.worker.get_position()
        if pos is None:
            return dict(_DEFAULT_VELOCITY)
        return {
            "vx": pos.get("vx", 0.0),
            "vy": pos.get("vy", 0.0),
            "vz": pos.get("vz", 0.0),
        }

    def get_relative_position(self, target_position: Dict[str, float]) -> Dict[str, float]:
        """Relative position to target (target - self) in meters (NED x, y, z)."""
        pos = self.get_position()
        return {
            "x": target_position["x"] - pos["x"],
            "y": target_position["y"] - pos["y"],
            "z": target_position["z"] - pos["z"],
        }

    def get_my_position(self) -> Dict[str, float]:
        """Return this drone's position; optionally log to file when logging_enabled.

        Returns:
            Dict with keys 'x', 'y', 'z' (NED position in meters).
        """
        pos = self.get_position()
        if self.logging_enabled and self.logfile:
            self.log_iter += 1
            if self.log_iter % 20 == 0:
                self.logfile.write(f"{pos}\n")
                self.logfile.flush()
        return pos

    def update_other_drone_position(
        self, drone_id: int, position: Dict[str, float]
    ) -> None:
        """Store another drone's position for formation/coordination."""
        with self.lock:
            self.other_drones_positions[drone_id] = position

    def get_other_drones_positions(self) -> Dict[int, Dict[str, float]]:
        """Return a copy of other drones' positions (thread-safe).

        Returns:
            Shallow copy of drone_id -> position dict; inner position dicts are copied.
        """
        with self.lock:
            return {k: dict(v) for k, v in self.other_drones_positions.items()}

    def stop(self) -> None:
        """Send neutral RC, command LAND via worker, then stop worker."""
        if self.worker:
            self.worker.send_rc_override(
                RC_NEUTRAL,
                RC_NEUTRAL,
                RC_NEUTRAL,
                RC_NEUTRAL,
                controller=self,
            )
            self.worker.send_set_mode(9)  # LAND
        if self.logging_enabled and self.logfile:
            try:
                self.logfile.close()
            except Exception:
                pass
            self.logfile = None
        if self.worker:
            self.worker.stop()
