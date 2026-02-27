"""
Monitors drone position (LOCAL_POSITION_NED) and attitude (ATTITUDE) via MAVLink.
"""

import logging
import threading
from typing import Any, Dict, Optional

from pymavlink import mavutil

logger = logging.getLogger(__name__)


class CoordsMonitor:
    """
    Monitors NED position and Euler attitude from a single drone MAVLink connection.

    Uses dedicated threads for LOCAL_POSITION_NED and ATTITUDE. No MAVLinkWorker
    integration in Stage 2; takes drone connection directly.
    """

    def __init__(self, drone: Any) -> None:
        """
        Args:
            drone: MAVLink connection (mavutil.mavlink_connection).
        """
        self.drone = drone
        self.position_ned: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.attitude: Dict[str, float] = {
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
        }  # roll, pitch, yaw (radians)
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.attitude_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

    def start(self) -> None:
        """Start position and attitude monitoring in separate threads."""
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()
        self.attitude_thread = threading.Thread(
            target=self._attitude_loop, daemon=True
        )
        self.attitude_thread.start()
        logger.info("Coordinates and attitude monitoring started")

    def stop(self) -> None:
        """Stop monitoring threads."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.attitude_thread:
            self.attitude_thread.join(timeout=1.0)
        logger.info("Coordinates monitoring stopped")

    def _monitor_loop(self) -> None:
        """Loop reading LOCAL_POSITION_NED (blocking)."""
        while self.running:
            try:
                msg = self.drone.recv_match(
                    type="LOCAL_POSITION_NED", blocking=True
                )
                if not msg:
                    continue
                if msg.get_type() == "BAD_DATA":
                    if mavutil.all_printable(msg.data):
                        logger.debug("BAD_DATA: %s", msg.data)
                    continue
                with self.lock:
                    self.position_ned["x"] = msg.x
                    self.position_ned["y"] = msg.y
                    self.position_ned["z"] = msg.z
            except Exception:
                pass

    def _attitude_loop(self) -> None:
        """Loop reading ATTITUDE (roll, pitch, yaw in radians)."""
        while self.running:
            try:
                msg = self.drone.recv_match(type="ATTITUDE", blocking=True)
                if not msg or msg.get_type() == "BAD_DATA":
                    continue
                with self.lock:
                    self.attitude["rx"] = msg.roll
                    self.attitude["ry"] = msg.pitch
                    self.attitude["rz"] = msg.yaw
            except Exception:
                pass

    def get_attitude(self) -> Dict[str, float]:
        """Return current Euler angles (roll, pitch, yaw) in radians."""
        with self.lock:
            return self.attitude.copy()

    def get_position(self) -> Dict[str, float]:
        """Return current NED position (x, y, z)."""
        with self.lock:
            return self.position_ned.copy()

    def get_distance_to(self, target_position: Dict[str, float]) -> float:
        """
        Distance in meters to the given target position (x, y, z).

        Args:
            target_position: Dict with keys 'x', 'y', 'z'.

        Returns:
            Distance in meters.
        """
        with self.lock:
            dx = target_position["x"] - self.position_ned["x"]
            dy = target_position["y"] - self.position_ned["y"]
            dz = target_position["z"] - self.position_ned["z"]
            return (dx**2 + dy**2 + dz**2) ** 0.5

    def get_relative_position(
        self, target_position: Dict[str, float]
    ) -> Dict[str, float]:
        """
        Relative position to target (target - self).

        Args:
            target_position: Dict with keys 'x', 'y', 'z'.

        Returns:
            Dict with 'x', 'y', 'z' relative offsets in meters.
        """
        with self.lock:
            return {
                "x": target_position["x"] - self.position_ned["x"],
                "y": target_position["y"] - self.position_ned["y"],
                "z": target_position["z"] - self.position_ned["z"],
            }
