"""
Monitors drone velocity via MAVLink (GLOBAL_POSITION_INT).
"""

import logging
import threading
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class VelocityMonitor:
    """
    Monitors NED velocity (vx, vy, vz) from GLOBAL_POSITION_INT.

    Takes drone connection directly; no MAVLinkWorker in Stage 2.
    """

    def __init__(self, drone: Any) -> None:
        """
        Args:
            drone: MAVLink connection (mavutil.mavlink_connection).
        """
        self.drone = drone
        self.velocity_ned: Dict[str, float] = {
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
        }
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

    def start(self) -> None:
        """Start velocity monitoring in a background thread."""
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()
        logger.info("Velocity monitoring started")

    def stop(self) -> None:
        """Stop the monitoring thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        logger.info("Velocity monitoring stopped")

    def _monitor_loop(self) -> None:
        """Loop reading GLOBAL_POSITION_INT (vx, vy, vz in cm/s -> m/s)."""
        while self.running:
            try:
                msg = self.drone.recv_match(
                    type="GLOBAL_POSITION_INT",
                    blocking=False,
                    timeout=0.1,
                )
                if msg:
                    with self.lock:
                        self.velocity_ned["vx"] = msg.vx / 100.0
                        self.velocity_ned["vy"] = msg.vy / 100.0
                        self.velocity_ned["vz"] = (
                            msg.vz / 100.0 if hasattr(msg, "vz") else 0.0
                        )
            except Exception:
                pass

    def get_velocity(self) -> Dict[str, float]:
        """Return current NED velocity (vx, vy, vz) in m/s."""
        with self.lock:
            return self.velocity_ned.copy()
