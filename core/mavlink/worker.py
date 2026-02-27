"""
MAVLink worker: thread-safe access to a single drone connection.

Skeleton only in Stage 2. Full implementation (dedicated MAVLink thread,
command queue, state locks) is planned for Stage 3+.
"""

from typing import Any, Dict, Optional


class MAVLinkWorker:
    """
    Thread-safe MAVLink access for one drone (skeleton).

    In Stage 2 this is a placeholder. All pymavlink operations (recv_match,
    rc_channels_override_send, etc.) must be performed from a single dedicated
    thread; the full loop, command queue, and get_position/get_attitude
    implementations are deferred to Stage 3+.
    """

    def __init__(self, connection_string: str, drone_id: int) -> None:
        """
        Initialize the worker (does not connect yet in skeleton).

        Args:
            connection_string: e.g. 'udp:127.0.0.1:14551'.
            drone_id: Drone identifier for logging and state.
        """
        self.connection_string = connection_string
        self.drone_id = drone_id

    def start(self) -> None:
        """Start the MAVLink thread. Skeleton: no-op (full impl in Stage 3+)."""
        pass

    def stop(self) -> None:
        """Stop the MAVLink thread. Skeleton: no-op (full impl in Stage 3+)."""
        pass

    def send_rc_override(
        self,
        chan1: int,
        chan2: int,
        chan3: int,
        chan4: int,
        controller: Optional[Any] = None,
    ) -> None:
        """
        Send RC_OVERRIDE (roll, pitch, throttle, yaw). Skeleton: no-op.

        Full implementation will enqueue command for the MAVLink thread.
        """
        pass

    def get_position(self) -> Optional[Dict[str, float]]:
        """
        Return last LOCAL_POSITION_NED (x, y, z, vx, vy, vz). Skeleton: returns None.

        Full implementation will read from thread-safe state cache.
        """
        return None

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """
        Return last ATTITUDE (roll, pitch, yaw in radians). Skeleton: returns None.

        Full implementation will read from thread-safe state cache.
        """
        return None
