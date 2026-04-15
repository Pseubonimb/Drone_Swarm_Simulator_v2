"""
MAVLink worker: single-threaded, thread-safe access to one drone connection.

All pymavlink operations (recv_match, rc_channels_override_send, set_mode, etc.)
run in one dedicated thread. Callers use get_position(), get_attitude(),
send_rc_override() and run_init_sequence() which enqueue commands or read
from thread-safe state cache.
"""

import logging
import queue
import threading
import time
from typing import Any, Dict, List, Optional, Union

from pymavlink import mavutil

logger = logging.getLogger(__name__)

# Main thread must not call recv_match / send on mavutil after the worker thread starts;
# TCP links are especially sensitive to split-thread use.

# Cap recv_msg drain per outer loop iteration so RC/command queue cannot starve indefinitely.
_MAX_RECV_DRAIN_PER_ITER = 512
_DEFAULT_TELEMETRY_HZ = 50


class MAVLinkWorker:
    """
    Thread-safe MAVLink access for one drone.

    One dedicated thread performs all recv_match() and mav.xxx_send() calls.
    Callers send commands via queue; state (position, attitude) is read via
    get_position() / get_attitude() under lock.
    """

    def __init__(self, connection_string: str, drone_id: int) -> None:
        """
        Initialize the worker (connection and thread start in start()).

        Args:
            connection_string: e.g. 'udp:127.0.0.1:14551'.
            drone_id: Drone identifier for logging and state.
        """
        self.connection_string = connection_string
        self.drone_id = drone_id
        self._command_queue: queue.Queue[Dict[str, Any]] = queue.Queue()
        self._state_lock = threading.Lock()
        self._last_position: Optional[Dict[str, float]] = None
        self._last_attitude: Optional[Dict[str, float]] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._master: Any = None
        self._bootstrap_ready = threading.Event()
        self._bootstrap_error: Optional[BaseException] = None

    def start(self) -> None:
        """Connect (in worker thread), wait for HEARTBEAT, then run the I/O loop."""
        if self._running:
            return
        if self._thread is not None and self._thread.is_alive():
            return

        self._bootstrap_ready.clear()
        self._bootstrap_error = None
        self._thread = threading.Thread(target=self._bootstrap_and_run, daemon=True)
        self._thread.start()

        connect_timeout = 120.0
        if not self._bootstrap_ready.wait(timeout=connect_timeout):
            self._running = False
            self._thread.join(timeout=3.0)
            self._thread = None
            raise TimeoutError(
                f"MAVLink worker bootstrap timeout for drone {self.drone_id} "
                f"({self.connection_string})"
            )
        if self._bootstrap_error is not None:
            err = self._bootstrap_error
            self._thread.join(timeout=3.0)
            self._thread = None
            raise err

        logger.info("MAVLinkWorker started for drone %s", self.drone_id)

    def _bootstrap_and_run(self) -> None:
        """Worker thread only: mavlink_connection, wait_heartbeat, then _run_loop."""
        conn_kw: Dict[str, Union[int, bool]] = {}
        is_tcp = self.connection_string.startswith("tcp:")
        if is_tcp:
            conn_kw["retries"] = 25
        hb_timeout = 90.0 if is_tcp else None
        try:
            self._master = mavutil.mavlink_connection(
                self.connection_string, **conn_kw
            )
            hb = self._master.wait_heartbeat(blocking=True, timeout=hb_timeout)
            if hb is None:
                logger.error(
                    "Drone %s: no HEARTBEAT on %s (timeout=%s)",
                    self.drone_id,
                    self.connection_string,
                    hb_timeout,
                )
                raise TimeoutError(
                    f"MAVLink HEARTBEAT timeout for drone {self.drone_id} "
                    f"({self.connection_string})"
                )
        except BaseException as exc:
            self._bootstrap_error = exc
            self._close_master_safe()
            self._bootstrap_ready.set()
            return

        try:
            self._prime_telemetry_streams(hz=_DEFAULT_TELEMETRY_HZ)
        except Exception as exc:
            logger.warning(
                "Drone %s: telemetry stream prime failed (continuing): %s",
                self.drone_id,
                exc,
            )

        self._running = True
        self._bootstrap_ready.set()
        try:
            self._run_loop()
        finally:
            self._running = False
            self._close_master_safe()

    def _close_master_safe(self) -> None:
        try:
            if self._master is not None and hasattr(self._master, "close"):
                self._master.close()
        except Exception:
            pass
        self._master = None

    def _prime_telemetry_streams(self, hz: int = _DEFAULT_TELEMETRY_HZ) -> None:
        """Request position/attitude streams early (SITL direct TCP often needs legacy + interval)."""
        if self._master is None:
            return
        m = self._master
        ts, tc = m.target_system, m.target_component
        rate = max(1, min(50, int(hz)))
        interval_us = max(1, int(1e6 / rate))
        # Legacy REQUEST_DATA_STREAM: ArduPilot still maps POSITION -> LOCAL_POSITION*, etc.
        m.mav.request_data_stream_send(
            ts,
            tc,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            rate,
            1,
        )
        m.mav.request_data_stream_send(
            ts,
            tc,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            rate,
            1,
        )
        m.mav.command_long_send(
            ts,
            tc,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )
        m.mav.command_long_send(
            ts,
            tc,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def stop(self) -> None:
        """Stop the MAVLink thread. Safe to call from any thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._master = None
        logger.info("MAVLinkWorker stopped for drone %s", self.drone_id)

    def _run_loop(self) -> None:
        """Single MAVLink thread: process queue, then read messages, update state."""
        while self._running and self._master:
            # Process one command from queue (non-blocking)
            try:
                cmd = self._command_queue.get_nowait()
                self._execute_command(cmd)
            except queue.Empty:
                pass

            # Drain inbound buffer via recv_msg (all types); update pose/attitude when seen.
            drained = 0
            while (
                self._running
                and self._master
                and drained < _MAX_RECV_DRAIN_PER_ITER
            ):
                msg = self._master.recv_msg()
                if msg is None:
                    break
                drained += 1
                mt = msg.get_type()
                if mt == "BAD_DATA":
                    continue
                if mt == "LOCAL_POSITION_NED":
                    with self._state_lock:
                        self._last_position = {
                            "x": msg.x,
                            "y": msg.y,
                            "z": msg.z,
                            "vx": getattr(msg, "vx", 0.0),
                            "vy": getattr(msg, "vy", 0.0),
                            "vz": getattr(msg, "vz", 0.0),
                        }
                elif mt == "ATTITUDE":
                    with self._state_lock:
                        self._last_attitude = {
                            "rx": msg.roll,
                            "ry": msg.pitch,
                            "rz": msg.yaw,
                        }

            time.sleep(0.01)

    def _execute_command(self, cmd: Dict[str, Any]) -> None:
        """Run one command (only called from _run_loop)."""
        cmd_type = cmd.get("type")
        if cmd_type == "rc_override":
            self._master.mav.rc_channels_override_send(
                self._master.target_system,
                self._master.target_component,
                cmd["chan1"],
                cmd["chan2"],
                cmd["chan3"],
                cmd["chan4"],
                0,
                0,
                0,
                0,
            )
        elif cmd_type == "set_mode":
            self._master.set_mode(cmd["mode_id"])
        elif cmd_type == "arm":
            self._master.arducopter_arm()
        elif cmd_type == "takeoff":
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1,
            )
        elif cmd_type == "request_position_stream":
            interval_us = int(1e6 / cmd.get("hz", 50))
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
        elif cmd_type == "request_attitude_stream":
            interval_us = int(1e6 / cmd.get("hz", 50))
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
        elif cmd_type == "sleep":
            time.sleep(cmd.get("sec", 0))
        elif cmd_type == "init_done":
            evt = cmd.get("event")
            if evt is not None:
                evt.set()

    def send_rc_override(
        self,
        chan1: int,
        chan2: int,
        chan3: int,
        chan4: int,
        controller: Optional[Any] = None,
    ) -> None:
        """
        Send RC_OVERRIDE (roll, pitch, throttle, yaw). Thread-safe via queue.

        Args:
            chan1: Roll.
            chan2: Pitch.
            chan3: Throttle.
            chan4: Yaw.
            controller: Optional; if has last_rc_channels and rc_channels_lock, update them.
        """
        try:
            self._command_queue.put_nowait(
                {
                    "type": "rc_override",
                    "chan1": chan1,
                    "chan2": chan2,
                    "chan3": chan3,
                    "chan4": chan4,
                }
            )
        except queue.Full:
            logger.warning("MAVLinkWorker command queue full, dropping rc_override")
        if controller is not None and hasattr(controller, "rc_channels_lock"):
            with controller.rc_channels_lock:
                controller.last_rc_channels["roll"] = chan1
                controller.last_rc_channels["pitch"] = chan2
                controller.last_rc_channels["throttle"] = chan3
                controller.last_rc_channels["yaw"] = chan4

    def send_set_mode(self, mode_id: int) -> None:
        """Enqueue set_mode. Thread-safe."""
        try:
            self._command_queue.put_nowait({"type": "set_mode", "mode_id": mode_id})
        except queue.Full:
            logger.warning("MAVLinkWorker command queue full, dropping set_mode")

    def get_position(self) -> Optional[Dict[str, float]]:
        """
        Return last LOCAL_POSITION_NED (x, y, z, vx, vy, vz). Thread-safe.

        Returns:
            Dict or None if no position received yet.
        """
        with self._state_lock:
            if self._last_position is None:
                return None
            return dict(self._last_position)

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """
        Return last ATTITUDE (roll, pitch, yaw in radians as rx, ry, rz). Thread-safe.

        Returns:
            Dict or None if no attitude received yet.
        """
        with self._state_lock:
            if self._last_attitude is None:
                return None
            return dict(self._last_attitude)

    def run_init_sequence(
        self, steps: List[Dict[str, Any]], timeout: float = 60.0
    ) -> bool:
        """
        Run a list of init commands in the MAVLink thread and block until done.

        Steps are dicts with "type": "set_mode"|"arm"|"takeoff"|"request_position_stream"|
        "request_attitude_stream"|"sleep" and corresponding args. A final "init_done"
        step is appended internally; the worker sets the event so this method unblocks.

        Args:
            steps: List of command dicts (e.g. {"type": "set_mode", "mode_id": 4}).
            timeout: Max seconds to wait for init_done.

        Returns:
            True if init_done was observed within timeout, False otherwise.
        """
        evt = threading.Event()
        for s in steps:
            try:
                self._command_queue.put_nowait(dict(s))
            except queue.Full:
                logger.warning("MAVLinkWorker init queue full")
                return False
        try:
            self._command_queue.put_nowait({"type": "init_done", "event": evt})
        except queue.Full:
            return False
        return evt.wait(timeout=timeout)
