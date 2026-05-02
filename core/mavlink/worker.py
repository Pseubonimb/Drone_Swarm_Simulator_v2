"""
MAVLink worker: single-threaded, thread-safe access to one drone connection.

All pymavlink operations (recv_match, rc_channels_override_send, set_mode, etc.)
run in one dedicated thread. Callers use get_position(), get_attitude(),
send_rc_override() and run_init_sequence() which enqueue commands or read
from thread-safe state cache (pose from SIM_STATE, NED from lat/lon vs home).
"""

import logging
import os
import queue
import threading
import time
from typing import Any, Dict, List, Optional, Union

from pymavlink import mavutil

from core.mavlink.geo_ned import (
    home_position_lat_lon_alt_m,
    ned_metres_from_home,
    sim_state_lat_lon_deg,
)

logger = logging.getLogger(__name__)

_SIM_STATE_MSG_ID: int = int(
    getattr(mavutil.mavlink, "MAVLINK_MSG_ID_SIM_STATE", 108)
)
_HOME_POSITION_MSG_ID: int = int(
    getattr(mavutil.mavlink, "MAVLINK_MSG_ID_HOME_POSITION", 242)
)


def _copter_mode_name_from_custom_mode(master: Any, custom_mode: int) -> str:
    """Resolve ArduCopter ``custom_mode`` from HEARTBEAT to a short mode label."""
    try:
        mm = master.mode_mapping()
        if mm:
            for name, mid in mm.items():
                if int(mid) == int(custom_mode):
                    return str(name).upper()
    except Exception:
        pass
    return f"MODE_{int(custom_mode)}"

# Main thread must not call recv_match / send on mavutil after the worker thread starts;
# TCP links are especially sensitive to split-thread use.

# Cap recv_msg drain per outer loop iteration so RC/command queue cannot starve indefinitely.
_MAX_RECV_DRAIN_PER_ITER = 512
_DEFAULT_TELEMETRY_HZ = 50
_TIME_SYNC_LOG_ENV = "MAVLINK_TIME_SYNC_LOG_DIR"


class MAVLinkWorker:
    """
    Thread-safe MAVLink access for one drone.

    One dedicated thread performs all recv_msg() / recv and mav.xxx_send() calls.
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
        self._last_position_sitl_time_boot_sec: Optional[float] = None
        self._last_position_py_rx_monotonic_sec: Optional[float] = None
        self._last_rc_enqueue_py_monotonic_sec: Optional[float] = None
        self._last_rc_sent_py_monotonic_sec: Optional[float] = None
        self._last_rc_sent_channels: Optional[Dict[str, int]] = None
        self._last_hypervisor_rx_to_tx_sec: Optional[float] = None
        self._pending_sitl_response_tx_py_monotonic_sec: Optional[float] = None
        self._pending_sitl_response_ref_sitl_time_boot_sec: Optional[float] = None
        self._last_sitl_cmd_to_response_py_sec: Optional[float] = None
        self._last_sitl_cmd_to_response_sitl_sec: Optional[float] = None
        self._last_flight_mode_name: str = ""
        self._home_lat_deg: float = 0.0
        self._home_lon_deg: float = 0.0
        self._home_alt_m: float = 0.0
        self._home_initialized: bool = False
        self._home_from_sim_fallback: bool = False
        # Latest SITL time_boot (s) from any MAVLink message carrying time_boot_ms
        # (SIM_STATE often omits it; HEARTBEAT keeps a usable clock for cmd->response deltas).
        self._last_vehicle_sitl_time_boot_sec: Optional[float] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._master: Any = None
        self._bootstrap_ready = threading.Event()
        self._bootstrap_error: Optional[BaseException] = None
        self._time_sync_log_file: Optional[Any] = None
        self._init_time_sync_log()

    def _init_time_sync_log(self) -> None:
        """Open optional CSV log for Python-vs-SITL time mapping."""
        log_dir = os.environ.get(_TIME_SYNC_LOG_ENV, "").strip()
        if not log_dir:
            return
        try:
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, f"mavlink_time_sync_drone_{self.drone_id}.csv")
            self._time_sync_log_file = open(path, "w", encoding="utf-8")
            self._time_sync_log_file.write(
                "event,drone_id,msg_type,py_monotonic_sec,sitl_time_boot_sec,"
                "queue_size,chan1,chan2,chan3,chan4\n"
            )
            self._time_sync_log_file.flush()
            logger.info(
                "MAVLinkWorker time-sync logging enabled for drone %s: %s",
                self.drone_id,
                path,
            )
        except OSError as exc:
            logger.warning(
                "Drone %s: cannot open time-sync log in %r: %s",
                self.drone_id,
                log_dir,
                exc,
            )
            self._time_sync_log_file = None

    def _close_time_sync_log(self) -> None:
        """Close optional time-sync CSV log."""
        if self._time_sync_log_file is None:
            return
        try:
            self._time_sync_log_file.close()
        except OSError:
            pass
        self._time_sync_log_file = None

    def _write_time_sync_row(
        self,
        *,
        event: str,
        msg_type: str = "",
        py_monotonic_sec: float,
        sitl_time_boot_sec: Optional[float] = None,
        queue_size: Optional[int] = None,
        chan1: Optional[int] = None,
        chan2: Optional[int] = None,
        chan3: Optional[int] = None,
        chan4: Optional[int] = None,
    ) -> None:
        """Append one row to optional time-sync CSV log."""
        if self._time_sync_log_file is None:
            return
        sitl_str = "" if sitl_time_boot_sec is None else f"{sitl_time_boot_sec:.6f}"
        queue_str = "" if queue_size is None else str(queue_size)
        c1 = "" if chan1 is None else str(chan1)
        c2 = "" if chan2 is None else str(chan2)
        c3 = "" if chan3 is None else str(chan3)
        c4 = "" if chan4 is None else str(chan4)
        try:
            self._time_sync_log_file.write(
                f"{event},{self.drone_id},{msg_type},{py_monotonic_sec:.6f},{sitl_str},"
                f"{queue_str},{c1},{c2},{c3},{c4}\n"
            )
            self._time_sync_log_file.flush()
        except OSError:
            logger.warning("Drone %s: failed writing time-sync log row", self.drone_id)

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
        """Request HOME_POSITION + SIM_STATE intervals (true sim pose, NED from lat/lon in worker)."""
        if self._master is None:
            return
        m = self._master
        ts, tc = m.target_system, m.target_component
        rate = max(1, min(50, int(hz)))
        interval_us = max(1, int(1e6 / rate))
        home_interval_us = max(1, int(5e5))  # 2 Hz: home for NED origin
        m.mav.command_long_send(
            ts,
            tc,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            _HOME_POSITION_MSG_ID,
            home_interval_us,
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
            _SIM_STATE_MSG_ID,
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
        self._close_time_sync_log()
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
                py_rx_ts = time.monotonic()
                if mt == "BAD_DATA":
                    continue
                msg_time_boot_ms = getattr(msg, "time_boot_ms", None)
                sitl_time_boot_sec = (
                    float(msg_time_boot_ms) / 1000.0
                    if msg_time_boot_ms is not None
                    else None
                )
                if sitl_time_boot_sec is not None:
                    self._last_vehicle_sitl_time_boot_sec = sitl_time_boot_sec
                self._write_time_sync_row(
                    event="rx",
                    msg_type=mt,
                    py_monotonic_sec=py_rx_ts,
                    sitl_time_boot_sec=sitl_time_boot_sec,
                    queue_size=self._command_queue.qsize(),
                )
                if mt == "HEARTBEAT" and self._master is not None:
                    try:
                        cm = int(getattr(msg, "custom_mode", -1))
                        if cm >= 0:
                            name = _copter_mode_name_from_custom_mode(self._master, cm)
                            with self._state_lock:
                                self._last_flight_mode_name = name
                    except (TypeError, ValueError):
                        pass
                if mt == "HOME_POSITION":
                    hlat, hlon, halt = home_position_lat_lon_alt_m(msg)
                    with self._state_lock:
                        self._home_lat_deg = hlat
                        self._home_lon_deg = hlon
                        self._home_alt_m = halt
                        self._home_initialized = True
                        self._home_from_sim_fallback = False
                elif mt == "SIM_STATE":
                    lat, lon = sim_state_lat_lon_deg(msg)
                    alt_m = float(getattr(msg, "alt", 0.0))
                    tbm = getattr(msg, "time_boot_ms", None)
                    if tbm is not None:
                        sitl_tb = float(tbm) / 1000.0
                    else:
                        sitl_tb = self._last_vehicle_sitl_time_boot_sec
                    with self._state_lock:
                        if not self._home_initialized:
                            self._home_lat_deg = lat
                            self._home_lon_deg = lon
                            self._home_alt_m = alt_m
                            self._home_initialized = True
                            self._home_from_sim_fallback = True
                            logger.info(
                                "Drone %s: NED origin latched from first SIM_STATE "
                                "(HOME_POSITION not yet applied)",
                                self.drone_id,
                            )
                        x, y, z = ned_metres_from_home(
                            lat,
                            lon,
                            alt_m,
                            self._home_lat_deg,
                            self._home_lon_deg,
                            self._home_alt_m,
                        )
                        self._last_position = {
                            "x": x,
                            "y": y,
                            "z": z,
                            "vx": float(getattr(msg, "vn", 0.0)),
                            "vy": float(getattr(msg, "ve", 0.0)),
                            "vz": float(getattr(msg, "vd", 0.0)),
                        }
                        self._last_attitude = {
                            "rx": float(getattr(msg, "roll", 0.0)),
                            "ry": float(getattr(msg, "pitch", 0.0)),
                            "rz": float(getattr(msg, "yaw", 0.0)),
                        }
                        self._last_position_sitl_time_boot_sec = sitl_tb
                        self._last_position_py_rx_monotonic_sec = py_rx_ts
                        if (
                            self._pending_sitl_response_tx_py_monotonic_sec is not None
                            and py_rx_ts >= self._pending_sitl_response_tx_py_monotonic_sec
                        ):
                            self._last_sitl_cmd_to_response_py_sec = (
                                py_rx_ts - self._pending_sitl_response_tx_py_monotonic_sec
                            )
                            if (
                                sitl_tb is not None
                                and self._pending_sitl_response_ref_sitl_time_boot_sec
                                is not None
                                and sitl_tb
                                >= self._pending_sitl_response_ref_sitl_time_boot_sec
                            ):
                                self._last_sitl_cmd_to_response_sitl_sec = (
                                    sitl_tb
                                    - self._pending_sitl_response_ref_sitl_time_boot_sec
                                )
                            self._pending_sitl_response_tx_py_monotonic_sec = None
                            self._pending_sitl_response_ref_sitl_time_boot_sec = None

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
            sent_ts = time.monotonic()
            with self._state_lock:
                self._last_rc_sent_py_monotonic_sec = sent_ts
                self._last_rc_sent_channels = {
                    "chan1": int(cmd["chan1"]),
                    "chan2": int(cmd["chan2"]),
                    "chan3": int(cmd["chan3"]),
                    "chan4": int(cmd["chan4"]),
                }
                if (
                    self._last_position_py_rx_monotonic_sec is not None
                    and sent_ts >= self._last_position_py_rx_monotonic_sec
                ):
                    self._last_hypervisor_rx_to_tx_sec = (
                        sent_ts - self._last_position_py_rx_monotonic_sec
                    )
                self._pending_sitl_response_tx_py_monotonic_sec = sent_ts
                self._pending_sitl_response_ref_sitl_time_boot_sec = (
                    self._last_position_sitl_time_boot_sec
                )
            self._write_time_sync_row(
                event="tx_sent_rc_override",
                msg_type="RC_OVERRIDE",
                py_monotonic_sec=sent_ts,
                queue_size=self._command_queue.qsize(),
                chan1=int(cmd["chan1"]),
                chan2=int(cmd["chan2"]),
                chan3=int(cmd["chan3"]),
                chan4=int(cmd["chan4"]),
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
            interval_us = int(1e6 / max(1, int(cmd.get("hz", 50))))
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                _SIM_STATE_MSG_ID,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
        elif cmd_type == "request_attitude_stream":
            interval_us = int(1e6 / max(1, int(cmd.get("hz", 50))))
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                _SIM_STATE_MSG_ID,
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
        py_enqueue_ts = time.monotonic()
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
            with self._state_lock:
                self._last_rc_enqueue_py_monotonic_sec = py_enqueue_ts
            self._write_time_sync_row(
                event="tx_enqueue_rc_override",
                msg_type="RC_OVERRIDE",
                py_monotonic_sec=py_enqueue_ts,
                queue_size=self._command_queue.qsize(),
                chan1=chan1,
                chan2=chan2,
                chan3=chan3,
                chan4=chan4,
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
        Return last pose from SIM_STATE: NED metres (x,y,z) from home and vn/ve/vd.

        Thread-safe.

        Returns:
            Dict or None if no SIM_STATE received yet.
        """
        with self._state_lock:
            if self._last_position is None:
                return None
            return dict(self._last_position)

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """
        Return last roll/pitch/yaw from SIM_STATE (radians as rx, ry, rz). Thread-safe.

        Returns:
            Dict or None if no SIM_STATE received yet.
        """
        with self._state_lock:
            if self._last_attitude is None:
                return None
            return dict(self._last_attitude)

    def get_timing_snapshot(self) -> Dict[str, Any]:
        """Return thread-safe snapshot of latest MAVLink timing markers and flight mode."""
        with self._state_lock:
            snap: Dict[str, Any] = {
                "last_position_sitl_time_boot_sec": self._last_position_sitl_time_boot_sec,
                "last_position_py_rx_monotonic_sec": self._last_position_py_rx_monotonic_sec,
                "last_rc_enqueue_py_monotonic_sec": self._last_rc_enqueue_py_monotonic_sec,
                "last_rc_sent_py_monotonic_sec": self._last_rc_sent_py_monotonic_sec,
                "hypervisor_rx_to_tx_sec": self._last_hypervisor_rx_to_tx_sec,
                "sitl_cmd_to_response_py_sec": self._last_sitl_cmd_to_response_py_sec,
                "sitl_cmd_to_response_sitl_sec": self._last_sitl_cmd_to_response_sitl_sec,
                "flight_mode": self._last_flight_mode_name or "",
            }
            if self._last_rc_sent_channels is not None:
                snap["last_rc_sent_chan1"] = float(self._last_rc_sent_channels["chan1"])
                snap["last_rc_sent_chan2"] = float(self._last_rc_sent_channels["chan2"])
                snap["last_rc_sent_chan3"] = float(self._last_rc_sent_channels["chan3"])
                snap["last_rc_sent_chan4"] = float(self._last_rc_sent_channels["chan4"])
            else:
                snap["last_rc_sent_chan1"] = None
                snap["last_rc_sent_chan2"] = None
                snap["last_rc_sent_chan3"] = None
                snap["last_rc_sent_chan4"] = None
            return snap

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
