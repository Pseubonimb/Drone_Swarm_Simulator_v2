"""
Leader forward-back: leader flies forward → stop → backward → stop.

All drones in POS_HOLD; NED/attitude via DroneController; PID from core.
"""
import json
import logging
import os
import sys
import threading
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

# Ensure project root is on path when run as script (e.g. python scenarios/leader_forward_back.py)
_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from core.control import DroneController, PIDRegulator
from core.logging.csv_logger import CSV_HEADER, write_metadata, write_row
from core.mavlink.utils import RC_NEUTRAL

try:
    from visualizer.position_publisher import publish_positions as _publish_positions
except ImportError:
    _publish_positions = None

logger = logging.getLogger(__name__)

RATES_SHARED = {"follower_hz": None, "exchange_hz": None, "webots_step_hz": None}
WEBOTS_LAST_TS = {}
FOLLOWER_HZ_HISTORY = []
FOLLOWER_HZ_WINDOW = 20
EXCHANGE_HZ_HISTORY = []
EXCHANGE_HZ_WINDOW = 20

START_TIME = 0.0

LEADER_INIT_STEPS = [
    {"type": "set_mode", "mode_id": 4},
    {"type": "sleep", "sec": 0.5},
    {"type": "arm"},
    {"type": "sleep", "sec": 2},
    {"type": "takeoff"},
    {"type": "sleep", "sec": 5},
    {"type": "set_mode", "mode_id": 16},
    {
        "type": "rc_override",
        "chan1": RC_NEUTRAL,
        "chan2": RC_NEUTRAL,
        "chan3": RC_NEUTRAL,
        "chan4": RC_NEUTRAL,
    },
    {"type": "sleep", "sec": 0.5},
    {"type": "request_position_stream", "hz": 50},
    {"type": "request_attitude_stream", "hz": 50},
    {"type": "sleep", "sec": 0.2},
]


class FollowerPIDPursuit:
    """Follow predecessor in common NED with roll/pitch PID (this scenario only).

    Bundles DroneController, fixed PID instances, and one-shot vs loop control so
    position/PID state is not threaded through long function signatures.
    """

    def __init__(
        self,
        drone: DroneController,
        predecessor_id: int,
        kp: float = 500.0,
        ki: float = 0.0,
        kd: float = 0.0,
        ki_pitch: Optional[float] = None,
        control_loop_period_sec: Optional[float] = None,
        experiment_duration: float = 0,
    ) -> None:
        self._drone = drone
        self._predecessor_id = predecessor_id
        self._control_loop_period_sec = control_loop_period_sec
        self._experiment_duration = experiment_duration
        pitch_ki = ki if ki_pitch is None else ki_pitch
        self._roll_pid = PIDRegulator(
            kp=kp, ki=ki, kd=kd, integral_limit=100.0, output_limit=200.0
        )
        self._pitch_pid = PIDRegulator(
            kp=kp, ki=pitch_ki, kd=kd, integral_limit=100.0, output_limit=200.0
        )

    def _my_position_common(self) -> Dict[str, float]:
        raw = self._drone.get_my_position()
        did = self._drone.config["id"]
        return {
            **raw,
            "y": raw["y"] - (did - 1) * 2.0,
        }

    def pid_step_toward(
        self,
        target_common: Dict[str, float],
        dt: Optional[float],
    ) -> Tuple[float, float]:
        """Apply roll/pitch PID toward ``target_common``; send RC_OVERRIDE.

        Same errors as before: error_x = rel x; error_y = rel y - 2 (formation spacing).

        Returns:
            ``(pitch_output, roll_output)`` before RC mapping.
        """
        w = self._drone.worker
        if w is None:
            return 0.0, 0.0
        me = self._my_position_common()
        rel_x = target_common["x"] - me["x"]
        rel_y = target_common["y"] - me["y"]
        rel_z = target_common["z"] - me["z"]
        error_x = rel_x
        error_y = rel_y - 2
        pitch_output = self._pitch_pid.update(error_x, dt=dt)
        roll_output = self._roll_pid.update(error_y, dt=dt)
        pitch_pwm = RC_NEUTRAL - int(pitch_output)
        roll_pwm = RC_NEUTRAL + int(roll_output)
        w.send_rc_override(
            roll_pwm, pitch_pwm, RC_NEUTRAL, RC_NEUTRAL, controller=self._drone
        )
        return float(pitch_output), float(roll_output)

    def run_control_loop(self) -> None:
        """Run until experiment end: read predecessor pose, PID step, optional fixed-period sleep."""
        global START_TIME, FOLLOWER_HZ_HISTORY, RATES_SHARED
        last_iter_time: Optional[float] = None
        dt_loop = (
            self._control_loop_period_sec
            if (
                self._control_loop_period_sec is not None
                and self._control_loop_period_sec > 0
            )
            else None
        )
        try:
            while True:
                if self._experiment_duration > 0 and (
                    time.time() - START_TIME
                ) >= self._experiment_duration:
                    break
                other = self._drone.get_other_drones_positions()
                if self._predecessor_id not in other:
                    time.sleep(0.1)
                    continue
                now = time.time()
                loop_dt = (now - last_iter_time) if last_iter_time is not None else 0.0
                last_iter_time = now
                if loop_dt > 0:
                    FOLLOWER_HZ_HISTORY.append(1.0 / loop_dt)
                    if len(FOLLOWER_HZ_HISTORY) > FOLLOWER_HZ_WINDOW:
                        FOLLOWER_HZ_HISTORY = FOLLOWER_HZ_HISTORY[
                            -FOLLOWER_HZ_WINDOW:
                        ]
                    RATES_SHARED["follower_hz"] = sum(FOLLOWER_HZ_HISTORY) / len(
                        FOLLOWER_HZ_HISTORY
                    )
                leader_pos = other[self._predecessor_id]
                self.pid_step_toward(leader_pos, dt=dt_loop)
                if dt_loop is not None:
                    time.sleep(dt_loop)
        except KeyboardInterrupt:
            pass


def leader_pattern(
    controller: DroneController,
    forward_duration: float = 10,
    experiment_duration: float = 0,
) -> None:
    """Leader: forward → stop → backward → stop. pitch 1400=forward, 1600=backward.

    Args:
        controller: Leader drone controller (MAVLink worker).
        forward_duration: Seconds to fly forward and again backward before stopping.
        experiment_duration: If > 0, stop when global experiment time exceeds this (s).
    """
    global START_TIME
    if not controller.worker:
        return
    start = time.time()
    while time.time() - start < forward_duration:
        if experiment_duration > 0 and (time.time() - START_TIME) >= experiment_duration:
            break
        controller.worker.send_rc_override(
            RC_NEUTRAL, 1400, 1500, 1500, controller
        )
        time.sleep(0.1)
    controller.worker.send_set_mode(17)  # BRAKE
    time.sleep(2)
    controller.worker.send_set_mode(16)  # POS_HOLD
    time.sleep(0.3)
    controller.worker.send_rc_override(
        RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, controller=controller
    )
    time.sleep(0.5)
    if experiment_duration > 0 and (time.time() - START_TIME) >= experiment_duration:
        return
    start = time.time()
    while time.time() - start < forward_duration:
        if experiment_duration > 0 and (time.time() - START_TIME) >= experiment_duration:
            break
        controller.worker.send_rc_override(
            RC_NEUTRAL, 1600, 1500, 1500, controller
        )
        time.sleep(0.1)
    controller.worker.send_set_mode(17)
    time.sleep(2)
    controller.worker.send_set_mode(16)
    time.sleep(0.3)
    controller.worker.send_rc_override(
        RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, controller=controller
    )


def _read_webots_step_hz(num_instances: int = 2) -> Optional[float]:
    """Read Webots step timestamps from /tmp and compute min step rate in Hz."""
    global WEBOTS_LAST_TS
    hz_list = []
    for i in range(num_instances):
        path = f"/tmp/webots_step_{i}.txt"
        try:
            with open(path, "r") as f:
                t_curr = float(f.read().strip())
        except (FileNotFoundError, ValueError, OSError):
            continue
        if i in WEBOTS_LAST_TS:
            dt = t_curr - WEBOTS_LAST_TS[i]
            if dt > 0:
                hz_list.append(1.0 / dt)
        WEBOTS_LAST_TS[i] = t_curr
    return min(hz_list) if hz_list else None


# Nominal formation spacing (m): desired distance between consecutive drones along Y
FORMATION_D_STAR_M = 2.0


def _distance_3d(pi: Dict[str, float], pj: Dict[str, float]) -> float:
    """Euclidean distance between two position dicts (x, y, z)."""
    return (
        (pi["x"] - pj["x"]) ** 2
        + (pi["y"] - pj["y"]) ** 2
        + (pi["z"] - pj["z"]) ** 2
    ) ** 0.5


def _formation_metrics_step(
    positions: Dict[int, Dict[str, float]],
    collision_radius: float,
    d_star: float = FORMATION_D_STAR_M,
) -> tuple:
    """Compute formation error, min distance and collision flag for current step.

    Returns:
        (formation_error_max_ij, min_distance_ij, has_collision_this_step)
        - formation_error_max_ij: max over pairs (i,j) of |d_ij - d*_ij|, d*_ij = d_star * |i - j|
        - min_distance_ij: minimum pairwise distance (m)
        - has_collision_this_step: True if any pair has d_ij < 2*collision_radius
    """
    ids = sorted(positions.keys())
    formation_error = 0.0
    min_dist = float("inf")
    has_collision = False
    for i in ids:
        for j in ids:
            if i >= j:
                continue
            pi, pj = positions[i], positions[j]
            d_ij = _distance_3d(pi, pj)
            d_star_ij = d_star * abs(i - j)
            formation_error = max(formation_error, abs(d_ij - d_star_ij))
            min_dist = min(min_dist, d_ij)
            if d_ij < 2 * collision_radius:
                has_collision = True
    min_dist = min_dist if min_dist != float("inf") else 0.0
    return (formation_error, min_dist, has_collision)


def _compute_collision_flags(
    positions: Dict[int, Dict[str, float]], collision_radius: float
) -> Dict[int, bool]:
    """For each drone_id return True if in collision with any other (spheres 2*R)."""
    ids = list(positions.keys())
    collision = {i: False for i in ids}
    for i in ids:
        for j in ids:
            if i >= j:
                continue
            pi, pj = positions[i], positions[j]
            d = _distance_3d(pi, pj)
            if d < 2 * collision_radius:
                collision[i] = True
                collision[j] = True
    return collision


def coordinate_exchange_loop(
    controllers: List[DroneController],
    experiment_log_files: Optional[Dict[int, Any]] = None,
    duration: float = 0,
    collision_radius: float = 0.2,
    log_hz: float = 0.0,
    exchange_loop_hz: float = 50.0,
    experiment_dir: Optional[str] = None,
) -> None:
    """Exchange positions between drones, compute collisions, write CSV rows until duration.

    Loop rate is synchronized with SITL position stream (exchange_loop_hz, default 50 Hz).
    CSV is written only when position/attitude changed; log_hz optionally caps write rate.
    Aligning loop rate with SITL avoids flooding the 2D visualizer and reduces lag.
    Reported exchange_hz is the actual loop rate (full period including sleep), not just body time.
    Formation metrics (max |d_ij - d*|, min distance, steps with collision) are written to
    experiment_dir/formation_metrics.json on exit.

    Args:
        controllers: List of drone controllers (get_my_position, update_other_drone_position).
        experiment_log_files: Optional dict drone_id -> file handle for CSV rows.
        duration: If > 0, stop when time since START_TIME exceeds this (s).
        collision_radius: Sphere radius for collision detection (m); collision if d < 2*radius.
        log_hz: Max CSV write rate (Hz); 0 = write when position/attitude changed (sync SITL).
        exchange_loop_hz: Main loop rate (Hz); 0 = no limit. Use 50 to match SITL rate.
        experiment_dir: If set, write formation_metrics.json here on exit.
    """
    global EXCHANGE_HZ_HISTORY, START_TIME
    last_written: Dict[int, tuple] = {}  # drone_id -> (x, y, z, rx, ry, rz)
    last_csv_log_time = 0.0
    log_period = (1.0 / log_hz) if log_hz > 0 else 0.0
    loop_period = (1.0 / exchange_loop_hz) if exchange_loop_hz > 0 else 0.0
    # Formation metrics over the run
    formation_error_max_run = 0.0
    min_distance_run = float("inf")
    steps_with_collision = 0
    try:
        while True:
            if duration > 0 and (time.time() - START_TIME) >= duration:
                break
            t0 = time.time()
            positions = {}
            attitudes = {}
            for controller in controllers:
                did = controller.config["id"]
                pos_raw = controller.get_my_position()
                # Convert to common NED frame for control/metrics:
                # each SITL has its own home (Y offset 0, 2, 4, ... m) in launch setup.
                # Subtract (did-1)*2 so common initial positions are 0, -2, -4, ...
                # and match desired formation sign in follower control.
                positions[did] = {
                    **pos_raw,
                    "y": pos_raw["y"] - (did - 1) * 2.0,
                }
                attitudes[did] = controller.get_attitude()
            for controller in controllers:
                for drone_id, position in positions.items():
                    if drone_id != controller.config["id"]:
                        controller.update_other_drone_position(drone_id, position)
            RATES_SHARED["webots_step_hz"] = _read_webots_step_hz(
                num_instances=len(controllers)
            )
            if _publish_positions is not None:
                try:
                    _publish_positions(positions, rates=RATES_SHARED)
                except Exception:
                    pass
            if experiment_log_files or experiment_dir:
                now = time.time()
                t = now - START_TIME
                rate_ok = log_period <= 0 or (now - last_csv_log_time) >= log_period
                collision_flags = _compute_collision_flags(
                    positions, collision_radius
                )
                # Formation metrics this step
                formation_err_step, min_dist_step, has_collision_step = (
                    _formation_metrics_step(positions, collision_radius)
                )
                formation_error_max_run = max(
                    formation_error_max_run, formation_err_step
                )
                if min_dist_step < min_distance_run:
                    min_distance_run = min_dist_step
                if has_collision_step:
                    steps_with_collision += 1
                wrote_this_iter = False
                if experiment_log_files:
                    for controller in controllers:
                        did = controller.config["id"]
                        pos = positions[did]  # already in common frame (y includes (did-1)*2)
                        att = attitudes[did]
                        state = (
                            pos["x"], pos["y"], pos["z"],
                            att["rx"], att["ry"], att["rz"],
                        )
                        changed = last_written.get(did) != state
                        if changed and rate_ok:
                            last_written[did] = state
                            hc = 1 if collision_flags[did] else 0
                            write_row(
                                experiment_log_files[did],
                                did,
                                t,
                                pos["x"],
                                pos["y"],
                                pos["z"],
                                att["rx"],
                                att["ry"],
                                att["rz"],
                                hc,
                            )
                            wrote_this_iter = True
                if wrote_this_iter:
                    last_csv_log_time = now
            if loop_period > 0:
                elapsed = time.time() - t0
                sleep_sec = loop_period - elapsed
                if sleep_sec > 0:
                    time.sleep(sleep_sec)
            # Report actual loop rate (full period including sleep), not just body time
            dt_full = time.time() - t0
            if dt_full > 0:
                EXCHANGE_HZ_HISTORY.append(1.0 / dt_full)
                if len(EXCHANGE_HZ_HISTORY) > EXCHANGE_HZ_WINDOW:
                    EXCHANGE_HZ_HISTORY = EXCHANGE_HZ_HISTORY[-EXCHANGE_HZ_WINDOW:]
                RATES_SHARED["exchange_hz"] = (
                    sum(EXCHANGE_HZ_HISTORY) / len(EXCHANGE_HZ_HISTORY)
                )
    finally:
        if experiment_log_files:
            for did, f in experiment_log_files.items():
                try:
                    f.close()
                except Exception:
                    pass
        if experiment_dir:
            min_dist_final = (
                min_distance_run
                if min_distance_run != float("inf")
                else None
            )
            metrics = {
                "formation_error_max_m": round(formation_error_max_run, 6),
                "min_distance_m": (
                    round(min_dist_final, 6) if min_dist_final is not None else None
                ),
                "steps_with_collision": steps_with_collision,
                "d_star_m": FORMATION_D_STAR_M,
            }
            path = os.path.join(experiment_dir, "formation_metrics.json")
            try:
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(metrics, f, indent=2)
                logger.info("Wrote formation metrics to %s", path)
            except OSError as e:
                logger.warning("Could not write formation_metrics.json: %s", e)


def initialize_drone_parallel(
    controller: DroneController, init_barrier: threading.Barrier
) -> None:
    """Connect, initialize, start RC keepalive, then wait on barrier (called from worker thread)."""
    try:
        controller.connect()
        controller.initialize(LEADER_INIT_STEPS)
        controller.start_rc_keepalive()
        init_barrier.wait()
    except Exception:
        logger.exception("Drone init failed")
        raise


def main() -> None:
    """Parse args, create controllers, run leader + followers and coordinate_exchange_loop."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Leader forward-back + follower (leader_forward_back)"
    )
    parser.add_argument("--kp", type=float, default=8.0, help="P gain for follower PID")
    parser.add_argument("--ki", type=float, default=0.0, help="I gain for roll (lateral); pitch too if --ki-pitch omitted")
    parser.add_argument(
        "--ki-pitch",
        type=float,
        default=None,
        help=(
            "I gain for pitch (longitudinal X) only; roll keeps --ki. "
            "Omit to use --ki for both axes (legacy). Try 0.02–0.15 to trim steady-state X gap."
        ),
    )
    parser.add_argument("--kd", type=float, default=10.0, help="D gain")
    parser.add_argument(
        "--control-hz",
        type=float,
        default=50.0,
        help="Control loop rate in Hz; 0 = no limit",
    )
    parser.add_argument(
        "--leader-duration",
        type=float,
        default=10.0,
        help="Leader forward/back duration (s)",
    )
    parser.add_argument("--run-id", type=str, default=None, help="Suffix for log file")
    parser.add_argument(
        "--drones",
        type=int,
        default=2,
        help="Number of drones (must match launcher); drone 1 = leader, rest = followers",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0,
        help="Max experiment duration in seconds (0 = no limit)",
    )
    parser.add_argument(
        "--experiment-dir",
        type=str,
        default=None,
        help="Folder for experiment logs",
    )
    parser.add_argument(
        "--collision-radius",
        type=float,
        default=0.2,
        help="Drone sphere radius for collision detection (m)",
    )
    parser.add_argument(
        "--log-hz",
        type=float,
        default=0.0,
        help=(
            "Max CSV log write rate (Hz); 0 = write when position/attitude changed "
            "(sync with SITL)"
        ),
    )
    parser.add_argument(
        "--exchange-hz",
        type=float,
        default=50.0,
        help=(
            "Coordinate exchange loop rate (Hz); match SITL position stream "
            "(default 50). 0 = no limit."
        ),
    )
    args = parser.parse_args()
    args.control_loop_period_sec = (
        1.0 / args.control_hz if args.control_hz and args.control_hz > 0 else None
    )
    args.log_run_id = args.run_id

    drones_config = [
        {
            "id": i + 1,
            "udp_port": 14551 + i * 10,
            "role": "leader" if i == 0 else "follower",
        }
        for i in range(args.drones)
    ]

    experiment_dir = args.experiment_dir
    if experiment_dir is None:
        if args.run_id:
            experiment_dir = os.path.join("experiments", f"exp_{args.run_id}")
        else:
            experiment_dir = os.path.join(
                "experiments",
                datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            )
    os.makedirs(experiment_dir, exist_ok=True)
    experiment_log_files = {}
    for cfg in drones_config:
        did = cfg["id"]
        path = os.path.join(experiment_dir, f"drone_{did}.csv")
        f = open(path, "w")
        f.write(CSV_HEADER + "\n")
        f.flush()
        experiment_log_files[did] = f
    write_metadata(
        experiment_dir,
        args.duration,
        args.collision_radius,
        args.drones,
        "leader_forward_back",
    )

    controllers = []
    for config in drones_config:
        controllers.append(DroneController(config, logging_enabled=False))
    init_barrier = threading.Barrier(len(controllers) + 1)
    for controller in controllers:
        threading.Thread(
            target=initialize_drone_parallel,
            args=(controller, init_barrier),
            daemon=False,
        ).start()
    try:
        init_barrier.wait(timeout=30)
    except threading.BrokenBarrierError:
        return
    time.sleep(2)
    global START_TIME
    START_TIME = time.time()
    threading.Thread(
        target=coordinate_exchange_loop,
        args=(controllers,),
        kwargs={
            "experiment_log_files": experiment_log_files,
            "duration": args.duration,
            "collision_radius": args.collision_radius,
            "log_hz": getattr(args, "log_hz", 0.0),
            "exchange_loop_hz": getattr(args, "exchange_hz", 50.0),
            "experiment_dir": experiment_dir,
        },
        daemon=True,
    ).start()
    time.sleep(1)

    try:
        leader = next((c for c in controllers if c.config["role"] == "leader"), None)
        followers = [c for c in controllers if c.config["role"] == "follower"]
        if not leader:
            return
        leader_duration = getattr(args, "leader_duration", 10.0)
        threading.Thread(
            target=leader_pattern,
            args=(leader, leader_duration),
            kwargs={"experiment_duration": args.duration},
            daemon=True,
        ).start()
        def run_follower(follower: DroneController) -> None:
            FollowerPIDPursuit(
                follower,
                predecessor_id=follower.config["id"] - 1,
                kp=args.kp,
                ki=args.ki,
                kd=args.kd,
                ki_pitch=args.ki_pitch,
                control_loop_period_sec=args.control_loop_period_sec,
                experiment_duration=args.duration,
            ).run_control_loop()

        follower_threads = []
        for fc in followers:
            t = threading.Thread(target=run_follower, args=(fc,), daemon=False)
            t.start()
            follower_threads.append(t)
        if follower_threads:
            for t in follower_threads:
                t.join()
            if args.duration > 0:
                for controller in controllers:
                    try:
                        controller.stop()
                    except Exception:
                        pass
        else:
            threading.Event().wait()
    except KeyboardInterrupt:
        for controller in controllers:
            try:
                controller.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()
