"""
Leader forward-back: leader flies forward → stop → backward → stop.
All drones in POS_HOLD; CoordsMonitor and VelocityMonitor from core; PID from core.
"""
import os
import sys

# Ensure project root is on path when run as script (e.g. python scenarios/leader_forward_back.py)
_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

import logging
import threading
import time
from datetime import datetime
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

from core.control import DroneController, PIDRegulator
from core.logging.csv_logger import CSV_HEADER, write_metadata, write_row
from core.mavlink.utils import RC_NEUTRAL

try:
    from visualizer.position_publisher import publish_positions as _publish_positions
except ImportError:
    _publish_positions = None

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


def move_towards_with_pid(
    controller: DroneController,
    target_position: Dict[str, float],
    roll_pid: Optional[PIDRegulator] = None,
    pitch_pid: Optional[PIDRegulator] = None,
    dt: Optional[float] = None,
    distance_threshold: float = 0.4,
) -> None:
    """Move controller toward target using optional PIDs; send neutral if within threshold.

    Args:
        controller: Drone controller with coords_monitor and worker.
        target_position: Target NED position dict with 'x', 'y', 'z'.
        roll_pid: Optional PID for lateral (y) error.
        pitch_pid: Optional PID for longitudinal (x) error.
        dt: Optional time step for PID update (seconds).
        distance_threshold: Distance below which to send neutral and reset PIDs (meters).

    Returns:
        None. Sends RC_OVERRIDE via controller.worker.
    """
    if controller.coords_monitor is None or controller.worker is None:
        return
    rel_pos = controller.coords_monitor.get_relative_position(target_position)
    error_x = rel_pos["x"]
    error_y = rel_pos["y"] - 2
    distance = (error_x**2 + error_y**2) ** 0.5
    if distance < distance_threshold:
        controller.worker.send_rc_override(
            RC_NEUTRAL,
            RC_NEUTRAL,
            RC_NEUTRAL,
            RC_NEUTRAL,
            controller=controller,
        )
        if roll_pid is not None:
            roll_pid.reset()
        if pitch_pid is not None:
            pitch_pid.reset()
        return
    pitch_output = (
        pitch_pid.update(error_x, dt=dt)
        if pitch_pid is not None
        else error_x * 500.0
    )
    roll_output = (
        roll_pid.update(error_y, dt=dt)
        if roll_pid is not None
        else error_y * 500.0
    )
    pitch = RC_NEUTRAL - int(pitch_output)
    roll = RC_NEUTRAL + int(roll_output)
    controller.worker.send_rc_override(
        roll, pitch, RC_NEUTRAL, RC_NEUTRAL, controller=controller
    )


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


def follower_loop(
    controller: DroneController,
    target_drone_id: int,
    kp: float = 500.0,
    ki: float = 0.0,
    kd: float = 0.0,
    control_loop_period_sec: Optional[float] = None,
    log_run_id: Optional[str] = None,
    experiment_duration: float = 0,
) -> None:
    """Run follower control loop: track leader position with PID and log to CSV.

    Args:
        controller: Follower drone controller.
        target_drone_id: Leader drone id to follow (used as key in other_drones_positions).
        kp: Proportional gain for roll/pitch PID.
        ki: Integral gain.
        kd: Derivative gain.
        control_loop_period_sec: If set, sleep this long between iterations.
        log_run_id: Optional suffix for log filename (logs/two_drones_log_<suffix>.csv).
        experiment_duration: If > 0, exit when global experiment time exceeds this (s).
    """
    global START_TIME, FOLLOWER_HZ_HISTORY, RATES_SHARED
    roll_pid = PIDRegulator(
        kp=kp, ki=ki, kd=kd, integral_limit=100.0, output_limit=200.0
    )
    pitch_pid = PIDRegulator(
        kp=kp, ki=ki, kd=kd, integral_limit=100.0, output_limit=200.0
    )
    os.makedirs("logs", exist_ok=True)
    suffix = f"_{log_run_id}" if log_run_id else ""
    log_filename = f"logs/two_drones_log{suffix}.csv"
    log_file = open(log_filename, "w")
    log_file.write(
        "t,leader_x,leader_y,follower_x,follower_y,error_x,error_y,follower_vx,follower_vy,loop_dt\n"
    )
    log_file.flush()
    last_iter_time = None
    try:
        while True:
            if experiment_duration > 0 and (time.time() - START_TIME) >= experiment_duration:
                break
            other_positions = controller.get_other_drones_positions()
            if target_drone_id not in other_positions:
                time.sleep(0.1)
                continue
            now = time.time()
            loop_dt = (now - last_iter_time) if last_iter_time is not None else 0.0
            last_iter_time = now
            if loop_dt > 0:
                FOLLOWER_HZ_HISTORY.append(1.0 / loop_dt)
                if len(FOLLOWER_HZ_HISTORY) > FOLLOWER_HZ_WINDOW:
                    FOLLOWER_HZ_HISTORY = FOLLOWER_HZ_HISTORY[-FOLLOWER_HZ_WINDOW:]
                RATES_SHARED["follower_hz"] = sum(FOLLOWER_HZ_HISTORY) / len(FOLLOWER_HZ_HISTORY)
            leader_pos = other_positions[target_drone_id]
            follower_pos = controller.get_my_position()
            rel_pos = controller.coords_monitor.get_relative_position(leader_pos)
            error_x = rel_pos["x"]
            error_y = rel_pos["y"] - 2.0
            vel = controller.velocity_monitor.get_velocity()
            vx, vy = vel["vx"], vel["vy"]
            t = now - START_TIME
            log_file.write(
                f"{t:.3f},{leader_pos['x']:.3f},{leader_pos['y']:.3f},"
                f"{follower_pos['x']:.3f},{follower_pos['y']:.3f},"
                f"{error_x:.3f},{error_y:.3f},{vx:.3f},{vy:.3f},{loop_dt:.4f}\n"
            )
            log_file.flush()
            dt_loop = (
                control_loop_period_sec
                if (control_loop_period_sec is not None and control_loop_period_sec > 0)
                else None
            )
            move_towards_with_pid(
                controller, leader_pos, roll_pid, pitch_pid, dt=dt_loop
            )
            if dt_loop is not None:
                time.sleep(dt_loop)
    except KeyboardInterrupt:
        pass
    finally:
        log_file.close()


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
            d = (
                (pi["x"] - pj["x"]) ** 2
                + (pi["y"] - pj["y"]) ** 2
                + (pi["z"] - pj["z"]) ** 2
            ) ** 0.5
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
) -> None:
    """Exchange positions between drones, compute collisions, write CSV rows until duration.

    Loop rate is synchronized with SITL position stream (exchange_loop_hz, default 50 Hz).
    CSV is written only when position/attitude changed; log_hz optionally caps write rate.
    Aligning loop rate with SITL avoids flooding the 2D visualizer and reduces lag.
    Reported exchange_hz is the actual loop rate (full period including sleep), not just body time.

    Args:
        controllers: List of drone controllers (each has get_my_position, update_other_drone_position).
        experiment_log_files: Optional dict drone_id -> file handle for CSV rows (t,x,y,z,rx,ry,rz,hasCollision).
        duration: If > 0, stop when time since START_TIME exceeds this (s).
        collision_radius: Sphere radius for collision detection (m); collision if distance < 2*radius.
        log_hz: Max CSV write rate (Hz); 0 = write only when position/attitude changed (full sync with SITL).
        exchange_loop_hz: Main loop rate (Hz); 0 = no limit. Use 50 to match SITL LOCAL_POSITION_NED rate.
    """
    global EXCHANGE_HZ_HISTORY, START_TIME
    last_written: Dict[int, tuple] = {}  # drone_id -> (x, y, z, rx, ry, rz)
    last_csv_log_time = 0.0
    log_period = (1.0 / log_hz) if log_hz > 0 else 0.0
    loop_period = (1.0 / exchange_loop_hz) if exchange_loop_hz > 0 else 0.0
    try:
        while True:
            if duration > 0 and (time.time() - START_TIME) >= duration:
                break
            t0 = time.time()
            positions = {}
            attitudes = {}
            for controller in controllers:
                did = controller.config["id"]
                positions[did] = controller.get_my_position()
                attitudes[did] = controller.coords_monitor.get_attitude()
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
            if experiment_log_files:
                now = time.time()
                t = now - START_TIME
                rate_ok = log_period <= 0 or (now - last_csv_log_time) >= log_period
                collision_flags = _compute_collision_flags(
                    positions, collision_radius
                )
                wrote_this_iter = False
                for controller in controllers:
                    did = controller.config["id"]
                    pos = positions[did]
                    att = attitudes[did]
                    # Initial row: enforce Y=(k-1)*2 so 2D viz and replay show correct spacing
                    is_first_row = did not in last_written
                    y_val = (did - 1) * 2.0 if is_first_row else pos["y"]
                    state = (
                        pos["x"], y_val, pos["z"],
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
                            y_val,
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
    parser.add_argument("--ki", type=float, default=0.0, help="I gain")
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
        help="Max CSV log write rate (Hz); 0 = write only when position/attitude changed (sync with SITL)",
    )
    parser.add_argument(
        "--exchange-hz",
        type=float,
        default=50.0,
        help="Coordinate exchange loop rate (Hz); match SITL position stream (default 50). 0 = no limit.",
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
        def run_follower(follower_controller: DroneController) -> None:
            target_id = follower_controller.config["id"] - 1
            run_id_val = getattr(args, "log_run_id", "") or "run"
            log_id = f"{run_id_val}_follower_{follower_controller.config['id']}".strip("_")
            follower_loop(
                follower_controller,
                target_id,
                kp=getattr(args, "kp", 8.0),
                ki=getattr(args, "ki", 0.0),
                kd=getattr(args, "kd", 10.0),
                control_loop_period_sec=getattr(args, "control_loop_period_sec", None),
                log_run_id=log_id,
                experiment_duration=args.duration,
            )
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
