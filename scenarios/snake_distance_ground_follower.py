"""
Two drones: leader flies like snake_pursuit (+Y via roll bias in POS_HOLD); follower
stays on the ground (no arm / no takeoff) but participates in swarm coordinate exchange.

Logs ``inter_drone_distance_m.csv`` (t, d_m) — 3D Euclidean distance in common NED — for
identification experiments (e.g. Dyadic). Standard per-drone CSVs are still written.
"""

import argparse
import logging
import os
import sys
import threading
import time
from datetime import datetime
from typing import Any, Dict, TextIO

_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from core.control import DroneController  # noqa: E402
from core.logging.csv_logger import CSV_HEADER, write_metadata  # noqa: E402
from core.mavlink.utils import RC_NEUTRAL, sitl_tcp_connection_string  # noqa: E402
from core.network import CoordExchangeStepContext, distance_3d  # noqa: E402

logger = logging.getLogger(__name__)

START_TIME = 0.0

# Same leader pipeline as snake_pursuit: takeoff, POS_HOLD, then streams for +Y roll loop.
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

# Follower: streams only — no arm, no takeoff (remains on ground; SITL still publishes pose).
FOLLOWER_GROUND_INIT_STEPS = [
    {"type": "request_position_stream", "hz": 50},
    {"type": "request_attitude_stream", "hz": 50},
    {"type": "sleep", "sec": 1.0},
]

DISTANCE_CSV_HEADER = "t,d_m\n"
FORMATION_D_STAR_M = 2.0


def leader_roll_positive_y_loop(
    controller: DroneController,
    experiment_duration: float = 0,
    roll_pwm: int = 1600,
    step_sec: float = 0.1,
) -> None:
    """Leader: roll bias for steady +Y in NED under POS_HOLD (same as snake_pursuit)."""
    global START_TIME
    if not controller.worker:
        return
    while True:
        if experiment_duration > 0 and (time.time() - START_TIME) >= experiment_duration:
            break
        controller.worker.send_rc_override(
            roll_pwm, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, controller=controller
        )
        time.sleep(step_sec)


def initialize_drone_parallel(
    controller: DroneController,
    init_steps: list,
    init_barrier: threading.Barrier,
) -> None:
    """Connect, run role-specific init, RC keepalive, then barrier."""
    try:
        controller.connect()
        controller.initialize(init_steps)
        controller.start_rc_keepalive()
        init_barrier.wait()
    except Exception:
        logger.exception("Drone init failed (id=%s)", controller.config.get("id"))
        raise


def _make_distance_on_step(distance_file: TextIO):
    """Return on_step callback: log elapsed time and 3D distance drone 1 ↔ 2 (common NED)."""

    def on_step(ctx: CoordExchangeStepContext) -> None:
        pos = ctx.positions
        if 1 not in pos or 2 not in pos:
            return
        d_m = distance_3d(pos[1], pos[2])
        distance_file.write(f"{ctx.time_elapsed:.6f},{d_m:.6f}\n")
        distance_file.flush()

    return on_step


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Leader +Y roll (snake-style); follower on ground; log inter-drone distance "
            "to inter_drone_distance_m.csv"
        )
    )
    parser.add_argument(
        "--leader-roll-pwm",
        type=int,
        default=1600,
        help="Leader roll RC (PWM); bias for +Y in NED under POS_HOLD",
    )
    parser.add_argument(
        "--leader-roll-step-sec",
        type=float,
        default=0.1,
        help="Sleep between leader RC_OVERRIDE updates (s)",
    )
    parser.add_argument("--run-id", type=str, default=None, help="Suffix for log folder name")
    parser.add_argument(
        "--drones",
        type=int,
        default=2,
        help="Must be 2 (leader + one ground follower)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Experiment duration in seconds (default 30)",
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
        help="Cap per-drone CSV rate (Hz); 0 = same cadence as exchange loop",
    )
    parser.add_argument(
        "--exchange-hz",
        type=float,
        default=50.0,
        help="Coordinate exchange / logging loop rate (Hz)",
    )
    parser.add_argument(
        "--sitl-direct-tcp",
        action="store_true",
        help="MAVLink TCP 5760+10*i (use with sim_vehicle --no-mavproxy)",
    )
    args = parser.parse_args()

    if args.drones != 2:
        logger.error("This scenario requires exactly --drones 2 (got %s).", args.drones)
        raise SystemExit(1)

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

    distance_path = os.path.join(experiment_dir, "inter_drone_distance_m.csv")
    distance_file = open(distance_path, "w", encoding="utf-8")
    distance_file.write(DISTANCE_CSV_HEADER)
    distance_file.flush()

    drones_config = []
    for i in range(args.drones):
        cfg: Dict[str, Any] = {
            "id": i + 1,
            "role": "leader" if i == 0 else "follower",
        }
        if args.sitl_direct_tcp or os.environ.get("DRONE_SWARM_SITL_DIRECT_TCP") == "1":
            cfg["mavlink_connection"] = sitl_tcp_connection_string(i)
        else:
            cfg["udp_port"] = 14551 + i * 10
        drones_config.append(cfg)

    experiment_log_files = {}
    for cfg in drones_config:
        did = cfg["id"]
        path = os.path.join(experiment_dir, f"drone_{did}.csv")
        f = open(path, "w", encoding="utf-8")
        f.write(CSV_HEADER + "\n")
        f.flush()
        experiment_log_files[did] = f

    write_metadata(
        experiment_dir,
        args.duration,
        args.collision_radius,
        args.drones,
        "snake_distance_ground_follower",
        extra={
            "distance_csv": "inter_drone_distance_m.csv",
            "distance_description": "3D Euclidean distance between drones in common NED (m)",
            "follower_behavior": "ground_no_takeoff",
        },
    )

    controllers = []
    for config in drones_config:
        controllers.append(DroneController(config, logging_enabled=False))

    init_barrier = threading.Barrier(len(controllers) + 1)

    def run_init(ctrl: DroneController) -> None:
        steps = LEADER_INIT_STEPS if ctrl.config["role"] == "leader" else FOLLOWER_GROUND_INIT_STEPS
        initialize_drone_parallel(ctrl, steps, init_barrier)

    for controller in controllers:
        threading.Thread(target=run_init, args=(controller,), daemon=False).start()

    try:
        init_barrier.wait(timeout=120)
    except threading.BrokenBarrierError:
        distance_file.close()
        return

    time.sleep(2)
    global START_TIME
    START_TIME = time.time()

    on_step = _make_distance_on_step(distance_file)
    DroneController.start_swarm_coord_exchange(
        controllers,
        experiment_start_time=START_TIME,
        experiment_log_files=experiment_log_files,
        duration=args.duration,
        collision_radius=args.collision_radius,
        log_hz=args.log_hz,
        exchange_loop_hz=args.exchange_hz,
        experiment_dir=experiment_dir,
        formation_d_star_m=FORMATION_D_STAR_M,
        on_step=on_step,
        daemon=True,
    )
    time.sleep(1)

    leader = next((c for c in controllers if c.config["role"] == "leader"), None)
    if not leader:
        distance_file.close()
        return

    leader_thread = threading.Thread(
        target=leader_roll_positive_y_loop,
        args=(leader,),
        kwargs={
            "experiment_duration": args.duration,
            "roll_pwm": args.leader_roll_pwm,
            "step_sec": args.leader_roll_step_sec,
        },
        daemon=True,
    )
    leader_thread.start()

    try:
        time.sleep(max(0.0, args.duration) + 2.0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            distance_file.close()
        except Exception:
            pass
        for controller in controllers:
            try:
                controller.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()
