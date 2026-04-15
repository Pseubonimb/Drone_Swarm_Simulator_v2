"""
Snake pursuit: leader holds POS_HOLD and biases roll for steady +Y motion (NED, East);
followers chase predecessor in common NED (chain). Separate module state from leader_forward_back.
"""

import logging
import os
import sys
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from core.control import DroneController, PIDRegulator  # noqa: E402
from core.logging.csv_logger import CSV_HEADER, write_metadata  # noqa: E402
from core.mavlink.utils import RC_NEUTRAL, sitl_tcp_connection_string  # noqa: E402

logger = logging.getLogger(__name__)

RATES_SHARED = {"follower_hz": None, "exchange_hz": None, "webots_step_hz": None}
FOLLOWER_HZ_HISTORY: list[float] = []
FOLLOWER_HZ_WINDOW = 20

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
    """Follow predecessor in common NED with roll/pitch PID (snake_pursuit only)."""

    def __init__(
        self,
        drone: DroneController,
        predecessor_id: int,
        kp: float = 500.0,
        ki: float = 0.0,
        kd: float = 0.0,
        ki_pitch: Optional[float] = None,
        derivative_alpha: float = 0.0,
        control_loop_period_sec: Optional[float] = None,
        experiment_duration: float = 0,
    ) -> None:
        self._drone = drone
        self._predecessor_id = predecessor_id
        self._control_loop_period_sec = control_loop_period_sec
        self._experiment_duration = experiment_duration
        pitch_ki = ki if ki_pitch is None else ki_pitch
        self._roll_pid = PIDRegulator(
            kp=kp,
            ki=ki,
            kd=kd,
            integral_limit=100.0,
            output_limit=200.0,
            derivative_alpha=derivative_alpha,
        )
        self._pitch_pid = PIDRegulator(
            kp=kp,
            ki=pitch_ki,
            kd=kd,
            integral_limit=100.0,
            output_limit=200.0,
            derivative_alpha=derivative_alpha,
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
        """Apply roll/pitch PID toward ``target_common``; send RC_OVERRIDE."""
        w = self._drone.worker
        if w is None:
            return 0.0, 0.0
        me = self._my_position_common()
        rel_x = target_common["x"] - me["x"]
        rel_y = target_common["y"] - me["y"]
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
                if (
                    self._experiment_duration > 0
                    and (time.time() - START_TIME) >= self._experiment_duration
                ):
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
                        FOLLOWER_HZ_HISTORY = FOLLOWER_HZ_HISTORY[-FOLLOWER_HZ_WINDOW:]
                    RATES_SHARED["follower_hz"] = sum(FOLLOWER_HZ_HISTORY) / len(
                        FOLLOWER_HZ_HISTORY
                    )
                leader_pos = other[self._predecessor_id]
                self.pid_step_toward(leader_pos, dt=dt_loop)
                if dt_loop is not None:
                    time.sleep(dt_loop)
        except KeyboardInterrupt:
            pass


def leader_roll_positive_y_loop(
    controller: DroneController,
    experiment_duration: float = 0,
    roll_pwm: int = 1600,
    step_sec: float = 0.1,
) -> None:
    """Leader: continuous roll bias, pitch neutral — POS_HOLD trajectory toward +Y (tune PWM in SITL).

    Args:
        controller: Leader drone controller.
        experiment_duration: If > 0, stop when global experiment time exceeds this (s).
        roll_pwm: RC channel 1 (roll); default 1600 vs neutral 1500 — adjust sign/magnitude for your stack.
        step_sec: Sleep between override commands.
    """
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


FORMATION_D_STAR_M = 2.0


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
    """Parse args, create controllers, run roll leader + followers and coord exchange."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Snake pursuit: roll leader +Y + chain followers (snake_pursuit)"
    )
    parser.add_argument("--kp", type=float, default=8.0, help="P gain for follower PID")
    parser.add_argument(
        "--ki",
        type=float,
        default=0.0,
        help="I gain for roll (lateral); pitch too if --ki-pitch omitted",
    )
    parser.add_argument(
        "--ki-pitch",
        type=float,
        default=None,
        help=(
            "I gain for pitch (longitudinal X) only; roll keeps --ki. "
            "Omit to use --ki for both axes."
        ),
    )
    parser.add_argument("--kd", type=float, default=10.0, help="D gain")
    parser.add_argument(
        "--derivative-alpha",
        type=float,
        default=0.0,
        metavar="A",
        help=(
            "Low-pass factor for D term on both follower PIDs (0..1); "
            "0 disables derivative filtering (default 0)."
        ),
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=50.0,
        help="Control loop rate in Hz; 0 = no limit",
    )
    parser.add_argument(
        "--leader-roll-pwm",
        type=int,
        default=1600,
        help=(
            "Leader roll RC (PWM); not neutral to pull +Y in NED under POS_HOLD — tune per SITL"
        ),
    )
    parser.add_argument(
        "--leader-roll-step-sec",
        type=float,
        default=0.1,
        help="Sleep between leader RC_OVERRIDE updates (s)",
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
            "Max CSV log write rate (Hz); 0 = one row per coordinate-exchange step "
            "(same cadence as --exchange-hz). Above 0 caps how often rows are written."
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
    parser.add_argument(
        "--exchange-position-noise-sigma-m",
        type=float,
        nargs=3,
        default=None,
        metavar=("SX", "SY", "SZ"),
        help=(
            "Gaussian std dev (m) on exchanged neighbor positions (x,y,z); "
            "omit for no position noise"
        ),
    )
    parser.add_argument(
        "--exchange-packet-loss",
        type=float,
        default=0.0,
        help="Probability in [0,1] to drop each neighbor position update (interference)",
    )
    parser.add_argument(
        "--exchange-noise-seed",
        type=int,
        default=None,
        help="RNG seed for exchange noise and packet loss (repeatable runs)",
    )
    parser.add_argument(
        "--sitl-direct-tcp",
        action="store_true",
        help=(
            "MAVLink over ArduPilot SITL TCP (5760+10*i); use with launcher "
            "sim_vehicle --no-mavproxy (no MAVProxy UDP)"
        ),
    )
    args = parser.parse_args()
    args.control_loop_period_sec = (
        1.0 / args.control_hz if args.control_hz and args.control_hz > 0 else None
    )
    args.log_run_id = args.run_id

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
        f = open(path, "w", encoding="utf-8")
        f.write(CSV_HEADER + "\n")
        f.flush()
        experiment_log_files[did] = f
    write_metadata(
        experiment_dir,
        args.duration,
        args.collision_radius,
        args.drones,
        "snake_pursuit",
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
        init_barrier.wait(timeout=120)
    except threading.BrokenBarrierError:
        return
    time.sleep(2)
    global START_TIME
    START_TIME = time.time()
    noise_opts: Dict[str, Any] = {}
    if args.exchange_position_noise_sigma_m is not None:
        noise_opts["position_sigma_m"] = tuple(args.exchange_position_noise_sigma_m)
    if args.exchange_packet_loss and args.exchange_packet_loss > 0:
        noise_opts["packet_loss_probability"] = float(args.exchange_packet_loss)
    if args.exchange_noise_seed is not None:
        noise_opts["seed"] = int(args.exchange_noise_seed)
    noise_dict: Optional[Dict[str, Any]] = noise_opts if noise_opts else None
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
        rates_shared=RATES_SHARED,
        noise_dict=noise_dict,
        daemon=True,
    )
    time.sleep(1)

    try:
        leader = next((c for c in controllers if c.config["role"] == "leader"), None)
        followers = [c for c in controllers if c.config["role"] == "follower"]
        if not leader:
            return
        threading.Thread(
            target=leader_roll_positive_y_loop,
            args=(leader,),
            kwargs={
                "experiment_duration": args.duration,
                "roll_pwm": args.leader_roll_pwm,
                "step_sec": args.leader_roll_step_sec,
            },
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
                derivative_alpha=float(args.derivative_alpha),
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
