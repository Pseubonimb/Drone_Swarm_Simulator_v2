"""
Decentralized task assignment: swarm coord exchange, cost matrix from NED poses vs
fixed target waypoints (global column indices), Chopra et al. distributed Hungarian
(or optional centralized LSAP for debugging), PID flight to assigned targets.

Physical «mission converged» (default): when the post-assignment flight window ends,
each assigned drone must lie within the horizontal disc of radius
``--target-reach-radius-m`` in common NED (X/Y) around its waypoint (Z ignored).
A single fly-through does not count — overshoot at stop time fails the check. Common-frame
Y uses ``FORMATION_D_STAR_M`` east spacing (same for PID, exchange, metrics — not an extra
per-drone reach radius). Optional ``--target-reach-epsilon-m`` adds to R for the final
check only.
``assignment_metrics.json`` field ``converged`` is updated accordingly unless
``--allow-converged-without-reaching-targets``.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import sys
import threading
import time
from datetime import datetime
from typing import Any, Dict, List, Optional

import numpy as np

_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from core.assignment import (  # noqa: E402
    ConnectivityGraph,
    MetricsCollector,
    solve_lsap_reference,
    run_decentralized_hungarian_chopra2018,
)
from core.control import DroneController, PIDRegulator  # noqa: E402
from core.logging.csv_logger import CSV_HEADER, write_metadata  # noqa: E402
from core.mavlink.utils import RC_NEUTRAL, sitl_tcp_connection_string  # noqa: E402
from core.mavlink.worker import recommended_init_barrier_timeout_sec  # noqa: E402
from core.network import (  # noqa: E402
    CoordExchangeStepContext,
    default_local_to_common_ned,
)
from visualizer.ned_xy_snapshot import save_ned_xy_png  # noqa: E402

logger = logging.getLogger(__name__)

RATES_SHARED: Dict[str, Optional[float]] = {
    "follower_hz": None,
    "exchange_hz": None,
    "webots_step_hz": None,
}

START_TIME = 0.0

INIT_STEPS = [
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

FORMATION_D_STAR_M = 2.0


def _close_drone_csv_handles(files: Dict[int, Any]) -> None:
    """Close drone CSV streams (e.g. init failure before CoordExchange owns them)."""
    for fh in list(files.values()):
        try:
            fh.close()
        except Exception:
            pass


def targets_circle_common_ned(
    num_points: int,
    radius_m: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> List[Dict[str, float]]:
    """Target waypoints on a circle in common NED; column ``k`` has global index ``k``.

    Points are equally spaced in angle: ``center + radius * (cos θ, sin θ)`` with
    ``θ = 2π k / num_points``.
    """
    if num_points <= 0:
        return []
    out: List[Dict[str, float]] = []
    for k in range(num_points):
        ang = 2.0 * math.pi * k / num_points
        out.append(
            {
                "x": center_x + radius_m * math.cos(ang),
                "y": center_y + radius_m * math.sin(ang),
                "z": 0.0,
            }
        )
    return out


def distance_cost(pos: Dict[str, float], tgt: Dict[str, float]) -> float:
    """Euclidean distance in NED (m)."""
    return float(
        math.sqrt(
            (pos["x"] - tgt["x"]) ** 2
            + (pos["y"] - tgt["y"]) ** 2
            + (pos["z"] - tgt["z"]) ** 2
        )
    )


def build_cost_matrix(
    positions_common: Dict[int, Dict[str, float]],
    targets: List[Dict[str, float]],
) -> np.ndarray:
    """Cost matrix ``C[i,j]`` = distance(agent i, target j); rows = sorted drone ids."""
    ids = sorted(positions_common.keys())
    n_agents = len(ids)
    k = len(targets)
    c = np.zeros((n_agents, k), dtype=float)
    for ii, did in enumerate(ids):
        for j in range(k):
            c[ii, j] = distance_cost(positions_common[did], targets[j])
    return c


def assignment_graph(n: int, kind: str) -> ConnectivityGraph:
    """Undirected graph on agents ``0..n-1``."""
    kind_l = kind.strip().lower()
    edges: List[tuple[int, int]] = []
    if n >= 2:
        if kind_l == "complete":
            for i in range(n):
                for j in range(i + 1, n):
                    edges.append((i, j))
        else:
            # ring (default): path 0-1-...-(n-1) plus (0, n-1) when n > 2
            for i in range(n - 1):
                edges.append((i, i + 1))
            if n > 2:
                edges.append((0, n - 1))
    return ConnectivityGraph(num_nodes=n, edges=tuple(edges))


class TowardTargetPID:
    """Roll/pitch PID toward a fixed NED point in common frame (no formation offset)."""

    @staticmethod
    def _pid_output_to_rc_delta(raw: float, output_limit: float = 200.0) -> int:
        """Map PID output to integer PWM delta.

        Using plain ``int()`` on the output zeros small corrections (e.g. 0 < |out| < 1), so
        the craft never closes the last decimetres toward a tight reach radius. Prefer
        rounding, then a single-PWM nudge when the continuous output is still non-zero.
        """
        lim = int(output_limit)
        if abs(raw) < 1e-12:
            return 0
        step = int(round(raw))
        if step == 0 and abs(raw) > 0.01:
            step = 1 if raw > 0 else -1
        return max(-lim, min(lim, step))


    def __init__(
        self,
        drone: DroneController,
        kp: float = 8.0,
        ki: float = 0.0,
        kd: float = 10.0,
        derivative_alpha: float = 0.0,
        control_loop_period_sec: Optional[float] = None,
        experiment_duration: float = 0.0,
    ) -> None:
        self._drone = drone
        self._control_loop_period_sec = control_loop_period_sec
        self._experiment_duration = experiment_duration
        lim = 200.0
        self._roll_pid = PIDRegulator(
            kp=kp,
            ki=ki,
            kd=kd,
            integral_limit=100.0,
            output_limit=lim,
            derivative_alpha=derivative_alpha,
        )
        self._pitch_pid = PIDRegulator(
            kp=kp,
            ki=ki,
            kd=kd,
            integral_limit=100.0,
            output_limit=lim,
            derivative_alpha=derivative_alpha,
        )

    @staticmethod
    def _my_position_common(drone: DroneController) -> Dict[str, float]:
        raw = drone.get_my_position()
        did = int(drone.config["id"])
        return default_local_to_common_ned(
            did, raw, east_spacing_m=FORMATION_D_STAR_M
        )

    def pid_step_toward(
        self,
        target_common: Dict[str, float],
        dt: Optional[float],
    ) -> None:
        w = self._drone.worker
        if w is None:
            return
        me = self._my_position_common(self._drone)
        error_x = target_common["x"] - me["x"]
        error_y = target_common["y"] - me["y"]
        pitch_output = self._pitch_pid.update(error_x, dt=dt)
        roll_output = self._roll_pid.update(error_y, dt=dt)
        d_pitch = self._pid_output_to_rc_delta(pitch_output, self._pitch_pid.output_limit)
        d_roll = self._pid_output_to_rc_delta(roll_output, self._roll_pid.output_limit)
        pitch_pwm = RC_NEUTRAL - d_pitch
        roll_pwm = RC_NEUTRAL + d_roll
        w.send_rc_override(
            roll_pwm, pitch_pwm, RC_NEUTRAL, RC_NEUTRAL, controller=self._drone
        )

    def run_until(
        self,
        target_common: Dict[str, float],
        global_start: float,
        wall_deadline: Optional[float] = None,
    ) -> None:
        """Run PID until experiment duration cap and/or absolute wall-clock deadline.

        Does not stop early when near the waypoint — final arrival is verified after the
        flight window ends so overshoot cannot count as success.
        """
        last_iter_time: Optional[float] = None
        dt_loop = (
            self._control_loop_period_sec
            if self._control_loop_period_sec is not None
            and self._control_loop_period_sec > 0
            else None
        )
        try:
            while True:
                now_wall = time.time()
                if wall_deadline is not None and now_wall >= wall_deadline:
                    return
                if (
                    self._experiment_duration > 0
                    and (now_wall - global_start) >= self._experiment_duration
                ):
                    return
                now = now_wall
                loop_dt = (now - last_iter_time) if last_iter_time is not None else 0.0
                last_iter_time = now
                dt_use = dt_loop if dt_loop is not None else (loop_dt if loop_dt > 0 else None)
                self.pid_step_toward(target_common, dt=dt_use)
                if dt_loop is not None:
                    time.sleep(dt_loop)
        except KeyboardInterrupt:
            return


def initialize_drone_parallel(
    controller: DroneController, init_barrier: threading.Barrier
) -> None:
    try:
        controller.connect()
        controller.initialize(INIT_STEPS)
        controller.start_rc_keepalive()
        init_barrier.wait()
    except Exception:
        logger.exception("Drone init failed")
        raise


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Decentralized task assignment (task_assignment_decentralized)"
    )
    parser.add_argument("--kp", type=float, default=8.0, help="PID P gain (NED x / pitch)")
    parser.add_argument("--ki", type=float, default=0.0, help="PID I gain")
    parser.add_argument("--kd", type=float, default=10.0, help="PID D gain")
    parser.add_argument(
        "--derivative-alpha",
        type=float,
        default=0.0,
        metavar="A",
        help="Low-pass on D term (0..1); 0 = off",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=50.0,
        help="Control loop rate (Hz); 0 = no limit",
    )
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--drones", type=int, default=3)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--experiment-dir", type=str, default=None)
    parser.add_argument("--collision-radius", type=float, default=0.2)
    parser.add_argument(
        "--log-hz",
        type=float,
        default=0.0,
        help="CSV rate cap (Hz); 0 = same as exchange",
    )
    parser.add_argument("--exchange-hz", type=float, default=50.0)
    parser.add_argument(
        "--target-radius-m",
        type=float,
        default=12.0,
        help="Radius (m) of target circle in common NED",
    )
    parser.add_argument(
        "--num-targets",
        type=int,
        default=None,
        metavar="K",
        help=(
            "Number of target waypoints (columns). Default: equals --drones (square). "
            "If K ≠ N, SciPy LSAP is used (rectangular); Chopra decentralized requires K=N."
        ),
    )
    parser.add_argument(
        "--target-center-x",
        type=float,
        default=0.0,
        help="Circle center X (m, North, common NED)",
    )
    parser.add_argument(
        "--target-center-y",
        type=float,
        default=0.0,
        help="Circle center Y (m, East, common NED)",
    )
    parser.add_argument(
        "--assignment-settle-sec",
        type=float,
        default=1.0,
        help="Wait this long after start before solving assignment",
    )
    parser.add_argument(
        "--graph",
        type=str,
        choices=("ring", "complete"),
        default="ring",
        help="Communication graph for decentralized Hungarian",
    )
    parser.add_argument(
        "--centralized-assignment",
        action="store_true",
        help=(
            "Debug: skip distributed rounds; use centralized LSAP (SciPy) only. "
            "Produces assignment_metrics.json with solver=centralized."
        ),
    )
    parser.add_argument(
        "--sitl-direct-tcp",
        action="store_true",
        help="MAVLink TCP 5760+10*i (use with launcher --without-mavproxy-consoles)",
    )
    parser.add_argument(
        "--no-exit-on-converged",
        action="store_true",
        help=(
            "Keep running until --duration even if the solver reports converged=True "
            "(default: exit shortly after a converged decentralized run)."
        ),
    )
    parser.add_argument(
        "--post-assign-fly-sec",
        type=float,
        default=5.0,
        metavar="SEC",
        help=(
            "When exiting on converge (and --allow-converged-without-reaching-targets): "
            "fly at most this many seconds after assignment solve; "
            "coord-exchange logging stops at the same wall time. "
            "If physical arrival is required (default), the fly window is the rest of "
            "--duration instead."
        ),
    )
    parser.add_argument(
        "--target-reach-radius-m",
        type=float,
        default=0.5,
        metavar="R",
        help=(
            "Drone «at target» in common NED XY if horizontal distance to assigned waypoint "
            "is <= R (m). Z ignored. Used when not --allow-converged-without-reaching-targets."
        ),
    )
    parser.add_argument(
        "--target-reach-epsilon-m",
        type=float,
        default=0.0,
        metavar="E",
        help=(
            "Optional margin (m) added to --target-reach-radius-m for the final horizontal "
            "check only — same R for every drone; useful if SITL leaves one agent barely "
            "outside a tight radius."
        ),
    )
    parser.add_argument(
        "--allow-converged-without-reaching-targets",
        action="store_true",
        help=(
            "Legacy: treat solver matching as sufficient; use short --post-assign-fly-sec window; "
            "may save viz_converged.png even if drones did not enter the reach circle. "
            "Default is to require all assigned drones inside R+ε."
        ),
    )
    args = parser.parse_args()
    args.control_loop_period_sec = (
        1.0 / args.control_hz if args.control_hz and args.control_hz > 0 else None
    )

    n = args.drones
    if n < 1:
        logger.error("--drones must be >= 1")
        return

    k_targets = int(args.num_targets) if args.num_targets is not None else n
    if k_targets < 1:
        logger.error("--num-targets must be >= 1 when set")
        return
    drones_config: List[Dict[str, Any]] = []
    for i in range(n):
        cfg: Dict[str, Any] = {"id": i + 1, "role": "agent"}
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

    if k_targets != n:
        logger.info(
            "Rectangular instance: %s drones, %s targets — using SciPy LSAP "
            "(Chopra decentralized only when N==K and not --centralized-assignment).",
            n,
            k_targets,
        )

    targets = targets_circle_common_ned(
        k_targets,
        float(args.target_radius_m),
        float(args.target_center_x),
        float(args.target_center_y),
    )
    graph = assignment_graph(n, args.graph)

    require_physical = not bool(args.allow_converged_without_reaching_targets)
    meta_extra_init: Dict[str, Any] = {
        "assignment_graph": args.graph,
        "num_targets": k_targets,
        "target_radius_m": float(args.target_radius_m),
        "target_center_x": float(args.target_center_x),
        "target_center_y": float(args.target_center_y),
        "target_reach_radius_m": float(args.target_reach_radius_m),
        "target_reach_epsilon_m": float(args.target_reach_epsilon_m),
        "require_targets_for_converged_report": require_physical,
        "centralized_assignment": bool(args.centralized_assignment),
        "exit_on_converged": (not args.no_exit_on_converged),
        "post_assign_fly_sec": float(args.post_assign_fly_sec),
        "targets_common_ned": targets,
        "graph_edges": [list(e) for e in graph.edges],
        "graph_num_nodes": graph.num_nodes,
        "decentralized_chopra_eligible": (
            k_targets == n and not bool(args.centralized_assignment)
        ),
    }

    experiment_log_files: Dict[int, Any] = {}
    for cfg in drones_config:
        did = int(cfg["id"])
        path = os.path.join(experiment_dir, f"drone_{did}.csv")
        fh = open(path, "w", encoding="utf-8")
        fh.write(CSV_HEADER + "\n")
        fh.flush()
        experiment_log_files[did] = fh

    write_metadata(
        experiment_dir,
        args.duration,
        args.collision_radius,
        n,
        "task_assignment_decentralized",
        extra=dict(meta_extra_init),
    )

    controllers: List[DroneController] = []
    for cfg in drones_config:
        controllers.append(DroneController(cfg, logging_enabled=False))

    init_barrier = threading.Barrier(len(controllers) + 1)
    for c in controllers:
        threading.Thread(
            target=initialize_drone_parallel,
            args=(c, init_barrier),
            daemon=False,
        ).start()
    try:
        init_barrier.wait(timeout=recommended_init_barrier_timeout_sec())
    except threading.BrokenBarrierError:
        logger.error(
            "Init barrier broken (timeout or worker failure); closing logs and stopping drones."
        )
        _close_drone_csv_handles(experiment_log_files)
        for c in controllers:
            try:
                c.stop()
            except Exception:
                pass
        return

    global START_TIME
    START_TIME = time.time()

    assign_lock = threading.Lock()
    assign_done = threading.Event()
    target_by_drone: Dict[int, int] = {}
    assignment_metrics_blob: Dict[str, Any] = {}
    coord_mgr_holder: Dict[str, Any] = {}
    assignment_timing: Dict[str, Any] = {"fly_wall_deadline": None, "solve_wall_time": None}
    early_coord_timer_holder: List[Optional[threading.Timer]] = [None]
    metadata_snapshot: Dict[str, Any] = {"extra": dict(meta_extra_init)}
    viz_extra: Dict[str, Any] = {"targets": targets}
    viz_lock = threading.Lock()
    snapshot_flags = {"initial_done": False}
    initial_snapshot_elapsed_s = min(
        2.0,
        max(0.12, float(args.assignment_settle_sec) * 0.45 + 0.08),
    )
    converged_viz_holder: Dict[str, Any] = {"save_after_flight": False}

    def solve_and_record(positions_common: Dict[int, Dict[str, float]]) -> None:
        nonlocal assignment_metrics_blob
        cmat = build_cost_matrix(positions_common, targets)
        n_rows, n_cols = cmat.shape
        square = n_rows == n_cols
        ids_sorted = sorted(positions_common.keys())
        use_chopra = (not args.centralized_assignment) and square

        if use_chopra:
            mc = MetricsCollector()
            res = run_decentralized_hungarian_chopra2018(
                cmat, graph, collect_metrics=mc
            )
            assignment_metrics_blob = dict(res.metrics)
            assignment_metrics_blob["solver"] = "decentralized_chopra2018"
            assignment_metrics_blob["assignment_columns"] = [
                int(res.assignment[i]) for i in range(n_rows)
            ]
            assignment_metrics_blob["row_to_drone_id"] = [
                int(ids_sorted[i]) for i in range(n_rows)
            ]
            assignment_metrics_blob["cost_matrix"] = cmat.tolist()
            assign = res.assignment
        else:
            ref = solve_lsap_reference(cmat)
            assign = np.full(n_rows, -1, dtype=np.int64)
            assign[ref.row_ind] = ref.col_ind
            total = float(ref.optimal_cost)
            if not square:
                solver_label = "centralized_lsap_rectangular"
            else:
                solver_label = "centralized_lsap"
            assignment_metrics_blob = {
                "solver": solver_label,
                "converged": True,
                "reference_optimal_cost": total,
                "decentralized_cost": total,
                "scipy_delta_abs": 0.0,
                "assignment_columns": [int(assign[i]) for i in range(n_rows)],
                "row_to_drone_id": [int(ids_sorted[i]) for i in range(n_rows)],
                "cost_matrix": cmat.tolist(),
                "rectangular": not square,
                "graph": {
                    "num_nodes": graph.num_nodes,
                    "edges": [list(e) for e in graph.edges],
                },
            }
        target_by_drone.clear()
        for i in range(n_rows):
            col = int(assign[i])
            if col >= 0:
                target_by_drone[ids_sorted[i]] = col

        converged_now = bool(assignment_metrics_blob.get("converged"))
        time_to_solver_converged_sec: Optional[float] = (
            round(time.time() - START_TIME, 6) if converged_now else None
        )
        assignment_metrics_blob["solver_converged"] = converged_now
        assignment_metrics_blob["time_to_solver_converged_sec"] = time_to_solver_converged_sec
        # Legacy keys: same as solver until final merge may redefine «converged» for mission sense.
        assignment_metrics_blob["time_to_converged_sec"] = time_to_solver_converged_sec

        path_m = os.path.join(experiment_dir, "assignment_metrics.json")
        with open(path_m, "w", encoding="utf-8") as f:
            json.dump(assignment_metrics_blob, f, indent=2)
        logger.info("Wrote %s", path_m)
        if converged_now and time_to_solver_converged_sec is not None:
            logger.info(
                "[task_assignment] solver converged after %.3f s (wall since experiment start)",
                time_to_solver_converged_sec,
            )

        meta_extra_final = dict(meta_extra_init)
        meta_extra_final["assignment"] = {
            "drone_id_to_target_column": {str(k): v for k, v in target_by_drone.items()},
            "solver_converged": converged_now,
            "time_to_solver_converged_sec": time_to_solver_converged_sec,
            "rounds_executed": assignment_metrics_blob.get("rounds_executed"),
            "decentralized_cost": assignment_metrics_blob.get("decentralized_cost"),
            "reference_optimal_cost": assignment_metrics_blob.get("reference_optimal_cost"),
            "scipy_delta_abs": assignment_metrics_blob.get("scipy_delta_abs"),
            "solver": assignment_metrics_blob.get("solver"),
        }
        if converged_now:
            converged_viz_holder["save_after_flight"] = True
        exit_on_cv = not args.no_exit_on_converged
        solve_wall = time.time()
        assignment_timing["solve_wall_time"] = solve_wall
        require_targets = not bool(args.allow_converged_without_reaching_targets)
        fly_deadline_opt: Optional[float] = None
        if exit_on_cv and converged_now:
            if require_targets:
                if float(args.duration) > 0:
                    fly_deadline_opt = START_TIME + float(args.duration)
                else:
                    fly_deadline_opt = None
                logger.info(
                    "[task_assignment] physical targets required → PID/cutoff at experiment "
                    "end (START+%.1fs).",
                    float(args.duration),
                )
            else:
                fly_deadline_opt = solve_wall + max(0.0, float(args.post_assign_fly_sec))
                logger.info(
                    "[task_assignment] solver converged → wall fly deadline %.3f (+ %.2fs after solve)",
                    fly_deadline_opt,
                    float(args.post_assign_fly_sec),
                )
        assignment_timing["fly_wall_deadline"] = fly_deadline_opt
        meta_extra_final["experiment_exit"] = {
            "exit_on_converged_requested": exit_on_cv,
            "solver_reported_converged": converged_now,
            "scheduled_fly_wall_deadline_unix": fly_deadline_opt,
            "post_assign_fly_sec": float(args.post_assign_fly_sec),
            "require_physical_target_arrival": require_targets,
        }
        write_metadata(
            experiment_dir,
            args.duration,
            args.collision_radius,
            n,
            "task_assignment_decentralized",
            extra=meta_extra_final,
        )
        metadata_snapshot["extra"] = dict(meta_extra_final)

        mgr_o = coord_mgr_holder.get("mgr")
        pt = early_coord_timer_holder[0]
        if pt is not None:
            try:
                pt.cancel()
            except Exception:
                pass
            early_coord_timer_holder[0] = None

        def _coord_stop_early() -> None:
            mgr_in = coord_mgr_holder.get("mgr")
            if mgr_in is None:
                return
            logger.info(
                "[task_assignment] stopping swarm coordination (post-assign window elapsed)."
            )
            try:
                mgr_in.stop(join_timeout_sec=5.0)
            except Exception:
                logger.exception("coord_mgr.stop() during early converge failed")

        if fly_deadline_opt is not None and mgr_o is not None:
            delay = max(0.0, fly_deadline_opt - time.time())
            tmr = threading.Timer(delay, _coord_stop_early)
            tmr.daemon = True
            early_coord_timer_holder[0] = tmr
            tmr.start()

        assign_done.set()

    def on_step(ctx: CoordExchangeStepContext) -> None:
        ids_early = sorted(ctx.positions.keys())
        if (
            not snapshot_flags["initial_done"]
            and len(ids_early) == n
            and not assign_done.is_set()
            and ctx.time_elapsed >= initial_snapshot_elapsed_s
        ):
            outp_i = os.path.join(experiment_dir, "viz_initial.png")
            try:
                save_ned_xy_png(
                    outp_i,
                    dict(ctx.positions),
                    targets=targets,
                    title="Начало эксперимента (до назначения целей)",
                    inset_drone_zoom=True,
                )
                logger.info("Wrote initial NED snapshot %s", outp_i)
                if os.environ.get("DRONE_SWARM_WITH_2D_VIZ") == "1":
                    with viz_lock:
                        viz_extra["pending_png"] = outp_i
            except Exception:
                logger.exception("Initial viz snapshot failed")
            snapshot_flags["initial_done"] = True
        if assign_done.is_set():
            return
        if ctx.time_elapsed < float(args.assignment_settle_sec):
            return
        ids = sorted(ctx.positions.keys())
        if len(ids) != n:
            return
        with assign_lock:
            if assign_done.is_set():
                return
            try:
                solve_and_record(dict(ctx.positions))
            except Exception:
                logger.exception("Assignment solve failed; releasing flight threads")
                assign_done.set()

    coord_mgr = None
    try:
        coord_mgr = DroneController.start_swarm_coord_exchange(
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
            visualizer_extra=viz_extra,
            visualizer_extra_lock=viz_lock,
            on_step=on_step,
            daemon=True,
        )
        coord_mgr_holder["mgr"] = coord_mgr
        time.sleep(0.2)

        def run_drone(c: DroneController) -> None:
            if not assign_done.wait(timeout=600.0):
                logger.error("Assignment did not complete (timeout)")
                return
            did = int(c.config["id"])
            col = target_by_drone.get(did)
            if col is None:
                logger.info(
                    "Drone %s: unassigned in optimal LSAP — holding (no target PID).", did
                )
                wall_deadline_cf = assignment_timing.get("fly_wall_deadline")
                try:
                    while True:
                        now_wall = time.time()
                        if wall_deadline_cf is not None and now_wall >= wall_deadline_cf:
                            break
                        if (
                            float(args.duration) > 0
                            and (now_wall - START_TIME) >= float(args.duration)
                        ):
                            break
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    pass
                return
            tgt = targets[col]
            wall_deadline_cf: Optional[float] = assignment_timing.get("fly_wall_deadline")
            TowardTargetPID(
                c,
                kp=args.kp,
                ki=args.ki,
                kd=args.kd,
                derivative_alpha=float(args.derivative_alpha),
                control_loop_period_sec=args.control_loop_period_sec,
                experiment_duration=float(args.duration),
            ).run_until(tgt, START_TIME, wall_deadline=wall_deadline_cf)

        threads = []
        for c in controllers:
            t = threading.Thread(target=run_drone, args=(c,), daemon=False)
            t.start()
            threads.append(t)

        try:
            for t in threads:
                t.join()
            require_physical = not bool(args.allow_converged_without_reaching_targets)
            solver_cv = bool(assignment_metrics_blob.get("converged"))
            assigned_ids = sorted(target_by_drone.keys())
            rr_m = float(args.target_reach_radius_m)
            eps_m = max(0.0, float(args.target_reach_epsilon_m))
            limit_m = rr_m + eps_m
            ctr_by_id = {int(c.config["id"]): c for c in controllers}
            per_drone_final_horiz_dist_m: Dict[str, float] = {}
            all_at_targets = True
            if require_physical and assigned_ids:
                for did_i in assigned_ids:
                    c_i = ctr_by_id.get(did_i)
                    if c_i is None:
                        all_at_targets = False
                        continue
                    col_i = target_by_drone[did_i]
                    tg = targets[col_i]
                    pcom = default_local_to_common_ned(
                        did_i,
                        c_i.get_my_position(),
                        east_spacing_m=FORMATION_D_STAR_M,
                    )
                    d_h = math.hypot(
                        float(tg["x"]) - float(pcom["x"]),
                        float(tg["y"]) - float(pcom["y"]),
                    )
                    per_drone_final_horiz_dist_m[str(did_i)] = round(d_h, 4)
                    if d_h > limit_m:
                        all_at_targets = False
                        logger.warning(
                            "Drone %s: final horiz dist to assigned target %.4f m > limit "
                            "%.4f (R=%.4f + ε=%.4f), east_spacing_m=%.3f",
                            did_i,
                            d_h,
                            limit_m,
                            rr_m,
                            eps_m,
                            FORMATION_D_STAR_M,
                        )
            elif require_physical and not assigned_ids:
                all_at_targets = False

            if require_physical and assigned_ids:
                logger.info(
                    "[task_assignment] reach frame: common NED XY; "
                    "east_spacing_m=%.3f — used for PID, coord exchange, and final check "
                    "(not the reach circle radius).",
                    FORMATION_D_STAR_M,
                )
                logger.info(
                    "[task_assignment] mission reach: base_R=%.6f m epsilon=%.6f m → limit=%.6f m "
                    "(identical for every assigned drone)",
                    rr_m,
                    eps_m,
                    limit_m,
                )
                for sid in sorted(per_drone_final_horiz_dist_m.keys(), key=int):
                    logger.info(
                        "[task_assignment] drone %s: horiz dist to assigned target = %.6f m",
                        sid,
                        per_drone_final_horiz_dist_m[sid],
                    )

            verify_wall = time.time()
            time_to_all_targets: Optional[float] = None
            time_solver_to_all_targets: Optional[float] = None
            per_drone_from_start: Dict[str, float] = {}
            if require_physical and all_at_targets and assigned_ids:
                time_to_all_targets = round(verify_wall - START_TIME, 6)
                per_drone_from_start = {
                    str(did_i): time_to_all_targets for did_i in assigned_ids
                }
                sw = assignment_timing.get("solve_wall_time")
                if isinstance(sw, (int, float)):
                    time_solver_to_all_targets = round(verify_wall - float(sw), 6)

            mission_converged = bool(
                solver_cv
                and (all_at_targets if require_physical else True)
                and (bool(assigned_ids) or not require_physical)
            )

            path_metrics = os.path.join(experiment_dir, "assignment_metrics.json")
            try:
                with open(path_metrics, encoding="utf-8") as fm:
                    metrics_final = json.load(fm)
            except Exception:
                metrics_final = dict(assignment_metrics_blob)
                logger.exception("Reading assignment_metrics.json for final patch failed")
            metrics_final["solver_converged"] = bool(
                metrics_final.get("solver_converged", metrics_final.get("converged"))
            )
            metrics_final["all_targets_reached"] = all_at_targets
            metrics_final["target_reach_radius_m"] = float(args.target_reach_radius_m)
            metrics_final["target_reach_epsilon_m"] = float(args.target_reach_epsilon_m)
            metrics_final["effective_reach_limit_m"] = (
                limit_m if require_physical else None
            )
            metrics_final["east_spacing_common_m"] = FORMATION_D_STAR_M
            metrics_final["require_physical_target_arrival"] = require_physical
            metrics_final["converged"] = mission_converged
            metrics_final["per_drone_final_horiz_dist_m"] = per_drone_final_horiz_dist_m
            metrics_final["time_to_all_targets_sec"] = time_to_all_targets
            metrics_final["time_solver_to_all_targets_sec"] = time_solver_to_all_targets
            metrics_final["per_drone_time_to_target_from_start_sec"] = per_drone_from_start
            if time_to_all_targets is not None:
                metrics_final["time_to_converged_sec"] = time_to_all_targets
            with open(path_metrics, "w", encoding="utf-8") as fm:
                json.dump(metrics_final, fm, indent=2)
            logger.info(
                "Patched %s: mission_converged=%s all_targets=%s time_to_all_targets_s=%s",
                path_metrics,
                mission_converged,
                all_at_targets,
                time_to_all_targets,
            )

            meta_patch = dict(metadata_snapshot["extra"])
            a_prev = dict(meta_patch.get("assignment", {}))
            a_prev.update(
                {
                    "mission_converged": mission_converged,
                    "solver_converged": solver_cv,
                    "all_targets_reached": all_at_targets,
                    "time_to_all_targets_sec": time_to_all_targets,
                    "time_solver_to_all_targets_sec": time_solver_to_all_targets,
                    "target_reach_radius_m": float(args.target_reach_radius_m),
                    "target_reach_epsilon_m": float(args.target_reach_epsilon_m),
                    "effective_reach_limit_m": limit_m if require_physical else None,
                    "per_drone_final_horiz_dist_m": per_drone_final_horiz_dist_m,
                }
            )
            meta_patch["assignment"] = a_prev
            write_metadata(
                experiment_dir,
                args.duration,
                args.collision_radius,
                n,
                "task_assignment_decentralized",
                extra=meta_patch,
            )
            metadata_snapshot["extra"] = meta_patch

            save_viz = bool(converged_viz_holder.get("save_after_flight")) and mission_converged
            if converged_viz_holder.get("save_after_flight") and not mission_converged:
                logger.warning(
                    "Skipping viz_converged.png: mission not complete "
                    "(solver=%s all_at_targets=%s require_physical=%s).",
                    solver_cv,
                    all_at_targets,
                    require_physical,
                )
            if save_viz:
                outp_cf = os.path.join(experiment_dir, "viz_converged.png")
                try:
                    positions_flight_end: Dict[int, Dict[str, float]] = {}
                    for ctr in controllers:
                        did_i = int(ctr.config["id"])
                        raw_p = ctr.get_my_position()
                        positions_flight_end[did_i] = default_local_to_common_ned(
                            did_i, raw_p, east_spacing_m=FORMATION_D_STAR_M
                        )
                    rlim = float(args.target_reach_radius_m)
                    epsl = float(args.target_reach_epsilon_m)
                    save_ned_xy_png(
                        outp_cf,
                        positions_flight_end,
                        targets=targets,
                        title=(
                            f"Все дроны внутри окрестности (лимит {rlim + max(0.0, epsl):.2f} m "
                            f"= R {rlim:.2f} + ε {epsl:.2f}, NED XY common)"
                        ),
                        inset_drone_zoom=True,
                    )
                    logger.info("Wrote post-mission NED snapshot %s", outp_cf)
                    if os.environ.get("DRONE_SWARM_WITH_2D_VIZ") == "1":
                        with viz_lock:
                            viz_extra["pending_png"] = outp_cf
                except Exception:
                    logger.exception("Post-mission viz snapshot failed")
        except KeyboardInterrupt:
            pass
    finally:
        et = early_coord_timer_holder[0]
        if et is not None:
            try:
                et.cancel()
            except Exception:
                pass
            early_coord_timer_holder[0] = None
        if coord_mgr is not None:
            coord_mgr.stop(join_timeout_sec=5.0)
        for c in controllers:
            try:
                c.stop()
            except Exception:
                pass
        if START_TIME > 0:
            try:
                extra_final = dict(metadata_snapshot["extra"])
                extra_final.setdefault("experiment_exit", {}).update(
                    {"wall_duration_sec_actual": round(time.time() - START_TIME, 6)}
                )
                write_metadata(
                    experiment_dir,
                    args.duration,
                    args.collision_radius,
                    n,
                    "task_assignment_decentralized",
                    extra=extra_final,
                )
            except Exception:
                logger.exception("Final metadata rewrite failed")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    main()
