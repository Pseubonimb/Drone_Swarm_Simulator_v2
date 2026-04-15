"""
Swarm coordinate exchange: common NED frame, neighbor updates, optional CSV/metrics.

Runs in a dedicated thread; scenarios activate via DroneController.start_swarm_coord_exchange().
"""

from __future__ import annotations

import json
import logging
import os
import random
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

from core.control.drone_controller import DroneController
from core.logging.csv_logger import write_row

logger = logging.getLogger(__name__)

try:
    from visualizer.position_publisher import publish_positions as _publish_positions
except ImportError:
    _publish_positions = None


def default_local_to_common_ned(
    drone_id: int,
    position: Dict[str, float],
    *,
    east_spacing_m: float = 2.0,
) -> Dict[str, float]:
    """Map per-SITL local NED to common frame (matches launch_simulation home East offset).

    Each instance is offset by (drone_id - 1) * east_spacing_m m East; subtract that from y.
    """
    return {
        **position,
        "y": position["y"] - (drone_id - 1) * east_spacing_m,
    }


def read_webots_step_hz(
    webots_last_ts: Dict[int, float],
    *,
    num_instances: int = 2,
) -> Optional[float]:
    """Read Webots step timestamps from /tmp and compute min step rate in Hz."""
    hz_list: List[float] = []
    for i in range(num_instances):
        path = f"/tmp/webots_step_{i}.txt"
        try:
            with open(path, "r", encoding="utf-8") as f:
                t_curr = float(f.read().strip())
        except (FileNotFoundError, ValueError, OSError):
            continue
        if i in webots_last_ts:
            dt = t_curr - webots_last_ts[i]
            if dt > 0:
                hz_list.append(1.0 / dt)
        webots_last_ts[i] = t_curr
    return min(hz_list) if hz_list else None


def distance_3d(pi: Dict[str, float], pj: Dict[str, float]) -> float:
    """Euclidean distance between two position dicts (x, y, z)."""
    return (
        (pi["x"] - pj["x"]) ** 2
        + (pi["y"] - pj["y"]) ** 2
        + (pi["z"] - pj["z"]) ** 2
    ) ** 0.5


def compute_collision_flags(
    positions: Dict[int, Dict[str, float]],
    collision_radius: float,
) -> Dict[int, bool]:
    """For each drone_id return True if in collision with any other (spheres 2*R)."""
    ids = list(positions.keys())
    collision = {i: False for i in ids}
    for i in ids:
        for j in ids:
            if i >= j:
                continue
            pi, pj = positions[i], positions[j]
            d = distance_3d(pi, pj)
            if d < 2 * collision_radius:
                collision[i] = True
                collision[j] = True
    return collision


def formation_metrics_step(
    positions: Dict[int, Dict[str, float]],
    collision_radius: float,
    d_star: float,
) -> Tuple[float, float, bool]:
    """Return (max formation error, min pairwise distance, any collision this step)."""
    ids = sorted(positions.keys())
    formation_error = 0.0
    min_dist = float("inf")
    has_collision = False
    for i in ids:
        for j in ids:
            if i >= j:
                continue
            pi, pj = positions[i], positions[j]
            d_ij = distance_3d(pi, pj)
            d_star_ij = d_star * abs(i - j)
            formation_error = max(formation_error, abs(d_ij - d_star_ij))
            min_dist = min(min_dist, d_ij)
            if d_ij < 2 * collision_radius:
                has_collision = True
    min_dist = min_dist if min_dist != float("inf") else 0.0
    return (formation_error, min_dist, has_collision)


@dataclass
class CoordExchangeNoiseConfig:
    """Gaussian noise on exchanged neighbor positions and optional packet loss."""

    position_sigma_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    packet_loss_probability: float = 0.0
    seed: Optional[int] = None

    @classmethod
    def from_mapping(cls, data: Dict[str, Any]) -> CoordExchangeNoiseConfig:
        """Build from a config dict (e.g. JSON or scenario kwargs)."""
        pos = data.get("position_sigma_m")
        if pos is not None:
            t = tuple(float(x) for x in pos)  # type: ignore[assignment]
            if len(t) != 3:
                raise ValueError("position_sigma_m must have 3 elements")
            sigma = (t[0], t[1], t[2])
        else:
            sx = float(data.get("position_sigma_x_m", 0.0))
            sy = float(data.get("position_sigma_y_m", 0.0))
            sz = float(data.get("position_sigma_z_m", 0.0))
            sigma = (sx, sy, sz)
        pl = float(data.get("packet_loss_probability", 0.0))
        seed = data.get("seed")
        return cls(
            position_sigma_m=sigma,
            packet_loss_probability=max(0.0, min(1.0, pl)),
            seed=int(seed) if seed is not None else None,
        )


@dataclass
class CoordExchangeStepContext:
    """Snapshot passed to optional on_step callback."""

    controllers: List[DroneController]
    positions: Dict[int, Dict[str, float]]
    attitudes: Dict[int, Dict[str, float]]
    time_elapsed: float
    wall_time: float
    rates_shared: Dict[str, Optional[float]]


class CoordExchangeManager:
    """Threaded loop: read poses, optional neighbor exchange (with noise/loss), viz, CSV."""

    _EXCHANGE_HZ_WINDOW = 20

    def __init__(
        self,
        controllers: List[DroneController],
        *,
        experiment_start_time: Optional[float] = None,
        experiment_log_files: Optional[Dict[int, Any]] = None,
        duration: float = 0.0,
        collision_radius: float = 0.2,
        log_hz: float = 0.0,
        exchange_loop_hz: float = 50.0,
        experiment_dir: Optional[str] = None,
        formation_d_star_m: float = 2.0,
        east_spacing_m: float = 2.0,
        local_to_common: Optional[Callable[[int, Dict[str, float]], Dict[str, float]]] = None,
        rates_shared: Optional[Dict[str, Optional[float]]] = None,
        noise: Optional[CoordExchangeNoiseConfig] = None,
        noise_dict: Optional[Dict[str, Any]] = None,
        update_neighbors: bool = True,
        publish_visualizer: bool = True,
        publish_measured_exchange_hz: bool = True,
        on_step: Optional[Callable[[CoordExchangeStepContext], None]] = None,
        daemon: bool = True,
    ) -> None:
        self._controllers = controllers
        self._experiment_t0 = experiment_start_time
        self._experiment_log_files = experiment_log_files
        self._duration = duration
        self._collision_radius = collision_radius
        self._log_hz = log_hz
        self._exchange_loop_hz = exchange_loop_hz
        self._experiment_dir = experiment_dir
        self._formation_d_star_m = formation_d_star_m
        self._east_spacing_m = east_spacing_m
        self._local_to_common = local_to_common or (
            lambda did, pos: default_local_to_common_ned(
                did, pos, east_spacing_m=east_spacing_m
            )
        )
        self.rates_shared: Dict[str, Optional[float]] = rates_shared if rates_shared is not None else {}
        self._update_neighbors = update_neighbors
        self._publish_visualizer = publish_visualizer
        self._publish_measured_exchange_hz = publish_measured_exchange_hz
        self._on_step = on_step
        self._daemon = daemon

        if noise is not None:
            self._noise = noise
        elif noise_dict is not None:
            self._noise = CoordExchangeNoiseConfig.from_mapping(noise_dict)
        else:
            self._noise = CoordExchangeNoiseConfig()

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._webots_last_ts: Dict[int, float] = {}
        self._exchange_hz_history: List[float] = []

    def set_experiment_start_time(self, t: float) -> None:
        """Set reference time for duration (e.g. global START_TIME in a scenario)."""
        self._experiment_t0 = t

    def start(self) -> None:
        """Start the exchange loop in a background thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=self._daemon)
        self._thread.start()

    def stop(self, join_timeout_sec: float = 5.0) -> None:
        """Request stop and join the loop thread."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout_sec)
            self._thread = None

    def _elapsed(self) -> float:
        ref = self._experiment_t0
        if ref is None:
            return 0.0
        return time.time() - ref

    def _apply_position_noise(self, pos: Dict[str, float], rng: random.Random) -> Dict[str, float]:
        sx, sy, sz = self._noise.position_sigma_m
        if sx == 0.0 and sy == 0.0 and sz == 0.0:
            return dict(pos)
        return {
            "x": pos["x"] + rng.gauss(0.0, sx),
            "y": pos["y"] + rng.gauss(0.0, sy),
            "z": pos["z"] + rng.gauss(0.0, sz),
        }

    def _run_loop(self) -> None:
        if self._experiment_t0 is None:
            self._experiment_t0 = time.time()

        rng = random.Random(self._noise.seed)

        last_csv_log_time = 0.0
        log_period = (1.0 / self._log_hz) if self._log_hz > 0 else 0.0
        loop_period = (1.0 / self._exchange_loop_hz) if self._exchange_loop_hz > 0 else 0.0

        formation_error_max_run = 0.0
        min_distance_run = float("inf")
        steps_with_collision = 0

        try:
            while not self._stop_event.is_set():
                if self._duration > 0 and self._elapsed() >= self._duration:
                    break
                t0_iter = time.time()
                positions: Dict[int, Dict[str, float]] = {}
                attitudes: Dict[int, Dict[str, float]] = {}
                for controller in self._controllers:
                    did = int(controller.config["id"])
                    pos_raw = controller.get_my_position()
                    positions[did] = self._local_to_common(did, pos_raw)
                    attitudes[did] = controller.get_attitude()

                if self._update_neighbors:
                    pl = self._noise.packet_loss_probability
                    for controller in self._controllers:
                        my_id = int(controller.config["id"])
                        for drone_id, position in positions.items():
                            if drone_id == my_id:
                                continue
                            if pl > 0.0 and rng.random() < pl:
                                continue
                            noisy = self._apply_position_noise(position, rng)
                            controller.update_other_drone_position(drone_id, noisy)

                wb = read_webots_step_hz(
                    self._webots_last_ts,
                    num_instances=len(self._controllers),
                )
                self.rates_shared["webots_step_hz"] = wb

                if self._publish_visualizer and _publish_positions is not None:
                    try:
                        _publish_positions(positions, rates=self.rates_shared)
                    except Exception:
                        pass

                ctx = CoordExchangeStepContext(
                    controllers=self._controllers,
                    positions=positions,
                    attitudes=attitudes,
                    time_elapsed=self._elapsed(),
                    wall_time=time.time(),
                    rates_shared=self.rates_shared,
                )
                if self._on_step is not None:
                    try:
                        self._on_step(ctx)
                    except Exception:
                        logger.exception("on_step callback failed")

                if self._experiment_log_files or self._experiment_dir:
                    now = time.time()
                    t_elapsed = self._elapsed()
                    rate_ok = log_period <= 0 or (now - last_csv_log_time) >= log_period
                    collision_flags = compute_collision_flags(
                        positions, self._collision_radius
                    )
                    formation_err_step, min_dist_step, has_collision_step = (
                        formation_metrics_step(
                            positions,
                            self._collision_radius,
                            self._formation_d_star_m,
                        )
                    )
                    formation_error_max_run = max(
                        formation_error_max_run, formation_err_step
                    )
                    if min_dist_step < min_distance_run:
                        min_distance_run = min_dist_step
                    if has_collision_step:
                        steps_with_collision += 1
                    wrote_this_iter = False
                    if self._experiment_log_files and rate_ok:
                        for controller in self._controllers:
                            did = int(controller.config["id"])
                            pos = positions[did]
                            att = attitudes[did]
                            hc = 1 if collision_flags[did] else 0
                            write_row(
                                self._experiment_log_files[did],
                                did,
                                t_elapsed,
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
                    elapsed = time.time() - t0_iter
                    sleep_sec = loop_period - elapsed
                    if sleep_sec > 0:
                        time.sleep(sleep_sec)

                if self._publish_measured_exchange_hz:
                    dt_full = time.time() - t0_iter
                    if dt_full > 0:
                        self._exchange_hz_history.append(1.0 / dt_full)
                        if len(self._exchange_hz_history) > self._EXCHANGE_HZ_WINDOW:
                            self._exchange_hz_history = self._exchange_hz_history[
                                -self._EXCHANGE_HZ_WINDOW :
                            ]
                        self.rates_shared["exchange_hz"] = sum(
                            self._exchange_hz_history
                        ) / len(self._exchange_hz_history)
        finally:
            if self._experiment_log_files:
                for _, f in self._experiment_log_files.items():
                    try:
                        f.close()
                    except Exception:
                        pass
            if self._experiment_dir:
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
                    "d_star_m": self._formation_d_star_m,
                }
                path = os.path.join(self._experiment_dir, "formation_metrics.json")
                try:
                    with open(path, "w", encoding="utf-8") as f:
                        json.dump(metrics, f, indent=2)
                    logger.info("Wrote formation metrics to %s", path)
                except OSError as e:
                    logger.warning("Could not write formation_metrics.json: %s", e)
