"""Load experiment drone CSV (t,x,y,z,...) for analysis and plotting."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np


def load_drone_csv(path: str | Path) -> np.ndarray:
    """Load one drone CSV; return array (t, x, y, z, rx, ry, rz, hasCollision)."""
    p = Path(path)
    data: list[list[float]] = []
    with p.open(encoding="utf-8") as f:
        header = f.readline()
        if "t,x,y,z" not in header:
            raise ValueError(f"Unexpected CSV header: {header.strip()}")
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) != 8:
                continue
            row = [
                float(parts[0]),
                float(parts[1]),
                float(parts[2]),
                float(parts[3]),
                float(parts[4]),
                float(parts[5]),
                float(parts[6]),
                int(parts[7]),
            ]
            data.append(row)
    return np.array(data, dtype=float) if data else np.zeros((0, 8))


def interpolate_scalar_on_times(
    t_target: np.ndarray, t_src: np.ndarray, v_src: np.ndarray
) -> np.ndarray:
    """Linearly interpolate ``v_src(t_src)`` onto ``t_target``.

    If ``len(t_src) < 2`` or ``t_target`` is empty, returns zeros like ``t_target``
    (same contract as legacy ``interpolate_x_on_times``).
    """
    if len(t_src) < 2 or len(t_target) == 0:
        return np.zeros_like(t_target, dtype=float)
    return np.interp(t_target, t_src, v_src)


def interpolate_x_on_times(
    t_target: np.ndarray, t_src: np.ndarray, x_src: np.ndarray
) -> np.ndarray:
    """Linearly interpolate x_src(t_src) onto t_target (for aligned leader/follower series)."""
    return interpolate_scalar_on_times(t_target, t_src, x_src)


def euclidean_xy_distance(
    x1: float | np.ndarray,
    y1: float | np.ndarray,
    x2: float | np.ndarray,
    y2: float | np.ndarray,
) -> float | np.ndarray:
    """Euclidean distance in the horizontal plane (X–Y only; Z ignored)."""
    dx = x2 - x1
    dy = y2 - y1
    return np.sqrt(dx * dx + dy * dy)


def leader_follower_distance_xy_series(
    drone_leader: np.ndarray, drone_follower: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(t, d)`` with leader time base: planar distance between positions.

    Interpolates follower's ``x,y`` onto the leader's timestamps. Rows follow
    ``load_drone_csv`` layout: columns 0–2 are ``t,x,y``.

    Args:
        drone_leader: Leader samples (at least one row).
        drone_follower: Follower samples (needs ≥2 rows for interpolation).

    Returns:
        Time vector (leader) and distance ``sqrt((xf-xl)^2 + (yf-yl)^2)``.
    """
    if len(drone_leader) == 0:
        return np.array([]), np.array([])
    t = drone_leader[:, 0]
    xl = drone_leader[:, 1]
    yl = drone_leader[:, 2]
    xf = interpolate_scalar_on_times(t, drone_follower[:, 0], drone_follower[:, 1])
    yf = interpolate_scalar_on_times(t, drone_follower[:, 0], drone_follower[:, 2])
    d = euclidean_xy_distance(xl, yl, xf, yf)
    return t, np.asarray(d, dtype=float)


def chain_mean_distance_xy_series(
    drones: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Mean planar XY distance over consecutive drones at each leader time.

    Uses ``drones[0]`` timestamps as the common time base. For each drone,
    ``x`` and ``y`` are linearly interpolated onto that base (same rules as
    :func:`interpolate_scalar_on_times`). For each step ``j``,

    .. math::

        d_{\\mathrm{mean}}(t_j) = \\frac{1}{N-1} \\sum_{i=0}^{N-2}
        \\big\\| \\mathbf{p}_{i+1}(t_j) - \\mathbf{p}_i(t_j) \\big\\|

    where :math:`\\mathbf{p}=(x,y)` in the coordinate frame stored in the CSV.

    Args:
        drones: Ordered chain ``[drone_1, …, drone_N]`` (leader first).

    Returns:
        ``(t, mean_distance)``. Empty arrays if ``drones`` is empty, any array
        has zero rows, or ``N < 2``.
    """
    if len(drones) < 2:
        return np.array([]), np.array([])
    if any(len(d) == 0 for d in drones):
        return np.array([]), np.array([])
    t = drones[0][:, 0]
    if len(t) == 0:
        return np.array([]), np.array([])
    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    for arr in drones:
        xs.append(interpolate_scalar_on_times(t, arr[:, 0], arr[:, 1]))
        ys.append(interpolate_scalar_on_times(t, arr[:, 0], arr[:, 2]))
    link_dists: list[np.ndarray] = []
    for i in range(len(drones) - 1):
        link_dists.append(
            np.asarray(
                euclidean_xy_distance(xs[i], ys[i], xs[i + 1], ys[i + 1]),
                dtype=float,
            )
        )
    stacked = np.vstack(link_dists)
    mean_d = np.mean(stacked, axis=0)
    return t, mean_d.astype(float)


def leader_follower_dx_series(
    drone1: np.ndarray, drone2: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Return (t, x_follower - x_leader) using drone 1 = leader, 2 = follower; times from leader."""
    if len(drone1) == 0:
        return np.array([]), np.array([])
    t = drone1[:, 0]
    x1 = drone1[:, 1]
    x2 = interpolate_x_on_times(t, drone2[:, 0], drone2[:, 1])
    return t, x2 - x1


def rms_tail(values: np.ndarray, times: np.ndarray, tail_sec: float) -> Optional[float]:
    """RMS of values where times >= max(times) - tail_sec."""
    if len(values) == 0 or tail_sec <= 0:
        return None
    tmax = float(np.max(times))
    mask = times >= tmax - tail_sec
    if not np.any(mask):
        return None
    v = values[mask]
    return float(np.sqrt(np.mean(v * v)))
