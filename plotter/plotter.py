#!/usr/bin/env python3
"""
Plot experiment CSV logs: **batch** runs (``batch_run.json`` + ``drone_*.csv`` per run dir) or a **single**
run folder (``metadata.json`` + ``drone_*.csv`` at e.g. ``experiments/<YYYY-MM-DD_HH-MM-SS>/``).

Run from project root (or pass ``--project-root``).

Examples:
  python plotter/plotter.py --experiment-dir experiments/2026-04-19_15-59-36 --out-dir plots
  # → plots/<stamp>/single_run_time_overlay.png (лидер drone_1, follower drone_2)
  python plotter/plotter.py --experiment-dir experiments/… --time-overlay-metric chain_mean_dxy
  # → single_run_time_overlay_chain.png — среднее расстояние по звеньям 1→2→…→N (все drone_*.csv)
  python plotter/plotter.py --mode time_overlay --out-dir plots
  # → plots/<same session stamp>/batch_time_overlay.png when runs are under experiments/<stamp>/
  python plotter/plotter.py --runs-glob 'experiments/2026-04-08_22-01-21/batch_*_run_*'  # явная сессия
  python plotter/plotter.py --mode bar_metric --vary pid.p_gain --metric-tail-sec 15
  python plotter/plotter.py --mode heatmap --vary-x pid.p_gain --vary-y sitl.ANGLE_MAX
"""

from __future__ import annotations

import argparse
import json
import logging
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Non-interactive backend
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from core.experiment.csv_io import (  # noqa: E402
    chain_mean_distance_xy_series,
    leader_follower_distance_xy_series,
    leader_follower_dx_series,
    load_drone_csv,
    rms_tail,
)

BATCH_RUN_JSON = "batch_run.json"
METADATA_JSON = "metadata.json"

# Batch session folders: experiments/YYYY-MM-DD_HH-MM-SS/ (4-digit year only).
_EXPERIMENT_SESSION_DIR_RE = re.compile(
    r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}$"
)


def _params_legend(params: Dict[str, Any], max_keys: int = 4) -> str:
    items = list(params.items())[:max_keys]
    return ", ".join(
        f"{k}={v:g}" if isinstance(v, float) else f"{k}={v}" for k, v in items
    )


def discover_run_dirs(project_root: Path, pattern: str) -> List[Path]:
    """Sorted list of existing directories matching glob under project_root."""
    paths = sorted(project_root.glob(pattern))
    return [p for p in paths if p.is_dir()]


def latest_batch_session_dir(project_root: Path) -> Optional[Path]:
    """Newest ``experiments/YYYY-MM-DD_HH-MM-SS`` directory (lexicographic max on name)."""
    exp_root = project_root / "experiments"
    if not exp_root.is_dir():
        return None
    candidates = [
        p
        for p in exp_root.iterdir()
        if p.is_dir() and _EXPERIMENT_SESSION_DIR_RE.fullmatch(p.name)
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.name)


def discover_batch_run_dirs_latest_session(
    project_root: Path,
) -> Tuple[List[Path], str]:
    """Run dirs under the latest session only: ``experiments/<stamp>/batch_*_run_*``.

    Returns:
        (directories, glob pattern string used for messages).
    """
    session = latest_batch_session_dir(project_root)
    if session is None:
        return [], ""
    rel = f"experiments/{session.name}/batch_*_run_*"
    return discover_run_dirs(project_root, rel), rel


def infer_experiment_session_stamp_for_outputs(run_dirs: List[Path]) -> Optional[str]:
    """Session folder name ``YYYY-MM-DD_HH-MM-SS`` if all runs live under one ``experiments/<stamp>/``.

    Used to mirror experiment layout under ``plots/<stamp>/``. If runs are flat under
    ``experiments/`` or span several stamps, returns ``None`` (figures go directly under
    ``--out-dir``). Logs a warning when multiple stamps are mixed.
    """
    stamps: set[str] = set()
    for rd in run_dirs:
        p = rd.resolve()
        parent = p.parent
        grand = parent.parent
        # Batch layout: experiments/<stamp>/batch_*_run_*
        if (
            grand.name == "experiments"
            and _EXPERIMENT_SESSION_DIR_RE.fullmatch(parent.name) is not None
        ):
            stamps.add(parent.name)
        # Single run at experiments/<stamp>/ (metadata.json + drone_*.csv at session root)
        elif (
            parent.name == "experiments"
            and _EXPERIMENT_SESSION_DIR_RE.fullmatch(p.name) is not None
        ):
            stamps.add(p.name)
    if len(stamps) == 1:
        return next(iter(stamps))
    if len(stamps) > 1:
        logger.warning(
            "Run directories span %d different experiment session folders; "
            "writing figures directly under --out-dir (no session subfolder).",
            len(stamps),
        )
    return None


def _time_x_for_interp(t: np.ndarray, x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Sort by time; collapse duplicate timestamps with mean x (``np.interp`` needs increasing t)."""
    if len(t) == 0:
        return t.astype(float), x.astype(float)
    order = np.argsort(t)
    t_s = t[order].astype(float)
    x_s = x[order].astype(float)
    t_u, inv = np.unique(t_s, return_inverse=True)
    x_u = np.bincount(inv, weights=x_s) / np.bincount(inv)
    return t_u, x_u


def leader_follower_dx_resampled(
    drone1: np.ndarray,
    drone2: np.ndarray,
    num_points: int = 800,
) -> Tuple[np.ndarray, np.ndarray]:
    """Δx on a uniform time grid over the overlap of both CSVs.

    Raw logs often have irregular spacing (write on pose change); plotting ``(t, dx)`` as-is
    looks jagged. Linear interpolation of each drone's ``x(t)`` onto ``linspace`` recovers a
    smooth polyline without changing the underlying samples.
    """
    if len(drone1) == 0 or len(drone2) == 0:
        return np.array([]), np.array([])
    t1, x1 = _time_x_for_interp(drone1[:, 0], drone1[:, 1])
    t2, x2 = _time_x_for_interp(drone2[:, 0], drone2[:, 1])
    t_start = max(float(t1[0]), float(t2[0]))
    t_end = min(float(t1[-1]), float(t2[-1]))
    if t_end <= t_start:
        return leader_follower_dx_series(drone1, drone2)
    n = int(np.clip(num_points, 2, 10000))
    t_fine = np.linspace(t_start, t_end, n)
    x1i = np.interp(t_fine, t1, x1)
    x2i = np.interp(t_fine, t2, x2)
    return t_fine, x2i - x1i


def leader_follower_distance_xy_resampled(
    drone1: np.ndarray,
    drone2: np.ndarray,
    num_points: int = 800,
) -> Tuple[np.ndarray, np.ndarray]:
    """Planar distance sqrt((x2-x1)^2+(y2-y1)^2) on a uniform time grid (see dx resampled)."""
    if len(drone1) == 0 or len(drone2) == 0:
        return np.array([]), np.array([])
    t1x, x1 = _time_x_for_interp(drone1[:, 0], drone1[:, 1])
    t1y, y1 = _time_x_for_interp(drone1[:, 0], drone1[:, 2])
    t2x, x2 = _time_x_for_interp(drone2[:, 0], drone2[:, 1])
    t2y, y2 = _time_x_for_interp(drone2[:, 0], drone2[:, 2])
    t_start = max(float(t1x[0]), float(t1y[0]), float(t2x[0]), float(t2y[0]))
    t_end = min(float(t1x[-1]), float(t1y[-1]), float(t2x[-1]), float(t2y[-1]))
    if t_end <= t_start:
        return leader_follower_distance_xy_series(drone1, drone2)
    n = int(np.clip(num_points, 2, 10000))
    t_fine = np.linspace(t_start, t_end, n)
    x1i = np.interp(t_fine, t1x, x1)
    y1i = np.interp(t_fine, t1y, y1)
    x2i = np.interp(t_fine, t2x, x2)
    y2i = np.interp(t_fine, t2y, y2)
    d = np.hypot(x2i - x1i, y2i - y1i)
    return t_fine, d.astype(float)


def chain_mean_distance_xy_resampled(
    drones: List[np.ndarray],
    num_points: int = 800,
) -> Tuple[np.ndarray, np.ndarray]:
    """Mean planar XY distance along consecutive drones on a uniform time grid.

    Time window: intersection of all drones' CSV time spans. Interpolates each ``(x,y)`` onto
    ``linspace`` then averages link lengths (same definition as
    :func:`core.experiment.csv_io.chain_mean_distance_xy_series`).
    """
    if len(drones) < 2 or any(len(d) == 0 for d in drones):
        return np.array([]), np.array([])
    t_starts: List[float] = []
    t_ends: List[float] = []
    for d in drones:
        if len(d) == 0:
            return np.array([]), np.array([])
        t_starts.append(float(np.min(d[:, 0])))
        t_ends.append(float(np.max(d[:, 0])))
    t_start = max(t_starts)
    t_end = min(t_ends)
    if t_end <= t_start:
        return chain_mean_distance_xy_series(drones)
    n = int(np.clip(num_points, 2, 10000))
    t_fine = np.linspace(t_start, t_end, n)
    xs: List[np.ndarray] = []
    ys: List[np.ndarray] = []
    for d in drones:
        tx, x = _time_x_for_interp(d[:, 0], d[:, 1])
        ty, y = _time_x_for_interp(d[:, 0], d[:, 2])
        xi = np.interp(t_fine, tx, x)
        yi = np.interp(t_fine, ty, y)
        xs.append(xi)
        ys.append(yi)
    link_dists: List[np.ndarray] = []
    for i in range(len(drones) - 1):
        link_dists.append(np.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]))
    stacked = np.vstack(link_dists)
    mean_d = np.mean(stacked, axis=0)
    return t_fine, mean_d.astype(float)


def _leader_follower_series(
    d1: np.ndarray,
    d2: np.ndarray,
    *,
    metric: str,
    resample_points: int,
) -> Tuple[np.ndarray, np.ndarray]:
    if metric == "dx":
        if resample_points > 0:
            return leader_follower_dx_resampled(d1, d2, num_points=resample_points)
        return leader_follower_dx_series(d1, d2)
    if metric == "dxy":
        if resample_points > 0:
            return leader_follower_distance_xy_resampled(d1, d2, num_points=resample_points)
        return leader_follower_distance_xy_series(d1, d2)
    raise ValueError(f"unknown overlay metric {metric!r}")


def load_run_pair_csvs(
    run_dir: Path,
) -> Tuple[Optional[Dict[str, Any]], Optional[np.ndarray], Optional[np.ndarray]]:
    """Load ``batch_run.json`` and both drone CSV arrays, or Nones if missing."""
    meta_path = run_dir / BATCH_RUN_JSON
    if not meta_path.is_file():
        logger.warning("Skip %s: no %s", run_dir, BATCH_RUN_JSON)
        return None, None, None
    with meta_path.open(encoding="utf-8") as f:
        meta = json.load(f)
    d1p = run_dir / "drone_1.csv"
    d2p = run_dir / "drone_2.csv"
    if not d1p.is_file() or not d2p.is_file():
        logger.warning("Skip %s: missing drone_1/2 CSV", run_dir)
        return meta, None, None
    d1 = load_drone_csv(d1p)
    d2 = load_drone_csv(d2p)
    return meta, d1, d2


def load_single_run_pair_csvs(
    experiment_dir: Path,
) -> Tuple[Optional[Dict[str, Any]], Optional[np.ndarray], Optional[np.ndarray]]:
    """Load standalone experiment: ``metadata.json`` + ``drone_1.csv`` / ``drone_2.csv`` (no batch_run).

    Returns a dict shaped like batch metadata so :func:`plot_time_overlay` and RMS metrics work:
    ``params`` is filled from ``scenario`` / ``num_drones`` for the legend.
    """
    meta_path = experiment_dir / METADATA_JSON
    if not meta_path.is_file():
        logger.warning("Skip %s: no %s", experiment_dir, METADATA_JSON)
        return None, None, None
    with meta_path.open(encoding="utf-8") as f:
        meta_raw = json.load(f)
    d1p = experiment_dir / "drone_1.csv"
    d2p = experiment_dir / "drone_2.csv"
    if not d1p.is_file() or not d2p.is_file():
        logger.warning("Skip %s: missing drone_1/2 CSV", experiment_dir)
        return None, None, None
    d1 = load_drone_csv(d1p)
    d2 = load_drone_csv(d2p)
    params: Dict[str, Any] = {}
    sc = meta_raw.get("scenario")
    if sc is not None:
        params["scenario"] = sc
    nd = meta_raw.get("num_drones")
    if nd is not None:
        params["num_drones"] = int(nd)
    meta: Dict[str, Any] = {
        "run_index": 1,
        "params": params,
        "scenario_id": sc,
        "metadata": meta_raw,
    }
    return meta, d1, d2


def load_experiment_dir_pair_csvs(
    experiment_dir: Path,
) -> Tuple[Optional[Dict[str, Any]], Optional[np.ndarray], Optional[np.ndarray]]:
    """Prefer ``batch_run.json`` if present (one batch run dir); else ``metadata.json`` standalone."""
    if (experiment_dir / BATCH_RUN_JSON).is_file():
        return load_run_pair_csvs(experiment_dir)
    return load_single_run_pair_csvs(experiment_dir)


def sequential_drone_csv_paths(run_dir: Path) -> List[Path]:
    """``drone_1.csv``, ``drone_2.csv``, … until the first missing index."""
    paths: List[Path] = []
    i = 1
    while True:
        p = run_dir / f"drone_{i}.csv"
        if not p.is_file():
            break
        paths.append(p)
        i += 1
    return paths


def load_experiment_meta_dict(run_dir: Path) -> Optional[Dict[str, Any]]:
    """Same metadata shape as batch/single pair loaders, without requiring drone CSV presence."""
    if (run_dir / BATCH_RUN_JSON).is_file():
        with (run_dir / BATCH_RUN_JSON).open(encoding="utf-8") as f:
            return json.load(f)
    meta_path = run_dir / METADATA_JSON
    if not meta_path.is_file():
        return None
    with meta_path.open(encoding="utf-8") as f:
        meta_raw = json.load(f)
    params: Dict[str, Any] = {}
    sc = meta_raw.get("scenario")
    if sc is not None:
        params["scenario"] = sc
    nd = meta_raw.get("num_drones")
    if nd is not None:
        params["num_drones"] = int(nd)
    return {
        "run_index": 1,
        "params": params,
        "scenario_id": sc,
        "metadata": meta_raw,
    }


def load_experiment_dir_drone_chain(
    run_dir: Path,
) -> Tuple[Optional[Dict[str, Any]], Optional[List[np.ndarray]]]:
    """Load ``batch_run.json`` or ``metadata.json`` and all sequential ``drone_*.csv``.

    Returns:
        ``(meta, drones)`` with ``drones = [drone_1, …, drone_N]``, or Nones if unusable.
    """
    meta = load_experiment_meta_dict(run_dir)
    paths = sequential_drone_csv_paths(run_dir)
    if meta is None:
        logger.warning("Skip %s: no batch_run.json or metadata.json", run_dir)
        return None, None
    if len(paths) < 2:
        logger.warning(
            "Skip %s: chain_mean needs ≥2 sequential drone_*.csv (found %d)",
            run_dir,
            len(paths),
        )
        return meta, None
    drones = [load_drone_csv(p) for p in paths]
    return meta, drones


def load_run_dx(
    run_dir: Path,
    *,
    resample_points: int = 800,
) -> Tuple[Optional[Dict[str, Any]], Optional[np.ndarray], Optional[np.ndarray]]:
    """Load batch_run.json and leader–follower Δx series (drone 1 vs 2).

    Args:
        run_dir: Experiment directory with ``batch_run.json`` and ``drone_*.csv``.
        resample_points: If > 0, resample Δx on a uniform grid over CSV time overlap
            (smoother plots for sparse rows). If 0, use leader time base only
            (:func:`leader_follower_dx_series`).
    """
    meta, d1, d2 = load_run_pair_csvs(run_dir)
    if meta is None or d1 is None or d2 is None:
        return meta, None, None
    t, dx = _leader_follower_series(
        d1, d2, metric="dx", resample_points=resample_points
    )
    return meta, t, dx


def plot_time_overlay(
    runs: List[Tuple[Path, Dict[str, Any], np.ndarray, np.ndarray]],
    out_path: Path,
    title: str,
    ylabel: str,
) -> None:
    """One figure: overlay metric vs time for each run."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(10, 5))
    for run_dir, meta, t, yvals in runs:
        params = meta.get("params") or {}
        label = _params_legend(params)
        plt.plot(t, yvals, label=label[:80], linewidth=1.0, antialiased=True)
    plt.xlabel("t (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend(fontsize=7, loc="best")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def plot_bar_metric(
    runs: List[Tuple[Path, Dict[str, Any], float]],
    out_path: Path,
    vary_key: Optional[str],
    title: str,
) -> None:
    """Bar chart: one bar per run; x = vary_key value or run_index, y = metric."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    xs: List[float] = []
    ys: List[float] = []
    for _run_dir, meta, metric in runs:
        params = meta.get("params") or {}
        if vary_key and vary_key in params:
            xs.append(float(params[vary_key]))
        else:
            xs.append(float(meta.get("run_index", 0)))
        ys.append(metric)
    order = np.argsort(np.array(xs))
    xs_o = [xs[i] for i in order]
    ys_o = [ys[i] for i in order]
    plt.figure(figsize=(max(8, len(xs) * 0.4), 5))
    xpos = np.arange(len(xs_o))
    plt.bar(xpos, ys_o, color="steelblue", alpha=0.85)
    plt.xticks(xpos, [f"{x:g}" for x in xs_o], rotation=45, ha="right")
    plt.ylabel("RMS Δx (tail, m)")
    xlab = vary_key if vary_key else "run"
    plt.xlabel(xlab)
    plt.title(title)
    plt.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    logger.info("Wrote %s (%s runs)", out_path, len(runs))


def plot_heatmap(
    runs: List[Tuple[Dict[str, Any], float]],
    out_path: Path,
    vx: str,
    vy: str,
    title: str,
) -> bool:
    """2D grid heatmap of RMS metric vs two parameter axes."""
    pts: Dict[Tuple[float, float], float] = {}
    for meta, metric in runs:
        p = meta.get("params") or {}
        if vx not in p or vy not in p:
            continue
        pts[(float(p[vx]), float(p[vy]))] = metric
    if not pts:
        logger.error("No runs with both %s and %s in params.", vx, vy)
        return False
    ux = sorted({a for a, _ in pts.keys()})
    uy = sorted({b for _, b in pts.keys()})
    grid = np.full((len(uy), len(ux)), np.nan, dtype=float)
    for i, yv in enumerate(uy):
        for j, xv in enumerate(ux):
            grid[i, j] = pts.get((xv, yv), np.nan)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(7, 5))
    im = plt.imshow(
        grid,
        aspect="auto",
        origin="lower",
        extent=(
            min(ux) - 0.25,
            max(ux) + 0.25,
            min(uy) - 0.25,
            max(uy) + 0.25,
        ),
    )
    plt.colorbar(im, label="RMS Δx (m)")
    plt.xlabel(vx)
    plt.ylabel(vy)
    plt.title(title)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()
    return True


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    parser = argparse.ArgumentParser(
        description=(
            "Experiment plotter: batch runs (batch_run.json) or single run "
            "(metadata.json under experiments/<stamp>/)."
        )
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        default=_PROJECT_ROOT,
        help="Repository root (default: parent of plotter/).",
    )
    parser.add_argument(
        "--experiment-dir",
        type=Path,
        default=None,
        metavar="DIR",
        help=(
            "Single experiment folder (metadata.json + drone_*.csv), or one batch run folder "
            "with batch_run.json. Relative paths are resolved under --project-root. "
            "When set, --runs-glob is ignored."
        ),
    )
    parser.add_argument(
        "--runs-glob",
        type=str,
        default=None,
        metavar="PATTERN",
        help=(
            "Glob relative to project root for run directories. "
            "Default: experiments/<latest YYYY-MM-DD_HH-MM-SS>/batch_*_run_* "
            "(newest session folder under experiments/)."
        ),
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("plots"),
        help=(
            "Output directory for figures (relative to project root unless absolute). "
            "If runs sit under experiments/YYYY-MM-DD_HH-MM-SS/, PNGs go to "
            "<out-dir>/<same-stamp>/."
        ),
    )
    parser.add_argument(
        "--mode",
        choices=("time_overlay", "bar_metric", "heatmap"),
        default="time_overlay",
        help="time_overlay: Δx(t) curves; bar_metric: RMS tail vs run or --vary; heatmap: two axes.",
    )
    parser.add_argument(
        "--metric-tail-sec",
        type=float,
        default=10.0,
        help="Window at end of run for RMS(Δx) metric (bar_metric / heatmap).",
    )
    parser.add_argument(
        "--vary",
        type=str,
        default=None,
        help="Parameter key for bar chart x-axis (e.g. pid.p_gain).",
    )
    parser.add_argument(
        "--vary-x",
        type=str,
        default=None,
        metavar="KEY",
        help="heatmap: first axis key in batch_run params.",
    )
    parser.add_argument(
        "--vary-y",
        type=str,
        default=None,
        metavar="KEY",
        help="heatmap: second axis key.",
    )
    parser.add_argument(
        "--resample-points",
        type=int,
        default=800,
        metavar="N",
        help=(
            "Resample series on N uniform times over CSV overlap (smoother curves); "
            "0 = use native timestamps (sparse/jagged)."
        ),
    )
    parser.add_argument(
        "--time-overlay-metric",
        choices=("dx", "dxy", "chain_mean_dxy"),
        default="dxy",
        help=(
            "time_overlay: dx = Δx между drone_1 и drone_2; "
            "dxy = √(Δx²+Δy²) между ними же; "
            "chain_mean_dxy = среднее по звеньям (1→2,…,N−1→N) по всем drone_*.csv, N≥2."
        ),
    )
    args = parser.parse_args()
    root = args.project_root.resolve()
    out_base = args.out_dir if args.out_dir.is_absolute() else root / args.out_dir

    single_experiment_dir: Optional[Path] = None
    if args.experiment_dir is not None:
        single_experiment_dir = (
            args.experiment_dir
            if args.experiment_dir.is_absolute()
            else (root / args.experiment_dir)
        ).resolve()
        if not single_experiment_dir.is_dir():
            logger.error("Not a directory: %s", single_experiment_dir)
            sys.exit(1)
        run_dirs = [single_experiment_dir]
        glob_desc = str(single_experiment_dir.relative_to(root)) if single_experiment_dir.is_relative_to(root) else str(single_experiment_dir)
        logger.info("Single experiment: %s", glob_desc)
    elif args.runs_glob is None:
        run_dirs, glob_desc = discover_batch_run_dirs_latest_session(root)
        if not glob_desc:
            logger.error(
                "No batch session folder matching YYYY-MM-DD_HH-MM-SS under %s",
                root / "experiments",
            )
            sys.exit(1)
        logger.info("Using latest batch session: %s", glob_desc)
    else:
        run_dirs = discover_run_dirs(root, args.runs_glob)
        glob_desc = args.runs_glob
    if not run_dirs:
        logger.error("No run directories matched %s under %s", glob_desc, root)
        sys.exit(1)

    session_for_plots = infer_experiment_session_stamp_for_outputs(run_dirs)
    plot_out_base = out_base / session_for_plots if session_for_plots else out_base
    if session_for_plots:
        logger.info("Output under session folder: %s", plot_out_base)

    if args.mode == "heatmap" and single_experiment_dir is not None:
        logger.error("heatmap needs multiple runs; omit --experiment-dir.")
        sys.exit(1)

    rs = max(0, args.resample_points)
    use_chain = args.time_overlay_metric == "chain_mean_dxy"
    loaded: List[Tuple[Path, Dict[str, Any], np.ndarray, np.ndarray]] = []
    metrics: List[Tuple[Path, Dict[str, Any], float]] = []
    for rd in run_dirs:
        if use_chain:
            meta, drones = load_experiment_dir_drone_chain(rd)
            if meta is None or drones is None or len(drones) < 2:
                continue
            if rs > 0:
                t_ov, y_ov = chain_mean_distance_xy_resampled(
                    drones, num_points=rs
                )
            else:
                t_ov, y_ov = chain_mean_distance_xy_series(drones)
            d1, d2 = drones[0], drones[1]
        else:
            meta, d1, d2 = load_experiment_dir_pair_csvs(rd)
            if meta is None or d1 is None or d2 is None:
                continue
            t_ov, y_ov = _leader_follower_series(
                d1, d2, metric=args.time_overlay_metric, resample_points=rs
            )
        if len(t_ov) == 0:
            continue
        loaded.append((rd, meta, t_ov, y_ov))
        t_dx, dx = _leader_follower_series(d1, d2, metric="dx", resample_points=rs)
        if len(t_dx) > 0:
            m = rms_tail(dx, t_dx, args.metric_tail_sec)
            if m is not None:
                metrics.append((rd, meta, m))

    if args.mode == "time_overlay":
        if not loaded:
            logger.error("No plottable runs.")
            sys.exit(1)
        chain_suffix = "_chain" if args.time_overlay_metric == "chain_mean_dxy" else ""
        if single_experiment_dir is not None:
            out = plot_out_base / f"single_run_time_overlay{chain_suffix}.png"
            batch_suffix = ""
        else:
            out = plot_out_base / f"batch_time_overlay{chain_suffix}.png"
            batch_suffix = " (batch)"
        if args.time_overlay_metric == "dx":
            ylab = "x_follower − x_leader (m)"
            ttl = f"Follower − leader Δx{batch_suffix}"
        elif args.time_overlay_metric == "chain_mean_dxy":
            ylab = "Mean planar link distance (chain 1→…→N) (m)"
            ttl = f"Mean XY distance along drone chain{batch_suffix}"
        else:
            ylab = "Planar distance √(Δx² + Δy²) (m)"
            ttl = f"Follower − leader planar distance{batch_suffix}"
        plot_time_overlay(loaded, out, title=ttl, ylabel=ylab)
        logger.info("Wrote %s", out)
    elif args.mode == "bar_metric":
        if not metrics:
            logger.error("No metrics computed (need drone CSVs + data in tail).")
            sys.exit(1)
        out = (
            plot_out_base / "single_run_bar_rms_dx.png"
            if single_experiment_dir is not None
            else plot_out_base / "batch_bar_rms_dx.png"
        )
        plot_bar_metric(
            [(a, b, c) for a, b, c in metrics],
            out,
            vary_key=args.vary,
            title=f"RMS Δx last {args.metric_tail_sec:g}s",
        )
    else:
        if not args.vary_x or not args.vary_y:
            logger.error("heatmap requires --vary-x and --vary-y.")
            sys.exit(1)
        pairs: List[Tuple[Dict[str, Any], float]] = []
        for _rd, meta, m in metrics:
            pairs.append((meta, m))
        out = plot_out_base / "batch_heatmap_rms_dx.png"
        if not plot_heatmap(
            pairs, out, args.vary_x, args.vary_y, title="RMS Δx heatmap"
        ):
            sys.exit(1)
        logger.info("Wrote %s", out)


if __name__ == "__main__":
    main()
