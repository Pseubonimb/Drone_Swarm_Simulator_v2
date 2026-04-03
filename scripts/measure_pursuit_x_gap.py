#!/usr/bin/env python3
"""
Measure leader–follower gap along X from experiment CSV (common NED, see leader_forward_back).

Exits 0 if max |x_leader - x_follower| on the evaluation window is <= --max-abs-dx.

Usage:
    python scripts/measure_pursuit_x_gap.py experiments/2026-03-29_12-00-00
    python scripts/measure_pursuit_x_gap.py /path/to/exp --tail-seconds 5 --max-abs-dx 0.2

For automation (pursuit-x-gap-orchestrate): run after launch_simulation with --experiment-dir.
"""

from __future__ import annotations

import argparse
import csv
import sys
from bisect import bisect_left
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

# Project root on path when run as python scripts/measure_pursuit_x_gap.py
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))


def _load_tx(csv_path: Path) -> List[Tuple[float, float]]:
    """Load (t, x) from drone CSV; skip bad lines."""
    rows: List[Tuple[float, float]] = []
    with csv_path.open(encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return rows
        h = ",".join(header)
        if "t" not in h or "x" not in h:
            raise ValueError(f"Unexpected header in {csv_path}: {h}")
        for parts in reader:
            if len(parts) < 2:
                continue
            try:
                t = float(parts[0])
                x = float(parts[1])
                rows.append((t, x))
            except ValueError:
                continue
    rows.sort(key=lambda r: r[0])
    return rows


def _interp_x(series: List[Tuple[float, float]], t: float) -> Optional[float]:
    """Linear interpolation of x at time t; None if out of range or empty."""
    if not series:
        return None
    ts = [p[0] for p in series]
    if t < ts[0] or t > ts[-1]:
        return None
    i = bisect_left(ts, t)
    if i == 0:
        return float(series[0][1])
    if i >= len(series):
        return float(series[-1][1])
    t0, x0 = series[i - 1]
    t1, x1 = series[i]
    if t1 == t0:
        return float(x0)
    alpha = (t - t0) / (t1 - t0)
    return float(x0 + alpha * (x1 - x0))


def compute_pursuit_x_gap_metrics(
    experiment_dir: Path,
    leader_id: int = 1,
    follower_id: int = 2,
    tail_seconds: float = 5.0,
    max_time_delta: float = 0.05,
) -> Tuple[Optional[float], Optional[float], str]:
    """Compute max and mean |Δx| on the last ``tail_seconds`` of overlapping timeline.

    Uses follower's time samples; x_leader interpolated at each t; drops samples where
    leader extrapolation is impossible or |t_leader_interp - t| notionally fails —
    here we only interpolate inside leader's [t_min, t_max].

    Args:
        experiment_dir: Directory with ``drone_{id}.csv``.
        leader_id: Leader drone id (default 1).
        follower_id: Follower drone id (default 2).
        tail_seconds: Evaluate on [t_end - tail_seconds, t_end] where t_end is max time.
        max_time_delta: Not enforced as hard filter on interp; reserved for future pairing.

    Returns:
        (max_abs_dx, mean_abs_dx, message). max/mean None if not computable.
    """
    _ = max_time_delta  # pairing tolerance for future nearest-neighbor mode
    leader_path = experiment_dir / f"drone_{leader_id}.csv"
    follower_path = experiment_dir / f"drone_{follower_id}.csv"
    if not leader_path.is_file():
        return None, None, f"Missing {leader_path}"
    if not follower_path.is_file():
        return None, None, f"Missing {follower_path}"

    leader_tx = _load_tx(leader_path)
    follower_tx = _load_tx(follower_path)
    # Leader needs >= 2 samples to interpolate x(t); follower can be a single timestamp.
    if len(leader_tx) < 2:
        return None, None, "Insufficient rows in leader CSV (need >= 2 for interpolation)"
    if len(follower_tx) < 1:
        return None, None, "Insufficient rows in follower CSV"

    t_end = max(leader_tx[-1][0], follower_tx[-1][0])
    t_start = max(0.0, t_end - tail_seconds)

    diffs: List[float] = []
    for t, xf in follower_tx:
        if t < t_start or t > t_end:
            continue
        xl = _interp_x(leader_tx, t)
        if xl is None:
            continue
        diffs.append(abs(xl - xf))

    if not diffs:
        return None, None, "No samples in evaluation window with valid leader interpolation"

    max_dx = max(diffs)
    mean_dx = sum(diffs) / len(diffs)
    return max_dx, mean_dx, f"n={len(diffs)} window=[{t_start:.3f},{t_end:.3f}]s"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check pursuit X gap (common NED) from experiment CSVs."
    )
    parser.add_argument(
        "experiment_dir",
        type=Path,
        help="Experiment folder with drone_1.csv, drone_2.csv",
    )
    parser.add_argument("--leader-id", type=int, default=1)
    parser.add_argument("--follower-id", type=int, default=2)
    parser.add_argument(
        "--tail-seconds",
        type=float,
        default=5.0,
        help="Last N seconds of timeline to evaluate (default 5)",
    )
    parser.add_argument(
        "--max-abs-dx",
        type=float,
        default=0.2,
        help="Accept if max |Δx| <= this (meters); exit 0 (default 0.2)",
    )
    parser.add_argument(
        "--max-time-delta",
        type=float,
        default=0.05,
        help="Reserved / pairing tolerance (default 0.05)",
    )
    args = parser.parse_args()
    exp = args.experiment_dir.expanduser().resolve()
    if not exp.is_dir():
        print(f"Not a directory: {exp}", file=sys.stderr)
        return 2

    max_dx, mean_dx, msg = compute_pursuit_x_gap_metrics(
        exp,
        leader_id=args.leader_id,
        follower_id=args.follower_id,
        tail_seconds=args.tail_seconds,
        max_time_delta=args.max_time_delta,
    )
    if max_dx is None or mean_dx is None:
        print(f"FAIL: {msg}", file=sys.stderr)
        return 2

    ok = max_dx <= args.max_abs_dx
    print(
        f"pursuit_x_gap: max|Δx|={max_dx:.4f}m mean|Δx|={mean_dx:.4f}m "
        f"threshold={args.max_abs_dx}m {msg} — {'PASS' if ok else 'FAIL'}"
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
