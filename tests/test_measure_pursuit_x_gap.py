"""Unit tests for scripts/measure_pursuit_x_gap.py (no SITL required)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.logging.csv_logger import CSV_HEADER
from scripts.measure_pursuit_x_gap import compute_pursuit_x_gap_metrics


def _write_drone_csv(path: Path, rows: list[tuple[float, ...]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write(CSV_HEADER + "\n")
        for row in rows:
            t, x, y, z, rx, ry, rz, hc = row
            f.write(f"{t},{x},{y},{z},{rx},{ry},{rz},{int(hc)}\n")


def test_compute_pursuit_x_gap_passes_when_gap_small(tmp_path: Path) -> None:
    """Last 5 s: |x1-x2| small -> max below 0.2."""
    exp = tmp_path / "exp"
    exp.mkdir()
    # Leader and follower x differ by 0.1 at t=10..15
    rows_l = [(float(t), 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0) for t in range(100, 160)]
    rows_f = [(float(t), 9.9, -2.0, 0.0, 0.0, 0.0, 0.0, 0) for t in range(100, 160)]
    _write_drone_csv(exp / "drone_1.csv", rows_l)
    _write_drone_csv(exp / "drone_2.csv", rows_f)

    max_dx, mean_dx, msg = compute_pursuit_x_gap_metrics(exp, tail_seconds=5.0)
    assert max_dx is not None and mean_dx is not None
    assert max_dx == pytest.approx(0.1, abs=1e-6)
    assert "n=" in msg


def test_compute_pursuit_x_gap_fails_when_gap_large(tmp_path: Path) -> None:
    exp = tmp_path / "exp"
    exp.mkdir()
    rows_l = [(float(t), 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0) for t in range(100, 160)]
    rows_f = [(float(t), 7.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0) for t in range(100, 160)]
    _write_drone_csv(exp / "drone_1.csv", rows_l)
    _write_drone_csv(exp / "drone_2.csv", rows_f)

    max_dx, _, _ = compute_pursuit_x_gap_metrics(exp, tail_seconds=5.0)
    assert max_dx is not None
    assert max_dx == pytest.approx(3.0, abs=1e-6)


def test_interpolates_leader_at_follower_times(tmp_path: Path) -> None:
    """Leader sampled sparsely; follower dense — interp should match."""
    exp = tmp_path / "exp"
    exp.mkdir()
    rows_l = [(100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0), (110.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)]
    rows_f = [(105.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)]
    _write_drone_csv(exp / "drone_1.csv", rows_l)
    _write_drone_csv(exp / "drone_2.csv", rows_f)

    max_dx, _, _ = compute_pursuit_x_gap_metrics(exp, tail_seconds=500.0)
    assert max_dx is not None
    # At t=105 leader x = 5, follower x = 4 -> |dx|=1
    assert max_dx == pytest.approx(1.0, abs=1e-6)
