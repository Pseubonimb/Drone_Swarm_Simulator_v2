"""Smoke tests for plotter batch discovery (no full matplotlib UI)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from plotter.plotter import (  # noqa: E402
    chain_mean_distance_xy_resampled,
    discover_batch_run_dirs_latest_session,
    discover_run_dirs,
    infer_experiment_session_stamp_for_outputs,
    latest_batch_session_dir,
    leader_follower_distance_xy_resampled,
    leader_follower_dx_resampled,
    load_experiment_dir_drone_chain,
    load_experiment_dir_pair_csvs,
    load_run_dx,
    plot_time_overlay,
    sequential_drone_csv_paths,
)


def _write_minimal_csv(path: Path, rows: list[tuple[float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    hdr = "t,x,y,z,rx,ry,rz,hasCollision\n"
    lines = hdr + "".join(f"{t},{x},0,0,0,0,0,0\n" for t, x in rows)
    path.write_text(lines, encoding="utf-8")


def test_discover_and_load_run_dx(tmp_path: Path) -> None:
    run = tmp_path / "experiments" / "batch_test_run_1"
    run.mkdir(parents=True)
    (run / "batch_run.json").write_text(
        json.dumps(
            {
                "run_index": 1,
                "scenario_id": "leader_forward_back",
                "params": {"pid.p_gain": 2.0},
            }
        ),
        encoding="utf-8",
    )
    _write_minimal_csv(run / "drone_1.csv", [(0.0, 10.0), (1.0, 12.0)])
    _write_minimal_csv(run / "drone_2.csv", [(0.0, 9.0), (1.0, 11.0)])
    found = discover_run_dirs(tmp_path, "experiments/batch_*_run_*")
    assert len(found) == 1
    meta, t, dx = load_run_dx(found[0], resample_points=0)
    assert meta is not None and t is not None and dx is not None
    assert len(dx) == 2


def test_discover_latest_session_single(tmp_path: Path) -> None:
    """Default discovery uses the only YYYY-MM-DD_HH-MM-SS session folder."""
    run = (
        tmp_path / "experiments" / "2026-04-08_22-01-21" / "batch_nested_run_1"
    )
    run.mkdir(parents=True)
    (run / "batch_run.json").write_text(
        json.dumps({"run_index": 1, "params": {}}),
        encoding="utf-8",
    )
    _write_minimal_csv(run / "drone_1.csv", [(0.0, 1.0), (1.0, 1.0)])
    _write_minimal_csv(run / "drone_2.csv", [(0.0, 0.0), (1.0, 0.0)])
    found, pattern = discover_batch_run_dirs_latest_session(tmp_path)
    assert pattern == "experiments/2026-04-08_22-01-21/batch_*_run_*"
    assert len(found) == 1
    assert found[0] == run
    assert latest_batch_session_dir(tmp_path).name == "2026-04-08_22-01-21"


def test_discover_latest_session_prefers_lexicographically_newer_stamp(tmp_path: Path) -> None:
    """Two session folders → runs from the newer timestamp only (not yy-2-digit dirs)."""
    old_run = (
        tmp_path / "experiments" / "2026-04-07_12-00-00" / "batch_old_run_1"
    )
    new_run = (
        tmp_path / "experiments" / "2026-04-08_22-01-21" / "batch_new_run_1"
    )
    for run in (old_run, new_run):
        run.mkdir(parents=True)
        (run / "batch_run.json").write_text(
            json.dumps({"run_index": 1, "params": {}}),
            encoding="utf-8",
        )
        _write_minimal_csv(run / "drone_1.csv", [(0.0, 1.0), (1.0, 1.0)])
        _write_minimal_csv(run / "drone_2.csv", [(0.0, 0.0), (1.0, 0.0)])
    # Legacy 2-digit year folder must be ignored for "latest session"
    legacy = tmp_path / "experiments" / "26-04-09_23-00-00"
    legacy.mkdir(parents=True)
    found, _ = discover_batch_run_dirs_latest_session(tmp_path)
    assert len(found) == 1
    assert found[0] == new_run


def test_latest_batch_session_dir_ignores_non_matching_names(tmp_path: Path) -> None:
    exp = tmp_path / "experiments"
    exp.mkdir(parents=True)
    (exp / "_batch_parms").mkdir()
    (exp / "exp_1").mkdir()
    assert latest_batch_session_dir(tmp_path) is None


def test_infer_experiment_session_stamp_single_session(tmp_path: Path) -> None:
    run = (
        tmp_path / "experiments" / "2026-04-08_22-01-21" / "batch_x_run_1"
    )
    run.mkdir(parents=True)
    assert infer_experiment_session_stamp_for_outputs([run]) == "2026-04-08_22-01-21"


def test_infer_experiment_session_stamp_flat_experiments(tmp_path: Path) -> None:
    run = tmp_path / "experiments" / "batch_x_run_1"
    run.mkdir(parents=True)
    assert infer_experiment_session_stamp_for_outputs([run]) is None


def test_infer_experiment_session_stamp_mixed_sessions_returns_none(tmp_path: Path) -> None:
    a = tmp_path / "experiments" / "2026-04-07_12-00-00" / "batch_a_run_1"
    b = tmp_path / "experiments" / "2026-04-08_22-01-21" / "batch_b_run_1"
    a.mkdir(parents=True)
    b.mkdir(parents=True)
    assert infer_experiment_session_stamp_for_outputs([a, b]) is None


def test_load_run_dx_resampled_smooths_sparse(tmp_path: Path) -> None:
    """Sparse CSV rows → many points on uniform grid (no tiny leader-only grid)."""
    run = tmp_path / "experiments" / "batch_sparse_run_1"
    run.mkdir(parents=True)
    (run / "batch_run.json").write_text(
        json.dumps({"run_index": 1, "params": {}}),
        encoding="utf-8",
    )
    # Wide gaps; only two samples each — raw leader-follower series has len 2.
    _write_minimal_csv(
        run / "drone_1.csv",
        [(0.0, 10.0), (10.0, 10.0)],
    )
    _write_minimal_csv(
        run / "drone_2.csv",
        [(0.0, 9.0), (10.0, 9.5)],
    )
    meta, t, dx = load_run_dx(run, resample_points=400)
    assert meta is not None and t is not None and dx is not None
    assert len(t) == len(dx) == 400
    assert float(dx[0]) == pytest.approx(-1.0)
    assert float(dx[-1]) == pytest.approx(-0.5)


def test_leader_follower_distance_xy_resampled_right_triangle() -> None:
    """(0,0) vs (3,4) → distance 5 at all times."""
    tail = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    d1 = np.array(
        [[0.0, 0.0, 0.0, *tail], [1.0, 0.0, 0.0, *tail]], dtype=float
    )
    d2 = np.array(
        [[0.0, 3.0, 4.0, *tail], [1.0, 3.0, 4.0, *tail]], dtype=float
    )
    t, d = leader_follower_distance_xy_resampled(d1, d2, num_points=100)
    assert len(t) == 100
    np.testing.assert_array_almost_equal(d, np.full(100, 5.0))


def test_leader_follower_dx_resampled_overlap() -> None:
    d1 = np.array(
        [[0.0, 10.0, 0, 0, 0, 0, 0, 0], [1.0, 10.0, 0, 0, 0, 0, 0, 0]], dtype=float
    )
    d2 = np.array(
        [[0.0, 9.0, 0, 0, 0, 0, 0, 0], [1.0, 9.0, 0, 0, 0, 0, 0, 0]], dtype=float
    )
    t, dx = leader_follower_dx_resampled(d1, d2, num_points=50)
    assert len(t) == 50
    np.testing.assert_array_almost_equal(dx, np.full(50, -1.0))


def test_load_single_run_metadata_no_batch_json(tmp_path: Path) -> None:
    """Standalone experiment: metadata.json + drone CSVs (folder like experiments/<stamp>/)."""
    exp = tmp_path / "experiments" / "2026-04-19_15-59-36"
    exp.mkdir(parents=True)
    (exp / "metadata.json").write_text(
        json.dumps(
            {
                "duration_sec": 120.0,
                "collision_radius_m": 0.5,
                "num_drones": 2,
                "scenario": "snake_pursuit",
            }
        ),
        encoding="utf-8",
    )
    _write_minimal_csv(exp / "drone_1.csv", [(0.0, 1.0), (1.0, 1.0)])
    _write_minimal_csv(exp / "drone_2.csv", [(0.0, 0.0), (1.0, 0.0)])
    meta, d1, d2 = load_experiment_dir_pair_csvs(exp)
    assert meta is not None and d1 is not None and d2 is not None
    assert meta.get("params", {}).get("scenario") == "snake_pursuit"
    assert infer_experiment_session_stamp_for_outputs([exp]) == "2026-04-19_15-59-36"


def _write_xy_csv(path: Path, rows: list[tuple[float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    hdr = "t,x,y,z,rx,ry,rz,hasCollision\n"
    body = "".join(f"{t},{x},{y},0,0,0,0,0\n" for t, x, y in rows)
    path.write_text(hdr + body, encoding="utf-8")


def test_chain_mean_distance_xy_resampled_three_drones_colinear() -> None:
    """Three drones on x-axis: links 3 m and 3 m → mean 3 m."""
    tail = [0.0, 0.0, 0.0, 0.0, 0.0]  # z, rx, ry, rz, hasCollision
    d1 = np.array([[0.0, 0.0, 0.0, *tail], [1.0, 0.0, 0.0, *tail]], dtype=float)
    d2 = np.array([[0.0, 3.0, 0.0, *tail], [1.0, 3.0, 0.0, *tail]], dtype=float)
    d3 = np.array([[0.0, 6.0, 0.0, *tail], [1.0, 6.0, 0.0, *tail]], dtype=float)
    t, d = chain_mean_distance_xy_resampled([d1, d2, d3], num_points=50)
    assert len(t) == 50
    np.testing.assert_array_almost_equal(d, np.full(50, 3.0))


def test_load_experiment_dir_drone_chain_three_csvs(tmp_path: Path) -> None:
    exp = tmp_path / "exp_chain"
    exp.mkdir(parents=True)
    (exp / "metadata.json").write_text(
        json.dumps(
            {
                "duration_sec": 1.0,
                "collision_radius_m": 0.5,
                "num_drones": 3,
                "scenario": "snake_pursuit",
            }
        ),
        encoding="utf-8",
    )
    _write_xy_csv(exp / "drone_1.csv", [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    _write_xy_csv(exp / "drone_2.csv", [(0.0, 3.0, 0.0), (1.0, 3.0, 0.0)])
    _write_xy_csv(exp / "drone_3.csv", [(0.0, 6.0, 0.0), (1.0, 6.0, 0.0)])
    assert len(sequential_drone_csv_paths(exp)) == 3
    meta, drones = load_experiment_dir_drone_chain(exp)
    assert meta is not None and drones is not None and len(drones) == 3


def test_load_experiment_dir_prefers_batch_run_json(tmp_path: Path) -> None:
    """If both would apply, batch_run.json wins (one batch run subfolder via --experiment-dir)."""
    run = tmp_path / "experiments" / "2026-01-01_12-00-00" / "batch_x_run_1"
    run.mkdir(parents=True)
    (run / "batch_run.json").write_text(
        json.dumps({"run_index": 1, "params": {"pid.p_gain": 3.0}}),
        encoding="utf-8",
    )
    (run / "metadata.json").write_text(
        json.dumps(
            {
                "duration_sec": 1.0,
                "collision_radius_m": 0.5,
                "num_drones": 2,
                "scenario": "ignored_when_batch",
            }
        ),
        encoding="utf-8",
    )
    _write_minimal_csv(run / "drone_1.csv", [(0.0, 1.0), (1.0, 1.0)])
    _write_minimal_csv(run / "drone_2.csv", [(0.0, 0.0), (1.0, 0.0)])
    meta, d1, d2 = load_experiment_dir_pair_csvs(run)
    assert meta is not None and d1 is not None and d2 is not None
    assert meta["params"]["pid.p_gain"] == 3.0


def test_plot_time_overlay_writes_file(tmp_path: Path) -> None:
    run = tmp_path / "batch_x_run_1"
    run.mkdir(parents=True)
    (run / "batch_run.json").write_text(
        json.dumps({"run_index": 1, "params": {}}),
        encoding="utf-8",
    )
    _write_minimal_csv(run / "drone_1.csv", [(0.0, 0.0), (1.0, 1.0)])
    _write_minimal_csv(run / "drone_2.csv", [(0.0, 0.0), (1.0, 0.5)])
    t = np.array([0.0, 1.0])
    dx = np.array([0.0, -0.5])
    meta = {"params": {"pid.p_gain": 1.0}}
    out = tmp_path / "fig.png"
    plot_time_overlay([(run, meta, t, dx)], out, title="t", ylabel="Δx (m)")
    assert out.is_file()
