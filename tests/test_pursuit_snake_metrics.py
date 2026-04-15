"""Tests for planar pursuit distance metrics (snake / chain experiments)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.experiment.csv_io import (  # noqa: E402
    chain_mean_distance_xy_series,
    euclidean_xy_distance,
    interpolate_scalar_on_times,
    leader_follower_distance_xy_series,
)
from core.network import default_local_to_common_ned  # noqa: E402


def test_euclidean_xy_distance_colocated() -> None:
    assert euclidean_xy_distance(0.0, 0.0, 0.0, 0.0) == pytest.approx(0.0)
    assert euclidean_xy_distance(1.0, -2.0, 1.0, -2.0) == pytest.approx(0.0)


def test_euclidean_xy_distance_3_4_5() -> None:
    assert euclidean_xy_distance(0.0, 0.0, 3.0, 4.0) == pytest.approx(5.0)


def test_euclidean_xy_distance_vectorized() -> None:
    x1 = np.array([0.0, 0.0])
    y1 = np.array([0.0, 0.0])
    x2 = np.array([3.0, 0.0])
    y2 = np.array([4.0, 5.0])
    d = euclidean_xy_distance(x1, y1, x2, y2)
    np.testing.assert_array_almost_equal(d, np.array([5.0, 5.0]))


def test_interpolate_scalar_on_times_empty_target() -> None:
    t_src = np.array([0.0, 1.0])
    v_src = np.array([10.0, 20.0])
    out = interpolate_scalar_on_times(np.array([]), t_src, v_src)
    assert len(out) == 0


def test_interpolate_scalar_on_times_insufficient_source() -> None:
    t_target = np.array([0.0, 0.5, 1.0])
    out = interpolate_scalar_on_times(t_target, np.array([0.0]), np.array([1.0]))
    np.testing.assert_array_equal(out, np.zeros(3))


def test_interpolate_scalar_on_times_linear() -> None:
    t_target = np.array([0.0, 0.5, 1.0])
    t_src = np.array([0.0, 1.0])
    v_src = np.array([0.0, 10.0])
    out = interpolate_scalar_on_times(t_target, t_src, v_src)
    np.testing.assert_array_almost_equal(out, np.array([0.0, 5.0, 10.0]))


def test_leader_follower_distance_xy_series_basic() -> None:
    # Leader: (t,x,y); follower same times, offset in y
    leader = np.array(
        [[0.0, 0.0, 0.0, 0, 0, 0, 0, 0], [1.0, 0.0, 0.0, 0, 0, 0, 0, 0]],
        dtype=float,
    )
    follower = np.array(
        [[0.0, 0.0, 3.0, 0, 0, 0, 0, 0], [1.0, 0.0, 4.0, 0, 0, 0, 0, 0]],
        dtype=float,
    )
    t, d = leader_follower_distance_xy_series(leader, follower)
    np.testing.assert_array_equal(t, np.array([0.0, 1.0]))
    np.testing.assert_array_almost_equal(d, np.array([3.0, 4.0]))


def test_leader_follower_distance_xy_matches_abs_dx_when_dy_zero() -> None:
    leader = np.array([[0.0, 10.0, 0.0, 0, 0, 0, 0, 0]], dtype=float)
    follower = np.array(
        [[0.0, 7.0, 0.0, 0, 0, 0, 0, 0], [1.0, 8.0, 0.0, 0, 0, 0, 0, 0]],
        dtype=float,
    )
    _, d = leader_follower_distance_xy_series(leader, follower)
    assert d[0] == pytest.approx(3.0)


def test_leader_follower_distance_xy_empty_leader() -> None:
    leader = np.zeros((0, 8))
    follower = np.array([[0.0, 0.0, 0.0, 0, 0, 0, 0, 0]], dtype=float)
    t, d = leader_follower_distance_xy_series(leader, follower)
    assert len(t) == 0 and len(d) == 0


def test_leader_follower_distance_xy_interpolates_follower_on_leader_times() -> None:
    """Sparse leader timeline; follower dense — distances at leader samples."""
    leader = np.array(
        [[100.0, 0.0, 0.0, 0, 0, 0, 0, 0], [110.0, 10.0, 0.0, 0, 0, 0, 0, 0]],
        dtype=float,
    )
    follower = np.array(
        [
            [100.0, 0.0, 0.0, 0, 0, 0, 0, 0],
            [105.0, 5.0, 0.0, 0, 0, 0, 0, 0],
            [110.0, 9.0, 0.0, 0, 0, 0, 0, 0],
        ],
        dtype=float,
    )
    t, d = leader_follower_distance_xy_series(leader, follower)
    assert len(t) == 2
    assert d[0] == pytest.approx(0.0)
    # At t=110: leader (10,0), follower x interp 9 -> distance 1
    assert d[1] == pytest.approx(1.0)


def test_chain_two_drones_matches_pair_series() -> None:
    d1 = np.array([[0.0, 0.0, 0.0, 0, 0, 0, 0, 0], [1.0, 3.0, 0.0, 0, 0, 0, 0, 0]], float)
    d2 = np.array([[0.0, 0.0, 4.0, 0, 0, 0, 0, 0], [1.0, 0.0, 0.0, 0, 0, 0, 0, 0]], float)
    t_p, dist_p = leader_follower_distance_xy_series(d1, d2)
    t_c, dist_c = chain_mean_distance_xy_series([d1, d2])
    np.testing.assert_array_equal(t_p, t_c)
    np.testing.assert_array_almost_equal(dist_p, dist_c)


def test_chain_three_drones_mean_of_two_links() -> None:
    # Same time base; positions chosen so link1 dist = 1, link2 dist = 3 -> mean 2
    # Two samples each so interpolation onto leader times is defined (>=2 source rows).
    z = np.zeros(8)
    a = np.array([[0.0, 0.0, 0.0, *z[3:]], [1.0, 0.0, 0.0, *z[3:]]], dtype=float)
    b = np.array([[0.0, 1.0, 0.0, *z[3:]], [1.0, 1.0, 0.0, *z[3:]]], dtype=float)
    c = np.array([[0.0, 1.0, 3.0, *z[3:]], [1.0, 1.0, 3.0, *z[3:]]], dtype=float)
    _, mean_d = chain_mean_distance_xy_series([a, b, c])
    assert mean_d[0] == pytest.approx(2.0)
    assert mean_d[1] == pytest.approx(2.0)


def test_chain_four_drones_three_links_mean() -> None:
    tail = [0.0, 0.0, 0.0, 0, 0, 0, 0, 0]

    def two_rows(xya: tuple[float, float], xyb: tuple[float, float]) -> np.ndarray:
        return np.array(
            [
                [0.0, xya[0], xya[1], *tail[3:]],
                [1.0, xyb[0], xyb[1], *tail[3:]],
            ],
            dtype=float,
        )

    d1 = two_rows((0.0, 0.0), (0.0, 0.0))
    d2 = two_rows((3.0, 0.0), (3.0, 0.0))  # link d1–d2: 3
    d3 = two_rows((3.0, 4.0), (3.0, 4.0))  # link d2–d3: 4
    d4 = two_rows((3.0, -3.0), (3.0, -3.0))  # link d3–d4: 7
    _, mean_d = chain_mean_distance_xy_series([d1, d2, d3, d4])
    expected = (3.0 + 4.0 + 7.0) / 3.0
    assert mean_d[0] == pytest.approx(expected)
    assert mean_d[1] == pytest.approx(expected)


def test_chain_empty_or_short_returns_empty() -> None:
    a = np.array([[0.0, 0.0, 0.0, 0, 0, 0, 0, 0]], float)
    t, d = chain_mean_distance_xy_series([])
    assert len(t) == 0 and len(d) == 0
    t, d = chain_mean_distance_xy_series([a])
    assert len(t) == 0 and len(d) == 0
    t, d = chain_mean_distance_xy_series([a, np.zeros((0, 8))])
    assert len(t) == 0 and len(d) == 0


def test_common_ned_xy_distance_matches_manual_y_shift() -> None:
    """After common-NED y shift, planar distance equals hand calculation."""
    raw_a = {"x": 0.0, "y": 10.0, "z": 0.0}
    raw_b = {"x": 0.0, "y": 10.0, "z": 0.0}
    east_spacing = 2.0
    ca = default_local_to_common_ned(1, raw_a, east_spacing_m=east_spacing)
    cb = default_local_to_common_ned(2, raw_b, east_spacing_m=east_spacing)
    d = float(euclidean_xy_distance(ca["x"], ca["y"], cb["x"], cb["y"]))
    # Drone 2 local y same as drone 1; common y_b = 10 - 2 = 8, y_a = 10 -> |dy|=2
    assert d == pytest.approx(2.0)
