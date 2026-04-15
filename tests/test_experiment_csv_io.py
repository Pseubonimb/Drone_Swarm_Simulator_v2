"""Tests for core.experiment.csv_io."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.experiment.csv_io import leader_follower_dx_series, rms_tail  # noqa: E402


def test_leader_follower_dx_series() -> None:
    d1 = np.array(
        [[0.0, 10.0, 0, 0, 0, 0, 0, 0], [1.0, 11.0, 0, 0, 0, 0, 0, 0]], dtype=float
    )
    d2 = np.array(
        [[0.0, 9.0, 0, 0, 0, 0, 0, 0], [1.0, 10.5, 0, 0, 0, 0, 0, 0]], dtype=float
    )
    t, dx = leader_follower_dx_series(d1, d2)
    np.testing.assert_array_almost_equal(dx, np.array([-1.0, -0.5]))


def test_rms_tail() -> None:
    t = np.array([0.0, 5.0, 9.0, 10.0])
    v = np.array([2.0, 2.0, 0.0, 0.0])
    r = rms_tail(v, t, tail_sec=2.0)
    assert r == pytest.approx(np.sqrt((0.0 + 0.0) / 2))
