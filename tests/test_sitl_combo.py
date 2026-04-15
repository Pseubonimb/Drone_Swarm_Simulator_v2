"""Tests for core.batch.sitl_combo."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.batch.sitl_combo import partition_combo_for_batch, write_sitl_overlay_parm  # noqa: E402


def test_partition_combo() -> None:
    sc, si = partition_combo_for_batch(
        {"pid.p_gain": 1.0, "sitl.ANGLE_MAX": 4500.0},
    )
    assert sc == {"pid.p_gain": 1.0}
    assert si == {"sitl.ANGLE_MAX": 4500.0}


def test_write_sitl_overlay_angle_max(tmp_path: Path) -> None:
    p = tmp_path / "o.parm"
    write_sitl_overlay_parm(str(p), {"sitl.ANGLE_MAX": 4500.4})
    assert p.read_text(encoding="utf-8").strip() == "ANGLE_MAX 4500"


def test_write_sitl_unknown_key(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="Unsupported"):
        write_sitl_overlay_parm(str(tmp_path / "x.parm"), {"sitl.UNKNOWN": 1.0})
