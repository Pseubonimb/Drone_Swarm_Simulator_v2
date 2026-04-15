"""CoordExchangeManager: CSV row cadence when pose is static (log_hz=0)."""

import os
import tempfile
import time

from core.logging.csv_logger import CSV_HEADER
from core.network import CoordExchangeManager


def _mock_controller(did: int):
    m = type("C", (), {})()
    m.config = {"id": did}
    pos = {"x": 1.0, "y": 2.0, "z": -0.5, "vx": 0.0, "vy": 0.0, "vz": 0.0}
    att = {"rx": 0.0, "ry": 0.0, "rz": 0.0}
    m.get_my_position = lambda: dict(pos)
    m.get_attitude = lambda: dict(att)
    m.update_other_drone_position = lambda *a, **k: None
    return m


def test_csv_logs_each_exchange_step_with_static_pose() -> None:
    """With log_hz=0, rows must still be written every tick (not only on pose change)."""
    tdir = tempfile.mkdtemp()
    f1 = open(os.path.join(tdir, "d1.csv"), "w", encoding="utf-8")
    f1.write(CSV_HEADER + "\n")
    f2 = open(os.path.join(tdir, "d2.csv"), "w", encoding="utf-8")
    f2.write(CSV_HEADER + "\n")
    files = {1: f1, 2: f2}
    ctrls = [_mock_controller(1), _mock_controller(2)]
    mgr = CoordExchangeManager(
        ctrls,
        experiment_start_time=time.time(),
        experiment_log_files=files,
        duration=0.2,
        collision_radius=0.2,
        log_hz=0.0,
        exchange_loop_hz=200.0,
        experiment_dir=None,
        update_neighbors=False,
        publish_visualizer=False,
        publish_measured_exchange_hz=False,
        daemon=True,
    )
    mgr.start()
    time.sleep(0.3)
    mgr.stop()
    f1.close()
    f2.close()
    with open(os.path.join(tdir, "d1.csv"), encoding="utf-8") as f:
        n = len(f.readlines()) - 1
    assert n > 15, f"expected many CSV data rows with fixed pose, got {n}"
