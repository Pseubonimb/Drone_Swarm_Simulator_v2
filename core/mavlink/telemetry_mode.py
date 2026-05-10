"""Telemetry source selection for MAVLinkWorker (no pymavlink import)."""

import os

# ``sim_state`` (default): SIM_STATE + HOME_POSITION. ``local``: LOCAL_POSITION_NED + ATTITUDE
# (use for delay-comparison experiments vs EKF/estimator streams).
_TELEMETRY_MODE_ENV = "MAVLINK_TELEMETRY_MODE"


def telemetry_uses_sim_state() -> bool:
    """True if worker should use SIM_STATE (+ HOME) instead of LOCAL_POSITION_NED + ATTITUDE."""
    v = (os.environ.get(_TELEMETRY_MODE_ENV) or "sim_state").strip().lower()
    return v in ("sim_state", "sim", "simstate")
