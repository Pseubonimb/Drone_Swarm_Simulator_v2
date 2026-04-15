"""Map logical batch parameter names (YAML target_parameter) to scenario CLI flags."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Mapping, MutableMapping, Tuple

# Logical keys from batch YAML -> scenario script argv (--kp, --ki, ...).
SCENARIO_PID_CLI_FLAGS: Dict[str, Dict[str, str]] = {
    "leader_forward_back": {
        "pid.p_gain": "--kp",
        "pid.i_gain": "--ki",
        "pid.d_gain": "--kd",
        "pid.ki_pitch": "--ki-pitch",
        "pid.derivative_alpha": "--derivative-alpha",
    },
    "snake_pursuit": {
        "pid.p_gain": "--kp",
        "pid.i_gain": "--ki",
        "pid.d_gain": "--kd",
        "pid.ki_pitch": "--ki-pitch",
        "pid.derivative_alpha": "--derivative-alpha",
    },
}


def effective_launch_block(doc: Mapping[str, Any]) -> Mapping[str, Any]:
    """Merge optional ``batch:`` and ``launch:``; ``launch`` keys override ``batch``."""
    merged: Dict[str, Any] = {}
    batch = doc.get("batch")
    launch = doc.get("launch")
    if isinstance(batch, MutableMapping):
        merged.update(batch)
    if isinstance(launch, MutableMapping):
        merged.update(launch)
    return merged


def batch_params_to_scenario_argv(
    scenario_id: str, params: Mapping[str, float]
) -> List[str]:
    """Convert a parameter combination dict into extra argv tokens for the scenario script.

    Args:
        scenario_id: Scenario registry id (e.g. ``leader_forward_back``).
        params: Maps ``target_parameter`` string from YAML to numeric value.

    Returns:
        Flat list ``[flag, str(value), ...]`` to append after base scenario command.

    Raises:
        ValueError: Unknown scenario or unknown ``target_parameter`` key for that scenario.
    """
    if not params:
        return []
    table = SCENARIO_PID_CLI_FLAGS.get(scenario_id)
    if table is None:
        raise ValueError(
            f"Scenario {scenario_id!r} has no PID batch mapping; supported scenarios: "
            f"{sorted(SCENARIO_PID_CLI_FLAGS)}"
        )
    out: List[str] = []
    for key, value in params.items():
        flag = table.get(key)
        if flag is None:
            raise ValueError(
                f"Unknown target_parameter {key!r} for scenario {scenario_id!r}. "
                f"Known keys: {sorted(table)}"
            )
        out.extend([flag, str(float(value))])
    return out


def merge_batch_launch(
    doc: Mapping[str, Any],
    args: Any,
) -> Tuple[str, int, float]:
    """Resolve scenario id, drone count, and duration for a batch YAML run.

    Precedence: ``launch:`` values from YAML when the key is present; otherwise
    attributes from ``args`` (``scenario``, ``drones``, ``duration``).

    Args:
        doc: Parsed batch YAML root mapping.
        args: ``argparse.Namespace`` from the launcher.

    Returns:
        ``(scenario_id, num_drones, duration_sec)``.

    Raises:
        ValueError: Missing scenario id or invalid values.
    """
    launch = effective_launch_block(doc)
    if doc.get("launch") is not None and not isinstance(doc.get("launch"), Mapping):
        raise ValueError("launch must be a mapping when present")
    if doc.get("batch") is not None and not isinstance(doc.get("batch"), Mapping):
        raise ValueError("batch must be a mapping when present")

    raw_scenario = getattr(args, "scenario", None)
    scenario_id = raw_scenario if raw_scenario else launch.get("scenario")
    if not scenario_id or not isinstance(scenario_id, str):
        raise ValueError(
            "Batch YAML must set launch.scenario or pass -c/--scenario on the command line"
        )
    scenario_id = scenario_id.strip()

    if "drones" in launch:
        num_drones = int(launch["drones"])
    else:
        num_drones = int(getattr(args, "drones", 2))

    if "duration_sec" in launch:
        duration = float(launch["duration_sec"])
    else:
        duration = float(getattr(args, "duration", 0.0))

    if num_drones < 1:
        raise ValueError("drones must be >= 1")
    return scenario_id, num_drones, duration


def batch_experiment_label(doc: Mapping[str, Any], yaml_path: str) -> str:
    """Folder name token for ``batch_<label>_run_<n>`` (under ``experiments/<yyyy-mm-dd_hh-mm-ss>/``)."""
    launch = effective_launch_block(doc)
    bid = launch.get("batch_id")
    if isinstance(bid, str) and bid.strip():
        return bid.strip()
    safe = Path(yaml_path).stem
    return safe or "batch"
