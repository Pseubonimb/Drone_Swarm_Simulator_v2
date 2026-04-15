"""Load and expand user batch parameter YAML (PID sweeps, Cartesian product)."""

from __future__ import annotations

import itertools
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, MutableMapping

import yaml

YamlDict = MutableMapping[str, Any]


def linear_range_inclusive(start: float, stop: float, step: float) -> List[float]:
    """Return ``start, start+step, ...`` up to ``stop`` inclusive.

    ``stop`` must lie on the grid defined by ``start`` and ``step`` (within float tolerance).

    Args:
        start: First value.
        stop: Last value (inclusive).
        step: Non-zero increment; sign must move from start toward stop.

    Returns:
        List of sweep values.

    Raises:
        ValueError: If step is zero, unreachable stop, or grid mismatch.
    """
    if step == 0.0:
        raise ValueError("range step must be non-zero")
    if (stop - start) * step < 0:
        raise ValueError(
            "stop is not reachable from start with the given step direction"
        )
    n_steps = round((stop - start) / step)
    last = start + n_steps * step
    if not math.isclose(last, stop, rel_tol=1e-9, abs_tol=1e-9):
        raise ValueError(
            f"stop {stop} is not on the grid from {start} with step {step} "
            f"(last grid point would be {last})"
        )
    # Mitigate binary float drift (e.g. 0.1*3 != 0.3) for stable YAML round-trips and tests.
    return [round(start + i * step, 12) for i in range(n_steps + 1)]


def _range_stop(raw: Mapping[str, Any]) -> float:
    if "stop" in raw and "end" in raw and raw["stop"] != raw["end"]:
        raise ValueError(
            "range must not specify both 'stop' and 'end' with different values"
        )
    if "stop" in raw:
        return float(raw["stop"])
    if "end" in raw:
        return float(raw["end"])
    raise ValueError("range requires 'stop' or 'end'")


def _sweep_values(range_spec: Mapping[str, Any]) -> List[float]:
    start = float(range_spec["start"])
    step = float(range_spec["step"])
    stop = _range_stop(range_spec)
    return linear_range_inclusive(start, stop, step)


def _collect_sweep_axes(doc: Mapping[str, Any]) -> List[Dict[str, Any]]:
    sweeps = doc.get("parameter_sweeps")
    if sweeps is not None:
        if not isinstance(sweeps, list) or not sweeps:
            raise ValueError("parameter_sweeps must be a non-empty list when present")
        return list(sweeps)
    settings = doc.get("experiment_settings")
    if not isinstance(settings, Mapping):
        raise ValueError("missing experiment_settings or parameter_sweeps")
    if "target_parameter" not in settings or "range" not in settings:
        raise ValueError("experiment_settings must include target_parameter and range")
    return [
        {"target_parameter": settings["target_parameter"], "range": settings["range"]}
    ]


def _iterations_from_doc(doc: Mapping[str, Any]) -> int:
    settings = doc.get("experiment_settings")
    if isinstance(settings, Mapping) and "iterations" in settings:
        n = int(settings["iterations"])
        if n < 1:
            raise ValueError("experiment_settings.iterations must be >= 1")
        return n
    return 1


def generate_parameter_combinations(doc: Mapping[str, Any]) -> List[Dict[str, float]]:
    """Build list of parameter dicts from parsed batch YAML (Cartesian product of sweeps).

    Each sweep axis is ``target_parameter`` + numeric ``range`` (``start``, ``stop`` or ``end``,
    ``step``). If ``parameter_sweeps`` is present, it defines all axes; otherwise a single axis
    is read from ``experiment_settings``.

    ``experiment_settings.iterations`` repeats each combination that many times (identical dicts).

    Args:
        doc: Top-level mapping loaded from YAML (e.g. via ``load_user_batch_yaml``).

    Returns:
        List of dicts mapping parameter path string to float value, length =
        (product of axis lengths) * iterations.
    """
    axes = _collect_sweep_axes(doc)
    iterations = _iterations_from_doc(doc)
    value_lists: List[List[float]] = []
    keys: List[str] = []
    for axis in axes:
        if not isinstance(axis, Mapping):
            raise ValueError("each parameter_sweeps entry must be a mapping")
        key = axis.get("target_parameter")
        if not isinstance(key, str) or not key.strip():
            raise ValueError("each sweep needs non-empty target_parameter string")
        rng = axis.get("range")
        if not isinstance(rng, Mapping):
            raise ValueError(f"range for {key!r} must be a mapping")
        keys.append(key.strip())
        value_lists.append(_sweep_values(rng))

    combos: List[Dict[str, float]] = []
    for tup in itertools.product(*value_lists):
        combos.append(dict(zip(keys, tup, strict=True)))
    out: List[Dict[str, float]] = []
    for _ in range(iterations):
        out.extend(dict(c) for c in combos)
    return out


def load_user_batch_yaml(path: str | Path) -> YamlDict:
    """Load batch YAML from disk.

    Args:
        path: File path (e.g. ``scenarios/batch_parameters/user_batch_params.yaml``).

    Returns:
        Parsed top-level mapping.

    Raises:
        FileNotFoundError: If path does not exist.
        yaml.YAMLError: On invalid YAML.
    """
    p = Path(path)
    with p.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    if not isinstance(data, MutableMapping):
        raise ValueError("batch YAML root must be a mapping")
    return data
