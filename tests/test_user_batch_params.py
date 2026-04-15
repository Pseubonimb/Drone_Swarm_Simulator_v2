"""Unit tests for core.batch.user_batch_params (YAML batch parameter sweeps)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.batch.user_batch_params import (  # noqa: E402
    generate_parameter_combinations,
    linear_range_inclusive,
    load_user_batch_yaml,
)


def test_linear_range_inclusive_standard_grid() -> None:
    assert linear_range_inclusive(0.5, 2.0, 0.5) == [0.5, 1.0, 1.5, 2.0]


def test_linear_range_inclusive_rejects_bad_stop() -> None:
    with pytest.raises(ValueError, match="not on the grid"):
        linear_range_inclusive(0.1, 1.05, 0.1)


def test_generate_single_axis_combinations() -> None:
    doc = yaml.safe_load("""
experiment_settings:
  iterations: 1
  target_parameter: "pid.p_gain"
  range:
    start: 0.5
    stop: 2.0
    step: 0.5
""")
    combos = generate_parameter_combinations(doc)
    assert combos == [
        {"pid.p_gain": 0.5},
        {"pid.p_gain": 1.0},
        {"pid.p_gain": 1.5},
        {"pid.p_gain": 2.0},
    ]


def test_generate_combinations_accepts_end_alias() -> None:
    doc = yaml.safe_load("""
experiment_settings:
  target_parameter: "pid.p_gain"
  range:
    start: 0.1
    end: 0.3
    step: 0.1
""")
    assert generate_parameter_combinations(doc) == [
        {"pid.p_gain": 0.1},
        {"pid.p_gain": 0.2},
        {"pid.p_gain": 0.3},
    ]


def test_generate_cartesian_product_two_axes() -> None:
    doc = yaml.safe_load("""
parameter_sweeps:
  - target_parameter: "pid.p_gain"
    range: { start: 1.0, stop: 2.0, step: 1.0 }
  - target_parameter: "pid.i_gain"
    range: { start: 0.0, stop: 0.1, step: 0.1 }
experiment_settings:
  iterations: 1
""")
    combos = generate_parameter_combinations(doc)
    assert len(combos) == 4
    assert {"pid.p_gain": 1.0, "pid.i_gain": 0.0} in combos
    assert {"pid.p_gain": 1.0, "pid.i_gain": 0.1} in combos
    assert {"pid.p_gain": 2.0, "pid.i_gain": 0.0} in combos
    assert {"pid.p_gain": 2.0, "pid.i_gain": 0.1} in combos


def test_iterations_repeats_each_combo() -> None:
    doc = yaml.safe_load("""
experiment_settings:
  iterations: 3
  target_parameter: "pid.p_gain"
  range:
    start: 1.0
    stop: 1.0
    step: 1.0
""")
    combos = generate_parameter_combinations(doc)
    assert combos == [{"pid.p_gain": 1.0}] * 3


def test_example_user_batch_params_file() -> None:
    path = _ROOT / "scenarios" / "batch_parameters" / "user_batch_params.yaml"
    doc = load_user_batch_yaml(path)
    combos = generate_parameter_combinations(doc)
    settings = doc["experiment_settings"]
    rng = settings["range"]
    stop = float(rng["stop"]) if "stop" in rng else float(rng["end"])
    values = linear_range_inclusive(float(rng["start"]), stop, float(rng["step"]))
    iterations = int(settings.get("iterations", 1))
    expected = [{"pid.p_gain": v} for v in values for _ in range(iterations)]
    assert combos == expected


def test_load_missing_file() -> None:
    with pytest.raises(FileNotFoundError):
        load_user_batch_yaml(
            _ROOT / "scenarios" / "batch_parameters" / "nonexistent.yaml"
        )
