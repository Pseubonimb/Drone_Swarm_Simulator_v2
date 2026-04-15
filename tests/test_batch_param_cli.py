"""Tests for batch YAML -> scenario CLI mapping (core.batch.param_cli)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pytest
import yaml

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.batch.param_cli import (  # noqa: E402
    batch_experiment_label,
    batch_params_to_scenario_argv,
    effective_launch_block,
    merge_batch_launch,
)


def test_batch_params_to_scenario_argv_leader() -> None:
    argv = batch_params_to_scenario_argv(
        "leader_forward_back",
        {"pid.p_gain": 1.5, "pid.i_gain": 0.02},
    )
    assert argv == ["--kp", "1.5", "--ki", "0.02"]


def test_batch_params_derivative_alpha_argv() -> None:
    argv = batch_params_to_scenario_argv(
        "leader_forward_back",
        {"pid.derivative_alpha": 0.25},
    )
    assert argv == ["--derivative-alpha", "0.25"]


def test_batch_params_to_scenario_argv_snake_pursuit() -> None:
    argv = batch_params_to_scenario_argv(
        "snake_pursuit",
        {"pid.p_gain": 8.0},
    )
    assert argv == ["--kp", "8.0"]


def test_batch_params_unknown_key() -> None:
    with pytest.raises(ValueError, match="Unknown target_parameter"):
        batch_params_to_scenario_argv("leader_forward_back", {"pid.unknown": 1.0})


def test_batch_params_unsupported_scenario() -> None:
    with pytest.raises(ValueError, match="no PID batch mapping"):
        batch_params_to_scenario_argv("square_pid", {"pid.p_gain": 1.0})


def test_merge_batch_launch_from_yaml_only() -> None:
    doc = yaml.safe_load("""
launch:
  scenario: leader_forward_back
  drones: 3
  duration_sec: 90.0
""")
    args = argparse.Namespace(scenario=None, drones=2, duration=0.0)
    sid, n, d = merge_batch_launch(doc, args)
    assert sid == "leader_forward_back"
    assert n == 3
    assert d == 90.0


def test_merge_batch_launch_cli_overrides_scenario() -> None:
    doc = yaml.safe_load("""
launch:
  scenario: square_pid
  duration_sec: 10.0
""")
    args = argparse.Namespace(scenario="leader_forward_back", drones=2, duration=0.0)
    sid, n, d = merge_batch_launch(doc, args)
    assert sid == "leader_forward_back"
    assert n == 2
    assert d == 10.0


def test_batch_experiment_label_prefers_batch_id() -> None:
    doc = yaml.safe_load("launch:\n  batch_id: sweep_apr\n")
    assert batch_experiment_label(doc, "/tmp/user_batch_params.yaml") == "sweep_apr"


def test_batch_experiment_label_fallback_stem() -> None:
    doc = yaml.safe_load("{}")
    assert batch_experiment_label(doc, "/x/y/myfile.yaml") == "myfile"


def test_effective_launch_merges_batch_then_launch() -> None:
    doc = yaml.safe_load("""
batch:
  scenario: square_pid
  drones: 5
launch:
  scenario: leader_forward_back
  duration_sec: 30
""")
    eff = effective_launch_block(doc)
    assert eff["scenario"] == "leader_forward_back"
    assert eff["drones"] == 5
    assert eff["duration_sec"] == 30


def test_merge_batch_launch_uses_batch_block() -> None:
    doc = yaml.safe_load("""
batch:
  scenario: leader_forward_back
  drones: 2
  duration_sec: 45
""")
    args = argparse.Namespace(scenario=None, drones=9, duration=0.0)
    sid, n, d = merge_batch_launch(doc, args)
    assert sid == "leader_forward_back"
    assert n == 2
    assert d == 45.0
