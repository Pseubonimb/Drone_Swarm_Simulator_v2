"""Unit tests for swarm coordinate exchange (common frame, metrics, noise config)."""

import random

from core.network import (
    CoordExchangeNoiseConfig,
    compute_collision_flags,
    default_local_to_common_ned,
    formation_metrics_step,
)


def test_default_local_to_common_ned_y_shift() -> None:
    pos = {"x": 1.0, "y": 10.0, "z": -3.0}
    out = default_local_to_common_ned(2, pos, east_spacing_m=2.0)
    assert out["x"] == 1.0
    assert out["z"] == -3.0
    assert out["y"] == 8.0  # 10 - (2-1)*2


def test_compute_collision_flags() -> None:
    positions = {
        1: {"x": 0.0, "y": 0.0, "z": 0.0},
        2: {"x": 0.0, "y": 0.15, "z": 0.0},
    }
    flags = compute_collision_flags(positions, collision_radius=0.2)
    assert flags[1] and flags[2]


def test_formation_metrics_step() -> None:
    positions = {
        1: {"x": 0.0, "y": 0.0, "z": 0.0},
        2: {"x": 0.0, "y": 2.0, "z": 0.0},
    }
    ferr, dmin, coll = formation_metrics_step(positions, 0.2, d_star=2.0)
    assert ferr < 1e-6
    assert abs(dmin - 2.0) < 1e-6
    assert coll is False


def test_noise_config_from_mapping() -> None:
    cfg = CoordExchangeNoiseConfig.from_mapping(
        {
            "position_sigma_m": (0.1, 0.0, 0.05),
            "packet_loss_probability": 0.2,
            "seed": 42,
        }
    )
    assert cfg.position_sigma_m == (0.1, 0.0, 0.05)
    assert cfg.packet_loss_probability == 0.2
    assert cfg.seed == 42


def test_position_noise_zero_is_identity() -> None:
    cfg = CoordExchangeNoiseConfig()
    rng = random.Random(1)
    pos = {"x": 1.0, "y": 2.0, "z": 3.0}
    from core.network import CoordExchangeManager

    mgr = CoordExchangeManager(
        [],
        noise=cfg,
    )
    out = mgr._apply_position_noise(pos, rng)  # noqa: SLF001
    assert out == pos
