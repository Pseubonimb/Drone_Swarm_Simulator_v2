"""Split batch YAML combos into scenario vs SITL parameters; write .parm overlays."""

from __future__ import annotations

import os
from typing import Dict, Mapping, Tuple


def partition_combo_for_batch(
    combo: Mapping[str, float],
) -> Tuple[Dict[str, float], Dict[str, float]]:
    """Split combination dict into scenario keys vs ``sitl.*`` keys (values unchanged)."""
    scenario: Dict[str, float] = {}
    sitl: Dict[str, float] = {}
    for k, v in combo.items():
        if isinstance(k, str) and k.startswith("sitl."):
            sitl[k] = float(v)
        else:
            scenario[k] = float(v)
    return scenario, sitl


def write_sitl_overlay_parm(path: str, sitl_params: Mapping[str, float]) -> None:
    """Write a minimal ArduPilot ``.parm`` file for SITL ``--add-param-file``.

    Keys must be logical names ``sitl.PARAM_NAME`` (e.g. ``sitl.ANGLE_MAX``).
    ``ANGLE_MAX`` is written as an integer (centidegrees).

    Args:
        path: File to create (parent dirs created if needed).
        sitl_params: Subset of a batch combo containing only ``sitl.*`` keys.

    Raises:
        ValueError: Unknown or invalid SITL parameter key.
    """
    if not sitl_params:
        return
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    lines: list[str] = []
    for k, v in sitl_params.items():
        if not isinstance(k, str) or not k.startswith("sitl."):
            continue
        name = k[5:].strip()
        if not name or not name.replace("_", "").isalnum():
            raise ValueError(f"Invalid SITL parameter key: {k!r}")
        if name == "ANGLE_MAX":
            lines.append(f"ANGLE_MAX {int(round(float(v)))}\n")
        else:
            raise ValueError(
                f"Unsupported SITL batch key {k!r}; extend write_sitl_overlay_parm to map it."
            )
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)
