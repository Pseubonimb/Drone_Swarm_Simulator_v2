"""Serialization for assignment exchange messages (wire size / JSON logs).

Wire format (JSON object, UTF-8) — one message payload:

.. code-block:: json

    {
      "schema": "assignment_exchange/v1",
      "agent_id": 0,
      "n": 5,
      "Ey": [[0, 2], [1, 3]],
      "Ecand": [[2, 4]],
      "y_r": [1.0, 2.0, ...],
      "y_p": [0.0, 0.0, ...],
      "gamma": -1
    }

- ``Ey`` / ``Ecand`` are edge lists ``[row, col]`` with the project's row=agent,
  col=target convention.
- ``y_r[k]`` / ``y_p[k]`` are dual values for agent ``k`` and target ``k``.
- ``gamma`` is the counter :math:`\\gamma^i` from Chopra et al.
"""

from __future__ import annotations

import json
from typing import Any

import numpy as np
from numpy.typing import NDArray

from core.assignment.types import AssignmentExchangeMessage

WIRE_SCHEMA_V1 = "assignment_exchange/v1"


def chopra_state_to_wire_dict(
    *,
    agent_id: int,
    n: int,
    Ey: set[tuple[int, int]],
    Ecand: set[tuple[int, int]],
    y_r: NDArray[np.floating],
    y_p: NDArray[np.floating],
    gamma: int,
) -> dict[str, Any]:
    """Build the JSON-serializable dict for one agent message."""
    return {
        "schema": WIRE_SCHEMA_V1,
        "agent_id": int(agent_id),
        "n": int(n),
        "Ey": [list(e) for e in sorted(Ey)],
        "Ecand": [list(e) for e in sorted(Ecand)],
        "y_r": [float(x) for x in np.asarray(y_r, dtype=float).ravel().tolist()],
        "y_p": [float(x) for x in np.asarray(y_p, dtype=float).ravel().tolist()],
        "gamma": int(gamma),
    }


def wire_dict_to_chopra_fields(
    d: dict[str, Any],
) -> tuple[
    int, int, set[tuple[int, int]], set[tuple[int, int]], np.ndarray, np.ndarray, int
]:
    """Inverse of :func:`chopra_state_to_wire_dict` (round-trip for tests)."""
    if d.get("schema") != WIRE_SCHEMA_V1:
        raise ValueError("unsupported wire schema")
    n = int(d["n"])
    agent_id = int(d["agent_id"])
    Ey = {tuple(map(int, e)) for e in d["Ey"]}
    Ecand = {tuple(map(int, e)) for e in d["Ecand"]}
    y_r = np.asarray(d["y_r"], dtype=float).ravel()
    y_p = np.asarray(d["y_p"], dtype=float).ravel()
    if y_r.size != n or y_p.size != n:
        raise ValueError("y_r and y_p must have length n")
    gamma = int(d["gamma"])
    return agent_id, n, Ey, Ecand, y_r, y_p, gamma


def assignment_exchange_wire_payload_bytes(
    *,
    agent_id: int,
    n: int,
    Ey: set[tuple[int, int]],
    Ecand: set[tuple[int, int]],
    y_r: NDArray[np.floating],
    y_p: NDArray[np.floating],
    gamma: int,
) -> tuple[bytes, int]:
    """
    Serialize one message and return ``(utf8_payload, byte_length)``.

    The length is ``len(payload)`` (UTF-8 JSON, compact separators).
    """
    d = chopra_state_to_wire_dict(
        agent_id=agent_id,
        n=n,
        Ey=Ey,
        Ecand=Ecand,
        y_r=y_r,
        y_p=y_p,
        gamma=gamma,
    )
    payload = json.dumps(d, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return payload, len(payload)


def pack_assignment_exchange_dense(
    cost: NDArray[np.floating],
    Ey: set[tuple[int, int]],
    Ecand: set[tuple[int, int]],
    y_r: NDArray[np.floating],
    y_p: NDArray[np.floating],
    gamma: int,
) -> AssignmentExchangeMessage:
    """
    Pack a lean subgraph into :class:`~core.assignment.types.AssignmentExchangeMessage`.

    ``g_lean[i, j]`` holds reduced cost / slack :math:`C_{ij} - y_r[i] - y_p[j]`
    for ``(i, j) \\in E_y \\cup E_{cand}``; other entries are ``NaN`` (inactive).
    ``y`` is ``concat(y_r, y_p)`` (length ``2n``). ``gamma`` is length-1 array.
    """
    c = np.asarray(cost, dtype=float)
    n = c.shape[0]
    g = np.full((n, n), np.nan, dtype=float)
    for i, j in Ey | Ecand:
        g[int(i), int(j)] = float(c[i, j]) - float(y_r[i]) - float(y_p[j])
    y_pack = np.concatenate(
        [np.asarray(y_r, dtype=float).ravel(), np.asarray(y_p, dtype=float).ravel()]
    )
    g_arr = np.asarray(g, dtype=float)
    return AssignmentExchangeMessage(
        g_lean=g_arr, y=y_pack, gamma=np.asarray([float(gamma)], dtype=float)
    )


def unpack_assignment_exchange_dense(
    msg: AssignmentExchangeMessage,
    n: int,
) -> tuple[set[tuple[int, int]], set[tuple[int, int]], np.ndarray, np.ndarray, int]:
    """
    Recover edge sets where ``g_lean`` is finite; duals from ``y``; gamma from ``gamma[0]``.

    Candidate vs equality edges are **not** distinguished (return ``Ecand=∅``, all in ``Ey``).
    Callers needing ``E_{cand}`` should use wire dict or keep explicit state.
    """
    y = np.asarray(msg.y, dtype=float).ravel()
    if y.size != 2 * n:
        raise ValueError("y must have length 2n")
    y_r = y[:n].copy()
    y_p = y[n:].copy()
    g = np.asarray(msg.g_lean, dtype=float)
    Ey: set[tuple[int, int]] = set()
    for i in range(n):
        for j in range(n):
            if np.isfinite(g[i, j]):
                Ey.add((i, j))
    gamma = int(round(float(np.asarray(msg.gamma, dtype=float).ravel()[0])))
    return Ey, set(), y_r, y_p, gamma
