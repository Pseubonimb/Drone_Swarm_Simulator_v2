"""Datatypes for decentralized assignment messages and local agent state."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass
class AssignmentExchangeMessage:
    """
    Message 𝓖^i = (G_lean^i, y^i, γ^i) broadcast or sent along edges in a round.

    Field shapes are conventionally consistent within one algorithm instance
    (e.g. ``g_lean`` is 2-D, ``y`` and ``gamma`` are 1-D); callers enforce sizes.

    Attributes:
        g_lean: Local reduced-cost or lean matrix block (symbolically G_lean^i).
        y: Dual or price vector segment y^i.
        gamma: Auxiliary vector γ^i (e.g. bids or offsets).
    """

    g_lean: NDArray[np.floating]
    y: NDArray[np.floating]
    gamma: NDArray[np.floating]


@dataclass
class AgentLocalState:
    """
    Local state maintained by agent ``agent_id`` during assignment iterations.

    Uses the same structural fields as :class:`AssignmentExchangeMessage` for
    the negotiation variables held at node ``agent_id``.

    Attributes:
        agent_id: Agent index (row index in the global cost matrix), 0-based.
        g_lean: Local G_lean block at this agent.
        y: Local y vector at this agent.
        gamma: Local γ vector at this agent.
    """

    agent_id: int
    g_lean: NDArray[np.floating]
    y: NDArray[np.floating]
    gamma: NDArray[np.floating]
