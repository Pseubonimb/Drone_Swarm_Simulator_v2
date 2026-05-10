"""Reference LSAP (Hungarian) via SciPy for benchmarks and tests."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import linear_sum_assignment


@dataclass(frozen=True)
class LSAPResult:
    """
    Optimal assignment from :func:`solve_lsap_reference`.

    Attributes:
        row_ind: Assigned row (agent) indices, shape ``(k,)`` — each in ``[0, n_rows)``.
        col_ind: Assigned column (target) indices, shape ``(k,)`` — each in ``[0, n_cols)``.
        optimal_cost: Minimum total cost ``sum(C[row_ind[i], col_ind[i]])``.

    Indexing convention:
        - Row ``i`` = agent ``i`` (0-based).
        - Column ``j`` = target / slot ``j`` (0-based).
        ``row_ind`` and ``col_ind`` have equal length; for full square problems,
        each agent maps to exactly one target.

    Arrays are **copies**: mutating returned ``row_ind`` / ``col_ind`` does not
    affect cached solver output elsewhere.
    """

    row_ind: NDArray[np.integer]
    col_ind: NDArray[np.integer]
    optimal_cost: float

    def __post_init__(self) -> None:
        ri = np.asarray(self.row_ind, dtype=np.int64).copy()
        ci = np.asarray(self.col_ind, dtype=np.int64).copy()
        object.__setattr__(self, "row_ind", ri)
        object.__setattr__(self, "col_ind", ci)


def solve_lsap_reference(cost: NDArray[np.floating]) -> LSAPResult:
    """
    Solve the linear sum assignment problem (LSAP) using SciPy's Hungarian method.

    **Objective:** minimize ``sum_k C[row_ind[k], col_ind[k]]`` over one-to-one
    assignments. :func:`~scipy.optimize.linear_sum_assignment` **minimizes** cost
    entries. For a **maximization** (reward) problem, pass ``-cost`` (or negate
    profits) so the minimum of negated values corresponds to maximum profit.

    **Indexing:** ``cost[i, j]`` is the cost of assigning agent (row) ``i`` to
    target (column) ``j``. Agents are ``i = 0 .. n_rows-1``; targets are
    ``j = 0 .. n_cols-1``.

    **Rectangular matrices:** If ``cost`` has shape ``(n, m)`` with ``n <= m``,
    at most one column is chosen per row; SciPy pads internally as needed. If
    ``n > m``, fewer than ``n`` rows receive matches — see SciPy docs for the
    exact rectangular behavior.

    Args:
        cost: 2-D float array of assignment costs.

    Returns:
        :class:`LSAPResult` with optimal index pairs and total cost.

    Raises:
        ValueError: If ``cost`` is not a finite 2-D array.
    """
    c = np.asarray(cost, dtype=float)
    if c.ndim != 2:
        raise ValueError("cost must be a 2-D array")
    if not np.all(np.isfinite(c)):
        raise ValueError("cost must contain only finite values")
    row_ind, col_ind = linear_sum_assignment(c)
    total = float(c[row_ind, col_ind].sum())
    return LSAPResult(
        row_ind=np.asarray(row_ind, dtype=np.int64),
        col_ind=np.asarray(col_ind, dtype=np.int64),
        optimal_cost=total,
    )
