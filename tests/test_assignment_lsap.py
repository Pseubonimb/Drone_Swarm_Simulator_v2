"""Tests for reference LSAP (Hungarian) wrapper."""

from itertools import combinations, permutations

import numpy as np
import pytest

from core.assignment import ConnectivityGraph, solve_lsap_reference


def _brute_min(cost: np.ndarray) -> float:
    n_rows, n_cols = cost.shape
    if n_rows <= n_cols:
        best = float("inf")
        for cols in combinations(range(n_cols), n_rows):
            for perm in permutations(cols):
                s = sum(cost[i, perm[i]] for i in range(n_rows))
                best = min(best, s)
        return best
    best = float("inf")
    for rows in combinations(range(n_rows), n_cols):
        for perm in permutations(range(n_cols)):
            s = sum(cost[rows[i], perm[i]] for i in range(n_cols))
            best = min(best, s)
    return best


def _assert_assignment_feasible(cost: np.ndarray, result) -> None:
    """Check SciPy assignment vs matrix shape: bounds, length, one-to-one."""
    n_rows, n_cols = cost.shape
    ri = result.row_ind
    ci = result.col_ind
    assert ri.shape == ci.shape
    k = ri.size
    assert k == min(n_rows, n_cols)
    assert len(set(ri.tolist())) == k
    assert len(set(ci.tolist())) == k
    assert np.all((ri >= 0) & (ri < n_rows))
    assert np.all((ci >= 0) & (ci < n_cols))
    assert result.optimal_cost == pytest.approx(float(cost[ri, ci].sum()))


def test_connectivity_graph_valid_edge_list() -> None:
    g = ConnectivityGraph(3, ((0, 1), (1, 2)))
    assert g.num_nodes == 3
    assert len(g.edges) == 2


def test_connectivity_graph_deduplicates_edges() -> None:
    g = ConnectivityGraph(3, ((0, 1), (0, 1), (1, 2)))
    assert g.edges == ((0, 1), (1, 2))


def test_connectivity_graph_sorts_edges() -> None:
    g = ConnectivityGraph(4, ((2, 3), (0, 1)))
    assert g.edges == ((0, 1), (2, 3))


def test_connectivity_graph_rejects_bad_edge() -> None:
    with pytest.raises(ValueError):
        ConnectivityGraph(2, ((0, 2),))


def test_square_minimizes_known_optimum() -> None:
    cost = np.array(
        [
            [4.0, 1.0, 3.0],
            [2.0, 0.0, 5.0],
            [3.0, 2.0, 2.0],
        ]
    )
    result = solve_lsap_reference(cost)
    brute = _brute_min(cost)
    assert result.optimal_cost == pytest.approx(brute)
    _assert_assignment_feasible(cost, result)


def test_rectangular_tall_matrix() -> None:
    cost = np.array([[1.0, 2.0, 3.0], [2.0, 4.0, 2.0]])
    result = solve_lsap_reference(cost)
    brute = _brute_min(cost)
    assert result.optimal_cost == pytest.approx(brute)
    _assert_assignment_feasible(cost, result)


def test_rectangular_wide_matrix() -> None:
    cost = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
    result = solve_lsap_reference(cost)
    brute = _brute_min(cost)
    assert result.optimal_cost == pytest.approx(brute)
    _assert_assignment_feasible(cost, result)


def test_rejects_nonfinite() -> None:
    cost = np.array([[1.0, np.inf], [0.0, 1.0]])
    with pytest.raises(ValueError):
        solve_lsap_reference(cost)


def test_lsap_result_indices_independent_between_calls() -> None:
    cost = np.eye(3, dtype=float)
    a = solve_lsap_reference(cost)
    b = solve_lsap_reference(cost)
    a.row_ind.fill(-1)
    assert np.all(b.row_ind >= 0)
    assert b.optimal_cost == pytest.approx(0.0)
