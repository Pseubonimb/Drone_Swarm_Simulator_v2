"""Decentralized Hungarian (Chopra et al. 2018) vs SciPy reference — no SITL.

Optimum comparison (square costs, minimization):

- **Primary check:** ``run.total_cost`` matches ``solve_lsap_reference(c).optimal_cost``
  within tolerance (mirrors the simulator's convergence criterion).
- **Degenerate optima:** several permutations can attain the same minimal total. In
  that case we assert **cost agreement only** — the decentralized assignment may
  disagree with ``linear_sum_assignment`` row/column indices without indicating a bug.
- **Unique optimum (small n):** we brute-count how many permutations hit the minimum;
  if exactly one, we additionally require ``run.assignment`` to match SciPy's labeling
  (same column per row as the reference solution).
"""

from __future__ import annotations

import itertools

import numpy as np
import pytest

from core.assignment import (
    WIRE_SCHEMA_V1,
    ConnectivityGraph,
    MetricsCollector,
    assignment_exchange_wire_payload_bytes,
    chopra_state_to_wire_dict,
    pack_assignment_exchange_dense,
    run_decentralized_hungarian_chopra2018,
    solve_lsap_reference,
    unpack_assignment_exchange_dense,
    wire_dict_to_chopra_fields,
)

_COST_TOL = 1e-8


def _complete_graph(n: int) -> ConnectivityGraph:
    edges = tuple((i, j) for i in range(n) for j in range(i + 1, n))
    return ConnectivityGraph(n, edges)


def _path_graph(n: int) -> ConnectivityGraph:
    if n <= 1:
        return ConnectivityGraph(n, ())
    edges = tuple((i, i + 1) for i in range(n - 1))
    return ConnectivityGraph(n, edges)


def _empty_graph(n: int) -> ConnectivityGraph:
    """No edges: each robot only has itself in R0, so gamma stays negative for n>1."""
    return ConnectivityGraph(n, ())


def _degree_sum(graph: ConnectivityGraph) -> int:
    """Undirected: sum_i deg(i) = 2 * |E| (simulator counts one message per directed send)."""
    return int(2 * len(graph.edges))


def _reference_column_vector(ref) -> np.ndarray:
    n = int(ref.row_ind.size)
    vec = np.empty(n, dtype=np.int64)
    vec[ref.row_ind] = ref.col_ind
    return vec


def _permutation_optima_count(cost: np.ndarray) -> tuple[int, float]:
    """For square n, return (#permutations achieving min cost, min cost)."""
    n = int(cost.shape[0])
    best = float("inf")
    cnt = 0
    for perm in itertools.permutations(range(n)):
        s = float(sum(float(cost[i, perm[i]]) for i in range(n)))
        if s < best - _COST_TOL:
            best, cnt = s, 1
        elif abs(s - best) <= _COST_TOL:
            cnt += 1
    return cnt, best


def _assert_cost_matches_scipy(c: np.ndarray, run) -> None:
    ref = solve_lsap_reference(c)
    assert run.total_cost == pytest.approx(float(ref.optimal_cost), rel=0, abs=1e-5)
    assert float(run.metrics["scipy_delta_abs"]) == pytest.approx(0.0, abs=1e-5)


def test_n_equals_one_converges_to_reference() -> None:
    c = np.array([[7.5]])
    g = _empty_graph(1)
    ref = solve_lsap_reference(c)
    run = run_decentralized_hungarian_chopra2018(
        c, g, max_rounds=50, collect_metrics=MetricsCollector()
    )
    assert run.converged
    assert run.total_cost == pytest.approx(float(ref.optimal_cost))
    assert run.metrics["scipy_delta_abs"] == pytest.approx(0.0, abs=1e-5)
    assert run.rounds >= 1


def test_isolated_agents_hit_max_rounds_without_convergence() -> None:
    """Isolated agents never activate Local Hungarian; run exhausts max_rounds."""
    rng = np.random.RandomState(42)
    n = 4
    c = rng.rand(n, n) * 10.0
    max_rounds = 20
    run = run_decentralized_hungarian_chopra2018(
        c,
        _empty_graph(n),
        max_rounds=max_rounds,
        collect_metrics=MetricsCollector(),
    )
    assert not run.converged
    assert run.rounds == max_rounds


@pytest.mark.parametrize("n", [2, 3, 4, 6])
@pytest.mark.parametrize("graph_fn", [_complete_graph, _path_graph])
def test_decentralized_matches_scipy_cost_and_assignment_when_unique(
    n: int, graph_fn
) -> None:
    """Random dense costs (almost surely unique optimum): cost + SciPy assignment."""
    rng = np.random.RandomState(17 * n + len(graph_fn.__name__))
    for _ in range(5):
        c = rng.rand(n, n) * 10.0
        ref = solve_lsap_reference(c)
        n_opt, brute_min = _permutation_optima_count(c)
        assert float(ref.optimal_cost) == pytest.approx(brute_min, rel=0, abs=1e-5)
        assert n_opt >= 1

        g = graph_fn(n)
        run = run_decentralized_hungarian_chopra2018(
            c, g, max_rounds=5000, collect_metrics=MetricsCollector()
        )
        assert run.converged, (n, graph_fn.__name__)
        _assert_cost_matches_scipy(c, run)
        if n_opt == 1:
            ref_vec = _reference_column_vector(ref)
            np.testing.assert_array_equal(run.assignment, ref_vec)


def test_degenerate_optimum_cost_only_not_scipy_rows() -> None:
    """Flat costs: many optimal permutations; compare **total cost**, not SciPy's tie-break."""
    c = np.ones((3, 3), dtype=float) * 5.0
    ref = solve_lsap_reference(c)
    n_opt, _ = _permutation_optima_count(c)
    assert n_opt == 6  # all 3! permutations attain 15.0
    g = _complete_graph(3)
    run = run_decentralized_hungarian_chopra2018(c, g, collect_metrics=MetricsCollector())
    assert run.converged
    assert run.total_cost == pytest.approx(float(ref.optimal_cost), rel=0, abs=1e-5)


def test_metrics_messages_scale_with_edges_and_rounds() -> None:
    """Each round records sum_i deg(i) messages; total == iterations * degree_sum."""
    c = np.array([[4.0, 1.0, 3.0], [2.0, 0.0, 5.0], [3.0, 2.0, 2.0]])
    mc = MetricsCollector()
    g = _complete_graph(3)
    run = run_decentralized_hungarian_chopra2018(c, g, collect_metrics=mc)
    assert run.converged
    ds = _degree_sum(g)
    it = int(run.metrics["iterations"])
    mt = int(run.metrics["messages_total"])
    assert mt == it * ds
    assert mt >= ds  # at least one synchronous round with non-isolated graph


def test_serialization_wire_nonempty_and_stable_for_fixed_state() -> None:
    payload_a, nbytes_a = assignment_exchange_wire_payload_bytes(
        agent_id=0,
        n=2,
        Ey={(0, 1)},
        Ecand=set(),
        y_r=np.zeros(2),
        y_p=np.zeros(2),
        gamma=-1,
    )
    payload_b, nbytes_b = assignment_exchange_wire_payload_bytes(
        agent_id=0,
        n=2,
        Ey={(0, 1)},
        Ecand=set(),
        y_r=np.zeros(2),
        y_p=np.zeros(2),
        gamma=-1,
    )
    assert len(payload_a) > 0 and nbytes_a > 0
    assert nbytes_a == nbytes_b
    assert payload_a == payload_b
    assert b"assignment_exchange/v1" in payload_a


def test_post_run_wire_payload_metric_positive() -> None:
    c = np.eye(2)
    run = run_decentralized_hungarian_chopra2018(
        c, _complete_graph(2), collect_metrics=MetricsCollector()
    )
    assert run.converged
    b = int(run.metrics["wire_payload_utf8_json_bytes_agent0"])
    assert b > 0


def test_wire_round_trip() -> None:
    d = chopra_state_to_wire_dict(
        agent_id=2,
        n=3,
        Ey={(0, 1), (1, 2)},
        Ecand={(2, 0)},
        y_r=np.array([1.0, 2.0, 3.0]),
        y_p=np.zeros(3),
        gamma=1,
    )
    assert d["schema"] == WIRE_SCHEMA_V1
    aid, n, Ey, Ecand, yr, yp, g = wire_dict_to_chopra_fields(d)
    assert aid == 2 and n == 3
    assert Ey == {(0, 1), (1, 2)}
    assert Ecand == {(2, 0)}
    assert g == 1


def _dense_packed_nbytes(msg) -> int:
    """Stable in-memory footprint of :class:`~core.assignment.types.AssignmentExchangeMessage`."""
    return int(msg.g_lean.nbytes + msg.y.nbytes + msg.gamma.nbytes)


def test_pack_unpack_dense_preserves_duals() -> None:
    c = np.eye(3)
    Ey = {(0, 0), (1, 1)}
    Ecand: set[tuple[int, int]] = set()
    yr = np.array([1.0, 0.0, 0.0])
    yp = np.array([0.0, 1.0, 0.0])
    msg = pack_assignment_exchange_dense(c, Ey, Ecand, yr, yp, -1)
    assert _dense_packed_nbytes(msg) > 0
    Ey2, _ec, yr2, yp2, gm = unpack_assignment_exchange_dense(msg, 3)
    assert gm == -1
    np.testing.assert_allclose(yr2, yr)
    np.testing.assert_allclose(yp2, yp)
    assert (0, 0) in Ey2 and (1, 1) in Ey2


def test_pack_dense_byte_size_stable() -> None:
    c = np.eye(3)
    m1 = pack_assignment_exchange_dense(c, {(0, 0)}, set(), np.zeros(3), np.zeros(3), -1)
    m2 = pack_assignment_exchange_dense(c, {(0, 0)}, set(), np.zeros(3), np.zeros(3), -1)
    n1 = _dense_packed_nbytes(m1)
    assert n1 == _dense_packed_nbytes(m2) > 0


def test_raises_on_graph_size_mismatch() -> None:
    c = np.eye(2)
    g = ConnectivityGraph(3, ((0, 1), (1, 2)))
    with pytest.raises(ValueError, match="num_nodes"):
        run_decentralized_hungarian_chopra2018(c, g)
