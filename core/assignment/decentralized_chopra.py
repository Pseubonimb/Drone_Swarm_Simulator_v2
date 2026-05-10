"""Synchronous distributed Hungarian method (Chopra et al., arXiv:1805.08712).

This module simulates Algorithm *Distributed-Hungarian* from Section IV with
synchronous communication rounds over an undirected connectivity graph (each
robot exchanges state with all adjacent peers; bidirectional links model the
paper's strongly connected directed network).

**Simulation note (oracle ``C``):** Re-computation of the equality subgraph after
dual Step 1(b) uses the full cost matrix ``C`` (oracle), matching a setting where
each robot holds one row of ``C`` but a testbench / coordinator has the global
instance for verification. Edge weights on :math:`E_y \\cup E_{cand}` are
implicitly ``C[i, j]``.

**Deviation from Sec. IV (``Reduce Edge Set``):** :func:`_reduce_edge_set_identity`
keeps the full equality graph :math:`E_y` after each dual update instead of the
paper's lean edge-set reduction. That is intentional for this **oracle/testbench**
implementation: correctness checks and small-:math:`n` tests stay closer to a
standard Hungarian equality layer. **Implications:** communicated subgraphs and
payload sizes (e.g. :class:`~core.assignment.metrics.MetricsCollector` message
proxies, wire packing) are **not** comparable to the communication-complexity
story in the paper; interpret those metrics as simulator overhead for the full-
:math:`E_y` model, not as a bound for the lean exchange protocol.

Indexing matches :mod:`core.assignment.reference_lsap`: agents are rows ``0..n-1``,
targets are columns ``0..n-1``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np
from numpy.typing import NDArray

from core.assignment.graph import ConnectivityGraph
from core.assignment.metrics import MetricsCollector
from core.assignment.reference_lsap import LSAPResult, solve_lsap_reference
from core.assignment.serialization import assignment_exchange_wire_payload_bytes

_FLOAT_EPS = 1e-9


def _adjacency_list(graph: ConnectivityGraph) -> list[list[int]]:
    adj: list[list[int]] = [[] for _ in range(graph.num_nodes)]
    for a, b in graph.edges:
        adj[a].append(b)
        adj[b].append(a)
    for lst in adj:
        lst.sort()
    return adj


def _bfs_max_distance_from(adj: list[list[int]], start: int) -> tuple[int, int]:
    """Return (max_distance, reachable_count) from start; unreachable -> inf distance."""
    from collections import deque

    dist = [-1] * len(adj)
    q: deque[int] = deque([start])
    dist[start] = 0
    reachable = 0
    while q:
        u = q.popleft()
        reachable += 1
        for v in adj[u]:
            if dist[v] == -1:
                dist[v] = dist[u] + 1
                q.append(v)
    if reachable < len(adj):
        return 10**9, reachable
    return max(dist), reachable


def graph_connectivity_stats(graph: ConnectivityGraph) -> dict[str, float | int | bool]:
    """Indicators for logging (not part of the paper)."""
    adj = _adjacency_list(graph)
    n = graph.num_nodes
    if n == 0:
        return {
            "num_nodes": 0,
            "num_edges": 0,
            "is_connected": True,
            "graph_diameter": 0,
        }
    diam = 0
    connected = True
    for s in range(n):
        d, r = _bfs_max_distance_from(adj, s)
        if r < n:
            connected = False
            diam = max(diam, 10**9)
        else:
            diam = max(diam, d)
    return {
        "num_nodes": n,
        "num_edges": len(graph.edges),
        "is_connected": connected,
        "graph_diameter": int(diam) if diam < 10**8 else -1,
    }


@dataclass
class ChopraRobotState:
    """Local state :math:`G^i = (G^i_{lean}, y^i, \\gamma^i)` for robot ``i``."""

    Ey: set[tuple[int, int]]
    Ecand: set[tuple[int, int]]
    y_r: NDArray[np.floating]
    y_p: NDArray[np.floating]
    gamma: int


def _init_robot_state(
    cost_row: NDArray[np.floating], robot_id: int, n: int
) -> ChopraRobotState:
    row = np.asarray(cost_row, dtype=float).ravel()
    j_star = int(np.argmin(row))
    val = float(row[j_star])
    Ey = {(robot_id, j_star)}
    y_r = np.zeros(n, dtype=float)
    y_p = np.zeros(n, dtype=float)
    y_r[robot_id] = val
    return ChopraRobotState(Ey=set(Ey), Ecand=set(), y_r=y_r, y_p=y_p, gamma=-1)


def _equality_edges(
    cost: NDArray[np.floating],
    y_r: NDArray[np.floating],
    y_p: NDArray[np.floating],
) -> set[tuple[int, int]]:
    n = cost.shape[0]
    out: set[tuple[int, int]] = set()
    for i in range(n):
        for j in range(n):
            if abs(float(cost[i, j]) - float(y_r[i]) - float(y_p[j])) <= _FLOAT_EPS:
                out.add((i, j))
    return out


def _max_bipartite_matching(
    n: int,
    edges: Iterable[tuple[int, int]],
) -> tuple[list[int], int]:
    """Maximum matching on ``edges``; ``match_r[i]`` is matched column or ``-1``."""
    edge_set = set(edges)
    match_r = [-1] * n
    match_p = [-1] * n

    def dfs(i: int, seen_p: list[bool]) -> bool:
        for j in range(n):
            if (i, j) not in edge_set:
                continue
            if seen_p[j]:
                continue
            seen_p[j] = True
            if match_p[j] == -1 or dfs(match_p[j], seen_p):
                match_r[i] = j
                match_p[j] = i
                return True
        return False

    for i in range(n):
        dfs(i, [False] * n)
    sz = sum(1 for x in match_r if x >= 0)
    return match_r, sz


def _min_vertex_cover(
    n: int,
    edges: set[tuple[int, int]],
    match_r: list[int],
) -> tuple[set[int], set[int]]:
    """König: cover = (L \\ B_L) ∪ B_R from alternating reachability from free left."""
    match_p = [-1] * n
    for i in range(n):
        if match_r[i] != -1:
            match_p[match_r[i]] = i
    from collections import deque

    vis_l = [False] * n
    vis_r = [False] * n
    q: deque[tuple[str, int]] = deque()
    for i in range(n):
        if match_r[i] == -1:
            vis_l[i] = True
            q.append(("L", i))
    while q:
        kind, v = q.popleft()
        if kind == "L":
            i = v
            for j in range(n):
                if (i, j) not in edges:
                    continue
                if match_r[i] == j:
                    continue
                if not vis_r[j]:
                    vis_r[j] = True
                    q.append(("R", j))
        else:
            j = v
            pi = match_p[j]
            if pi != -1 and not vis_l[pi]:
                vis_l[pi] = True
                q.append(("L", pi))
    cover_r = {i for i in range(n) if not vis_l[i]}
    cover_p = {j for j in range(n) if vis_r[j]}
    return cover_r, cover_p


def _get_best_edge(
    robot_id: int,
    cost: NDArray[np.floating],
    y_r: NDArray[np.floating],
    y_p: NDArray[np.floating],
    cover_r: set[int],
    cover_p: set[int],
) -> tuple[int, int] | None:
    """``Get Best Edge`` for robot ``robot_id`` (paper Section IV-B)."""
    n = cost.shape[0]
    if robot_id in cover_r:
        return None
    best_j = -1
    best_slack = float("inf")
    for j in range(n):
        if j in cover_p:
            continue
        slack = float(cost[robot_id, j]) - float(y_r[robot_id]) - float(y_p[j])
        if slack < best_slack - _FLOAT_EPS:
            best_slack = slack
            best_j = j
    if best_j < 0:
        return None
    return (robot_id, best_j)


def _reduce_edge_set_identity(
    n: int,
    Ey: set[tuple[int, int]],
    _match_r: list[int],
    _cover_r: set[int],
    _cover_p: set[int],
) -> set[tuple[int, int]]:
    """Placeholder for ``Reduce Edge Set``; keeps full ``Ey`` (small :math:`n` in tests)."""
    return set(Ey)


def build_latest_graph(
    r0: Iterable[int],
    states: dict[int, ChopraRobotState],
    n: int,
) -> ChopraRobotState:
    """``Build Latest Graph`` (Section IV-A)."""
    r0_list = list(r0)
    gammas = [states[j].gamma for j in r0_list]
    if all(g == -1 for g in gammas):
        Ey: set[tuple[int, int]] = set()
        for j in r0_list:
            Ey |= states[j].Ey
        Ecand: set[tuple[int, int]] = set()
        y_r = np.zeros(n, dtype=float)
        y_p = np.zeros(n, dtype=float)
        for j in r0_list:
            y_r[j] = float(states[j].y_r[j])
        gamma = -1
        robots_touched = {i for (i, _) in Ey}
        if len(robots_touched) == n:
            gamma = 0
        return ChopraRobotState(Ey=Ey, Ecand=Ecand, y_r=y_r, y_p=y_p, gamma=gamma)
    gmax = max(gammas)
    r_lead = [j for j in r0_list if states[j].gamma == gmax]
    j_star = min(r_lead)
    lead = states[j_star]
    Ey = set(lead.Ey)
    y_r = lead.y_r.copy()
    y_p = lead.y_p.copy()
    gamma = lead.gamma
    Ecand = set()
    for j in r_lead:
        Ecand |= states[j].Ecand
    return ChopraRobotState(Ey=Ey, Ecand=Ecand, y_r=y_r, y_p=y_p, gamma=gamma)


def local_hungarian(
    gtmp: ChopraRobotState,
    cost: NDArray[np.floating],
    robot_id: int,
) -> ChopraRobotState:
    """``Local Hungarian`` (Section IV-B) for robot ``robot_id``."""
    st, _, _ = local_hungarian_with_meta(gtmp, cost, robot_id)
    return st


def local_hungarian_with_meta(
    gtmp: ChopraRobotState,
    cost: NDArray[np.floating],
    robot_id: int,
) -> tuple[ChopraRobotState, int, int]:
    """Like :func:`local_hungarian` but returns ``(state, label_updates, edges_in_M)``."""
    n = cost.shape[0]
    Ey = set(gtmp.Ey)
    Ecand = set(gtmp.Ecand)
    y_r = gtmp.y_r.copy()
    y_p = gtmp.y_p.copy()
    gamma = gtmp.gamma

    match_r, msz = _max_bipartite_matching(n, Ey)
    if msz == n:
        return ChopraRobotState(Ey, Ecand, y_r, y_p, gamma), 0, msz

    cover_r, cover_p = _min_vertex_cover(n, Ey, match_r)
    uncovered_r = [r for r in range(n) if r not in cover_r]
    ecand = _get_best_edge(robot_id, cost, y_r, y_p, cover_r, cover_p)
    l_updates = 0
    if ecand is not None:
        Ecand.add(ecand)

    if len(uncovered_r) == len(Ecand) and len(Ecand) > 0:
        delta = min(
            float(cost[i, j]) - float(y_r[i]) - float(y_p[j]) for (i, j) in Ecand
        )
        for r in cover_r:
            y_r[r] -= delta
        for p in range(n):
            if p not in cover_p:
                y_p[p] += delta
        l_updates = len(cover_r) + (n - len(cover_p))
        Ey = _equality_edges(cost, y_r, y_p)
        match_r, msz = _max_bipartite_matching(n, Ey)
        cover_r, cover_p = _min_vertex_cover(n, Ey, match_r)
        gamma = gamma + 1
        Ecand = set()
        ec2 = _get_best_edge(robot_id, cost, y_r, y_p, cover_r, cover_p)
        if ec2 is not None:
            Ecand.add(ec2)
        Ey = _reduce_edge_set_identity(n, Ey, match_r, cover_r, cover_p)
        msz = sum(1 for x in match_r if x >= 0)

    return ChopraRobotState(Ey, Ecand, y_r, y_p, gamma), l_updates, msz


def _assignment_from_perfect_ey(
    n: int, Ey: set[tuple[int, int]]
) -> NDArray[np.int64] | None:
    """Greedy perfect matching on ``Ey`` rows; None if not realizable greedily."""
    match_r, msz = _max_bipartite_matching(n, Ey)
    if msz < n:
        return None
    return np.asarray(match_r, dtype=np.int64)


def _cost_of_assignment(cost: NDArray[np.floating], assign: NDArray[np.int64]) -> float:
    n = cost.shape[0]
    return float(sum(float(cost[i, int(assign[i])]) for i in range(n)))


@dataclass
class DecentralizedRunResult:
    """Outcome of :func:`run_decentralized_hungarian_chopra2018`."""

    assignment: NDArray[np.int64]
    """``assignment[i]`` is the target column for agent ``i``."""

    row_ind: NDArray[np.int64]
    col_ind: NDArray[np.int64]
    total_cost: float
    reference: LSAPResult
    states_final: dict[int, ChopraRobotState]
    metrics: dict[str, object]
    converged: bool
    rounds: int


def run_decentralized_hungarian_chopra2018(
    cost: NDArray[np.floating],
    graph: ConnectivityGraph,
    *,
    max_rounds: int | None = None,
    collect_metrics: MetricsCollector | None = None,
) -> DecentralizedRunResult:
    """
    Run synchronous *Distributed-Hungarian* rounds (Section IV, Evolution loop).

    Each round, every robot ``i`` builds ``R0 = N(i) ∪ {i}``, merges via
    :func:`build_latest_graph`, then applies :func:`local_hungarian` iff
    :math:`\\gamma_{tmp} \\geq 0`, else adopts ``G_tmp`` (paper pseudocode).

    **Stopping (simulator):** all robots hold a perfect matching on ``E_y`` with
    the **same** column assignment and total cost matches
    :func:`solve_lsap_reference`, or ``max_rounds`` exceeded (then best effort
    from agent ``0``).

    Args:
        cost: Square ``(n, n)`` matrix ``C[i, j]``.
        graph: Communication graph on ``n`` robots.
        max_rounds: Upper bound on synchronous rounds; default ``n ** 3 + 64``.
        collect_metrics: Optional :class:`~core.assignment.metrics.MetricsCollector`.

    Returns:
        :class:`DecentralizedRunResult` with assignment, SciPy reference, and
        serialized-ready metrics dict.

    Raises:
        ValueError: If ``cost`` is not finite square ``(n, n)`` or ``graph.num_nodes != n``.
    """
    c = np.asarray(cost, dtype=float)
    if c.ndim != 2 or c.shape[0] != c.shape[1]:
        raise ValueError("cost must be a square 2-D array")
    n = c.shape[0]
    if not np.all(np.isfinite(c)):
        raise ValueError("cost must contain only finite values")
    if graph.num_nodes != n:
        raise ValueError(
            f"graph.num_nodes ({graph.num_nodes}) must match cost.shape[0] ({n})"
        )

    ref = solve_lsap_reference(c)
    if max_rounds is None:
        max_rounds = n * n * n + 64

    adj = _adjacency_list(graph)
    states: dict[int, ChopraRobotState] = {
        i: _init_robot_state(c[i], i, n) for i in range(n)
    }

    mc = collect_metrics or MetricsCollector()
    gstat = graph_connectivity_stats(graph)
    mc.record_graph_stats(gstat)

    best_assign: NDArray[np.int64] | None = None
    best_cost = float("inf")
    converged = False
    last_r = -1

    for r in range(max_rounds):
        last_r = r
        mc.start_round()
        msgs_sent = 0
        # messages: each robot sends state to outgoing neighbors — undirected: deg(i) out
        for i in range(n):
            msgs_sent += len(adj[i])
        mc.record_messages(msgs_sent)

        new_states: dict[int, ChopraRobotState] = {}
        prop_delay_proxy_round = 0
        label_round = 0
        max_edges_m = 0
        for i in range(n):
            r0 = [i] + adj[i]
            prop_delay_proxy_round = max(prop_delay_proxy_round, len(r0) - 1)
            gtmp = build_latest_graph(r0, states, n)
            if gtmp.gamma >= 0:
                st, lu, em = local_hungarian_with_meta(gtmp, c, i)
                label_round += lu
                max_edges_m = max(max_edges_m, em)
                new_states[i] = st
            else:
                new_states[i] = ChopraRobotState(
                    set(gtmp.Ey),
                    set(gtmp.Ecand),
                    gtmp.y_r.copy(),
                    gtmp.y_p.copy(),
                    gtmp.gamma,
                )
        mc.record_label_updates(label_round)
        mc.record_propagation_delay_proxy(prop_delay_proxy_round)
        mc.record_edges_in_matching(max_edges_m)

        states = new_states

        assigns: list[NDArray[np.int64] | None] = []
        for i in range(n):
            ass = _assignment_from_perfect_ey(n, states[i].Ey)
            if ass is None:
                assigns.append(None)
            else:
                assigns.append(ass.copy())

        incompatibility = 0
        scipy_mismatch = 0
        if assigns and all(a is not None for a in assigns):
            ref_assign = np.empty(n, dtype=np.int64)
            ref_assign[ref.row_ind] = ref.col_ind
            base = assigns[0]
            assert base is not None
            costs_local = [_cost_of_assignment(c, assigns[k]) for k in range(n)]
            for k in range(1, n):
                other = assigns[k]
                assert other is not None
                if not np.array_equal(base, other):
                    incompatibility += 1
            for k in range(n):
                ak = assigns[k]
                assert ak is not None
                if not np.array_equal(ak, ref_assign):
                    scipy_mismatch += 1
            total_c = _cost_of_assignment(c, base)
            agree = all(
                assigns[j] is not None and np.array_equal(assigns[j], base)
                for j in range(n)
            )
            opt_cost = float(ref.optimal_cost)
            tol = max(1e-8, _FLOAT_EPS * max(1.0, abs(opt_cost)))
            if agree and abs(total_c - opt_cost) <= tol:
                converged = True
                best_assign = base.copy()
                best_cost = total_c
                mc.record_scipy_delta(0.0)
                mc.record_incompatibility(incompatibility)
                break

            worst_delta = max(
                abs(c_local - float(ref.optimal_cost)) for c_local in costs_local
            )
            mc.record_scipy_delta(float(worst_delta))
        mc.record_incompatibility(incompatibility)

        if assigns[0] is not None:
            c0 = _cost_of_assignment(c, assigns[0])
            if c0 < best_cost:
                best_cost = c0
                best_assign = assigns[0].copy()

    if best_assign is None:
        best_assign = np.asarray(
            [int(np.argmin(c[i])) for i in range(n)], dtype=np.int64
        )

    row_ind = np.arange(n, dtype=np.int64)
    col_ind = best_assign.copy()
    total_cost = _cost_of_assignment(c, best_assign)

    ref_assign_vec = np.empty(n, dtype=np.int64)
    ref_assign_vec[ref.row_ind] = ref.col_ind
    scipy_mismatch_final = int(
        sum(1 for k in range(n) if int(best_assign[k]) != int(ref_assign_vec[k]))
    )

    s0 = states[0]
    _, wire_bytes = assignment_exchange_wire_payload_bytes(
        agent_id=0,
        n=n,
        Ey=s0.Ey,
        Ecand=s0.Ecand,
        y_r=s0.y_r,
        y_p=s0.y_p,
        gamma=s0.gamma,
    )
    rounds_done = last_r + 1 if last_r >= 0 else 0

    metrics_out = mc.finalize(
        iterations=mc.iterations,
        reference_optimal_cost=float(ref.optimal_cost),
        decentralized_cost=float(total_cost),
        converged=converged,
        rounds_executed=rounds_done,
        extra={
            "wire_payload_utf8_json_bytes_agent0": wire_bytes,
            "scipy_assignment_mismatch_count_final": scipy_mismatch_final,
        },
    )

    return DecentralizedRunResult(
        assignment=best_assign,
        row_ind=row_ind,
        col_ind=col_ind,
        total_cost=float(total_cost),
        reference=ref,
        states_final=states,
        metrics=metrics_out,
        converged=converged,
        rounds=rounds_done,
    )


__all__ = [
    "ChopraRobotState",
    "DecentralizedRunResult",
    "build_latest_graph",
    "graph_connectivity_stats",
    "local_hungarian",
    "local_hungarian_with_meta",
    "run_decentralized_hungarian_chopra2018",
]
