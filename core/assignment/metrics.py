"""Experiment metrics for decentralized assignment (JSON-friendly aggregates)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class MetricsCollector:
    """Accumulates counters for one *Distributed-Hungarian* experiment run.

    Per-round optimality gaps are stored in :attr:`scipy_delta_per_round`: each
    entry is recorded only in rounds where every robot had a perfect matching on
    its local ``E_y`` (see :func:`~core.assignment.decentralized_chopra.run_decentralized_hungarian_chopra2018`).
    The JSON payload from :meth:`finalize` always includes ``scipy_delta_abs``,
    the absolute gap between the *final* decentralized cost and the reference
    LSAP optimum (end of run), which is the single scalar summary for consumers.
    """

    iterations: int = 0
    messages_total: int = 0
    label_updates_total: int = 0
    propagation_delay_proxy_sum: int = 0
    edges_in_matching_max_per_round: int = 0
    scipy_optimal_cost: float = 0.0
    decentralized_final_cost: float = 0.0
    incompatibility_max: int = 0
    graph_stats: dict[str, Any] = field(default_factory=dict)
    scipy_delta_per_round: list[float] = field(default_factory=list)
    incompatibility_per_round: list[int] = field(default_factory=list)

    def start_round(self) -> None:
        """Mark the beginning of a new synchronous communication round."""
        self.iterations += 1

    def record_messages(self, count: int) -> None:
        """Add messages sent in the current round (e.g. sum of out-degrees)."""
        self.messages_total += int(count)

    def record_label_updates(self, count: int) -> None:
        """Add dual label component updates in the round (proxy for Step 1(b) work)."""
        self.label_updates_total += int(count)

    def record_propagation_delay_proxy(self, hops: int) -> None:
        """Add per-round neighborhood radius proxy (max |R0|-1 over robots)."""
        self.propagation_delay_proxy_sum += int(hops)

    def record_edges_in_matching(self, count: int) -> None:
        """Track maximum cardinality matching size seen in any Local Hungarian call."""
        self.edges_in_matching_max_per_round = max(
            self.edges_in_matching_max_per_round,
            int(count),
        )

    def record_scipy_delta(self, delta: float) -> None:
        """Append a per-round snapshot of the optimality gap (when recorded by the runner)."""
        self.scipy_delta_per_round.append(float(delta))

    def record_incompatibility(self, count: int) -> None:
        """Robots whose extracted assignment differs from agent 0 (consensus proxy)."""
        self.incompatibility_per_round.append(int(count))
        self.incompatibility_max = max(self.incompatibility_max, int(count))

    def record_graph_stats(self, stats: dict[str, Any]) -> None:
        """Store static connectivity indicators once per run."""
        self.graph_stats = dict(stats)

    def finalize(
        self,
        *,
        iterations: int,
        reference_optimal_cost: float,
        decentralized_cost: float,
        converged: bool,
        rounds_executed: int,
        extra: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """
        Build a structure suitable for ``assignment_metrics.json``.

        Field names are stable for experiment tooling; values are JSON-serializable.

        ``scipy_delta_abs`` is ``|decentralized_cost - reference_optimal_cost|`` for
        the returned assignment. ``scipy_delta_per_round`` is a time series of gaps
        recorded during the run (may be shorter than ``rounds_executed`` if some
        rounds had no full ``E_y`` matching at every robot).
        """
        out: dict[str, Any] = {
            "iterations": iterations,
            "rounds_executed": rounds_executed,
            "converged": converged,
            "messages_total": self.messages_total,
            "label_updates_total": self.label_updates_total,
            "propagation_delay_proxy_rounds_sum": self.propagation_delay_proxy_sum,
            "edges_in_matching_max_observed": self.edges_in_matching_max_per_round,
            "reference_optimal_cost": reference_optimal_cost,
            "decentralized_cost": decentralized_cost,
            "scipy_delta_abs": abs(decentralized_cost - reference_optimal_cost),
            "incompatibility_max": self.incompatibility_max,
            "graph": dict(self.graph_stats),
            "scipy_delta_per_round": list(self.scipy_delta_per_round),
            "incompatibility_per_round": list(self.incompatibility_per_round),
        }
        if extra:
            out.update(extra)
        return out
