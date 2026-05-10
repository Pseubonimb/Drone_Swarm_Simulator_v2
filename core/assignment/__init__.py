"""
Assignment package: centralized LSAP (SciPy), Chopra et al. distributed Hungarian,
metrics, and serialization.
"""

from core.assignment.decentralized_chopra import (
    ChopraRobotState,
    DecentralizedRunResult,
    build_latest_graph,
    graph_connectivity_stats,
    local_hungarian,
    local_hungarian_with_meta,
    run_decentralized_hungarian_chopra2018,
)
from core.assignment.graph import ConnectivityGraph
from core.assignment.metrics import MetricsCollector
from core.assignment.reference_lsap import LSAPResult, solve_lsap_reference
from core.assignment.serialization import (
    WIRE_SCHEMA_V1,
    assignment_exchange_wire_payload_bytes,
    chopra_state_to_wire_dict,
    pack_assignment_exchange_dense,
    unpack_assignment_exchange_dense,
    wire_dict_to_chopra_fields,
)
from core.assignment.types import AgentLocalState, AssignmentExchangeMessage

__all__ = [
    "WIRE_SCHEMA_V1",
    "AgentLocalState",
    "AssignmentExchangeMessage",
    "ChopraRobotState",
    "ConnectivityGraph",
    "DecentralizedRunResult",
    "LSAPResult",
    "MetricsCollector",
    "assignment_exchange_wire_payload_bytes",
    "build_latest_graph",
    "chopra_state_to_wire_dict",
    "graph_connectivity_stats",
    "local_hungarian",
    "local_hungarian_with_meta",
    "pack_assignment_exchange_dense",
    "run_decentralized_hungarian_chopra2018",
    "solve_lsap_reference",
    "unpack_assignment_exchange_dense",
    "wire_dict_to_chopra_fields",
]
