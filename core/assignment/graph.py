"""Communication graph for decentralized assignment (P2P rounds over edges)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ConnectivityGraph:
    """
    Undirected connectivity graph for assignment coordination.

    Representation: **edge list** (not an adjacency matrix). Vertices are agents
    labeled ``0 .. num_nodes - 1``. Each undirected edge is stored once as an
    unordered pair ``(i, j)`` with ``i < j``.

    **Duplicates:** If the input ``edges`` contains the same pair more than once,
    duplicates are **removed**; stored ``edges`` are sorted lexicographically by
    ``(i, j)``.

    Attributes:
        num_nodes: Number of vertices |V|.
        edges: Canonical undirected edges ``(i, j)`` with ``0 <= i < j < num_nodes``.
    """

    num_nodes: int
    edges: tuple[tuple[int, int], ...]

    def __post_init__(self) -> None:
        if self.num_nodes < 0:
            raise ValueError("num_nodes must be non-negative")
        unique: set[tuple[int, int]] = set()
        for i, j in self.edges:
            if not (0 <= i < j < self.num_nodes):
                raise ValueError(
                    f"invalid edge ({i}, {j}) for num_nodes={self.num_nodes}; "
                    "expect 0 <= i < j < num_nodes",
                )
            unique.add((i, j))
        canon = tuple(sorted(unique))
        object.__setattr__(self, "edges", canon)
