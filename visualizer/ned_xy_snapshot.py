"""Headless NED X–Y plots for experiment snapshots (matplotlib Agg)."""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

_PALETTE = [
    "#2563eb",
    "#dc2626",
    "#16a34a",
    "#ca8a04",
    "#9333ea",
    "#0d9488",
    "#ea580c",
    "#4f46e5",
    "#059669",
    "#b91c1c",
    "#1d4ed8",
    "#65a30d",
    "#c026d3",
    "#0e7490",
    "#d97706",
]


def _drone_zoom_limits(
    positions: Dict[int, Dict[str, float]], pad_m: float
) -> Optional[Tuple[float, float, float, float]]:
    """Square axis limits around drone positions only (for inset visibility)."""
    if not positions:
        return None
    xs = [float(positions[d]["x"]) for d in sorted(positions.keys())]
    ys = [float(positions[d]["y"]) for d in sorted(positions.keys())]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    cx = (xmin + xmax) / 2
    cy = (ymin + ymax) / 2
    half_span = max((xmax - xmin) / 2, (ymax - ymin) / 2, 1.0) + pad_m
    return (cx - half_span, cx + half_span, cy - half_span, cy + half_span)


def save_ned_xy_png(
    path: str,
    positions: Dict[int, Dict[str, float]],
    *,
    targets: Optional[List[Dict[str, float]]] = None,
    title: str = "",
    figsize: Tuple[float, float] = (9, 7),
    inset_drone_zoom: bool = False,
    inset_pad_m: float = 6.0,
) -> None:
    """Write a 2D NED plot (X North, Y East) with drones and optional target crosses.

    Args:
        path: Output ``.png`` path.
        positions: Drone id -> ``{"x","y","z"}`` in common NED (m).
        targets: Optional waypoint list in the same frame (shown as black crosses).
        title: Figure title.
        figsize: Matplotlib figure size in inches.
        inset_drone_zoom: If True, add a lower-right inset zoomed to **drone** positions only
            (plus ``inset_pad_m``), so motion is visible even when global axis spans distant targets.
        inset_pad_m: Half-margin (m) around drone span inside the inset.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.axes_grid1.inset_locator import inset_axes

    fig, ax = plt.subplots(figsize=figsize)
    ax.set_xlabel("X (m, North, NED)")
    ax.set_ylabel("Y (m, East, NED)")
    ax.set_title(title or "NED X–Y")
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="box")

    ids = sorted(positions.keys())
    for did in ids:
        p = positions[did]
        col = _PALETTE[int(did) % len(_PALETTE)]
        ax.plot(
            p["x"],
            p["y"],
            "o",
            color=col,
            markersize=9,
            label=f"Drone {did}",
        )

    tx: List[float] = []
    ty: List[float] = []
    if targets:
        tx = [float(t.get("x", 0)) for t in targets]
        ty = [float(t.get("y", 0)) for t in targets]
        ax.plot(
            tx,
            ty,
            linestyle="None",
            marker="+",
            color="black",
            markersize=14,
            markeredgewidth=2,
            label="Targets",
            zorder=5,
        )

    if ids or targets:
        ax.legend(loc="upper left", fontsize=9)

    if inset_drone_zoom and ids:
        zlim = _drone_zoom_limits(positions, inset_pad_m)
        if zlim is not None:
            x0, x1, y0, y1 = zlim
            axins = inset_axes(
                ax,
                width="36%",
                height="36%",
                loc="lower right",
                borderpad=0.35,
            )
            for did in ids:
                p = positions[did]
                col = _PALETTE[int(did) % len(_PALETTE)]
                axins.plot(p["x"], p["y"], "o", color=col, markersize=7)
            if targets:
                axins.plot(
                    tx,
                    ty,
                    linestyle="None",
                    marker="+",
                    color="black",
                    markersize=10,
                    markeredgewidth=1.5,
                    zorder=5,
                )
            axins.set_xlim(x0, x1)
            axins.set_ylim(y0, y1)
            axins.set_aspect("equal", adjustable="box")
            axins.grid(True, alpha=0.25)
            axins.set_title("Дроны (крупно)", fontsize=9)

    plt.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
