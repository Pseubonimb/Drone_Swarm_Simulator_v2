#!/usr/bin/env python3
"""
Build one delay plot from per-step time-sync CSV.

X axis: CSV record index (step_idx).
Y axis: delay (seconds).

Example:
    python3 scripts/plot_time_sync.py experiments/<run>/drone_1_time_sync_steps.csv
    python3 scripts/plot_time_sync.py experiments/<run>/drone_1_time_sync_steps.csv --out plots/time_sync_d1.png
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
import matplotlib.colors as mcolors
import numpy as np
from matplotlib.patches import Patch


def _load_step_sync_rows(csv_path: Path) -> Tuple[Dict[str, np.ndarray], Optional[List[str]]]:
    """Load per-step time-sync rows aligned with drone_*.csv writes."""
    cols: Dict[str, List[float]] = {
        "step_idx": [],
        "hypervisor_rx_to_tx_sec": [],
        "sitl_cmd_to_response_sitl_sec": [],
        "sitl_cmd_to_response_py_sec": [],
    }
    modes: List[str] = []
    has_flight_mode = False
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames and "flight_mode" in reader.fieldnames:
            has_flight_mode = True
        for row in reader:
            try:
                step_idx = float((row.get("step_idx") or "").strip())
            except ValueError:
                continue

            cols["step_idx"].append(step_idx)

            def parse_optional(name: str) -> float:
                raw = (row.get(name) or "").strip()
                try:
                    return float(raw) if raw else np.nan
                except ValueError:
                    return np.nan

            cols["hypervisor_rx_to_tx_sec"].append(parse_optional("hypervisor_rx_to_tx_sec"))
            cols["sitl_cmd_to_response_sitl_sec"].append(
                parse_optional("sitl_cmd_to_response_sitl_sec")
            )
            cols["sitl_cmd_to_response_py_sec"].append(
                parse_optional("sitl_cmd_to_response_py_sec")
            )
            if has_flight_mode:
                modes.append((row.get("flight_mode") or "").strip())
    out = {k: np.array(v, dtype=float) for k, v in cols.items()}
    mode_list = modes if has_flight_mode and len(modes) == len(out["step_idx"]) else None
    return out, mode_list


def _sitl_delay_metric_codes(
    sitl_sitl: np.ndarray, sitl_py: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Per CSV row: which column supplied the plotted SITL cmd->response delay.

    Returns:
        Tuple of (codes, sitl_plot) where codes[i] is 0 = sitl_cmd_to_response_sitl_sec,
        1 = sitl_cmd_to_response_py_sec (fallback), -1 = no finite value.
    """
    sitl_plot = np.array(sitl_sitl, dtype=float)
    fin_s = np.isfinite(sitl_plot)
    fin_p = np.isfinite(sitl_py)
    missing_sitl_clock = ~fin_s
    sitl_plot[missing_sitl_clock] = sitl_py[missing_sitl_clock]
    codes = np.full(sitl_plot.shape, -1, dtype=np.int8)
    codes[fin_s] = 0
    codes[~fin_s & fin_p] = 1
    return codes, sitl_plot


def _build_step_plot(
    rows: Dict[str, np.ndarray],
    title: str,
    out_path: Path | None,
    flight_modes: Optional[List[str]] = None,
) -> None:
    """Plot step index vs delays; bottom strip shows which metric feeds SITL delay each step."""
    import matplotlib.pyplot as plt

    step_idx = rows["step_idx"]
    hypervisor = rows["hypervisor_rx_to_tx_sec"]
    sitl_sitl = rows["sitl_cmd_to_response_sitl_sec"]
    sitl_py = rows["sitl_cmd_to_response_py_sec"]
    codes, sitl_plot = _sitl_delay_metric_codes(sitl_sitl, sitl_py)
    used_py_fallback = bool(np.any((codes == 1)))

    if step_idx.size < 2:
        raise ValueError("Not enough rows in step sync CSV (need at least 2).")

    fig, (ax, ax_src) = plt.subplots(
        2,
        1,
        figsize=(10, 7.2),
        sharex=True,
        gridspec_kw={"height_ratios": [3.4, 1.0], "hspace": 0.07},
    )
    has_hyper = np.any(~np.isnan(hypervisor))
    has_sitl = np.any(~np.isnan(sitl_plot))
    if not has_hyper and not has_sitl:
        raise ValueError("No delay values found in CSV.")
    if has_hyper:
        ax.plot(
            step_idx,
            hypervisor,
            label="Hypervisor delay rx->tx (s)",
            linewidth=1.4,
        )
    if has_sitl:
        sitl_label = "SITL delay cmd->response (combined)"
        if used_py_fallback:
            sitl_label += " — see lower strip for metric per step"
        ax.plot(
            step_idx,
            sitl_plot,
            label=sitl_label,
            linewidth=1.4,
            alpha=0.9,
        )
    ax.set_ylabel("Delay (s)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    if has_sitl:
        rgba_sitl = mcolors.to_rgba("#1f77b4", 0.42)
        rgba_py = mcolors.to_rgba("#ff7f0e", 0.42)
        w0 = codes == 0
        w1 = codes == 1
        if np.any(w0):
            ax_src.fill_between(
                step_idx, 0.0, 0.42, where=w0, facecolor=rgba_sitl, linewidth=0, interpolate=False
            )
        if np.any(w1):
            ax_src.fill_between(
                step_idx, 0.58, 1.0, where=w1, facecolor=rgba_py, linewidth=0, interpolate=False
            )
        ax_src.set_ylim(0.0, 1.0)
        ax_src.set_yticks([0.21, 0.79])
        ax_src.set_yticklabels(
            ["sitl_cmd_to_response_sitl_sec", "sitl_cmd_to_response_py_sec"],
            fontsize=8,
        )
        ax_src.set_ylabel("SITL delay\nmetric", fontsize=9)
        ax_src.grid(True, axis="x", alpha=0.25)
        ax_src.legend(
            handles=[
                Patch(facecolor=rgba_sitl, edgecolor="#1f55a0", linewidth=0.5, label="SITL clock (time_boot)"),
                Patch(facecolor=rgba_py, edgecolor="#cc6600", linewidth=0.5, label="Python monotonic"),
            ],
            loc="upper left",
            fontsize=7,
            title="Value plotted for SITL delay",
            title_fontsize=7,
        )
    else:
        ax_src.set_yticks([])
        ax_src.set_ylabel("")

    ax_src.set_xlabel("CSV record index (step_idx)")

    if flight_modes and len(flight_modes) == len(step_idx):
        parts: List[np.ndarray] = []
        if has_hyper:
            parts.append(hypervisor[np.isfinite(hypervisor)])
        if has_sitl:
            parts.append(sitl_plot[np.isfinite(sitl_plot)])
        stacked = np.concatenate(parts) if parts else np.array([], dtype=float)
        ymax = float(np.max(stacked)) if stacked.size > 0 else 0.1
        if not np.isfinite(ymax) or ymax <= 0:
            ymax = 0.1
        y_label = ymax * 0.92
        prev = ""
        for i, mode in enumerate(flight_modes):
            m = (mode or "").strip()
            if not m:
                continue
            if i == 0 or m != prev:
                x = float(step_idx[i])
                for a in (ax, ax_src):
                    a.axvline(x, color="#666666", linestyle=":", linewidth=0.9, alpha=0.75)
                ax.text(
                    x,
                    y_label,
                    m,
                    rotation=90,
                    verticalalignment="top",
                    horizontalalignment="right",
                    fontsize=7,
                    color="#333333",
                    clip_on=True,
                )
                prev = m

    fig.tight_layout()

    if out_path is not None:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=160, bbox_inches="tight")
        print(f"Saved plot: {out_path}")
    else:
        plt.show()
    plt.close(fig)


def main() -> None:
    """CLI entrypoint for one-graph delay plotting."""
    parser = argparse.ArgumentParser(
        description=(
            "Plot one graph: step index (X) vs delays (Y) "
            "for hypervisor and SITL."
        )
    )
    parser.add_argument(
        "csv_path",
        type=Path,
        help="Path to drone_<id>_time_sync_steps.csv",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help=(
            "Optional PNG output path. If omitted and GUI is available, show window. "
            "If GUI unavailable, save next to CSV as <csv_name>_time_sync.png."
        ),
    )
    parser.add_argument(
        "--title",
        type=str,
        default="Hypervisor/SITL delay by CSV record",
        help="Plot title.",
    )
    args = parser.parse_args()

    interactive_mode = args.out is None
    if interactive_mode:
        if os.environ.get("DISPLAY"):
            try:
                matplotlib.use("TkAgg")
            except Exception:
                matplotlib.use("Agg")
        else:
            matplotlib.use("Agg")
    else:
        matplotlib.use("Agg")

    if not args.csv_path.is_file():
        raise FileNotFoundError(f"CSV not found: {args.csv_path}")
    with args.csv_path.open("r", encoding="utf-8", newline="") as f:
        header = f.readline()
    if "step_idx" not in header:
        raise ValueError(
            "Unsupported CSV format. Use drone_<id>_time_sync_steps.csv from experiment directory."
        )

    out_path = args.out
    if interactive_mode and matplotlib.get_backend().lower() == "agg":
        out_path = args.csv_path.with_name(f"{args.csv_path.stem}_time_sync.png")
        print(
            "Interactive backend unavailable (no DISPLAY/Tk); "
            f"saved PNG instead: {out_path}"
        )

    rows, flight_modes = _load_step_sync_rows(args.csv_path)
    _build_step_plot(rows, args.title, out_path, flight_modes=flight_modes)


if __name__ == "__main__":
    main()
