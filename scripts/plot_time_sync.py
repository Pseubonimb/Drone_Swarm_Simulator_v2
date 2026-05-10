#!/usr/bin/env python3
"""
Delay plots: hypervisor (Python) + SITL (patched ArduPilot trace).

**Default (host wall-clock):** one axis ``t - t₀`` where ``t`` is host wall time
(``py_write_wall_sec`` in ``drone_*_time_sync_steps.csv`` and ``host_wall_us`` on
the TX line in ``sitl_rc_telem_*.csv``). ``t₀`` is the earliest sampled wall time
among plotted points so both curves share comparable time.

**Mixed axis:** If the trace has ``mono_us`` but no usable ``host_wall_us`` on TX
rows, the SITL series is plotted vs ``mono_us − trace_min`` (seconds); Python
still uses ``py_write_wall_sec − t₀``. The subtitle/x-label notes that those
horizontal scales are not the same clock.

**SITL ``--sitl-tx``:** ArduPilot may log ``TX_ATTITUDE`` / ``TX_LOCAL_POS`` but not
``TX_SIM_STATE``. Default ``AUTO`` tries ``TX_SIM_STATE``, then ``TX_ATTITUDE``, then
``TX_LOCAL_POS`` (first with RC_RX pairs wins).

Examples::
    python3 scripts/plot_time_sync.py experiments/<run>/drone_1_time_sync_steps.csv \\
        --sitl-latest --sitl-instance 0
    python3 scripts/plot_time_sync.py experiments/<run>/drone_1_time_sync_steps.csv \\
        --legacy-split
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPT_DIR.parent

# RC_RX pairs with first matching TX row after each RC_RX (see _deltas_rc_to_tx).
_SITL_TX_AUTO_ORDER = ("TX_SIM_STATE", "TX_ATTITUDE", "TX_LOCAL_POS")


def time_sync_log_dir() -> Path:
    return _PROJECT_ROOT / "logs" / "time_sync"


def _trace_search_dirs() -> List[Path]:
    dirs: List[Path] = []
    v = (os.environ.get("DRONE_SWARM_SITL_TELEM_TRACE_DIR") or "").strip()
    if v:
        dirs.append(Path(v).resolve())
    dirs.append(time_sync_log_dir())
    dirs.append(Path("/tmp"))
    seen: set[Path] = set()
    out: List[Path] = []
    for d in dirs:
        if d not in seen:
            seen.add(d)
            out.append(d)
    return out


def glob_latest_sitl_trace(instance: int) -> Optional[Path]:
    matches: List[Path] = []
    for d in _trace_search_dirs():
        pattern = str(d / f"sitl_rc_telem_{instance}_*.csv")
        for p in glob.glob(pattern):
            matches.append(Path(p))
    if not matches:
        return None
    return max(matches, key=lambda p: p.stat().st_mtime)


def _load_step_rows(
    csv_path: Path,
) -> Tuple[Dict[str, np.ndarray], Optional[List[str]]]:
    cols: Dict[str, List[float]] = {
        "step_idx": [],
        "py_write_wall_sec": [],
        "hypervisor_rx_to_tx_sec": [],
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

            cols["py_write_wall_sec"].append(parse_optional("py_write_wall_sec"))
            cols["hypervisor_rx_to_tx_sec"].append(
                parse_optional("hypervisor_rx_to_tx_sec")
            )
            if has_flight_mode:
                modes.append((row.get("flight_mode") or "").strip())
    out = {k: np.array(v, dtype=float) for k, v in cols.items()}
    mode_list = modes if has_flight_mode and len(modes) == len(out["step_idx"]) else None
    return out, mode_list


def _load_sitl_trace_rows(path: Path) -> Tuple[List[dict], Optional[List[str]]]:
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        fields = list(reader.fieldnames or ())
        rows = list(reader)
    return rows, fields


def _tx_events_in_trace(rows: List[dict]) -> List[str]:
    names: set[str] = set()
    for r in rows:
        e = (r.get("event") or "").strip()
        if e.startswith("TX_"):
            names.add(e)
    return sorted(names)


def _sitl_pair_wall_mono(
    rows: List[dict], tx_event: str
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    pair_idx, dsec = _deltas_rc_to_tx(rows, tx_event)
    wall, _ = _deltas_rc_to_tx_wall(rows, tx_event)
    mono_rel, _ = _deltas_rc_to_tx_mono_rel(rows, tx_event)
    return pair_idx, dsec, wall, mono_rel


def resolve_sitl_tx_for_plot(
    rows: List[dict],
    *,
    requested: str,
    trace_path: Path,
) -> Tuple[str, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Pick TX event column; returns (effective_event, pair_idx, deltas, wall_abs, mono_rel)."""
    if requested == "AUTO":
        for ev in _SITL_TX_AUTO_ORDER:
            pj, d, w, m = _sitl_pair_wall_mono(rows, ev)
            if d.size > 0:
                print(f"SITL TX event (--sitl-tx AUTO → {ev})", file=sys.stderr)
                return ev, pj, d, w, m
        print(
            f"No RC_RX→TX_* pairs (tried {', '.join(_SITL_TX_AUTO_ORDER)}) in {trace_path}",
            file=sys.stderr,
        )
        tx_like = ", ".join(_tx_events_in_trace(rows)) or "(none)"
        print(f"TX-like events in trace: {tx_like}", file=sys.stderr)
        pj, d, w, m = _sitl_pair_wall_mono(rows, _SITL_TX_AUTO_ORDER[0])
        return _SITL_TX_AUTO_ORDER[0], pj, d, w, m
    pj, d, w, m = _sitl_pair_wall_mono(rows, requested)
    if d.size == 0:
        print(f"No RC_RX->{requested} pairs in {trace_path}", file=sys.stderr)
        tx_like = ", ".join(_tx_events_in_trace(rows)) or "(none)"
        print(
            f"TX-like events present: {tx_like} — use --sitl-tx AUTO "
            "or an event that follows RC_RX in this build.",
            file=sys.stderr,
        )
    return requested, pj, d, w, m


def _deltas_rc_to_tx(
    rows: List[dict], tx_event: str
) -> Tuple[np.ndarray, np.ndarray]:
    """Pair index and Δt (mono_us) for legacy plot."""
    idx_list: List[int] = []
    dsec: List[float] = []
    pending_mono: Optional[int] = None
    n = 0
    for r in rows:
        ev = r.get("event", "").strip()
        try:
            mono = int(r["mono_us"])
        except (KeyError, ValueError):
            continue
        if ev == "RC_RX":
            pending_mono = mono
            continue
        if pending_mono is None:
            continue
        if ev == tx_event:
            idx_list.append(n)
            n += 1
            dsec.append((mono - pending_mono) * 1e-6)
            pending_mono = None
    return np.array(idx_list, dtype=float), np.array(dsec, dtype=float)


def _deltas_rc_to_tx_wall(
    rows: List[dict], tx_event: str
) -> Tuple[np.ndarray, np.ndarray]:
    """Host wall time (s, Unix) at TX row, Δt from mono_us."""
    t_wall: List[float] = []
    dsec: List[float] = []
    pending_mono: Optional[int] = None
    for r in rows:
        ev = r.get("event", "").strip()
        try:
            mono = int(r["mono_us"])
        except (KeyError, ValueError):
            continue
        if ev == "RC_RX":
            pending_mono = mono
            continue
        if pending_mono is None:
            continue
        if ev == tx_event:
            raw_hw = (r.get("host_wall_us") or "").strip()
            if raw_hw:
                try:
                    t_wall.append(int(raw_hw) * 1e-6)
                except ValueError:
                    t_wall.append(np.nan)
            else:
                t_wall.append(np.nan)
            dsec.append((mono - pending_mono) * 1e-6)
            pending_mono = None
    return np.array(t_wall, dtype=float), np.array(dsec, dtype=float)


def _deltas_rc_to_tx_mono_rel(
    rows: List[dict], tx_event: str
) -> Tuple[np.ndarray, np.ndarray]:
    """X = (mono_tx − global trace min mono) seconds; Y = Δt RC_RX→TX from mono_us."""
    global_min: Optional[int] = None
    for r in rows:
        try:
            m = int(r["mono_us"])
            global_min = m if global_min is None else min(global_min, m)
        except (KeyError, ValueError):
            continue
    if global_min is None:
        return np.array([], dtype=float), np.array([], dtype=float)
    x_rel: List[float] = []
    dsec: List[float] = []
    pending_mono: Optional[int] = None
    for r in rows:
        ev = r.get("event", "").strip()
        try:
            mono = int(r["mono_us"])
        except (KeyError, ValueError):
            continue
        if ev == "RC_RX":
            pending_mono = mono
            continue
        if pending_mono is None:
            continue
        if ev == tx_event:
            x_rel.append((mono - global_min) * 1e-6)
            dsec.append((mono - pending_mono) * 1e-6)
            pending_mono = None
    return np.array(x_rel, dtype=float), np.array(dsec, dtype=float)


def _flight_mode_markers(
    ax,
    x: np.ndarray,
    flight_modes: List[str],
    y_series: List[np.ndarray],
) -> None:
    parts: List[np.ndarray] = []
    for y in y_series:
        yy = y[np.isfinite(y)]
        if yy.size:
            parts.append(yy)
    stacked = np.concatenate(parts) if parts else np.array([], dtype=float)
    ymax = float(np.max(stacked)) if stacked.size > 0 else 0.1
    if not np.isfinite(ymax) or ymax <= 0:
        ymax = 0.1
    y_label = ymax * 0.92
    prev = ""
    for i, mode in enumerate(flight_modes):
        m = (mode or "").strip()
        if not m or i >= len(x):
            continue
        if i == 0 or m != prev:
            xv = float(x[i])
            ax.axvline(xv, color="#666666", linestyle=":", linewidth=0.9, alpha=0.75)
            ax.text(
                xv,
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


def _build_figure_legacy(
    *,
    step_rows: Optional[Dict[str, np.ndarray]],
    flight_modes: Optional[List[str]],
    sitl_pair_idx: Optional[np.ndarray],
    sitl_deltas: Optional[np.ndarray],
    sitl_trace_basename: Optional[str],
    sitl_tx_label: str,
    title: str,
):
    import matplotlib.pyplot as plt

    has_step = step_rows is not None and step_rows["step_idx"].size >= 1
    has_sitl = (
        sitl_pair_idx is not None and sitl_deltas is not None and sitl_deltas.size > 0
    )
    if not has_step and not has_sitl:
        raise ValueError("No plottable data.")

    n_axes = int(has_step) + int(has_sitl)
    fig_h = 4.6 * n_axes + 0.8
    fig, axes = plt.subplots(n_axes, 1, figsize=(10, fig_h), squeeze=False)
    ax_list = axes.ravel().tolist()
    ai = 0
    if has_step:
        ax = ax_list[ai]
        ai += 1
        step_idx = step_rows["step_idx"]
        hyper = step_rows["hypervisor_rx_to_tx_sec"]
        if np.any(np.isfinite(hyper)):
            ax.plot(
                step_idx,
                hyper,
                label="Hypervisor delay rx→tx (Python, s)",
                linewidth=1.3,
                color="#1f77b4",
            )
        else:
            ax.text(
                0.5,
                0.5,
                "No finite hypervisor_rx_to_tx_sec",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
        ax.set_ylabel("Delay (s)")
        ax.set_xlabel("step_idx")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        y_for_mode = [hyper] if np.any(np.isfinite(hyper)) else []
        if flight_modes and len(flight_modes) == len(step_idx) and y_for_mode:
            _flight_mode_markers(ax, step_idx, flight_modes, y_for_mode)

    if has_sitl:
        ax = ax_list[ai]
        ax.plot(
            sitl_pair_idx,
            sitl_deltas,
            ".-",
            markersize=3,
            alpha=0.85,
            color="#d62728",
            label=f"SITL Δt RC_RX→{sitl_tx_label} (s)",
        )
        ax.set_ylabel("Delay (s)")
        ax.set_xlabel("Pair index")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
        if sitl_trace_basename:
            ax.set_title(os.path.basename(sitl_trace_basename), fontsize=9, color="#555555")

    fig.suptitle(title + " (legacy axes)", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return fig


def _build_figure_wall(
    *,
    t0: float,
    x_hyp_rel: Optional[np.ndarray],
    y_hyp: Optional[np.ndarray],
    x_sitl_rel: Optional[np.ndarray],
    y_sitl: Optional[np.ndarray],
    flight_modes: Optional[List[str]],
    x_mode_abs: Optional[np.ndarray],
    title: str,
    sitl_tx_label: str,
    sitl_trace_basename: Optional[str],
    sitl_x_from_mono: bool = False,
):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 5.2))
    y_parts: List[np.ndarray] = []

    if x_hyp_rel is not None and y_hyp is not None and y_hyp.size > 0:
        ax.plot(
            x_hyp_rel,
            y_hyp,
            label="Hypervisor rx→tx (Python, s)",
            linewidth=1.2,
            color="#1f77b4",
            alpha=0.9,
        )
        y_parts.append(y_hyp[np.isfinite(y_hyp)])

    if x_sitl_rel is not None and y_sitl is not None and y_sitl.size > 0:
        ax.plot(
            x_sitl_rel,
            y_sitl,
            ".-",
            markersize=3,
            label=f"SITL RC_RX→{sitl_tx_label} (C++ trace, s)",
            color="#d62728",
            alpha=0.88,
        )
        y_parts.append(y_sitl[np.isfinite(y_sitl)])

    if sitl_x_from_mono and x_hyp_rel is not None:
        ax.set_xlabel(
            "Python: py_write_wall_sec − t₀ (s); SITL: mono − trace_min (s) "
            "(not wall-synced to Python)"
        )
    elif sitl_x_from_mono:
        ax.set_xlabel(
            "SITL mono time − trace min (s) — internal AP_HAL monotonic clock"
        )
    else:
        ax.set_xlabel("Host wall time − t₀ (s) — t₀ = earliest plotted wall sample")
    ax.set_ylabel("Delay (s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    if sitl_trace_basename:
        ax.set_title(os.path.basename(sitl_trace_basename), fontsize=9, color="#555555")

    if (
        flight_modes
        and x_mode_abs is not None
        and len(flight_modes) == len(x_mode_abs)
        and y_parts
    ):
        stacked = np.concatenate(y_parts)
        ymax = float(np.nanmax(stacked)) if stacked.size else 0.1
        if not np.isfinite(ymax) or ymax <= 0:
            ymax = 0.1
        y_label = ymax * 0.92
        prev = ""
        for i, mode in enumerate(flight_modes):
            m = (mode or "").strip()
            if not m or i >= len(x_mode_abs):
                continue
            w = x_mode_abs[i]
            if not np.isfinite(w):
                continue
            if i == 0 or m != prev:
                ax.axvline(w - t0, color="#666666", linestyle=":", linewidth=0.9, alpha=0.75)
                ax.text(
                    w - t0,
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

    fig.suptitle(title, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig


def _default_out_path(
    step_path: Optional[Path],
    sitl_path: Optional[Path],
    sitl_tx: str,
    *,
    wall_unified: bool,
) -> Path:
    time_sync_log_dir().mkdir(parents=True, exist_ok=True)
    suf = "_wall_delays.png" if wall_unified else "_delays.png"
    if step_path is not None and sitl_path is not None:
        return time_sync_log_dir() / f"{step_path.stem}_and_{sitl_path.stem}{suf}"
    if step_path is not None:
        return time_sync_log_dir() / f"{step_path.stem}{suf}"
    if sitl_path is not None:
        return time_sync_log_dir() / f"{sitl_path.stem}_rc_to_{sitl_tx.lower()}{suf}"
    return time_sync_log_dir() / "delay_plot.png"


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "step_csv",
        nargs="?",
        type=Path,
        default=None,
        help="drone_<id>_time_sync_steps.csv",
    )
    parser.add_argument("--sitl-trace", type=Path, default=None, help="sitl_rc_telem_*.csv")
    parser.add_argument(
        "--sitl-latest",
        action="store_true",
        help="Newest sitl_rc_telem_<instance>_*.csv",
    )
    parser.add_argument("--sitl-instance", type=int, default=0, metavar="N")
    parser.add_argument(
        "--sitl-tx",
        choices=("AUTO", "TX_SIM_STATE", "TX_LOCAL_POS", "TX_ATTITUDE"),
        default="AUTO",
        help="SITL TX row for RC_RX→TX latency (AUTO prefers first with data)",
    )
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument(
        "--legacy-split",
        action="store_true",
        help="Two subplots (step_idx / pair index) instead of host wall time",
    )
    parser.add_argument(
        "--title",
        type=str,
        default="Delays: hypervisor vs SITL (host wall time)",
        help="Figure title (wall-unified mode)",
    )
    parser.add_argument(
        "--title-legacy",
        type=str,
        default="Hypervisor vs SITL delays",
        help="Figure title (legacy mode)",
    )
    args = parser.parse_args()

    step_rows: Optional[Dict[str, np.ndarray]] = None
    flight_modes: Optional[List[str]] = None
    if args.step_csv is not None:
        if not args.step_csv.is_file():
            raise FileNotFoundError(f"Step CSV not found: {args.step_csv}")
        with args.step_csv.open("r", encoding="utf-8", newline="") as f:
            head = f.readline()
        if "step_idx" not in head or "hypervisor_rx_to_tx_sec" not in head:
            raise ValueError(f"Expected drone_*_time_sync_steps.csv: {args.step_csv}")
        step_rows, flight_modes = _load_step_rows(args.step_csv)

    sitl_path: Optional[Path] = None
    if args.sitl_latest:
        found = glob_latest_sitl_trace(args.sitl_instance)
        if not found:
            searched = ", ".join(str(d) for d in _trace_search_dirs())
            raise FileNotFoundError(f"No sitl_rc_telem_{args.sitl_instance}_*.csv in {searched}")
        sitl_path = found
        print("Using SITL trace:", sitl_path, file=sys.stderr)
    elif args.sitl_trace is not None:
        sitl_path = args.sitl_trace.resolve()
        if not sitl_path.is_file():
            raise FileNotFoundError(f"SITL trace not found: {sitl_path}")

    if args.step_csv is None and sitl_path is None:
        raise SystemExit("Provide step CSV and/or --sitl-trace / --sitl-latest.")

    sitl_rows: List[dict] = []
    sitl_fields: Optional[List[str]] = None
    sitl_pair_idx: Optional[np.ndarray] = None
    sitl_deltas: Optional[np.ndarray] = None
    sitl_wall_abs: Optional[np.ndarray] = None
    sitl_mono_rel: Optional[np.ndarray] = None
    sitl_tx_effective: str = args.sitl_tx
    if sitl_path is not None:
        sitl_rows, sitl_fields = _load_sitl_trace_rows(sitl_path)
        sitl_tx_effective, sitl_pair_idx, sitl_deltas, sitl_wall_abs, sitl_mono_rel = resolve_sitl_tx_for_plot(
            sitl_rows,
            requested=args.sitl_tx,
            trace_path=sitl_path,
        )

    if step_rows is None and (
        sitl_deltas is None or (isinstance(sitl_deltas, np.ndarray) and sitl_deltas.size == 0)
    ):
        raise SystemExit("No plottable delay data.")

    w_step = step_rows["py_write_wall_sec"] if step_rows is not None else None
    h_step = step_rows["hypervisor_rx_to_tx_sec"] if step_rows is not None else None
    step_plot_ok = bool(
        w_step is not None
        and h_step is not None
        and np.any(np.isfinite(w_step) & np.isfinite(h_step))
    )
    sitl_wall_ok = bool(
        sitl_wall_abs is not None
        and sitl_deltas is not None
        and sitl_deltas.size > 0
        and np.any(np.isfinite(sitl_wall_abs) & np.isfinite(sitl_deltas))
    )
    sitl_mono_ok = bool(
        sitl_mono_rel is not None
        and sitl_deltas is not None
        and sitl_deltas.size > 0
        and np.any(np.isfinite(sitl_mono_rel) & np.isfinite(sitl_deltas))
    )
    sitl_plot_ok = sitl_wall_ok or sitl_mono_ok

    if args.legacy_split:
        use_wall = False
    elif step_rows is not None and sitl_path is not None and sitl_deltas is not None and sitl_deltas.size > 0:
        use_wall = step_plot_ok and sitl_plot_ok
    elif step_rows is not None:
        use_wall = step_plot_ok
    elif sitl_path is not None and sitl_deltas is not None and sitl_deltas.size > 0:
        use_wall = sitl_plot_ok
    else:
        use_wall = False

    show_win = not args.no_show and bool(os.environ.get("DISPLAY"))
    if show_win:
        try:
            matplotlib.use("TkAgg")
        except Exception:
            show_win = False
            print("TkAgg unavailable; save only.", file=sys.stderr)
    else:
        matplotlib.use("Agg")

    fig = None
    wall_unified = False
    if use_wall:
        t_candidates: List[float] = []
        if step_rows is not None and step_plot_ok:
            w = step_rows["py_write_wall_sec"]
            h = step_rows["hypervisor_rx_to_tx_sec"]
            m = np.isfinite(w) & np.isfinite(h)
            t_candidates.append(float(np.min(w[m])))
        if sitl_wall_ok and sitl_wall_abs is not None and sitl_deltas is not None:
            m_w = np.isfinite(sitl_wall_abs) & np.isfinite(sitl_deltas)
            if np.any(m_w):
                t_candidates.append(float(np.min(sitl_wall_abs[m_w])))

        if not t_candidates:
            if sitl_mono_ok:
                t0 = 0.0
            else:
                use_wall = False
        if use_wall:
            wall_unified = True
            if t_candidates:
                t0 = min(t_candidates)
            else:
                t0 = 0.0
            x_hyp_rel = y_hyp = None
            if step_plot_ok and step_rows is not None:
                w = step_rows["py_write_wall_sec"]
                h = step_rows["hypervisor_rx_to_tx_sec"]
                m = np.isfinite(w) & np.isfinite(h)
                x_hyp_rel = w[m] - t0
                y_hyp = h[m]
            x_sitl_rel = y_sitl = None
            sitl_x_from_mono = sitl_mono_ok and not sitl_wall_ok
            if sitl_wall_ok and sitl_wall_abs is not None and sitl_deltas is not None:
                m = np.isfinite(sitl_wall_abs) & np.isfinite(sitl_deltas)
                x_sitl_rel = sitl_wall_abs[m] - t0
                y_sitl = sitl_deltas[m]
            elif sitl_mono_ok and sitl_mono_rel is not None and sitl_deltas is not None:
                m = np.isfinite(sitl_mono_rel) & np.isfinite(sitl_deltas)
                x_sitl_rel = sitl_mono_rel[m]
                y_sitl = sitl_deltas[m]

            if x_hyp_rel is None and x_sitl_rel is None:
                wall_unified = False
                use_wall = False
            else:
                fig = _build_figure_wall(
                    t0=t0,
                    x_hyp_rel=x_hyp_rel,
                    y_hyp=y_hyp,
                    x_sitl_rel=x_sitl_rel,
                    y_sitl=y_sitl,
                    flight_modes=flight_modes,
                    x_mode_abs=step_rows["py_write_wall_sec"] if step_rows is not None else None,
                    title=args.title,
                    sitl_tx_label=sitl_tx_effective,
                    sitl_trace_basename=str(sitl_path) if sitl_path else None,
                    sitl_x_from_mono=sitl_x_from_mono,
                )

    if fig is None:
        wall_unified = False
        fig = _build_figure_legacy(
            step_rows=step_rows,
            flight_modes=flight_modes,
            sitl_pair_idx=sitl_pair_idx
            if sitl_deltas is not None and sitl_deltas.size
            else None,
            sitl_deltas=sitl_deltas if sitl_deltas is not None and sitl_deltas.size else None,
            sitl_trace_basename=str(sitl_path) if sitl_path else None,
            sitl_tx_label=sitl_tx_effective,
            title=args.title_legacy,
        )

    import matplotlib.pyplot as plt

    out_path = args.out
    if out_path is None:
        out_path = _default_out_path(
            args.step_csv,
            sitl_path,
            sitl_tx_effective if sitl_path else args.sitl_tx,
            wall_unified=wall_unified,
        )
    else:
        out_path.parent.mkdir(parents=True, exist_ok=True)

    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    print(f"Saved plot: {out_path}")
    if show_win:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
