#!/usr/bin/env python3
"""
Real-time 2D drone position visualizer (any number of drones).

Receives NED coordinates via UDP from scenarios (via position_publisher).
Coordinates match CoordsMonitor (x=North, y=East, z=Down). Drones appear on the
plot as they appear in the stream.

Run:
  python visualizer/drone_position_visualizer.py
  python visualizer/drone_position_visualizer.py --port 15551

Order does not matter: start visualizer first or simulation first; the plot
updates when the first data arrives.
"""
import argparse
import json
import logging
import socket
import sys
import threading
import warnings
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

warnings.filterwarnings("ignore", message=".*fixed.*aspect.*adjustable.*", category=UserWarning)

try:
    import matplotlib
    matplotlib.use("TkAgg")
    try:
        matplotlib.set_loglevel("warning")
    except AttributeError:
        pass
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

DEFAULT_PORT = 15551
BUFFER_SIZE = 4096


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed namespace with port, trail, and interval attributes.
    """
    parser = argparse.ArgumentParser(
        description="Real-time drone position visualizer (UDP, NED coordinates)"
    )
    parser.add_argument(
        "--port", "-p", type=int, default=DEFAULT_PORT,
        help=f"UDP port to listen on (default {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--trail", type=int, default=30,
        help="Number of last points to show as trail (0 = current only)",
    )
    parser.add_argument(
        "--interval", type=float, default=0.1,
        help="Plot update interval in seconds",
    )
    return parser.parse_args()


class PositionReceiver:
    """Listens on UDP and accumulates drone positions and loop rates.

    Thread-safe: receive loop runs in a daemon thread; get_positions/get_rates
    return copies under lock.
    """

    MAX_HISTORY = 500

    def __init__(self, port: int) -> None:
        """Bind to the given UDP port (socket created in start()).

        Args:
            port: UDP port number to listen on.
        """
        self.port = port
        self.positions: Dict[Any, Dict[str, float]] = {}
        self.history: Dict[Any, List[Tuple[float, float, float]]] = {}
        self.rates: Dict[str, Optional[float]] = {}
        self.lock = threading.Lock()
        self.running = False
        self.sock: Optional[socket.socket] = None

    def start(self) -> None:
        """Bind socket and start receive thread."""
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(0.5)
        t = threading.Thread(target=self._recv_loop, daemon=True)
        t.start()

    def stop(self) -> None:
        """Stop receive thread and close socket. Receiver thread exits on next loop."""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _recv_loop(self) -> None:
        while self.running and self.sock:
            try:
                data, _ = self.sock.recvfrom(BUFFER_SIZE)
                obj = json.loads(data.decode("utf-8"))
                pos_dict = obj.get("positions", {})
                rates_dict = obj.get("rates")
                with self.lock:
                    for drone_id_str, p in pos_dict.items():
                        drone_id = int(drone_id_str) if drone_id_str.isdigit() else drone_id_str
                        x = float(p.get("x", 0))
                        y = float(p.get("y", 0))
                        z = float(p.get("z", 0))
                        self.positions[drone_id] = {"x": x, "y": y, "z": z}
                        if drone_id not in self.history:
                            self.history[drone_id] = []
                        h = self.history[drone_id]
                        h.append((x, y, z))
                        if len(h) > self.MAX_HISTORY:
                            self.history[drone_id] = h[-self.MAX_HISTORY:]
                    if rates_dict is not None:
                        self.rates = {
                            "follower_hz": rates_dict.get("follower_hz"),
                            "exchange_hz": rates_dict.get("exchange_hz"),
                            "webots_step_hz": rates_dict.get("webots_step_hz"),
                        }
            except socket.timeout:
                continue
            except (json.JSONDecodeError, KeyError, ValueError):
                pass
            except OSError:
                if not self.running:
                    break
                raise
            except Exception:
                if self.running:
                    raise

    def get_positions(
        self,
    ) -> Tuple[Dict[Any, Dict[str, float]], Dict[Any, List[Tuple[float, float, float]]]]:
        """Return copy of positions and history (thread-safe).

        Returns:
            Tuple of (positions dict, history dict). Positions: drone_id -> {x,y,z};
            history: drone_id -> list of (x,y,z) tuples.
        """
        with self.lock:
            return self.positions.copy(), {k: list(v) for k, v in self.history.items()}

    def get_rates(self) -> Dict[str, Optional[float]]:
        """Return copy of rates (thread-safe).

        Returns:
            Dict with keys follower_hz, exchange_hz, webots_step_hz (values may be None).
        """
        with self.lock:
            return self.rates.copy()


def run_visualizer(port: int, trail: int, interval: float) -> None:
    """Run the matplotlib window and animation loop.

    Args:
        port: UDP port to listen on for position updates.
        trail: Number of last points to show as trail (0 = current only).
        interval: Plot update interval in seconds.
    """
    if not HAS_MATPLOTLIB:
        logger.error("matplotlib required. Install: pip install matplotlib")
        sys.exit(1)

    receiver = PositionReceiver(port)
    receiver.start()
    logger.info("Listening for drone positions on UDP port %s", port)
    logger.info("Start simulation to see positions. Supports any number of drones.")
    logger.info("Press Ctrl+C to stop.")

    fig, (ax, ax_rates) = plt.subplots(2, 1, figsize=(9, 9), gridspec_kw={"height_ratios": [3, 1]})
    ax.set_xlabel("X (m, North, NED)")
    ax.set_ylabel("Y (m, East, NED)")
    ax.set_title("Drone positions (real-time X–Y)")
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="box")
    ax_rates.set_axis_off()
    rates_text = ax_rates.text(
        0.5, 0.5, "", transform=ax_rates.transAxes, fontsize=10,
        verticalalignment="center", horizontalalignment="center",
        family="monospace",
    )

    PALETTE = [
        "#2563eb", "#dc2626", "#16a34a", "#ca8a04", "#9333ea",
        "#0d9488", "#ea580c", "#4f46e5", "#059669", "#b91c1c",
        "#1d4ed8", "#65a30d", "#c026d3", "#0e7490", "#d97706",
    ]

    def color_for_drone(drone_id: Any) -> str:
        return PALETTE[int(drone_id) % len(PALETTE)]

    def label_for_drone(drone_id: Any) -> str:
        if drone_id == 1:
            return "Drone 1 (leader)"
        if drone_id == 2:
            return "Drone 2 (follower)"
        return f"Drone {drone_id}"

    lines: Dict[Any, Any] = {}
    points: Dict[Any, Any] = {}

    def _format_hz(v: Optional[float]) -> str:
        if v is None:
            return "—"
        try:
            return f"{float(v):.1f}"
        except (TypeError, ValueError):
            return "—"

    def init() -> List[Any]:
        return list(lines.values()) + list(points.values())

    def animate(_: Any) -> List[Any]:
        pos, hist = receiver.get_positions()
        rates = receiver.get_rates()
        vals = [
            rates.get("follower_hz"),
            rates.get("exchange_hz"),
            rates.get("webots_step_hz"),
        ]
        vals = [v for v in vals if v is not None]
        min_hz = min(vals) if vals else None
        rates_str = (
            f"follower_loop: {_format_hz(rates.get('follower_hz'))} Hz  |  "
            f"exchange_loop: {_format_hz(rates.get('exchange_hz'))} Hz  |  "
            f"Webots step: {_format_hz(rates.get('webots_step_hz'))} Hz  |  "
            f"Min loop: {_format_hz(min_hz)} Hz  |  Drones: {len(hist)}"
        )
        rates_text.set_text(rates_str)

        trail_n = max(0, trail)
        for drone_id, pts in list(hist.items()):
            if drone_id not in lines:
                col = color_for_drone(drone_id)
                lbl = label_for_drone(drone_id)
                lines[drone_id], = ax.plot(
                    [], [], "-", color=col, linewidth=1.5, alpha=0.6, label=lbl
                )
                points[drone_id], = ax.plot([], [], "o", color=col, markersize=10)

            if trail_n > 0 and len(pts) > trail_n:
                pts = pts[-trail_n:]
            elif trail_n == 0 and pts:
                pts = pts[-1:]

            if pts:
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                lines[drone_id].set_data(xs, ys)
                points[drone_id].set_data([xs[-1]], [ys[-1]])

        if lines:
            handles = [lines[did] for did in sorted(lines.keys())]
            labels = [label_for_drone(did) for did in sorted(lines.keys())]
            ax.legend(handles, labels, loc="upper left", fontsize=9)

        all_x, all_y = [], []
        for pts in hist.values():
            if pts:
                trail_pts = pts[-trail_n:] if trail_n > 0 else pts[-1:]
                all_x.extend(p[0] for p in trail_pts)
                all_y.extend(p[1] for p in trail_pts)
        if all_x and all_y:
            margin = 2.0
            x_min, x_max = min(all_x) - margin, max(all_x) + margin
            y_min, y_max = min(all_y) - margin, max(all_y) + margin
            range_x = x_max - x_min
            range_y = y_max - y_min
            max_range = max(range_x, range_y, 1.0)
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            half = max_range / 2
            ax.set_xlim(x_center - half, x_center + half)
            ax.set_ylim(y_center - half, y_center + half)

        return list(lines.values()) + list(points.values())

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        interval=int(interval * 1000),
        blit=False,
        cache_frame_data=False,
    )
    plt.tight_layout()
    try:
        plt.show()
    finally:
        receiver.stop()


def main() -> None:
    """Entry point: parse args, configure logging, run visualizer until Ctrl+C."""
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        run_visualizer(args.port, args.trail, args.interval)
    except KeyboardInterrupt:
        logger.info("Visualizer stopped.")


if __name__ == "__main__":
    main()
