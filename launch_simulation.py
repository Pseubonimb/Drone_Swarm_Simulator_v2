#!/usr/bin/env python3
"""
Launcher for Drone Swarm Simulator v2: SITL + optional Webots + scenario.

Usage:
  python launch_simulation.py                    # interactive menu
  python launch_simulation.py --help
  python launch_simulation.py -s -c leader_forward_back   # SITL-only, no 2D visualizer

Simulation modes:
  --webots, -w      Webots 3D + SITL
  --sitl-only, -s   SITL only (default)

Scenarios run from project root so they can import core.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from typing import List, Optional, Tuple

project_root: str = os.path.dirname(os.path.abspath(__file__))
APM_HOME: str = os.path.join(project_root, "..", "ardupilot")
SIM_VEHICLE_PATH: str = os.path.join(APM_HOME, "Tools", "autotest", "sim_vehicle.py")

# Scenario id, description, script path relative to project_root, cwd = project_root
SCENARIOS: List[Tuple[str, str, str, str]] = [
    (
        "leader_forward_back",
        "Leader forward-back (leader_forward_back)",
        "scenarios/leader_forward_back.py",
        project_root,
    ),
    (
        "square_pid",
        "Square PID (square_formation)",
        "scenarios/square_formation.py",
        project_root,
    ),
]


def start_sitl_only(
    proj_root: str,
    num_drones: int = 2,
    param_file: Optional[str] = None,
) -> List[subprocess.Popen]:
    """Start SITL instances without Webots.

    Args:
        proj_root: Project root directory path.
        num_drones: Number of drone instances to start.
        param_file: Optional path to ArduPilot parameter file (relative to proj_root).

    Returns:
        List of started subprocess Popen objects, or empty list on error.
    """
    if not os.path.isfile(SIM_VEHICLE_PATH):
        print(f"Error: not found {SIM_VEHICLE_PATH}")
        print("ArduPilot must be at ../ardupilot relative to project.")
        return []
    extra_params = []
    if param_file:
        p = os.path.join(proj_root, param_file)
        if os.path.isfile(p):
            extra_params.append(f"--add-param-file={os.path.abspath(p)}")
    processes = []
    BASE_UDP_PORT = 14551
    cwd = APM_HOME if os.path.isdir(APM_HOME) else proj_root
    for i in range(num_drones):
        udp_port = BASE_UDP_PORT + i * 10
        args = [
            sys.executable,
            SIM_VEHICLE_PATH,
            "-v", "ArduCopter", "-w",
            f"--instance={i}",
            f"--sysid={i + 1}",
            f"--out=127.0.0.1:{udp_port}",
            "--console",
        ]
        if extra_params:
            args.extend(extra_params)
        print(f"[SITL] instance={i}, UDP -> 127.0.0.1:{udp_port}")
        proc = subprocess.Popen(args, cwd=cwd)
        processes.append(proc)
        time.sleep(5)
    return processes


def start_sitl_webots(
    proj_root: str,
    num_drones: int = 2,
    param_file: Optional[str] = None,
) -> Tuple[List[subprocess.Popen], None]:
    """Start SITL instances with Webots (webots-python model).

    Args:
        proj_root: Project root directory path.
        num_drones: Number of drone instances to start.
        param_file: Optional path to ArduPilot parameter file (relative to proj_root).

    Returns:
        Tuple of (list of SITL Popen processes, None placeholder).
    """
    if not os.path.isfile(SIM_VEHICLE_PATH):
        print(f"Error: not found {SIM_VEHICLE_PATH}")
        return [], None
    BASE_TCP = 5770
    BASE_UDP = 14551
    p = os.path.join(proj_root, param_file) if param_file else None
    extra = [f"--add-param-file={os.path.abspath(p)}"] if p and os.path.isfile(p) else []
    processes = []
    cwd = APM_HOME if os.path.isdir(APM_HOME) else proj_root
    for i in range(num_drones):
        tcp_port = BASE_TCP + i * 10
        udp_port = BASE_UDP + i * 10
        args = [
            sys.executable,
            SIM_VEHICLE_PATH,
            "-v", "ArduCopter", "-w",
            "--model", "webots-python",
            f"--instance={i}",
            f"--sysid={i + 1}",
            f"--out=127.0.0.1:{tcp_port}",
            f"--out=127.0.0.1:{udp_port}",
            "--console",
        ]
        if extra:
            args.extend(extra)
        print(f"[SITL] instance={i}, TCP={tcp_port}, UDP={udp_port}")
        proc = subprocess.Popen(args, cwd=cwd)
        processes.append(proc)
        time.sleep(5)
    return processes, None


def launch_webots(
    proj_root: str, num_drones: int = 2
) -> Optional[subprocess.Popen]:
    """Start Webots world with configured drone count.

    Args:
        proj_root: Project root directory path.
        num_drones: Number of drones to insert into the world.

    Returns:
        Popen instance of Webots process, or None if world file not found or error.
    """
    WORLDS_DIR = os.path.join(proj_root, "worlds")
    INPUT_WORLD = os.path.join(WORLDS_DIR, "irisAuto.wbt")
    OUTPUT_WORLD = os.path.join(WORLDS_DIR, "temp_world.wbt")
    if not os.path.isfile(INPUT_WORLD):
        print(f"Error: world not found {INPUT_WORLD}")
        return None
    with open(INPUT_WORLD, "r") as f:
        content = f.read()
    insert_marker = "# Insert drones"
    pos = content.find(insert_marker)
    if pos == -1:
        print("Marker # Insert drones not found in world")
        return None
    pos += len(insert_marker)
    dx, dy, z = 2.0, 2.0, 0.0549632125
    drones = []
    for i in range(num_drones):
        row, col = i % 10, i // 10
        x, y = col * dx, row * dy
        iris_block = (
            f'\nIris {{\n  translation {x} {y} {z}\n  rotation 0 1 0 0\n'
            f'  name "Iris_{i}"\n  controller "ardupilot_vehicle_controller"\n'
            f'  controllerArgs [ "--instance" "{i}" "--motors" '
            '"m1_motor, m2_motor, m3_motor, m4_motor" ]\n}}\n'
        )
        drones.append(iris_block)
    new_content = content[:pos] + "".join(drones) + content[pos:]
    with open(OUTPUT_WORLD, "w") as f:
        f.write(new_content)
    env = os.environ.copy()
    env["WEBOTS_PROTO_PATH"] = os.path.join(proj_root, "protos")
    print("[Webots] Starting...")
    return subprocess.Popen(["webots", OUTPUT_WORLD], env=env)


def run_interactive_menu() -> Tuple[bool, Tuple[str, str, str, str], int]:
    """Interactive menu for mode, scenario, drone count.

    Returns:
        Tuple of (use_webots, scenario_tuple, num_drones). scenario_tuple is
        (scenario_id, description, script_path, cwd).
    """
    print("\n=== Simulation launcher ===\n")
    print("1. Mode:")
    print("   1) Webots 3D + SITL")
    print("   2) SITL only (no Webots)")
    mode = input("   Choice [1/2, default 2]: ").strip() or "2"
    use_webots = mode == "1"
    print("\n2. Scenario:")
    for i, (sid, desc, _, _) in enumerate(SCENARIOS, 1):
        print(f"   {i}) {sid}: {desc}")
    idx = input(f"   Number [1-{len(SCENARIOS)}, default 1]: ").strip()
    idx = int(idx) if idx.isdigit() else 1
    scenario = SCENARIOS[idx - 1] if 1 <= idx <= len(SCENARIOS) else SCENARIOS[0]
    print("\n3. Number of drones:")
    d = input("   Count [default 2]: ").strip()
    num_drones = int(d) if d.isdigit() and int(d) >= 1 else 2
    return use_webots, scenario, num_drones


def main() -> None:
    """Parse arguments, start SITL (and optionally Webots), then run scenario.

    If scenario file is missing or SITL cannot be started, exits with code 1.
    Registers signal handlers for SIGINT/SIGTERM to terminate child processes.
    """
    parser = argparse.ArgumentParser(
        description="Launcher: SITL + optional Webots + scenario; use --with-2d-visualizer for live 2D plot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python launch_simulation.py -s -c leader_forward_back
  python launch_simulation.py -s -c square_pid -n 2
        """,
    )
    g = parser.add_mutually_exclusive_group(required=False)
    g.add_argument("-w", "--webots", action="store_true", help="Webots 3D + SITL")
    g.add_argument("-s", "--sitl-only", action="store_true", help="SITL only (default)")
    parser.add_argument(
        "-c", "--scenario", type=str,
        help="Scenario ID: " + ", ".join(s[0] for s in SCENARIOS),
    )
    parser.add_argument("-n", "--drones", type=int, default=2, metavar="N", help="Number of drones (default 2)")
    parser.add_argument(
        "--param-file",
        type=str,
        default=None,
        help=(
            "Optional ArduPilot parameter file (e.g. config/iris.parm). "
            "If not provided or file missing, SITL runs with defaults."
        ),
    )
    parser.add_argument(
        "--duration", type=float, default=0, metavar="T",
        help="Experiment duration (s); 0 = no limit. Passed to all scenarios; leader_forward_back has full support.",
    )
    parser.add_argument(
        "--experiment-dir", type=str, default=None,
        help="Experiment log folder. Passed to all scenarios; leader_forward_back has full support.",
    )
    parser.add_argument(
        "--with-2d-visualizer",
        action="store_true",
        help="Start 2D matplotlib visualizer subprocess before the scenario.",
    )
    args = parser.parse_args()

    if args.webots or args.sitl_only:
        use_webots = args.webots
        num_drones = args.drones
        scenario = (
            next((s for s in SCENARIOS if s[0] == args.scenario), SCENARIOS[0])
            if args.scenario
            else SCENARIOS[0]
        )
    else:
        use_webots, scenario, num_drones = run_interactive_menu()

    _, scenario_desc, script_rel, scenario_cwd = scenario
    script_path = (
        os.path.join(project_root, script_rel)
        if not os.path.isabs(script_rel)
        else script_rel
    )
    if not os.path.isfile(script_path):
        print(f"Error: scenario not found: {script_path}")
        sys.exit(1)

    processes = []
    if use_webots:
        webots_proc = launch_webots(project_root, num_drones)
        if webots_proc:
            processes.append(webots_proc)
            time.sleep(5)
    if use_webots:
        sitl_procs, _ = start_sitl_webots(project_root, num_drones, args.param_file)
        processes.extend(sitl_procs)
    else:
        sitl_procs = start_sitl_only(project_root, num_drones, args.param_file)
        processes.extend(sitl_procs)

    if not processes:
        sys.exit(1)

    print("[Launcher] Waiting for SITL (arm/takeoff)...")
    time.sleep(8)

    if getattr(args, "with_2d_visualizer", False):
        visualizer_script = os.path.join(project_root, "visualizer", "drone_position_visualizer.py")
        if os.path.isfile(visualizer_script):
            visualizer_proc = subprocess.Popen(
                [sys.executable, "visualizer/drone_position_visualizer.py"],
                cwd=project_root,
                stdout=sys.stdout,
                stderr=sys.stderr,
            )
            processes.append(visualizer_proc)
            print("[Launcher] Started 2D visualizer subprocess.")
        else:
            print(f"[Launcher] 2D visualizer script not found: {visualizer_script}")

    print(f"[Scenario] Starting: {scenario_desc}")
    scenario_cmd = [sys.executable, script_rel, "--drones", str(num_drones)]
    if getattr(args, "duration", 0) > 0:
        scenario_cmd.extend(["--duration", str(args.duration)])
    if getattr(args, "experiment_dir", None):
        scenario_cmd.extend(["--experiment-dir", args.experiment_dir])
    scenario_proc = subprocess.Popen(
        scenario_cmd,
        cwd=scenario_cwd,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )
    processes.append(scenario_proc)

    def shutdown(signum=None, frame=None):
        print("\n[Launcher] Stopping processes...")
        for p in processes:
            try:
                p.terminate()
            except Exception:
                pass
        time.sleep(1)
        for p in processes:
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        scenario_proc.wait()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == "__main__":
    main()
