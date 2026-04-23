#!/usr/bin/env python3
"""
Launcher for Drone Swarm Simulator v2: SITL + optional Webots + scenario.

Usage:
  python launch_simulation.py                    # interactive: YAML batch or single-run menu (SITL)
  python launch_simulation.py --help
  python launch_simulation.py -s -c leader_forward_back   # SITL-only, no 2D visualizer
  python launch_simulation.py -s -c leader_forward_back --with-2d-visualizer   # SITL + 2D visualizer
  python launch_simulation.py -s -c leader_forward_back --with-2d-visualizer \\
      --viz-fixed-axes --viz-xlim -40 40 --viz-ylim -10 50   # fixed matplotlib window (NED m)
  python launch_simulation.py -s --batch-params   # sequential PID sweep from YAML (SITL-only)
  python launch_simulation.py -s -c leader_forward_back --with-mavproxy-consoles  # MAVProxy GUIs + pkill cleanup

Simulation modes:
  --webots, -w      Webots 3D + SITL
  --sitl-only, -s   SITL only (default)

In interactive mode: optional YAML path for batch runs; empty Enter starts a single-run menu
(scenario, 2D visualizer, drone count). 2D plot uses UDP port 15551.

Scenarios run from project root so they can import core.
"""

import argparse
import json
import logging
import math
import os
from datetime import datetime
import re
import signal
import subprocess
import sys
import time
from typing import List, Optional, Tuple

import yaml

logger = logging.getLogger(__name__)

project_root: str = os.path.dirname(os.path.abspath(__file__))


def _sitl_subprocess_popen_kwargs() -> dict:
    """Give each sim_vehicle its own session so SITL/terminal children share one process group."""
    if sys.platform == "win32":
        return {}
    return {"start_new_session": True}


def _terminate_sitl_process_group(proc: subprocess.Popen) -> None:
    """SIGTERM the whole sim_vehicle process group (e.g. xterm/SITL, MAVProxy if present)."""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.terminate()
        except (ProcessLookupError, PermissionError, OSError):
            pass


def _kill_sitl_process_group(proc: subprocess.Popen) -> None:
    """SIGKILL the whole sim_vehicle group if SIGTERM was not enough."""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.kill()
        except (ProcessLookupError, PermissionError, OSError):
            pass


def _pkill_mavproxy_processes() -> None:
    """Run ``pkill -f mavproxy.py`` (POSIX). Kills all matching MAVProxy on the host."""
    if sys.platform == "win32":
        return
    try:
        completed = subprocess.run(
            ["pkill", "-f", "mavproxy.py"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if completed.returncode == 0:
            logger.info("[Launcher] pkill -f mavproxy.py: terminated matching process(es).")
        else:
            logger.info(
                "[Launcher] pkill -f mavproxy.py: exit %s (no match or already gone).",
                completed.returncode,
            )
    except FileNotFoundError:
        logger.warning("[Launcher] pkill not found; MAVProxy cleanup skipped.")
    except subprocess.TimeoutExpired:
        logger.warning("[Launcher] pkill -f mavproxy.py timed out.")


APM_HOME: str = os.path.join(project_root, "..", "ardupilot")
SIM_VEHICLE_PATH: str = os.path.join(APM_HOME, "Tools", "autotest", "sim_vehicle.py")

DEFAULT_BATCH_PARAMS_PATH = os.path.join(
    "scenarios", "batch_parameters", "user_batch_params.yaml"
)
BATCH_RUN_JSON = "batch_run.json"
BATCH_SITL_PARM_DIR = os.path.join("experiments", "_batch_parms")


def experiments_batch_session_stamp() -> str:
    """Timestamp folder name for one batch series: ``yyyy-mm-dd_hh-mm-ss`` under ``experiments/``."""
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

# Per-drone home: sim_vehicle.py uses -l/--custom-location (lat,lon,alt,heading). We use one base
# and offset East by (drone_index * 2) m so that in a common NED frame drone 0 is at Y=0, 1 at Y=2, etc.
# East offset in degrees = meters_east / (111320 * cos(lat_rad)).
BASE_HOME_LAT = 0.0
BASE_HOME_LON = 0.0
BASE_HOME_ALT = 0.0
METERS_PER_DEGREE_EAST = 111320.0 * math.cos(math.radians(BASE_HOME_LAT))


def _sitl_home_for_drone(drone_index: int) -> str:
    """Build lat,lon,alt,heading string for SITL so local NED Y (East) = drone_index * 2 m in common frame.

    Args:
        drone_index: 0-based drone index (0 -> Y=0, 1 -> Y=2, ...).

    Returns:
        String for sim_vehicle.py -l/--custom-location (e.g. "47.0,8.0,0,0" or "47.0,8.000018,0,0").
    """
    lon_offset_deg = (drone_index * 2.0) / METERS_PER_DEGREE_EAST
    lon = BASE_HOME_LON + lon_offset_deg
    return f"{BASE_HOME_LAT},{lon},{BASE_HOME_ALT},0"


# Scenario id, description, script path relative to project_root, cwd = project_root
SCENARIOS: List[Tuple[str, str, str, str]] = [
    (
        "leader_forward_back",
        "Leader forward-back (leader_forward_back)",
        "scenarios/leader_forward_back.py",
        project_root,
    ),
    (
        "snake_pursuit",
        "Snake pursuit +Y roll leader (snake_pursuit)",
        "scenarios/snake_pursuit.py",
        project_root,
    ),
    (
        "square_pid",
        "Square PID (square_formation)",
        "scenarios/square_formation.py",
        project_root,
    ),
    (
        "snake_distance_ground_follower",
        "Snake leader +Y, follower on ground, distance CSV",
        "scenarios/snake_distance_ground_follower.py",
        project_root,
    ),
]

# Russian labels for the interactive scenario menu (see образец_интерактивного_меню.txt).
INTERACTIVE_SCENARIO_LABELS_RU: Tuple[str, ...] = (
    "Лидер: вперёд–назад",
    "Преследование змейкой (+Y)",
    "Движение формации",
    "Лидер +Y, фолловер на земле, лог расстояния",
)


def _scenario_by_id(scenario_id: str) -> Optional[Tuple[str, str, str, str]]:
    """Return scenario tuple if ``scenario_id`` matches a known scenario."""
    for s in SCENARIOS:
        if s[0] == scenario_id:
            return s
    return None


def _scenario_from_custom_input(raw: str) -> Optional[Tuple[str, str, str, str]]:
    """Resolve custom scenario: known id, or relative path to an existing ``.py`` under project root."""
    text = raw.strip()
    if not text:
        return None
    by_id = _scenario_by_id(text)
    if by_id is not None:
        return by_id
    rel = text.replace("\\", "/")
    candidate = os.path.join(project_root, rel) if not os.path.isabs(rel) else rel
    if os.path.isfile(candidate) and candidate.endswith(".py"):
        sid = os.path.splitext(os.path.basename(candidate))[0]
        desc = f"Свой сценарий ({sid})"
        script_rel = os.path.relpath(candidate, project_root)
        return (sid, desc, script_rel, project_root)
    return None


def _close_sitl_log_files(handles) -> None:
    """Close SITL stdout log handles opened in main(); safe if already closed."""
    for f in handles:
        try:
            f.close()
        except OSError:
            pass


def start_sitl_only(
    proj_root: str,
    num_drones: int = 2,
    param_file: Optional[str] = None,
    log_files=None,
    extra_param_files: Optional[List[str]] = None,
    no_mavproxy: bool = False,
) -> List[subprocess.Popen]:
    """Start SITL instances without Webots.

    Args:
        proj_root: Project root directory path.
        num_drones: Number of drone instances to start.
        param_file: Optional path to ArduPilot parameter file (relative to proj_root).
        log_files: Open text handles for stdout/stderr (one per drone); length must equal num_drones.
        extra_param_files: Additional ``.parm`` paths (relative to proj_root or absolute).
        no_mavproxy: If True, pass ``--no-mavproxy`` (scenarios must use TCP ``5760+10*i``).

    Returns:
        List of started subprocess Popen objects, or empty list on error.
    """
    if log_files is None:
        raise ValueError(
            "log_files is required (open SITL logs in main() and pass them in)."
        )
    if len(log_files) != num_drones:
        raise ValueError(
            f"log_files must have length num_drones ({num_drones}), got {len(log_files)}"
        )
    if not os.path.isfile(SIM_VEHICLE_PATH):
        logger.error(
            "Not found %s; ArduPilot must be at ../ardupilot relative to project.",
            SIM_VEHICLE_PATH,
        )
        return []
    extra_params: List[str] = []
    if param_file:
        p = os.path.join(proj_root, param_file)
        if os.path.isfile(p):
            extra_params.append(f"--add-param-file={os.path.abspath(p)}")
    for rel in extra_param_files or []:
        p = os.path.join(proj_root, rel) if rel and not os.path.isabs(rel) else rel
        if p and os.path.isfile(p):
            extra_params.append(f"--add-param-file={os.path.abspath(p)}")
    processes = []
    BASE_UDP_PORT = 14551
    cwd = APM_HOME if os.path.isdir(APM_HOME) else proj_root
    for i in range(num_drones):
        udp_port = BASE_UDP_PORT + i * 10
        tcp_sitl = 5760 + i * 10
        home_str = _sitl_home_for_drone(i)
        args = [
            sys.executable,
            SIM_VEHICLE_PATH,
            "-v",
            "ArduCopter",
            "-w",
            f"--instance={i}",
            f"--sysid={i + 1}",
            "-l",
            home_str,
        ]
        if no_mavproxy:
            args.append("--no-mavproxy")
            logger.info(
                "[SITL] instance=%s, no MAVProxy; scenario TCP 127.0.0.1:%s, custom_location=%s",
                i,
                tcp_sitl,
                home_str,
            )
        else:
            args.extend(
                [
                    f"--out=127.0.0.1:{udp_port}",
                    "--console",
                ]
            )
            logger.info(
                "[SITL] instance=%s, UDP -> 127.0.0.1:%s, custom_location=%s",
                i,
                udp_port,
                home_str,
            )
        if extra_params:
            args.extend(extra_params)
        log_f = log_files[i]
        proc = subprocess.Popen(
            args,
            cwd=cwd,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            **_sitl_subprocess_popen_kwargs(),
        )
        processes.append(proc)
        time.sleep(5)
    return processes


def start_sitl_webots(
    proj_root: str,
    num_drones: int = 2,
    param_file: Optional[str] = None,
    log_files=None,
    extra_param_files: Optional[List[str]] = None,
) -> List[subprocess.Popen]:
    """Start SITL instances with Webots (webots-python model).

    Args:
        proj_root: Project root directory path.
        num_drones: Number of drone instances to start.
        param_file: Optional path to ArduPilot parameter file (relative to proj_root).
        log_files: Open text handles for stdout/stderr (one per drone); length must equal num_drones.
        extra_param_files: Additional ``.parm`` paths (relative to proj_root or absolute).

    Returns:
        List of started SITL subprocess Popen objects, or empty list on error.
    """
    if log_files is None:
        raise ValueError(
            "log_files is required (open SITL logs in main() and pass them in)."
        )
    if len(log_files) != num_drones:
        raise ValueError(
            f"log_files must have length num_drones ({num_drones}), got {len(log_files)}"
        )
    if not os.path.isfile(SIM_VEHICLE_PATH):
        logger.error("Not found %s", SIM_VEHICLE_PATH)
        return []
    BASE_TCP = 5770
    BASE_UDP = 14551
    extra: List[str] = []
    p0 = os.path.join(proj_root, param_file) if param_file else None
    if p0 and os.path.isfile(p0):
        extra.append(f"--add-param-file={os.path.abspath(p0)}")
    for rel in extra_param_files or []:
        p = os.path.join(proj_root, rel) if rel and not os.path.isabs(rel) else rel
        if p and os.path.isfile(p):
            extra.append(f"--add-param-file={os.path.abspath(p)}")
    processes = []
    cwd = APM_HOME if os.path.isdir(APM_HOME) else proj_root
    for i in range(num_drones):
        tcp_port = BASE_TCP + i * 10
        udp_port = BASE_UDP + i * 10
        home_str = _sitl_home_for_drone(i)
        args = [
            sys.executable,
            SIM_VEHICLE_PATH,
            "-v",
            "ArduCopter",
            "-w",
            "--model",
            "webots-python",
            f"--instance={i}",
            f"--sysid={i + 1}",
            f"--out=127.0.0.1:{tcp_port}",
            f"--out=127.0.0.1:{udp_port}",
            "-l",
            home_str,
        ]
        if extra:
            args.extend(extra)
        logger.info(
            "[SITL] instance=%s, TCP=%s, UDP=%s, custom_location=%s",
            i,
            tcp_port,
            udp_port,
            home_str,
        )
        log_f = log_files[i]
        proc = subprocess.Popen(
            args,
            cwd=cwd,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            **_sitl_subprocess_popen_kwargs(),
        )
        processes.append(proc)
        time.sleep(5)
    return processes


def launch_webots(proj_root: str, num_drones: int = 2) -> Optional[subprocess.Popen]:
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
        logger.error("World not found: %s", INPUT_WORLD)
        return None
    with open(INPUT_WORLD, "r") as f:
        content = f.read()
    insert_marker = "# Insert drones"
    pos = content.find(insert_marker)
    if pos == -1:
        logger.error("Marker # Insert drones not found in world")
        return None
    pos += len(insert_marker)
    dx, dy, z = 2.0, 2.0, 0.0549632125
    drones = []
    for i in range(num_drones):
        row, col = i % 10, i // 10
        x, y = col * dx, row * dy
        iris_block = (
            f"\nIris {{\n  translation {x} {y} {z}\n  rotation 0 1 0 0\n"
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
    logger.info("[Webots] Starting...")
    return subprocess.Popen(["webots", OUTPUT_WORLD], env=env)


def _sanitize_batch_id(raw: str) -> str:
    """Make a filesystem-safe batch directory name component."""
    s = re.sub(r"[^a-zA-Z0-9_.-]+", "_", raw.strip())
    return (s or "batch")[:80]


def _run_batch_from_yaml(args: argparse.Namespace) -> int:
    """Run one launcher subprocess per parameter combination; wait for each to finish.

    Args:
        args: Parsed CLI including ``batch_params`` (path to YAML), ``param_file``,
            ``exchange_hz``, and optional 2D visualizer flags.

    Returns:
        0 if every subprocess exited successfully, else 1.
    """
    from core.batch.param_cli import (
        batch_experiment_label,
        batch_params_to_scenario_argv,
        merge_batch_launch,
    )
    from core.batch.sitl_combo import partition_combo_for_batch, write_sitl_overlay_parm
    from core.batch.user_batch_params import (
        generate_parameter_combinations,
        load_user_batch_yaml,
    )

    yaml_path = args.batch_params
    if not yaml_path:
        logger.error("[Batch] No YAML path.")
        return 1
    if not os.path.isabs(yaml_path):
        yaml_path = os.path.join(project_root, yaml_path)
    try:
        doc = load_user_batch_yaml(yaml_path)
    except FileNotFoundError:
        logger.error("[Batch] YAML not found: %s", yaml_path)
        return 1
    except yaml.YAMLError as exc:
        logger.error("[Batch] YAML parse error in %s: %s", yaml_path, exc)
        return 1
    except (TypeError, ValueError) as exc:
        logger.error("[Batch] Invalid YAML document: %s", exc)
        return 1

    try:
        scenario_id, num_drones, duration = merge_batch_launch(doc, args)
    except ValueError as exc:
        logger.error("[Batch] %s", exc)
        return 1

    known = {s[0] for s in SCENARIOS}
    if scenario_id not in known:
        logger.error(
            "[Batch] Unknown scenario %r (expected one of: %s)",
            scenario_id,
            ", ".join(sorted(known)),
        )
        return 1

    try:
        combos = generate_parameter_combinations(doc)
    except ValueError as exc:
        logger.error("[Batch] Parameter sweep config error: %s", exc)
        return 1

    if duration <= 0:
        logger.error(
            "[Batch] Positive duration required (set launch.duration_sec in YAML or --duration)."
        )
        return 1
    if not combos:
        logger.error("[Batch] No parameter combinations generated from YAML.")
        return 1

    batch_label = _sanitize_batch_id(batch_experiment_label(doc, yaml_path))
    experiments_root = os.path.join(project_root, "experiments")
    os.makedirs(experiments_root, exist_ok=True)
    batch_session = experiments_batch_session_stamp()
    session_abs = os.path.join(experiments_root, batch_session)
    os.makedirs(session_abs, exist_ok=True)
    logger.info(
        "[Batch] Session directory (all runs): experiments/%s", batch_session
    )
    scratch_parm = os.path.join(project_root, BATCH_SITL_PARM_DIR)
    os.makedirs(scratch_parm, exist_ok=True)
    launcher = os.path.abspath(__file__)

    failed = 0
    for run_idx, combo in enumerate(combos, start=1):
        exp_rel = os.path.join(
            "experiments", batch_session, f"batch_{batch_label}_run_{run_idx}"
        )
        exp_abs = os.path.join(project_root, exp_rel)
        os.makedirs(exp_abs, exist_ok=True)

        scenario_combo, sitl_combo = partition_combo_for_batch(combo)
        overlay_rel: Optional[str] = None
        if sitl_combo:
            overlay_abs = os.path.join(
                scratch_parm,
                f"{batch_session}_{batch_label}_run_{run_idx}.parm",
            )
            try:
                write_sitl_overlay_parm(overlay_abs, sitl_combo)
            except ValueError as exc:
                logger.error("[Batch] SITL overlay: %s", exc)
                return 1
            overlay_rel = os.path.relpath(overlay_abs, project_root)

        batch_run_path = os.path.join(exp_abs, BATCH_RUN_JSON)
        with open(batch_run_path, "w", encoding="utf-8") as bf:
            json.dump(
                {
                    "run_index": run_idx,
                    "scenario_id": scenario_id,
                    "params": {k: float(v) for k, v in combo.items()},
                },
                bf,
                indent=2,
            )

        logger.info(
            "[Batch] Run %s/%s -> %s params=%s",
            run_idx,
            len(combos),
            exp_rel,
            combo,
        )
        try:
            sweep_argv = batch_params_to_scenario_argv(scenario_id, scenario_combo)
        except ValueError as exc:
            logger.error("[Batch] %s", exc)
            return 1

        cmd: List[str] = [
            sys.executable,
            launcher,
            "-s",
            "-c",
            scenario_id,
            "-n",
            str(num_drones),
            "--param-file",
            args.param_file,
            "--exchange-hz",
            str(args.exchange_hz),
        ]
        if duration > 0:
            cmd.extend(["--duration", str(duration)])
        cmd.extend(["--experiment-dir", exp_rel])
        if overlay_rel:
            cmd.extend(["--sitl-param-overlay", overlay_rel])
        cmd.extend(sweep_argv)
        # OFAT: fixed PID/SITL args from parent CLI apply to every batch child unless that
        # logical key is part of the current sweep (sweep_argv already set it).
        if scenario_id in ("leader_forward_back", "snake_pursuit"):
            if args.kp is not None and "pid.p_gain" not in scenario_combo:
                cmd.extend(["--kp", str(args.kp)])
            if args.ki is not None and "pid.i_gain" not in scenario_combo:
                cmd.extend(["--ki", str(args.ki)])
            if args.kd is not None and "pid.d_gain" not in scenario_combo:
                cmd.extend(["--kd", str(args.kd)])
            if args.derivative_alpha is not None and "pid.derivative_alpha" not in scenario_combo:
                cmd.extend(["--derivative-alpha", str(args.derivative_alpha)])
        if args.with_2d_visualizer:
            cmd.append("--with-2d-visualizer")
            if args.viz_fixed_axes:
                cmd.append("--viz-fixed-axes")
                if args.viz_xlim is not None:
                    cmd.extend(
                        ["--viz-xlim", str(args.viz_xlim[0]), str(args.viz_xlim[1])]
                    )
                if args.viz_ylim is not None:
                    cmd.extend(
                        ["--viz-ylim", str(args.viz_ylim[0]), str(args.viz_ylim[1])]
                    )
        if args.with_mavproxy_consoles:
            cmd.append("--with-mavproxy-consoles")

        proc = subprocess.run(
            cmd,
            cwd=project_root,
            stdout=sys.stdout,
            stderr=sys.stderr,
        )
        if proc.returncode != 0:
            failed += 1
            logger.error(
                "[Batch] Run %s exited with code %s (continuing with remaining runs)",
                run_idx,
                proc.returncode,
            )

    if failed:
        logger.error(
            "[Batch] Finished with %s failed run(s) out of %s", failed, len(combos)
        )
        return 1
    logger.info("[Batch] All %s run(s) completed successfully.", len(combos))
    return 0


def _prompt_interactive_batch_yaml_path() -> Optional[str]:
    """Ask for batch YAML path; empty input means single interactive run."""
    print()
    print("- Файл YAML с параметрами экспериментов:")
    print("    Относительный путь до файла")
    print(
        "    (Нажмите Enter с пустым полем для интерактивного запуска симуляции "
        "для одного прохода):"
    )
    path = input().strip()
    return path or None


def run_interactive_menu() -> Tuple[bool, Tuple[str, str, str, str], int, bool]:
    """Single-run menu: scenario, 2D visualizer, drone count (SITL only).

    Structure matches ``образец_интерактивного_меню.txt``.

    Returns:
        ``use_webots`` is always False. Also returns scenario tuple, drone count,
        and whether to start the 2D matplotlib visualizer.
    """
    print()
    print("- Выбор сценария:")
    n_builtin = len(INTERACTIVE_SCENARIO_LABELS_RU)
    for i, label in enumerate(INTERACTIVE_SCENARIO_LABELS_RU, 1):
        print(f"    {i}. {label}")
    print(f"    {n_builtin + 1}. Свой сценарий")
    choice = input("    Выбор (стандартно 1): ").strip() or "1"
    choice_n = int(choice) if choice.isdigit() else 1
    max_choice = n_builtin + 1
    if choice_n < 1 or choice_n > max_choice:
        choice_n = 1

    if choice_n <= n_builtin:
        scenario = SCENARIOS[choice_n - 1]
    else:
        scenario = SCENARIOS[0]
        while True:
            custom = input(
                "    Введите id сценария (leader_forward_back, snake_pursuit, square_pid, "
                "snake_distance_ground_follower) "
                "или относительный путь к .py: "
            ).strip()
            found = _scenario_from_custom_input(custom)
            if found is not None:
                scenario = found
                break
            print("    Не найдено. Повторите ввод.")

    print()
    print("- 2D-визуализация:")
    print("    1. Включить")
    print("    2. Не включать")
    viz_choice = input("    Выбор (стандартно 1): ").strip() or "1"
    use_2d_visualizer = viz_choice.strip() != "2"

    print()
    print("- Количество дронов:")
    d = input("    Количество (стандартно 2): ").strip()
    num_drones = int(d) if d.isdigit() and int(d) >= 1 else 2

    return False, scenario, num_drones, use_2d_visualizer


def main() -> None:
    """Parse arguments, start SITL (and optionally Webots), then run scenario.

    If scenario file is missing or SITL cannot be started, exits with code 1.
    Registers signal handlers for SIGINT/SIGTERM to terminate child processes.
    """
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    parser = argparse.ArgumentParser(
        description=(
            "Launcher: SITL + optional Webots + scenario; "
            "use --with-2d-visualizer for live 2D plot"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python launch_simulation.py -s -c leader_forward_back
  python launch_simulation.py -s -c square_pid -n 2
  python launch_simulation.py -s --batch-params
  python launch_simulation.py -s --batch-params path/to/matrix.yaml
        """,
    )
    g = parser.add_mutually_exclusive_group(required=False)
    g.add_argument("-w", "--webots", action="store_true", help="Webots 3D + SITL")
    g.add_argument("-s", "--sitl-only", action="store_true", help="SITL only (default)")
    parser.add_argument(
        "-c",
        "--scenario",
        type=str,
        help="Scenario ID: " + ", ".join(s[0] for s in SCENARIOS),
    )
    parser.add_argument(
        "-n",
        "--drones",
        type=int,
        default=2,
        metavar="N",
        help="Number of drones (default 2)",
    )
    parser.add_argument(
        "--param-file",
        type=str,
        default="config/iris.parm",
        help=(
            "ArduPilot parameter file (default: config/iris.parm). "
            "Used for SITL in all modes (SITL-only, Webots, interactive). "
            "If file is missing, SITL runs with defaults."
        ),
    )
    parser.add_argument(
        "--sitl-param-overlay",
        action="append",
        default=None,
        metavar="PATH",
        help=(
            "Extra ArduPilot .parm file (relative to project root or absolute); "
            "applied after --param-file via additional --add-param-file. Repeatable."
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0,
        metavar="T",
        help=(
            "Experiment duration (s); 0 = no limit. "
            "Passed to all scenarios; leader_forward_back has full support."
        ),
    )
    parser.add_argument(
        "--experiment-dir",
        type=str,
        default=None,
        help=(
            "Experiment log folder. Passed to all scenarios; "
            "leader_forward_back has full support."
        ),
    )
    parser.add_argument(
        "--with-2d-visualizer",
        action="store_true",
        help="Start 2D matplotlib visualizer subprocess before the scenario.",
    )
    parser.add_argument(
        "--viz-fixed-axes",
        action="store_true",
        help=(
            "With --with-2d-visualizer: do not auto pan/zoom the plot; "
            "pass --fixed-axes to the visualizer (optional --viz-xlim / --viz-ylim)."
        ),
    )
    parser.add_argument(
        "--viz-xlim",
        nargs=2,
        type=float,
        metavar=("MIN", "MAX"),
        default=None,
        help="North (X) limits in meters for the 2D visualizer when --viz-fixed-axes is set.",
    )
    parser.add_argument(
        "--viz-ylim",
        nargs=2,
        type=float,
        metavar=("MIN", "MAX"),
        default=None,
        help="East (Y) limits in meters for the 2D visualizer when --viz-fixed-axes is set.",
    )
    parser.add_argument(
        "--exchange-hz",
        type=float,
        default=50.0,
        help=(
            "Coordinate exchange loop rate (Hz); match SITL position stream. "
            "Passed to scenario (default 50)."
        ),
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=None,
        metavar="K",
        help="leader_forward_back / snake_pursuit: follower P gain; forwarded to scenario (--kp).",
    )
    parser.add_argument(
        "--ki",
        type=float,
        default=None,
        metavar="K",
        help="leader_forward_back / snake_pursuit: follower I gain (roll and pitch).",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=None,
        metavar="K",
        help="leader_forward_back / snake_pursuit: follower D gain; forwarded to scenario (--kd).",
    )
    parser.add_argument(
        "--derivative-alpha",
        type=float,
        default=None,
        metavar="A",
        help=(
            "leader_forward_back / snake_pursuit: low-pass on derivative term for follower PIDs "
            "(0..1, 0=no filter); passed as --derivative-alpha to scenario."
        ),
    )
    parser.add_argument(
        "--leader-roll-pwm",
        type=int,
        default=None,
        metavar="PWM",
        help=(
            "snake_pursuit / snake_distance_ground_follower: leader roll RC override; "
            "passed as --leader-roll-pwm (omit to use scenario default 1600)."
        ),
    )
    parser.add_argument(
        "--batch-params",
        nargs="?",
        const=DEFAULT_BATCH_PARAMS_PATH,
        default=None,
        metavar="YAML",
        help=(
            "Sequential batch from YAML (SITL-only). Uses core.batch: launch:/batch: block, "
            "experiment_settings or parameter_sweeps; writes batch_run.json per run; "
            "sitl.ANGLE_MAX via overlay .parm. Each run: child launcher + --experiment-dir "
            "experiments/<yyyy-mm-dd_hh-mm-ss>/batch_<id>_run_<n> (new folder per batch invocation). "
            "If the flag is given without a path, uses "
            f"{DEFAULT_BATCH_PARAMS_PATH}."
        ),
    )
    parser.add_argument(
        "--with-mavproxy-consoles",
        action="store_true",
        help=(
            "SITL-only: start MAVProxy with --console (UDP outs to scenarios). "
            "After each launcher run (normal or interrupt), runs: pkill -f mavproxy.py "
            "— terminates every MAVProxy on this machine matching that pattern. "
            "Incompatible with --webots."
        ),
    )
    args = parser.parse_args()

    if args.with_mavproxy_consoles and args.webots:
        logger.error(
            "[Launcher] --with-mavproxy-consoles is only for SITL-only; do not use --webots."
        )
        sys.exit(1)

    if args.batch_params is not None:
        if args.webots:
            logger.error(
                "[Batch] --batch-params works only in SITL-only mode; do not use --webots."
            )
            sys.exit(1)
        sys.exit(_run_batch_from_yaml(args))

    if not args.webots and not args.sitl_only:
        yaml_path = _prompt_interactive_batch_yaml_path()
        if yaml_path:
            args.batch_params = yaml_path
            sys.exit(_run_batch_from_yaml(args))
        use_webots, scenario, num_drones, use_2d_visualizer = run_interactive_menu()
    else:
        use_webots = args.webots
        num_drones = args.drones
        scenario = (
            next((s for s in SCENARIOS if s[0] == args.scenario), SCENARIOS[0])
            if args.scenario
            else SCENARIOS[0]
        )
        use_2d_visualizer = args.with_2d_visualizer

    _, scenario_desc, script_rel, scenario_cwd = scenario
    if scenario[0] == "snake_distance_ground_follower" and num_drones != 2:
        logger.info(
            "[Launcher] Scenario snake_distance_ground_follower requires exactly 2 drones; "
            "overriding num_drones from %s to 2.",
            num_drones,
        )
        num_drones = 2
    script_path = (
        os.path.join(project_root, script_rel)
        if not os.path.isabs(script_rel)
        else script_rel
    )
    if not os.path.isfile(script_path):
        logger.error("Scenario not found: %s", script_path)
        sys.exit(1)

    sitl_direct_tcp = (not use_webots) and (not args.with_mavproxy_consoles)

    sitl_log_dir = os.path.join(project_root, "logs", "sitl")
    os.makedirs(sitl_log_dir, exist_ok=True)
    sitl_log_basename = "sitl_webots_instance" if use_webots else "sitl_instance"
    sitl_log_files = []
    for i in range(num_drones):
        log_path = os.path.join(sitl_log_dir, f"{sitl_log_basename}_{i}.log")
        sitl_log_files.append(open(log_path, "w", encoding="utf-8"))

    processes = []
    if use_webots:
        webots_proc = launch_webots(project_root, num_drones)
        if webots_proc:
            processes.append(webots_proc)
            time.sleep(5)
        sitl_procs = start_sitl_webots(
            project_root,
            num_drones,
            args.param_file,
            log_files=sitl_log_files,
            extra_param_files=args.sitl_param_overlay or [],
        )
        processes.extend(sitl_procs)
    else:
        sitl_procs = start_sitl_only(
            project_root,
            num_drones,
            args.param_file,
            log_files=sitl_log_files,
            extra_param_files=args.sitl_param_overlay or [],
            no_mavproxy=sitl_direct_tcp,
        )
        processes.extend(sitl_procs)

    if not sitl_procs:
        logger.error(
            "[Launcher] SITL did not start (check %s and ../ardupilot).",
            SIM_VEHICLE_PATH,
        )
        _sitl_cleanup_set = frozenset(sitl_procs)
        for p in processes:
            try:
                if p in _sitl_cleanup_set:
                    _terminate_sitl_process_group(p)
                else:
                    p.terminate()
            except Exception:
                pass
        time.sleep(1)
        for p in processes:
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                if p in _sitl_cleanup_set:
                    _kill_sitl_process_group(p)
                else:
                    p.kill()
        _close_sitl_log_files(sitl_log_files)
        if not use_webots and args.with_mavproxy_consoles:
            _pkill_mavproxy_processes()
        sys.exit(1)

    if not processes:
        _close_sitl_log_files(sitl_log_files)
        sys.exit(1)

    sitl_boot_wait_sec = 18 if sitl_direct_tcp else 10
    logger.info(
        "[Launcher] Waiting for SITL (arm/takeoff); sleep %s s "
        "(longer for direct TCP / --no-mavproxy on slow hosts).",
        sitl_boot_wait_sec,
    )
    time.sleep(sitl_boot_wait_sec)

    if use_2d_visualizer:
        visualizer_script = os.path.join(
            project_root, "visualizer", "drone_position_visualizer.py"
        )
        if os.path.isfile(visualizer_script):
            viz_cmd = [sys.executable, "visualizer/drone_position_visualizer.py"]
            if args.viz_fixed_axes:
                viz_cmd.append("--fixed-axes")
                if args.viz_xlim is not None:
                    viz_cmd.extend(
                        ["--xlim", str(args.viz_xlim[0]), str(args.viz_xlim[1])]
                    )
                if args.viz_ylim is not None:
                    viz_cmd.extend(
                        ["--ylim", str(args.viz_ylim[0]), str(args.viz_ylim[1])]
                    )
            visualizer_proc = subprocess.Popen(
                viz_cmd,
                cwd=project_root,
                stdout=sys.stdout,
                stderr=sys.stderr,
            )
            processes.append(visualizer_proc)
            logger.info("[Launcher] Started 2D visualizer subprocess.")
        else:
            logger.warning(
                "[Launcher] 2D visualizer script not found: %s", visualizer_script
            )

    logger.info("[Scenario] Starting: %s", scenario_desc)
    scenario_cmd = [sys.executable, script_rel, "--drones", str(num_drones)]
    scenario_env = os.environ.copy()
    if not use_webots and not args.with_mavproxy_consoles:
        scenario_env["DRONE_SWARM_SITL_DIRECT_TCP"] = "1"
    if args.duration > 0:
        scenario_cmd.extend(["--duration", str(args.duration)])
    if args.experiment_dir:
        scenario_cmd.extend(["--experiment-dir", args.experiment_dir])
    exchange_hz = args.exchange_hz
    if exchange_hz > 0:
        scenario_cmd.extend(["--exchange-hz", str(exchange_hz)])
    if scenario[0] in ("leader_forward_back", "snake_pursuit"):
        if args.kp is not None:
            scenario_cmd.extend(["--kp", str(args.kp)])
        if args.ki is not None:
            scenario_cmd.extend(["--ki", str(args.ki)])
        if args.kd is not None:
            scenario_cmd.extend(["--kd", str(args.kd)])
    elif any(getattr(args, x, None) is not None for x in ("kp", "ki", "kd")):
        logger.warning(
            "[Launcher] --kp/--ki/--kd apply only to leader_forward_back and snake_pursuit; ignoring."
        )
    if args.derivative_alpha is not None:
        if scenario[0] in ("leader_forward_back", "snake_pursuit"):
            scenario_cmd.extend(["--derivative-alpha", str(args.derivative_alpha)])
        else:
            logger.warning(
                "[Launcher] --derivative-alpha applies only to leader_forward_back and snake_pursuit; ignoring."
            )
    if args.leader_roll_pwm is not None:
        if scenario[0] in ("snake_pursuit", "snake_distance_ground_follower"):
            scenario_cmd.extend(["--leader-roll-pwm", str(args.leader_roll_pwm)])
        else:
            logger.warning(
                "[Launcher] --leader-roll-pwm applies only to snake_pursuit and "
                "snake_distance_ground_follower; ignoring."
            )
    scenario_proc = subprocess.Popen(
        scenario_cmd,
        cwd=scenario_cwd,
        stdout=sys.stdout,
        stderr=sys.stderr,
        env=scenario_env,
    )
    processes.append(scenario_proc)

    _sitl_proc_set = frozenset(sitl_procs)
    _pkill_mavproxy_after_run = bool(not use_webots and args.with_mavproxy_consoles)

    def shutdown(signum: Optional[int] = None, frame: Optional[object] = None) -> None:
        logger.info("[Launcher] Stopping processes...")
        for p in processes:
            try:
                if p in _sitl_proc_set:
                    _terminate_sitl_process_group(p)
                else:
                    p.terminate()
            except Exception:
                pass
        time.sleep(1)
        for p in processes:
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                if p in _sitl_proc_set:
                    _kill_sitl_process_group(p)
                else:
                    p.kill()
        _close_sitl_log_files(sitl_log_files)
        if _pkill_mavproxy_after_run:
            _pkill_mavproxy_processes()
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
