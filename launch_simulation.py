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
  python launch_simulation.py -s -c leader_forward_back  # pkill MAVProxy before + after run; MAVProxy GUIs default
  python launch_simulation.py -s -c leader_forward_back --drones 3   # three SITL; no waf (default)

By default ``sim_vehicle`` uses ``-N`` (launcher skips waf). Build once under ../ardupilot or use
``--sitl-rebuild-waf`` / ``DRONE_SWARM_SITL_ALLOW_WAF_REBUILD=1`` after editing ArduPilot.

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


def _sitl_subprocess_env() -> dict:
    """Environment for ``sim_vehicle`` so nested waf builds see extra ``CXXFLAGS`` (e.g. SITL trace).

    Prepend ``ARDUPILOT_SITL_CXXFLAGS`` to existing ``CXXFLAGS`` when set.

    Sets ``DRONE_SWARM_SITL_TELEM_TRACE_DIR`` to ``<project>/logs/time_sync`` (absolute) unless
    already set, for patched ArduPilot ``AP_SITL_RC_TELEM_TRACE`` CSV output.
    """
    env = os.environ.copy()

    trace_dir = (env.get("DRONE_SWARM_SITL_TELEM_TRACE_DIR") or "").strip()
    if not trace_dir:
        trace_dir = os.path.join(project_root, "logs", "time_sync")
    try:
        os.makedirs(trace_dir, exist_ok=True)
    except OSError as exc:
        logger.warning("[SITL] Could not create trace dir %s: %s", trace_dir, exc)
    env["DRONE_SWARM_SITL_TELEM_TRACE_DIR"] = os.path.abspath(trace_dir)

    extra = (os.environ.get("ARDUPILOT_SITL_CXXFLAGS") or "").strip()
    if not extra:
        return env
    prev = (env.get("CXXFLAGS") or "").strip()
    merged = f"{extra} {prev}".strip() if prev else extra
    env["CXXFLAGS"] = merged
    logger.info(
        "[SITL] Merged CXXFLAGS for sim_vehicle (first ~120 chars): %s",
        merged[:120],
    )
    return env


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
    """Kill stray MAVProxy processes (POSIX): ``mavproxy.py`` and ``MAVProxy`` on the cmdline.

    Called before starting SITL (each launch) and after shutdown when MAVProxy consoles were used.
    """
    if sys.platform == "win32":
        return
    for pattern in ("mavproxy.py", "MAVProxy"):
        try:
            completed = subprocess.run(
                ["pkill", "-f", pattern],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if completed.returncode == 0:
                logger.info(
                    "[Launcher] pkill -f %s: terminated matching process(es).", pattern
                )
            else:
                logger.info(
                    "[Launcher] pkill -f %s: exit %s (no match or already gone).",
                    pattern,
                    completed.returncode,
                )
        except FileNotFoundError:
            logger.warning("[Launcher] pkill not found; MAVProxy cleanup skipped.")
            return
        except subprocess.TimeoutExpired:
            logger.warning("[Launcher] pkill -f %s timed out.", pattern)


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
    (
        "task_assignment_decentralized",
        "Decentralized task assignment (Hungarian) + PID to targets",
        "scenarios/task_assignment_decentralized.py",
        project_root,
    ),
]

# Russian labels for the interactive scenario menu (see образец_интерактивного_меню.txt).
INTERACTIVE_SCENARIO_LABELS_RU: Tuple[str, ...] = (
    "Лидер: вперёд–назад",
    "Преследование змейкой (+Y)",
    "Движение формации",
    "Лидер +Y, фолловер на земле, лог расстояния",
    "Децентрализованное назначение целей (венгерский метод)",
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


def prebuild_ardupilot_sitl_copter() -> None:
    """Run ``waf configure --board sitl`` then ``waf copter`` once under ``../ardupilot``.

    Invoked via ``sys.executable`` plus the bundled ``waf`` script so it behaves like
    ``python waf``. Uses :func:`_sitl_subprocess_env` so optional ``CXXFLAGS`` matches
    ``sim_vehicle``. Call only when explicitly requested (--sitl-rebuild-waf).
    """
    if not os.path.isdir(APM_HOME):
        logger.error("[SITL] Prebuild skipped: ../ardupilot not found (%s)", APM_HOME)
        raise FileNotFoundError(APM_HOME)
    waf_py = os.path.join(APM_HOME, "waf")
    if not os.path.isfile(waf_py):
        logger.error("[SITL] Prebuild skipped: missing waf script in %s", APM_HOME)
        raise FileNotFoundError(waf_py)

    cwd = APM_HOME
    env = _sitl_subprocess_env()
    waf_cmd = [sys.executable, waf_py]

    logger.info("[SITL] Running single waf configure+build copter under %s", cwd)
    subprocess.run(
        waf_cmd + ["configure", "--board", "sitl"],
        cwd=cwd,
        env=env,
        check=True,
    )
    subprocess.run(waf_cmd + ["copter"], cwd=cwd, env=env, check=True)
    logger.info("[SITL] Prebuild finished (configure + copter).")


def _sitl_vehicle_bin_dir() -> str:
    return os.path.join(APM_HOME, "build", "sitl", "bin")


def sitl_vehicle_binary_maybe_present() -> bool:
    """Heuristic: Copter binary exists under ``../ardupilot/build/sitl/bin``."""
    bd = _sitl_vehicle_bin_dir()
    if not os.path.isdir(bd):
        return False
    for name in os.listdir(bd):
        if name.startswith("ardu") or name == "ardu":
            fp = os.path.join(bd, name)
            if os.path.isfile(fp):
                return True
    return False


def _close_sitl_log_files(handles) -> None:
    """Close SITL stdout log handles opened in main(); safe if already closed."""
    for f in handles:
        try:
            f.close()
        except OSError:
            pass


def _shutdown_sim_vehicle_spawns(
    handles: List[subprocess.Popen],
    *,
    pkill_mavproxy_after: bool,
) -> None:
    """SIGTERM/KILL ``sim_vehicle`` process groups (+ optional stray MAVProxy pkill)."""
    if not handles:
        return
    for p in handles:
        try:
            _terminate_sitl_process_group(p)
        except Exception:
            try:
                p.terminate()
            except Exception:
                pass
    time.sleep(0.5)
    for p in handles:
        try:
            p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            try:
                _kill_sitl_process_group(p)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        except Exception:
            pass
    if pkill_mavproxy_after:
        _pkill_mavproxy_processes()


def start_sitl_only(
    proj_root: str,
    num_drones: int = 2,
    param_file: Optional[str] = None,
    log_files=None,
    extra_param_files: Optional[List[str]] = None,
    no_mavproxy: bool = False,
    sitl_no_rebuild: bool = False,
) -> List[subprocess.Popen]:
    """Start SITL instances without Webots.

    Args:
        proj_root: Project root directory path.
        num_drones: Number of drone instances to start.
        param_file: Optional path to ArduPilot parameter file (relative to proj_root).
        log_files: Open text handles for stdout/stderr (one per drone); length must equal num_drones.
        extra_param_files: Additional ``.parm`` paths (relative to proj_root or absolute).
        no_mavproxy: If True, pass ``--no-mavproxy`` (scenarios must use TCP ``5760+10*i``).
        sitl_no_rebuild: If True, pass ``-N`` every ``sim_vehicle.py`` (no nested waf).
            Use when binaries already built (or after launcher multi-drone prebuild).

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
    processes_spawned: List[subprocess.Popen] = []
    BASE_UDP_PORT = 14551
    cwd = APM_HOME if os.path.isdir(APM_HOME) else proj_root
    try:
        for i in range(num_drones):
            udp_port = BASE_UDP_PORT + i * 10
            tcp_sitl = 5760 + i * 10
            home_str = _sitl_home_for_drone(i)
            args_list: List[str] = [
                sys.executable,
                SIM_VEHICLE_PATH,
            ]
            if sitl_no_rebuild:
                args_list.append("-N")
            args_list.extend(
                [
                    "-v",
                    "ArduCopter",
                    "-w",
                    f"--instance={i}",
                    f"--sysid={i + 1}",
                    "-l",
                    home_str,
                ]
            )
            args = args_list
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
                env=_sitl_subprocess_env(),
                **_sitl_subprocess_popen_kwargs(),
            )
            processes_spawned.append(proc)
            time.sleep(5)
    except KeyboardInterrupt:
        logger.info(
            "[SITL] KeyboardInterrupt during sim_vehicle startup; terminating %s instance(s)...",
            len(processes_spawned),
        )
        _shutdown_sim_vehicle_spawns(
            processes_spawned,
            pkill_mavproxy_after=(not no_mavproxy),
        )
        raise KeyboardInterrupt from None
    return processes_spawned


def start_sitl_webots(
    proj_root: str,
    num_drones: int = 2,
    param_file: Optional[str] = None,
    log_files=None,
    extra_param_files: Optional[List[str]] = None,
    sitl_no_rebuild: bool = False,
) -> List[subprocess.Popen]:
    """Start SITL instances with Webots (webots-python model).

    Args:
        proj_root: Project root directory.
        num_drones: Number of drone instances.
        param_file: Optional path to ArduPilot parameter file (relative to proj_root).
        log_files: Open text handles per drone.
        extra_param_files: Additional ``.parm`` paths.
        sitl_no_rebuild: If True, pass ``-N`` to every ``sim_vehicle.py``.

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
    processes_spawned: List[subprocess.Popen] = []
    cwd = APM_HOME if os.path.isdir(APM_HOME) else proj_root
    try:
        for i in range(num_drones):
            tcp_port = BASE_TCP + i * 10
            udp_port = BASE_UDP + i * 10
            home_str = _sitl_home_for_drone(i)
            args_list: List[str] = [
                sys.executable,
                SIM_VEHICLE_PATH,
            ]
            if sitl_no_rebuild:
                args_list.append("-N")
            args_list.extend(
                [
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
            )
            args = args_list
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
                env=_sitl_subprocess_env(),
                **_sitl_subprocess_popen_kwargs(),
            )
            processes_spawned.append(proc)
            time.sleep(5)
    except KeyboardInterrupt:
        logger.info(
            "[SITL/Webots] KeyboardInterrupt during sim_vehicle startup; terminating %s instance(s)...",
            len(processes_spawned),
        )
        _shutdown_sim_vehicle_spawns(processes_spawned, pkill_mavproxy_after=True)
        raise KeyboardInterrupt from None
    return processes_spawned


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

        scenario_combo, sitl_combo, launch_combo = partition_combo_for_batch(combo)
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

        if launch_combo.get("num_drones") is not None:
            run_num_drones = int(round(float(launch_combo["num_drones"])))
        else:
            run_num_drones = num_drones
        if run_num_drones < 1:
            logger.error("[Batch] swarm.num_drones override must be >= 1 (run %s)", run_idx)
            return 1

        cmd: List[str] = [
            sys.executable,
            launcher,
            "-s",
            "-c",
            scenario_id,
            "-n",
            str(run_num_drones),
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
        if scenario_id in (
            "leader_forward_back",
            "snake_pursuit",
            "task_assignment_decentralized",
        ):
            if args.kp is not None and "pid.p_gain" not in scenario_combo:
                cmd.extend(["--kp", str(args.kp)])
            if args.ki is not None and "pid.i_gain" not in scenario_combo:
                cmd.extend(["--ki", str(args.ki)])
            if args.kd is not None and "pid.d_gain" not in scenario_combo:
                cmd.extend(["--kd", str(args.kd)])
            if args.derivative_alpha is not None and "pid.derivative_alpha" not in scenario_combo:
                cmd.extend(["--derivative-alpha", str(args.derivative_alpha)])
        if scenario_id == "task_assignment_decentralized":
            if (
                getattr(args, "num_targets", None) is not None
                and "task.num_targets" not in scenario_combo
            ):
                cmd.extend(["--num-targets", str(args.num_targets)])
            nt = getattr(args, "target_radius_m", None)
            if nt is not None and "task.target_radius_m" not in scenario_combo:
                cmd.extend(["--target-radius-m", str(nt)])
            if getattr(args, "target_center_x", None) is not None and "task.target_center_x" not in scenario_combo:
                cmd.extend(["--target-center-x", str(args.target_center_x)])
            if getattr(args, "target_center_y", None) is not None and "task.target_center_y" not in scenario_combo:
                cmd.extend(["--target-center-y", str(args.target_center_y)])
            if args.no_exit_on_converged:
                cmd.append("--no-exit-on-converged")
            cmd.extend(["--post-assign-fly-sec", str(args.post_assign_fly_sec)])
            cmd.extend(
                ["--target-reach-radius-m", str(getattr(args, "target_reach_radius_m", 0.5))]
            )
            cmd.extend(
                ["--target-reach-epsilon-m", str(getattr(args, "target_reach_epsilon_m", 0.0))]
            )
            if getattr(args, "allow_converged_without_reaching_targets", False):
                cmd.append("--allow-converged-without-reaching-targets")
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
        if args.without_mavproxy_consoles:
            cmd.append("--without-mavproxy-consoles")

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
                "snake_distance_ground_follower, task_assignment_decentralized) "
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
        help="leader_forward_back / snake_pursuit / task_assignment_decentralized: P gain; forwarded.",
    )
    parser.add_argument(
        "--ki",
        type=float,
        default=None,
        metavar="K",
        help="leader_forward_back / snake_pursuit / task_assignment_decentralized: I gain.",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=None,
        metavar="K",
        help="leader_forward_back / snake_pursuit / task_assignment_decentralized: D gain.",
    )
    parser.add_argument(
        "--derivative-alpha",
        type=float,
        default=None,
        metavar="A",
        help=(
            "leader_forward_back / snake_pursuit / task_assignment_decentralized: low-pass on D term "
            "(0..1, 0=no filter); passed as --derivative-alpha to scenario."
        ),
    )
    parser.add_argument(
        "--post-assign-fly-sec",
        type=float,
        default=5.0,
        metavar="SEC",
        help=(
            "task_assignment_decentralized only: fly at most SEC seconds after a converged assignment "
            "before stopping coordination; passed as --post-assign-fly-sec."
        ),
    )
    parser.add_argument(
        "--no-exit-on-converged",
        action="store_true",
        help=(
            "task_assignment_decentralized only: ignore early exit after converge and run until "
            "--duration; forwarded as --no-exit-on-converged."
        ),
    )
    parser.add_argument(
        "--target-reach-radius-m",
        type=float,
        default=0.5,
        metavar="R",
        help=(
            "task_assignment_decentralized: horizontal NED XY distance (m) to count as «at target»; "
            "forwarded as --target-reach-radius-m."
        ),
    )
    parser.add_argument(
        "--target-reach-epsilon-m",
        type=float,
        default=0.0,
        metavar="E",
        help=(
            "task_assignment_decentralized: optional margin (m) added to R for final at-target check; "
            "forwarded as --target-reach-epsilon-m."
        ),
    )
    parser.add_argument(
        "--allow-converged-without-reaching-targets",
        action="store_true",
        help=(
            "task_assignment_decentralized: solver-only converged + short post-assign window; "
            "forwarded as --allow-converged-without-reaching-targets."
        ),
    )
    parser.add_argument(
        "--num-targets",
        type=int,
        default=None,
        metavar="K",
        help=(
            "task_assignment_decentralized only: fixed number of target waypoints; "
            "omit for K = number of drones (square)."
        ),
    )
    parser.add_argument(
        "--target-center-x",
        type=float,
        default=0.0,
        help="task_assignment_decentralized: target circle center X (m, NED North).",
    )
    parser.add_argument(
        "--target-center-y",
        type=float,
        default=0.0,
        help="task_assignment_decentralized: target circle center Y (m, NED East).",
    )
    parser.add_argument(
        "--target-radius-m",
        type=float,
        default=12.0,
        help="task_assignment_decentralized: target circle radius (m).",
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
        "--without-mavproxy-consoles",
        action="store_true",
        help=(
            "SITL-only: disable default MAVProxy --console mode and run direct TCP "
            "(scenarios connect to 127.0.0.1:5760+10*i). "
            "The launcher always runs pkill for MAVProxy (mavproxy.py / MAVProxy on cmdline) "
            "before starting SITL; with default consoles it also runs pkill on shutdown."
        ),
    )
    parser.add_argument(
        "--sitl-rebuild-waf",
        action="store_true",
        help=(
            "Run ../ardupilot waf configure (--board sitl) and waf copter once before spawning "
            "SITL, then pass -N to every sim_vehicle. Default is NO waf. "
            "Env: DRONE_SWARM_SITL_ALLOW_WAF_REBUILD=1."
        ),
    )
    parser.add_argument(
        "--sitl-no-rebuild",
        action="store_true",
        help=(
            "(Default behaviour already.) Pass -N — kept for backwards compatibility "
            "(scripts). Env DRONE_SWARM_SITL_NO_REBUILD=1 is redundant unless you need "
            "to stay explicit."
        ),
    )
    args = parser.parse_args()

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

    sitl_direct_tcp = (not use_webots) and args.without_mavproxy_consoles
    use_mavproxy_consoles = (not use_webots) and (not sitl_direct_tcp)

    sitl_rebuild_requested = bool(
        args.sitl_rebuild_waf
        or os.environ.get("DRONE_SWARM_SITL_ALLOW_WAF_REBUILD", "").strip() == "1"
    )
    if sitl_rebuild_requested:
        try:
            prebuild_ardupilot_sitl_copter()
        except FileNotFoundError as exc:
            logger.error("[SITL] Prebuild path error: %s", exc)
            sys.exit(1)
        except subprocess.CalledProcessError as exc:
            logger.error("[SITL] Prebuild failed (waf exited %s).", exc.returncode)
            sys.exit(1)
        logger.info("[SITL] sim_vehicle launches use -N after prebuild.")
    else:
        logger.info("[SITL] sim_vehicle launches use -N (no waf this run).")

    sitl_no_rebuild_effective = True
    if (
        os.path.isdir(APM_HOME)
        and os.path.isfile(SIM_VEHICLE_PATH)
        and not sitl_vehicle_binary_maybe_present()
    ):
        logger.warning(
            "[SITL] No vehicle binary found under ../ardupilot/build/sitl/bin — "
            "build once there (python waf configure --board sitl && python waf copter), "
            "or rerun with --sitl-rebuild-waf."
        )

    _pkill_mavproxy_processes()

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
        try:
            sitl_procs = start_sitl_webots(
                project_root,
                num_drones,
                args.param_file,
                log_files=sitl_log_files,
                extra_param_files=args.sitl_param_overlay or [],
                sitl_no_rebuild=sitl_no_rebuild_effective,
            )
        except KeyboardInterrupt:
            for p in processes:
                try:
                    p.terminate()
                except Exception:
                    pass
            try:
                time.sleep(0.5)
            except KeyboardInterrupt:
                pass
            for p in processes:
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    try:
                        p.kill()
                    except Exception:
                        pass
                except Exception:
                    pass
            _close_sitl_log_files(sitl_log_files)
            _pkill_mavproxy_processes()
            sys.exit(130)
        processes.extend(sitl_procs)
    else:
        try:
            sitl_procs = start_sitl_only(
                project_root,
                num_drones,
                args.param_file,
                log_files=sitl_log_files,
                extra_param_files=args.sitl_param_overlay or [],
                no_mavproxy=sitl_direct_tcp,
                sitl_no_rebuild=sitl_no_rebuild_effective,
            )
        except KeyboardInterrupt:
            _close_sitl_log_files(sitl_log_files)
            if use_mavproxy_consoles:
                _pkill_mavproxy_processes()
            sys.exit(130)
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
        if use_mavproxy_consoles:
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
    try:
        time.sleep(sitl_boot_wait_sec)
    except KeyboardInterrupt:
        _sitl_frozen = frozenset(sitl_procs)
        _shutdown_sim_vehicle_spawns(list(sitl_procs), pkill_mavproxy_after=False)
        for p in processes:
            if p not in _sitl_frozen:
                try:
                    p.terminate()
                except Exception:
                    pass
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        for p in processes:
            if p not in _sitl_frozen:
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    try:
                        p.kill()
                    except Exception:
                        pass
                except Exception:
                    pass
        _close_sitl_log_files(sitl_log_files)
        if use_mavproxy_consoles:
            _pkill_mavproxy_processes()
        sys.exit(130)

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
    if sitl_direct_tcp:
        scenario_env["DRONE_SWARM_SITL_DIRECT_TCP"] = "1"
    if use_2d_visualizer:
        scenario_env["DRONE_SWARM_WITH_2D_VIZ"] = "1"
    if args.duration > 0:
        scenario_cmd.extend(["--duration", str(args.duration)])
    if args.experiment_dir:
        scenario_cmd.extend(["--experiment-dir", args.experiment_dir])
    exchange_hz = args.exchange_hz
    if exchange_hz > 0:
        scenario_cmd.extend(["--exchange-hz", str(exchange_hz)])
    if scenario[0] in (
        "leader_forward_back",
        "snake_pursuit",
        "task_assignment_decentralized",
    ):
        if args.kp is not None:
            scenario_cmd.extend(["--kp", str(args.kp)])
        if args.ki is not None:
            scenario_cmd.extend(["--ki", str(args.ki)])
        if args.kd is not None:
            scenario_cmd.extend(["--kd", str(args.kd)])
    elif any(getattr(args, x, None) is not None for x in ("kp", "ki", "kd")):
        logger.warning(
            "[Launcher] --kp/--ki/--kd apply only to leader_forward_back, snake_pursuit, "
            "and task_assignment_decentralized; ignoring."
        )
    if args.derivative_alpha is not None:
        if scenario[0] in (
            "leader_forward_back",
            "snake_pursuit",
            "task_assignment_decentralized",
        ):
            scenario_cmd.extend(["--derivative-alpha", str(args.derivative_alpha)])
        else:
            logger.warning(
                "[Launcher] --derivative-alpha applies only to leader_forward_back, snake_pursuit, "
                "and task_assignment_decentralized; ignoring."
            )
    if scenario[0] == "task_assignment_decentralized":
        scenario_cmd.extend(["--post-assign-fly-sec", str(args.post_assign_fly_sec)])
        scenario_cmd.extend(["--target-reach-radius-m", str(args.target_reach_radius_m)])
        scenario_cmd.extend(["--target-reach-epsilon-m", str(args.target_reach_epsilon_m)])
        if args.allow_converged_without_reaching_targets:
            scenario_cmd.append("--allow-converged-without-reaching-targets")
        if args.no_exit_on_converged:
            scenario_cmd.append("--no-exit-on-converged")
        scenario_cmd.extend(
            [
                "--target-radius-m",
                str(args.target_radius_m),
                "--target-center-x",
                str(args.target_center_x),
                "--target-center-y",
                str(args.target_center_y),
            ]
        )
        if args.num_targets is not None:
            scenario_cmd.extend(["--num-targets", str(args.num_targets)])
    else:
        if args.no_exit_on_converged:
            logger.warning(
                "[Launcher] --no-exit-on-converged applies only to "
                "task_assignment_decentralized; ignoring."
            )
        if (
            args.num_targets is not None
            or args.target_center_x != 0.0
            or args.target_center_y != 0.0
            or args.target_radius_m != 12.0
        ):
            logger.warning(
                "[Launcher] target geometry flags apply only to task_assignment_decentralized; "
                "ignoring for scenario %s.",
                scenario[0],
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
    _pkill_mavproxy_after_run = bool(use_mavproxy_consoles)

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
