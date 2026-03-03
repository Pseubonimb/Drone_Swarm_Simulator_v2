# Replay module

Load experiment logs (CSV + metadata) and play them back in ROS/RViz for 3D visualization. Two launch options: **(1) two terminals** (replay in the first, RViz in the second) or **(2) single-command script** that starts replay then RViz. See also `docs/new_Noetic_replay-setup.md`.

**Prerequisites (both options):** Docker, X11. Once per session run `export DISPLAY=:0` and `xhost +local:docker` so containers can use the host display.

## Launch options

### Option 1: Two terminals (current flow)

Use when you want to keep replay and RViz in separate terminals (e.g. to see replay logs). Steps below.

### Option 2: Single-command script

From project root, one script starts the replay container in the background and then runs the RViz container. When you close RViz, the replay container is stopped automatically.

```bash
export DISPLAY=:0
xhost +local:docker

./scripts/replay-with-rviz.sh EXPERIMENT_PATH [RATE] [INTERACTIVE]
```

- **EXPERIMENT_PATH** — path to experiment dir (e.g. `experiments/2026-02-28_18-35-11`).
- **RATE** — optional speed 0.25–4.0 (default 1.0).
- **INTERACTIVE** — optional: `1`, `true`, or `yes` for play/pause/seek/speed via ROS topics.

**Environment:** Set `DISPLAY` (e.g. `:0`) and run `xhost +local:docker` once per session so both containers can use the host X11.

---

## CSV format

Per-drone CSV files have header:

```text
t,x,y,z,rx,ry,rz,hasCollision
```

- `t`: time from experiment start (seconds)
- `x`, `y`, `z`: position in NED frame (meters)
- `rx`, `ry`, `rz`: Euler angles roll, pitch, yaw (radians)
- `hasCollision`: 0 or 1

## metadata.json

- `duration_sec`: max experiment duration (0 = no limit)
- `collision_radius_m`: drone sphere radius for collision detection (m)
- `num_drones`: number of drones
- `scenario`: scenario name

## Launch order (Docker, two terminals) — Option 1

Use **ROS 1 Noetic** in Docker. Replay and roscore run in the first container; RViz runs in a second container (same host network).

### Step 1. First terminal: start replay (roscore + replay in one container)

From project root:

```bash
export DISPLAY=:0
xhost +local:docker

./scripts/replay-docker.sh EXPERIMENT_PATH [RATE] [INTERACTIVE]
```

- **EXPERIMENT_PATH** — path to experiment dir (e.g. `experiments/2026-02-28_18-35-11`).
- **RATE** — optional speed 0.25–4.0 (default 1.0).
- **INTERACTIVE** — optional: `1`, `true`, or `yes` to enable play/pause/seek/speed via ROS topics (keyboard not available in Docker).

Leave this terminal/container running.

### Step 2. Second terminal (host): X11 once per session

```bash
export DISPLAY=:0
xhost +local:docker
```

### Step 3. Second terminal: start container for RViz

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/user/Документы/Kursach/Drone_Swarm_Simulator_v2:/workspace \
  --network host \
  osrf/ros:noetic-desktop-full \
  bash
```

Replace the path in `-v` if the project is elsewhere. `LIBGL_ALWAYS_SOFTWARE=1` avoids libGL/amdgpu errors in the container; omit if you use host GPU passthrough.

### Step 4. Inside the second container: run RViz

```bash
source /opt/ros/noetic/setup.bash
rosrun rviz rviz
```

### Step 5. Configure RViz for drones

1. **Fixed Frame:** In Global Options set **Fixed Frame** to `world` (same as `frame_id` in `replay_rviz.py`).
2. **Add poses:** Add → By topic → `/swarm/drone_1/pose` (PoseStamped). Add `/swarm/drone_2/pose`, `/swarm/drone_3/pose`, etc. for more drones.
3. Optionally set display type (Arrow/Axes), color, and scale per pose.

### Step 6. Control playback (first terminal / first container)

- **Keyboard** (if replay is run on host with TTY): `Space` = play/pause, `a`/`d` or Left/Right = step, `+`/`-` = speed, `q` = quit.
- **ROS topics** (works in Docker): publish to `/replay/play`, `/replay/pause`, `/replay/seek` (Float64), `/replay/speed` (Float32). See table below.

---

## Arguments (replay_rviz.py)

| Argument        | Required | Description                                                                 |
|-----------------|----------|-----------------------------------------------------------------------------|
| `--experiment`  | Yes      | Path to experiment directory (contains `metadata.json` and `drone_*.csv`).  |
| `--rate`        | No       | Playback speed multiplier (default 1.0). With `--interactive`, initial speed.|
| `--interactive` | No       | Enable play/pause, seek, speed via keyboard and ROS topics.                 |

## ROS topics

| Topic                     | Type                       | Description                                      |
|---------------------------|----------------------------|--------------------------------------------------|
| `/swarm/drone_<id>/pose`  | `geometry_msgs/PoseStamped`| Pose of drone `id` (ENU frame).                  |
| `/swarm/metadata`         | `std_msgs/String`          | Experiment metadata JSON (once at start).         |
| `/replay/play`            | `std_msgs/Empty`           | Start or resume playback (interactive mode).     |
| `/replay/pause`           | `std_msgs/Empty`           | Pause playback (interactive mode).               |
| `/replay/seek`            | `std_msgs/Float64`         | Seek to time in seconds (interactive mode).       |
| `/replay/speed`           | `std_msgs/Float32`         | Set speed multiplier 0.25–4.0 (interactive mode).|

Poses are published in ENU for RViz (converted from NED in the CSV). Use Ctrl+C to stop replay (SIGINT handled).

---

## Running without Docker (host with ROS Noetic)

If ROS Noetic is installed on the host:

```bash
# Terminal 1
source /opt/ros/noetic/setup.bash
roscore

# Terminal 2
cd /path/to/Drone_Swarm_Simulator_v2
python replay/replay_rviz.py --experiment experiments/2025-01-15_10-30-00 --rate 1.0

# Terminal 3 (optional, for interactive controls use --interactive in terminal 2)
source /opt/ros/noetic/setup.bash
rosrun rviz rviz
```

Or as module: `python -m replay.replay_rviz --experiment experiments/... --rate 1.0`.

---

## References

- Full setup and troubleshooting: `docs/new_Noetic_replay-setup.md`
- Script usage and workspace path: `scripts/replay-docker.sh` (default project root = directory above `scripts/`)
- **Исправления 2D-визуализатора и MAVLink (01.03.26):** в `docs/found errors/01_03_26_errors_in_project.md` описаны внесённые изменения: один поток на одно MAVLink-подключение (MAVLinkWorker), отказ от хранения истории в реальном времени в 2D-визуализаторе (только текущие позиции).
- **Исправления replay и core (03.03.26):** в `docs/found errors/03_03_26_errors_in_project.md` — один скрипт для Docker replay+RViz, синхронное пошаговое воспроизведение в RViz, начальные позиции дронов по Y (+2 м на дрон), вынос DroneController в `core/control`.
