# Replay module

Purpose: load experiment logs (CSV + metadata) and play them back in ROS/RViz for 3D visualization.

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

Fields:

- `duration_sec`: max experiment duration (0 = no limit)
- `collision_radius_m`: drone sphere radius for collision detection (m)
- `num_drones`: number of drones
- `scenario`: scenario name

## RViz replay (replay_rviz.py)

Publishes drone poses to ROS for 3D playback in RViz. Requires ROS (e.g. Noetic) and `rospy`.

### Prerequisites

- ROS installed (e.g. `ros-noetic-desktop` on Ubuntu 20.04)
- Source ROS before running: `source /opt/ros/noetic/setup.bash`

### Launch commands

Run from the **project root** (`Drone_Swarm_Simulator_v2/`):

```bash
# Start ROS master (if not already running)
roscore

# Replay at real-time speed (default rate=1.0)
python replay/replay_rviz.py --experiment experiments/2025-01-15_10-30-00

# Replay at 2x speed
python replay/replay_rviz.py --experiment experiments/2025-01-15_10-30-00 --rate 2.0

# Replay at half speed
python replay/replay_rviz.py --experiment experiments/2025-01-15_10-30-00 --rate 0.5
```

Or using the module:

```bash
python -m replay.replay_rviz --experiment experiments/2025-01-15_10-30-00 --rate 1.0
```

### Arguments

| Argument         | Required | Description                                                                 |
|------------------|----------|-----------------------------------------------------------------------------|
| `--experiment`   | Yes      | Path to experiment directory (contains `metadata.json` and `drone_*.csv`).  |
| `--rate`         | No       | Playback speed multiplier (default: 1.0). With `--interactive`, initial speed. |
| `--interactive`  | No       | Enable play/pause, seek, and speed via keyboard and ROS topics.            |

### Playback controls (play / pause / rewind / speed)

With `--interactive`, playback supports:

- **Keyboard** (when stdin is a TTY): `Space` = play/pause, `a`/`d` (or Left/Right) = step back/forward, `+`/`-` = speed up/down, `q` = quit.
- **ROS topics** (work in all environments, including Docker where keyboard is unavailable): publish to control playback — see table below.

Without `--interactive`, replay runs once at fixed `--rate` and then exits (original behavior).

### ROS topics

| Topic                     | Type                       | Description                                      |
|---------------------------|----------------------------|--------------------------------------------------|
| `/swarm/drone_<id>/pose`  | `geometry_msgs/PoseStamped`| Pose of drone `id` (ENU frame).                  |
| `/swarm/metadata`         | `std_msgs/String`          | Experiment metadata JSON (once at start).        |
| `/replay/play`            | `std_msgs/Empty`           | Start or resume playback (interactive mode).     |
| `/replay/pause`           | `std_msgs/Empty`           | Pause playback (interactive mode).               |
| `/replay/seek`            | `std_msgs/Float64`         | Seek to time in seconds (interactive mode).     |
| `/replay/speed`           | `std_msgs/Float32`         | Set speed multiplier, e.g. 0.25–4.0 (interactive mode). |

Poses are published in ENU for RViz (converted from NED in the CSV). Use Ctrl+C to stop replay cleanly (SIGINT handled).

### Docker launch

From project root, use the helper script (see also `docs/replay-setup.md`):

```bash
./scripts/replay-docker.sh EXPERIMENT_PATH [RATE] [INTERACTIVE]
```

- **EXPERIMENT_PATH** — path to experiment dir (e.g. `experiments/2025-01-15_10-30-00`).
- **RATE** — optional playback speed 0.25–4.0 (default 1.0).
- **INTERACTIVE** — optional: `1`, `true`, or `yes` to enable play/pause/rewind/speed controls. In Docker the keyboard is unavailable; use ROS topics `/replay/play`, `/replay/pause`, `/replay/seek`, `/replay/speed` from another node or host.

Example:

```bash
export DISPLAY=:0
xhost +local:docker

# Linear playback (default rate 1.0)
./scripts/replay-docker.sh experiments/2025-01-15_10-30-00 1.0

# Interactive: control via ROS topics (keyboard does not work inside Docker)
./scripts/replay-docker.sh experiments/2025-01-15_10-30-00 1.0 1
```

This starts one container with roscore + replay. Run RViz separately (e.g. second terminal or container) with `rosrun rviz rviz`. Workspace path is configurable via `PROJECT_ROOT` (default: directory above `scripts/`).
