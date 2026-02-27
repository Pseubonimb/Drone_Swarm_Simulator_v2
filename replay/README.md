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

| Argument       | Required | Description                                                                 |
|----------------|----------|-----------------------------------------------------------------------------|
| `--experiment` | Yes      | Path to experiment directory (contains `metadata.json` and `drone_*.csv`).  |
| `--rate`       | No       | Playback speed multiplier (default: 1.0). 2.0 = 2x, 0.5 = half speed.      |

### ROS topics

| Topic                     | Type                  | Description                    |
|---------------------------|-----------------------|--------------------------------|
| `/swarm/drone_<id>/pose`  | `geometry_msgs/PoseStamped` | Pose of drone `id` (ENU frame). |
| `/swarm/metadata`         | `std_msgs/String`     | Experiment metadata JSON (once at start). |

Poses are published in ENU for RViz (converted from NED in the CSV). Use Ctrl+C to stop replay cleanly (SIGINT handled).
