# Replay module

Purpose: load experiment logs (CSV + metadata) and play them back, e.g. in ROS/RViz for 3D visualization.

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

Full RViz playback is implemented in a later stage.
