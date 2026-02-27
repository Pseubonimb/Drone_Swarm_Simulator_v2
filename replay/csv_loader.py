"""
Load experiment data from CSV logs and metadata.json.

Replay module is isolated: reads only CSV and metadata, does not import from core/ or scenarios/.
"""

import json
import os
from typing import Any, Dict


def load_experiment(experiment_dir: str) -> Dict[str, Any]:
    """
    Load an experiment from disk.

    Expects experiment_dir to contain:
      - metadata.json: duration_sec, collision_radius_m, num_drones, scenario
      - drone_1.csv, drone_2.csv, ... with header t,x,y,z,rx,ry,rz,hasCollision

    Returns:
        dict with keys:
          - "metadata": parsed metadata.json
          - "drone_logs": list of paths to drone CSV files (drone_1.csv, ...)
          - "experiment_dir": experiment_dir path
    """
    metadata_path = os.path.join(experiment_dir, "metadata.json")
    if not os.path.isfile(metadata_path):
        return {
            "metadata": {},
            "drone_logs": [],
            "experiment_dir": experiment_dir,
        }
    with open(metadata_path, "r", encoding="utf-8") as f:
        metadata = json.load(f)
    num_drones = metadata.get("num_drones", 0)
    drone_logs = []
    for i in range(1, num_drones + 1):
        path = os.path.join(experiment_dir, f"drone_{i}.csv")
        if os.path.isfile(path):
            drone_logs.append(path)
    return {
        "metadata": metadata,
        "drone_logs": drone_logs,
        "experiment_dir": experiment_dir,
    }
