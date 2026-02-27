#!/usr/bin/env python3
"""
ROS node for 3D replay of drone swarm experiments in RViz.

Loads experiment data via replay.csv_loader, publishes PoseStamped for each drone
on /swarm/drone_<id>/pose at a configurable playback rate. Uses only replay.csv_loader
and rospy; no imports from core/ or scenarios/.
"""

import argparse
import json
import os
import signal
import sys
from typing import Any, Dict, List

# Ensure project root is on path when run as script (replay/replay_rviz.py or from replay/)
_script_dir = os.path.dirname(os.path.abspath(__file__))
_replay_parent = os.path.dirname(_script_dir)
if _replay_parent not in sys.path:
    sys.path.insert(0, _replay_parent)

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from tf.transformations import quaternion_from_euler
except ImportError as e:
    print(
        "replay_rviz.py requires ROS and rospy. Install ROS (e.g. Noetic) and source setup.bash:\n"
        "  source /opt/ros/noetic/setup.bash\n"
        "Error: " + str(e),
        file=sys.stderr,
    )
    sys.exit(1)

from replay.csv_loader import load_experiment, iter_steps


def _ned_to_enu(x: float, y: float, z: float) -> tuple:
    """Convert NED coordinates to ENU for RViz (x→y, y→x, z→-z)."""
    return (float(y), float(x), float(-z))


def _row_to_pose_stamped(
    row: Dict[str, Any], frame_id: str = "world", stamp: Any = None
) -> PoseStamped:
    """Build PoseStamped from CSV row (t,x,y,z,rx,ry,rz,hasCollision). NED → ENU."""
    enu_x, enu_y, enu_z = _ned_to_enu(
        float(row["x"]), float(row["y"]), float(row["z"])
    )
    roll = float(row["rx"])
    pitch = float(row["ry"])
    yaw = float(row["rz"])
    q = quaternion_from_euler(roll, pitch, yaw)
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp or rospy.Time.now()
    msg.pose.position.x = enu_x
    msg.pose.position.y = enu_y
    msg.pose.position.z = enu_z
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    return msg


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay drone swarm experiment in RViz via ROS topics."
    )
    parser.add_argument(
        "--experiment",
        type=str,
        required=True,
        help="Path to experiment directory (contains metadata.json and drone_*.csv).",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Playback speed multiplier (1.0 = real-time, 2.0 = 2x, 0.5 = half).",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    experiment_dir = args.experiment
    rate = args.rate
    if rate <= 0:
        print("replay_rviz: --rate must be positive.", file=sys.stderr)
        sys.exit(1)

    try:
        data = load_experiment(experiment_dir)
    except (ValueError, OSError) as e:
        print(f"replay_rviz: Failed to load experiment: {e}", file=sys.stderr)
        sys.exit(1)

    metadata = data.get("metadata") or {}
    drone_logs: List[str] = data.get("drone_logs") or []

    if not drone_logs:
        print(
            f"replay_rviz: No drone CSV files in experiment dir: {experiment_dir}",
            file=sys.stderr,
        )
        sys.exit(1)

    num_drones = len(drone_logs)
    rospy.init_node("replay_rviz", anonymous=True)

    # Publishers: /swarm/drone_<id>/pose (drone_id 1..num_drones)
    pose_pubs: Dict[int, Any] = {}
    for i in range(1, num_drones + 1):
        pose_pubs[i] = rospy.Publisher(
            f"/swarm/drone_{i}/pose", PoseStamped, queue_size=10
        )
    meta_pub = rospy.Publisher("/swarm/metadata", String, queue_size=1)

    # Publish metadata once
    meta_pub.publish(String(data=json.dumps(metadata)))

    def shutdown(_signum=None, _frame=None) -> None:
        rospy.signal_shutdown("SIGINT or shutdown")

    signal.signal(signal.SIGINT, shutdown)

    rospy.loginfo(
        "Replay starting: %d drones, rate=%.2f, experiment=%s",
        num_drones,
        rate,
        experiment_dir,
    )

    prev_t: float = 0.0
    for t, step_list in iter_steps(loaded=data, align="shortest"):
        if rospy.is_shutdown():
            break
        stamp = rospy.Time.now()
        for i, row in enumerate(step_list):
            if row is None:
                continue
            drone_id = i + 1
            msg = _row_to_pose_stamped(row, stamp=stamp)
            pose_pubs[drone_id].publish(msg)
        dt = t - prev_t
        prev_t = t
        if dt > 0 and rate > 0:
            rospy.sleep(dt / rate)

    rospy.loginfo("Replay finished.")


if __name__ == "__main__":
    main()
