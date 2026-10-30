import csv
import json
import math
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def git_value(root, *args):
    result = subprocess.run(
        ["git", "-C", str(root), *args],
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


class FeedbackRecorder(Node):
    def __init__(self, writer):
        super().__init__("p2_feedback_recorder")
        self.writer = writer
        self.count = 0
        self.invalid = 0
        self.first_wall = None
        self.first_sim = None
        self.last_wall = None
        self.last_sim = None
        self.sim_resets = 0
        self.max_gap = 0.0
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.receive, qos_profile_sensor_data
        )

    def receive(self, msg):
        wall = time.monotonic()
        try:
            indices = [msg.name.index(name) for name in JOINTS]
            positions = [msg.position[i] for i in indices]
            velocities = [msg.velocity[i] for i in indices]
            if not all(math.isfinite(x) for x in positions + velocities):
                raise ValueError("non-finite feedback")
        except (ValueError, IndexError):
            self.invalid += 1
            return

        sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.first_wall is None:
            self.first_wall = wall
            self.first_sim = sim
        if self.last_wall is not None:
            self.max_gap = max(self.max_gap, wall - self.last_wall)
            if sim < self.last_sim:
                self.sim_resets += 1

        self.writer.writerow(
            [sim, sim - self.first_sim, wall - self.first_wall]
            + positions + velocities
        )
        self.last_wall = wall
        self.last_sim = sim
        self.count += 1


def main():
    root = Path.home() / "ROS2"
    config = {
        "experiment": "feedback_check",
        "data_version": "data_v1",
        "topic": "/joint_states",
        "duration_wall_s": 10.0,
        "first_feedback_timeout_wall_s": 5.0,
        "stale_timeout_wall_s": 1.0,
        "joints": JOINTS,
        "position_unit": "rad",
        "velocity_unit": "rad/s",
        "wall_clock": "time.monotonic",
        "simulation_clock": "joint_states.header.stamp",
    }
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_feedback_check"
    folder = root / "arm/P2/data/data_v1" / run_id
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "config.yaml").write_text(
        json.dumps(config, indent=2), encoding="utf-8"
    )
    changes = git_value(root, "status", "--porcelain")
    run = {
        "run_id": run_id,
        "started_at": datetime.now().astimezone().isoformat(),
        "code_commit": git_value(root, "rev-parse", "HEAD"),
        "git_dirty": None if changes is None else bool(changes),
        "git_status": changes,
        "controller": "existing joint_trajectory_controller; unchanged",
        "status": "running",
        "reason": None,
    }
    meta = folder / "run.json"
    meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
    print("保存位置：", folder)

    rclpy.init()
    node = None
    try:
        with (folder / "joint_log.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["sim_stamp_s", "sim_elapsed_s", "wall_elapsed_s"]
                + [name + "_position_rad" for name in JOINTS]
                + [name + "_velocity_rad_s" for name in JOINTS]
            )
            node = FeedbackRecorder(writer)
            start = time.monotonic()
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                if node.first_wall is None:
                    if now - start > config["first_feedback_timeout_wall_s"]:
                        raise RuntimeError("no valid feedback within 5 wall seconds")
                    continue
                if node.sim_resets:
                    raise RuntimeError("simulation timestamp moved backwards")
                if now - node.last_wall > config["stale_timeout_wall_s"]:
                    raise RuntimeError("no valid feedback for 1 wall second")
                if now - node.first_wall >= config["duration_wall_s"]:
                    if node.last_sim <= node.first_sim:
                        raise RuntimeError("simulation time did not advance")
                    run["status"] = "completed"
                    break
            else:
                run["status"] = "interrupted"
                run["reason"] = "ROS context stopped"
    except KeyboardInterrupt:
        run["status"] = "interrupted"
        run["reason"] = "Ctrl+C"
    except Exception as exc:
        run["status"] = "failed"
        run["reason"] = str(exc)
    finally:
        if node is not None:
            run.update({
                "valid_samples": node.count,
                "invalid_samples": node.invalid,
                "max_receive_gap_wall_s": node.max_gap,
                "sim_elapsed_s": (
                    node.last_sim - node.first_sim
                    if node.first_sim is not None else None
                ),
                "wall_elapsed_s": (
                    node.last_wall - node.first_wall
                    if node.first_wall is not None else None
                ),
            })
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        run["finished_at"] = datetime.now().astimezone().isoformat()
        meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
        print(json.dumps(run, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()