import csv
import hashlib
import json
import math
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
CONFIG = {
    "experiment": "zero_hold",
    "data_version": "data_v1",
    "controller": "forward_velocity_controller",
    "duration_wall_s": 10.0,
    "publish_period_wall_s": 0.02,
    "startup_timeout_wall_s": 10.0,
    "feedback_gap_limit_wall_s": 0.2,
    "simulation_stall_limit_wall_s": 1.0,
    "controller_check_period_wall_s": 1.0,
    "controller_response_timeout_wall_s": 2.0,
    "stop_publish_wall_s": 1.0,
    "stop_speed_limit_rad_s": 0.001,
    "stop_min_sim_span_s": 0.2,
    "hold_position_range_limit_rad": 0.001,
    "joints": JOINTS,
    "command_topic": "/forward_velocity_controller/commands",
    "state_topic": "/joint_states",
    "simulation_clock": "joint_states.header.stamp",
    "wall_clock": "time.monotonic",
    "scope": "zero commands only; no switching; no crash watchdog",
}


def git_read(root, *args):
    result = subprocess.run(
        ["git", "-C", str(root), *args],
        capture_output=True, text=True, timeout=5,
    )
    if result.returncode:
        return None
    return result.stdout.strip()


class ZeroHold(Node):
    def __init__(self, folder):
        super().__init__("p2_zero_hold")
        self.started = time.monotonic()
        self.phase = "waiting"
        self.fault = None
        self.samples = 0
        self.invalid = 0
        self.sent = 0
        self.max_gap = 0.0
        self.last_receive = None
        self.first_sim = None
        self.last_sim = None
        self.last_advance = None
        self.positions = None
        self.min_position = None
        self.max_position = None
        self.controllers = None
        self.last_controller_check = None
        self.pending = None
        self.request_wall = None
        self.next_request = 0.0
        self.next_publish = 0.0
        self.stop_samples = []
        self.feedback_file = (folder / "joint_log.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.command_file = (folder / "command_log.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.feedback_log = csv.writer(self.feedback_file)
        self.command_log = csv.writer(self.command_file)
        self.feedback_log.writerow(
            ["phase", "sim_stamp_s", "wall_elapsed_s", "receive_gap_wall_s"]
            + [j + "_position_rad" for j in JOINTS]
            + [j + "_velocity_rad_s" for j in JOINTS]
        )
        self.command_log.writerow(
            ["phase", "wall_elapsed_s", "latest_feedback_sim_stamp_s"]
            + [j + "_command_rad_s" for j in JOINTS]
        )
        self.publisher = self.create_publisher(
            Float64MultiArray, CONFIG["command_topic"],
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.subscription = self.create_subscription(
            JointState, CONFIG["state_topic"], self.receive, qos_profile_sensor_data
        )
        self.client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

    def receive(self, msg):
        now = time.monotonic()
        try:
            if len(set(msg.name)) != len(msg.name):
                raise ValueError("duplicate joint names")
            indices = [msg.name.index(j) for j in JOINTS]
            q = [msg.position[i] for i in indices]
            v = [msg.velocity[i] for i in indices]
            if not all(math.isfinite(x) for x in q + v):
                raise ValueError("non-finite feedback")
        except (ValueError, IndexError):
            self.invalid += 1
            self.fault = self.fault or "invalid_joint_feedback"
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gap = None if self.last_receive is None else now - self.last_receive
        if gap is not None:
            self.max_gap = max(self.max_gap, gap)
            if self.phase == "hold" and gap > CONFIG["feedback_gap_limit_wall_s"]:
                self.fault = self.fault or "feedback_receive_gap_exceeded"
        if self.last_sim is not None and stamp < self.last_sim:
            self.fault = self.fault or "simulation_time_reversed"
        if self.last_sim is None or stamp > self.last_sim:
            self.last_advance = now
        if self.first_sim is None:
            self.first_sim = stamp
        self.last_receive = now
        self.last_sim = stamp
        self.positions = q
        if self.phase == "hold":
            self.min_position = [min(a, b) for a, b in zip(self.min_position, q)]
            self.max_position = [max(a, b) for a, b in zip(self.max_position, q)]
        if self.phase == "stopping":
            self.stop_samples.append((now, stamp, max(abs(x) for x in v)))
        self.samples += 1
        self.feedback_log.writerow([self.phase, stamp, now - self.started, gap] + q + v)

    def check_controllers(self, now):
        if self.pending is not None:
            if self.pending.done():
                result = self.pending.result()
                if result is None:
                    raise RuntimeError("controller_query_failed")
                self.controllers = {c.name: c.state for c in result.controller}
                active = {c.name for c in result.controller if c.state == "active"}
                expected = {CONFIG["controller"], "joint_state_broadcaster"}
                if active != expected:
                    raise RuntimeError("unexpected_active_controllers: " + str(self.controllers))
                self.last_controller_check = now
                self.pending = None
                self.next_request = now + CONFIG["controller_check_period_wall_s"]
            elif now - self.request_wall > CONFIG["controller_response_timeout_wall_s"]:
                raise RuntimeError("controller_query_timeout")
        if self.pending is None and now >= self.next_request and self.client.service_is_ready():
            self.pending = self.client.call_async(ListControllers.Request())
            self.request_wall = now

    def publish_zero(self, now):
        if now < self.next_publish:
            return
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self.publisher.publish(msg)
        self.sent += 1
        self.next_publish = now + CONFIG["publish_period_wall_s"]
        self.command_log.writerow(
            [self.phase, now - self.started, self.last_sim] + list(msg.data)
        )

    def stop_and_check(self):
        self.phase = "stopping"
        end = time.monotonic() + CONFIG["stop_publish_wall_s"]
        while rclpy.ok() and time.monotonic() < end:
            self.publish_zero(time.monotonic())
            rclpy.spin_once(self, timeout_sec=0.005)
        recent = [x for x in self.stop_samples if x[0] >= end - 0.5]
        if len(recent) < 5:
            return {"verified": False, "reason": "insufficient_stop_feedback"}
        fresh = time.monotonic() - recent[-1][0] <= CONFIG["feedback_gap_limit_wall_s"]
        advancing = recent[-1][1] - recent[0][1] >= CONFIG["stop_min_sim_span_s"]
        max_speed = max(x[2] for x in recent)
        return {
            "verified": fresh and advancing and max_speed <= CONFIG["stop_speed_limit_rad_s"],
            "recent_samples": len(recent),
            "observed_sim_span_s": recent[-1][1] - recent[0][1],
            "max_abs_velocity_rad_s": max_speed,
            "feedback_fresh": fresh,
            "simulation_advancing": advancing,
        }


def main():
    root = Path.home() / "ROS2"
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_zero_hold"
    folder = root / "arm/P2/data" / CONFIG["data_version"] / run_id
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "config.yaml").write_text(json.dumps(CONFIG, indent=2), encoding="utf-8")
    dirty = git_read(root, "status", "--porcelain")
    run = {
        "run_id": run_id,
        "started_at": datetime.now().astimezone().isoformat(),
        "code_commit": git_read(root, "rev-parse", "HEAD"),
        "git_dirty": None if dirty is None else bool(dirty),
        "git_status": dirty,
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "status": "running", "reason": None,
        "timing_note": "wall publication and duration; feedback stamps are simulation time",
        "limitations": "receive-gap check is not absolute message-age verification; no crash watchdog",
    }
    meta = folder / "run.json"
    meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
    interrupted = []
    old_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        old_handlers[signum] = signal.signal(signum, lambda number, frame: interrupted.append(number))
    node = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    print("保存目录：", folder)
    print("仅发零；检查通过后记录 10 秒墙钟时间；Ctrl+C 请求发零收尾")
    try:
        node = ZeroHold(folder)
        hold_start = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            if interrupted:
                run.update(status="interrupted", reason="signal_" + str(interrupted[0]))
                break
            if node.fault:
                raise RuntimeError(node.fault)
            node.check_controllers(now)
            if node.count_publishers(CONFIG["command_topic"]) > 1:
                raise RuntimeError("another_command_publisher_detected")
            if node.phase == "waiting":
                if now - node.started > CONFIG["startup_timeout_wall_s"]:
                    raise RuntimeError("startup_checks_timeout")
                ready = (
                    node.last_controller_check is not None
                    and node.last_receive is not None
                    and now - node.last_receive <= CONFIG["feedback_gap_limit_wall_s"]
                    and node.last_sim > node.first_sim
                    and node.publisher.get_subscription_count() > 0
                )
                if not ready:
                    continue
                node.min_position = list(node.positions)
                node.max_position = list(node.positions)
                node.phase = "hold"
                hold_start = now
                print("检查通过，开始零速记录")
            if now - node.last_receive > CONFIG["feedback_gap_limit_wall_s"]:
                raise RuntimeError("feedback_not_received_in_time")
            if now - node.last_advance > CONFIG["simulation_stall_limit_wall_s"]:
                raise RuntimeError("simulation_time_stalled")
            node.publish_zero(now)
            if max(b - a for a, b in zip(node.min_position, node.max_position)) > CONFIG["hold_position_range_limit_rad"]:
                raise RuntimeError("hold_position_range_exceeded")
            if now - hold_start >= CONFIG["duration_wall_s"]:
                run.update(status="completed", reason=None)
                break
        else:
            raise RuntimeError("ros_context_stopped")
    except Exception as exc:
        run.update(status="failed", reason=str(exc))
    finally:
        if node is not None:
            try:
                if node.sent:
                    run["stop_check"] = node.stop_and_check()
                else:
                    run["stop_check"] = {"verified": False, "reason": "test_not_started"}
            except Exception as exc:
                run["stop_check"] = {"verified": False, "reason": str(exc)}
            if run["status"] == "completed" and not run["stop_check"]["verified"]:
                run.update(status="failed", reason="stop_feedback_not_verified")
            run.update(
                controllers_observed=node.controllers,
                valid_samples=node.samples, invalid_samples=node.invalid,
                zero_commands_published=node.sent,
                max_receive_gap_wall_s=node.max_gap,
                position_range_rad=(
                    [b-a for a, b in zip(node.min_position, node.max_position)]
                    if node.min_position is not None else None
                ),
                simulation_span_s=(
                    node.last_sim - node.first_sim if node.first_sim is not None else None
                ),
                total_wall_s=time.monotonic() - node.started,
            )
            node.feedback_file.close()
            node.command_file.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        run["finished_at"] = datetime.now().astimezone().isoformat()
        meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
        print(json.dumps(run, indent=2, ensure_ascii=False))
        print("测试已结束；请在仿真启动终端退出 P2，并检查残留进程。")
    return 0 if run["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
