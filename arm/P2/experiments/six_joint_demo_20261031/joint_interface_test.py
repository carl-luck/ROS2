import argparse
import csv
import hashlib
import json
import math
import signal
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ur5e_mpc_control import zero_hold


C = {
    "experiment": "joint_interface_test",
    "joints": zero_hold.JOINTS,
    "request_topic": "/p2/joint_velocity_request",
    "command_topic": "/forward_velocity_controller/commands",
    "request_type": "JointState; name and velocity only; header stamp is source feedback simulation stamp",
    "peak_velocity_rad_s": 0.02,
    "motion_duration_sim_s": 8.0,
    "publish_period_wall_s": 0.02,
    "command_rate_limit_rad_s2": 0.1,
    "request_timeout_wall_s": 0.3,
    "feedback_timeout_wall_s": 0.2,
    "request_stamp_tolerance_sim_s": 0.15,
    "position_excursion_limit_rad": [0.07, 0.002, 0.002, 0.002, 0.002, 0.002],
    "actual_speed_limit_rad_s": 0.04,
    "guard_limit_wall_s": 90.0,
    "sender_limit_wall_s": 45.0,
    "stop_limit_wall_s": 4.0,
    "stop_min_wall_s": 1.0,
    "stop_min_sim_s": 0.2,
    "stop_speed_limit_rad_s": 0.001,
    "time_note": "velocity profile uses joint feedback simulation time; deadlines and publication use monotonic wall time",
    "limitations": "simulation only; zero velocity on other joints is not position feedback hold; emergency zero bypasses rate limit; no guard crash watchdog; feedback receipt age is not absolute state age",
}


def profile(t):
    return C["peak_velocity_rad_s"] * math.sin(2.0*math.pi*t/C["motion_duration_sim_s"])


def slew(target, previous, sim_dt):
    step = C["command_rate_limit_rad_s2"]*min(max(sim_dt, 0.0), 0.04)
    return max(previous-step, min(previous+step, target))


class Guard(zero_hold.ZeroHold):
    def __init__(self, folder):
        super().__init__(folder)
        self.origin = None
        self.last_velocity = None
        self.last_request_wall = None
        self.last_request_stamp = None
        self.request_value = 0.0
        self.accepted = 0
        self.motion_complete = False
        self.output = 0.0
        self.output_stamp = None
        self.max_actual_speed = 0.0
        self.peak_excursion = [0.0]*6
        self.requests_file = (folder/"request_log.csv").open("w", newline="", encoding="utf-8")
        self.requests_log = csv.writer(self.requests_file)
        self.requests_log.writerow(["wall_elapsed_s", "source_sim_stamp_s", "accepted"] + [j+"_request_rad_s" for j in C["joints"]])
        self.request_sub = self.create_subscription(JointState, C["request_topic"], self.request, qos_profile_sensor_data)

    def receive(self, msg):
        super().receive(msg)
        if self.fault:
            return
        self.last_velocity = [msg.velocity[msg.name.index(j)] for j in C["joints"]]
        speed = max(abs(v) for v in self.last_velocity)
        self.max_actual_speed = max(self.max_actual_speed, speed)
        if speed > C["actual_speed_limit_rad_s"]:
            self.fault = "actual_speed_limit_exceeded"
        if self.origin is not None:
            excursion = [abs(a-b) for a, b in zip(self.positions, self.origin)]
            self.peak_excursion = [max(a, b) for a, b in zip(self.peak_excursion, excursion)]
            if any(a>b for a, b in zip(excursion, C["position_excursion_limit_rad"])):
                self.fault = self.fault or "position_excursion_exceeded"

    def request(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        values = list(msg.velocity)
        valid = (
            self.phase == "hold" and list(msg.name) == C["joints"] and len(values) == 6
            and all(math.isfinite(x) for x in values)
            and abs(values[0]) <= C["peak_velocity_rad_s"]+1e-9
            and all(x == 0.0 for x in values[1:])
            and self.last_sim is not None
            and abs(self.last_sim-stamp) <= C["request_stamp_tolerance_sim_s"]
            and (self.last_request_stamp is None or stamp >= self.last_request_stamp)
        )
        self.requests_log.writerow([now-self.started, stamp, valid]+values)
        if not valid:
            self.fault = self.fault or "invalid_or_stale_request"
            return
        self.last_request_stamp = stamp
        self.last_request_wall = now
        self.request_value = values[0]
        self.accepted += 1
        if msg.header.frame_id == "p2_motion_complete":
            if any(x != 0.0 for x in values):
                self.fault = self.fault or "nonzero_completion_request"
            else:
                self.motion_complete = True

    def emit(self, now):
        if now < self.next_publish:
            return
        dt = 0.0 if self.output_stamp is None else self.last_sim-self.output_stamp
        self.output = slew(self.request_value, self.output, dt)
        msg = Float64MultiArray()
        msg.data = [self.output, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(msg)
        self.sent += 1
        self.output_stamp = self.last_sim
        self.next_publish = now+C["publish_period_wall_s"]
        self.command_log.writerow([self.phase, now-self.started, self.last_sim]+list(msg.data))

    def stop_and_check(self):
        self.phase = "stopping"
        started = time.monotonic()
        while rclpy.ok() and time.monotonic()-started < C["stop_limit_wall_s"]:
            self.publish_zero(time.monotonic())
            rclpy.spin_once(self, timeout_sec=0.005)
            samples = self.stop_samples
            if not samples:
                continue
            good = []
            for sample in reversed(samples):
                if sample[2] > C["stop_speed_limit_rad_s"]:
                    break
                if good and (good[-1][0]-sample[0] > C["feedback_timeout_wall_s"] or good[-1][1] < sample[1]):
                    break
                good.append(sample)
                if good[0][1]-sample[1] >= C["stop_min_sim_s"]:
                    break
            if (len(good) >= 5 and good[0][1]-good[-1][1] >= C["stop_min_sim_s"]
                    and time.monotonic()-good[0][0] <= C["feedback_timeout_wall_s"]
                    and time.monotonic()-started >= C["stop_min_wall_s"]):
                return dict(verified=True, observed_sim_span_s=good[0][1]-good[-1][1],
                            max_abs_velocity_rad_s=max(x[2] for x in good), samples=len(good))
        return dict(verified=False, reason="insufficient_contiguous_fresh_low_speed_feedback_within_4_wall_seconds")


def run_guard(interrupted):
    root = Path.home()/"ROS2"
    folder = root/"arm/P2/data/data_v1"/(datetime.now().strftime("%Y%m%d_%H%M%S_%f")+"_joint_interface_test")
    folder.mkdir(parents=True, exist_ok=False)
    (folder/"config.yaml").write_text(json.dumps(C, indent=2), encoding="utf-8")
    dirty = zero_hold.git_read(root, "status", "--porcelain")
    run = dict(run_id=folder.name, started_at=datetime.now().astimezone().isoformat(),
               status="running", reason=None, code_commit=zero_hold.git_read(root, "rev-parse", "HEAD"),
               git_dirty=None if dirty is None else bool(dirty), git_status=dirty,
               source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               zero_hold_sha256=hashlib.sha256(Path(zero_hold.__file__).read_bytes()).hexdigest(),
               time_note=C["time_note"], limitations=C["limitations"])
    meta = folder/"run.json"
    meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
    node = None
    print("保存目录：", folder, flush=True)
    try:
        node = Guard(folder)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            if interrupted:
                run.update(status="interrupted", reason="signal_"+str(interrupted[0]))
                break
            if node.fault:
                raise RuntimeError(node.fault)
            node.check_controllers(now)
            if node.count_publishers(C["command_topic"]) > 1 or node.count_publishers(C["request_topic"]) > 1:
                raise RuntimeError("multiple_command_or_request_publishers")
            if now-node.started > C["guard_limit_wall_s"]:
                raise RuntimeError("guard_wall_timeout")
            if node.phase == "waiting":
                if now-node.started > 10.0:
                    raise RuntimeError("startup_checks_timeout")
                ready = (node.last_controller_check is not None and node.last_receive is not None
                         and now-node.last_receive <= C["feedback_timeout_wall_s"]
                         and node.last_sim > node.first_sim and node.publisher.get_subscription_count() > 0)
                if not ready:
                    continue
                node.origin = list(node.positions)
                node.min_position = list(node.positions)
                node.max_position = list(node.positions)
                node.phase = "hold"
                print("READY：先发零，等待 B 的单关节请求", flush=True)
            if now-node.last_receive > C["feedback_timeout_wall_s"]:
                raise RuntimeError("feedback_not_received_in_time")
            if now-node.last_advance > 0.5:
                raise RuntimeError("simulation_time_stalled")
            if node.last_controller_check is None or now-node.last_controller_check > 3.0:
                raise RuntimeError("controller_status_stale")
            if node.motion_complete:
                run.update(status="completed", reason="motion_complete_received")
                break
            if node.last_request_wall is not None:
                if now-node.last_request_wall > C["request_timeout_wall_s"]:
                    raise RuntimeError("request_stream_lost_without_completion")
                if abs(node.last_sim-node.last_request_stamp) > C["request_stamp_tolerance_sim_s"]:
                    raise RuntimeError("request_source_sim_stamp_stale")
            node.emit(now)
        else:
            raise RuntimeError("ros_context_stopped")
    except Exception as exc:
        run.update(status="failed", reason=str(exc))
    finally:
        if node is not None:
            try:
                run["stop_check"] = node.stop_and_check() if node.sent else dict(verified=False, reason="test_not_started")
            except Exception as exc:
                run["stop_check"] = dict(verified=False, reason=str(exc))
            if run["status"] == "completed" and not run["stop_check"]["verified"]:
                run.update(status="failed", reason="stop_not_verified")
            run.update(controllers_observed=node.controllers, accepted_requests=node.accepted,
                       motion_complete_received=node.motion_complete,
                       commands_published=node.sent, valid_samples=node.samples,
                       max_receive_gap_wall_s=node.max_gap, max_actual_speed_rad_s=node.max_actual_speed,
                       peak_excursion_rad=node.peak_excursion,
                       final_offset_rad=None if node.origin is None else [a-b for a, b in zip(node.positions, node.origin)],
                       total_wall_s=time.monotonic()-node.started)
            for stream in (node.feedback_file, node.command_file, node.requests_file):
                stream.close()
            node.destroy_node()
        run["finished_at"] = datetime.now().astimezone().isoformat()
        meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
        print(json.dumps(run, indent=2), flush=True)
    return 0 if run["status"] == "completed" else 1


def run_motion(interrupted):
    node = Node("p2_joint_interface_request")
    publisher = node.create_publisher(JointState, C["request_topic"], qos_profile_sensor_data)
    latest = {}
    def receive(msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if "sim" in latest and stamp < latest["sim"]:
            latest["fault"] = "simulation_time_reversed"
        if "sim" not in latest or stamp > latest["sim"]:
            latest["advance_wall"] = time.monotonic()
        latest.update(sim=stamp, stamp=msg.header.stamp, wall=time.monotonic())
    subscription = node.create_subscription(JointState, "/joint_states", receive, qos_profile_sensor_data)
    started = time.monotonic()
    start_sim = None
    last_publish = 0.0
    done_wall = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            if interrupted:
                break
            if now-started > C["sender_limit_wall_s"]:
                raise RuntimeError("sender_wall_timeout")
            if "fault" in latest:
                raise RuntimeError(latest["fault"])
            if "sim" not in latest or (start_sim is None and publisher.get_subscription_count() == 0):
                if now-started > 10.0:
                    raise RuntimeError("no_guard_or_feedback")
                continue
            if now-latest["wall"] > C["feedback_timeout_wall_s"] or now-latest["advance_wall"] > 0.5:
                raise RuntimeError("sender_feedback_stale_or_stalled")
            if start_sim is None:
                start_sim = latest["sim"]
                print("开始8秒仿真时间的第一关节缓慢往返", flush=True)
            elapsed = latest["sim"]-start_sim
            value = profile(elapsed) if elapsed < C["motion_duration_sim_s"] else 0.0
            if elapsed >= C["motion_duration_sim_s"]:
                if done_wall is None:
                    done_wall = now
                if now-done_wall >= 1.0:
                    print("运动请求正常结束；A将检测断流并验证静止", flush=True)
                    return 0
            if now-last_publish >= C["publish_period_wall_s"]:
                msg = JointState()
                msg.header.stamp = latest["stamp"]
                msg.header.frame_id = "p2_motion_complete" if elapsed >= C["motion_duration_sim_s"] else ""
                msg.name = C["joints"]
                msg.velocity = [value, 0.0, 0.0, 0.0, 0.0, 0.0]
                publisher.publish(msg)
                last_publish = now
        return 1
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["guard", "motion"])
    mode = parser.parse_args().mode
    interrupted = []
    handlers = {s: signal.signal(s, lambda number, frame: interrupted.append(number)) for s in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        return run_guard(interrupted) if mode == "guard" else run_motion(interrupted)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        for s, handler in handlers.items():
            signal.signal(s, handler)


if __name__ == "__main__":
    raise SystemExit(main())

