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

from ur5e_mpc_control import joint_interface_test as interface
from ur5e_mpc_control.zero_hold import git_read


C = {
    "controller": "velocity_feedforward_plus_position_p",
    "kp_s_inv": 2.0,
    "ts_sim_s": 0.02,
    "reference_scale_rad": 0.01,
    "reference_period_sim_s": 8.0,
    "duration_sim_s": 16.0,
    "settle_sim_s": 1.0,
    "velocity_limit_rad_s": 0.02,
    "command_rate_limit_rad_s2": 0.1,
    "heartbeat_wall_s": 0.02,
    "feedback_timeout_wall_s": 0.2,
    "sim_stall_wall_s": 0.5,
    "maximum_wall_s": 80.0,
    "maximum_cycle_sim_s": 0.1,
    "reference": "q0 + 0.01*(1-cos(2*pi*t/8)); range q0 to q0+0.02; two cycles then q0 hold",
    "timing": "reference and control scheduling use JointState simulation stamps; heartbeat, freshness and compute timing use monotonic wall time",
    "limitations": "simulation only; other joints receive zero velocity, not position feedback; guard output may differ from request; completion needs separate guard stop verification; no guard crash watchdog",
}


def reference(t, q0):
    if t >= C["duration_sim_s"]:
        return q0, 0.0
    omega = 2.0*math.pi/C["reference_period_sim_s"]
    return q0+C["reference_scale_rad"]*(1.0-math.cos(omega*t)), C["reference_scale_rad"]*omega*math.sin(omega*t)


def control(q, target, velocity, previous, dt):
    raw = velocity+C["kp_s_inv"]*(target-q)
    delta = C["command_rate_limit_rad_s2"]*dt
    limited = max(-C["velocity_limit_rad_s"], min(C["velocity_limit_rad_s"], raw))
    return raw, max(previous-delta, min(previous+delta, limited))


class Tracking(Node):
    def __init__(self, folder):
        super().__init__("p2_joint_tracking_baseline")
        self.started = time.monotonic()
        self.latest = None
        self.fault = None
        self.last_advance_wall = None
        self.initial = None
        self.q0 = None
        self.start_sim = None
        self.rows = []
        self.files = []
        self.feedback_log = self.log_file(folder/"joint_log.csv", ["wall_elapsed_s", "sim_stamp_s"]+[j+"_position_rad" for j in interface.C["joints"]]+[j+"_velocity_rad_s" for j in interface.C["joints"]])
        self.request_log = self.log_file(folder/"request_log.csv", ["wall_elapsed_s", "source_sim_stamp_s", "completion", "requested_first_joint_rad_s"])
        self.applied_log = self.log_file(folder/"observed_command_log.csv", ["receive_wall_elapsed_s", "latest_feedback_sim_stamp_s"]+[j+"_command_rad_s" for j in interface.C["joints"]])
        self.publisher = self.create_publisher(JointState, interface.C["request_topic"], qos_profile_sensor_data)
        self.state_sub = self.create_subscription(JointState, "/joint_states", self.receive, qos_profile_sensor_data)
        self.command_sub = self.create_subscription(Float64MultiArray, interface.C["command_topic"], self.receive_command, qos_profile_sensor_data)

    def log_file(self, path, header):
        stream = path.open("w", newline="", encoding="utf-8")
        self.files.append(stream)
        writer = csv.writer(stream)
        writer.writerow(header)
        return writer

    def receive(self, msg):
        now = time.monotonic()
        try:
            if len(set(msg.name)) != len(msg.name):
                raise ValueError("duplicate_joint_names")
            indices = [msg.name.index(j) for j in interface.C["joints"]]
            q = [msg.position[i] for i in indices]
            v = [msg.velocity[i] for i in indices]
            if not all(math.isfinite(x) for x in q+v):
                raise ValueError("nonfinite_state")
        except (ValueError, IndexError) as exc:
            self.fault = "invalid_feedback: "+str(exc)
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.latest is not None and stamp < self.latest["sim"]:
            self.fault = "simulation_time_reversed"
        if self.latest is None or stamp > self.latest["sim"]:
            self.last_advance_wall = now
        self.latest = dict(sim=stamp, stamp=msg.header.stamp, wall=now, q=q, v=v)
        self.feedback_log.writerow([now-self.started, stamp]+q+v)

    def receive_command(self, msg):
        self.applied_log.writerow([time.monotonic()-self.started, None if self.latest is None else self.latest["sim"]]+list(msg.data))

    def publish(self, command, complete=False):
        msg = JointState()
        msg.header.stamp = self.latest["stamp"]
        msg.header.frame_id = "p2_motion_complete" if complete else ""
        msg.name = interface.C["joints"]
        msg.velocity = [command, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(msg)
        self.request_log.writerow([time.monotonic()-self.started, self.latest["sim"], complete, command])


def analyze(folder, node):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    rows = node.rows
    keys = list(rows[0])
    with (folder/"control_log.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    times = np.array([r["elapsed_sim_s"] for r in rows])
    errors = np.array([r["error_rad"] for r in rows])
    dt = np.diff(times)
    duration = times[-1]-times[0]
    integral_squared = np.sum((errors[:-1]**2+errors[1:]**2)*0.5*dt)
    metric = dict(controller=C["controller"], analyzed_duration_sim_s=float(duration),
                  time_weighted_rmse_rad=float(math.sqrt(integral_squared/duration)),
                  max_abs_error_rad=float(np.max(np.abs(errors))),
                  iae_rad_s=float(np.sum((np.abs(errors[:-1])+np.abs(errors[1:]))*0.5*dt)),
                  compute_p95_wall_ms=float(np.percentile([r["compute_wall_s"] for r in rows], 95)*1000),
                  compute_over_ts_rate=float(np.mean([r["compute_wall_s"]>C["ts_sim_s"] for r in rows])),
                  cycle_gap_over_1p5_ts_rate=float(np.mean(dt>1.5*C["ts_sim_s"])),
                  max_control_gap_sim_s=float(np.max(dt)),
                  requested_speed_max_rad_s=float(max(abs(r["requested_rad_s"]) for r in rows)))
    with (folder/"metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metric))
        writer.writeheader()
        writer.writerow(metric)
    figure_folder = folder.parents[2]/"figures"/folder.name
    figure_folder.mkdir(parents=True, exist_ok=False)
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(times, [r["position_rad"]-node.q0 for r in rows], label="measured relative position")
    axes[0].plot(times, [r["reference_rad"]-node.q0 for r in rows], "--", label="reference")
    axes[0].set_ylabel("Position offset (rad)")
    axes[1].plot(times, errors, label="measured minus reference")
    axes[1].set_ylabel("Error (rad)")
    axes[2].plot(times, [r["requested_rad_s"] for r in rows], label="requested velocity")
    axes[2].plot(times, [r["velocity_rad_s"] for r in rows], label="measured velocity")
    axes[2].set_ylabel("Velocity (rad/s)")
    axes[2].set_xlabel("Simulation time since tracking start (s)")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(figure_folder/"tracking.png", dpi=180)
    fig.savefig(figure_folder/"tracking.svg")
    plt.close(fig)
    return metric, figure_folder


def main():
    root = Path.home()/"ROS2"
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")+"_gazebo_baseline"
    folder = root/"arm/P2/data/data_v1"/run_id
    folder.mkdir(parents=True, exist_ok=False)
    (folder/"config.yaml").write_text(json.dumps(dict(tracking=C, guard=interface.C), indent=2), encoding="utf-8")
    dirty = git_read(root, "status", "--porcelain")
    run = dict(run_id=run_id, started_at=datetime.now().astimezone().isoformat(), status="running", reason=None,
               code_commit=git_read(root, "rev-parse", "HEAD"), git_dirty=None if dirty is None else bool(dirty), git_status=dirty,
               source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               guard_sha256=hashlib.sha256(Path(interface.__file__).read_bytes()).hexdigest(),
               time_note=C["timing"], limitations=C["limitations"], completion_sent=False)
    meta = folder/"run.json"
    meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
    interrupted = []
    handlers = {s: signal.signal(s, lambda number, frame: interrupted.append(number)) for s in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    print("保存目录：", folder, flush=True)
    try:
        node = Tracking(folder)
        previous = 0.0
        last_control_sim = None
        next_publish_wall = 0.0
        last_control_wall = None
        complete_at = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.002)
            now = time.monotonic()
            if interrupted:
                run.update(status="interrupted", reason="signal_"+str(interrupted[0]))
                break
            if node.fault:
                raise RuntimeError(node.fault)
            if now-node.started > C["maximum_wall_s"]:
                raise RuntimeError("tracking_wall_timeout")
            if node.start_sim is None:
                if now-node.started > 10.0:
                    raise RuntimeError("startup_no_guard_or_feedback")
                if node.latest is None or node.publisher.get_subscription_count() == 0:
                    continue
                node.initial = list(node.latest["q"])
                node.q0 = node.initial[0]
                node.start_sim = node.latest["sim"]
                print("闭环开始：16秒仿真时间，两次小幅往返，再保持1秒", flush=True)
            state = node.latest
            if now-state["wall"] > C["feedback_timeout_wall_s"] or now-node.last_advance_wall > C["sim_stall_wall_s"]:
                raise RuntimeError("feedback_stale_or_simulation_stalled")
            if node.count_publishers(interface.C["request_topic"]) > 1:
                raise RuntimeError("another_request_publisher")
            if node.publisher.get_subscription_count() == 0 and complete_at is None:
                raise RuntimeError("guard_disappeared")
            if any(abs(a-b)>limit for a, b, limit in zip(state["q"], node.initial, interface.C["position_excursion_limit_rad"])):
                raise RuntimeError("position_excursion_limit")
            elapsed = state["sim"]-node.start_sim
            if elapsed >= C["duration_sim_s"]+C["settle_sim_s"]:
                if complete_at is None:
                    complete_at = now
                    node.publish(0.0, complete=True)
                    run["completion_sent"] = True
                if now-complete_at >= 0.5:
                    run.update(status="completed", reason="reference_finished_completion_sent_check_guard_result")
                    break
                if now >= next_publish_wall:
                    node.publish(0.0, complete=True)
                    next_publish_wall = now+C["heartbeat_wall_s"]
                continue
            control_dt = C["ts_sim_s"] if last_control_sim is None else state["sim"]-last_control_sim
            if last_control_sim is None or control_dt >= C["ts_sim_s"]-1e-9:
                if control_dt > C["maximum_cycle_sim_s"]:
                    raise RuntimeError("control_cycle_gap_exceeded")
                started = time.perf_counter()
                target, velocity = reference(elapsed, node.q0)
                raw, command = control(state["q"][0], target, velocity, previous, control_dt)
                compute = time.perf_counter()-started
                node.rows.append(dict(elapsed_sim_s=elapsed, cycle_sim_s=control_dt,
                                      position_rad=state["q"][0], reference_rad=target,
                                      velocity_rad_s=state["v"][0], reference_velocity_rad_s=velocity,
                                      error_rad=state["q"][0]-target, raw_rad_s=raw,
                                      requested_rad_s=command, compute_wall_s=compute))
                previous = command
                last_control_sim = state["sim"]
                last_control_wall = now
            if last_control_wall is None or now-last_control_wall > C["feedback_timeout_wall_s"]:
                raise RuntimeError("control_update_stale")
            if now >= next_publish_wall:
                node.publish(previous)
                next_publish_wall = now+C["heartbeat_wall_s"]
        else:
            raise RuntimeError("ros_context_stopped")
    except Exception as exc:
        run.update(status="failed", reason=str(exc))
    finally:
        if node is not None:
            if not run["completion_sent"] and node.latest is not None:
                try:
                    node.publish(0.0)
                except Exception:
                    pass
            for stream in node.files:
                stream.close()
            if len(node.rows)>1:
                try:
                    metric, figure_folder = analyze(folder, node)
                    run["metrics_scope"] = "observed samples only; check status and duration before comparison"
                    print(json.dumps(metric, indent=2), flush=True)
                    print("Figures:", figure_folder, flush=True)
                except Exception as exc:
                    run["analysis_error"] = str(exc)
            run.update(initial_position_rad=node.q0, control_samples=len(node.rows), total_wall_s=time.monotonic()-node.started)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for s, handler in handlers.items():
            signal.signal(s, handler)
        run["finished_at"] = datetime.now().astimezone().isoformat()
        meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
        print(json.dumps(run, indent=2), flush=True)
    return 0 if run["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
