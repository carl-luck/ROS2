import csv
import argparse
import hashlib
import json
import math
import importlib.metadata

import numpy as np
import osqp
from scipy import sparse
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
    "controller": "kinematic_mpc",
    "kp_s_inv": 2.0,
    "horizon_steps": 25,
    "controller_mode": "mpc",
    "scenario": "normal",
    "weight_position": 100.0,
    "weight_velocity_reference": 1.0,
    "weight_command_difference": 0.1,
    "solver_time_limit_wall_s": 0.02,
    "solver_eps": 1e-6,
    "solver_max_iter": 10000,
    "preview_note": "MPC knows future reference; baseline uses current reference plus analytic feedforward",
    "model_note": "q_next=q+0.02*u; fixed prediction step despite measured cycle jitter; previous input is previous request, not confirmed guard output",
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


class JointMpc:
    def __init__(self):
        n = C["horizon_steps"]
        self.s = C["ts_sim_s"]*np.tril(np.ones((n, n)))
        self.d = np.eye(n)-np.eye(n, k=-1)
        h = C["weight_position"]*self.s.T@self.s
        h += C["weight_velocity_reference"]*np.eye(n)
        h += C["weight_command_difference"]*self.d.T@self.d
        self.solver = osqp.OSQP()
        self.solver.setup(P=sparse.csc_matrix(np.triu(2*h)), q=np.zeros(n),
                          A=sparse.csc_matrix(np.vstack([np.eye(n), self.d])),
                          l=-np.ones(2*n), u=np.ones(2*n), verbose=False,
                          eps_abs=C["solver_eps"], eps_rel=C["solver_eps"],
                          max_iter=C["solver_max_iter"], time_limit=C["solver_time_limit_wall_s"])
        self.records = []

    def command(self, q, previous, elapsed, q0, measured_dt):
        if C["controller_mode"] == "baseline":
            target, velocity = reference(elapsed, q0)
            raw, command = control(q, target, velocity, previous, measured_dt)
            self.records.append(dict(elapsed_sim_s=elapsed, solve_wall_s=0.0,
                status="baseline_no_solver", iterations=0, fallback=False,
                raw_command_rad_s=raw, requested_rad_s=command,
                limiter_correction_rad_s=command-raw))
            return raw, command
        started = time.perf_counter()
        n = C["horizon_steps"]
        future = [reference(elapsed+(i+1)*C["ts_sim_s"], q0) for i in range(n)]
        targets = np.array([x[0] for x in future])
        velocities = np.array([x[1] for x in future])
        prior = np.zeros(n)
        prior[0] = previous
        linear = C["weight_position"]*self.s.T@(q-targets)
        linear -= C["weight_velocity_reference"]*velocities
        linear -= C["weight_command_difference"]*self.d.T@prior
        delta = C["command_rate_limit_rad_s2"]*C["ts_sim_s"]
        first_delta = C["command_rate_limit_rad_s2"]*measured_dt
        differences = np.full(n, delta)
        differences[0] = first_delta
        lower = np.r_[np.full(n, -C["velocity_limit_rad_s"]), prior-differences]
        upper = np.r_[np.full(n, C["velocity_limit_rad_s"]), prior+differences]
        status = "exception"
        iterations = 0
        raw = None
        try:
            self.solver.update(q=2*linear, l=lower, u=upper)
            result = self.solver.solve()
            status = result.info.status
            iterations = result.info.iter
            if result.info.status_val == 1 and result.x is not None and np.all(np.isfinite(result.x)):
                candidate = np.r_[result.x, self.d@result.x]
                if np.all(candidate >= lower-1e-5) and np.all(candidate <= upper+1e-5):
                    raw = float(result.x[0])
                else:
                    status += "; predicted_constraint_residual_exceeded"
        except Exception as exc:
            status = str(exc)
        solve_time = time.perf_counter()-started
        if solve_time > C["solver_time_limit_wall_s"]:
            raw = None
            status += "; wall_time_limit_exceeded"
        fallback = raw is None
        target, velocity = reference(elapsed, q0)
        if fallback:
            raw, command = control(q, target, velocity, previous, measured_dt)
        else:
            vmax = C["velocity_limit_rad_s"]
            raw_step = C["command_rate_limit_rad_s2"]*measured_dt
            command = max(max(-vmax, previous-raw_step), min(min(vmax, previous+raw_step), raw))
        self.records.append(dict(elapsed_sim_s=elapsed, solve_wall_s=solve_time, status=status,
                                 iterations=iterations, fallback=fallback, raw_command_rad_s=raw,
                                 requested_rad_s=command, limiter_correction_rad_s=command-raw))
        return raw, command


class Tracking(Node):
    def __init__(self, folder):
        super().__init__("p2_joint_tracking_"+C["controller_mode"])
        self.started = time.monotonic()
        self.latest = None
        self.fault = None
        self.last_advance_wall = None
        self.initial = None
        self.q0 = None
        self.start_sim = None
        self.rows = []
        self.command_wall = None
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
        if command != 0.0:
            now = time.monotonic()
            if (now-self.latest["wall"] > C["feedback_timeout_wall_s"]
                    or self.command_wall is None
                    or now-self.command_wall > C["feedback_timeout_wall_s"]):
                raise RuntimeError("publish_blocked_stale_feedback_or_control")
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
                  requested_speed_max_rad_s=float(max(abs(r["requested_rad_s"]) for r in rows)),
                  solve_median_wall_ms=float(np.median([r["solve_wall_s"] for r in node.mpc_records])*1000) if C["controller_mode"] == "mpc" else None,
                  solve_p95_wall_ms=float(np.percentile([r["solve_wall_s"] for r in node.mpc_records],95)*1000) if C["controller_mode"] == "mpc" else None,
                  fallback_count=sum(r["fallback"] for r in node.mpc_records),
                  limiter_correction_count=sum(abs(r["limiter_correction_rad_s"])>1e-5 for r in node.mpc_records))
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=["baseline", "mpc"], required=True)
    parser.add_argument("--scenario", choices=["normal", "near_limit", "limited"], required=True)
    args = parser.parse_args()
    periods = dict(normal=8.0, near_limit=3.5, limited=2.5)
    C["controller_mode"] = args.controller
    C["controller"] = "velocity_feedforward_plus_position_p" if args.controller == "baseline" else "kinematic_mpc"
    C["scenario"] = args.scenario
    C["reference_period_sim_s"] = periods[args.scenario]
    C["duration_sim_s"] = 2.0*periods[args.scenario]
    C["reference"] = "q0+0.01*(1-cos(2*pi*t/T)); two cycles, then q0 hold; T from reference_period_sim_s"
    root = Path.home()/"ROS2"
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")+"_gazebo_"+C["controller_mode"]+"_"+C["scenario"]
    folder = root/"arm/P2/data/data_v2"/run_id
    folder.mkdir(parents=True, exist_ok=False)
    (folder/"config.yaml").write_text(json.dumps(dict(tracking=C, guard=interface.C), indent=2), encoding="utf-8")
    dirty = git_read(root, "status", "--porcelain")
    run = dict(run_id=run_id, started_at=datetime.now().astimezone().isoformat(), status="running", reason=None,
               code_commit=git_read(root, "rev-parse", "HEAD"), git_dirty=None if dirty is None else bool(dirty), git_status=dirty,
               source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               dependencies={p: importlib.metadata.version(p) for p in ("numpy", "scipy", "osqp", "matplotlib")},
               guard_sha256=hashlib.sha256(Path(interface.__file__).read_bytes()).hexdigest(),
               time_note=C["timing"], limitations=C["limitations"], completion_sent=False)
    meta = folder/"run.json"
    meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
    interrupted = []
    handlers = {s: signal.signal(s, lambda number, frame: interrupted.append(number)) for s in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    mpc = None
    print("保存目录：", folder, flush=True)
    try:
        mpc = JointMpc()
        node = Tracking(folder)
        previous = 0.0
        last_control_sim = None
        next_publish_wall = 0.0
        last_control_wall = None
        complete_at = None
        previous_loop_wall = time.monotonic()
        max_loop_gap = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.002)
            now = time.monotonic()
            loop_gap = now-previous_loop_wall
            max_loop_gap = max(max_loop_gap, loop_gap)
            previous_loop_wall = now
            run["max_loop_gap_wall_s"] = max_loop_gap
            if node.start_sim is not None and loop_gap > C["feedback_timeout_wall_s"]:
                run["fault_context"] = dict(loop_gap_wall_s=loop_gap,
                    feedback_receive_age_wall_s=None if node.latest is None else now-node.latest["wall"],
                    control_age_wall_s=None if last_control_wall is None else now-last_control_wall,
                    latest_sim_stamp_s=None if node.latest is None else node.latest["sim"],
                    last_control_sim_stamp_s=last_control_sim)
                raise RuntimeError("control_loop_wall_gap_exceeded")
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
                print("闭环开始：", C["controller_mode"], C["scenario"], "周期", C["reference_period_sim_s"], flush=True)
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
                    used_fallback = any(r["fallback"] for r in mpc.records)
                    run.update(status="degraded" if used_fallback else "completed", reason="MPC_used_baseline_fallback" if used_fallback else "reference_finished_completion_sent_check_guard_result")
                    break
                if now >= next_publish_wall:
                    node.publish(0.0, complete=True)
                    next_publish_wall = now+C["heartbeat_wall_s"]
                continue
            control_dt = C["ts_sim_s"] if last_control_sim is None else state["sim"]-last_control_sim
            if last_control_sim is None or control_dt >= C["ts_sim_s"]-1e-9:
                if control_dt > C["maximum_cycle_sim_s"]:
                    run["fault_context"] = dict(control_gap_sim_s=control_dt,
                        latest_sim_stamp_s=state["sim"], last_control_sim_stamp_s=last_control_sim)
                    raise RuntimeError("control_cycle_gap_exceeded")
                started = time.perf_counter()
                target, velocity = reference(elapsed, node.q0)
                raw, command = mpc.command(state["q"][0], previous, elapsed, node.q0, control_dt)
                compute = time.perf_counter()-started
                node.rows.append(dict(elapsed_sim_s=elapsed, cycle_sim_s=control_dt,
                                      position_rad=state["q"][0], reference_rad=target,
                                      velocity_rad_s=state["v"][0], reference_velocity_rad_s=velocity,
                                      error_rad=state["q"][0]-target, raw_rad_s=raw,
                                      requested_rad_s=command, compute_wall_s=compute))
                previous = command
                last_control_sim = state["sim"]
                last_control_wall = now
                node.command_wall = now
            now = time.monotonic()
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
        if mpc is not None and mpc.records:
            with (folder/"solver_log.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(mpc.records[0]))
                writer.writeheader()
                writer.writerows(mpc.records)
            run["fallback_count"] = sum(r["fallback"] for r in mpc.records)
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
                    node.mpc_records = mpc.records
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
