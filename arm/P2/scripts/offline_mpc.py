import argparse
import csv
import hashlib
import importlib.metadata
import json
import platform
import subprocess
import time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import osqp
from scipy import sparse


CONFIG = {
    "experiment": "offline_mpc_sine",
    "model": "ideal single joint integrator; no ROS or Gazebo",
    "ts_s": 0.02,
    "duration_model_s": 12.0,
    "horizon_steps": 25,
    "amplitude_rad": 0.08,
    "reference_period_s": 6.0,
    "initial_position_rad": 0.0,
    "initial_command_rad_s": 0.0,
    "velocity_limit_rad_s": 0.12,
    "command_rate_limit_rad_s2": 0.5,
    "baseline_kp_s_inv": 2.0,
    "weight_position": 100.0,
    "weight_velocity_reference": 1.0,
    "weight_command_difference": 0.1,
    "solver_time_limit_wall_s": 0.02,
    "solver_eps_abs": 1e-6,
    "solver_eps_rel": 1e-6,
    "solver_max_iter": 10000,
    "constraint_metric_tolerance": 1e-5,
    "preview": "MPC uses known future reference; baseline uses current reference and analytic velocity feedforward",
    "limits_note": "same applied velocity and command difference limits; no position limits; command rate is not a physical acceleration guarantee",
    "time_note": "model time advances exactly Ts; wall time measures computation, not real-time ROS scheduling",
}


def reference(t):
    omega = 2.0 * np.pi / CONFIG["reference_period_s"]
    return CONFIG["amplitude_rad"] * np.sin(omega * t), CONFIG["amplitude_rad"] * omega * np.cos(omega * t)


def limit_command(raw, previous):
    vmax = CONFIG["velocity_limit_rad_s"]
    delta = CONFIG["command_rate_limit_rad_s2"] * CONFIG["ts_s"]
    return float(np.clip(raw, max(-vmax, previous-delta), min(vmax, previous+delta)))


class JointMpc:
    def __init__(self):
        n = CONFIG["horizon_steps"]
        self.s = CONFIG["ts_s"] * np.tril(np.ones((n, n)))
        self.d = np.eye(n) - np.eye(n, k=-1)
        h = CONFIG["weight_position"] * self.s.T @ self.s
        h += CONFIG["weight_velocity_reference"] * np.eye(n)
        h += CONFIG["weight_command_difference"] * self.d.T @ self.d
        self.constraints = sparse.csc_matrix(np.vstack([np.eye(n), self.d]))
        self.solver = osqp.OSQP()
        self.solver.setup(
            P=sparse.csc_matrix(np.triu(2.0*h)), q=np.zeros(n),
            A=self.constraints, l=-np.ones(2*n), u=np.ones(2*n),
            verbose=False, eps_abs=CONFIG["solver_eps_abs"],
            eps_rel=CONFIG["solver_eps_rel"], max_iter=CONFIG["solver_max_iter"],
            time_limit=CONFIG["solver_time_limit_wall_s"],
        )

    def command(self, position, previous, t):
        n = CONFIG["horizon_steps"]
        target, velocity = reference(t + CONFIG["ts_s"] * np.arange(1, n+1))
        prior = np.zeros(n)
        prior[0] = previous
        linear = CONFIG["weight_position"] * self.s.T @ (position-target)
        linear -= CONFIG["weight_velocity_reference"] * velocity
        linear -= CONFIG["weight_command_difference"] * self.d.T @ prior
        vmax = CONFIG["velocity_limit_rad_s"]
        delta = CONFIG["command_rate_limit_rad_s2"] * CONFIG["ts_s"]
        lower = np.r_[np.full(n, -vmax), prior-delta]
        upper = np.r_[np.full(n, vmax), prior+delta]
        started = time.perf_counter()
        self.solver.update(q=2.0*linear, l=lower, u=upper)
        result = self.solver.solve()
        elapsed = time.perf_counter()-started
        valid = (
            result.info.status_val == 1 and result.x is not None
            and np.all(np.isfinite(result.x))
            and elapsed <= CONFIG["solver_time_limit_wall_s"]
        )
        return (float(result.x[0]) if valid else None), elapsed, result.info.status, result.info.iter


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def git_read(root, *args):
    try:
        result = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True, timeout=5)
        return result.stdout.strip() if result.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def simulate(name, mpc, joint_rows, solver_rows):
    ts = CONFIG["ts_s"]
    count = round(CONFIG["duration_model_s"]/ts)
    position = CONFIG["initial_position_rad"]
    previous = CONFIG["initial_command_rad_s"]
    positions = [position]
    commands, raw_commands, durations, solve_times = [], [], [], []
    fallbacks = 0
    for k in range(count):
        t = k*ts
        target, velocity = reference(t)
        start = time.perf_counter()
        fallback = float(velocity + CONFIG["baseline_kp_s_inv"]*(target-position))
        raw = fallback
        if name == "mpc":
            raw, elapsed, status, iterations = mpc.command(position, previous, t)
            failed = raw is None
            if failed:
                raw = fallback
                fallbacks += 1
            solve_times.append(elapsed)
            solver_rows.append(dict(step=k, model_time_s=t, solve_wall_s=elapsed,
                                    status=status, iterations=iterations, fallback=failed))
        command = limit_command(raw, previous)
        compute = time.perf_counter()-start
        joint_rows.append(dict(controller=name, step=k, model_time_s=t,
                               position_rad=position, reference_rad=float(target),
                               reference_velocity_rad_s=float(velocity),
                               error_rad=float(position-target), raw_command_rad_s=raw,
                               applied_command_rad_s=command,
                               command_rate_rad_s2=(command-previous)/ts,
                               compute_wall_s=compute))
        position += ts*command
        positions.append(position)
        commands.append(command)
        raw_commands.append(raw)
        durations.append(compute)
        previous = command
    times = np.arange(count+1)*ts
    error = np.array(positions)-reference(times)[0]
    commands = np.array(commands)
    changes = np.diff(np.r_[CONFIG["initial_command_rad_s"], commands])
    speed_violation = np.maximum(np.abs(commands)-CONFIG["velocity_limit_rad_s"], 0.0)
    rate_violation = np.maximum(np.abs(changes)/ts-CONFIG["command_rate_limit_rad_s2"], 0.0)
    tol = CONFIG["constraint_metric_tolerance"]
    metric = dict(
        controller=name, rmse_rad=float(np.sqrt(np.mean(error**2))),
        max_abs_error_rad=float(np.max(np.abs(error))),
        iae_rad_s=float(np.sum((np.abs(error[:-1])+np.abs(error[1:]))*0.5*ts)),
        velocity_violation_count=int(np.sum(speed_violation>tol)),
        max_velocity_violation_rad_s=float(np.max(speed_violation)),
        command_rate_violation_count=int(np.sum(rate_violation>tol)),
        max_command_rate_violation_rad_s2=float(np.max(rate_violation)),
        command_total_variation_rad_s=float(np.sum(np.abs(changes))),
        command_rate_rms_rad_s2=float(np.sqrt(np.mean((changes/ts)**2))),
        limiter_correction_count=int(np.sum(np.abs(np.array(raw_commands)-commands)>tol)),
        solve_median_wall_ms=float(np.median(solve_times)*1000) if solve_times else None,
        solve_p95_wall_ms=float(np.percentile(solve_times, 95)*1000) if solve_times else None,
        compute_over_ts_rate=float(np.mean(np.array(durations)>ts)),
        fallback_count=fallbacks,
    )
    return times, positions, commands, metric


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, default=Path.home()/"ROS2/arm/P2")
    args = parser.parse_args()
    root = args.project_root.expanduser().resolve()
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")+"_offline_mpc_sine"
    folder = root/"data/data_v1"/run_id
    folder.mkdir(parents=True, exist_ok=False)
    figure_folder = root/"figures"/run_id
    figure_folder.mkdir(parents=True, exist_ok=False)
    (folder/"config.yaml").write_text(json.dumps(CONFIG, indent=2), encoding="utf-8")
    dirty = git_read(root, "status", "--porcelain")
    run = dict(run_id=run_id, started_at=datetime.now().astimezone().isoformat(),
               status="running", reason=None, scope="ideal model only; no ROS or Gazebo validation",
               code_commit=git_read(root, "rev-parse", "HEAD"), git_dirty=None if dirty is None else bool(dirty),
               git_status=dirty, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               python=platform.python_version(), platform=platform.platform(),
               dependencies={p: importlib.metadata.version(p) for p in ("numpy", "scipy", "osqp", "matplotlib")},
               time_note=CONFIG["time_note"])
    meta = folder/"run.json"
    meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
    joint_rows, solver_rows, metrics = [], [], []
    started = time.perf_counter()
    try:
        mpc = JointMpc()
        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        for name in ("baseline", "mpc"):
            times, positions, commands, metric = simulate(name, mpc, joint_rows, solver_rows)
            metrics.append(metric)
            axes[0].plot(times, positions, label=name)
            axes[1].plot(times, np.array(positions)-reference(times)[0], label=name)
            axes[2].step(times[:-1], commands, where="post", label=name)
        axes[0].plot(times, reference(times)[0], "k--", label="reference")
        axes[0].set_ylabel("Position (rad)")
        axes[1].set_ylabel("Error (rad)")
        axes[2].set_ylabel("Command (rad/s)")
        axes[2].set_xlabel("Ideal model time (s)")
        for sign in (-1, 1):
            axes[2].axhline(sign*CONFIG["velocity_limit_rad_s"], color="gray", linestyle=":")
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.legend()
        fig.suptitle("Single joint ideal model: feedforward + P vs MPC")
        fig.tight_layout()
        fig.savefig(figure_folder/"tracking.png", dpi=180)
        fig.savefig(figure_folder/"tracking.svg")
        plt.close(fig)
        write_csv(folder/"metrics.csv", metrics)
        write_csv(folder/"reference.csv", [dict(model_time_s=float(t), position_rad=float(q), velocity_rad_s=float(v))
                  for t, q, v in zip(times, *reference(times))])
        run["status"] = "completed" if not any(m["fallback_count"] for m in metrics) else "degraded"
        run["reason"] = None if run["status"] == "completed" else "MPC used fallback; do not report as a clean MPC comparison"
        print(json.dumps(metrics, indent=2))
    except Exception as exc:
        run.update(status="failed", reason=str(exc))
        raise
    finally:
        if joint_rows:
            write_csv(folder/"joint_log.csv", joint_rows)
        if solver_rows:
            write_csv(folder/"solver_log.csv", solver_rows)
        run.update(finished_at=datetime.now().astimezone().isoformat(), total_wall_s=time.perf_counter()-started)
        meta.write_text(json.dumps(run, indent=2), encoding="utf-8")
        print("Data:", folder)
        print("Figures:", figure_folder)
        print("Status:", run["status"])
    return 0 if run["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
