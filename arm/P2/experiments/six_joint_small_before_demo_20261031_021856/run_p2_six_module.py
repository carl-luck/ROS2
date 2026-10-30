import argparse
import csv
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from ur5e_mpc_control.p2_six_core import C, JOINTS


def save(path, value):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")
    temp.replace(path)


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def wait(process, timeout, interrupted, progress_log=None):
    deadline = time.monotonic()+timeout
    last_line = None
    next_read = 0.0
    while process.poll() is None:
        if interrupted or time.monotonic() > deadline:
            raise RuntimeError("batch_interrupted_or_child_timeout")
        if progress_log is not None and time.monotonic() >= next_read:
            next_read = time.monotonic()+0.5
            text = progress_log.read_text(encoding="utf-8", errors="replace")
            lines = [line for line in text.splitlines() if line.startswith(("进度：", "六关节闭环开始"))]
            if lines and lines[-1] != last_line:
                last_line = lines[-1]
                print(last_line, flush=True)
        time.sleep(0.05)
    return process.returncode


def stop(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            raise RuntimeError("child_did_not_exit; stop_Gazebo_in_launch_terminal")


def spawn(arguments, path, env):
    stream = path.open("w", encoding="utf-8")
    try:
        process = subprocess.Popen([sys.executable, "-m", "ur5e_mpc_control.p2_six_control"]+arguments,
                                   stdout=stream, stderr=subprocess.STDOUT, env=env)
    except Exception:
        stream.close()
        raise
    return process, stream


def analyze(folder, guard_folder):
    controls = rows(folder/"control_log.csv")
    outputs = rows(guard_folder/"output_log.csv")
    t = np.array([float(r["elapsed_sim_s"]) for r in controls])
    dt = np.diff(t)
    if len(t) < 2 or np.any(dt <= 0) or t[-1]-t[0] < C["duration"]+C["settle"]-0.05:
        raise RuntimeError("incomplete_or_nonmonotonic_tracking_window")
    errors = np.array([[float(r[j+"_error_rad"]) for j in JOINTS] for r in controls])
    commands = np.array([[float(r[j+"_request_rad_s"]) for j in JOINTS] for r in controls])
    rates = np.diff(np.vstack([np.zeros(6), commands]), axis=0)/np.array([float(r["cycle_sim_s"]) for r in controls])[:, None]
    observed_outputs = np.array([[float(r[j+"_command_rad_s"]) for j in JOINTS] for r in outputs])
    if np.any(np.abs(commands) > np.asarray(C["vmax"])+1e-5) or np.any(np.abs(rates) > np.asarray(C["rate"])+1e-5):
        raise RuntimeError("request_speed_or_rate_limit_violation")
    if np.any(np.abs(observed_outputs) > np.asarray(C["vmax"])+1e-5):
        raise RuntimeError("guard_output_speed_limit_violation")
    metrics = []
    for i, j in enumerate(JOINTS):
        e = errors[:, i]
        metric = dict(joint=j, analyzed_duration_sim_s=float(t[-1]-t[0]),
                      rmse_rad=float(np.sqrt(np.trapz(e*e, t)/(t[-1]-t[0]))),
                      iae_rad_s=float(np.trapz(np.abs(e), t)), max_abs_error_rad=float(np.max(np.abs(e))),
                      command_total_variation_rad_s=float(np.sum(np.abs(np.diff(commands[:, i])))),
                      command_rate_rms_rad_s2=float(np.sqrt(np.mean(rates[:, i]**2))),
                      max_requested_speed_rad_s=float(np.max(np.abs(commands[:, i]))))
        metrics.append(metric)
    with (folder/"metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(6, 2, figsize=(13, 14), sharex=True)
    for i, j in enumerate(JOINTS):
        position = np.array([float(r[j+"_position_rad"]) for r in controls])
        target = np.array([float(r[j+"_reference_rad"]) for r in controls])
        axes[i, 0].plot(t, position-position[0], label="measured")
        axes[i, 0].plot(t, target-position[0], "--", label="reference")
        axes[i, 0].set_ylabel(j+"\n(rad)")
        axes[i, 1].plot(t, errors[:, i])
        axes[i, 1].set_ylabel("Error (rad)")
        for ax in axes[i]:
            ax.grid(alpha=0.3)
    axes[0, 0].legend()
    axes[-1, 0].set_xlabel("Simulation elapsed (s)")
    axes[-1, 1].set_xlabel("Simulation elapsed (s)")
    fig.tight_layout()
    fig.savefig(folder/"six_tracking.png", dpi=160)
    plt.close(fig)
    return metrics


def run_case(batch, tag, assets, mode, injection, crash, interrupted, env):
    case = dict(case=tag, controller=mode, injection=injection, status="running")
    case_folder = batch/tag
    case_folder.mkdir()
    guard_folder, tracking_folder = case_folder/"guard", case_folder/"tracking"
    a = b = None
    stream_a = stream_b = None
    try:
        guard_log = case_folder/"guard.txt"
        a, stream_a = spawn(["guard", "--folder", str(guard_folder), "--assets", str(assets)], guard_log, env)
        deadline = time.monotonic()+60
        print("  准备保护与碰撞进程，暂不运动……", flush=True)
        while "READY six_joint_guard" not in guard_log.read_text(encoding="utf-8", errors="replace"):
            if interrupted or a.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("six_guard_not_ready; read "+str(guard_log))
            time.sleep(0.05)
        arguments = ["tracking", "--folder", str(tracking_folder), "--assets", str(assets), "--controller", mode, "--inject", injection]
        if crash:
            arguments.append("--crash-test")
        b, stream_b = spawn(arguments, case_folder/"tracking.txt", env)
        if crash:
            deadline = time.monotonic()+20
            while True:
                if interrupted or a.poll() is not None or b.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("motion_not_observed_before_crash_deadline")
                progress = tracking_folder/"progress.json"
                if progress.exists():
                    state = load(progress)
                    if state["elapsed_sim_s"] >= 2 and max(abs(v) for v in state["last_request"]) > 0.003:
                        case["injection_observation"] = state
                        case["injection_runner_monotonic_s"] = time.monotonic()
                        b.kill()
                        b.wait(timeout=3)
                        break
                time.sleep(0.02)
            code_b = b.returncode
        else:
            code_b = wait(b, 110, interrupted, case_folder/"tracking.txt")
        code_a = wait(a, 10, interrupted)
        result_a, result_b = load(guard_folder/"run.json"), load(tracking_folder/"run.json")
        case.update(guard_directory=str(guard_folder), tracking_directory=str(tracking_folder),
                    guard_status=result_a["status"], guard_reason=result_a.get("reason"), tracking_status=result_b["status"],
                    stop_verified=result_a.get("stop_check", {}).get("verified", False),
                    actual_speed_max_rad_s=result_a.get("max_actual_speed_rad_s"))
        if not case["stop_verified"] or np.any(np.asarray(case["actual_speed_max_rad_s"]) > 0.02001):
            raise RuntimeError("stop_not_verified_or_sampled_actual_speed_violation")
        accepted = [r for r in rows(guard_folder/"request_log.csv") if r["accepted"] == "True"]
        if not accepted:
            raise RuntimeError("no_accepted_requests")
        if crash:
            allowed = ["request_source_sim_stamp_stale", "request_stream_lost_without_completion"]
            if code_b != -signal.SIGKILL or code_a != 1 or result_a.get("reason") not in allowed or result_a.get("motion_complete_received"):
                raise RuntimeError("crash_not_detected_as_expected")
            outputs = rows(guard_folder/"output_log.csv")
            stopping = [r for r in outputs if r["phase"] == "stopping"]
            case["last_request_to_first_stopping_zero_wall_s"] = float(stopping[0]["publish_wall_elapsed_s"])-float(accepted[-1]["wall_elapsed_s"])
            case["timing_note"] = "guard receive to publication wall time; not exact kill-to-physical-stop latency"
        else:
            if code_a or result_a["status"] != "completed":
                raise RuntimeError("guard_not_completed: "+str(result_a.get("reason"))+"; tracking: "+str(result_b.get("reason")))
            expected = injection != "none"
            if code_b != int(expected) or result_b["status"] != ("degraded" if expected else "completed"):
                raise RuntimeError("tracking_status_not_expected")
            if result_b.get("fallback_count") != int(expected) or result_b.get("uninjected_fallback_count") != 0:
                raise RuntimeError("unexpected_fallback_count")
            solver_rows = rows(tracking_folder/"solver_log.csv")
            if expected:
                injected = [i for i, r in enumerate(solver_rows) if r["injection"] != "none"]
                if len(injected) != 1 or solver_rows[injected[0]]["fallback"] != "True" or not any(r["fallback"] == "False" for r in solver_rows[injected[0]+1:]):
                    raise RuntimeError("single_injected_fallback_and_recovery_not_verified")
                case["injected_solver_row"] = solver_rows[injected[0]]
                row = case["injected_solver_row"]
                if injection == "failure" and "injected_solver_exception" not in row["status"]:
                    raise RuntimeError("injected_exception_not_observed")
                if injection == "timeout" and (float(row["solve_wall_s"]) <= C["solve_budget"] or "wall_time_limit_exceeded" not in row["status"]):
                    raise RuntimeError("injected_timeout_not_discarded")
            case["metrics"] = analyze(tracking_folder, guard_folder)
        if np.any(np.asarray(case["actual_speed_max_rad_s"]) < 0.002):
            raise RuntimeError("not_all_six_joints_observed_moving")
        case["status"] = "passed"
    except Exception as exc:
        case.update(status="failed", reason=str(exc))
        raise
    finally:
        try:
            stop(b)
        finally:
            try:
                stop(a)
            finally:
                for stream in [stream_a, stream_b]:
                    if stream is not None:
                        stream.close()
                save(case_folder/"case.json", case)
    return case


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["preflight", "compare", "faults", "all"], default="compare")
    parser.add_argument("--repeats", type=int, choices=[1, 3], default=1)
    args = parser.parse_args()
    root = Path.home()/"ROS2/arm/P2/data/data_v4"
    batch = root/(datetime.now().strftime("%Y%m%d_%H%M%S_%f")+"_six_joint_batch")
    batch.mkdir(parents=True)
    manifest = dict(status="running", mode=args.mode, repeats=args.repeats, cases=[], config=C,
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    note="six independent kinematic joint integrators, sampled collision checks; no guard crash watchdog")
    interrupted = []
    handlers = {s:signal.signal(s, lambda number, frame:interrupted.append(number)) for s in (signal.SIGINT, signal.SIGTERM)}
    env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
    env.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    process = stream = None
    print("六关节批次：", batch, flush=True)
    try:
        process, stream = spawn(["preflight", "--folder", str(batch/"assets")], batch/"preflight.txt", env)
        if wait(process, 70, interrupted):
            raise RuntimeError("preflight_failed; read "+str(batch/"preflight.txt"))
        print("模型、限位和采样碰撞预检查通过", flush=True)
        if args.mode in ["compare", "all"]:
            for trial in range(1, args.repeats+1):
                order = ["baseline", "mpc"] if trial%2 else ["mpc", "baseline"]
                for mode in order:
                    tag = str(trial)+"_"+mode
                    print("开始：", tag, flush=True)
                    case = run_case(batch, tag, batch/"assets", mode, "none", False, interrupted, env)
                    manifest["cases"].append(case)
                    save(batch/"run.json", manifest)
                    print("通过：", tag, "六关节静止确认", flush=True)
        if args.mode in ["faults", "all"]:
            for tag, injection, crash in [("solver_failure", "failure", False), ("solver_timeout", "timeout", False), ("request_crash", "none", True)]:
                print("开始：", tag, flush=True)
                case = run_case(batch, tag, batch/"assets", "mpc", injection, crash, interrupted, env)
                manifest["cases"].append(case)
                save(batch/"run.json", manifest)
                print("通过：", tag, flush=True)
        manifest.update(status="completed", reason=None)
    except Exception as exc:
        manifest.update(status="failed", reason=str(exc))
        print("批次停止：", exc, flush=True)
    finally:
        stop(process)
        if stream:
            stream.close()
        recorded = {c["case"] for c in manifest["cases"]}
        for path in sorted(batch.glob("*/case.json")):
            case = load(path)
            if case["case"] not in recorded:
                manifest["cases"].append(case)
        manifest["finished_at"] = datetime.now().astimezone().isoformat()
        save(batch/"run.json", manifest)
        for s, handler in handlers.items():
            signal.signal(s, handler)
        print("状态：", manifest["status"], "结果目录：", batch, flush=True)
    return 0 if manifest["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
