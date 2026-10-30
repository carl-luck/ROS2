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


def wait_process(process, seconds, interrupted):
    deadline = time.monotonic()+seconds
    while process.poll() is None:
        if interrupted or time.monotonic() > deadline:
            raise RuntimeError("batch_interrupted_or_child_timeout")
        time.sleep(0.05)
    return process.returncode


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        raise RuntimeError("child_did_not_exit; stop_simulation_in_launch_terminal")


def new_run(root, before):
    candidates = [p for p in root.iterdir() if p.is_dir() and p.name not in before and (p/"run.json").exists()]
    if len(candidates) != 1:
        raise RuntimeError("cannot_identify_unique_child_run")
    folder = candidates[0]
    return folder, json.loads((folder/"run.json").read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, choices=[1, 3], default=1)
    args = parser.parse_args()
    root = Path.home()/"ROS2/arm/P2"
    batch = root/"data/data_v2"/(datetime.now().strftime("%Y%m%d_%H%M%S_%f")+"_comparison_batch")
    batch.mkdir(parents=True, exist_ok=False)
    first = root/"data/data_v1"
    second = root/"data/data_v2"
    first.mkdir(parents=True, exist_ok=True)
    config = dict(repeats=args.repeats, scenarios=["normal", "near_limit", "limited"],
                  controllers=["baseline", "mpc"], prediction_steps=25, control_steps=25,
                  velocity_limit_rad_s=0.02, command_rate_limit_rad_s2=0.1,
                  reference_scale_rad=0.01, periods_sim_s=[8.0, 3.5, 2.5],
                  initial_state_note="relative references from current state; no automatic reset; inspect initial positions before final comparison",
                  guard_note="existing independent guard; does not survive its own process crash",
                  failure_policy="stop batch on any failed or degraded tracking or unverified guard stop")
    (batch/"config.yaml").write_text(json.dumps(config, indent=2), encoding="utf-8")
    manifest = dict(run_id=batch.name, status="running", reason=None,
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), cases=[])
    interrupted = []
    handlers = {s: signal.signal(s, lambda number, frame: interrupted.append(number)) for s in (signal.SIGINT, signal.SIGTERM)}
    env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
    summaries = []
    print("批次目录：", batch, flush=True)
    try:
        for trial in range(1, args.repeats+1):
            for scenario in config["scenarios"]:
                order = config["controllers"] if trial % 2 else list(reversed(config["controllers"]))
                for controller in order:
                    print("开始：", trial, scenario, controller, flush=True)
                    tag = str(trial)+"_"+scenario+"_"+controller
                    before_a = {p.name for p in first.iterdir()}
                    before_b = {p.name for p in second.iterdir()}
                    guard = worker = None
                    guard_log = batch/(tag+"_guard.txt")
                    try:
                        with guard_log.open("w", encoding="utf-8") as output_a, (batch/(tag+"_tracking.txt")).open("w", encoding="utf-8") as output_b:
                            guard = subprocess.Popen([sys.executable, "-m", "ur5e_mpc_control.joint_interface_test", "guard"], stdout=output_a, stderr=subprocess.STDOUT, env=env)
                            deadline = time.monotonic()+15
                            while "READY" not in guard_log.read_text(encoding="utf-8", errors="replace"):
                                if interrupted or guard.poll() is not None or time.monotonic()>deadline:
                                    raise RuntimeError("guard_not_ready; read "+str(guard_log))
                                time.sleep(0.05)
                            worker = subprocess.Popen([sys.executable, "-m", "ur5e_mpc_control.p2_tracking", "--controller", controller, "--scenario", scenario], stdout=output_b, stderr=subprocess.STDOUT, env=env)
                            code_b = wait_process(worker, 85, interrupted)
                            code_a = wait_process(guard, 8, interrupted)
                        folder_a, result_a = new_run(first, before_a)
                        folder_b, result_b = new_run(second, before_b)
                        case = dict(trial=trial, scenario=scenario, controller=controller,
                                    guard_directory=str(folder_a), tracking_directory=str(folder_b),
                                    guard_status=result_a["status"], tracking_status=result_b["status"],
                                    stop_verified=result_a.get("stop_check", {}).get("verified", False))
                        manifest["cases"].append(case)
                        if code_a or code_b or not case["stop_verified"] or case["tracking_status"] != "completed":
                            raise RuntimeError("case_failed_or_degraded; inspect child run.json and terminal logs")
                        with (folder_b/"metrics.csv").open(newline="", encoding="utf-8") as stream:
                            metric = next(csv.DictReader(stream))
                        summaries.append(dict(trial=trial, scenario=scenario, **metric,
                                              stop_verified=case["stop_verified"], tracking_directory=str(folder_b), guard_directory=str(folder_a)))
                        print("完成：", tag, "RMSE", metric["time_weighted_rmse_rad"], flush=True)
                    finally:
                        try:
                            stop_process(worker)
                        finally:
                            stop_process(guard)
                        (batch/"run.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        manifest.update(status="completed", reason=None)
    except Exception as exc:
        manifest.update(status="failed", reason=str(exc))
        print("本批已停止：", exc, flush=True)
    finally:
        if summaries:
            with (batch/"summary.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
                writer.writeheader()
                writer.writerows(summaries)
        (batch/"run.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        for s, handler in handlers.items():
            signal.signal(s, handler)
        print("状态：", manifest["status"], "结果目录：", batch, flush=True)
    return 0 if manifest["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
