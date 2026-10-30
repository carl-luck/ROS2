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
            raise RuntimeError("interrupted_or_child_timeout")
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
    folders = [p for p in root.iterdir() if p.is_dir() and p.name not in before and (p/"run.json").exists()]
    if len(folders) != 1:
        raise RuntimeError("cannot_identify_unique_child_run")
    return folders[0], json.loads((folders[0]/"run.json").read_text(encoding="utf-8"))


def read_rows(path):
    if not path.exists():
        return []
    text = path.read_text(encoding="utf-8")
    if not text.endswith("\n"):
        text = text[:text.rfind("\n")+1]
    return list(csv.DictReader(text.splitlines()))


def validate_guard(folder, result):
    if not result.get("stop_check", {}).get("verified"):
        raise RuntimeError("guard_stop_not_verified")
    if result.get("max_actual_speed_rad_s", float("inf")) > 0.02001:
        raise RuntimeError("sampled_actual_speed_limit_exceeded")
    requests = read_rows(folder/"request_log.csv")
    accepted = [r for r in requests if r["accepted"] == "True"]
    if not any(abs(float(r["shoulder_pan_joint_request_rad_s"])) >= 0.003 for r in accepted):
        raise RuntimeError("guard_did_not_accept_motion")
    return accepted


def crash_during_motion(worker, guard, data_root, before, interrupted):
    deadline = time.monotonic()+20
    while time.monotonic() < deadline:
        if interrupted or worker.poll() is not None or guard.poll() is not None:
            raise RuntimeError("child_exited_before_crash_injection")
        folders = [p for p in data_root.iterdir() if p.name not in before and (p/"run.json").exists()]
        if len(folders) == 1:
            rows = read_rows(folders[0]/"request_log.csv")
            if len(rows) >= 2:
                first_stamp = float(rows[0]["source_sim_stamp_s"])
                last = rows[-1]
                if (float(last["source_sim_stamp_s"])-first_stamp >= 2.0
                        and abs(float(last["requested_first_joint_rad_s"])) >= 0.003
                        and last["completion"] == "False"):
                    before_kill = time.monotonic()
                    worker.kill()
                    worker.wait(timeout=3)
                    return dict(method="request_process_SIGKILL_only", last_request=last,
                                child_returncode=worker.returncode,
                                runner_monotonic_before_kill_s=before_kill,
                                note="guard is kept alive; last logged request is not an exact kill timestamp")
        time.sleep(0.02)
    raise RuntimeError("no_live_motion_before_crash_deadline")


def validate_fallback(folder, result, code):
    rows = read_rows(folder/"solver_log.csv")
    injected = [i for i, r in enumerate(rows) if r["injection"] != "none"]
    if (code != 1 or result.get("status") != "degraded" or not result.get("completion_sent")
            or result.get("fallback_count") != 1 or len(injected) != 1
            or result.get("injected_fallback_count") != 1 or result.get("uninjected_fallback_count") != 0):
        raise RuntimeError("unexpected_fallback_result")
    index = injected[0]
    row = rows[index]
    if row["fallback"] != "True" or not any(r["fallback"] == "False" for r in rows[index+1:]):
        raise RuntimeError("fallback_or_solver_recovery_not_observed")
    if row["injection"] == "failure" and "injected_solver_exception" not in row["status"]:
        raise RuntimeError("missing_injected_exception")
    if row["injection"] == "timeout" and (float(row["solve_wall_s"]) <= 0.02 or "wall_time_limit_exceeded" not in row["status"]):
        raise RuntimeError("timeout_not_discarded")
    controls = read_rows(folder/"control_log.csv")
    previous = 0.0
    for control in controls:
        command = float(control["requested_rad_s"])
        allowed_delta = 0.1*float(control["cycle_sim_s"])+1e-5
        if abs(command) > 0.02001 or abs(command-previous) > allowed_delta:
            raise RuntimeError("request_speed_or_slew_limit_exceeded")
        previous = command
    return row


def stop_observation(folder, accepted):
    last_request_wall = float(accepted[-1]["wall_elapsed_s"])
    commands = read_rows(folder/"command_log.csv")
    zeros = [r for r in commands if r["phase"] == "stopping"
             and all(float(value) == 0.0 for key, value in r.items() if key.endswith("_command_rad_s"))]
    if not zeros:
        raise RuntimeError("no_stopping_zero_in_guard_log")
    first_zero = float(zeros[0]["wall_elapsed_s"])
    return dict(last_accepted_request_wall_s=last_request_wall,
                first_stopping_zero_wall_s=first_zero,
                last_accepted_request_to_zero_wall_s=first_zero-last_request_wall,
                note="guard-relative receive/publication wall times; not exact SIGKILL-to-physical-stop latency")


def main():
    if os.name != "posix":
        raise SystemExit("Run in the Ubuntu ROS2 terminal, not Windows")
    root = Path.home()/"ROS2/arm/P2"
    first = root/"data/data_v1"
    third = root/"data/data_v3"
    first.mkdir(parents=True, exist_ok=True)
    third.mkdir(parents=True, exist_ok=True)
    batch = third/(datetime.now().strftime("%Y%m%d_%H%M%S_%f")+"_fault_batch")
    batch.mkdir(exist_ok=False)
    manifest = dict(run_id=batch.name, status="running", reason=None, cases=[],
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    note="controlled fault tests; expected guard failure or tracking degraded must satisfy case-specific acceptance; no guard crash watchdog")
    (batch/"config.json").write_text(json.dumps(dict(cases=["solver_failure", "solver_timeout", "request_crash"],
        motion="normal scenario, 0.01rad scale, 8s period, speed limit 0.02rad/s",
        injection_at_sim_s=2.0, artificial_delay_wall_s=0.025,
        stop_policy="halt batch on unexpected result; keep guard alive until stop verification"), indent=2), encoding="utf-8")
    interrupted = []
    handlers = {s: signal.signal(s, lambda number, frame: interrupted.append(number)) for s in (signal.SIGINT, signal.SIGTERM)}
    env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
    print("异常测试批次：", batch, flush=True)
    try:
        for name, inject in [("solver_failure", "failure"), ("solver_timeout", "timeout"), ("request_crash", "none")]:
            print("开始：", name, flush=True)
            before_a = {p.name for p in first.iterdir()}
            before_b = {p.name for p in third.iterdir()}
            guard = worker = None
            case = dict(case=name, status="running", injection=inject)
            manifest["cases"].append(case)
            guard_log = batch/(name+"_guard.txt")
            try:
                with guard_log.open("w", encoding="utf-8") as out_a, (batch/(name+"_tracking.txt")).open("w", encoding="utf-8") as out_b:
                    guard = subprocess.Popen([sys.executable, "-m", "ur5e_mpc_control.joint_interface_test", "guard"], stdout=out_a, stderr=subprocess.STDOUT, env=env)
                    deadline = time.monotonic()+15
                    while "READY" not in guard_log.read_text(encoding="utf-8", errors="replace"):
                        if interrupted or guard.poll() is not None or time.monotonic()>deadline:
                            raise RuntimeError("guard_not_ready; read "+str(guard_log))
                        time.sleep(0.05)
                    worker = subprocess.Popen([sys.executable, "-m", "ur5e_mpc_control.p2_fault_tracking", "--inject", inject], stdout=out_b, stderr=subprocess.STDOUT, env=env)
                    if name == "request_crash":
                        case["injection_observation"] = crash_during_motion(worker, guard, third, before_b, interrupted)
                        code_b = worker.returncode
                    else:
                        code_b = wait_process(worker, 85, interrupted)
                    code_a = wait_process(guard, 8, interrupted)
                folder_a, result_a = new_run(first, before_a)
                folder_b, result_b = new_run(third, before_b)
                case.update(guard_directory=str(folder_a), tracking_directory=str(folder_b),
                            guard_status=result_a["status"], guard_reason=result_a.get("reason"),
                            tracking_status=result_b["status"], stop_verified=result_a.get("stop_check", {}).get("verified", False),
                            guard_returncode=code_a, tracking_returncode=code_b)
                accepted = validate_guard(folder_a, result_a)
                if name == "request_crash":
                    allowed = ["request_source_sim_stamp_stale", "request_stream_lost_without_completion"]
                    if code_b != -signal.SIGKILL or code_a != 1 or result_a["status"] != "failed" or result_a.get("reason") not in allowed or result_a.get("motion_complete_received"):
                        raise RuntimeError("crash_not_detected_as_expected")
                    case["last_accepted_request"] = accepted[-1]
                    case["stop_observation"] = stop_observation(folder_a, accepted)
                    case["tracking_metadata_note"] = "SIGKILL bypasses finally; raw tracking run.json may remain running and is deliberately preserved"
                else:
                    if code_a != 0 or result_a["status"] != "completed":
                        raise RuntimeError("guard_did_not_complete_after_fallback")
                    case["injected_solver_row"] = validate_fallback(folder_b, result_b, code_b)
                case["status"] = "passed"
                print("通过：", name, "静止确认：", case["stop_verified"], flush=True)
            except Exception as exc:
                case.update(status="failed", reason=str(exc))
                raise
            finally:
                try:
                    stop_process(worker)
                finally:
                    try:
                        stop_process(guard)
                    finally:
                        (batch/"run.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        manifest.update(status="completed", reason=None)
    except Exception as exc:
        manifest.update(status="failed", reason=str(exc))
        print("批次停止：", exc, flush=True)
    finally:
        manifest["finished_at"] = datetime.now().astimezone().isoformat()
        (batch/"run.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        for number, handler in handlers.items():
            signal.signal(number, handler)
        print("状态：", manifest["status"], "结果目录：", batch, flush=True)
    return 0 if manifest["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
