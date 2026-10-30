import argparse
import csv
import hashlib
import io
import json
import math
import statistics
import tarfile
from pathlib import Path


SMALL = "data/data_v4/20261031_020227_994815_six_joint_batch"
DEMO = "data/data_v5/20261031_022029_568481_six_joint_batch"
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def percentile(values, fraction):
    values = sorted(values)
    position = (len(values)-1)*fraction
    low, high = math.floor(position), math.ceil(position)
    return values[low]+(values[high]-values[low])*(position-low)


def main():
    parser = argparse.ArgumentParser(description="Offline audit of published P2 evidence; no ROS2 or motion")
    parser.add_argument("--archive", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    archive = args.archive or root/"evidence/p2_six_evidence_20261031.tar.gz"
    output = args.output or root/"results"
    output.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    cases = []
    measured = []
    source_checks = []
    with tarfile.open(archive, "r:gz") as bundle:
        def raw(name):
            stream = bundle.extractfile(name)
            if stream is None:
                raise RuntimeError("missing_archive_file: "+name)
            return stream.read()

        def obj(name):
            return json.loads(raw(name))

        def rows(name):
            return list(csv.DictReader(io.StringIO(raw(name).decode("utf-8"))))

        for batch, profile in [(SMALL, "small"), (DEMO, "demo")]:
            manifest = obj(batch+"/run.json")
            assert manifest["status"] == "completed"
            snapshot = root/"experiments"/("six_joint_small_before_demo_20261031_021856" if profile == "small" else "six_joint_demo_20261031")
            expected_core = hashlib.sha256((snapshot/"p2_six_core.py").read_bytes()).hexdigest()
            expected_control = hashlib.sha256((snapshot/"p2_six_control.py").read_bytes()).hexdigest()
            assert hashlib.sha256((snapshot/"run_p2_six_module.py").read_bytes()).hexdigest() == manifest["source_sha256"]
            for case in manifest["cases"]:
                name = case["case"]
                folder = batch+"/"+name
                guard = obj(folder+"/guard/run.json")
                tracking = obj(folder+"/tracking/run.json")
                assert case["status"] == "passed" and guard["stop_check"]["verified"]
                assert guard["core_sha256"] == expected_core and guard["source_sha256"] == expected_control
                assert tracking["core_sha256"] == expected_core and tracking["source_sha256"] == expected_control
                source_checks.append(dict(profile=profile, case=name, matching_core_and_control=True))
                feedback = rows(folder+"/guard/joint_log.csv")
                max_gap = {}
                for phase in ["waiting", "hold", "stopping"]:
                    max_gap[phase] = max((float(r["receive_gap_wall_s"] or 0) for r in feedback if r["phase"] == phase), default=0)
                actual_speed = max(abs(float(r[j+"_velocity_rad_s"])) for r in feedback for j in JOINTS)
                assert actual_speed <= 0.02001
                commands = rows(folder+"/guard/output_log.csv")
                assert all(abs(float(r[j+"_command_rad_s"])) <= 0.02001 for r in commands for j in JOINTS)
                result = dict(profile=profile, case=name, stop_verified=True, guard_reason=guard["reason"],
                              maximum_sampled_actual_speed_rad_s=actual_speed,
                              feedback_max_gap_wall_s_by_phase=max_gap,
                              minimum_self_clearance_m=guard["minimum_self_clearance_m"],
                              minimum_floor_clearance_m=guard["minimum_floor_clearance_m"])
                if name == "request_crash":
                    assert guard["status"] == "failed" and guard["reason"] in ["request_source_sim_stamp_stale", "request_stream_lost_without_completion"]
                    assert not guard["motion_complete_received"]
                    result["request_to_zero_wall_s"] = case["last_request_to_first_stopping_zero_wall_s"]
                    result["tracking_note"] = "SIGKILL bypasses finally; raw metadata remains running; batch case is authoritative for this injected exit"
                    cases.append(result)
                    continue
                assert guard["status"] == "completed" and tracking["completion_sent"]
                solver = rows(folder+"/tracking/solver_log.csv")
                injected = [i for i, r in enumerate(solver) if r["injection"] != "none"]
                fallbacks = sum(r["fallback"] == "True" for r in solver)
                if case["injection"] == "none":
                    assert tracking["status"] == "completed" and fallbacks == 0
                else:
                    assert tracking["status"] == "degraded" and fallbacks == 1 and len(injected) == 1
                    assert solver[injected[0]]["fallback"] == "True" and solver[injected[0]+1]["fallback"] == "False"
                    row = solver[injected[0]]
                    if case["injection"] == "timeout":
                        assert float(row["solve_wall_s"]) > .02 and "wall_time_limit_exceeded" in row["status"]
                    else:
                        assert "injected_solver_exception" in row["status"]
                    result["injected_row"] = row
                result["fallback_count"] = fallbacks
                result["solve_p95_wall_ms"] = percentile([float(r["solve_wall_s"])*1000 for r in solver], .95)
                controls = rows(folder+"/tracking/control_log.csv")
                times = [float(r["elapsed_sim_s"]) for r in controls]
                duration = times[-1]-times[0]
                assert duration >= tracking["config"]["duration"]+tracking["config"]["settle"]-.05
                assert all(0 < b-a <= .1 for a, b in zip(times, times[1:]))
                previous = [0.0]*6
                for row in controls:
                    dt = float(row["cycle_sim_s"])
                    command = [float(row[j+"_request_rad_s"]) for j in JOINTS]
                    assert all(abs(v) <= .02001 and abs(v-p) <= .1*dt+1e-5 for v, p in zip(command, previous))
                    previous = command
                stored = {r["joint"]:r for r in rows(folder+"/tracking/metrics.csv")}
                for joint in JOINTS:
                    error = [float(r[joint+"_position_rad"])-float(r[joint+"_reference_rad"]) for r in controls]
                    squared = sum((a*a+b*b)*.5*(v-u) for a,b,u,v in zip(error,error[1:],times,times[1:]))
                    iae = sum((abs(a)+abs(b))*.5*(v-u) for a,b,u,v in zip(error,error[1:],times,times[1:]))
                    rmse = math.sqrt(squared/duration)
                    assert math.isclose(rmse,float(stored[joint]["rmse_rad"]),rel_tol=1e-8,abs_tol=1e-12)
                    assert math.isclose(iae,float(stored[joint]["iae_rad_s"]),rel_tol=1e-8,abs_tol=1e-12)
                    measured.append(dict(profile=profile, case=name, controller=case["controller"], joint=joint,
                                         duration_sim_s=duration, rmse_rad=rmse, iae_rad_s=iae,
                                         max_abs_error_rad=max(abs(e) for e in error)))
                cases.append(result)
    comparison = []
    for joint in JOINTS:
        row = dict(joint=joint)
        for controller in ["baseline", "mpc"]:
            selected = [r for r in measured if r["profile"] == "small" and r["joint"] == joint
                        and r["case"] in [str(i)+"_"+controller for i in [1,2,3]]]
            assert len(selected) == 3
            for metric in ["rmse_rad", "iae_rad_s", "max_abs_error_rad"]:
                values = [r[metric] for r in selected]
                row[controller+"_"+metric+"_mean"] = statistics.mean(values)
                row[controller+"_"+metric+"_sample_sd"] = statistics.stdev(values)
        row["rmse_reduction_percent"] = 100*(1-row["mpc_rmse_rad_mean"]/row["baseline_rmse_rad_mean"])
        comparison.append(row)
    for name, records in [("joint_comparison_summary.csv", comparison), ("per_run_metrics.csv", measured)]:
        with (output/name).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    (output/"evidence_audit.json").write_text(json.dumps(dict(archive_sha256=digest, cases=cases,
        source_checks=source_checks, source_version_note="historical runs used dirty working tree; matching snapshots preserve exact recorded core/control/runner hashes; later Git commit is packaging, not the original execution commit",
        metric_scope="per-run sampled valid-window trapezoidal integrals; then n=3 mean and sample SD; no common-window resampling or significance claim"), indent=2), encoding="utf-8")
    print("Offline audit passed:", len(cases), "cases; 3 pairs x 6 joints; demo checked; source hashes match")
    print("Output:", output)


if __name__ == "__main__":
    main()
