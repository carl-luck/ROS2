import argparse
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import uuid

import numpy as np

from prepare_detour_execution import coefficients, sample_segment


JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def evaluate_reference(times, segment_starts, segment_durations, coeff):
    times = np.asarray(times, dtype=float)
    end = segment_starts[-1]+segment_durations[-1]
    clipped = np.clip(times, 0, end)
    index = np.clip(np.searchsorted(segment_starts, clipped, side='right')-1, 0, len(segment_starts)-1)
    h = segment_durations[index]
    s = (clipped-segment_starts[index])/h
    c = coeff[index]
    q = sum(c[..., k, :]*s[..., None]**k for k in range(6))
    v = sum(k*c[..., k, :]*s[..., None]**(k-1) for k in range(1, 6))/h[..., None]
    a = sum(k*(k-1)*c[..., k, :]*s[..., None]**(k-2) for k in range(2, 6))/h[..., None]**2
    outside = (times < 0) | (times > end)
    v = np.where(outside[..., None], 0, v)
    a = np.where(outside[..., None], 0, a)
    return q, v, a


def main():
    parser = argparse.ArgumentParser(description='Offline common reference export; no ROS imports, commands, controller switching or motion.')
    parser.add_argument('--inputs-dir', required=True, type=Path)
    args = parser.parse_args()
    root = Path.home()/'ROS2/arm/P3'
    out = root/'data/runs'/(datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_tracking_reference_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    report = {'passed': False, 'robot_motion_requested': False, 'execution_ready': False,
              'input_dir': str(args.inputs_dir.resolve()), 'joint_names': JOINTS,
              'sample_period_sim_s': 0.02, 'velocity_limit_rad_s': 0.02,
              'command_rate_limit_rad_s2': 0.1, 'reference_limit_fraction': 0.9,
              'limits': ['Uses previously validated saved geometry; no live scene/current-state verification.',
                         'Uniform time scaling preserves the normalized quintic geometric curve.',
                         'Numerical polynomial extrema and finite differences are checks, not formal guarantees.',
                         'CSV is for inspection; both controllers must use the same polynomial evaluator.',
                         'P2 world check and small-excursion guard cannot be reused for this obstacle trajectory.',
                         'This export does not certify online feasibility, collision clearance under tracking error or MPC performance.']}
    try:
        for name in ('prepare_tracking_reference.py', 'prepare_detour_execution.py'):
            shutil.copyfile(Path(__file__).with_name(name), out/name)
        evidence = json.loads((args.inputs_dir/'execution_inputs_report.json').read_text())
        assert evidence['passed'] and evidence['paths']['detour']['passed'], 'Source validation failed'
        raw = (args.inputs_dir/'detour/trajectory_and_feedback.json').read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        assert digest == evidence['paths']['detour']['retimed_sha256'], 'Source trajectory hash mismatch'
        saved = json.loads(raw)['display_trajectory']['trajectory']
        assert len(saved) == 1
        trajectory = saved[0]['joint_trajectory']
        assert trajectory['joint_names'] == JOINTS, 'Unexpected joint order'
        points = trajectory['points']
        assert len(points) >= 2
        for point in points:
            for key in ('positions', 'velocities', 'accelerations'):
                assert len(point[key]) == 6 and np.all(np.isfinite(point[key]))
        assert points[0]['time_from_start'] == {'sec': 0, 'nanosec': 0}
        assert max(abs(x) for x in points[0]['velocities']+points[-1]['velocities']) < 1e-8, 'Endpoint speed not zero'
        starts, durations, coeff, samples = [], [], [], []
        for a, b in zip(points, points[1:]):
            t, h, c = coefficients(a, b)
            starts.append(t); durations.append(h); coeff.append(c)
            samples.extend(sample_segment(a, b, 0.01, 0.005))
        maximum_v = np.max(np.abs([s[2] for s in samples]), axis=0)
        maximum_a = np.max(np.abs([s[3] for s in samples]), axis=0)
        scale = max(1.0, float(maximum_v.max())/0.018, np.sqrt(float(maximum_a.max())/0.09))
        starts, durations, coeff = np.asarray(starts), np.asarray(durations), np.asarray(coeff)
        scaled_starts, scaled_durations = starts*scale, durations*scale
        end = float(scaled_starts[-1]+scaled_durations[-1])
        sample_times = np.r_[np.arange(0, end, 0.02), end]
        q, v, acc = evaluate_reference(sample_times, scaled_starts, scaled_durations, coeff)
        assert np.all(np.isfinite(q)) and np.all(np.isfinite(v)) and np.all(np.isfinite(acc))
        assert (maximum_v/scale <= 0.018+1e-9).all()
        assert (maximum_a/scale**2 <= 0.09+1e-9).all()
        assert np.max(np.abs(q[0]-points[0]['positions'])) < 1e-8
        assert np.max(np.abs(q[-1]-points[-1]['positions'])) < 1e-8
        # Independent numerical derivative check away from segment boundaries.
        mid = scaled_starts+0.5*scaled_durations
        eps = min(1e-4, float(scaled_durations.min())*1e-3)
        qm, vm, _ = evaluate_reference(mid-eps, scaled_starts, scaled_durations, coeff)
        qp, vp, _ = evaluate_reference(mid+eps, scaled_starts, scaled_durations, coeff)
        _, vc, ac = evaluate_reference(mid, scaled_starts, scaled_durations, coeff)
        velocity_error = float(np.max(np.abs((qp-qm)/(2*eps)-vc)))
        acceleration_error = float(np.max(np.abs((vp-vm)/(2*eps)-ac)))
        assert velocity_error < 1e-6 and acceleration_error < 1e-6
        continuity_error = 0.0
        for i in range(len(coeff)-1):
            left = coeff[i].sum(axis=0)
            continuity_error = max(continuity_error, float(np.max(np.abs(left-coeff[i+1, 0]))))
        assert continuity_error < 1e-8
        np.savez(out/'reference_polynomials.npz', segment_starts=scaled_starts,
                 segment_durations=scaled_durations, coefficients=coeff,
                 joint_names=np.asarray(JOINTS))
        with (out/'reference.csv').open('w', newline='') as stream:
            w = csv.writer(stream)
            w.writerow(['time_from_start_s']+['q_'+n for n in JOINTS]+['v_'+n for n in JOINTS]+['a_'+n for n in JOINTS])
            for i, t in enumerate(sample_times):
                w.writerow([t]+q[i].tolist()+v[i].tolist()+acc[i].tolist())
        (out/'source_trajectory_and_feedback.json').write_bytes(raw)
        for name in ('robot.urdf', 'robot.srdf'):
            shutil.copyfile(args.inputs_dir/'detour'/name, out/name)
        shutil.copyfile(args.inputs_dir/'execution_inputs_report.json', out/'source_execution_inputs_report.json')
        core = Path.home()/'ROS2/arm/P2/ros2_ws/src/ur5e_mpc_control/ur5e_mpc_control/p2_six_core.py'
        report.update(passed=True, source_sha256=digest, source_scene_signature=evidence['source_scene_signature'],
                      p2_core_sha256=hashlib.sha256(core.read_bytes()).hexdigest(),
                      source_duration_sim_s=float(starts[-1]+durations[-1]), time_scale_from_validated_input=scale,
                      duration_sim_s=end, samples=len(sample_times),
                      initial_q_rad=q[0].tolist(), final_q_rad=q[-1].tolist(),
                      per_joint_peak_to_peak_rad=np.ptp(q, axis=0).tolist(),
                      maximum_velocity_rad_s=(maximum_v/scale).tolist(),
                      maximum_acceleration_rad_s2=(maximum_a/scale**2).tolist(),
                      derivative_velocity_error=velocity_error, derivative_acceleration_error=acceleration_error,
                      position_continuity_error_rad=continuity_error,
                      artifacts_sha256={n: hashlib.sha256((out/n).read_bytes()).hexdigest()
                                        for n in ('reference.csv', 'reference_polynomials.npz', 'robot.urdf', 'robot.srdf', 'prepare_tracking_reference.py')})
    except Exception as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
    (out/'tracking_reference_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('SAVED:', out)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
