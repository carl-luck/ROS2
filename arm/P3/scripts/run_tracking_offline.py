import argparse
from collections import deque
import csv
from datetime import datetime
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
import sys
import uuid
import xml.etree.ElementTree as ET

import numpy as np

from prepare_tracking_reference import evaluate_reference, JOINTS
from tracking_control_core import Reference, TrackingController, plant_step


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description='Offline P/MPC comparison, no ROS or robot commands.')
    parser.add_argument('--reference-dir', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    args = parser.parse_args()
    root = Path.home()/'ROS2/arm/P3'
    out = root/'data/runs'/(datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_tracking_offline_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    report = {'passed': False, 'robot_motion_requested': False, 'execution_ready': False,
              'reference_dir': str(args.reference_dir.resolve()), 'runs': {},
              'limits': ['Offline synthetic plants only, not Gazebo or real hardware measurements.',
                         'Both controllers share reference, limits and plant; MPC predicts an ideal integrator even in lag_delay.',
                         'No obstacle constraints in MPC; no actual-state collision checks in offline plants.',
                         'Wall timings exclude ROS and supervision and do not prove real-time capability.',
                         'Compute budget overruns are recorded; this offline runner does not enforce a real-time scheduling deadline.',
                         'Passing checks does not require MPC to outperform P; fallback counts remain visible.']}
    try:
        cfg = json.loads(args.config.read_text())
        assert cfg['ts'] == 0.02 and cfg['horizon'] == 25
        assert len({s['name'] for s in cfg['scenarios']}) == len(cfg['scenarios'])
        assert cfg['vmax'] > 0 and cfg['rate'] > 0
        source = json.loads((args.reference_dir/'tracking_reference_report.json').read_text())
        assert source['passed'], 'Reference export did not pass'
        for name, expected in source['artifacts_sha256'].items():
            assert digest(args.reference_dir/name) == expected, 'Reference hash mismatch: '+name
        assert digest(Path(__file__).with_name('prepare_tracking_reference.py')) == source['artifacts_sha256']['prepare_tracking_reference.py'], 'Reference evaluator changed'
        report['reference_sha256'] = source['artifacts_sha256']['reference_polynomials.npz']
        report['config'] = cfg
        report['python'] = sys.version
        report['versions'] = {n: importlib.metadata.version(n) for n in ('numpy', 'scipy', 'osqp')}
        for name in ('run_tracking_offline.py', 'tracking_control_core.py', 'prepare_tracking_reference.py', 'prepare_detour_execution.py'):
            shutil.copyfile(Path(__file__).with_name(name), out/name)
        shutil.copyfile(args.config, out/'config.json')
        shutil.copyfile(args.reference_dir/'tracking_reference_report.json', out/'source_reference_report.json')
        ref = Reference(args.reference_dir/'reference_polynomials.npz', evaluate_reference)
        assert ref.names == JOINTS
        model = ET.fromstring((args.reference_dir/'robot.urdf').read_text())
        joints = {j.attrib['name']: j for j in model.findall('joint')}
        lower, upper = [], []
        for name in JOINTS:
            j = joints[name]; limit = j.find('limit'); safety = j.find('safety_controller')
            lo, hi = float(limit.attrib['lower']), float(limit.attrib['upper'])
            if safety is not None:
                lo = max(lo, float(safety.attrib.get('soft_lower_limit', lo)))
                hi = min(hi, float(safety.attrib.get('soft_upper_limit', hi)))
            lower.append(lo+cfg['joint_margin']); upper.append(hi-cfg['joint_margin'])
        lower, upper = np.array(lower), np.array(upper)
        dt = cfg['ts']
        steps = int(np.ceil((ref.duration+cfg['settle_s'])/dt))
        grid = np.arange(steps+1)*dt
        reference_q, _, _ = ref.at(grid)
        assert np.all(reference_q >= lower) and np.all(reference_q <= upper), 'Reference exceeds joint bounds'
        report['duration_reference_sim_s'] = ref.duration
        report['sample_count_per_run'] = len(grid)
        for scenario in cfg['scenarios']:
            delay, tau = scenario['delay_steps'], scenario['velocity_tau_s']
            assert isinstance(delay, int) and delay >= 0 and tau >= 0
            for mode in ('p', 'mpc'):
                label = scenario['name']+'_'+mode
                print('RUN '+label+': offline only', flush=True)
                control = TrackingController(mode, cfg, lower, upper, ref)
                q = np.asarray(source['initial_q_rad'], dtype=float)
                velocity, previous = np.zeros(6), np.zeros(6)
                queue = deque([np.zeros(6) for _ in range(delay)])
                errors, tracking_errors, computations = [], [], []
                max_u = max_rate = max_actual_v = 0.0
                fallback_count = misses = limited = bound_violations = 0
                statuses = {}
                log_path = out/(label+'.csv')
                with log_path.open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(['t_sim_s']+['ref_'+n for n in JOINTS]+['q_'+n for n in JOINTS]+
                                    ['v_actual_'+n for n in JOINTS]+['u_'+n for n in JOINTS]+
                                    ['compute_wall_s', 'status', 'fallback', 'iterations', 'limiter_correction_rad_s'])
                    for k, t in enumerate(grid[:-1]):
                        error = q-reference_q[k]
                        errors.append(error)
                        if t <= ref.duration:
                            tracking_errors.append(error)
                        u, info = control.command(q, previous, float(t))
                        assert np.all(np.isfinite(u)) and np.all(np.isfinite(q))
                        max_u = max(max_u, float(np.max(np.abs(u))))
                        max_rate = max(max_rate, float(np.max(np.abs(u-previous)))/dt)
                        max_actual_v = max(max_actual_v, float(np.max(np.abs(velocity))))
                        fallback_count += info['fallback']
                        misses += info['compute_wall_s'] > cfg['solver_time_limit_s']
                        limited += info['limiter_correction_rad_s'] > 1e-8
                        statuses[info['status']] = statuses.get(info['status'], 0)+1
                        computations.append(info['compute_wall_s'])
                        writer.writerow([t]+reference_q[k].tolist()+q.tolist()+velocity.tolist()+u.tolist()+
                                        [info[n] for n in ('compute_wall_s', 'status', 'fallback', 'iterations', 'limiter_correction_rad_s')])
                        queue.append(u.copy()); applied = queue.popleft()
                        q, velocity = plant_step(q, velocity, applied, dt, tau)
                        previous = u
                        bound_violations += bool(np.any(q < lower) or np.any(q > upper))
                    errors.append(q-reference_q[-1])
                error = np.asarray(errors)
                tracked = np.asarray(tracking_errors)
                timing = np.asarray(computations)
                checks = {'finite_states': bool(np.all(np.isfinite(error))),
                          'command_velocity_limit': max_u <= cfg['vmax']+1e-9,
                          'command_rate_limit': max_rate <= cfg['rate']+1e-9,
                          'joint_position_limits': bound_violations == 0}
                metrics = {'passed': all(checks.values()), 'checks': checks,
                           'mode': mode, 'scenario': scenario,
                           'tracking_rmse_rad': float(np.sqrt(np.mean(tracked**2))),
                           'full_run_rmse_rad': float(np.sqrt(np.mean(error**2))),
                           'per_joint_tracking_rmse_rad': np.sqrt(np.mean(tracked**2, axis=0)).tolist(),
                           'max_abs_error_rad': float(np.max(np.abs(error))),
                           'endpoint_after_settle_error_rad': (q-reference_q[-1]).tolist(),
                           'max_command_rad_s': max_u, 'max_command_rate_rad_s2': max_rate,
                           'max_observed_actual_velocity_rad_s': max_actual_v,
                           'fallback_count': int(fallback_count), 'compute_budget_overrun_count': int(misses),
                           'limited_command_count': int(limited), 'statuses': statuses,
                           'compute_median_wall_s': float(np.median(timing)),
                           'compute_p95_wall_s': float(np.percentile(timing, 95)),
                           'compute_max_wall_s': float(np.max(timing)), 'log_sha256': digest(log_path)}
                report['runs'][label] = metrics
                (out/'offline_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
                print(label+' RMSE(rad)='+str(round(metrics['tracking_rmse_rad'], 8))+' fallback='+str(fallback_count), flush=True)
        report['passed'] = all(r['passed'] for r in report['runs'].values()) and len(report['runs']) == 4
        with (out/'metrics.csv').open('w', newline='') as stream:
            fields = ['run', 'passed', 'tracking_rmse_rad', 'max_abs_error_rad', 'compute_median_wall_s',
                      'compute_p95_wall_s', 'compute_max_wall_s', 'fallback_count', 'compute_budget_overrun_count', 'limited_command_count']
            w = csv.DictWriter(stream, fieldnames=fields); w.writeheader()
            for label, metrics in report['runs'].items():
                w.writerow({'run': label, **{k: metrics[k] for k in fields if k != 'run'}})
    except Exception as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
    report['finished_at'] = datetime.now().astimezone().isoformat()
    (out/'offline_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('SAVED:', out)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
