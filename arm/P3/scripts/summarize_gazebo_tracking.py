import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import uuid

import numpy as np


def main():
    parser = argparse.ArgumentParser(description='Offline summary of completed full Gazebo P/MPC runs.')
    parser.add_argument('--p-dir', required=True, type=Path)
    parser.add_argument('--mpc-dir', required=True, type=Path)
    args = parser.parse_args()
    root = Path.home()/'ROS2/arm/P3'
    out = root/'data/runs'/(datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_gazebo_comparison_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    report = {'passed': False, 'motion_requested': False, 'runs': {},
              'limits': ['One full run per controller is a development comparison, not a statistical reliability study.',
                         'Errors use measured feedback at logged control times; sample spacing may differ between runs.',
                         'No claim of real hardware accuracy or continuous collision safety.']}
    try:
        originals = {}
        for mode, folder in [('p', args.p_dir), ('mpc', args.mpc_dir)]:
            r = json.loads((folder/'gazebo_tracking_report.json').read_text())
            assert r['passed'] and r['mode'] == mode and r['pilot_seconds'] == 0, 'Need a passed full run of '+mode
            originals[mode] = r
            source = json.loads((Path(r['reference_dir'])/'tracking_reference_report.json').read_text())
            duration = source['duration_sim_s']
            rows = list(csv.DictReader((folder/'tracking_log.csv').open()))
            errors = []; timing = []; elapsed = []
            names = source['joint_names']
            for row in rows:
                if float(row['elapsed_sim_s']) <= duration:
                    elapsed.append(float(row['elapsed_sim_s']))
                    errors.append([float(row['q_'+n])-float(row['ref_'+n]) for n in names])
                    timing.append(float(row['compute_wall_s']))
            e = np.asarray(errors); t = np.asarray(elapsed)
            assert len(e) > 2 and t[-1] >= duration-0.1, 'Incomplete tracking log'
            report['runs'][mode] = {'run_dir': str(folder.resolve()),
                                    'rmse_rad_at_logged_times': float(np.sqrt(np.mean(e**2))),
                                    'time_weighted_rmse_rad': float(np.sqrt(np.sum(np.trapz(e**2, t, axis=0))/(6*(t[-1]-t[0])))),
                                    'per_joint_rmse_rad': np.sqrt(np.mean(e**2, axis=0)).tolist(),
                                    'max_abs_error_rad': float(np.max(np.abs(e))),
                                    'endpoint_error_rad': r['endpoint_error_rad'],
                                    'compute_median_wall_s': float(np.median(timing)),
                                    'compute_p95_wall_s': float(np.percentile(timing, 95)),
                                    'solver_fallback_count': r['solver_fallback_count'],
                                    'deadline_fallback_count': r['deadline_fallback_count'],
                                    'stopped_confirmed': r['stopped_confirmed']}
        assert originals['p']['reference_sha256'] == originals['mpc']['reference_sha256'], 'References differ'
        assert originals['p']['config'] == originals['mpc']['config'], 'Controller configs differ'
        report['reference_sha256'] = originals['p']['reference_sha256']
        report['initial_state_difference_rad'] = (np.array(originals['p']['initial_q_rad'])-np.array(originals['mpc']['initial_q_rad'])).tolist()
        report['passed'] = True
    except Exception as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
    (out/'gazebo_comparison_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('SAVED:', out)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
