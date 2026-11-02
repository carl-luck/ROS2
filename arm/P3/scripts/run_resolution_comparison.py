import argparse
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import time
import uuid

import rclpy
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams


def main():
    parser = argparse.ArgumentParser(description='Plan-only comparison of OMPL checking resolution; temporarily updates planner config and restores it.')
    parser.add_argument('--case-dir', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    if not args.run:
        parser.error('Requires --run for planner parameter changes and planning requests')
    root = Path.home() / 'ROS2/arm/P3'
    out = root / 'data/runs' / (datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z') + '_resolution_comparison_' + uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    cfg = json.loads(args.config.read_text())
    for src in [Path(__file__), Path(__file__).with_name('plan_detour_case.py'), args.config, args.case_dir/'candidate.json']:
        shutil.copyfile(src, out/src.name)
    report = {'passed': False, 'execution_requested': False, 'parameter_restored': False,
              'design': '10 baseline followed by 10 tuned; unpaired stochastic development trials, no retries',
              'source_sha256': hashlib.sha256(Path(__file__).with_name('plan_detour_case.py').read_bytes()).hexdigest(),
              'limits': ['Planner parameter readback does not prove internal interpolation density.',
                         'External linear sampling is unchanged; no controller spline or continuous collision guarantee.',
                         'Sequential groups may have order effects; 10 trials per group are exploratory.']}
    rows = []
    original = None
    changed = False
    rclpy.init()
    node = rclpy.create_node('p3_resolution_comparison', enable_rosout=False)

    def call(cls, endpoint, req):
        client = node.create_client(cls, endpoint)
        try:
            if not client.wait_for_service(timeout_sec=4):
                raise RuntimeError('Service unavailable: '+endpoint)
            f = client.call_async(req)
            end = time.monotonic()+8
            while not f.done() and time.monotonic()<end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not f.done():
                raise RuntimeError('Service timeout; parameter state may be unknown')
            return f.result()
        finally:
            node.destroy_client(client)

    def request(cls):
        req = cls.Request()
        req.pipeline_id = cfg['pipeline_id']
        req.group = cfg['group']
        req.planner_config = cfg['planner_id']
        return req

    def read():
        p = call(GetPlannerParams, '/get_planner_params', request(GetPlannerParams)).params
        return dict(zip(p.keys, p.values))

    def write(params):
        req = request(SetPlannerParams)
        req.replace = True
        req.params.keys = list(params)
        req.params.values = list(params.values())
        call(SetPlannerParams, '/set_planner_params', req)
        if read() != params:
            raise RuntimeError('Planner parameter readback differs')

    try:
        original = read()
        (out/'original_planner_params.json').write_text(json.dumps(original, indent=2))
        key = 'longest_valid_segment_fraction'
        if key not in original or abs(float(original[key])-0.01)>1e-10:
            raise RuntimeError('Expected baseline fraction 0.01; stop for review, no parameter changed')
        for label, fraction in [('baseline', '0.01'), ('dense', '0.001')]:
            params = dict(original)
            if label == 'dense':
                params[key] = fraction
                changed = True
                write(params)
            (out/(label+'_planner_params.json')).write_text(json.dumps(read(), indent=2))
            for i in range(1, 11):
                if read() != params:
                    raise RuntimeError('Planner configuration changed during batch')
                print(f'{label} {i}/10: planning and validation only, no motion', flush=True)
                log = out/f'{label}_{i:02}.log'
                with log.open('w') as stream:
                    proc = subprocess.run([sys.executable, str(Path(__file__).with_name('plan_detour_case.py')),
                                           '--case-dir', str(args.case_dir), '--config', str(args.config)],
                                          stdout=stream, stderr=subprocess.STDOUT)
                saved = [line[7:].strip() for line in log.read_text().splitlines() if line.startswith('SAVED: ')]
                if not saved:
                    raise RuntimeError('Missing trial report; inspect '+str(log))
                r = json.loads((Path(saved[-1])/'detour_plan_report.json').read_text())
                d = r.get('paths', {}).get('detour', {})
                row = {'condition': label, 'trial': i, 'run_dir': saved[-1], 'exit_code': proc.returncode,
                       'scene_signature': r.get('scene_signature'), 'validated': d.get('passed') is True,
                       'error_code': d.get('error_code'), 'planning_time_s': d.get('interface_planning_time_s'),
                       'path_length_rad': d.get('path_length_rad'), 'duration_s': d.get('planned_duration_s'),
                       'invalid_samples': d.get('invalid_sample_count'), 'error': r.get('error', '')}
                rows.append(row)
                with (out/'trials.csv').open('w', newline='') as stream:
                    w = csv.DictWriter(stream, fieldnames=list(row)); w.writeheader(); w.writerows(rows)
                if not row['scene_signature'] or row['scene_signature'] != rows[0]['scene_signature']:
                    raise RuntimeError('Missing or changed scene signature; batch stopped')
                print(f"  error_code={row['error_code']}, validated={row['validated']}", flush=True)
        report['conditions'] = {}
        for label in ('baseline', 'dense'):
            group = [r for r in rows if r['condition']==label]
            valid = [r for r in group if r['validated']]
            metrics = {'trials': len(group), 'validated_count': len(valid), 'validated_rate': len(valid)/len(group),
                       'planner_success_count': sum(r['error_code']==1 for r in group)}
            for key in ('planning_time_s', 'path_length_rad', 'duration_s'):
                values = [r[key] for r in valid if r[key] is not None]
                if values:
                    metrics[key+'_validated_only'] = {'median': statistics.median(values), 'min': min(values), 'max': max(values)}
            report['conditions'][label] = metrics
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
    finally:
        if original is not None:
            try:
                if changed:
                    write(original)
                report['parameter_restored'] = read()==original
            except (Exception, KeyboardInterrupt) as exc:
                report['restore_error'] = str(exc)
        report['passed'] = report['passed'] and report['parameter_restored']
        report['completed_trials'] = len(rows)
        (out/'comparison_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print('SAVED:', out)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
