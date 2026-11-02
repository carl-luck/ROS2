import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys
import uuid


def main():
    parser = argparse.ArgumentParser(description='Manually invoked preparation workflow. Plans from current position, validates and executes preparation only.')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        parser.error('Requires --execute; this command moves the robot to the fixed tracking start')
    root = Path.home()/'ROS2/arm/P3'
    out = root/'data/runs'/(datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_tracking_start_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    shutil.copyfile(Path(__file__), out/Path(__file__).name)
    report = {'passed': False, 'motion_authorized': True, 'planning_trials': [],
              'limits': ['Bounded planning retries are development preparation, not a planning success-rate benchmark.',
                         'No controller switching, automatic execution retry or execution after failed validation.']}

    def run(name, script, arguments):
        print('STAGE '+name, flush=True)
        command = [sys.executable, str(Path(__file__).with_name(script)), *arguments]
        proc = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (out/(name+'.log')).write_text(proc.stdout)
        print(proc.stdout[-4000:], flush=True)
        saved = [line[7:].strip() for line in proc.stdout.splitlines() if line.startswith('SAVED: ')]
        return proc.returncode, Path(saved[-1]) if saved else None

    try:
        plan = None
        for i in range(1, 4):
            code, candidate = run('plan_'+str(i), 'plan_detour_case.py',
                                  ['--case-dir', str(root/'data/runs/20261101_224558+0800_detour_search_wELTEptP'),
                                   '--config', str(root/'config/detour_plan_v1.json')])
            report['planning_trials'].append({'exit_code': code, 'run_dir': str(candidate)})
            if code == 0 and candidate and json.loads((candidate/'detour_plan_report.json').read_text())['passed']:
                plan = candidate
                break
        if plan is None:
            raise RuntimeError('No passed plan in 3 attempts; no motion sent; retain all evidence')
        code, inputs = run('retime_check', 'prepare_detour_execution.py',
                           ['--plan-dir', str(plan), '--config', str(root/'config/detour_retime_v1.json')])
        if code or inputs is None or not json.loads((inputs/'execution_inputs_report.json').read_text())['passed']:
            raise RuntimeError('Retiming/validation failed; no motion sent')
        code, execution = run('preparation', 'execute_tracking_preparation.py',
                              ['--inputs-dir', str(inputs), '--config', str(root/'config/detour_execute_v1.json'), '--execute'])
        report['execution_dir'] = str(execution)
        if code or execution is None or not json.loads((execution/'sequence_report.json').read_text())['passed']:
            raise RuntimeError('Preparation did not pass; no automatic retry; inspect child report')
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
    (out/'tracking_start_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('SAVED:', out)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
