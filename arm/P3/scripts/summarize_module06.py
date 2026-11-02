import argparse
from datetime import datetime
import json
from pathlib import Path
import uuid


def main():
    parser=argparse.ArgumentParser(description='Offline summary of module 6; never sends ROS requests.')
    parser.add_argument('--failure-dir',required=True,type=Path)
    parser.add_argument('--cancel-dir',required=True,type=Path)
    args=parser.parse_args()
    failure=json.loads((args.failure_dir/'failure_report.json').read_text())
    cancel=json.loads((args.cancel_dir/'sequence_report.json').read_text())
    out=Path.home()/'ROS2/arm/P3/data/runs'/(datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_module06_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True,exist_ok=False)
    checks={
        'negative_tests_passed':failure.get('passed') is True,
        'negative_tests_no_execution':failure.get('execution_requests_sent')==0 and failure.get('action_client_created') is False,
        'real_cancel_triggered':cancel.get('scheduled_cancel_triggered') is True,
        'cancel_acknowledged':cancel.get('cancel_acknowledged') is True,
        'terminal_canceled':cancel.get('final_action_status')==5,
        'stopped_from_feedback':cancel.get('stopped_after_failure') is True,
        'post_cancel_state_valid':cancel.get('post_cancel_state_valid') is True,
        'scene_unchanged':cancel.get('post_cancel_scene_unchanged') is True,
        'no_detour_sent':set(cancel.get('phases',{}))=={'preparation'},
        'cancel_report_passed':cancel.get('passed') is True}
    report={'passed':all(checks.values()),'checks':checks,'failure_dir':str(args.failure_dir.resolve()),
            'cancel_dir':str(args.cancel_dir.resolve()),'next_motion_allowed':False,
            'limits':['Action cancellation in simulation is not an emergency-stop guarantee.',
                      'Synthetic fault injection is separate from genuine planning outcomes.',
                      'Feedback and collision supervision are sampled; not continuous collision verification.',
                      'Robot is left at the cancellation stop state; any next execution needs fresh planning and validation.']}
    (out/'module06_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    (out/'summarize_module06.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(report,ensure_ascii=False,indent=2))
    print('SAVED:',out)
    return 0 if report['passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
