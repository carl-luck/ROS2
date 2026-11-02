import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--wait', type=float, default=120.0)
    parser.add_argument('--output-root', default=str(Path.home() / 'ROS2/arm/P3/data/runs'))
    args = parser.parse_args()
    if not math.isfinite(args.wait) or args.wait <= 0:
        parser.error('--wait must be finite and positive')
    rclpy.init()
    node = rclpy.create_node('p3_capture_rviz_plan', enable_rosout=False)
    state = {'joint': None, 'received': None, 'plan': None, 'at_plan': None}

    def joint_callback(msg):
        state['joint'] = msg
        state['received'] = time.monotonic()

    def plan_callback(msg):
        if state['plan'] is None:
            state['plan'] = msg
            state['at_plan'] = (state['joint'], state['received'], time.monotonic())

    subscriptions = [
        node.create_subscription(JointState, '/joint_states', joint_callback, qos_profile_sensor_data),
        node.create_subscription(DisplayTrajectory, '/display_planned_path', plan_callback, 10),
    ]
    try:
        started = time.monotonic()
        announced = False
        print('Waiting for fresh joint feedback and a planning publisher...', flush=True)
        while time.monotonic() - started < args.wait and state['plan'] is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not announced and state['received'] is not None:
                if time.monotonic() - state['received'] < 1.0 and node.count_publishers('/display_planned_path') > 0:
                    print('READY: in RViz click Plan ONCE. Do not click Execute.', flush=True)
                    announced = True
        if state['plan'] is None:
            print('WAIT_TIMEOUT: no DisplayTrajectory received. No execution was requested.')
            return 2
        plan = state['plan']
        joint, received, captured = state['at_plan']
        issues = []
        if not announced:
            issues.append('plan_arrived_before_ready')
        age = None if received is None else captured - received
        if age is None or age > 1.0:
            issues.append('feedback_missing_or_receive_age_over_1_wall_second')
        expected = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'}
        segments = []
        if len(plan.trajectory) != 1:
            issues.append('expected_one_robot_trajectory')
        for index, segment in enumerate(plan.trajectory):
            trajectory = segment.joint_trajectory
            names = list(trajectory.joint_names)
            if len(names) != 6 or set(names) != expected:
                issues.append(f'segment_{index}_joint_names_invalid')
            if segment.multi_dof_joint_trajectory.points:
                issues.append(f'segment_{index}_unsupported_multi_dof')
            times = []
            for point in trajectory.points:
                times.append(point.time_from_start.sec * 1000000000 + point.time_from_start.nanosec)
                if len(point.positions) != len(names):
                    issues.append(f'segment_{index}_position_dimension')
                for field in ('positions', 'velocities', 'accelerations', 'effort'):
                    values = list(getattr(point, field))
                    if field != 'positions' and values and len(values) != len(names):
                        issues.append(f'segment_{index}_{field}_dimension')
                    if not all(math.isfinite(value) for value in values):
                        issues.append(f'segment_{index}_{field}_not_finite')
            if len(times) < 2 or times[0] < 0 or any(b <= a for a, b in zip(times, times[1:])):
                issues.append(f'segment_{index}_empty_short_or_invalid_time_sequence')
            summary = {'joint_names': names, 'point_count': len(times),
                       'duration_s': times[-1] / 1e9 if times else None}
            if trajectory.points:
                summary['start_rad'] = list(trajectory.points[0].positions)
                summary['goal_rad'] = list(trajectory.points[-1].positions)
                if joint is not None and len(joint.name) == len(joint.position) and len(set(joint.name)) == len(joint.name):
                    actual = dict(zip(joint.name, joint.position))
                    if all(name in actual for name in names) and len(summary['start_rad']) == len(names):
                        summary['start_minus_feedback_rad'] = [q - actual[name] for name, q in zip(names, summary['start_rad'])]
            segments.append(summary)
        stamp = datetime.now().astimezone().isoformat()
        folder = Path(args.output_root) / (datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z') + '_rviz_capture_' + uuid.uuid4().hex[:10])
        folder.mkdir(parents=True, exist_ok=False)
        raw = {'display_trajectory': message_to_ordereddict(plan),
               'feedback_at_plan_receive': None if joint is None else message_to_ordereddict(joint)}
        raw_path = folder / 'trajectory_and_feedback.json'
        raw_path.write_text(json.dumps(raw, ensure_ascii=False, indent=2, allow_nan=True), encoding='utf-8')
        repo = Path.home() / 'ROS2'
        git = {}
        for label, command in [('commit', ['rev-parse', 'HEAD']), ('dirty_status', ['status', '--short'])]:
            result = subprocess.run(['git', '-C', str(repo), *command], capture_output=True, text=True, timeout=5)
            git[label] = result.stdout.strip() if result.returncode == 0 else 'unavailable'
        report = {'captured_at': stamp, 'topic': '/display_planned_path',
                  'feedback_receive_age_wall_s': age, 'git': git,
                  'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  'raw_sha256': hashlib.sha256(raw_path.read_bytes()).hexdigest(),
                  'numerical_check_passed': not issues, 'issues': sorted(set(issues)),
                  'segments': segments, 'execution_requested_by_script': False,
                  'limits': ['Preview capture does not contain the planning request or its return code.',
                             'No joint limit, collision, feedback stamp age, or execution readiness validation.',
                             'Receive age is not source message age. Do not execute based on this report.',
                             'One CLI capture is development evidence, not a frozen formal experiment.']}
        (folder / 'capture_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print('SAVED:', folder)
        return 0 if not issues else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
