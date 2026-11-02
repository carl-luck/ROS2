import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from rosidl_runtime_py.set_message import set_message_fields


def main():
    parser = argparse.ArgumentParser(description='One small saved-trajectory execution; explicit --execute required.')
    parser.add_argument('capture_dir', type=Path)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        parser.error('This sends motion. Run only with --execute after reading the module instructions.')
    folder = args.capture_dir.resolve()
    script = Path(__file__).resolve()
    raw_bytes = (folder / 'trajectory_and_feedback.json').read_bytes()
    raw = json.loads(raw_bytes)
    saved = raw['display_trajectory']['trajectory']
    if len(saved) != 1:
        raise RuntimeError('Expected exactly one saved trajectory')
    traj = RobotTrajectory()
    set_message_fields(traj, saved[0])
    names = list(traj.joint_trajectory.joint_names)
    points = traj.joint_trajectory.points
    if len(names) != 6 or len(set(names)) != 6 or len(points) < 2 or traj.multi_dof_joint_trajectory.points:
        raise RuntimeError('Unsupported trajectory structure')
    times = [p.time_from_start.sec + p.time_from_start.nanosec / 1e9 for p in points]
    if times[0] < 0 or times[-1] <= 0 or any(b <= a for a, b in zip(times, times[1:])):
        raise RuntimeError('Invalid trajectory times')
    for p in points:
        if any(len(getattr(p, f)) != 6 or not all(math.isfinite(x) for x in getattr(p, f)) for f in ('positions', 'velocities', 'accelerations')):
            raise RuntimeError('Position, velocity, acceleration arrays must each contain six finite values')
        if max(abs(x - y) for x, y in zip(p.positions, points[0].positions)) > 0.06:
            raise RuntimeError('This introductory executor permits at most 0.06 rad departure per joint')
    before = set(folder.glob('validation_*'))
    print('Rechecking saved path against current scene/state. No execution yet.', flush=True)
    checked = subprocess.run([sys.executable, str(script.with_name('validate_saved_plan.py')), str(folder)], timeout=180)
    new = set(folder.glob('validation_*')) - before
    if checked.returncode != 0 or len(new) != 1:
        raise RuntimeError('Fresh validation did not pass; no motion sent')
    validation_dir = next(iter(new))
    validation = json.loads((validation_dir / 'validation_report.json').read_text())
    if not validation['passed'] or validation['raw_sha256'] != hashlib.sha256(raw_bytes).hexdigest():
        raise RuntimeError('Validation/hash mismatch; no motion sent')
    if (folder / 'trajectory_and_feedback.json').read_bytes() != raw_bytes:
        raise RuntimeError('Saved trajectory changed; no motion sent')
    out = folder / ('execution_' + datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z') + '_' + uuid.uuid4().hex[:8])
    out.mkdir(exist_ok=False)
    report = {'started_at': datetime.now().astimezone().isoformat(), 'validation_dir': str(validation_dir),
              'script_sha256': hashlib.sha256(script.read_bytes()).hexdigest(),
              'raw_sha256': hashlib.sha256(raw_bytes).hexdigest(), 'sent': False, 'passed': False,
              'thresholds': {'start_error_rad': 0.01, 'endpoint_error_rad': 0.005, 'stopped_speed_rad_s': 0.001,
                             'stopped_hold_sim_s': 0.5, 'wall_result_timeout_s': 15.0,
                             'feedback_receive_age_wall_s': 1.0, 'clock_stall_wall_s': 3.0},
              'limitations': ['Simulation only. Discrete preflight checks are not continuous collision guarantees.',
                              'Velocity/acceleration checks cover saved waypoints, not polynomial extrema between them.',
                              'Scene must remain unchanged throughout this manual single-run procedure.',
                              'Supervision runs in this process and is not an independent safety system.',
                              'Any unresolved action state or stop failure requires stopping the simulation manually.']}
    rclpy.init()
    node = rclpy.create_node('p3_execute_saved_plan', enable_rosout=False)
    state = {'joint': None, 'received': None, 'clock': None, 'clock_received': None, 'clock_progress': None}
    log = (out / 'execution_log.csv').open('w', newline='')
    writer = csv.writer(log)
    writer.writerow(['receive_monotonic_wall_s', 'feedback_sim_s'] + ['q_' + n for n in names] + ['v_' + n for n in names])

    def joints(msg):
        if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity) or len(set(msg.name)) != len(msg.name):
            return
        q, v = dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))
        if not all(n in q and n in v and math.isfinite(q[n]) and math.isfinite(v[n]) for n in names):
            return
        state['joint'] = (msg, q, v)
        state['received'] = time.monotonic()
        writer.writerow([state['received'], msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9] + [q[n] for n in names] + [v[n] for n in names])

    def clock(msg):
        now = time.monotonic()
        value = msg.clock.sec + msg.clock.nanosec / 1e9
        if state['clock'] is None or value > state['clock']:
            state['clock_progress'] = now
        state['clock'] = value
        state['clock_received'] = now

    subs = [node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data),
            node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)]
    action = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    handle = None
    result_future = None

    def spin_until(future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError('Application wall timeout; remote state may be unknown')
        return future.result()

    def query(srv, endpoint, req):
        client = node.create_client(srv, endpoint)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Service unavailable: ' + endpoint)
            return spin_until(client.call_async(req), 4)
        finally:
            node.destroy_client(client)

    def fresh():
        now = time.monotonic()
        if state['joint'] is None or state['clock'] is None:
            return False
        stamp = state['joint'][0].header.stamp
        return (now - state['received'] < 1 and now - state['clock_received'] < 1 and
                now - state['clock_progress'] < 3 and abs(state['clock'] - stamp.sec - stamp.nanosec / 1e9) < 0.5)

    def settled(require_goal):
        deadline = time.monotonic() + 5
        first_stamp = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not fresh():
                first_stamp = None
                continue
            msg, q, v = state['joint']
            ok = max(abs(v[n]) for n in names) <= 0.001
            if require_goal:
                ok = ok and max(abs(q[n] - points[-1].positions[i]) for i, n in enumerate(names)) <= 0.005
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if not ok:
                first_stamp = None
            elif first_stamp is None:
                first_stamp = stamp
            elif stamp - first_stamp >= 0.5:
                return True
        return False

    try:
        controllers = query(ListControllers, '/controller_manager/list_controllers', ListControllers.Request()).controller
        active = [c for c in controllers if c.state == 'active']
        report['controllers'] = [{'name': c.name, 'state': c.state, 'claimed_interfaces': list(c.claimed_interfaces)} for c in controllers]
        target = next((c for c in active if c.name == 'joint_trajectory_controller'), None)
        required = {n + '/position' for n in names}
        if target is None or set(target.claimed_interfaces) != required:
            raise RuntimeError('Trajectory controller resource ownership mismatch')
        if any(c.name != target.name and any(i.startswith(tuple(n + '/' for n in names)) for i in c.claimed_interfaces) for c in active):
            raise RuntimeError('Another active controller claims target joints')
        req = GetParameters.Request()
        req.names = ['moveit_simple_controller_manager.joint_trajectory_controller.default']
        req.names += ['robot_description_planning.joint_limits.' + n + '.max_acceleration' for n in names]
        values = query(GetParameters, '/move_group/get_parameters', req).values
        if values[0].type != 1 or not values[0].bool_value:
            raise RuntimeError('MoveIt default controller mapping mismatch')
        model = ET.fromstring((validation_dir / 'robot.urdf').read_text())
        model_joints = {j.attrib['name']: j for j in model.findall('joint')}
        report['waypoint_limits'] = {}
        for i, n in enumerate(names):
            vmax = float(model_joints[n].find('limit').attrib['velocity'])
            if values[i + 1].type != 3 or values[i + 1].double_value <= 0:
                raise RuntimeError('Missing acceleration limit: ' + n)
            amax = values[i + 1].double_value
            report['waypoint_limits'][n] = {'urdf_velocity_rad_s': vmax, 'moveit_acceleration_rad_s2': amax}
            if any(abs(p.velocities[i]) > 0.1 * vmax + 1e-6 or abs(p.accelerations[i]) > 0.1 * amax + 1e-6 for p in points):
                raise RuntimeError('Saved waypoint exceeds this module 0.10 scaling check: ' + n)
        if not action.wait_for_server(timeout_sec=4):
            raise RuntimeError('ExecuteTrajectory server unavailable')
        if not settled(False):
            raise RuntimeError('Initial fresh/stopped feedback not confirmed')
        if max(abs(state['joint'][1][n] - points[0].positions[i]) for i, n in enumerate(names)) > 0.01:
            raise RuntimeError('Current state does not match saved start')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        goal.trajectory.joint_trajectory.header.stamp.sec = 0
        goal.trajectory.joint_trajectory.header.stamp.nanosec = 0
        started = time.monotonic()
        report['sent'] = True
        report['status'] = 'goal_request_sent_acceptance_pending'
        (out / 'execution_report.json').write_text(json.dumps(report, indent=2))
        handle = spin_until(action.send_goal_async(goal), 4)
        if not handle.accepted:
            raise RuntimeError('Execution rejected')
        print('Execution accepted. Monitoring feedback and action result.', flush=True)
        result_future = handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - started > 15 or not fresh():
                raise RuntimeError('Execution wall timeout or stale/stalled feedback; requesting cancellation')
        result = result_future.result()
        report.update({'action_status': result.status, 'moveit_error_code': result.result.error_code.val,
                       'action_elapsed_wall_s': time.monotonic() - started})
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != 1:
            raise RuntimeError('Action did not succeed')
        report['endpoint_and_stopped_confirmed'] = settled(True)
        report['endpoint_error_rad'] = [state['joint'][1][n] - points[-1].positions[i] for i, n in enumerate(names)]
        report['passed'] = report['endpoint_and_stopped_confirmed']
        report['status'] = 'passed' if report['passed'] else 'endpoint_or_stop_check_failed'
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        report['status'] = 'failed_or_unknown'
        if handle is not None and handle.accepted and result_future is not None and not result_future.done():
            try:
                cancel = spin_until(handle.cancel_goal_async(), 3)
                report['cancel_goals_count'] = len(cancel.goals_canceling)
                final = spin_until(result_future, 5)
                report['final_action_status'] = final.status
            except (Exception, KeyboardInterrupt) as cancel_exc:
                report['cancel_error'] = str(cancel_exc)
        try:
            report['stopped_after_failure'] = settled(False)
        except (Exception, KeyboardInterrupt):
            report['stopped_after_failure'] = False
        print('FAILURE: Do not send another motion. If action state or stop is unresolved, Ctrl+C the simulation in terminal A.', flush=True)
    finally:
        report['finished_at'] = datetime.now().astimezone().isoformat()
        (out / 'execution_report.json').write_text(json.dumps(report, indent=2))
        log.close()
        print(json.dumps(report, indent=2))
        print('SAVED:', out)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
