import argparse
import copy
import csv
from datetime import datetime
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from plan_detour_case import scene_signature
from prepare_tracking_reference import evaluate_reference, JOINTS
from tracking_control_core import Reference, TrackingController
from tracking_velocity_watchdog import watchdog_main


def main():
    parser = argparse.ArgumentParser(description='Guarded simulation-only P/MPC velocity tracking. Does not switch controllers or reset robot.')
    parser.add_argument('--reference-dir', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--mode', choices=['p', 'mpc'], required=True)
    parser.add_argument('--pilot-seconds', type=float, default=0.0)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        parser.error('Requires --execute; this sends velocity commands')
    if not np.isfinite(args.pilot_seconds) or args.pilot_seconds < 0:
        parser.error('Invalid pilot duration')
    root = Path.home()/'ROS2/arm/P3'
    out = root/'data/runs'/(datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_gazebo_tracking_'+args.mode+'_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    report = {'passed': False, 'mode': args.mode, 'pilot_seconds': args.pilot_seconds,
              'reference_dir': str(args.reference_dir.resolve()), 'motion_command_requested': False,
              'limits': ['Simulation experiment, not an independent hardware safety system.',
                         'Watchdog and controller are separate processes, but joint failure can leave a latched command.',
                         'Collision supervision samples actual and constant-command lookahead states, not continuous collision avoidance.',
                         'MPC has no obstacle constraints; tracking error can invalidate nominal collision clearance.',
                         'No automatic controller switching, reset, execution retry or next run.',
                         'Zero command on stop bypasses slew limits; this is not an emergency-stop guarantee.']}
    cfg = json.loads(args.config.read_text())
    cfg['scenarios'] = []
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('p3_gazebo_tracking_'+args.mode, parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'q': None, 'v': None, 'stamp': None, 'wall': None, 'clock': None, 'clock_wall': None, 'fault': None}
    process = None; incoming = outgoing = None; finished = None
    pending = []; clients = []

    def receive(msg):
        try:
            assert len(set(msg.name)) == len(msg.name)
            i = [msg.name.index(n) for n in JOINTS]
            q = np.array([msg.position[k] for k in i]); v = np.array([msg.velocity[k] for k in i])
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)) or state['stamp'] is not None and stamp < state['stamp']:
                raise RuntimeError('invalid_or_reversed_joint_feedback')
            state.update(q=q, v=v, stamp=stamp, wall=time.monotonic())
        except Exception as exc:
            state['fault'] = str(exc)

    def clock(msg):
        value = msg.clock.sec+msg.clock.nanosec/1e9
        if state['clock'] is not None and value < state['clock']:
            state['fault'] = 'clock_reversed'
        state['clock'] = value; state['clock_wall'] = time.monotonic()

    subscriptions = [node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data),
                     node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)]

    def client(cls, endpoint):
        c = node.create_client(cls, endpoint); clients.append(c)
        if not c.wait_for_service(timeout_sec=4):
            raise RuntimeError('Service unavailable: '+endpoint)
        return c

    def call(c, request):
        f = c.call_async(request); end = time.monotonic()+6
        while not f.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not f.done():
            raise RuntimeError('Read-only service timeout')
        return f.result()

    def feedback():
        now = time.monotonic()
        if (state['fault'] or state['wall'] is None or now-state['wall'] > 0.3
                or state['clock_wall'] is None or now-state['clock_wall'] > 0.3
                or abs(state['clock']-state['stamp']) > 0.15):
            raise RuntimeError('Feedback/clock invalid, stale or inconsistent: '+str(state['fault']))

    def controller_ok(response):
        active = [c for c in response.controller if c.state == 'active']
        target = next((c for c in active if c.name == 'forward_velocity_controller'), None)
        return (target is not None and set(target.claimed_interfaces) == {n+'/velocity' for n in JOINTS}
                and not any(c.name != target.name and any(i.startswith(tuple(n+'/' for n in JOINTS)) for i in c.claimed_interfaces) for c in active))

    def scene_req():
        request = GetPlanningScene.Request(); request.components.components = 1023
        return request

    try:
        source = json.loads((args.reference_dir/'tracking_reference_report.json').read_text())
        assert source['passed']
        for name, digest in source['artifacts_sha256'].items():
            assert hashlib.sha256((args.reference_dir/name).read_bytes()).hexdigest() == digest, 'Changed reference artifact: '+name
        assert hashlib.sha256(Path(__file__).with_name('prepare_tracking_reference.py').read_bytes()).hexdigest() == source['artifacts_sha256']['prepare_tracking_reference.py']
        assert cfg['ts'] == 0.02 and cfg['vmax'] == 0.02 and cfg['rate'] == 0.1
        for name in ('run_tracking_gazebo.py', 'tracking_velocity_watchdog.py', 'tracking_control_core.py', 'prepare_tracking_reference.py', 'prepare_detour_execution.py', 'plan_detour_case.py'):
            shutil.copyfile(Path(__file__).with_name(name), out/name)
        shutil.copyfile(args.config, out/'config.json')
        report['config'] = cfg
        report['reference_sha256'] = source['artifacts_sha256']['reference_polynomials.npz']
        ref = Reference(args.reference_dir/'reference_polynomials.npz', evaluate_reference)
        assert ref.names == JOINTS
        if args.pilot_seconds and not 1 <= args.pilot_seconds < ref.duration:
            raise RuntimeError('Pilot must be >=1 s and shorter than complete reference')
        with (out/'box_verify.log').open('w') as stream:
            proc = subprocess.run([sys.executable, str(Path(__file__).with_name('manage_box_scene.py')),
                                   '--config', str(root/'config/box_scene_v1.json'), '--mode', 'verify'],
                                  stdout=stream, stderr=subprocess.STDOUT, timeout=60)
        if proc.returncode:
            raise RuntimeError('Gazebo/MoveIt scene verification failed; inspect box_verify.log')
        cm = client(ListControllers, '/controller_manager/list_controllers')
        if not controller_ok(call(cm, ListControllers.Request())):
            raise RuntimeError('Expected only velocity ownership of arm joints; switch manually first')
        params = client(GetParameters, '/forward_velocity_controller/get_parameters')
        req = GetParameters.Request(); req.names = ['joints']
        assert list(call(params, req).values[0].string_array_value) == JOINTS, 'Velocity controller joint order mismatch'
        params_model = client(GetParameters, '/move_group/get_parameters')
        req = GetParameters.Request(); req.names = ['robot_description', 'robot_description_semantic', 'use_sim_time']
        values = call(params_model, req).values
        assert values[2].bool_value
        for i, name in enumerate(('robot.urdf', 'robot.srdf')):
            assert values[i].string_value == (args.reference_dir/name).read_text(), 'Live model changed'
        sc = client(GetPlanningScene, '/get_planning_scene')
        validity = client(GetStateValidity, '/check_state_validity')
        scene = call(sc, scene_req()).scene
        assert scene_signature(scene) == source['source_scene_signature'], 'Scene changed'
        model = ET.fromstring((args.reference_dir/'robot.urdf').read_text())
        joints = {j.attrib['name']: j for j in model.findall('joint')}
        lower, upper = [], []
        for n in JOINTS:
            j = joints[n]; limit = j.find('limit'); safety = j.find('safety_controller')
            lo, hi = float(limit.attrib['lower']), float(limit.attrib['upper'])
            if safety is not None:
                lo = max(lo, float(safety.attrib.get('soft_lower_limit', lo)))
                hi = min(hi, float(safety.attrib.get('soft_upper_limit', hi)))
            lower.append(lo+cfg['joint_margin']); upper.append(hi-cfg['joint_margin'])
        lower, upper = np.array(lower), np.array(upper)

        def state_req(q):
            req = GetStateValidity.Request(); req.group_name = 'ur_manipulator'
            req.robot_state = copy.deepcopy(scene.robot_state); req.robot_state.is_diff = False
            req.robot_state.joint_state.name = JOINTS
            req.robot_state.joint_state.position = q.tolist()
            req.robot_state.joint_state.velocity = []; req.robot_state.joint_state.effort = []
            req.robot_state.joint_state.header.stamp.sec = 0; req.robot_state.joint_state.header.stamp.nanosec = 0
            return req

        deadline = time.monotonic()+8; initial = None; stopped_at = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if state['q'] is None or state['clock'] is None:
                continue
            feedback()
            if np.max(np.abs(state['v'])) > 0.001:
                stopped_at = None; continue
            if stopped_at is None:
                stopped_at = state['stamp']
            if state['stamp']-stopped_at >= 0.5:
                initial = state['q'].copy(); break
        if initial is None:
            raise RuntimeError('Fresh advancing feedback and stationary start not confirmed')
        report['initial_q_rad'] = initial.tolist()
        assert np.max(np.abs(initial-source['initial_q_rad'])) <= 0.001, 'Prepare common start again; never replay old preparation'
        assert call(validity, state_req(initial)).valid, 'Current state invalid'
        if node.count_publishers('/forward_velocity_controller/commands'):
            raise RuntimeError('Another velocity command publisher exists')
        control = TrackingController(args.mode, cfg, lower, upper, ref)
        context = multiprocessing.get_context('spawn')
        incoming = context.Queue(maxsize=2); outgoing = context.Queue()
        process = context.Process(target=watchdog_main, args=(incoming, outgoing, str(out), JOINTS, lower, upper, os.getpid()))
        process.start()
        deadline = time.monotonic()+6
        ready = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            try:
                message = outgoing.get_nowait()
                if message['kind'] == 'ready':
                    ready = True; break
                raise RuntimeError('Watchdog startup: '+str(message))
            except queue.Empty:
                if not process.is_alive():
                    raise RuntimeError('Watchdog exited during startup')
        assert ready, 'Watchdog did not become ready'
        feedback()
        start_sim = state['stamp']; start_wall = time.monotonic(); last_sim = start_sim-cfg['ts']
        duration = args.pilot_seconds or ref.duration+cfg['settle_s']
        report['requested_tracking_sim_s'] = duration
        report['motion_command_requested'] = True
        previous = np.zeros(6); errors = []; costs = []
        fallbacks = deadline_fallbacks = checks = scenes = 0
        next_check = next_scene = next_cm = time.monotonic()
        completed = False
        print('TRACKING '+args.mode+': '+str(round(duration, 2))+' simulation seconds; no other commands allowed.', flush=True)
        with (out/'tracking_log.csv').open('w', newline='') as stream:
            w = csv.writer(stream)
            w.writerow(['elapsed_sim_s', 'feedback_sim_s', 'compute_wall_s', 'fallback', 'deadline_fallback', 'status']+
                       ['q_'+n for n in JOINTS]+['ref_'+n for n in JOINTS]+['u_'+n for n in JOINTS])
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.002)
                now = time.monotonic(); feedback()
                if not process.is_alive():
                    raise RuntimeError('Watchdog process exited')
                try:
                    message = outgoing.get_nowait()
                    raise RuntimeError('Watchdog reported: '+str(message))
                except queue.Empty:
                    pass
                if now-start_wall > duration*5+60:
                    raise RuntimeError('Tracking wall timeout')
                for item in list(pending):
                    future, sent, kind = item
                    if future.done():
                        response = future.result()
                        if kind == 'state':
                            if not response.valid:
                                raise RuntimeError('Sampled actual/lookahead state invalid')
                            checks += 1
                        elif kind == 'scene':
                            if scene_signature(response.scene) != source['source_scene_signature']:
                                raise RuntimeError('Scene changed during tracking')
                            scenes += 1
                        elif not controller_ok(response):
                            raise RuntimeError('Controller ownership changed')
                        pending.remove(item)
                    elif now-sent > 0.5:
                        raise RuntimeError('Supervision service response timeout')
                elapsed = state['stamp']-start_sim
                if elapsed >= duration:
                    completed = True; break
                if state['stamp']-last_sim < cfg['ts']-1e-9:
                    continue
                dt = state['stamp']-last_sim
                if dt > 0.08:
                    raise RuntimeError('Control simulation step gap exceeded 0.08 s')
                q = state['q'].copy(); source_wall = state['wall']; stamp = state['stamp']
                target, feedforward, _ = ref.at(elapsed)
                error = q-target
                if np.max(np.abs(error)) > 0.02:
                    raise RuntimeError('Tracking error exceeds 0.02 rad')
                u, info = control.command(q, previous, elapsed)
                deadline_miss = info['compute_wall_s'] > cfg['solver_time_limit_s']
                if deadline_miss:
                    u = feedforward+cfg['kp']*(target-q)
                change = cfg['rate']*min(dt, 0.04)
                u = np.clip(u, np.maximum(-cfg['vmax'], previous-change), np.minimum(cfg['vmax'], previous+change))
                if np.any(q+0.1*u < lower) or np.any(q+0.1*u > upper):
                    raise RuntimeError('Predicted joint bounds exceeded')
                try:
                    incoming.put_nowait({'kind': 'command', 'u': u.tolist(), 'source_wall': source_wall, 'source_stamp': stamp})
                except queue.Full:
                    raise RuntimeError('Watchdog command queue blocked')
                errors.append(error); costs.append(info['compute_wall_s'])
                fallbacks += bool(info['fallback']); deadline_fallbacks += bool(deadline_miss)
                w.writerow([elapsed, stamp, info['compute_wall_s'], info['fallback'], deadline_miss, info['status']]+q.tolist()+target.tolist()+u.tolist())
                previous = u; last_sim = stamp
                if now >= next_check and not any(i[2] == 'state' for i in pending):
                    for probe in (q, q+0.1*u):
                        pending.append((validity.call_async(state_req(probe)), now, 'state'))
                    next_check = now+0.1
                if now >= next_scene and not any(i[2] == 'scene' for i in pending):
                    pending.append((sc.call_async(scene_req()), now, 'scene')); next_scene = now+1
                if now >= next_cm and not any(i[2] == 'controller' for i in pending):
                    pending.append((cm.call_async(ListControllers.Request()), now, 'controller')); next_cm = now+1
        report.update(tracking_completed=completed, sampled_state_checks=checks, unchanged_scene_checks=scenes,
                      solver_fallback_count=int(fallbacks), deadline_fallback_count=int(deadline_fallbacks),
                      samples=len(errors), error_rmse_rad=float(np.sqrt(np.mean(np.asarray(errors)**2))),
                      max_abs_error_rad=float(np.max(np.abs(errors))),
                      compute_median_wall_s=float(np.median(costs)), compute_max_wall_s=float(np.max(costs)))
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
    finally:
        if process is not None:
            try:
                incoming.put({'kind': 'stop', 'normal': 'error' not in report}, timeout=0.3)
            except Exception:
                pass
            end = time.monotonic()+10
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
                try:
                    message = outgoing.get_nowait()
                    if message['kind'] == 'finished':
                        finished = message; break
                    if message['kind'] == 'fault':
                        report['watchdog_fault'] = message['error']
                except queue.Empty:
                    if not process.is_alive():
                        break
            process.join(timeout=0.5)
            if process.is_alive() or finished is None or not finished.get('stopped_confirmed'):
                emergency = node.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 1)
                end = time.monotonic()+1
                while time.monotonic() < end:
                    message = Float64MultiArray(); message.data = [0.0]*6; emergency.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.01)
                report['emergency_zero_attempted'] = True
            report['watchdog_result'] = finished
            report['stopped_confirmed'] = bool(finished and finished['stopped_confirmed'])
            if process.is_alive():
                process.terminate(); process.join(timeout=2)
        if report.get('tracking_completed') and report.get('stopped_confirmed') and not report.get('error') and finished and not finished.get('error'):
            try:
                for future, sent, kind in pending:
                    deadline = time.monotonic()+0.5
                    while not future.done() and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.01)
                    if not future.done():
                        raise RuntimeError('Final pending supervision query timed out')
                    response = future.result()
                    if kind == 'state' and not response.valid:
                        raise RuntimeError('Pending sampled state was invalid')
                    if kind == 'scene' and scene_signature(response.scene) != source['source_scene_signature']:
                        raise RuntimeError('Pending scene check found a change')
                    if kind == 'controller' and not controller_ok(response):
                        raise RuntimeError('Pending controller check failed')
                feedback()
                report['final_state_valid'] = call(validity, state_req(state['q'])).valid
                report['final_scene_unchanged'] = scene_signature(call(sc, scene_req()).scene) == source['source_scene_signature']
                report['final_controller_ok'] = controller_ok(call(cm, ListControllers.Request()))
                report['endpoint_error_rad'] = (state['q']-np.asarray(source['final_q_rad'])).tolist()
                report['endpoint_passed'] = bool(args.pilot_seconds or np.max(np.abs(report['endpoint_error_rad'])) <= 0.005)
                report['passed'] = report['final_state_valid'] and report['final_scene_unchanged'] and report['final_controller_ok'] and report['endpoint_passed']
            except Exception as exc:
                report['final_verification_error'] = str(exc)
        report['finished_at'] = datetime.now().astimezone().isoformat()
        report['next_motion_allowed'] = False
        (out/'gazebo_tracking_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print('SAVED:', out)
        if not report['passed']:
            print('FAILED: no retry. If stop is unconfirmed or robot keeps moving, Ctrl+C Gazebo terminal A.', flush=True)
        node.destroy_node(); rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
