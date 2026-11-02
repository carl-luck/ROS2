import csv
import os
from pathlib import Path
import queue
import signal
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def validate_request(item, now, last_source, feedback_stamp):
    u = np.asarray(item['u'], dtype=float)
    source = item['source_wall']
    stamp = item['source_stamp']
    reasons = []
    if u.shape != (6,) or not np.all(np.isfinite(u)):
        reasons.append('malformed_command')
    elif np.max(np.abs(u)) > 0.02000001:
        reasons.append('command_speed_limit')
    if now-source > 0.25:
        reasons.append('request_age_exceeded')
    if source > now:
        reasons.append('source_wall_in_future')
    if last_source is not None and source <= last_source:
        reasons.append('source_wall_not_increasing')
    if feedback_stamp is None or abs(feedback_stamp-stamp) > 0.15:
        reasons.append('request_feedback_stamp_difference')
    if reasons:
        raise RuntimeError('invalid_command_request: '+','.join(reasons)+
                           '; age_wall_s='+str(now-source)+
                           '; source_stamp='+str(stamp)+
                           '; watchdog_feedback_stamp='+str(feedback_stamp))
    return u, source, stamp


def watchdog_main(incoming, outgoing, log_dir, names, lower, upper, parent_pid):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('p3_velocity_watchdog')
    publisher = node.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 1)
    state = {'q': None, 'v': None, 'stamp': None, 'wall': None, 'progress': None, 'fault': None}

    def receive(msg):
        try:
            if len(set(msg.name)) != len(msg.name):
                raise RuntimeError('duplicate_joint_names')
            i = [msg.name.index(n) for n in names]
            q = np.array([msg.position[k] for k in i]); v = np.array([msg.velocity[k] for k in i])
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            now = time.monotonic()
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):
                raise RuntimeError('nonfinite_feedback')
            if state['stamp'] is not None and stamp < state['stamp']:
                raise RuntimeError('feedback_time_reversed')
            if state['stamp'] is None or stamp > state['stamp']:
                state['progress'] = now
            state.update(q=q, v=v, stamp=stamp, wall=now)
        except Exception as exc:
            state['fault'] = str(exc)

    sub = node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data)
    desired = np.zeros(6); output = np.zeros(6)
    request_wall = None; last_source = None; last_stamp = None
    started = time.monotonic(); stop_at = None; stopped_since = None
    ready = False; normal_stop = False; reason = None; confirmed = False
    path = Path(log_dir)/'watchdog_commands.csv'
    with path.open('w', newline='') as stream:
        w = csv.writer(stream)
        w.writerow(['wall_monotonic_s', 'feedback_sim_s', 'stopping']+['u_'+n for n in names])
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.005)
                now = time.monotonic()
                while True:
                    try:
                        item = incoming.get_nowait()
                    except queue.Empty:
                        break
                    if item['kind'] == 'stop':
                        normal_stop = bool(item.get('normal'))
                        stop_at = stop_at or now
                        continue
                    if stop_at is not None:
                        continue
                    # Read the clock after dequeue: a newly queued source may postdate the loop's initial clock sample.
                    now = time.monotonic()
                    u, source, stamp = validate_request(item, now, last_source, state['stamp'])
                    desired = u; request_wall = source; last_source = source
                if os.getppid() != parent_pid:
                    raise RuntimeError('parent_process_gone')
                if state['fault']:
                    raise RuntimeError(state['fault'])
                fresh = state['wall'] is not None and now-state['wall'] < 0.3 and now-state['progress'] < 0.5
                if not ready and fresh:
                    if np.max(np.abs(state['v'])) > 0.001:
                        raise RuntimeError('watchdog_initial_not_stationary')
                    if publisher.get_subscription_count() == 0:
                        if now-started > 4:
                            raise RuntimeError('velocity_controller_not_subscribed')
                    else:
                        ready = True; outgoing.put({'kind': 'ready'})
                if node.count_publishers('/forward_velocity_controller/commands') > 1:
                    raise RuntimeError('multiple_velocity_command_publishers')
                if ready and not fresh:
                    raise RuntimeError('stale_feedback_or_clock_stall')
                if fresh and (np.any(state['q'] < lower) or np.any(state['q'] > upper) or np.max(np.abs(state['v'])) > 0.03):
                    raise RuntimeError('actual_position_or_speed_limit')
                if request_wall is not None and now-request_wall > 0.25 and stop_at is None:
                    raise RuntimeError('command_heartbeat_timeout')
                if request_wall is None and now-started > 5 and stop_at is None:
                    raise RuntimeError('no_command_request')
                if stop_at is None:
                    dt = 0 if last_stamp is None or state['stamp'] is None else max(0, min(0.04, state['stamp']-last_stamp))
                    output = np.clip(desired, output-0.1*dt, output+0.1*dt)
                    output = np.clip(output, -0.02, 0.02)
                else:
                    output = np.zeros(6)
                last_stamp = state['stamp']
                message = Float64MultiArray(); message.data = output.tolist(); publisher.publish(message)
                w.writerow([now, state['stamp'], stop_at is not None]+list(message.data))
                if stop_at is not None:
                    if fresh and np.max(np.abs(state['v'])) <= 0.001:
                        if stopped_since is None:
                            stopped_since = state['stamp']
                        elif state['stamp']-stopped_since >= 0.5:
                            confirmed = True; break
                    else:
                        stopped_since = None
                    if now-stop_at > 8:
                        break
        except BaseException as exc:
            reason = type(exc).__name__+': '+str(exc)
            outgoing.put({'kind': 'fault', 'error': reason})
            end = time.monotonic()+8
            stopped_since = None
            while rclpy.ok() and time.monotonic() < end:
                m = Float64MultiArray(); m.data = [0.0]*6; publisher.publish(m)
                w.writerow([time.monotonic(), state['stamp'], True]+list(m.data))
                rclpy.spin_once(node, timeout_sec=0.01)
                fresh = state['wall'] is not None and time.monotonic()-state['wall'] < 0.3 and time.monotonic()-state['progress'] < 0.5
                if fresh and np.max(np.abs(state['v'])) <= 0.001:
                    if stopped_since is None:
                        stopped_since = state['stamp']
                    elif state['stamp']-stopped_since >= 0.5:
                        confirmed = True; break
                else:
                    stopped_since = None
    outgoing.put({'kind': 'finished', 'stopped_confirmed': confirmed, 'normal_stop': normal_stop, 'error': reason})
    node.destroy_node()
    rclpy.shutdown()
