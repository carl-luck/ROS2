import argparse
import copy
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from rosidl_runtime_py.set_message import set_message_fields

from plan_detour_case import scene_signature


def main():
    parser=argparse.ArgumentParser(description='Execute only fresh validated preparation, using direct controller action. No automatic reset/retries.')
    parser.add_argument('--inputs-dir',required=True,type=Path)
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if not args.execute:
        parser.error('Motion requires explicit --execute')
    inputs=args.inputs_dir.resolve()
    validated=json.loads((inputs/'execution_inputs_report.json').read_text())
    if not validated['passed'] or not all(validated['paths'][k]['passed'] for k in ('preparation','detour')):
        raise RuntimeError('Execution input validation did not pass')
    config_bytes=args.config.read_bytes()
    c=json.loads(config_bytes)
    if any(not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for k,v in c.items() if k!='config_version'):
        raise RuntimeError('Expected positive finite execution parameters')
    trajectories={}
    for label in ('preparation','detour'):
        raw_bytes=(inputs/label/'trajectory_and_feedback.json').read_bytes()
        if hashlib.sha256(raw_bytes).hexdigest()!=validated['paths'][label]['retimed_sha256']:
            raise RuntimeError('Retimed trajectory hash mismatch: '+label)
        data=json.loads(raw_bytes)['display_trajectory']['trajectory']
        if len(data)!=1:
            raise RuntimeError('Expected one robot trajectory')
        trajectory=RobotTrajectory()
        set_message_fields(trajectory,data[0])
        trajectories[label]=trajectory
    names=list(trajectories['preparation'].joint_trajectory.joint_names)
    if names!=list(trajectories['detour'].joint_trajectory.joint_names) or len(names)!=6:
        raise RuntimeError('Unexpected joint ordering')
    root=inputs/('tracking_preparation_'+datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_'+uuid.uuid4().hex[:8])
    root.mkdir(exist_ok=False)
    (root/'config.json').write_bytes(config_bytes)
    for filename in ('execute_tracking_preparation.py','plan_detour_case.py','manage_box_scene.py'):
        shutil.copyfile(Path(__file__).with_name(filename),root/filename)
    report={'started_at':datetime.now().astimezone().isoformat(),'inputs_dir':str(inputs),'config':c,
            'passed':False,'phases':{},'active_phase':None,'next_motion_allowed':False,
            'input_report_sha256':hashlib.sha256((inputs/'execution_inputs_report.json').read_bytes()).hexdigest(),
            'limitations':['Simulation only. Supervision is in the sending process, not an independent safety mechanism.',
                          'Runtime collision checks sample actual states; they are not continuous swept-volume guarantees.',
                          'Scene changes are detected by periodic queries, not instantaneously.',
                          'Client action elapsed time includes communication/processing; not pure robot motion time.',
                          'No automatic return to start, retry, or resumption after failure.']}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('p3_execute_tracking_preparation',enable_rosout=False,
                          parameter_overrides=[Parameter('use_sim_time',value=True)])
    state={'joint':None,'joint_wall':None,'clock':None,'clock_wall':None,'clock_progress':None,
           'clock_reversed':False,'control':None,'control_wall':None,'phase':None,'log':None,'control_log':None}
    handle=None
    send_future=None
    result_future=None
    failure_file=None

    def flush_report():
        (root/'sequence_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))

    def joints(msg):
        if len(msg.name)!=len(msg.position) or len(msg.name)!=len(msg.velocity) or len(set(msg.name))!=len(msg.name):
            return
        q,v=dict(zip(msg.name,msg.position)),dict(zip(msg.name,msg.velocity))
        if not all(n in q and n in v and math.isfinite(q[n]) and math.isfinite(v[n]) for n in names):
            return
        state['joint']=(msg,q,v)
        state['joint_wall']=time.monotonic()
        if state['log']:
            state['log'].writerow([state['joint_wall'],msg.header.stamp.sec+msg.header.stamp.nanosec/1e9]+[q[n] for n in names]+[v[n] for n in names])

    def clocks(msg):
        now=time.monotonic()
        value=msg.clock.sec+msg.clock.nanosec/1e9
        if state['clock'] is not None and value<state['clock']:
            state['clock_reversed']=True
        if state['clock'] is None or value>state['clock']:
            state['clock_progress']=now
        state['clock'],state['clock_wall']=value,now

    def control(msg):
        if len(msg.joint_names)!=6 or set(msg.joint_names)!=set(names):
            return
        if len(msg.actual.positions)!=6 or len(msg.desired.positions)!=6:
            return
        qa,qd=dict(zip(msg.joint_names,msg.actual.positions)),dict(zip(msg.joint_names,msg.desired.positions))
        if not all(math.isfinite(qa[n]) and math.isfinite(qd[n]) for n in names):
            return
        error=[qd[n]-qa[n] for n in names]
        state['control']=(msg,error)
        state['control_wall']=time.monotonic()
        if state['control_log']:
            state['control_log'].writerow([state['control_wall'],msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                msg.desired.time_from_start.sec+msg.desired.time_from_start.nanosec/1e9]+[qd[n] for n in names]+[qa[n] for n in names]+error)

    subs=[node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data),
          node.create_subscription(Clock,'/clock',clocks,qos_profile_sensor_data),
          node.create_subscription(JointTrajectoryControllerState,'/joint_trajectory_controller/state',control,qos_profile_sensor_data)]
    action=ActionClient(node,FollowJointTrajectory,'/joint_trajectory_controller/follow_joint_trajectory')

    def spin_until(future,seconds):
        deadline=time.monotonic()+seconds
        while not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.05)
        if not future.done():
            raise RuntimeError('Application wall timeout; remote request state may be unknown')
        return future.result()

    def query(cls,name,request):
        client=node.create_client(cls,name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Service unavailable: '+name)
            return spin_until(client.call_async(request),5)
        finally:
            node.destroy_client(client)

    def scene_request():
        req=GetPlanningScene.Request()
        req.components.components=1023
        return req

    def fresh(require_control=False):
        now=time.monotonic()
        if state['joint'] is None or state['clock'] is None or state['clock_reversed']:
            return False
        msg=state['joint'][0]
        return (now-state['joint_wall']<c['feedback_receive_limit_wall_s'] and
                now-state['clock_wall']<c['feedback_receive_limit_wall_s'] and
                now-state['clock_progress']<c['clock_stall_limit_wall_s'] and
                abs(state['clock']-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9)<c['feedback_clock_difference_limit_s'] and
                (not require_control or state['control_wall'] is not None and now-state['control_wall']<c['feedback_receive_limit_wall_s']))

    def settled(goal=None):
        deadline=time.monotonic()+c['stopped_wait_wall_s']
        began=None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=0.05)
            if not fresh():
                began=None
                continue
            msg,q,v=state['joint']
            good=max(abs(v[n]) for n in names)<=c['stopped_speed_limit_rad_s']
            if goal is not None:
                good=good and max(abs(q[n]-goal[i]) for i,n in enumerate(names))<=c['endpoint_error_limit_rad']
            t=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if not good:
                began=None
            elif began is None:
                began=t
            elif t-began>=c['stopped_hold_sim_s']:
                return True
        return False

    def runtime_state_request(scene):
        req=GetStateValidity.Request()
        req.group_name='ur_manipulator'
        req.robot_state=copy.deepcopy(scene.robot_state)
        req.robot_state.joint_state.name=list(names)
        req.robot_state.joint_state.position=[state['joint'][1][n] for n in names]
        req.robot_state.joint_state.velocity=[]
        req.robot_state.joint_state.effort=[]
        req.robot_state.joint_state.header.stamp.sec=0
        req.robot_state.joint_state.header.stamp.nanosec=0
        req.robot_state.is_diff=False
        return req

    try:
        for label in ('preparation',):
            handle=None;send_future=None;result_future=None
            report['active_phase']=label
            info={'passed':False,'sent':False,'status':'preflight'}
            report['phases'][label]=info
            flush_report()
            print('PREFLIGHT '+label+': no new motion yet.',flush=True)
            phase_dir=root/label
            phase_dir.mkdir()
            # Independent Gazebo and MoveIt box query. This child is read-only.
            with (phase_dir/'box_verify_console.txt').open('w') as stream:
                verify=subprocess.run([sys.executable,str(Path(__file__).with_name('manage_box_scene.py')),
                    '--config',str(Path.home()/'ROS2/arm/P3/config/box_scene_v1.json'),'--mode','verify'],
                    stdout=stream,stderr=subprocess.STDOUT,timeout=60)
            if verify.returncode!=0:
                raise RuntimeError('Dual scene verification failed; inspect box_verify_console.txt')
            controllers=query(ListControllers,'/controller_manager/list_controllers',ListControllers.Request()).controller
            info['controllers']=[{'name':x.name,'state':x.state,'claimed_interfaces':list(x.claimed_interfaces)} for x in controllers]
            active=[x for x in controllers if x.state=='active']
            target=next((x for x in active if x.name=='joint_trajectory_controller'),None)
            if target is None or set(target.claimed_interfaces)!={n+'/position' for n in names}:
                raise RuntimeError('Trajectory controller resource ownership mismatch')
            if any(x.name!=target.name and any(i.startswith(tuple(n+'/' for n in names)) for i in x.claimed_interfaces) for x in active):
                raise RuntimeError('Another controller claims target joints')
            req=GetParameters.Request()
            req.names=['interpolation_method']
            if query(GetParameters,'/joint_trajectory_controller/get_parameters',req).values[0].string_value!='splines':
                raise RuntimeError('Controller interpolation changed since trajectory validation')
            req.names=['robot_description','robot_description_semantic','use_sim_time',
                       'moveit_simple_controller_manager.joint_trajectory_controller.default']
            req.names+=['robot_description_planning.joint_limits.'+n+'.max_acceleration' for n in names]
            params=query(GetParameters,'/move_group/get_parameters',req).values
            if not params[2].bool_value or not params[3].bool_value:
                raise RuntimeError('MoveIt simulation time or default controller mapping mismatch')
            for i,filename in enumerate(('robot.urdf','robot.srdf')):
                if params[i].string_value!=(inputs/label/filename).read_text():
                    raise RuntimeError('Model changed since validation')
            expected_limits=validated['paths'][label]['acceleration_limits_rad_s2']
            scaling=validated['config']['acceleration_scaling']
            if any(p.type!=3 or abs(p.double_value*scaling-e)>1e-8 for p,e in zip(params[4:],expected_limits)):
                raise RuntimeError('Runtime acceleration limits differ from validated limits')
            scene=query(GetPlanningScene,'/get_planning_scene',scene_request()).scene
            if scene_signature(scene)!=validated['source_scene_signature']:
                raise RuntimeError('Scene differs from validated snapshot')
            if not action.wait_for_server(timeout_sec=4):
                raise RuntimeError('ExecuteTrajectory server unavailable')
            if not settled():
                raise RuntimeError('Fresh initial stopped state not confirmed')
            trajectory=copy.deepcopy(trajectories[label])
            pts=trajectory.joint_trajectory.points
            errors=[pts[0].positions[i]-state['joint'][1][n] for i,n in enumerate(names)]
            info['start_minus_actual_rad']=errors
            if max(abs(e) for e in errors)>c['start_error_limit_rad']:
                raise RuntimeError('Actual state differs from saved '+label+' start; do not skip preparation')
            if not query(GetStateValidity,'/check_state_validity',runtime_state_request(scene)).valid:
                raise RuntimeError('Current actual state invalid')
            joint_file=(phase_dir/'execution_log.csv').open('w',newline='')
            control_file=(phase_dir/'controller_log.csv').open('w',newline='')
            state['log']=csv.writer(joint_file)
            state['log'].writerow(['receive_monotonic_wall_s','feedback_sim_s']+['q_'+n for n in names]+['v_'+n for n in names])
            state['control_log']=csv.writer(control_file)
            state['control_log'].writerow(['receive_monotonic_wall_s','feedback_sim_s','desired_time_from_start_s']+
                ['desired_'+n for n in names]+['actual_'+n for n in names]+['error_'+n for n in names])
            state['phase']=label
            handle=None;send_future=None;result_future=None
            valid_client=node.create_client(GetStateValidity,'/check_state_validity')
            scene_client=node.create_client(GetPlanningScene,'/get_planning_scene')
            pending_valid=None;pending_scene=None
            validity_count=0;scene_count=0
            try:
                if not fresh(True):
                    raise RuntimeError('Fresh controller feedback not available')
                if hashlib.sha256((inputs/label/'trajectory_and_feedback.json').read_bytes()).hexdigest()!=validated['paths'][label]['retimed_sha256']:
                    raise RuntimeError('Trajectory file changed')
                goal=FollowJointTrajectory.Goal()
                goal.trajectory=trajectory.joint_trajectory
                goal.trajectory.header.stamp.sec=0
                goal.trajectory.header.stamp.nanosec=0
                began=time.monotonic()
                initial_clock=state['clock']
                info['sent']=True
                info['status']='acceptance_pending'
                info['send_at']=datetime.now().astimezone().isoformat()
                flush_report()
                send_future=action.send_goal_async(goal)
                handle=spin_until(send_future,c['accept_wait_wall_s'])
                if not handle.accepted:
                    raise RuntimeError('Execution rejected')
                result_future=handle.get_result_async()
                info['status']='executing'
                duration=validated['paths'][label]['retimed_duration_sim_s']
                wall_budget=duration*c['action_wall_timeout_scale']+c['action_wall_timeout_margin_s']
                info['action_wall_budget_s']=wall_budget
                flush_report()
                print('EXECUTING '+label+'; planned simulation duration '+str(round(duration,2))+' s.',flush=True)
                next_valid=next_scene=time.monotonic()
                max_error=0.0
                while not result_future.done():
                    rclpy.spin_once(node,timeout_sec=0.02)
                    now=time.monotonic()
                    if now-began>wall_budget or not fresh(True):
                        raise RuntimeError('Action wall timeout, clock stall or stale feedback')
                    error=max(abs(x) for x in state['control'][1])
                    max_error=max(max_error,error)
                    if error>c['controller_tracking_error_limit_rad']:
                        raise RuntimeError('Controller tracking error exceeds configured threshold')
                    if pending_valid is not None:
                        future,sent=pending_valid
                        if future.done():
                            if not future.result().valid:
                                raise RuntimeError('Runtime sampled actual state invalid')
                            validity_count+=1;pending_valid=None
                        elif now-sent>c['monitor_response_timeout_wall_s']:
                            raise RuntimeError('Runtime state check response timeout')
                    if pending_scene is not None:
                        future,sent=pending_scene
                        if future.done():
                            if scene_signature(future.result().scene)!=validated['source_scene_signature']:
                                raise RuntimeError('Planning scene changed during motion')
                            scene_count+=1;pending_scene=None
                        elif now-sent>c['monitor_response_timeout_wall_s']:
                            raise RuntimeError('Runtime scene check response timeout')
                    if pending_valid is None and now>=next_valid:
                        pending_valid=(valid_client.call_async(runtime_state_request(scene)),now)
                        next_valid=now+c['state_check_period_wall_s']
                    if pending_scene is None and now>=next_scene:
                        pending_scene=(scene_client.call_async(scene_request()),now)
                        next_scene=now+c['scene_check_period_wall_s']
                result=result_future.result()
                result_wall=time.monotonic()
                result_sim=state['clock']
                if pending_valid is not None:
                    final_valid=spin_until(pending_valid[0],c['monitor_response_timeout_wall_s'])
                    if not final_valid.valid:
                        raise RuntimeError('Last pending runtime state sample was invalid')
                    validity_count+=1
                if pending_scene is not None:
                    final_scene=spin_until(pending_scene[0],c['monitor_response_timeout_wall_s'])
                    if scene_signature(final_scene.scene)!=validated['source_scene_signature']:
                        raise RuntimeError('Last pending scene query detected a change')
                    scene_count+=1
                info.update({'action_status':result.status,'controller_error_code':result.result.error_code,
                             'action_elapsed_wall_s':result_wall-began,'client_observed_action_sim_s':result_sim-initial_clock,
                             'max_observed_controller_tracking_error_rad':max_error,'valid_runtime_state_checks':validity_count,
                             'unchanged_runtime_scene_checks':scene_count})
                if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code!=0:
                    raise RuntimeError('Execution action did not succeed')
                info['endpoint_and_stopped_confirmed']=settled(pts[-1].positions)
                info['endpoint_error_rad']=[state['joint'][1][n]-pts[-1].positions[i] for i,n in enumerate(names)]
                if not info['endpoint_and_stopped_confirmed']:
                    raise RuntimeError('Endpoint or stopped-state confirmation failed')
                if not query(GetStateValidity,'/check_state_validity',runtime_state_request(scene)).valid:
                    raise RuntimeError('Final actual state invalid')
                if scene_signature(query(GetPlanningScene,'/get_planning_scene',scene_request()).scene)!=validated['source_scene_signature']:
                    raise RuntimeError('Scene changed at phase completion')
                info['passed']=True
                info['status']='passed'
                print('PASSED '+label+'.',flush=True)
                flush_report()
            finally:
                state['log']=None;state['control_log']=None
                joint_file.close();control_file.close()
                node.destroy_client(valid_client);node.destroy_client(scene_client)
        report['passed']=True
        report['status']='preparation_only_passed'
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=type(exc).__name__+': '+str(exc)
        report['status']='failed_or_unknown_no_next_motion'
        if report['active_phase'] in report['phases']:
            report['phases'][report['active_phase']]['error']=report['error']
        failure_file=(root/'failure_stop_feedback.csv').open('w',newline='')
        state['log']=csv.writer(failure_file)
        state['log'].writerow(['receive_monotonic_wall_s','feedback_sim_s']+['q_'+n for n in names]+['v_'+n for n in names])
        try:
            if handle is None and send_future is not None:
                handle=spin_until(send_future,3)
            if handle is not None and handle.accepted:
                if result_future is None:
                    result_future=handle.get_result_async()
                if not result_future.done():
                    cancel=spin_until(handle.cancel_goal_async(),c['cancel_wait_wall_s'])
                    report['cancel_requested_goals']=len(cancel.goals_canceling)
                final=spin_until(result_future,c['cancel_result_wait_wall_s'])
                report['final_action_status']=final.status
        except (Exception,KeyboardInterrupt) as cancel_error:
            report['cancel_or_final_state_error']=type(cancel_error).__name__+': '+str(cancel_error)
        try:
            report['stopped_after_failure']=settled()
        except (Exception,KeyboardInterrupt):
            report['stopped_after_failure']=False
        print('FAILED: no further trajectory sent. If stop/action state is unresolved, Ctrl+C simulation terminal A. Keep logs.',flush=True)
    finally:
        state['log']=None
        if failure_file is not None:
            failure_file.close()
        report['finished_at']=datetime.now().astimezone().isoformat()
        flush_report()
        print(json.dumps(report,ensure_ascii=False,indent=2))
        print('SAVED:',root)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
