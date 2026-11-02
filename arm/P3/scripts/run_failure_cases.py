import argparse
import copy
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time
import uuid

import rclpy
from moveit_msgs.msg import Constraints,JointConstraint,RobotTrajectory
from moveit_msgs.srv import GetPlanningScene,GetStateValidity,GetPositionIK,GetPositionFK,GetMotionPlan
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from plan_detour_case import scene_signature


def result_decision(error_code,trajectory):
    if error_code!=1:
        return 'reject_error_code'
    if not trajectory.joint_trajectory.points:
        return 'reject_empty_trajectory'
    return 'requires_further_validation_never_auto_execute'


def main():
    parser=argparse.ArgumentParser(description='Non-motion failure cases. Never creates an action client or executes trajectories.')
    parser.add_argument('--plan-dir',required=True,type=Path)
    parser.add_argument('--config',required=True,type=Path)
    args=parser.parse_args()
    plan_report=json.loads((args.plan_dir/'detour_plan_report.json').read_text())
    if not plan_report['passed']:
        raise RuntimeError('Expected passed detour plan evidence')
    case_dir=Path(plan_report['case_dir'])
    case=json.loads((case_dir/'candidate.json').read_text())
    config_bytes=args.config.read_bytes()
    c=json.loads(config_bytes)
    out=Path.home()/'ROS2/arm/P3/data/runs'/(datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_failure_cases_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True,exist_ok=False)
    (out/'config.json').write_bytes(config_bytes)
    for name in ('run_failure_cases.py','plan_detour_case.py'):
        (out/name).write_bytes(Path(__file__).with_name(name).read_bytes())
    report={'started_at':datetime.now().astimezone().isoformat(),'config':c,'passed':False,'tests':[],
            'config_sha256':hashlib.sha256(config_bytes).hexdigest(),'source_plan_dir':str(args.plan_dir.resolve()),
            'execution_requests_sent':0,'action_client_created':False,
            'limits':['Uses virtual test start states; actual Gazebo robot is not reset or moved.',
                      'Tiny-budget outcome is observed, not guaranteed to fail.',
                      'Injected errors test local result handling, not a genuine OMPL timeout or full application cancellation path.',
                      'IK failure alone does not prove general global unreachability; this far target is a configured negative case.']}
    rclpy.init()
    node=rclpy.create_node('p3_failure_cases',enable_rosout=False)

    def save(name,obj):
        (out/name).write_text(json.dumps(message_to_ordereddict(obj),indent=2))

    def call(cls,endpoint,req,timeout=6):
        client=node.create_client(cls,endpoint)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Service unavailable: '+endpoint)
            future=client.call_async(req)
            end=time.monotonic()+timeout
            while not future.done() and time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=0.05)
            if not future.done():
                raise RuntimeError('Application timeout: '+endpoint+'; remote request may still be running; no automatic retry')
            return future.result()
        finally:
            node.destroy_client(client)

    def get_scene():
        req=GetPlanningScene.Request()
        req.components.components=1023
        return call(GetPlanningScene,'/get_planning_scene',req).scene

    try:
        scene=get_scene()
        if scene_signature(scene)!=plan_report['scene_signature']:
            raise RuntimeError('Scene changed since the fixed case; stop and review')
        save('planning_scene.json',scene)
        params_req=GetParameters.Request()
        params_req.names=['robot_description','robot_description_semantic']
        params=call(GetParameters,'/move_group/get_parameters',params_req).values
        for i,name in enumerate(('robot.urdf','robot.srdf')):
            if params[i].string_value!=(args.plan_dir/name).read_text():
                raise RuntimeError('Model differs from the fixed case')
            (out/name).write_text(params[i].string_value)

        def state_at(q):
            state=copy.deepcopy(scene.robot_state)
            state.is_diff=False
            state.joint_state.name=case['joint_names']
            state.joint_state.position=list(q)
            state.joint_state.velocity=[]
            state.joint_state.effort=[]
            state.joint_state.header.stamp.sec=0
            state.joint_state.header.stamp.nanosec=0
            return state

        print('CASE 1: far pose IK rejection; no motion.',flush=True)
        fk_req=GetPositionFK.Request()
        fk_req.header.frame_id=c['reference_frame']
        fk_req.fk_link_names=[c['tip_link']]
        fk_req.robot_state=state_at(case['start_rad'])
        fk=call(GetPositionFK,'/compute_fk',fk_req)
        if fk.error_code.val!=1 or len(fk.pose_stamped)!=1:
            raise RuntimeError('Cannot construct initial test pose')
        target=copy.deepcopy(fk.pose_stamped[0])
        target.header.stamp.sec=0;target.header.stamp.nanosec=0
        target.pose.position.x,target.pose.position.y,target.pose.position.z=map(float,c['unreachable_position_m'])
        ik_req=GetPositionIK.Request()
        r=ik_req.ik_request
        r.group_name,r.ik_link_name=c['group'],c['tip_link']
        r.robot_state=state_at(case['start_rad'])
        r.pose_stamped=target
        r.avoid_collisions=False
        r.timeout.sec=int(c['ik_timeout_s'])
        r.timeout.nanosec=int(c['ik_timeout_s']%1*1e9)
        save('unreachable_ik_request.json',ik_req)
        began=time.monotonic()
        ik=call(GetPositionIK,'/compute_ik',ik_req)
        elapsed=time.monotonic()-began
        save('unreachable_ik_response.json',ik)
        report['tests'].append({'name':'far_pose_ik','category':'expected_invalid_goal','error_code':ik.error_code.val,
                                'wall_s':elapsed,'passed':ik.error_code.val==-31,'planning_request_sent':False,'execution_request_sent':False})

        print('CASE 2: collision target from direct-interpolation evidence; no motion.',flush=True)
        with (case_dir/'direct_interpolation.csv').open() as stream:
            hits=[row for row in csv.DictReader(stream) if row['box_collision']=='1']
        if not hits:
            raise RuntimeError('Missing original direct-path collision evidence')
        selected=min(hits,key=lambda row:abs(float(row['alpha'])-.5))
        q=[float(selected[n+'_rad']) for n in case['joint_names']]
        check=GetStateValidity.Request()
        check.group_name=c['group']
        check.robot_state=state_at(q)
        save('collision_target_request.json',check)
        validity=call(GetStateValidity,'/check_state_validity',check)
        save('collision_target_response.json',validity)
        contacts=[(x.contact_body_1,x.contact_body_2) for x in validity.contacts]
        box_hit=any(case['obstacle_id'] in pair for pair in contacts)
        report['tests'].append({'name':'collision_target','category':'expected_invalid_goal','valid':validity.valid,
                                'contact_pairs':contacts,'passed':not validity.valid and box_hit,
                                'planning_request_sent':False,'execution_request_sent':False})

        print('CASE 3: genuine tiny-budget planning request; success is possible and will be recorded.',flush=True)
        request=GetMotionPlan.Request()
        r=request.motion_plan_request
        r.group_name,r.pipeline_id,r.planner_id=c['group'],c['pipeline_id'],c['planner_id']
        r.start_state=state_at(case['start_rad'])
        r.allowed_planning_time=float(c['tiny_planning_budget_s'])
        r.num_planning_attempts=int(c['planning_attempts'])
        r.max_velocity_scaling_factor=float(c['velocity_scaling'])
        r.max_acceleration_scaling_factor=float(c['acceleration_scaling'])
        r.workspace_parameters.header.frame_id=c['reference_frame']
        r.workspace_parameters.min_corner.x=r.workspace_parameters.min_corner.y=r.workspace_parameters.min_corner.z=-2.0
        r.workspace_parameters.max_corner.x=r.workspace_parameters.max_corner.y=r.workspace_parameters.max_corner.z=2.0
        constraints=Constraints()
        for n,q in zip(case['joint_names'],case['goal_rad']):
            jc=JointConstraint()
            jc.joint_name,jc.position=n,float(q)
            jc.tolerance_above=jc.tolerance_below=float(c['joint_goal_tolerance_rad'])
            jc.weight=1.0
            constraints.joint_constraints.append(jc)
        r.goal_constraints=[constraints]
        save('tiny_budget_request.json',request)
        began=time.monotonic()
        response=call(GetMotionPlan,'/plan_kinematic_path',request,10)
        elapsed=time.monotonic()-began
        save('tiny_budget_response.json',response)
        result=response.motion_plan_response
        decision=result_decision(result.error_code.val,result.trajectory)
        report['tests'].append({'name':'tiny_budget','category':'controlled_budget_observation',
                                'requested_budget_s':c['tiny_planning_budget_s'],'error_code':result.error_code.val,
                                'wall_s':elapsed,'interface_planning_time_s':result.planning_time,
                                'decision':decision,'observed_planning_success':result.error_code.val==1,
                                'passed':True,'execution_request_sent':False,
                                'note':'Observation only; a success is not mislabeled as timeout. Not included in normal success-rate statistics.'})

        print('CASE 4: synthetic result-handling checks; no planner failure is fabricated.',flush=True)
        saved=json.loads((args.plan_dir/'detour/trajectory_and_feedback.json').read_text())
        stale=RobotTrajectory()
        set_message_fields(stale,saved['display_trajectory']['trajectory'][0])
        if not stale.joint_trajectory.points:
            raise RuntimeError('Expected nonempty saved trajectory for fault injection')
        for label,code,tr in [('injected_error_with_stale_nonempty_path',-6,stale),
                              ('injected_success_with_empty_path',1,RobotTrajectory())]:
            decision=result_decision(code,tr)
            report['tests'].append({'name':label,'category':'synthetic_fault_injection','injected_error_code':code,
                                    'decision':decision,'passed':decision.startswith('reject_'),
                                    'planning_request_sent':False,'execution_request_sent':False})
        after=get_scene()
        if scene_signature(after)!=plan_report['scene_signature']:
            raise RuntimeError('Scene changed during tests')
        report['passed']=all(t['passed'] for t in report['tests'])
    except Exception as exc:
        report['error']=type(exc).__name__+': '+str(exc)
    finally:
        report['finished_at']=datetime.now().astimezone().isoformat()
        (out/'failure_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        with (out/'failure_trials.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=['name','category','passed','error_code','wall_s','decision','execution_request_sent'])
            writer.writeheader()
            for row in report['tests']:
                writer.writerow({k:row.get(k,'') for k in writer.fieldnames})
        print(json.dumps(report,ensure_ascii=False,indent=2))
        print('SAVED:',out)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
