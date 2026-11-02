import argparse
import copy
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import PlanningScene, Constraints, JointConstraint, DisplayTrajectory
from moveit_msgs.srv import GetPlanningScene, GetStateValidity, GetMotionPlan
from rosidl_runtime_py.convert import message_to_ordereddict


def scene_signature(scene):
    data = message_to_ordereddict(scene)
    selected = {key: data[key] for key in ('world', 'allowed_collision_matrix', 'fixed_frame_transforms', 'link_padding', 'link_scale')}
    selected['attached'] = data['robot_state']['attached_collision_objects']
    return hashlib.sha256(json.dumps(selected, sort_keys=True).encode()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description='Plan and sample-check preparation and detour paths only. Never execute.')
    parser.add_argument('--case-dir', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    case = json.loads((args.case_dir/'candidate.json').read_text())
    search = json.loads((args.case_dir/'search_report.json').read_text())
    cfg_bytes = args.config.read_bytes()
    cfg = json.loads(cfg_bytes)
    if not search['passed'] or not search['start_valid'] or not search['goal_valid'] or search['box_collision_samples'] <= 0:
        raise RuntimeError('Candidate does not establish valid endpoints and a blocked direct path')
    if not 0 < cfg['max_interpolation_step_rad'] <= 0.01:
        raise RuntimeError('Invalid sampling resolution')
    names = case['joint_names']
    out = Path.home()/'ROS2/arm/P3/data/runs'/(datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_detour_plan_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True, exist_ok=False)
    (out/'config.json').write_bytes(cfg_bytes)
    (out/'candidate.json').write_bytes((args.case_dir/'candidate.json').read_bytes())
    (out/'plan_detour_case.py').write_bytes(Path(__file__).read_bytes())
    report = {'created_at':datetime.now().astimezone().isoformat(), 'case_dir':str(args.case_dir.resolve()),
              'config':cfg,'passed':False,'stage':'preflight','execution_requested':False,
              'limitations':['Virtual detour start is not the actual Gazebo state.',
                            'Preparation is planned separately and is not part of the detour task metrics.',
                            'Interpolation checks sample linear joint segments, not exact controller spline extrema or continuous collision.',
                            'Validity is limited to the current MoveIt scene/model; physical world equivalence was checked in the prior module.',
                            'Neither trajectory may be executed without renewed state/scene/controller checks.']}
    rclpy.init()
    node = rclpy.create_node('p3_plan_detour_case',enable_rosout=False)
    received = {'joint':None,'wall':None,'clock':None,'clock_wall':None,'clock_first':None}

    def on_joint(msg):
        received['joint'],received['wall']=msg,time.monotonic()

    def on_clock(msg):
        t=msg.clock.sec+msg.clock.nanosec/1e9
        if received['clock_first'] is None:
            received['clock_first']=t
        received['clock'],received['clock_wall']=t,time.monotonic()

    subs=[node.create_subscription(JointState,'/joint_states',on_joint,qos_profile_sensor_data),
          node.create_subscription(Clock,'/clock',on_clock,qos_profile_sensor_data)]

    def save(path,msg):
        path.write_text(json.dumps(message_to_ordereddict(msg),indent=2))

    def call(cls,endpoint,req,timeout=5):
        client=node.create_client(cls,endpoint)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Unavailable: '+endpoint)
            future=client.call_async(req)
            end=time.monotonic()+timeout
            while not future.done() and time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=0.05)
            if not future.done():
                raise RuntimeError('Application wall timeout at '+endpoint+'; remote request may still be running; do not retry automatically')
            return future.result()
        finally:
            node.destroy_client(client)

    def get_scene():
        req=GetPlanningScene.Request()
        req.components.components=1023
        return call(GetPlanningScene,'/get_planning_scene',req).scene

    try:
        old_scene=deserialize_message((args.case_dir/'planning_scene.cdr').read_bytes(),PlanningScene)
        scene=get_scene()
        signature=scene_signature(scene)
        if signature!=scene_signature(old_scene):
            raise RuntimeError('Collision scene differs from candidate snapshot; do not use the old case unreviewed')
        save(out/'planning_scene.json',scene)
        report['scene_signature']=signature
        req=GetParameters.Request()
        req.names=['robot_description','robot_description_semantic','use_sim_time',cfg['pipeline_id']+'.planning_plugin',cfg['pipeline_id']+'.request_adapters']
        params=call(GetParameters,'/move_group/get_parameters',req).values
        for index,filename in enumerate(('robot.urdf','robot.srdf')):
            data=params[index].string_value
            (out/filename).write_text(data)
            if data!=(args.case_dir/filename).read_text():
                raise RuntimeError('Model changed since candidate search: '+filename)
        if not params[2].bool_value or params[3].string_value!='ompl_interface/OMPLPlanner':
            raise RuntimeError('Expected simulation-time MoveIt OMPL configuration')
        report['request_adapters']=params[4].string_value
        model=ET.fromstring(params[0].string_value)
        model_joints={j.attrib['name']:j for j in model.findall('joint')}

        def state_at(q):
            state=copy.deepcopy(scene.robot_state)
            state.is_diff=False
            state.joint_state.name=list(names)
            state.joint_state.position=list(q)
            state.joint_state.velocity=[]
            state.joint_state.effort=[]
            state.joint_state.header.stamp.sec=0
            state.joint_state.header.stamp.nanosec=0
            return state

        def checked_state(q):
            if len(q)!=len(names) or not all(math.isfinite(v) for v in q):
                raise RuntimeError('Invalid joint vector')
            bounded=True
            for n,value in zip(names,q):
                j=model_joints[n]
                if j.attrib['type']=='continuous':
                    continue
                limit=j.find('limit')
                lo,hi=float(limit.attrib['lower']),float(limit.attrib['upper'])
                safety=j.find('safety_controller')
                if safety is not None:
                    lo=max(lo,float(safety.attrib.get('soft_lower_limit',lo)))
                    hi=min(hi,float(safety.attrib.get('soft_upper_limit',hi)))
                bounded=bounded and lo<=value<=hi
            req=GetStateValidity.Request()
            req.group_name=cfg['group']
            req.robot_state=state_at(q)
            res=call(GetStateValidity,'/check_state_validity',req)
            return bounded and res.valid, res

        report['stage']='current_feedback'
        end=time.monotonic()+8
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.05)
            if received['joint'] is not None and received['clock'] is not None and received['clock']>received['clock_first']:
                break
        msg=received['joint']
        if msg is None or received['clock'] is None or time.monotonic()-received['wall']>1 or time.monotonic()-received['clock_wall']>1:
            raise RuntimeError('Fresh joint/clock feedback unavailable')
        if abs(received['clock']-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9)>0.5:
            raise RuntimeError('Joint feedback and clock stamps differ too much')
        if len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name):
            raise RuntimeError('Malformed joint feedback')
        current=dict(zip(msg.name,msg.position))
        current_q=[current[n] for n in names]
        report['captured_actual_start_rad']=current_q
        report['paths']={}

        for label,q0,q1 in [('detour',case['start_rad'],case['goal_rad']),('preparation',current_q,case['start_rad'])]:
            report['stage']=label+'_planning'
            folder=out/label
            folder.mkdir()
            for endpoint,q in [('start',q0),('goal',q1)]:
                valid,result=checked_state(q)
                save(folder/(endpoint+'_validity.json'),result)
                if not valid:
                    raise RuntimeError(label+' '+endpoint+' is invalid')
            request=GetMotionPlan.Request()
            r=request.motion_plan_request
            r.group_name,r.pipeline_id,r.planner_id=cfg['group'],cfg['pipeline_id'],cfg['planner_id']
            r.start_state=state_at(q0)
            r.allowed_planning_time=float(cfg['planning_time_s'])
            r.num_planning_attempts=int(cfg['planning_attempts'])
            r.max_velocity_scaling_factor=float(cfg['velocity_scaling'])
            r.max_acceleration_scaling_factor=float(cfg['acceleration_scaling'])
            r.workspace_parameters.header.frame_id=case['model_frame']
            r.workspace_parameters.min_corner.x=r.workspace_parameters.min_corner.y=r.workspace_parameters.min_corner.z=-2.0
            r.workspace_parameters.max_corner.x=r.workspace_parameters.max_corner.y=r.workspace_parameters.max_corner.z=2.0
            goal=Constraints()
            for n,q in zip(names,q1):
                jc=JointConstraint()
                jc.joint_name,jc.position=n,float(q)
                jc.tolerance_above=jc.tolerance_below=float(cfg['joint_goal_tolerance_rad'])
                jc.weight=1.0
                goal.joint_constraints.append(jc)
            r.goal_constraints=[goal]
            save(folder/'planning_request.json',request)
            print('Planning '+label+' only; no movement.',flush=True)
            began=time.monotonic()
            response=call(GetMotionPlan,'/plan_kinematic_path',request,cfg['planning_time_s']+10)
            wall=time.monotonic()-began
            save(folder/'planning_response.json',response)
            result=response.motion_plan_response
            tr=result.trajectory.joint_trajectory
            summary={'error_code':result.error_code.val,'planning_service_wall_s':wall,'interface_planning_time_s':result.planning_time,'passed':False}
            report['paths'][label]=summary
            if result.error_code.val!=1 or len(tr.points)<2 or len(tr.joint_names)!=6 or set(tr.joint_names)!=set(names):
                raise RuntimeError(label+' planning unsuccessful or malformed trajectory')
            order=[list(tr.joint_names).index(n) for n in names]
            times=[p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in tr.points]
            if times[0]<0 or any(b<=a for a,b in zip(times,times[1:])):
                raise RuntimeError(label+' invalid trajectory time sequence')
            for p in tr.points:
                for field in ('positions','velocities','accelerations'):
                    values=list(getattr(p,field))
                    if len(values)!=6 or not all(math.isfinite(v) for v in values):
                        raise RuntimeError(label+' invalid '+field)
            positions=[[p.positions[i] for i in order] for p in tr.points]
            if max(abs(a-b) for a,b in zip(positions[0],q0))>0.001 or max(abs(a-b) for a,b in zip(positions[-1],q1))>cfg['endpoint_tolerance_rad']:
                raise RuntimeError(label+' output start/goal mismatch')
            samples=[positions[0]]
            for a,b in zip(positions,positions[1:]):
                count=max(1,math.ceil(max(abs(y-x) for x,y in zip(a,b))/cfg['max_interpolation_step_rad']))
                samples.extend([[x+(y-x)*k/count for x,y in zip(a,b)] for k in range(1,count+1)])
            if len(samples)>5000:
                raise RuntimeError('Sampling exceeds module limit; retain trajectory for review')
            report['stage']=label+'_path_validity'
            invalid=0
            with (folder/'sample_checks.jsonl').open('w') as stream:
                for index,q in enumerate(samples):
                    valid,response=checked_state(q)
                    invalid+=not valid
                    stream.write(json.dumps({'index':index,'q_rad':q,'valid':valid,'response':message_to_ordereddict(response)})+'\n')
                    if index%100==0:
                        print(label+' validity sample '+str(index)+'/'+str(len(samples)),flush=True)
            display=DisplayTrajectory()
            display.model_id=model.attrib['name']
            display.trajectory_start=result.trajectory_start
            display.trajectory=[result.trajectory]
            raw={'display_trajectory':message_to_ordereddict(display),'feedback_at_plan_receive':message_to_ordereddict(msg)}
            raw_file=folder/'trajectory_and_feedback.json'
            raw_file.write_text(json.dumps(raw,indent=2))
            summary.update({'point_count':len(tr.points),'planned_duration_s':times[-1],
                            'path_length_rad':sum(math.dist(a,b) for a,b in zip(positions,positions[1:])),
                            'sample_count':len(samples),'invalid_sample_count':invalid,
                            'raw_sha256':hashlib.sha256(raw_file.read_bytes()).hexdigest(),'passed':invalid==0})
            if invalid:
                raise RuntimeError(label+' path has invalid sampled states; never execute')
        report['stage']='scene_and_state_recheck'
        after=get_scene()
        save(out/'planning_scene_after.json',after)
        if scene_signature(after)!=signature:
            raise RuntimeError('Scene changed during planning/validation')
        if received['joint'] is None or time.monotonic()-received['wall']>1:
            raise RuntimeError('Feedback stale at final check')
        actual=dict(zip(received['joint'].name,received['joint'].position))
        if max(abs(actual[n]-current[n]) for n in names)>0.001:
            raise RuntimeError('Robot moved during plan-only module; preparation path no longer current')
        report['passed']=True
        report['stage']='two_paths_saved_and_sample_checked_not_executed'
    except Exception as exc:
        report['error']=type(exc).__name__+': '+str(exc)
    finally:
        for key,cmd in [('commit',['rev-parse','HEAD']),('dirty_status',['status','--short'])]:
            try:
                report[key]=subprocess.run(['git','-C',str(Path.home()/'ROS2'),*cmd],capture_output=True,text=True,timeout=5).stdout.strip()
            except Exception:
                report[key]='unavailable'
        (out/'detour_plan_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False,indent=2))
        print('SAVED:',out)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
