import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from google.protobuf.text_encoding import CUnescape
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from shape_msgs.msg import SolidPrimitive
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped


def parse_proto(text):
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|[{}:]|[^\s{}:]+', text)
    index = 0

    def block(nested=False):
        nonlocal index
        result = {}
        while index < len(tokens):
            if tokens[index] == '}':
                if not nested:
                    raise ValueError('Unexpected closing brace')
                index += 1
                return result
            key = tokens[index]
            index += 1
            if index < len(tokens) and tokens[index] == ':':
                index += 1
            if index >= len(tokens):
                raise ValueError('Truncated protobuf text')
            value = tokens[index]
            index += 1
            if value == '{':
                value = block(True)
            elif value.startswith('"'):
                value = CUnescape(value[1:-1]).decode('utf-8')
            else:
                try:
                    value = float(value)
                except ValueError:
                    pass
            result.setdefault(key, []).append(value)
        if nested:
            raise ValueError('Unclosed protobuf block')
        return result

    return block()


def one(obj, key, default=None):
    return obj.get(key, [default])[0]


def pose_values(obj):
    p = one(obj, 'position', {})
    q = one(obj, 'orientation', {})
    return [one(p,k,0.0) for k in ('x','y','z')], [one(q,k,0.0) for k in ('x','y','z','w')]


def near_pose(a, b):
    pa, qa = a
    pb, qb = b
    if abs(math.sqrt(sum(x*x for x in qa))-1) > 1e-6 or abs(math.sqrt(sum(x*x for x in qb))-1) > 1e-6:
        return False
    return math.dist(pa,pb) < 1e-6 and min(math.dist(qa,qb),math.dist(qa,[-x for x in qb])) < 1e-6


def composed_primitive_pose(obj):
    local=obj.primitive_poses[0]
    parent=getattr(obj,'pose',None)
    lp=[local.position.x,local.position.y,local.position.z]
    lq=[local.orientation.x,local.orientation.y,local.orientation.z,local.orientation.w]
    if parent is None:
        return lp,lq
    p=[parent.position.x,parent.position.y,parent.position.z]
    q=[parent.orientation.x,parent.orientation.y,parent.orientation.z,parent.orientation.w]
    if (not all(math.isfinite(v) for v in p+q+lp+lq) or
            abs(math.sqrt(sum(v*v for v in q))-1)>1e-6 or
            abs(math.sqrt(sum(v*v for v in lq))-1)>1e-6):
        raise RuntimeError('Invalid object or primitive quaternion/position')
    x,y,z,w=q
    a,b,c,d=lq
    combined=[w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a,
              w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c]
    vx,vy,vz=lp
    tx,ty,tz=2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    rotated=[vx+w*tx+y*tz-z*ty,vy+w*ty+z*tx-x*tz,vz+w*tz+x*ty-y*tx]
    return [a+b for a,b in zip(p,rotated)],combined


def main():
    parser = argparse.ArgumentParser(description='Manual single-box scene loading or read-only verification. No robot motion.')
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--mode', choices=['apply','verify'], required=True)
    args = parser.parse_args()
    config_bytes = args.config.read_bytes()
    c = json.loads(config_bytes)
    size, center, quat = c['box_full_size_m'], c['center_in_gazebo_world_m'], c['orientation_xyzw']
    if (len(size)!=3 or len(center)!=3 or len(quat)!=4 or
            not all(math.isfinite(x) for x in size+center+quat) or min(size)<=0 or
            quat != [0.0,0.0,0.0,1.0] or c['static'] is not True):
        raise RuntimeError('This first scene supports a finite axis-aligned static box only')
    if c['world_mapping'] != 'identity_explicit_origin_installation':
        raise RuntimeError('Unsupported world mapping; do not infer it from identical frame names')
    object_id = c['object_id']
    if not re.fullmatch(r'p3_[a-z0-9_]+', object_id):
        raise RuntimeError('Expected a dedicated p3_ object name')
    out = Path.home() / 'ROS2/arm/P3/data/runs' / (datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_box_'+args.mode+'_'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True, exist_ok=False)
    (out/'config.json').write_bytes(config_bytes)
    (out/'manage_box_scene.py').write_bytes(Path(__file__).read_bytes())
    report = {'created_at':datetime.now().astimezone().isoformat(), 'mode':args.mode,
              'config_sha256':hashlib.sha256(config_bytes).hexdigest(), 'passed':False,
              'robot_motion_requested':False, 'stage':'preflight',
              'limitations':['World mapping is an explicitly configured origin installation, checked against UR model pose, URDF and TF; not physical calibration.',
                            'Gazebo scene service confirms visual geometry and model pose. Collision geometry/static settings are recorded in the creation SDF, not independently measured by this service.',
                            'Query success does not prove rendering on screen; screenshots are separate evidence.',
                            'This distant box is a scene integration case, not yet a necessary-detour case.',
                            'No rollback or deletion on partial failure. Inspect both states before retrying.']}
    rclpy.init()
    node=rclpy.create_node('p3_manage_box_scene',enable_rosout=False)
    buffer=Buffer(node=node)
    listener=TransformListener(buffer,node,spin_thread=False)

    def save(name, value):
        (out/name).write_text(json.dumps(message_to_ordereddict(value),indent=2))

    def call(cls, endpoint, req):
        client=node.create_client(cls,endpoint)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Unavailable: '+endpoint)
            future=client.call_async(req)
            end=time.monotonic()+5
            while not future.done() and time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=0.05)
            if not future.done():
                raise RuntimeError('Application timeout; remote operation state may be unknown: '+endpoint)
            return future.result()
        finally:
            node.destroy_client(client)

    def gazebo(service, reqtype, reptype, request, label):
        result=subprocess.run(['ign','service','-s','/world/'+c['gazebo_world']+'/'+service,
                               '--reqtype',reqtype,'--reptype',reptype,'--timeout','4000','--req',request],
                              capture_output=True,text=True,timeout=7)
        (out/(label+'.txt')).write_text(result.stdout+'\nSTDERR:\n'+result.stderr)
        if result.returncode!=0 or not result.stdout.strip():
            raise RuntimeError('Gazebo service failed: '+service)
        return parse_proto(result.stdout)

    def scene(label):
        req=GetPlanningScene.Request()
        req.components.components=1023
        value=call(GetPlanningScene,'/get_planning_scene',req).scene
        save(label+'.json',value)
        return value

    def transform(target, source):
        end=time.monotonic()+8
        while not buffer.can_transform(target,source,Time()) and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.05)
        return buffer.lookup_transform(target,source,Time())

    def find_model(gs, name):
        models=[m for m in gs.get('model',[]) if one(m,'name')==name]
        if len(models)>1:
            raise RuntimeError('Duplicate Gazebo model name: '+name)
        return models[0] if models else None

    def check_gazebo_model(model):
        if model is None or not near_pose(pose_values(one(model,'pose',{})),(center,quat)):
            raise RuntimeError('Gazebo box missing or model center/orientation differs')
        links=model.get('link',[])
        if len(links)!=1 or not near_pose(pose_values(one(links[0],'pose',{})),([0.0]*3,quat)):
            raise RuntimeError('Unexpected box link structure or offset')
        visuals=links[0].get('visual',[])
        if len(visuals)!=1 or not near_pose(pose_values(one(visuals[0],'pose',{})),([0.0]*3,quat)):
            raise RuntimeError('Unexpected box visual structure or offset')
        geometry=one(visuals[0],'geometry',{})
        box=one(geometry,'box',{})
        dims=one(box,'size',{})
        values=[one(dims,k,0.0) for k in ('x','y','z')]
        if math.dist(values,size)>1e-6:
            raise RuntimeError('Gazebo visual full size differs')
        report['gazebo_actual']={'center_m':pose_values(one(model,'pose',{}))[0],'full_size_m':values,'orientation_xyzw':quat}

    def check_moveit(obj, wanted):
        if obj.header.frame_id!=wanted.header.frame_id or len(obj.primitives)!=1 or len(obj.primitive_poses)!=1:
            raise RuntimeError('MoveIt object frame or primitive structure differs')
        if obj.meshes or obj.planes:
            raise RuntimeError('Unexpected additional geometry in the box object')
        primitive=obj.primitives[0]
        if primitive.type!=SolidPrimitive.BOX or len(primitive.dimensions)!=3 or math.dist(primitive.dimensions,size)>1e-6:
            raise RuntimeError('MoveIt box full size differs')
        actual_pose=composed_primitive_pose(obj)
        expected_pose=composed_primitive_pose(wanted)
        if not near_pose(actual_pose,expected_pose):
            raise RuntimeError('MoveIt box center/orientation differs')
        report['moveit_composed_primitive_pose']={'center_m':actual_pose[0],'orientation_xyzw':actual_pose[1]}

    try:
        req=GetParameters.Request()
        req.names=['robot_description']
        urdf=call(GetParameters,'/move_group/get_parameters',req).values[0].string_value
        (out/'robot.urdf').write_text(urdf)
        root=ET.fromstring(urdf)
        anchor=[j for j in root.findall('joint') if j.find('parent').attrib['link']==c['tf_world_frame'] and j.find('child').attrib['link']==c['robot_anchor_frame']]
        if len(anchor)!=1 or anchor[0].attrib['type']!='fixed':
            raise RuntimeError('Expected explicit fixed world-to-base anchor in current URDF')
        origin=anchor[0].find('origin')
        if origin is not None and any(abs(float(x))>1e-9 for key in ('xyz','rpy') for x in origin.attrib.get(key,'0 0 0').split()):
            raise RuntimeError('Nonidentity URDF anchor; needs a new mapping configuration')
        gs=gazebo('scene/info','ignition.msgs.Empty','ignition.msgs.Scene','','gazebo_before')
        robot=find_model(gs,c['gazebo_robot_model'])
        if robot is None or not near_pose(pose_values(one(robot,'pose',{})),([0.0]*3,quat)):
            raise RuntimeError('Gazebo robot model not at configured origin installation')
        anchor_tf=transform(c['tf_world_frame'],c['robot_anchor_frame'])
        save('anchor_tf.json',anchor_tf)
        t,q=anchor_tf.transform.translation,anchor_tf.transform.rotation
        if not near_pose(([t.x,t.y,t.z],[q.x,q.y,q.z,q.w]),([0.0]*3,quat)):
            raise RuntimeError('TF anchor not identity; world mapping unsupported')
        pose=PoseStamped()
        pose.header.frame_id=c['tf_world_frame']
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,center)
        pose.pose.orientation.w=1.0
        tf=transform(c['planning_reference_frame'],c['tf_world_frame'])
        wanted_pose=do_transform_pose_stamped(pose,tf)
        save('box_frame_transform.json',tf)
        save('box_planning_pose.json',wanted_pose)
        obj=CollisionObject()
        obj.id=object_id
        obj.header.frame_id=c['planning_reference_frame']
        obj.operation=CollisionObject.ADD
        if hasattr(obj,'pose'):
            obj.pose.orientation.w=1.0
        primitive=SolidPrimitive()
        primitive.type=SolidPrimitive.BOX
        primitive.dimensions=list(map(float,size))
        obj.primitives=[primitive]
        obj.primitive_poses=[wanted_pose.pose]
        before=scene('moveit_before')
        existing=[o for o in before.world.collision_objects if o.id==object_id]
        if len(existing)>1:
            raise RuntimeError('Duplicate MoveIt object id')
        if existing:
            check_moveit(existing[0],obj)
        gm=find_model(gs,object_id)
        if gm is not None:
            check_gazebo_model(gm)
        report['stage']='gazebo_load'
        sx,sy,sz=size
        x,y,z=center
        sdf=(f'<sdf version="1.7"><model name="{object_id}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>'
             f'<link name="box_link"><collision name="box_collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
             f'<visual name="box_visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
             '<material><ambient>0.1 0.35 0.8 1</ambient><diffuse>0.1 0.35 0.8 1</diffuse></material></visual></link></model></sdf>')
        ET.fromstring(sdf)
        (out/'box.sdf').write_text(sdf)
        if gm is None:
            if args.mode!='apply':
                raise RuntimeError('Gazebo box missing; verify mode makes no changes')
            response=gazebo('create','ignition.msgs.EntityFactory','ignition.msgs.Boolean',
                            'sdf: '+json.dumps(sdf)+' allow_renaming: false','gazebo_create_response')
            if one(response,'data')!='true':
                raise RuntimeError('Gazebo did not acknowledge creation; inspect both states before retrying')
        report['stage']='moveit_load'
        if not existing:
            if args.mode!='apply':
                raise RuntimeError('MoveIt box missing; verify mode makes no changes')
            req=ApplyPlanningScene.Request()
            req.scene=PlanningScene()
            req.scene.is_diff=True
            req.scene.robot_state.is_diff=True
            req.scene.world.collision_objects=[obj]
            save('apply_scene_request.json',req)
            if not call(ApplyPlanningScene,'/apply_planning_scene',req).success:
                raise RuntimeError('MoveIt did not acknowledge scene apply')
        report['stage']='query_both_states'
        deadline=time.monotonic()+10
        confirmed=False
        while time.monotonic()<deadline:
            after_gz=gazebo('scene/info','ignition.msgs.Empty','ignition.msgs.Scene','','gazebo_after')
            after=scene('moveit_after')
            candidate=find_model(after_gz,object_id)
            objects=[o for o in after.world.collision_objects if o.id==object_id]
            if candidate is not None and len(objects)==1:
                check_gazebo_model(candidate)
                check_moveit(objects[0],obj)
                confirmed=True
                break
            rclpy.spin_once(node,timeout_sec=0.1)
        if not confirmed:
            raise RuntimeError('Box not confirmed in both states before wall deadline')
        report['moveit_actual']=message_to_ordereddict(objects[0])
        report['stage']='current_state_validity'
        check=GetStateValidity.Request()
        check.group_name='ur_manipulator'
        check.robot_state=after.robot_state
        check.robot_state.is_diff=False
        valid=call(GetStateValidity,'/check_state_validity',check)
        save('current_state_validity.json',valid)
        report['current_scene_state_valid']=valid.valid
        if not valid.valid:
            raise RuntimeError('Current scene robot state invalid after loading; do not move')
        report['passed']=True
        report['stage']='both_states_confirmed_no_motion'
    except Exception as exc:
        report['error']=type(exc).__name__+': '+str(exc)
    finally:
        (out/'scene_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False,indent=2))
        print('SAVED:',out)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
