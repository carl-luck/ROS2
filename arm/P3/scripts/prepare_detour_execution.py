import argparse
import copy
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import uuid
import xml.etree.ElementTree as ET

import numpy as np


def coefficients(a,b):
    ta=a['time_from_start']['sec']+a['time_from_start']['nanosec']/1e9
    tb=b['time_from_start']['sec']+b['time_from_start']['nanosec']/1e9
    h=tb-ta
    if h<=0:
        raise RuntimeError('Non-increasing timestamps')
    q0,v0,a0=[np.asarray(a[k],dtype=float) for k in ('positions','velocities','accelerations')]
    q1,v1,a1=[np.asarray(b[k],dtype=float) for k in ('positions','velocities','accelerations')]
    c0,c1,c2=q0,h*v0,h*h*a0/2
    rhs=np.stack((q1-c0-c1-c2,h*v1-c1-2*c2,h*h*a1-2*c2))
    c3,c4,c5=np.linalg.solve(np.array([[1,1,1],[3,4,5],[6,12,20]],dtype=float),rhs)
    return ta,h,np.stack((c0,c1,c2,c3,c4,c5))


def sample_segment(a,b,period,max_step):
    ta,h,c=coefficients(a,b)
    extrema={0.0,1.0}
    for i in range(6):
        for poly in ([5*c[5,i],4*c[4,i],3*c[3,i],2*c[2,i],c[1,i]],
                     [20*c[5,i],12*c[4,i],6*c[3,i],2*c[2,i]],
                     [60*c[5,i],24*c[4,i],6*c[3,i]]):
            for root in np.roots(np.trim_zeros(np.asarray(poly),'f')) if np.any(poly) else []:
                if abs(root.imag)<1e-9 and 0<root.real<1:
                    extrema.add(float(root.real))

    def evaluate(s):
        q=sum(c[k]*s**k for k in range(6))
        v=sum(k*c[k]*s**(k-1) for k in range(1,6))/h
        acc=sum(k*(k-1)*c[k]*s**(k-2) for k in range(2,6))/(h*h)
        return q,v,acc

    max_velocity=max(float(np.max(np.abs(evaluate(s)[1]))) for s in extrema)
    count=max(1,math.ceil(h/period),math.ceil(h*max_velocity/max_step))
    locations=sorted(extrema | set(np.linspace(0,1,count+1)))
    return [(ta+h*s,*evaluate(s)) for s in locations]


def main():
    parser=argparse.ArgumentParser(description='Offline retiming, spline extrema checks and snapshot collision checks; never execute.')
    parser.add_argument('--plan-dir',required=True,type=Path)
    parser.add_argument('--config',required=True,type=Path)
    args=parser.parse_args()
    original_report=json.loads((args.plan_dir/'detour_plan_report.json').read_text())
    if not original_report['passed']:
        raise RuntimeError('Original two-path validation did not pass')
    config_bytes=args.config.read_bytes()
    cfg=json.loads(config_bytes)
    scale=float(cfg['time_scale'])
    if not math.isfinite(scale) or scale<1 or not 0<cfg['sample_period_sim_s']<=0.01 or not 0<cfg['max_joint_sample_step_rad']<=0.005:
        raise RuntimeError('Invalid retiming/sampling configuration')
    out=args.plan_dir/('execution_inputs_'+datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z')+'_'+uuid.uuid4().hex[:8])
    out.mkdir(exist_ok=False)
    (out/'config.json').write_bytes(config_bytes)
    (out/'prepare_detour_execution.py').write_bytes(Path(__file__).read_bytes())
    report={'created_at':datetime.now().astimezone().isoformat(),'source_plan_dir':str(args.plan_dir.resolve()),
            'source_scene_signature':original_report['scene_signature'],'config':cfg,'passed':False,
            'robot_motion_requested':False,'paths':{},
            'limitations':['Uniform time scaling preserves the saved quintic geometric curve but changes execution timing.',
                           'Collision checks are discrete samples of the controller-style quintic interpolation in the saved scene; not continuous collision guarantees.',
                           'Polynomial extrema checks are numerical, not formal interval proofs.',
                           'Live Gazebo/MoveIt scene, controller and current-start checks remain required before execution.']}
    try:
        case_dir=Path(original_report['case_dir'])
        model=ET.fromstring((args.plan_dir/'robot.urdf').read_text())
        joints={j.attrib['name']:j for j in model.findall('joint')}
        for label in ('preparation','detour'):
            folder=out/label
            folder.mkdir()
            source=args.plan_dir/label/'trajectory_and_feedback.json'
            source_bytes=source.read_bytes()
            if hashlib.sha256(source_bytes).hexdigest()!=original_report['paths'][label]['raw_sha256']:
                raise RuntimeError('Source trajectory hash mismatch: '+label)
            raw=json.loads(source_bytes)
            trajectory=raw['display_trajectory']['trajectory'][0]['joint_trajectory']
            names=trajectory['joint_names']
            expected=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
            if names!=expected:
                raise RuntimeError('Unexpected joint order')
            points=trajectory['points']
            for point in points:
                for field in ('positions','velocities','accelerations'):
                    if len(point[field])!=6 or not all(math.isfinite(v) for v in point[field]):
                        raise RuntimeError('Malformed saved trajectory')
                stamp=point['time_from_start']
                ns=round((stamp['sec']*1000000000+stamp['nanosec'])*scale)
                stamp['sec'],stamp['nanosec']=divmod(ns,1000000000)
                point['velocities']=[v/scale for v in point['velocities']]
                point['accelerations']=[v/(scale*scale) for v in point['accelerations']]
            raw_path=folder/'trajectory_and_feedback.json'
            raw_path.write_text(json.dumps(raw,indent=2))
            for filename in ('robot.urdf','robot.srdf'):
                shutil.copyfile(args.plan_dir/filename,folder/filename)
            shutil.copyfile(case_dir/'planning_scene.cdr',folder/'planning_scene.cdr')
            # The loaded configuration version has 5 rad/s^2 for all six joints.
            # Record the source YAML instead of presenting this as a hardware specification.
            acceleration_file=Path('/opt/ros/humble/share/ur_moveit_config/config/joint_limits.yaml')
            import yaml
            limits_config=yaml.safe_load(acceleration_file.read_text())['joint_limits']
            shutil.copyfile(acceleration_file,folder/'moveit_joint_limits.yaml')
            vlimits=np.array([float(joints[n].find('limit').attrib['velocity'])*cfg['velocity_scaling'] for n in names])
            alimits=np.array([float(limits_config[n]['max_acceleration'])*cfg['acceleration_scaling'] for n in names])
            bounds=[]
            for n in names:
                j=joints[n]
                lim=j.find('limit')
                lo,hi=float(lim.attrib['lower']),float(lim.attrib['upper'])
                soft=j.find('safety_controller')
                if soft is not None:
                    lo=max(lo,float(soft.attrib.get('soft_lower_limit',lo)))
                    hi=min(hi,float(soft.attrib.get('soft_upper_limit',hi)))
                bounds.append((lo,hi))
            maxv=np.zeros(6);maxa=np.zeros(6);count=0;numeric_ok=True
            with (folder/'spline_samples.csv').open('w',newline='') as stream:
                writer=csv.writer(stream,lineterminator='\n')
                writer.writerow(['time_from_start_s']+names)
                for a,b in zip(points,points[1:]):
                    for t,q,v,acc in sample_segment(a,b,cfg['sample_period_sim_s'],cfg['max_joint_sample_step_rad']):
                        maxv=np.maximum(maxv,np.abs(v));maxa=np.maximum(maxa,np.abs(acc))
                        numeric_ok=numeric_ok and all(lo-1e-9<=x<=hi+1e-9 for (lo,hi),x in zip(bounds,q))
                        writer.writerow([t]+q.tolist());count+=1
            numeric_ok=bool(numeric_ok and np.all(maxv<=vlimits+cfg['numerical_tolerance']) and np.all(maxa<=alimits+cfg['numerical_tolerance']))
            summary={'source_sha256':hashlib.sha256(source_bytes).hexdigest(),'retimed_sha256':hashlib.sha256(raw_path.read_bytes()).hexdigest(),
                     'sample_count':count,'max_velocity_rad_s':maxv.tolist(),'max_acceleration_rad_s2':maxa.tolist(),
                     'velocity_limits_rad_s':vlimits.tolist(),'acceleration_limits_rad_s2':alimits.tolist(),
                     'numerical_checks_passed':numeric_ok,'passed':False,
                     'retimed_duration_sim_s':points[-1]['time_from_start']['sec']+points[-1]['time_from_start']['nanosec']/1e9}
            report['paths'][label]=summary
            if not numeric_ok:
                raise RuntimeError(label+' interpolated limits check failed; retain evidence')
            print('Checking '+label+' spline samples in the saved collision scene...',flush=True)
            with (folder/'collision_console.txt').open('w') as log:
                result=subprocess.run(['ros2','run','ur5e_planning_eval','check_spline_samples','--ros-args','-p','input_dir:='+str(folder)],
                                      stdout=log,stderr=subprocess.STDOUT,timeout=120)
            collision=json.loads((folder/'spline_collision_report.json').read_text())
            summary['collision_report']=collision
            if result.returncode!=0 or not collision['passed']:
                raise RuntimeError(label+' spline collision validation did not pass')
            summary['passed']=True
        report['passed']=True
    except Exception as exc:
        report['error']=type(exc).__name__+': '+str(exc)
    finally:
        (out/'execution_inputs_report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
        print('SAVED:',out)
    return 0 if report['passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
