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
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.clock import ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import Constraints, JointConstraint, PlanningSceneComponents, DisplayTrajectory
from moveit_msgs.srv import GetPlanningScene, GetStateValidity, GetPositionIK, GetPositionFK, GetMotionPlan
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped


def main():
    parser = argparse.ArgumentParser(description='Plan only: a current-tool-relative pose goal via IK and joint planning.')
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    config_bytes = args.config.read_bytes()
    cfg = json.loads(config_bytes)
    offset = cfg['offset_in_tool_m']
    if len(offset) != 3 or not all(math.isfinite(x) for x in offset) or math.sqrt(sum(x*x for x in offset)) > 0.01:
        raise RuntimeError('Introductory pose module requires a finite offset <= 0.01 m')
    out = Path.home() / 'ROS2/arm/P3/data/runs' / (datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z') + '_pose_plan_' + uuid.uuid4().hex[:8])
    out.mkdir(parents=True, exist_ok=False)
    (out / 'config.json').write_bytes(config_bytes)
    (out / 'plan_pose_goal.py').write_bytes(Path(__file__).read_bytes())
    report = {'created_at': datetime.now().astimezone().isoformat(), 'config': cfg, 'passed': False,
              'execution_requested': False, 'stage': 'initialize', 'config_sha256': hashlib.sha256(config_bytes).hexdigest(),
              'method': 'Configured pose construction -> IK using current state seed without collision filtering -> bounds and state validity -> joint-goal planning -> endpoint FK check.',
              'limitations': ['Pose endpoint task; intermediate Cartesian motion is not constrained to a straight line.',
                             'IK failure means this solver/request failed, not proof of global unreachability.',
                             'Planning reference frame is explicit and checked against the model root; no MoveGroupInterface getPlanningFrame call.',
                             'No execution or TF publication. Further saved-path validation is required.',
                             'Development single trial, not a frozen batch benchmark.']}
    rclpy.init()
    node = rclpy.create_node('p3_plan_pose_goal', enable_rosout=False, parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    state = {'joint': None, 'received': None, 'clock': None, 'clock_received': None, 'first_clock': None}

    def joints(m):
        state['joint'], state['received'] = m, time.monotonic()

    def clocks(m):
        value = m.clock.sec + m.clock.nanosec / 1e9
        if state['first_clock'] is None:
            state['first_clock'] = value
        state['clock'], state['clock_received'] = value, time.monotonic()

    subs = [node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data),
            node.create_subscription(Clock, '/clock', clocks, qos_profile_sensor_data)]

    def save(name, obj):
        (out / name).write_text(json.dumps(message_to_ordereddict(obj), indent=2), encoding='utf-8')

    def call(cls, endpoint, request, timeout=5.0):
        client = node.create_client(cls, endpoint)
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('Service unavailable: ' + endpoint)
            future = client.call_async(request)
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done():
                raise RuntimeError('Application wall timeout at ' + endpoint + '; remote request may still be running. Do not issue another request.')
            return future.result()
        finally:
            node.destroy_client(client)

    def fresh():
        if state['joint'] is None or state['clock'] is None:
            return False
        now = time.monotonic()
        stamp = state['joint'].header.stamp
        return (now-state['received'] < 1 and now-state['clock_received'] < 1 and
                state['clock'] > state['first_clock'] and abs(state['clock']-stamp.sec-stamp.nanosec/1e9) < 0.5)

    try:
        report['stage'] = 'model_and_pipeline'
        req = GetParameters.Request()
        req.names = ['robot_description', 'robot_description_semantic', 'use_sim_time',
                     cfg['pipeline_id'] + '.planning_plugin', cfg['pipeline_id'] + '.request_adapters']
        params = call(GetParameters, '/move_group/get_parameters', req)
        urdf, srdf = params.values[0].string_value, params.values[1].string_value
        if not params.values[2].bool_value or params.values[3].string_value != 'ompl_interface/OMPLPlanner':
            raise RuntimeError('Expected simulation time and OMPL pipeline are not active')
        report['planning_plugin'] = params.values[3].string_value
        report['request_adapters'] = params.values[4].string_value
        (out / 'robot.urdf').write_text(urdf)
        (out / 'robot.srdf').write_text(srdf)
        model = ET.fromstring(urdf)
        children = {j.find('child').attrib['link'] for j in model.findall('joint')}
        roots = [l.attrib['name'] for l in model.findall('link') if l.attrib['name'] not in children]
        chain = ET.fromstring(srdf).find("group[@name='" + cfg['group'] + "']/chain")
        if roots != [cfg['target_reference_frame']] or chain is None or chain.attrib['tip_link'] != cfg['tip_link']:
            raise RuntimeError('Configured reference frame or group tip differs from actual model')
        report['model_root'] = roots[0]
        report['srdf_chain'] = chain.attrib
        report['stage'] = 'current_state_and_tf'
        end = time.monotonic() + 12
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if fresh() and buffer.can_transform(cfg['target_reference_frame'], cfg['tip_link'], Time()):
                break
        if not fresh():
            raise RuntimeError('Fresh current state and advancing simulation clock unavailable')
        feedback = copy.deepcopy(state['joint'])
        if len(feedback.name) != len(feedback.position) or len(set(feedback.name)) != len(feedback.name) or not all(math.isfinite(q) for q in feedback.position):
            raise RuntimeError('Malformed joint feedback')
        tf_stamp = Time.from_msg(feedback.header.stamp, clock_type=ClockType.ROS_TIME)
        end = time.monotonic() + 3
        while not buffer.can_transform(cfg['target_reference_frame'], cfg['tip_link'], tf_stamp) and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        transform = buffer.lookup_transform(cfg['target_reference_frame'], cfg['tip_link'], tf_stamp)
        relative = PoseStamped()
        relative.header.frame_id = cfg['tip_link']
        relative.header.stamp = feedback.header.stamp
        relative.pose.position.x, relative.pose.position.y, relative.pose.position.z = map(float, offset)
        relative.pose.orientation.w = 1.0
        target = do_transform_pose_stamped(relative, transform)
        if cfg.get('target_mode', 'tool_offset') == 'tool_offset':
            save('relative_target.json', relative)
            save('target_pose.json', target)
        save('target_transform.json', transform)
        scene_req = GetPlanningScene.Request()
        scene_req.components.components = 1023
        scene = call(GetPlanningScene, '/get_planning_scene', scene_req).scene
        save('planning_scene.json', scene)
        start = copy.deepcopy(scene.robot_state)
        start.joint_state = feedback
        start.is_diff = False
        names = list(feedback.name)
        expected = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'}
        if set(names) != expected or len(names) != 6:
            raise RuntimeError('Expected exactly the six UR joints')
        model_joints = {j.attrib['name']: j for j in model.findall('joint')}
        mode = cfg.get('target_mode', 'tool_offset')
        if mode == 'fk_fixture':
            report['stage'] = 'construct_known_pose_with_fk'
            fixture = copy.deepcopy(start)
            fixture_joint = cfg['fixture_joint']
            delta = float(cfg['fixture_delta_rad'])
            if fixture_joint not in names or not math.isfinite(delta) or abs(delta) > 0.03:
                raise RuntimeError('Invalid introductory FK fixture')
            fixture.joint_state.position[names.index(fixture_joint)] += delta
            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = cfg['target_reference_frame']
            fk_request.fk_link_names = [cfg['tip_link']]
            fk_request.robot_state = fixture
            fixture_fk = call(GetPositionFK, '/compute_fk', fk_request)
            save('fixture_fk_request.json', fk_request)
            save('fixture_fk_response.json', fixture_fk)
            if fixture_fk.error_code.val != 1 or len(fixture_fk.pose_stamped) != 1:
                raise RuntimeError('FK fixture construction failed')
            target = fixture_fk.pose_stamped[0]
            target.header.stamp.sec = 0
            target.header.stamp.nanosec = 0
            save('target_pose.json', target)
            report['target_construction'] = 'FK of current state with configured joint delta; orientation may change. IK seed remains current state, not fixture state.'
        elif mode != 'tool_offset':
            raise RuntimeError('Unsupported target_mode')

        def check_state(robot, label):
            q = dict(zip(robot.joint_state.name, robot.joint_state.position))
            for n in names:
                j = model_joints[n]
                if n not in q or not math.isfinite(q[n]):
                    raise RuntimeError(label + ': missing/nonfinite joint')
                if j.attrib['type'] != 'continuous':
                    lim, soft = j.find('limit'), j.find('safety_controller')
                    lo, hi = float(lim.attrib['lower']), float(lim.attrib['upper'])
                    if soft is not None:
                        lo = max(lo, float(soft.attrib.get('soft_lower_limit', lo)))
                        hi = min(hi, float(soft.attrib.get('soft_upper_limit', hi)))
                    if not lo <= q[n] <= hi:
                        raise RuntimeError(label + ': joint position limit violation: ' + n)
            req = GetStateValidity.Request()
            req.robot_state, req.group_name = robot, cfg['group']
            response = call(GetStateValidity, '/check_state_validity', req)
            save(label + '_validity.json', response)
            if not response.valid:
                raise RuntimeError(label + ': invalid state (inspect saved response)')

        report['stage'] = 'start_validity'
        check_state(start, 'start')
        report['stage'] = 'ik'
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = cfg['group']
        ik_req.ik_request.robot_state = start
        ik_req.ik_request.ik_link_name = cfg['tip_link']
        ik_req.ik_request.pose_stamped = target
        ik_req.ik_request.avoid_collisions = False
        ik_req.ik_request.timeout.sec = int(cfg['ik_timeout_s'])
        ik_req.ik_request.timeout.nanosec = int((cfg['ik_timeout_s'] % 1)*1e9)
        save('ik_request.json', ik_req)
        ik = call(GetPositionIK, '/compute_ik', ik_req)
        save('ik_response.json', ik)
        report['ik_error_code'] = ik.error_code.val
        if ik.error_code.val != 1:
            raise RuntimeError('IK unsuccessful; not proof that the pose is globally unreachable')
        report['stage'] = 'goal_validity'
        check_state(ik.solution, 'goal')
        goal_map = dict(zip(ik.solution.joint_state.name, ik.solution.joint_state.position))
        plan_req = GetMotionPlan.Request()
        request = plan_req.motion_plan_request
        request.group_name, request.pipeline_id, request.planner_id = cfg['group'], cfg['pipeline_id'], cfg['planner_id']
        request.start_state = start
        request.allowed_planning_time = float(cfg['planning_time_s'])
        request.num_planning_attempts = int(cfg['planning_attempts'])
        request.max_velocity_scaling_factor = float(cfg['velocity_scaling'])
        request.max_acceleration_scaling_factor = float(cfg['acceleration_scaling'])
        request.workspace_parameters.header.frame_id = cfg['target_reference_frame']
        request.workspace_parameters.min_corner.x = request.workspace_parameters.min_corner.y = request.workspace_parameters.min_corner.z = -2.0
        request.workspace_parameters.max_corner.x = request.workspace_parameters.max_corner.y = request.workspace_parameters.max_corner.z = 2.0
        constraints = Constraints()
        for n in names:
            jc = JointConstraint()
            jc.joint_name, jc.position = n, goal_map[n]
            jc.tolerance_above = jc.tolerance_below = float(cfg['joint_goal_tolerance_rad'])
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request.goal_constraints = [constraints]
        save('planning_request.json', plan_req)
        report['stage'] = 'planning'
        before = time.monotonic()
        plan = call(GetMotionPlan, '/plan_kinematic_path', plan_req, timeout=cfg['planning_time_s'] + 10)
        report['planning_service_wall_s'] = time.monotonic() - before
        result = plan.motion_plan_response
        report['planning_error_code'] = result.error_code.val
        report['interface_planning_time_s'] = result.planning_time
        save('planning_response.json', plan)
        if result.error_code.val != 1 or not result.trajectory.joint_trajectory.points:
            raise RuntimeError('Planning failed or empty trajectory; no execution')
        tr = result.trajectory.joint_trajectory
        if set(tr.joint_names) != expected or len(tr.joint_names) != 6:
            raise RuntimeError('Unexpected output joint names')
        times = [p.time_from_start.sec + p.time_from_start.nanosec/1e9 for p in tr.points]
        if len(times) < 2 or times[0] < 0 or any(b <= a for a,b in zip(times,times[1:])):
            raise RuntimeError('Invalid trajectory timing')
        if any(len(p.positions) != 6 or not all(math.isfinite(v) for v in p.positions) for p in tr.points):
            raise RuntimeError('Invalid output positions')
        report['stage'] = 'endpoint_fk'
        fk_req = GetPositionFK.Request()
        fk_req.header.frame_id = cfg['target_reference_frame']
        fk_req.fk_link_names = [cfg['tip_link']]
        fk_req.robot_state = copy.deepcopy(start)
        fk_req.robot_state.joint_state.name = list(tr.joint_names)
        fk_req.robot_state.joint_state.position = list(tr.points[-1].positions)
        fk_req.robot_state.joint_state.velocity = []
        fk_req.robot_state.joint_state.effort = []
        fk = call(GetPositionFK, '/compute_fk', fk_req)
        save('endpoint_fk.json', fk)
        if fk.error_code.val != 1 or len(fk.pose_stamped) != 1:
            raise RuntimeError('Endpoint FK failed')
        actual = fk.pose_stamped[0].pose
        desired = target.pose
        error = math.dist([actual.position.x, actual.position.y, actual.position.z], [desired.position.x, desired.position.y, desired.position.z])
        qa = [actual.orientation.x,actual.orientation.y,actual.orientation.z,actual.orientation.w]
        qb = [desired.orientation.x,desired.orientation.y,desired.orientation.z,desired.orientation.w]
        dot = abs(sum(a*b for a,b in zip(qa,qb)) / math.sqrt(sum(a*a for a in qa)*sum(b*b for b in qb)))
        angle = 2*math.acos(min(1.0, max(0.0,dot)))
        report.update({'endpoint_position_error_m': error, 'endpoint_orientation_error_rad': angle,
                       'point_count': len(tr.points), 'planned_duration_s': times[-1]})
        if error > cfg['endpoint_position_tolerance_m'] or angle > cfg['endpoint_orientation_tolerance_rad']:
            raise RuntimeError('Planned endpoint does not meet original pose tolerance')
        display = DisplayTrajectory()
        display.model_id, display.trajectory_start, display.trajectory = model.attrib['name'], result.trajectory_start, [result.trajectory]
        raw = {'display_trajectory': message_to_ordereddict(display), 'feedback_at_plan_receive': message_to_ordereddict(feedback)}
        raw_path = out / 'trajectory_and_feedback.json'
        raw_path.write_text(json.dumps(raw, indent=2))
        report['raw_sha256'] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
        report['max_path_departure_per_joint_rad'] = [max(abs(p.positions[i]-tr.points[0].positions[i]) for p in tr.points) for i in range(6)]
        report['intro_executor_displacement_check'] = max(report['max_path_departure_per_joint_rad']) <= 0.06
        report['stage'], report['passed'] = 'plan_saved_not_executed', True
    except Exception as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        for label, command in [('commit',['rev-parse','HEAD']),('dirty_status',['status','--short'])]:
            try:
                result = subprocess.run(['git','-C',str(Path.home()/'ROS2'),*command],capture_output=True,text=True,timeout=5)
                report[label] = result.stdout.strip()
            except Exception:
                report[label] = 'unavailable'
        (out / 'pose_plan_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print('SAVED:', out)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
