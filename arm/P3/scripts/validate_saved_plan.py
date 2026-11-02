import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import qos_profile_sensor_data
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('capture_dir', type=Path)
    args = parser.parse_args()
    raw_path = args.capture_dir / 'trajectory_and_feedback.json'
    raw = json.loads(raw_path.read_text())
    display = raw['display_trajectory']
    if len(display['trajectory']) != 1:
        raise RuntimeError('Expected exactly one trajectory')
    trajectory = display['trajectory'][0]['joint_trajectory']
    names = trajectory['joint_names']
    points = trajectory['points']
    if len(names) != 6 or len(set(names)) != 6 or len(points) < 2:
        raise RuntimeError('Invalid joint names or point count')
    if any(len(p['positions']) != 6 or not all(math.isfinite(q) for q in p['positions']) for p in points):
        raise RuntimeError('Invalid position arrays')
    out = args.capture_dir / ('validation_' + datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z') + '_' + uuid.uuid4().hex[:8])
    out.mkdir(exist_ok=False)
    report = {'checked_at': datetime.now().astimezone().isoformat(),
              'raw_sha256': hashlib.sha256(raw_path.read_bytes()).hexdigest(),
              'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'group': 'ur_manipulator', 'max_interpolation_step_per_joint_rad': 0.005,
              'start_tolerance_rad': 0.01, 'receive_age_limit_wall_s': 1.0,
              'feedback_sim_age_limit_s': 0.5, 'passed': False,
              'execution_requested': False,
              'limitations': ['Checks current MoveIt scene, not Gazebo world equivalence.',
                              'Linear joint interpolation sampled at <=0.005 rad per joint; not continuous collision checking or exact controller spline validation.',
                              'Pass is valid only for observed scene/state; execution requires renewed checks.',
                              'Does not validate velocity/acceleration limits, controller mapping, or cancellation behavior.']}
    rclpy.init()
    node = rclpy.create_node('p3_validate_saved_plan', enable_rosout=False)
    feedback = []
    clocks = []
    subs = [node.create_subscription(JointState, '/joint_states', lambda m: feedback.append((time.monotonic(), m)), qos_profile_sensor_data),
            node.create_subscription(Clock, '/clock', lambda m: clocks.append((time.monotonic(), m.clock.sec + m.clock.nanosec / 1e9)), qos_profile_sensor_data)]

    def call(srv_type, endpoint, req):
        client = node.create_client(srv_type, endpoint)
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('Service unavailable: ' + endpoint)
            future = client.call_async(req)
            deadline = time.monotonic() + 4.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done():
                raise RuntimeError('Read-only service timeout: ' + endpoint)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        req = GetParameters.Request()
        req.names = ['robot_description', 'robot_description_semantic', 'use_sim_time']
        params = call(GetParameters, '/move_group/get_parameters', req)
        urdf, srdf = params.values[0].string_value, params.values[1].string_value
        if not urdf or not srdf or not params.values[2].bool_value:
            raise RuntimeError('Missing model or MoveIt use_sim_time is not true')
        (out / 'robot.urdf').write_text(urdf)
        (out / 'robot.srdf').write_text(srdf)
        joints = {j.attrib['name']: j for j in ET.fromstring(urdf).findall('joint')}
        limits = {}
        for name in names:
            joint = joints[name]
            lim = joint.find('limit')
            if joint.attrib['type'] == 'continuous':
                limits[name] = None
            elif joint.attrib['type'] == 'revolute' and lim is not None:
                lower, upper = float(lim.attrib['lower']), float(lim.attrib['upper'])
                safety = joint.find('safety_controller')
                if safety is not None:
                    lower = max(lower, float(safety.attrib.get('soft_lower_limit', lower)))
                    upper = min(upper, float(safety.attrib.get('soft_upper_limit', upper)))
                limits[name] = [lower, upper]
            else:
                raise RuntimeError('Unsupported joint type/limit: ' + name)
        report['urdf_position_limits_with_soft_limits_rad'] = limits
        scene_req = GetPlanningScene.Request()
        scene_req.components.components = (PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                                           PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.OCTOMAP |
                                           PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.TRANSFORMS |
                                           PlanningSceneComponents.LINK_PADDING_AND_SCALING | PlanningSceneComponents.SCENE_SETTINGS)
        scene = call(GetPlanningScene, '/get_planning_scene', scene_req).scene
        (out / 'planning_scene.json').write_text(json.dumps(message_to_ordereddict(scene), indent=2))
        samples = [points[0]['positions']]
        for a, b in zip(points, points[1:]):
            qa, qb = a['positions'], b['positions']
            count = max(1, math.ceil(max(abs(y - x) for x, y in zip(qa, qb)) / 0.005))
            for k in range(1, count + 1):
                samples.append([x + (y - x) * k / count for x, y in zip(qa, qb)])
        rows = []
        for index, q in enumerate(samples):
            bounded = all(limits[n] is None or limits[n][0] <= value <= limits[n][1] for n, value in zip(names, q))
            req = GetStateValidity.Request()
            req.group_name = 'ur_manipulator'
            req.robot_state = scene.robot_state
            req.robot_state.joint_state.name = names
            req.robot_state.joint_state.position = q
            req.robot_state.joint_state.velocity = []
            req.robot_state.joint_state.effort = []
            req.robot_state.joint_state.header.stamp.sec = 0
            req.robot_state.joint_state.header.stamp.nanosec = 0
            req.robot_state.is_diff = False
            result = call(GetStateValidity, '/check_state_validity', req)
            rows.append({'index': index, 'q_rad': q, 'urdf_bounds_ok': bounded,
                         'moveit_valid': result.valid, 'response': message_to_ordereddict(result)})
        (out / 'sample_checks.json').write_text(json.dumps(rows, indent=2))
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not feedback or len(clocks) < 2:
            raise RuntimeError('No feedback or clock samples')
        now = time.monotonic()
        received, actual = feedback[-1]
        if len(actual.name) != len(actual.position) or len(set(actual.name)) != len(actual.name):
            raise RuntimeError('Malformed feedback')
        actual_map = dict(zip(actual.name, actual.position))
        errors = [points[0]['positions'][i] - actual_map[name] for i, name in enumerate(names)]
        actual_stamp = actual.header.stamp.sec + actual.header.stamp.nanosec / 1e9
        sim_age = clocks[-1][1] - actual_stamp
        fresh = now - received <= 1.0 and now - clocks[-1][0] <= 1.0 and abs(sim_age) <= 0.5
        advancing = clocks[-1][1] > clocks[0][1]
        start_ok = all(math.isfinite(e) and abs(e) <= 0.01 for e in errors)
        report.update({'sample_count': len(rows), 'invalid_sample_count': sum(not (r['urdf_bounds_ok'] and r['moveit_valid']) for r in rows),
                       'feedback_receive_age_wall_s': now - received, 'feedback_minus_clock_sim_s': -sim_age,
                       'clock_advanced': advancing, 'start_minus_current_rad': errors,
                       'feedback_checks_passed': fresh, 'start_matches': start_ok,
                       'passed': fresh and advancing and start_ok and all(r['urdf_bounds_ok'] and r['moveit_valid'] for r in rows)})
    except Exception as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        (out / 'validation_report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        print('SAVED:', out)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
