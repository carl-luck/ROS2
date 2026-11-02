import argparse
import copy
from datetime import datetime
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict


def main():
    parser = argparse.ArgumentParser(description='IK diagnostics only; no motion or planning.')
    parser.add_argument('failed_run', type=Path)
    args = parser.parse_args()
    original = GetPositionIK.Request()
    set_message_fields(original, json.loads((args.failed_run / 'ik_request.json').read_text()))
    out = args.failed_run / ('ik_diagnostic_' + datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z') + '_' + uuid.uuid4().hex[:8])
    out.mkdir(exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('p3_diagnose_pose_ik', enable_rosout=False)
    report = {'created_at': datetime.now().astimezone().isoformat(), 'execution_requested': False,
              'uses_saved_start': True, 'tests': {}}

    def call(cls, name, req):
        client = node.create_client(cls, name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Unavailable: ' + name)
            future = client.call_async(req)
            end = time.monotonic() + 5
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done():
                raise RuntimeError('Application wall timeout at ' + name + '; do not automatically retry')
            return future.result()
        finally:
            node.destroy_client(client)

    def fk(state, label):
        req = GetPositionFK.Request()
        req.header.frame_id = original.ik_request.pose_stamped.header.frame_id
        req.fk_link_names = [original.ik_request.ik_link_name]
        req.robot_state = state
        res = call(GetPositionFK, '/compute_fk', req)
        (out / (label + '_fk.json')).write_text(json.dumps(message_to_ordereddict(res), indent=2))
        if res.error_code.val != 1 or len(res.pose_stamped) != 1:
            raise RuntimeError('FK failed: ' + label)
        return res.pose_stamped[0]

    def ik(pose, seed, label):
        req = copy.deepcopy(original)
        req.ik_request.pose_stamped = copy.deepcopy(pose)
        # Frame is the fixed model root; a saved numerical pose is queried, not a live moving target.
        req.ik_request.pose_stamped.header.stamp.sec = 0
        req.ik_request.pose_stamped.header.stamp.nanosec = 0
        req.ik_request.robot_state = copy.deepcopy(seed)
        req.ik_request.avoid_collisions = False
        res = call(GetPositionIK, '/compute_ik', req)
        (out / (label + '_ik.json')).write_text(json.dumps({'request': message_to_ordereddict(req), 'response': message_to_ordereddict(res)}, indent=2))
        report['tests'][label] = {'ik_error_code': res.error_code.val, 'success': res.error_code.val == 1}

    try:
        start = copy.deepcopy(original.ik_request.robot_state)
        current_pose = fk(start, 'saved_start')
        tf = json.loads((args.failed_run / 'target_transform.json').read_text())['transform']
        p = current_pose.pose.position
        q = current_pose.pose.orientation
        tfq = tf['rotation']
        qa = [q.x, q.y, q.z, q.w]
        qb = [tfq[k] for k in ('x','y','z','w')]
        dot = abs(sum(a*b for a,b in zip(qa,qb)) / math.sqrt(sum(a*a for a in qa)*sum(b*b for b in qb)))
        report['saved_start_fk_vs_tf_position_m'] = math.dist([p.x,p.y,p.z], [tf['translation'][k] for k in ('x','y','z')])
        report['saved_start_fk_vs_tf_orientation_rad'] = 2*math.acos(min(1.0,dot))
        fixture = copy.deepcopy(start)
        index = list(fixture.joint_state.name).index('shoulder_pan_joint')
        fixture.joint_state.position[index] += 0.03
        fixture_pose = fk(fixture, 'known_joint_fixture')
        ik(current_pose, start, 'current_pose_same_seed')
        ik(original.ik_request.pose_stamped, start, 'original_offset_zero_stamp')
        ik(fixture_pose, start, 'known_fk_pose_current_seed')
        ik(fixture_pose, fixture, 'known_fk_pose_exact_seed')
        report['fixture_definition'] = 'Saved start with shoulder_pan_joint +0.03 rad; FK defines a kinematically reachable test pose. Collision validity is not implied.'
    except Exception as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        (out / 'ik_diagnostic_report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        print('SAVED:', out)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
