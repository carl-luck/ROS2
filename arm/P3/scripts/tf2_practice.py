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
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, PoseStamped
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point, do_transform_pose_stamped


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', default='tool0')
    parser.add_argument('--target', default='world')
    args = parser.parse_args()
    root = Path.home() / 'ROS2/arm/P3/evidence/tf'
    out = root / (datetime.now().astimezone().strftime('%Y%m%d_%H%M%S%z') + '_stamped_transform_' + uuid.uuid4().hex[:8])
    out.mkdir(parents=True, exist_ok=False)
    report = {'created_at': datetime.now().astimezone().isoformat(), 'source': args.source, 'target': args.target,
              'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'passed': False,
              'limits': ['Checks simulated TF consistency, not physical calibration or real positioning accuracy.',
                         'Target frame is explicitly selected; not a MoveGroupInterface planning-frame query.',
                         'No TF publication, planning request or execution request is sent.']}
    rclpy.init()
    node = rclpy.create_node('p3_tf2_practice', enable_rosout=False,
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(cache_time=Duration(seconds=10), node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    clocks = []
    sub = node.create_subscription(Clock, '/clock', lambda m: clocks.append((time.monotonic(), m.clock.sec + m.clock.nanosec / 1e9)), qos_profile_sensor_data)

    def query_model():
        client = node.create_client(GetParameters, '/move_group/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('MoveIt parameter service unavailable')
            req = GetParameters.Request()
            req.names = ['robot_description', 'robot_description_semantic', 'use_sim_time']
            future = client.call_async(req)
            end = time.monotonic() + 4
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done():
                raise RuntimeError('Model query wall timeout')
            values = future.result().values
            if not values[2].bool_value:
                raise RuntimeError('MoveIt is not configured with simulation time')
            return values[0].string_value, values[1].string_value
        finally:
            node.destroy_client(client)

    try:
        urdf, srdf = query_model()
        (out / 'robot.urdf').write_text(urdf)
        (out / 'robot.srdf').write_text(srdf)
        model = ET.fromstring(urdf)
        children = {j.find('child').attrib['link'] for j in model.findall('joint')}
        roots = [l.attrib['name'] for l in model.findall('link') if l.attrib['name'] not in children]
        chain = ET.fromstring(srdf).find("group[@name='ur_manipulator']/chain")
        report['model_definition'] = {'urdf_roots': roots, 'srdf_chain': None if chain is None else chain.attrib}
        print('Collecting TF and /clock; spinning callbacks with a 12-second wall deadline.', flush=True)
        begin = time.monotonic()
        ready = False
        while time.monotonic() - begin < 12:
            rclpy.spin_once(node, timeout_sec=0.05)
            # Nonblocking query; spin_once above keeps receiving TF and clock messages.
            if (len(clocks) >= 2 and clocks[-1][1] > clocks[0][1] and
                    time.monotonic() - clocks[-1][0] < 1 and
                    buffer.can_transform(args.target, args.source, Time())):
                ready = True
                break
        (out / 'tf_tree.yaml').write_text(buffer.all_frames_as_yaml())
        if not ready:
            raise RuntimeError('TF or advancing clock unavailable within wall deadline; inspect names, tree and clock')
        latest = buffer.lookup_transform(args.target, args.source, Time())
        # Time zero selects latest common time. Reuse its actual stamp for both directions.
        stamp = Time.from_msg(latest.header.stamp, clock_type=ClockType.ROS_TIME)
        if stamp.nanoseconds == 0:
            raise RuntimeError('This exercise expects a dynamic chain with a nonzero timestamp')
        forward = buffer.lookup_transform(args.target, args.source, stamp)
        reverse = buffer.lookup_transform(args.source, args.target, stamp)
        quat = forward.transform.rotation
        norm = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        if abs(norm - 1) > 1e-6:
            raise RuntimeError('Non-unit TF quaternion')
        point = PointStamped()
        point.header.frame_id = args.source
        point.header.stamp = stamp.to_msg()
        point.point.x = 0.02
        converted_point = do_transform_point(point, forward)
        returned_point = do_transform_point(converted_point, reverse)
        point_error = math.dist([point.point.x, point.point.y, point.point.z],
                                [returned_point.point.x, returned_point.point.y, returned_point.point.z])
        pose = PoseStamped()
        pose.header = point.header
        pose.pose.position.x = 0.02
        pose.pose.orientation.w = 1.0
        converted_pose = do_transform_pose_stamped(pose, forward)
        returned_pose = do_transform_pose_stamped(converted_pose, reverse)
        pose_position_error = math.dist([0.02, 0, 0], [returned_pose.pose.position.x, returned_pose.pose.position.y, returned_pose.pose.position.z])
        q = returned_pose.pose.orientation
        # Quaternion q and -q represent the same rotation. atan2 is stable near identity.
        orientation_error = 2 * math.atan2(math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z), abs(q.w))
        origin = PointStamped()
        origin.header = point.header
        transformed_origin = do_transform_point(origin, forward)
        trans = forward.transform.translation
        origin_error = math.dist([transformed_origin.point.x, transformed_origin.point.y, transformed_origin.point.z], [trans.x, trans.y, trans.z])
        missing = 'p3_deliberately_missing_frame_6d94c2'
        if buffer.can_transform(args.target, missing, Time()):
            raise RuntimeError('Test frame unexpectedly exists')
        diagnostics = {}
        try:
            buffer.lookup_transform(args.target, missing, stamp)
            diagnostics['missing_frame'] = {'expected': False, 'error': 'unexpected success'}
        except LookupException as exc:
            diagnostics['missing_frame'] = {'expected': True, 'type': type(exc).__name__, 'error': str(exc)}
        future_stamp = Time(nanoseconds=stamp.nanoseconds + 60000000000, clock_type=ClockType.ROS_TIME)
        try:
            buffer.lookup_transform(args.target, args.source, future_stamp)
            diagnostics['future_time'] = {'expected': False, 'error': 'unexpected success'}
        except ExtrapolationException as exc:
            diagnostics['future_time'] = {'expected': True, 'type': type(exc).__name__, 'error': str(exc)}
        payload = {'source_point': point, 'target_point': converted_point, 'returned_point': returned_point,
                   'source_pose': pose, 'target_pose': converted_pose, 'returned_pose': returned_pose,
                   'forward_transform': forward, 'reverse_transform': reverse}
        (out / 'transforms_and_messages.json').write_text(json.dumps({k: message_to_ordereddict(v) for k, v in payload.items()}, indent=2))
        report.update({'query_sim_stamp': {'sec': stamp.to_msg().sec, 'nanosec': stamp.to_msg().nanosec},
                       'input_meaning': 'A point and identity-oriented pose 0.02 m along tool/source +X; not a motion target.',
                       'tf_quaternion_norm': norm, 'point_roundtrip_error_m': point_error,
                       'pose_roundtrip_position_error_m': pose_position_error,
                       'pose_roundtrip_orientation_error_rad': orientation_error,
                       'source_origin_vs_translation_error_m': origin_error,
                       'diagnostics': diagnostics,
                       'passed': max(point_error, pose_position_error, origin_error) < 1e-8 and orientation_error < 1e-8 and all(d['expected'] for d in diagnostics.values())})
    except Exception as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        (out / 'tf_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print('SAVED:', out)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
