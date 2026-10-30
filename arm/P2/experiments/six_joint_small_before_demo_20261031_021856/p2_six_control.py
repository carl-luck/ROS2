import argparse
import ast
import csv
import gc
import hashlib
import importlib.metadata
import json
import multiprocessing
import queue
import re
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from ur5e_mpc_control import joint_interface_test as old_interface
from ur5e_mpc_control import zero_hold
from ur5e_mpc_control.p2_six_core import C, JOINTS, CollisionModel, RobotModel, SixMpc, reference


def write_json(path, value):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")
    temp.replace(path)


def decode_state(msg):
    if len(msg.name) != len(set(msg.name)):
        raise RuntimeError("duplicate_joint_names")
    indices = [msg.name.index(j) for j in JOINTS]
    q = np.array([msg.position[i] for i in indices])
    v = np.array([msg.velocity[i] for i in indices])
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):
        raise RuntimeError("nonfinite_feedback")
    return dict(q=q, v=v, sim=msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec,
                stamp=msg.header.stamp, wall=time.monotonic())


class Snapshot(Node):
    def __init__(self):
        super().__init__("p2_six_snapshot")
        self.latest = None
        self.first = None
        self.fault = None
        self.sub = self.create_subscription(JointState, "/joint_states", self.receive, qos_profile_sensor_data)
        self.client = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")

    def receive(self, msg):
        try:
            state = decode_state(msg)
            if self.latest and state["sim"] < self.latest["sim"]:
                raise RuntimeError("simulation_time_reversed")
            self.latest = state
            if self.first is None:
                self.first = state["sim"]
        except Exception as exc:
            self.fault = str(exc)

    def capture(self):
        deadline = time.monotonic()+10
        future = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.fault:
                raise RuntimeError(self.fault)
            if future is None and self.client.service_is_ready():
                request = GetParameters.Request()
                request.names = ["robot_description"]
                future = self.client.call_async(request)
            if (future and future.done() and self.latest is not None
                    and self.latest["sim"]-self.first >= 0.05
                    and time.monotonic()-self.latest["wall"] < C["feedback_timeout"]):
                if np.max(np.abs(self.latest["v"])) > 0.001:
                    raise RuntimeError("initial_robot_not_stationary")
                xml = future.result().values[0].string_value
                if not xml:
                    raise RuntimeError("empty_robot_description")
                return xml, self.latest
        raise RuntimeError("snapshot_no_advancing_feedback_or_robot_description")


def world_snapshot():
    info = subprocess.run(["ign", "service", "-i", "-s", "/world/empty/scene/info"], capture_output=True, text=True, timeout=8)
    providers = [line.strip() for line in info.stdout.splitlines() if "tcp://" in line]
    if info.returncode or len(providers) != 1:
        raise RuntimeError("need_exactly_one_Gazebo_world_provider; observed="+str(providers))
    result = subprocess.run(["ign", "service", "-s", "/world/empty/generate_world_sdf",
        "--reqtype", "ignition.msgs.SdfGeneratorConfig", "--reptype", "ignition.msgs.StringMsg",
        "--timeout", "4000", "--req", " "], capture_output=True, text=True, timeout=8)
    text = result.stdout.strip()
    if result.returncode or not text.startswith("data: "):
        raise RuntimeError("cannot_read_Gazebo_world_sdf: "+result.stderr[-200:])
    xml = ast.literal_eval(text[6:])
    xml = re.sub(r"<uri>file://<urdf-string></uri>", "<uri>file://&lt;urdf-string&gt;</uri>", xml)
    world = ET.fromstring(xml).find("world")
    models = world.findall("model")
    names = [m.get("name") for m in models]
    includes = world.findall("include")
    included_names = [item.findtext("name") for item in includes]
    if sorted(names+included_names) != ["ground_plane", "ur"]:
        raise RuntimeError("unsupported_world_models: "+str(names))
    for item in includes:
        if item.findtext("name") != "ur" or item.findtext("uri") != "file://<urdf-string>":
            raise RuntimeError("unsupported_world_include")
        pose = [float(x) for x in item.findtext("pose", "0 0 0 0 0 0").split()]
        if max(abs(x) for x in pose) > 1e-6:
            raise RuntimeError("included_robot_not_at_expected_origin")
    for model in models:
        pose = [float(x) for x in model.findtext("pose", "0 0 0 0 0 0").split()]
        if max(abs(x) for x in pose) > 1e-6:
            raise RuntimeError("world_model_not_at_expected_origin: "+model.get("name"))
        if model.get("name") == "ground_plane":
            planes = model.findall(".//collision/geometry/plane")
            if len(planes) != 1 or planes[0].findtext("normal").split() != ["0", "0", "1"]:
                raise RuntimeError("unsupported_ground_geometry")
            for item in model.findall("link")+model.findall(".//collision"):
                p = [float(x) for x in item.findtext("pose", "0 0 0 0 0 0").split()]
                if max(abs(x) for x in p) > 1e-6:
                    raise RuntimeError("unsupported_ground_pose")
    return xml


def read_model(folder, with_collision=True):
    expected = json.loads((folder/"preflight.json").read_text(encoding="utf-8"))
    if expected.get("status") != "passed" or expected.get("config") != C:
        raise RuntimeError("preflight_missing_or_config_changed")
    for name, digest in expected["asset_hashes"].items():
        path = folder/"robot.urdf" if name == "robot_description" else folder/"robot.srdf" if name == "srdf" else Path(name)
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise RuntimeError("preflight_model_asset_changed: "+name)
    model = RobotModel((folder/"robot.urdf").read_text(encoding="utf-8"))
    if not with_collision:
        return model, None
    collision = CollisionModel(model, (folder/"robot.srdf").read_text(encoding="utf-8"), get_package_share_directory)
    if expected["asset_hashes"] != collision.hashes:
        raise RuntimeError("preflight_missing_or_model_assets_changed")
    return model, collision


def collision_worker(assets, mailbox, wake, stopping, outgoing):
    try:
        _, collision = read_model(Path(assets))
        gc.collect()
        gc.disable()
        outgoing.put(dict(ready=True))
        last_sequence = 0
        while not stopping.is_set():
            if not wake.wait(timeout=0.1):
                continue
            wake.clear()
            with mailbox.get_lock():
                item = list(mailbox[:])
            sequence = int(item[0])
            if sequence <= last_sequence:
                continue
            last_sequence = sequence
            started = time.monotonic()
            common = dict(sequence=sequence, source_wall=item[1], submitted_wall=item[2],
                          started_wall=started)
            try:
                result = collision.check(np.asarray(item[3:]))
                finished = time.monotonic()
                outgoing.put(dict(**common, finished_wall=finished, compute_wall_s=finished-started,
                                  result=result, error=None))
            except Exception as exc:
                outgoing.put(dict(**common, error=str(exc)))
                break
    except Exception as exc:
        outgoing.put(dict(error=str(exc)))


class CollisionMonitor:
    def __init__(self, assets, interrupted, log_path=None):
        context = multiprocessing.get_context("spawn")
        self.mailbox = context.Array("d", 9)
        self.wake = context.Event()
        self.stopping = context.Event()
        self.outgoing = context.Queue()
        self.process = context.Process(target=collision_worker,
            args=(str(assets), self.mailbox, self.wake, self.stopping, self.outgoing))
        self.process.daemon = True
        self.last = None
        self.first_submit = None
        self.next_submit = 0.0
        self.submitted = 0
        self.checks = 0
        self.max_compute = 0.0
        self.minimum_self = float("inf")
        self.minimum_floor = float("inf")
        self.diagnostics = dict(policy="single overwrite mailbox; running check completes, next check uses latest snapshot",
            max_source_to_submit_wall_s=0.0, max_pending_wall_s=0.0,
            max_result_delivery_wall_s=0.0, max_result_age_wall_s=0.0, skipped_sequences=0)
        self.log_file = None if log_path is None else Path(log_path).open("w", newline="", encoding="utf-8")
        self.log = None if self.log_file is None else csv.DictWriter(self.log_file, fieldnames=[
            "sequence", "source_wall", "submitted_wall", "started_wall", "finished_wall", "received_wall",
            "source_to_submit_wall_s", "pending_wall_s", "compute_wall_s", "delivery_wall_s", "result_age_wall_s"])
        if self.log:
            self.log.writeheader()
        self.process.start()
        deadline = time.monotonic()+30
        try:
            while True:
                if interrupted or time.monotonic() > deadline:
                    raise RuntimeError("collision_worker_startup_timeout_or_interrupted")
                try:
                    result = self.outgoing.get(timeout=0.05)
                except queue.Empty:
                    if not self.process.is_alive():
                        raise RuntimeError("collision_worker_exited_at_startup")
                    continue
                if result.get("error"):
                    raise RuntimeError(result["error"])
                if result.get("ready"):
                    break
        except Exception:
            self.close()
            raise

    def update(self, q, source_wall):
        while True:
            try:
                result = self.outgoing.get_nowait()
            except queue.Empty:
                break
            if result.get("error"):
                self.diagnostics["worker_error"] = result
                raise RuntimeError("collision_worker: "+result["error"])
            if result.get("ready"):
                continue
            received = time.monotonic()
            timing = dict(sequence=result["sequence"], source_wall=result["source_wall"],
                submitted_wall=result["submitted_wall"], started_wall=result["started_wall"],
                finished_wall=result["finished_wall"], received_wall=received,
                source_to_submit_wall_s=result["submitted_wall"]-result["source_wall"],
                pending_wall_s=result["started_wall"]-result["submitted_wall"],
                compute_wall_s=result["compute_wall_s"], delivery_wall_s=received-result["finished_wall"],
                result_age_wall_s=received-result["source_wall"])
            prior_sequence = 0 if self.last is None else self.last["sequence"]
            self.diagnostics["skipped_sequences"] += max(0, result["sequence"]-prior_sequence-1)
            for key, value in [("max_source_to_submit_wall_s", timing["source_to_submit_wall_s"]),
                    ("max_pending_wall_s", timing["pending_wall_s"]),
                    ("max_result_delivery_wall_s", timing["delivery_wall_s"]),
                    ("max_result_age_wall_s", timing["result_age_wall_s"])]:
                self.diagnostics[key] = max(self.diagnostics[key], value)
            if self.log:
                self.log.writerow(timing)
            self.last = result
            self.checks += 1
            self.max_compute = max(self.max_compute, result["compute_wall_s"])
            self.minimum_self = min(self.minimum_self, result["result"]["self_clearance_m"])
            self.minimum_floor = min(self.minimum_floor, result["result"]["floor_clearance_m"])
        if not self.process.is_alive():
            raise RuntimeError("collision_worker_disappeared")
        now = time.monotonic()
        if now >= self.next_submit and self.mailbox.get_lock().acquire(block=False):
            try:
                self.submitted += 1
                self.mailbox[:] = [self.submitted, source_wall, now]+np.asarray(q).tolist()
            finally:
                self.mailbox.get_lock().release()
            self.wake.set()
            if self.first_submit is None:
                self.first_submit = now
            self.next_submit = now+C["collision_period"]
        origin = self.last["source_wall"] if self.last is not None else self.first_submit
        if origin is not None:
            age = time.monotonic()-origin
            if age > C["feedback_timeout"]:
                self.diagnostics["failure_result_age_wall_s"] = age
                self.diagnostics["last_completed_sequence"] = None if self.last is None else self.last["sequence"]
                self.diagnostics["latest_submitted_sequence"] = self.submitted
                raise RuntimeError("collision_result_stale")
        return self.last is not None

    def close(self):
        if self.process.is_alive():
            self.stopping.set()
            self.wake.set()
        self.process.join(timeout=2)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=2)
        self.outgoing.cancel_join_thread()
        self.outgoing.close()
        if self.log_file is not None:
            self.log_file.close()


def preflight(folder, interrupted):
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(status="running", config=C)
    write_json(folder/"preflight.json", report)
    node = None
    try:
        xml_world = world_snapshot()
        node = Snapshot()
        xml, state = node.capture()
        srdf = xacro.process_file(str(Path(get_package_share_directory("ur_moveit_config"))/"srdf/ur.srdf.xacro"), mappings={"name":"ur", "prefix":""}).toxml()
        model = RobotModel(xml)
        collision = CollisionModel(model, srdf, get_package_share_directory)
        sampled = collision.check_reference(state["q"], interrupted)
        (folder/"robot.urdf").write_text(xml, encoding="utf-8")
        (folder/"robot.srdf").write_text(srdf, encoding="utf-8")
        (folder/"world.sdf").write_text(xml_world, encoding="utf-8")
        report.update(status="passed", initial_q_rad=state["q"].tolist(), initial_v_rad_s=state["v"].tolist(),
                      asset_hashes=collision.hashes, urdf_lower_with_margin_rad=model.lower.tolist(),
                      urdf_upper_with_margin_rad=model.upper.tolist(), urdf_velocity_limit_rad_s=model.speed.tolist(),
                      reference_check=sampled,
                      world_sha256=hashlib.sha256(xml_world.encode()).hexdigest(),
                      environment_note="only origin-mounted ur plus horizontal ground_plane supported; no live dynamic obstacle tracking")
        print("预检查通过：", json.dumps(sampled), flush=True)
    except Exception as exc:
        report.update(status="failed", reason=str(exc))
        raise
    finally:
        write_json(folder/"preflight.json", report)
        if node:
            node.destroy_node()


class SixGuard(zero_hold.ZeroHold):
    def __init__(self, folder, model, collision):
        super().__init__(folder)
        self.model, self.collision = model, collision
        self.origin = None
        self.lower = self.upper = None
        self.velocity = None
        self.max_speed = np.zeros(6)
        self.peak_excursion = np.zeros(6)
        self.last_request_wall = self.last_request_stamp = None
        self.value = np.zeros(6)
        self.output = np.zeros(6)
        self.output_stamp = None
        self.accepted = 0
        self.complete = False
        self.sequence = 0
        self.collision_checks = 0
        self.status = "starting"
        self.requests_file = (folder/"request_log.csv").open("w", newline="", encoding="utf-8")
        self.requests_log = csv.writer(self.requests_file)
        self.requests_log.writerow(["wall_elapsed_s", "source_sim_stamp_s", "accepted"]+[j+"_request_rad_s" for j in JOINTS])
        self.output_file = (folder/"output_log.csv").open("w", newline="", encoding="utf-8")
        self.output_log = csv.writer(self.output_file)
        self.output_log.writerow(["sequence", "phase", "publish_wall_elapsed_s", "feedback_sim_stamp_s", "request_sim_stamp_s"]+[j+"_command_rad_s" for j in JOINTS])
        self.request_sub = self.create_subscription(JointState, C["request_topic"], self.request, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, C["status_topic"], 10)

    def receive(self, msg):
        super().receive(msg)
        if self.fault:
            return
        self.velocity = decode_state(msg)["v"]
        self.max_speed = np.maximum(self.max_speed, np.abs(self.velocity))
        if np.any(np.abs(self.velocity) > 0.03):
            self.fault = "actual_joint_speed_limit"
        if self.origin is not None:
            q = np.asarray(self.positions)
            self.peak_excursion = np.maximum(self.peak_excursion, np.abs(q-self.origin))
            if np.any(q < self.lower) or np.any(q > self.upper):
                self.fault = "joint_position_or_excursion_limit"

    def request(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        values = np.asarray(msg.velocity)
        valid = (self.phase == "hold" and self.fault is None and not self.complete and list(msg.name) == JOINTS
                 and values.shape == (6,) and np.all(np.isfinite(values))
                 and np.all(np.abs(values) <= np.asarray(C["vmax"])+1e-9)
                 and self.last_sim is not None and abs(self.last_sim-stamp) <= C["stamp_tolerance"]
                 and (self.last_request_stamp is None or stamp >= self.last_request_stamp))
        self.requests_log.writerow([now-self.started, stamp, valid]+values.tolist())
        if not valid:
            if not self.complete:
                self.fault = self.fault or "invalid_or_stale_request"
            return
        self.last_request_wall, self.last_request_stamp = now, stamp
        self.value = values
        self.accepted += 1
        if msg.header.frame_id == "p2_six_complete":
            if np.any(values != 0):
                self.fault = "nonzero_completion"
            else:
                self.complete = True

    def publish_output(self, values, now):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in values]
        self.publisher.publish(msg)
        self.sequence += 1
        self.sent += 1
        self.output_log.writerow([self.sequence, self.phase, time.monotonic()-self.started, self.last_sim, self.last_request_stamp]+list(msg.data))
        self.command_log.writerow([self.phase, now-self.started, self.last_sim]+list(msg.data))
        self.next_publish = now+C["heartbeat"]

    def publish_zero(self, now):
        if now >= self.next_publish:
            self.publish_output(np.zeros(6), now)

    def emit(self, now):
        if now < self.next_publish:
            return
        dt = 0.0 if self.output_stamp is None else min(max(self.last_sim-self.output_stamp, 0), 0.04)
        change = np.asarray(C["rate"])*dt
        self.output = np.clip(self.value, self.output-change, self.output+change)
        self.publish_output(self.output, now)
        self.output_stamp = self.last_sim

    def announce(self):
        msg = String()
        msg.data = json.dumps(dict(state=self.status, phase=self.phase, origin=None if self.origin is None else self.origin.tolist(), fault=self.fault))
        self.status_pub.publish(msg)

    def stop_and_check(self):
        return old_interface.Guard.stop_and_check(self)


def guard(folder, assets, interrupted, run):
    model, collision = read_model(assets)
    snapshot = Snapshot()
    try:
        live_xml, initial_state = snapshot.capture()
    finally:
        snapshot.destroy_node()
    if hashlib.sha256(live_xml.encode()).hexdigest() != collision.hashes["robot_description"]:
        raise RuntimeError("live_robot_description_changed_since_preflight")
    check = collision.check_reference(initial_state["q"], interrupted)
    del collision
    monitor = CollisionMonitor(assets, interrupted, folder/"collision_log.csv")
    gc_enabled = gc.isenabled()
    gc.collect()
    gc.disable()
    node = SixGuard(folder, model, None)
    next_status = 0.0
    previous_loop = time.monotonic()
    run["collision_policy"] = "latest-state overwrite mailbox in separate process; stale, failed or missing results stop motion; thresholds unchanged"
    run["gc_policy"] = "collect before node startup; cyclic GC disabled for bounded guard loop and restored at shutdown"
    try:
        while rclpy.ok():
            spin_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.002)
            now = time.monotonic()
            run["maximum_spin_wall_s"] = max(run.get("maximum_spin_wall_s", 0), now-spin_start)
            run["maximum_loop_gap_wall_s"] = max(run.get("maximum_loop_gap_wall_s", 0), now-previous_loop)
            previous_loop = now
            if interrupted:
                raise RuntimeError("signal_"+str(interrupted[0]))
            if node.fault:
                raise RuntimeError(node.fault)
            node.check_controllers(now)
            if node.count_publishers(C["command_topic"]) > 1 or node.count_publishers(C["request_topic"]) > 1:
                raise RuntimeError("multiple_command_or_request_publishers")
            if now-node.started > C["maximum_wall"]:
                raise RuntimeError("guard_wall_timeout")
            if node.phase == "waiting":
                if now-node.started > 10:
                    raise RuntimeError("guard_startup_timeout")
                ready = (node.last_controller_check is not None and node.last_receive is not None
                         and now-node.last_receive < C["feedback_timeout"] and node.last_sim-node.first_sim >= 0.05
                         and node.publisher.get_subscription_count() > 0 and node.velocity is not None)
                if not ready:
                    continue
                if np.max(np.abs(node.velocity)) > 0.001:
                    raise RuntimeError("guard_initial_not_stationary")
                if not monitor.update(node.positions, node.last_receive):
                    continue
                node.origin = np.asarray(node.positions)
                node.lower, node.upper = model.bounds(node.origin)
                initial = json.loads((assets/"preflight.json").read_text())["initial_q_rad"]
                if np.max(np.abs(node.origin-np.asarray(initial))) > 0.002:
                    raise RuntimeError("initial_pose_drift_from_preflight")
                if np.max(np.abs(node.origin-initial_state["q"])) > 0.0002:
                    raise RuntimeError("robot_moved_during_reference_precheck")
                run["reference_check"] = check
                run["initial_q_rad"] = node.origin.tolist()
                node.min_position, node.max_position = list(node.positions), list(node.positions)
                node.phase, node.status = "hold", "ready"
                print("READY six_joint_guard", flush=True)
                continue
            if now-node.last_receive > C["feedback_timeout"] or now-node.last_advance > C["stall_timeout"]:
                raise RuntimeError("feedback_stale_or_simulation_stalled")
            if node.last_controller_check is None or now-node.last_controller_check > 3:
                raise RuntimeError("controller_status_stale")
            if node.complete:
                run.update(status="completed", reason="motion_complete_received")
                break
            if node.last_request_wall is not None:
                if now-node.last_request_wall > C["request_timeout"]:
                    raise RuntimeError("request_stream_lost_without_completion")
                if abs(node.last_sim-node.last_request_stamp) > C["stamp_tolerance"]:
                    raise RuntimeError("request_source_sim_stamp_stale")
            monitor.update(node.positions, node.last_receive)
            node.emit(time.monotonic())
            if now >= next_status:
                node.announce()
                next_status = now+0.05
    except Exception as exc:
        run.update(status="failed", reason=str(exc))
    finally:
        node.status = "stopping"
        node.announce()
        try:
            run["stop_check"] = node.stop_and_check() if node.sent else dict(verified=False, reason="no_zero_or_motion_published")
        except Exception as exc:
            run["stop_check"] = dict(verified=False, reason=str(exc))
        if not run["stop_check"]["verified"]:
            run.update(status="failed", reason="stop_not_verified; "+str(run.get("reason")))
        run.update(accepted_requests=node.accepted, motion_complete_received=node.complete,
                   max_actual_speed_rad_s=node.max_speed.tolist(), peak_excursion_rad=node.peak_excursion.tolist(),
                   final_offset_rad=None if node.origin is None else (np.asarray(node.positions)-node.origin).tolist(),
                   collision_checks=monitor.checks, collision_timing=monitor.diagnostics,
                   maximum_collision_compute_wall_s=monitor.max_compute,
                   minimum_self_clearance_m=min(check["minimum_self_clearance_m"], monitor.minimum_self),
                   minimum_floor_clearance_m=min(check["minimum_floor_clearance_m"], monitor.minimum_floor),
                   max_receive_gap_wall_s=node.max_gap)
        for f in (node.feedback_file, node.command_file, node.requests_file, node.output_file):
            f.close()
        node.destroy_node()
        monitor.close()
        if gc_enabled:
            gc.enable()


class SixTracking(Node):
    def __init__(self, folder):
        super().__init__("p2_six_tracking")
        self.started = time.monotonic()
        self.latest = None
        self.fault = None
        self.guard_state = None
        self.guard_wall = None
        self.advance = None
        self.origin = None
        self.last_control_wall = None
        self.last_source_sim = None
        self.control_sequence = 0
        self.publish_sequence = 0
        self.files = []
        self.state_log = self.open_log(folder/"joint_log.csv", ["wall_elapsed_s", "sim_stamp_s"]+[j+"_position_rad" for j in JOINTS]+[j+"_velocity_rad_s" for j in JOINTS])
        self.request_log = self.open_log(folder/"request_log.csv", ["sequence", "control_sequence", "publish_wall_elapsed_s", "source_sim_stamp_s", "completion"]+[j+"_request_rad_s" for j in JOINTS])
        self.pub = self.create_publisher(JointState, C["request_topic"], qos_profile_sensor_data)
        self.sub = self.create_subscription(JointState, "/joint_states", self.receive, qos_profile_sensor_data)
        self.status_sub = self.create_subscription(String, C["status_topic"], self.status_receive, 10)

    def open_log(self, path, keys):
        f = path.open("w", newline="", encoding="utf-8")
        self.files.append(f)
        writer = csv.writer(f)
        writer.writerow(keys)
        return writer

    def receive(self, msg):
        try:
            state = decode_state(msg)
            if self.latest is not None and state["sim"] < self.latest["sim"]:
                raise RuntimeError("simulation_time_reversed")
            if self.latest is None or state["sim"] > self.latest["sim"]:
                self.advance = state["wall"]
            self.latest = state
            self.state_log.writerow([state["wall"]-self.started, state["sim"]]+state["q"].tolist()+state["v"].tolist())
        except Exception as exc:
            self.fault = str(exc)

    def status_receive(self, msg):
        self.guard_state = json.loads(msg.data)
        self.guard_wall = time.monotonic()

    def publish(self, values, complete=False):
        if self.latest is None:
            return
        now = time.monotonic()
        if np.any(values != 0):
            if (now-self.latest["wall"] > C["feedback_timeout"] or self.last_control_wall is None
                    or now-self.last_control_wall > C["feedback_timeout"]
                    or self.last_source_sim is None or self.latest["sim"]-self.last_source_sim > C["maximum_cycle"]):
                raise RuntimeError("publish_blocked_stale_feedback_or_control")
        msg = JointState()
        msg.name = JOINTS
        msg.header.stamp = self.latest["stamp"]
        msg.header.frame_id = "p2_six_complete" if complete else ""
        msg.velocity = [float(v) for v in values]
        self.pub.publish(msg)
        self.publish_sequence += 1
        self.request_log.writerow([self.publish_sequence, self.control_sequence, time.monotonic()-self.started,
                                  self.latest["sim"], complete]+list(msg.velocity))


def tracking(folder, assets, interrupted, run, mode, injection, crash):
    model, _ = read_model(assets, with_collision=False)
    mpc = None
    rows = []
    next_publish = 0.0
    start_sim = last_sim = None
    previous = np.zeros(6)
    complete_at = None
    injected = False
    progress_file = folder/"progress.json"
    next_progress = 0.0
    next_console_sim = 0.0
    previous_loop = time.monotonic()
    gc_enabled = gc.isenabled()
    gc_events = []
    gc_start = []
    def record_gc(phase, info):
        if phase == "start":
            gc_start.append(time.monotonic())
        elif gc_start:
            gc_events.append(dict(generation=info["generation"], elapsed_wall_s=time.monotonic()-gc_start.pop()))
    gc.callbacks.append(record_gc)
    gc.collect()
    gc.disable()
    node = SixTracking(folder)
    run["gc_policy"] = "collect before node startup then disable cyclic GC for bounded control loop; restore at shutdown"
    try:
        while rclpy.ok():
            spin_started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.002)
            now = time.monotonic()
            run["maximum_spin_wall_s"] = max(run.get("maximum_spin_wall_s", 0), now-spin_started)
            loop_gap, previous_loop = now-previous_loop, now
            run["maximum_loop_gap_wall_s"] = max(run.get("maximum_loop_gap_wall_s", 0), loop_gap)
            if interrupted:
                raise RuntimeError("signal_"+str(interrupted[0]))
            if node.fault:
                raise RuntimeError(node.fault)
            if now-node.started > C["maximum_wall"]:
                raise RuntimeError("tracking_wall_timeout")
            if start_sim is None:
                if now-node.started > 10:
                    raise RuntimeError("tracking_startup_timeout")
                if node.latest is None or node.guard_state is None or node.guard_state["state"] != "ready" or node.pub.get_subscription_count() == 0:
                    continue
                node.origin = np.asarray(node.guard_state["origin"])
                if np.max(np.abs(node.latest["q"]-node.origin)) > 0.001 or np.max(np.abs(node.latest["v"])) > 0.001:
                    raise RuntimeError("tracking_initial_state_mismatch")
                mpc = SixMpc(model, node.origin)
                run["initial_q_rad"] = node.origin.tolist()
                start_sim = node.latest["sim"]
                previous_loop = time.monotonic()
                print("六关节闭环开始：", mode, flush=True)
            elif loop_gap > C["feedback_timeout"]:
                raise RuntimeError("control_loop_wall_gap_exceeded")
            state = node.latest
            if now-state["wall"] > C["feedback_timeout"] or now-node.advance > C["stall_timeout"]:
                raise RuntimeError("feedback_stale_or_simulation_stalled")
            if complete_at is None and (node.guard_wall is None or now-node.guard_wall > 0.3
                    or node.guard_state["state"] != "ready" or node.guard_state.get("fault") is not None
                    or node.pub.get_subscription_count() == 0):
                raise RuntimeError("guard_not_fresh_or_ready")
            if node.count_publishers(C["request_topic"]) > 1:
                raise RuntimeError("another_six_request_publisher")
            if np.any(state["q"] < mpc.lower) or np.any(state["q"] > mpc.upper):
                raise RuntimeError("position_or_excursion_limit")
            elapsed = state["sim"]-start_sim
            if elapsed >= next_console_sim:
                print("进度：", mode, "仿真", round(elapsed, 2), "/", C["duration"]+C["settle"], "秒", flush=True)
                next_console_sim = elapsed+2.0
            if elapsed >= C["duration"]+C["settle"]:
                if complete_at is None:
                    complete_at = now
                    run["completion_sent"] = True
                if now-complete_at >= 0.5:
                    run.update(status="degraded" if any(r["fallback"] for r in mpc.records) else "completed", reason="reference_finished")
                    break
                if now >= next_publish:
                    node.publish(np.zeros(6), complete=True)
                    next_publish = now+C["heartbeat"]
                continue
            dt = C["ts"] if last_sim is None else state["sim"]-last_sim
            if last_sim is None or dt >= C["ts"]-1e-9:
                if dt > C["maximum_cycle"]:
                    raise RuntimeError("control_cycle_gap_exceeded")
                fault = injection if not injected and elapsed >= 2.0 else "none"
                if fault != "none":
                    injected = True
                compute_start = time.perf_counter()
                command = mpc.command(state["q"], previous, elapsed, dt, mode, fault)
                target, velocity = reference(elapsed, node.origin)
                record = dict(sequence=node.control_sequence+1, elapsed_sim_s=elapsed, source_sim_stamp_s=state["sim"],
                              cycle_sim_s=dt, compute_wall_s=time.perf_counter()-compute_start)
                run["maximum_control_compute_wall_s"] = max(run.get("maximum_control_compute_wall_s", 0), record["compute_wall_s"])
                for j, q, v, ref, vref, cmd in zip(JOINTS, state["q"], state["v"], target, velocity, command):
                    record.update({j+"_position_rad":q, j+"_velocity_rad_s":v, j+"_reference_rad":ref,
                                   j+"_reference_velocity_rad_s":vref, j+"_request_rad_s":cmd, j+"_error_rad":q-ref})
                rows.append(record)
                previous, last_sim = command, state["sim"]
                node.last_source_sim, node.last_control_wall = last_sim, now
                node.control_sequence += 1
            if now >= next_publish:
                publish_started = time.monotonic()
                node.publish(previous)
                run["maximum_publish_wall_s"] = max(run.get("maximum_publish_wall_s", 0), time.monotonic()-publish_started)
                next_publish = time.monotonic()+C["heartbeat"]
                if crash and now >= next_progress:
                    write_json(progress_file, dict(elapsed_sim_s=elapsed, last_request=previous.tolist(), crash_test=crash))
                    next_progress = now+0.05
    except Exception as exc:
        run.update(status="failed", reason=str(exc))
    finally:
        if not run.get("completion_sent"):
            try:
                node.publish(np.zeros(6))
            except Exception as exc:
                run["final_zero_error"] = str(exc)
        for f in node.files:
            f.close()
        if mpc is not None:
            run.update(fallback_count=sum(r["fallback"] for r in mpc.records),
                       injected_fallback_count=sum(r["fallback"] and r["injection"] != "none" for r in mpc.records),
                       uninjected_fallback_count=sum(r["fallback"] and r["injection"] == "none" for r in mpc.records))
            for name, records in [("solver_log.csv", mpc.records), ("control_log.csv", rows)]:
                if records:
                    with (folder/name).open("w", newline="", encoding="utf-8") as f:
                        writer = csv.DictWriter(f, fieldnames=list(records[0]))
                        writer.writeheader()
                        writer.writerows(records)
        run["control_samples"] = len(rows)
        run["gc_events"] = gc_events
        gc.callbacks.remove(record_gc)
        if gc_enabled:
            gc.enable()
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("role", choices=["preflight", "guard", "tracking"])
    parser.add_argument("--folder", type=Path, required=True)
    parser.add_argument("--assets", type=Path)
    parser.add_argument("--controller", choices=["baseline", "mpc"], default="mpc")
    parser.add_argument("--inject", choices=["none", "failure", "timeout"], default="none")
    parser.add_argument("--crash-test", action="store_true")
    args = parser.parse_args()
    interrupted = []
    handlers = {s:signal.signal(s, lambda number, frame: interrupted.append(number)) for s in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    run = dict(status="running", reason=None, role=args.role, controller=args.controller, injection=args.inject,
               completion_sent=False, started_at=datetime.now().astimezone().isoformat(), config=C,
               source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               core_sha256=hashlib.sha256(Path(__file__).with_name("p2_six_core.py").read_bytes()).hexdigest(),
               dependencies={p:importlib.metadata.version(p) for p in ["numpy", "scipy", "osqp", "trimesh", "python-fcl"]})
    try:
        if args.role == "preflight":
            preflight(args.folder, interrupted)
            run["status"] = "completed"
        else:
            if args.assets is None:
                raise RuntimeError("assets_required")
            args.folder.mkdir(parents=True, exist_ok=False)
            write_json(args.folder/"run.json", run)
            if args.role == "guard":
                guard(args.folder, args.assets, interrupted, run)
            else:
                tracking(args.folder, args.assets, interrupted, run, args.controller, args.inject, args.crash_test)
    except Exception as exc:
        run.update(status="failed", reason=str(exc))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        for s, handler in handlers.items():
            signal.signal(s, handler)
        run["finished_at"] = datetime.now().astimezone().isoformat()
        if args.folder.exists():
            write_json(args.folder/"run.json", run)
        print(json.dumps(run, indent=2), flush=True)
    return 0 if run["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
