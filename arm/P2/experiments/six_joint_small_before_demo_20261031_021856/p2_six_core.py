import hashlib
import itertools
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import osqp
from scipy import sparse


JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
C = dict(ts=0.02, horizon=25, kp=2.0, q_weight=100.0, v_weight=1.0, du_weight=0.1,
         vmax=[0.02]*6, rate=[0.1]*6, scales=[0.01, -0.006, 0.008, -0.006, 0.006, -0.008],
         periods=[8.0, 8.0, 4.0, 8.0, 4.0, 8.0], duration=16.0, settle=1.0,
         excursion=0.035, joint_margin=0.05, clearance=0.003,
         feedback_timeout=0.2, stall_timeout=0.5, request_timeout=0.3, stamp_tolerance=0.15,
         heartbeat=0.02, collision_period=0.05, maximum_cycle=0.1, maximum_wall=100.0,
         solve_budget=0.02, request_topic="/p2/six_joint_velocity_request",
         status_topic="/p2/six_guard_status", command_topic="/forward_velocity_controller/commands",
         model="q_next=q+Ts*u, six independent kinematic integrators; no torque dynamics or Cartesian obstacle constraints",
         limitations="simulation only; sampled mesh clearance and floor checks, not continuous collision avoidance; no guard crash watchdog; emergency zero bypasses slew limit")


def reference(elapsed, q0):
    t = np.asarray(elapsed, dtype=float)
    active = t < C["duration"]
    omega = 2*np.pi/np.asarray(C["periods"])
    a = np.asarray(C["scales"])
    phase = np.minimum(t, C["duration"])[..., None]*omega
    q = np.asarray(q0)+a*(1-np.cos(phase))*np.asarray(active)[..., None]
    v = a*omega*np.sin(phase)*np.asarray(active)[..., None]
    return q, v


def limit_command(raw, previous, dt):
    vmax = np.asarray(C["vmax"])
    change = np.asarray(C["rate"])*dt
    return np.clip(raw, np.maximum(-vmax, previous-change), np.minimum(vmax, previous+change))


def transform(xyz=None, rpy=None):
    xyz = [0.0]*3 if xyz is None else xyz
    r, p, y = [0.0]*3 if rpy is None else rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = xyz
    return t


def origin(element):
    if element is None:
        return np.eye(4)
    return transform([float(v) for v in element.get("xyz", "0 0 0").split()],
                     [float(v) for v in element.get("rpy", "0 0 0").split()])


def axis_rotation(axis, angle):
    a = np.asarray(axis, dtype=float)
    a /= np.linalg.norm(a)
    x, y, z = a
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    t = np.eye(4)
    t[:3, :3] = np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)
    return t


class RobotModel:
    def __init__(self, xml):
        self.xml = xml
        self.root = ET.fromstring(xml)
        self.links = {e.get("name"): e for e in self.root.findall("link")}
        self.joints = self.root.findall("joint")
        children = {j.find("child").get("link") for j in self.joints}
        roots = set(self.links)-children
        if len(roots) != 1:
            raise RuntimeError("ambiguous_robot_root")
        self.base = roots.pop()
        by_name = {j.get("name"): j for j in self.joints}
        self.lower, self.upper, self.speed = [], [], []
        for name in JOINTS:
            j = by_name[name]
            if j.get("type") != "revolute":
                raise RuntimeError("unsupported_joint_type: "+name)
            lim = j.find("limit")
            self.lower.append(float(lim.get("lower")))
            self.upper.append(float(lim.get("upper")))
            self.speed.append(float(lim.get("velocity")))
        self.lower = np.asarray(self.lower)+C["joint_margin"]
        self.upper = np.asarray(self.upper)-C["joint_margin"]
        self.speed = np.asarray(self.speed)
        if np.any(np.asarray(C["vmax"]) > self.speed):
            raise RuntimeError("experiment_speed_exceeds_urdf_limit")

    def poses(self, q):
        values = dict(zip(JOINTS, q))
        poses = {self.base: np.eye(4)}
        pending = list(self.joints)
        while pending:
            remaining = []
            for j in pending:
                parent, child = j.find("parent").get("link"), j.find("child").get("link")
                if parent not in poses:
                    remaining.append(j)
                    continue
                t = origin(j.find("origin"))
                if j.get("type") != "fixed":
                    if j.get("name") not in values:
                        raise RuntimeError("unexpected_moving_joint")
                    axis = j.find("axis")
                    t = t@axis_rotation([float(v) for v in ("1 0 0" if axis is None else axis.get("xyz")).split()], values[j.get("name")])
                poses[child] = poses[parent]@t
            if len(remaining) == len(pending):
                raise RuntimeError("disconnected_urdf_tree")
            pending = remaining
        return poses

    def bounds(self, q0):
        return np.maximum(self.lower, np.asarray(q0)-C["excursion"]), np.minimum(self.upper, np.asarray(q0)+C["excursion"])


class CollisionModel:
    def __init__(self, model, srdf_xml, package_path):
        import fcl
        import trimesh
        self.fcl = fcl
        self.model = model
        self.hashes = {"robot_description": hashlib.sha256(model.xml.encode()).hexdigest(),
                       "srdf": hashlib.sha256(srdf_xml.encode()).hexdigest()}
        disabled = {frozenset([e.get("link1"), e.get("link2")]) for e in ET.fromstring(srdf_xml).iter("disable_collisions")}
        self.parts = []
        self.models = []
        moving_links = set()
        for j in model.joints:
            if j.get("type") != "fixed":
                moving_links.add(j.find("child").get("link"))
        for name, link in model.links.items():
            for collision in link.findall("collision"):
                g = collision.find("geometry")
                mesh = g.find("mesh")
                if mesh is None:
                    box = g.find("box")
                    if name == "ground_plane" and box is not None and box.get("size").split() == ["5", "5", "0"]:
                        pose = model.poses(np.zeros(6))[name]@origin(collision.find("origin"))
                        if np.max(np.abs(pose[:3, :3]-np.eye(3))) < 1e-9 and pose[2, 3] <= 0:
                            continue
                    raise RuntimeError("unsupported_collision_primitive: "+name)
                uri = mesh.get("filename")
                if uri.startswith("package://"):
                    package, relative = uri[10:].split("/", 1)
                    path = Path(package_path(package))/relative
                elif uri.startswith("file://"):
                    path = Path(uri[7:])
                else:
                    path = Path(uri)
                self.hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
                shape = trimesh.load(str(path), force="mesh", process=False)
                vertices = np.asarray(shape.vertices, dtype=float)*np.asarray([float(v) for v in mesh.get("scale", "1 1 1").split()])
                bv = fcl.BVHModel()
                bv.beginModel(len(shape.faces), len(vertices))
                bv.addSubModel(vertices, np.asarray(shape.faces, dtype=np.int32))
                bv.endModel()
                obj = fcl.CollisionObject(bv)
                self.models.append(bv)
                self.parts.append(dict(link=name, local=origin(collision.find("origin")), vertices=vertices,
                                       obj=obj, moving=name in moving_links))
        self.pairs = [(a, b) for a, b in itertools.combinations(self.parts, 2)
                      if a["link"] != b["link"] and frozenset([a["link"], b["link"]]) not in disabled]
        self.minimum_self = float("inf")
        self.minimum_floor = float("inf")

    def check(self, q):
        if not np.all(np.isfinite(q)) or np.any(q < self.model.lower) or np.any(q > self.model.upper):
            raise RuntimeError("joint_position_limit")
        fcl = self.fcl
        poses = self.model.poses(q)
        floor = float("inf")
        for part in self.parts:
            t = poses[part["link"]]@part["local"]
            part["obj"].setTransform(fcl.Transform(t[:3, :3], t[:3, 3]))
            if part["moving"]:
                z = part["vertices"]@t[2, :3]+t[2, 3]
                floor = min(floor, float(z.min()))
        minimum = float("inf")
        for a, b in self.pairs:
            if fcl.collide(a["obj"], b["obj"], fcl.CollisionRequest(), fcl.CollisionResult()):
                raise RuntimeError("self_collision: "+a["link"]+" / "+b["link"])
            distance = fcl.distance(a["obj"], b["obj"], fcl.DistanceRequest(), fcl.DistanceResult())
            if not math.isfinite(distance) or distance < C["clearance"]:
                raise RuntimeError("self_clearance_below_margin: "+a["link"]+" / "+b["link"])
            minimum = min(minimum, distance)
        if floor < C["clearance"]:
            raise RuntimeError("floor_clearance_below_margin")
        self.minimum_self = min(self.minimum_self, minimum)
        self.minimum_floor = min(self.minimum_floor, floor)
        return dict(self_clearance_m=minimum, floor_clearance_m=floor)

    def check_reference(self, q0, interrupted=None):
        lower, upper = self.model.bounds(q0)
        samples, _ = reference(np.arange(0, C["duration"]+C["settle"]+0.001, 0.04), q0)
        started = time.monotonic()
        for q in samples:
            if interrupted:
                raise RuntimeError("reference_precheck_interrupted")
            if np.any(q < lower) or np.any(q > upper):
                raise RuntimeError("reference_outside_position_bounds")
            self.check(q)
        return dict(samples=len(samples), sample_step_sim_s=0.04, elapsed_wall_s=time.monotonic()-started,
                    minimum_self_clearance_m=self.minimum_self, minimum_floor_clearance_m=self.minimum_floor,
                    note="sampled nominal joint reference; excludes SRDF-disabled link pairs; not a swept-volume or tracking-tube guarantee")


class SixMpc:
    def __init__(self, model, q0):
        self.q0 = np.asarray(q0)
        self.lower, self.upper = model.bounds(q0)
        n = C["horizon"]
        s = C["ts"]*np.tril(np.ones((n, n)))
        d = np.eye(n)-np.eye(n, k=-1)
        self.s = sparse.kron(sparse.eye(6), sparse.csc_matrix(s), format="csc")
        self.d = sparse.kron(sparse.eye(6), sparse.csc_matrix(d), format="csc")
        self.eye = sparse.eye(6*n, format="csc")
        h = C["q_weight"]*(self.s.T@self.s)+C["v_weight"]*self.eye+C["du_weight"]*(self.d.T@self.d)
        self.solver = osqp.OSQP()
        self.solver.setup(P=sparse.triu(2*h, format="csc"), q=np.zeros(6*n),
                          A=sparse.vstack([self.eye, self.d, self.s], format="csc"),
                          l=-np.ones(18*n), u=np.ones(18*n), verbose=False,
                          eps_abs=1e-6, eps_rel=1e-6, max_iter=10000, time_limit=C["solve_budget"])
        self.records = []

    def command(self, q, previous, elapsed, dt, mode="mpc", injection="none"):
        started = time.perf_counter()
        target, velocity = reference(elapsed, self.q0)
        raw = velocity+C["kp"]*(target-q)
        status, fallback, iterations = "baseline_no_solver", False, 0
        if mode == "mpc":
            n = C["horizon"]
            targets, velocities = reference(elapsed+(np.arange(n)+1)*C["ts"], self.q0)
            prior = np.zeros((6, n))
            prior[:, 0] = previous
            prior = prior.ravel()
            repeat_q = np.repeat(q, n)
            linear = C["q_weight"]*self.s.T@(repeat_q-targets.T.ravel())
            linear -= C["v_weight"]*velocities.T.ravel()
            linear -= C["du_weight"]*self.d.T@prior
            deltas = np.repeat(np.asarray(C["rate"])*C["ts"], n)
            deltas[::n] = np.asarray(C["rate"])*dt
            bounds = np.repeat(C["vmax"], n)
            lower = np.r_[-bounds, prior-deltas, np.repeat(self.lower, n)-repeat_q]
            upper = np.r_[bounds, prior+deltas, np.repeat(self.upper, n)-repeat_q]
            try:
                if injection == "failure":
                    raise RuntimeError("injected_solver_exception")
                self.solver.update(q=2*linear, l=lower, u=upper)
                result = self.solver.solve()
                if injection == "timeout":
                    time.sleep(0.025)
                status, iterations = result.info.status, result.info.iter
                if result.info.status_val != 1 or result.x is None or not np.all(np.isfinite(result.x)):
                    raise RuntimeError("solver_not_solved: "+status)
                candidate = np.r_[result.x, self.d@result.x, self.s@result.x]
                if np.any(candidate < lower-1e-5) or np.any(candidate > upper+1e-5):
                    raise RuntimeError("predicted_constraint_residual_exceeded")
                raw = result.x[::n].copy()
            except Exception as exc:
                status, fallback = str(exc), True
            if time.perf_counter()-started > C["solve_budget"]:
                status += "; wall_time_limit_exceeded"
                fallback = True
            if fallback:
                raw = velocity+C["kp"]*(target-q)
        command = limit_command(raw, previous, dt)
        predicted = q+dt*command
        if np.any(predicted < self.lower) or np.any(predicted > self.upper):
            raise RuntimeError("next_request_predicted_position_outside_bounds")
        self.records.append(dict(elapsed_sim_s=float(elapsed), solve_wall_s=time.perf_counter()-started,
                                 status=status, fallback=fallback, injection=injection, iterations=iterations,
                                 limiter_correction_max_rad_s=float(np.max(np.abs(command-raw)))))
        return command
