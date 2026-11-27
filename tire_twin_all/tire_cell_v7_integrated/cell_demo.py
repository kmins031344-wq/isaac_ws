# Tire cell demo v6 for Isaac Sim 5.x  (run via run_cell_demo.py, or in Script Editor: PLAY then run)
# Scene: tire_cell_v3.usda
# One cycle ("한 템포"):
#   1. Robot B (RS080N + QST62-230CT nutrunner): loosens the 5 nuts in star order, puts each one in the nut tray
#   2. Robot A (RS080N + 4-jaw wheel gripper)  : pulls the wheel off the studs and lays it on the floor
#   -- pause (PAUSE_S, default 2 s) --
#   3. Robot A: exact reverse of 2 (picks the wheel up from the floor, pushes it back onto the studs)
#   4. Robot B: exact reverse of 1 (takes the nuts from the tray and tightens them in reverse order 4-2-5-3-1)
# The reverse half replays the joint paths recorded during removal backwards, so it only uses poses that were already
# reached during removal. The only new motion is a few-mm correction for how the wheel sits in the jaws after the floor.
#
# What is physical here: car body (dynamic, on lift pads), 4 suspension corners (spring/damper), hub pilot + 5 studs,
# the 22 kg wheel hanging on them by contact, the jaws squeezing the tire. What is NOT simulated: nut threads/torque
# (the 5 nuts are one FixedJoint "nut_clamp", enabled/disabled after the nut sequence), the nutrunner's 50 mm spindle travel.
#
# Vision (VISION=True): before step 1, robot B stops at an observation pose, its wrist D455 takes ONE colour+depth frame,
# and nut_vision.py finds the hub axis, wheel centre and the 5 nut positions. Both robots plan from that measurement
# instead of the wheel's physics pose ("look, then move": vision runs once per wheel, so it never slows the motion).
# If vision fails or the result is implausible, the demo falls back to the measured physics pose and says so.
#
# Motion is kinematic: joint position targets from the IK in cell_kinematics.py (no torque control).
# Extension points: MotionBackend (swap the IK / send targets to ROS 2), ContactMonitor (collision evidence), params in scene customData.
import asyncio
import json
import os
import time

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

try:
    from cell_kinematics import ARM_JOINTS, Arm, R_to_quat, T_from, cart_path, mat, min_duration, quat_to_R, sample, smooth
except ImportError:                                   # Script Editor: add this file's folder to sys.path
    import sys
    for _p in (os.path.dirname(os.path.abspath(globals().get("__file__", ""))), os.path.expanduser("~/cobot3_ws/isaac_assets/v3")):
        if os.path.exists(os.path.join(_p, "cell_kinematics.py")) and _p not in sys.path:
            sys.path.insert(0, _p)
    from cell_kinematics import ARM_JOINTS, Arm, R_to_quat, T_from, cart_path, mat, min_duration, quat_to_R, sample, smooth

# ----------------------------------------------------------------------------- options (can be overridden by the launcher)
GRASP_MODE = "physics"        # "physics": contact only | "joint": FixedJoint after closing (fallback)
MOUNT_MODE = None             # None: use scene value | "contact": wheel hangs on pilot+studs | "joint": prismatic stud joint (fallback)
SKIP_NUTS = False             # True: skip robot B
SAFETY = True                 # protective stop when the car body moves or a tool/wheel hits the car (limits in params.json)
SPEED = 1.0
TRACK_TO_REAR = True          # after the front wheel: both carriages travel along the track to the rear wheel
JAW_OPEN, JAW_APPROACH = 0.0, 0.012
APPROACH_A = 0.42             # gripper pre-grasp distance along the axle [m]
APPROACH_B = 0.10             # socket stand-off before engaging a nut [m]
PLACE_XY = (1.00, 1.05)
PLACE_CLEAR = 0.004
ABOVE_Z = 0.75
LOG_DIR = None                # where contact_log.csv is written (default: this file's folder)
PAUSE_S = 2.0                 # wait between "wheel off" and the reverse (mounting) half [s]
REASSEMBLE = True             # False: stop after the wheel is on the floor (v4 behaviour)
VISION = True                 # find the wheel/nuts with robot B's wrist camera (False: use the wheel's physics pose)
VISION_RES = (1280, 800)      # capture resolution (only rendered at the moment of capture)
VISION_DIST = 0.55            # camera distance from the nut-top plane at the observation pose [m]
VISION_FRAMES = 1             # depth frames averaged per capture
VISION_NOISE_MM = 0.0         # Gaussian depth noise added to the simulated depth (real D455: ~1 mm at 0.5 m)
VISION_MAX_SHIFT = 0.08       # reject a vision result further than this from the layout pose [m]
VISION_MAX_TILT = 5.0         # ... or with a hub axis tilted more than this [deg]
VISION_ROS = False            # also publish the result to ROS 2 (rqt / RViz): vision_ros.py. run_cell_demo turns it on with ROS
DO_FRONT = True               # service the front-left wheel (False = start with the rear wheel; for testing)
REAR = None                   # rear wheel config from rear_wheel.add_removable_wheel(); None = no rear wheel service
SCAN_DIST = 0.9               # robot B camera distance from the car side while scanning along the track [m]
SCAN_SPEED = 0.30             # carriage speed while scanning [m/s]
SCAN_ACCEL = 0.4              # carriage acceleration/deceleration while scanning [m/s^2]
SCAN_AIM_SHIFT = 0.6          # aim the scan camera this far from the wheel toward robot B's side [m]: aiming straight at the wheel
                              # from 0.9 m puts robot B's forearm into robot A's space (A blocked it). The wheel stays in view.
REAR_PLACE_REL = (1.10, 0.40) # where robot A lays the rear wheel, relative to its base (x, along the rail) [m]
REAR_GRIP_ROLL = -7.0         # rear wheel: turn the gripper about the axle so the 4 fingers miss the tighter rear wheel arch [deg]
REAR_JAW_APPROACH = 0.016     # rear wheel: jaws a little more closed while the fingers slide past the arch [m] (front: JAW_APPROACH)
REAR_RETURN_VIA = (-20.0, -140.0, 0.0, 70.0, 0.0)   # rear wheel: robot A folds back toward its base (joints 2-6) between the
                              # floor spot and home. The front path (via the pre-grasp pose) passes 6 mm from the rear body; this 148 mm

SEED_A_GRASP = [0.0, -10.0, -125.0, 0.0, 30.0, 0.0]
SEED_A_PLACE = [-45.0, 15.0, -90.0, 0.0, -75.0, 45.0]
SEED_B_NUT = [-37.5, 15.0, -103.8, 85.9, 82.6, -60.9]

R_TOOL_X = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)      # tool axis (tcp -Z) -> world +X
R_TOOL_DOWN = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)    # tool axis -> world -Z


def cell(stage, key, default=None):
    v = stage.GetPrimAtPath("/World").GetCustomDataByKey("cell:" + key)
    return default if v is None else v


# ----------------------------------------------------------------------------- planning (pure; testable offline)
def tool_rotation(n_out):
    """R_TOOL_X turned (minimal rotation) so that the tool axis runs along the measured hub axis n_out (wheel outer normal)"""
    a, b = R_TOOL_X[:, 2], np.asarray(n_out, dtype=float) / np.linalg.norm(n_out)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-12:
        return R_TOOL_X.copy()
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return (np.eye(3) + K + K @ K / (1 + c)) @ R_TOOL_X


def rot_z(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0.0], [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])


def plan_a(arm, wheel_center, tip_below_tcp, stud_exposed, R=R_TOOL_X, place_xy=None, return_via=None):
    """R: tool rotation at the hub (R[:,2] = wheel outer normal; the tool moves along -R[:,2] toward the car)"""
    wc = np.array(wheel_center, dtype=float)
    out = R[:, 2]
    T_grasp = T_from(wc, R)
    T_clear = T_from(wc + out * (stud_exposed + 0.03), R)
    T_pre = T_from(wc + out * APPROACH_A, R)
    px_, py_ = PLACE_XY if place_xy is None else place_xy
    T_place = T_from(np.array([px_, py_, tip_below_tcp + PLACE_CLEAR]), R_TOOL_DOWN)
    T_above = T_from(np.array([px_, py_, ABOVE_Z]), R_TOOL_DOWN)
    q_pre, ok1 = arm.ik(T_pre, np.array(SEED_A_GRASP))
    q_above, ok2 = arm.ik(T_above, np.array(SEED_A_PLACE))
    if not (ok1 and ok2):
        raise RuntimeError("robot A: IK failed for pre-grasp/above-place (ok=%s,%s)" % (ok1, ok2))
    p_in = cart_path(arm, q_pre, T_pre, T_grasp)
    p_out_a = cart_path(arm, p_in[-1], T_grasp, T_clear, n=8)
    p_out_b = cart_path(arm, p_out_a[-1], T_clear, T_pre, n=20)
    p_down = cart_path(arm, q_above, T_above, T_place)
    home = np.zeros(6)
    via = np.array([0.0, -30.0, -120.0, 0.0, 60.0, 0.0])
    tuck = p_out_b[-1].copy()
    tuck[0] = q_above[0]
    return {"home_to_pre": [home, via, q_pre], "approach": p_in, "pull_studs": p_out_a, "pull_clear": p_out_b,
            "pre_to_above": [p_out_b[-1], tuck, q_above], "lower": p_down, "raise": p_down[::-1],
            "above_to_home": [q_above, tuck, home] if return_via is None else [q_above, np.r_[q_above[0], return_via], home],
            "T_pre": T_pre, "T_clear": T_clear, "T_grasp": T_grasp}


def plan_b(arm, nut_points_world, order, R=R_TOOL_X):
    """order: 1-based nut numbers. Returns home->first stand-off, per-nut (in, out, transfer to next), last->home."""
    pts = [np.array(nut_points_world[i - 1], dtype=float) for i in order]
    back = -R[:, 2] * APPROACH_B                     # stand-off is outside the wheel, along the hub axis
    q0, ok = arm.ik(T_from(pts[0] - back, R), np.array(SEED_B_NUT))
    if not ok:
        raise RuntimeError("robot B: IK failed for the first nut stand-off")
    home = np.zeros(6)
    via = np.array([q0[0], -30.0, -120.0, q0[3], q0[4], q0[5]])
    steps, q = [], q0
    for k, p in enumerate(pts):
        T_off, T_on = T_from(p - back, R), T_from(p, R)
        p_in = cart_path(arm, q, T_off, T_on, n=8)
        p_out = p_in[::-1]
        nxt = None
        if k + 1 < len(pts):
            nxt = cart_path(arm, p_out[-1], T_off, T_from(pts[k + 1] - back, R), n=6)
            q = nxt[-1]
        steps.append({"nut": order[k], "in": p_in, "out": p_out, "next": nxt})
    return {"home_to_first": [home, via, q0], "steps": steps, "last_to_home": [steps[-1]["out"][-1], via, home]}


# ----------------------------------------------------------------------------- Isaac Sim runtime
class _Abort(Exception):
    pass


class _Protect(Exception):          # protective stop: something that must not happen was detected
    pass


_STATE = {"run_id": 0}


def run():
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from omni.physx import get_physx_interface

    stage = omni.usd.get_context().get_stage()
    px = get_physx_interface()
    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    my_id = _STATE["run_id"] = _STATE["run_id"] + 1
    # 시뮬레이션 시계: PhysX 스텝 dt 를 누적. 대기·궤적 시간은 이 값으로 잰다 (벽시계로 재면 렌더링이 느릴 때
    # 같은 '1.2 초' 동안 물리가 덜 진행돼서 로봇이 목표에 못 미친 채 다음 단계로 넘어간다)
    simclock = {"t": 0.0}

    def _on_physics_step(dt):
        simclock["t"] += dt
    simclock["sub"] = px.subscribe_physics_step_events(_on_physics_step)

    RA, RB = str(cell(stage, "robot_a", "")), str(cell(stage, "robot_b", ""))
    GA, GB = str(cell(stage, "gripper_path", "")), str(cell(stage, "nutrunner_path", ""))
    WHEEL, CORNER, CAR = str(cell(stage, "wheel_path", "")), str(cell(stage, "corner_path", "")), str(cell(stage, "car_path", ""))
    NUT_CLAMP, STUD_JOINT, GRASP_JOINT = "/World/nut_clamp_joint", "/World/stud_joint", "/World/grasp_joint"
    # every wheel this run may service (front-left always; rear-left if run_cell_demo made it removable)
    FRONT_CFG = {"name": "FL", "label": "", "wheel": WHEEL, "corner": CORNER, "nut_clamp": NUT_CLAMP, "stud_joint": STUD_JOINT,
                 "grasp_joint": GRASP_JOINT, "debug_png": "vision_debug.png"}
    WHEEL_CFGS = [FRONT_CFG]
    if REAR is not None and stage.GetPrimAtPath(REAR["wheel"]).IsValid():
        WHEEL_CFGS.append(dict(REAR, label=REAR["name"] + " ", debug_png="vision_debug_%s.png" % REAR["name"]))
    CUR = dict(FRONT_CFG)                               # the wheel being serviced now

    def log(msg):
        if CUR.get("label") and msg.startswith("["):     # "[1/4 loosen] ..." -> "[RL 1/4 loosen] ..." for the rear wheel
            msg = "[" + CUR["label"] + msg[1:]
        _STATE["step"] = msg.split(":")[0][:48].replace(",", ";")     # contact_log.csv records which step was running
        print("[cell_demo] " + msg, flush=True)

    # ---- MotionBackend: the only place that talks to the simulator's joints. Replace to drive via ROS 2 / another controller.
    class UsdDriveBackend:
        def __init__(self, robot_path):
            self.attr = [stage.GetPrimAtPath(robot_path + "/" + jp).GetAttribute("drive:angular:physics:targetPosition") for jp in ARM_JOINTS]
            self.ok = all(a and a.IsValid() for a in self.attr)

        def command(self, q_model, sign):
            for a, v in zip(self.attr, q_model):
                a.Set(float(sign * v))

    drive_a, drive_b = UsdDriveBackend(RA), UsdDriveBackend(RB)
    jaw_attr = [stage.GetPrimAtPath("%s/joints/jaw_%d_joint" % (GA, i)).GetAttribute("drive:linear:physics:targetPosition") for i in range(4)]
    spin_attr = stage.GetPrimAtPath(GB + "/joints/spindle_joint").GetAttribute("drive:angular:physics:targetVelocity")

    # ---- linear track: one rail, two carriages (one prismatic joint per robot). Targets in metres, 0 = front-wheel station
    track_attr = [stage.GetPrimAtPath(str(cell(stage, k, "/none"))).GetAttribute("drive:linear:physics:targetPosition")
                  for k in ("track_joint_a", "track_joint_b")]
    HAS_TRACK = all(a and a.IsValid() for a in track_attr)
    track_pos = [0.0, 0.0]
    TRACK_LIM = json.loads(str(cell(stage, "track_travel", "[0,0]")))
    TRACK_BASE_Y = json.loads(str(cell(stage, "track_base_y", "[0,0]")))
    TRACK_GAP, TRACK_VMAX = float(cell(stage, "track_min_gap", 1.0)), float(cell(stage, "track_max_speed", 1.0))

    def set_track(da, db):
        track_pos[0], track_pos[1] = float(da), float(db)
        if HAS_TRACK:
            track_attr[0].Set(float(da))
            track_attr[1].Set(float(db))

    guard = {"mon": None, "armed": False}
    try:
        LIM = json.loads(str(cell(stage, "protective_stop", "{}")))
    except Exception:
        LIM = {}

    async def tick():
        t0 = simclock["t"]
        await app.next_update_async()
        if _STATE["run_id"] != my_id or timeline.is_stopped():
            raise _Abort()
        m = guard["mon"]
        if SAFETY and guard["armed"] and m is not None:
            if m.trip is not None:
                raise _Protect("contact force %.0f N > %.0f N: %s (%s <-> %s)" % (m.trip[1], LIM.get("contact_force_N", 300.0), m.trip[0], m.trip[2], m.trip[3]))
            if m.car_dpos * 1000 > LIM.get("car_displacement_mm", 3.0) or m.car_tilt > LIM.get("car_tilt_deg", 0.2):
                raise _Protect("car body moved %.1f mm / %.2f deg" % (m.car_dpos * 1000, m.car_tilt))
        return (simclock["t"] - t0) * SPEED

    async def frames(n):
        for _ in range(n):
            await tick()

    async def sleep(sec):
        t = 0.0
        while t < sec:
            t += await tick()

    def set_jaws(v):
        for a in jaw_attr:
            a.Set(float(v))

    def body_pose(path):
        try:
            d = px.get_rigidbody_transformation(path)
            p, r = d["position"], d["rotation"]
            return T_from(np.array([p[0], p[1], p[2]]), quat_to_R(r[3], r[0], r[1], r[2]))
        except Exception:
            return None

    async def move(drive, arm, path, sec, sign):
        sec = max(sec, min_duration(arm, path))          # never exceed the joints' rated speed
        t = 0.0
        while True:
            s = min(t / sec, 1.0)
            q_cmd = sample(path, smooth(s))
            drive.command(q_cmd, sign)
            if s >= 1.0:
                break
            t += await tick()
        await frames(20)

    async def track_move(da, db, arms=()):
        """move both carriages to da, db [m]. Refuses targets outside the travel or closer than the minimum gap."""
        if not HAS_TRACK:
            raise RuntimeError("scene has no track joints")
        for name, d in (("A", da), ("B", db)):
            if not (TRACK_LIM[0] - 1e-6 <= d <= TRACK_LIM[1] + 1e-6):
                raise RuntimeError("track target %s=%.3f m is outside the travel %s" % (name, d, TRACK_LIM))
        a0, b0 = track_pos
        n = 50
        for k in range(n + 1):                              # gap along the whole move, not only at the ends
            u = smooth(k / n)
            gap = abs((TRACK_BASE_Y[0] + a0 + (da - a0) * u) - (TRACK_BASE_Y[1] + b0 + (db - b0) * u))
            if gap < TRACK_GAP - 1e-6:
                raise RuntimeError("track move would bring the carriages to %.3f m (minimum %.2f m)" % (gap, TRACK_GAP))
        sec = max(abs(da - a0), abs(db - b0)) * 1.5 / TRACK_VMAX      # smoothstep peak speed = 1.5 x average
        t = 0.0
        while sec > 0:
            u = min(t / sec, 1.0)
            set_track(a0 + (da - a0) * smooth(u), b0 + (db - b0) * smooth(u))
            if u >= 1.0:
                break
            t += await tick()
        set_track(da, db)
        await sleep(1.0)
        for arm, d in zip(arms, (da, db)):
            arm.set_track(d)

    async def jaws(a, b, sec):
        t = 0.0
        while True:
            s = min(t / sec, 1.0)
            set_jaws(a + (b - a) * smooth(s))
            if s >= 1.0:
                break
            t += await tick()
        await frames(20)

    def session_edit():
        # runtime switches go to the session layer: it is the strongest layer, so they also win over the rear-wheel joints
        # that rear_wheel.py authored there (a write to the scene layer would be silently overridden)
        return Usd.EditContext(stage, stage.GetSessionLayer())

    def joint_enabled(path, on):
        p = stage.GetPrimAtPath(path)
        if p.IsValid():
            with session_edit():
                p.GetAttribute("physics:jointEnabled").Set(bool(on))

    def nut_visible(i, on):
        p = stage.GetPrimAtPath("%s/nuts/nut_%d" % (CUR["corner"], i))
        if p.IsValid():
            img = UsdGeom.Imageable(p)
            img.MakeVisible() if on else img.MakeInvisible()

    def loose_nut(i, on=False, position=None, rotation=None):
        p = stage.GetPrimAtPath("/World/LooseNuts/nut_%d" % i)
        if not p.IsValid():
            return
        xform = UsdGeom.Xformable(p)
        ops = xform.GetOrderedXformOps()
        if position is not None and ops:
            ops[0].Set(Gf.Vec3d(*[float(v) for v in position]))
        if rotation is not None and len(ops) > 1:
            w, x, y, z = R_to_quat(rotation)
            ops[1].Set(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))
        img = UsdGeom.Imageable(p)
        img.MakeVisible() if on else img.MakeInvisible()

    def carried_nut(i, on):
        p = stage.GetPrimAtPath(GB + "/spindle/carried_nut_%d" % i)
        if p.IsValid():
            img = UsdGeom.Imageable(p)
            img.MakeVisible() if on else img.MakeInvisible()

    def reset_scene_state():
        drive_a.command(np.zeros(6), 1.0)
        drive_b.command(np.zeros(6), 1.0)
        set_track(0.0, 0.0)
        set_jaws(JAW_OPEN)
        spin_attr.Set(0.0)
        for wc in WHEEL_CFGS:
            joint_enabled(wc["grasp_joint"], False)
            joint_enabled(wc["stud_joint"], False)
            joint_enabled(wc["nut_clamp"], True)
            for i in range(5):
                p = stage.GetPrimAtPath("%s/nuts/nut_%d" % (wc["corner"], i))
                if p.IsValid():
                    UsdGeom.Imageable(p).MakeVisible()
        for i in range(5):
            loose_nut(i, False, (0.0, 0.0, 0.0), np.eye(3))
            carried_nut(i, False)

    # ---- ContactMonitor: evidence of collisions (which colliders touched, peak force) and of the car body moving
    class ContactMonitor:
        def __init__(self):
            self.pairs, self.sub, self.t0 = {}, None, simclock["t"]
            self.trip, self.f_limit = None, float(LIM.get("contact_force_N", 300.0))
            self.car0, self.car_dpos, self.car_tilt = None, 0.0, 0.0
            self.dt = 1.0 / float(json.loads(str(cell(stage, "params_json", "{}"))).get("sim", {}).get("time_steps_per_second", 60))
            self.phase = "init"
            try:
                from omni.physx import get_physx_simulation_interface
                self.sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact)
            except Exception as e:
                log("contact report unavailable: %s" % e)

        def _on_contact(self, headers, data):
            try:
                from pxr import PhysicsSchemaTools
                for h in headers:
                    c0, c1 = str(PhysicsSchemaTools.intToSdfPath(h.collider0)), str(PhysicsSchemaTools.intToSdfPath(h.collider1))
                    if "/Lift/" in c0 or "/Lift/" in c1 or "/Ground" in c0 or "/Ground" in c1:
                        continue
                    f = 0.0
                    for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                        imp = data[i].impulse
                        f = max(f, (imp[0] ** 2 + imp[1] ** 2 + imp[2] ** 2) ** 0.5 / self.dt)
                    if f > self.f_limit and self.trip is None:
                        cls = self.classify(c0, c1)
                        if cls.startswith(("TOOL-BODY", "TOOL-HUB", "WHEEL-BODY", "NUTRUNNER")):
                            self.trip = (cls, f, c0.replace("/World/", ""), c1.replace("/World/", ""))
                    key = (c0, c1) if c0 < c1 else (c1, c0)
                    e = self.pairs.get(key)
                    if e is None:
                        self.pairs[key] = {"t": simclock["t"] - self.t0, "phase": "%s / %s" % (self.phase, _STATE.get("step", "")), "n": 1, "fmax": f}
                    else:
                        e["n"] += 1
                        e["fmax"] = max(e["fmax"], f)
            except Exception:
                pass

        def sample_car(self):
            T = body_pose(CAR)
            if T is None:
                return
            if self.car0 is None:
                self.car0 = T
                return
            self.car_dpos = max(self.car_dpos, float(np.linalg.norm(T[:3, 3] - self.car0[:3, 3])))
            c = max(-1.0, min(1.0, (np.trace(self.car0[:3, :3].T @ T[:3, :3]) - 1) / 2))
            self.car_tilt = max(self.car_tilt, float(np.degrees(np.arccos(c))))

        @staticmethod
        def classify(c0, c1):
            a, b = (c0, c1) if ("/WheelGripper/" in c0 or "/Nutrunner/" in c0) else (c1, c0)
            tool = "/WheelGripper/" in a or "/Nutrunner/" in a
            on = lambda x, key: any(x.startswith(wc[key] + "/") for wc in WHEEL_CFGS)
            if tool and b.startswith(CAR + "/"):
                return "TOOL-BODY (scratch risk)"
            if tool and on(b, "corner"):
                return "TOOL-HUB/BRAKE (scratch risk)"
            if "/Nutrunner/" in a and on(b, "wheel"):
                return "NUTRUNNER-WHEEL (scratch risk)"
            if tool and on(b, "wheel"):
                soft = a.endswith("/pad") or "/lip_" in a
                return "gripper pad-tire (expected)" if (soft and "/tire_" in b) else "GRIPPER-WHEEL metal contact (scratch risk)"
            if (on(c0, "wheel") and c1.startswith(CAR + "/")) or (on(c1, "wheel") and c0.startswith(CAR + "/")):
                return "WHEEL-BODY (scratch risk)"
            if (on(c0, "wheel") and on(c1, "corner")) or (on(c1, "wheel") and on(c0, "corner")):
                return "wheel-hub/stud (expected)"
            return "other"

        def report(self):
            rows = []
            for (c0, c1), e in self.pairs.items():
                rows.append((self.classify(c0, c1), e["fmax"], e["n"], e["t"], e["phase"], c0, c1))
            rows.sort(key=lambda r: (0 if "risk" in r[0] else 1, -r[1]))
            risk = [r for r in rows if "risk" in r[0]]
            log("---- contact summary: %d collider pairs, %d flagged as scratch risk ----" % (len(rows), len(risk)))
            for r in risk[:12]:
                log("  %s | peak %.0f N | %d steps | first at %.1fs (%s) | %s <-> %s" % (r[0], r[1], r[2], r[3], r[4], r[5].replace("/World/", ""), r[6].replace("/World/", "")))
            if not risk:
                log("  no tool-to-car or metal-to-wheel contact was reported")
            log("car body max displacement %.2f mm, max tilt %.3f deg" % (self.car_dpos * 1000, self.car_tilt))
            try:
                d = LOG_DIR or (os.path.dirname(os.path.abspath(globals()["__file__"])) if "__file__" in globals() else os.path.expanduser("~"))
                fn = os.path.join(d, "contact_log.csv")
                with open(fn, "w", encoding="utf-8") as f:
                    f.write("class,peak_force_N,steps,first_time_s,phase,collider0,collider1\n")
                    for r in rows:
                        f.write("%s,%.1f,%d,%.2f,%s,%s,%s\n" % r)
                    f.write("car_body_max_displacement_mm,%.3f,,,,,\ncar_body_max_tilt_deg,%.4f,,,,,\n" % (self.car_dpos * 1000, self.car_tilt))
                log("contact log written: " + fn)
            except Exception as e:
                log("could not write contact log: %s" % e)

        def close(self):
            self.sub = None

    # ---- vision: one look with robot B's wrist camera (nut_vision.py / vision_capture.py)
    async def observe_wheel(arm_b, sign, nut_local, T_wheel_physics, T_prior=None):
        """Robot B goes home -> observation pose, takes one colour+depth frame, finds the wheel.
        Returns (T_wheel or None on failure, T_obs or None if B did not move, q_obs).
        The physics pose is used here only to REPORT the vision error (simulation ground truth), never to plan."""
        try:
            import cv2
            import nut_vision as nv
            import vision_capture as vc
        except Exception as e:
            log("vision: modules not available (%s) -> using the physics pose" % e)
            return None, None, None
        if T_prior is None:
            T_prior = _STATE.get("wheel_prior")
        if T_prior is None:
            T_prior = mat(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(CUR["wheel"])))
        nut_local = np.asarray(nut_local, dtype=float)
        z_top = float(np.mean(nut_local[:, 2])) + 0.015         # nut top = socket target + 15 mm (build_scene_v2 nut profile)
        cam = vc.WristCamera(stage, str(cell(stage, "camera_b", GB + "/Camera_RSD455/RSD455")), resolution=VISION_RES)
        T_tcp_cam = vc.tcp_to_camera(stage, GB + "/base/tcp", cam)
        T_obs = vc.observation_tcp(T_prior, z_top, T_tcp_cam, tool_rotation(T_prior[:3, 2]), VISION_DIST)
        q_obs, ok = arm_b.ik(T_obs, np.array(SEED_B_NUT))
        if not ok:
            log("vision: IK failed for the observation pose -> using the physics pose")
            return None, None, None
        log("[0/4 vision] B to the observation pose (camera %.2f m in front of the hub, on the %s axis)"
            % (VISION_DIST, "layout" if CUR["name"] == "FL" else "scanned"))
        await move(drive_b, arm_b, [np.zeros(6), np.array([q_obs[0], -30.0, -120.0, q_obs[3], q_obs[4], q_obs[5]]), q_obs], 5.0, sign)
        await sleep(0.3)                                        # let the arm stop swinging before the picture
        t0 = time.time()
        rgb, depth, T_wc = await cam.capture(tick, n_frames=VISION_FRAMES)
        t1 = time.time()
        if VISION_NOISE_MM > 0:
            depth = depth + np.random.default_rng().normal(0, VISION_NOISE_MM / 1000.0, depth.shape).astype(np.float32) * (depth > 0)
        res = nv.find_wheel(depth, cam.K, T_wc, T_prior, nut_local, rgb=rgb, debug=True)
        t2 = time.time()
        log("[0/4 vision] %dx%d capture %.0f ms, processing %.0f ms (wall clock); %s nut candidates, %d on the bolt pattern"
            % (VISION_RES[0], VISION_RES[1], (t1 - t0) * 1000, (t2 - t1) * 1000, res.get("n_candidates"), res["n_found"]))
        try:
            d = LOG_DIR or os.path.dirname(os.path.abspath(globals().get("__file__", "."))) or "."
            if res["debug"] is not None:
                cv2.imwrite(os.path.join(d, CUR["debug_png"]), res["debug"])
                log("[0/4 vision] annotated image: " + os.path.join(d, CUR["debug_png"]))
        except Exception as e:
            log("vision: could not save the debug image (%s)" % e)
        if VISION_ROS:
            try:
                import vision_ros
                vision_ros.publish(res["debug"], res["T_wheel"] if res["ok"] else None, res["nuts_world"] if res["ok"] else None)

                async def _republish():                     # rqt / RViz opened later still get the picture
                    while _STATE["run_id"] == my_id and not timeline.is_stopped():
                        await asyncio.sleep(1.0)
                        vision_ros.republish()
                asyncio.ensure_future(_republish())
                log("[0/4 vision] published /robot_b/vision/debug_image, /robot_b/vision/nuts, /robot_b/vision/wheel_pose")
            except Exception as e:
                log("vision: ROS 2 publish failed (%s); the image is still in vision_debug.png" % e)
        T = res["T_wheel"]
        if not res["ok"] or T is None:
            log("[0/4 vision] FAILED (%s) -> FALLBACK: using the physics pose" % res["reason"])
            return None, T_obs, q_obs
        shift = float(np.linalg.norm(T[:3, 3] - T_prior[:3, 3]))
        tilt = float(np.degrees(np.arccos(min(1.0, float(T[:3, 2] @ T_prior[:3, 2])))))
        if shift > VISION_MAX_SHIFT or tilt > VISION_MAX_TILT:
            log("[0/4 vision] result implausible (%.0f mm / %.1f deg from the prior) -> FALLBACK: using the physics pose" % (shift * 1000, tilt))
            return None, T_obs, q_obs
        Rr = T_prior[:3, :3].T @ T[:3, :3]
        spin = float(np.degrees(np.arctan2(Rr[1, 0], Rr[0, 0])))
        log("[0/4 vision] wheel found %.1f mm / %.2f deg (axis) away from the layout pose, turned %.2f deg about the axle; pattern fit rms %.2f mm"
            % (shift * 1000, tilt, spin, res["rms"] * 1000))
        # simulation ground truth: report only
        gt = (T_wheel_physics @ np.c_[nut_local, np.ones(len(nut_local))].T).T[:, :3]
        e = np.linalg.norm(res["nuts_world"] - gt, axis=1) * 1000
        e_ax = float(np.degrees(np.arccos(min(1.0, float(T[:3, 2] @ T_wheel_physics[:3, 2])))))
        e_fixed = np.linalg.norm((T_prior @ np.c_[nut_local, np.ones(len(nut_local))].T).T[:, :3] - gt, axis=1) * 1000
        log("[0/4 vision] check vs simulation truth: nut error %s mm (max %.2f), hub axis %.3f deg | %s coordinates would be off by %.1f mm"
            % (np.array2string(e, precision=2), e.max(), e_ax, "layout" if CUR["name"] == "FL" else "scan-estimate", e_fixed.max()))
        return T, T_obs, q_obs

    async def main():
        mon = None
        try:
            for _ in range(120):
                if timeline.is_playing():
                    break
                await app.next_update_async()
            else:
                log("Timeline is not playing. Press PLAY.")
                return
            if not (drive_a.ok and drive_b.ok and spin_attr and all(a and a.IsValid() for a in jaw_attr)):
                log("Scene mismatch: open tire_cell_v3.usda")
                return
            mount = MOUNT_MODE or str(cell(stage, "wheel_mount_mode", "joint"))
            jaw_close = float(cell(stage, "jaw_close", 0.05))
            log("payload A: gripper %.0f + wheel %.0f = %.0f kg / 80 kg | payload B: nutrunner %.1f kg | grasp=%s mount=%s"
                % (cell(stage, "gripper_mass"), cell(stage, "wheel_mass"), cell(stage, "gripper_mass") + cell(stage, "wheel_mass"),
                   cell(stage, "nutrunner_mass"), GRASP_MODE, mount))
            reset_scene_state()
            mon = ContactMonitor()
            guard["mon"] = mon

            async def car_watch():
                try:
                    while _STATE["run_id"] == my_id and not timeline.is_stopped():
                        mon.sample_car()
                        await app.next_update_async()
                except Exception:
                    pass
            asyncio.ensure_future(car_watch())
            await sleep(1.5)                                   # let the car settle on the lift pads
            mon.car0 = None
            mon.car_dpos = mon.car_tilt = 0.0
            mon.sample_car()
            guard["armed"] = True

            arm_a, arm_b = Arm(stage, RA, GA), Arm(stage, RB, GB)
            log("FK check at rest: A %.1e, B %.1e (should be ~0)" % (arm_a.rest_err, arm_b.rest_err))

            # ---- joint sign calibration with robot A (both robots are the same model)
            mon.phase = "calibration"
            sign = 1.0
            q_test = np.array([8.0, 8.0, 0, 0, 0, 0])
            drive_a.command(q_test, 1.0)
            await sleep(1.2)
            T6 = body_pose(RA + "/link6")
            for _ in range(15):                                # wait (max ~4.5 s more) until the arm has settled
                if T6 is None:
                    break
                e_pos = np.linalg.norm(arm_a.fk6(q_test)[:3, 3] - T6[:3, 3])
                e_neg = np.linalg.norm(arm_a.fk6(-q_test)[:3, 3] - T6[:3, 3])
                if min(e_pos, e_neg) < 0.01:
                    break
                await sleep(0.3)
                T6 = body_pose(RA + "/link6")
            if T6 is not None:
                e_pos = np.linalg.norm(arm_a.fk6(q_test)[:3, 3] - T6[:3, 3])
                e_neg = np.linalg.norm(arm_a.fk6(-q_test)[:3, 3] - T6[:3, 3])
                sign = 1.0 if e_pos <= e_neg else -1.0
                log("sign calibration: err(+)=%.4f m, err(-)=%.4f m -> sign %+d" % (e_pos, e_neg, sign))
                if min(e_pos, e_neg) > 0.03:
                    log("WARNING: FK model does not match physics (>3 cm). Aborting.")
                    drive_a.command(np.zeros(6), 1.0)
                    return
            else:
                log("physics pose query unavailable -> assuming sign +1")
            drive_a.command(np.zeros(6), 1.0)
            await sleep(1.0)
            arm_a.sign = arm_b.sign = sign

            nut_local = json.loads(str(cell(stage, "nut_points_wheel_local")))
            async def track_to_rear():
                # =============================== track: both robots to the rear wheel ===============================
                if HAS_TRACK and TRACK_TO_REAR:
                    mon.phase = "track"
                    fl, rl = body_pose(CORNER), body_pose(CORNER.replace("_FL", "_RL"))
                    if fl is not None and rl is not None:
                        d = float(rl[1, 3] - fl[1, 3])                # along the rail (world Y); negative = toward the rear
                        ya, yb = body_pose(RA + "/base_link"), body_pose(RB + "/base_link")
                        log("T 1/2 both carriages travel %.3f m along the track to the rear wheel (max %.1f m/s)" % (d, TRACK_VMAX))
                        await track_move(d, d, (arm_a, arm_b))
                        ya2, yb2 = body_pose(RA + "/base_link"), body_pose(RB + "/base_link")
                        if all(v is not None for v in (ya, yb, ya2, yb2)):
                            ea, eb = (ya2[:3, 3] - ya[:3, 3]) - [0, d, 0], (yb2[:3, 3] - yb[:3, 3]) - [0, d, 0]
                            log("T 2/2 at the rear wheel: A moved %.4f m (error %.2f mm), B moved %.4f m (error %.2f mm), gap %.3f m"
                                % (ya2[1, 3] - ya[1, 3], np.linalg.norm(ea) * 1000, yb2[1, 3] - yb[1, 3], np.linalg.norm(eb) * 1000, abs(ya2[1, 3] - yb2[1, 3])))
                            T6 = body_pose(RA + "/link6")
                            if T6 is not None:
                                log("  FK with track offset vs physics (robot A flange): %.2f mm" % (np.linalg.norm(arm_a.fk6(np.zeros(6))[:3, 3] - T6[:3, 3]) * 1000))
                    else:
                        log("track: rear corner not found in the scene -> carriages stay at the front wheel")

            # =============================== one wheel: vision -> loosen -> remove -> mount -> tighten ===============================
            async def service_wheel(T_prior):
                """Service the wheel in CUR. T_prior: where to look first (None = the layout pose of the front wheel).
                Returns the wheel pose the robots planned with, 'removed' with --no-reassembly, or None if it stopped early."""
                Tw = body_pose(CUR["wheel"])
                if Tw is None:
                    Tw = mat(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(CUR["wheel"])))
                log("wheel center (physics): (%.4f, %.4f, %.4f)" % tuple(Tw[:3, 3]))
                T_wheel_seated = Tw.copy()                          # physical seated pose: only for the mount/alignment checks
                nut_local = json.loads(str(cell(stage, "nut_points_wheel_local")))
                order = json.loads(str(cell(stage, "nut_order", "[1,3,5,2,4]")))

                # =============================== 0. vision: robot B looks at the wheel once ===============================
                B_PRE = []                                         # robot B moves made for the observation (replayed back at the end)
                T_obs = None
                vision_used = False
                if VISION:
                    mon.phase = "vision"
                    Tw_vision, T_obs, q_obs = await observe_wheel(arm_b, sign, nut_local, Tw, T_prior)
                    if T_obs is not None:
                        B_PRE.append(([np.zeros(6), np.array([q_obs[0], -30.0, -120.0, q_obs[3], q_obs[4], q_obs[5]]), q_obs], 5.0))
                    if Tw_vision is not None:
                        Tw = Tw_vision
                        vision_used = True
                R_hub = tool_rotation(Tw[:3, 2])                    # tool rotation along the (measured) hub axis
                nut_world = [(Tw @ np.array([p[0], p[1], p[2], 1.0]))[:3] for p in nut_local]
                rpm = float(cell(stage, "spin_rpm", 330.0))
                spin_s = float(cell(stage, "spin_seconds", 2.0))
                t_cycle0 = simclock["t"]

                # Every joint path robot B and robot A use while taking the wheel off is recorded here.
                # The mounting half plays the same paths backwards, so it only passes through poses that were already
                # reached (and checked for collisions) during removal.
                B_REC = {"to_first": None, "nuts": [], "to_home": None}
                Tw_phys = T_wheel_seated.copy()

                # =============================== 1. robot B: loosen 5 nuts -> tray ===============================
                if not SKIP_NUTS:
                    mon.phase = "B-loosen"
                    PB = plan_b(arm_b, nut_world, order, R_hub)
                    log("[1/4 loosen] B nutrunner to the wheel (order %s, %.0f r/min, nut positions from %s)"
                        % (order, rpm, "VISION" if vision_used else "the physics pose"))
                    back = -R_hub[:, 2] * APPROACH_B
                    T_first_off = T_from(nut_world[order[0] - 1] - back, R_hub)
                    if T_obs is not None:                            # already in front of the wheel: straight line obs -> first stand-off
                        p_first = cart_path(arm_b, B_PRE[-1][0][-1], T_from(T_obs[:3, 3], R_hub), T_first_off, n=16)
                        await move(drive_b, arm_b, p_first, 2.0, sign)
                        B_REC["to_first"] = B_PRE + [(p_first, 2.0)]
                    else:
                        await move(drive_b, arm_b, PB["home_to_first"], 5.0, sign)
                        B_REC["to_first"] = [(PB["home_to_first"], 5.0)]
                    tray_slots = [np.array(p, dtype=float) for p in (CUR.get("tray_slots") or json.loads(str(cell(stage, "nut_tray_slots"))))]
                    tray_nut_positions = [np.array(p, dtype=float) for p in (CUR.get("tray_nut_positions") or json.loads(str(cell(stage, "nut_tray_nut_positions"))))]
                    tray_rotation = np.array(json.loads(str(cell(stage, "nut_tray_tool_rotation"))), dtype=float)
                    tray_normal = tray_rotation[:, 2]
                    route_point = np.array([1.45, -1.05 + track_pos[1], 1.55])     # same point relative to robot B wherever the carriage is
                    q_current = np.array(B_REC["to_first"][-1][0][-1], dtype=float)
                    T_current = T_first_off
                    for st in PB["steps"]:
                        nut_index = st["nut"] - 1
                        T_nut_off = T_from(nut_world[nut_index] - back, R_hub)
                        T_nut_on = T_from(nut_world[nut_index], R_hub)
                        p_in = cart_path(arm_b, q_current, T_current, T_nut_on, n=8)
                        await move(drive_b, arm_b, p_in, 1.5, sign)
                        spin_attr.Set(-rpm * 6.0)                   # loosen (deg/s)
                        await sleep(spin_s)
                        spin_attr.Set(0.0)
                        nut_visible(nut_index, False)
                        carried_nut(nut_index, True)
                        log("[1/4 loosen] nut %d: spindle spun %.1f s, retract" % (st["nut"], spin_s))
                        p_out = p_in[::-1]
                        await move(drive_b, arm_b, p_out, 1.2, sign)

                        T_route_nut = T_from(route_point, R_hub)
                        T_route_tray = T_from(route_point, tray_rotation)
                        route_from_nut = cart_path(arm_b, p_out[-1], T_nut_off, T_route_nut, n=12)
                        await move(drive_b, arm_b, route_from_nut, 2.0, sign)
                        q_route_nut = route_from_nut[-1]
                        q_route_tray, route_ok = arm_b.ik(T_route_tray, np.array(SEED_B_NUT))
                        if not route_ok:
                            raise RuntimeError("robot B: IK failed while turning toward the nut tray")
                        await move(drive_b, arm_b, [q_route_nut, q_route_tray], 1.0, sign)

                        tray_point = np.array(tray_slots[nut_index], dtype=float)
                        tray_standoff = tray_point + tray_normal * 0.10
                        tray_off_path = cart_path(arm_b, q_route_tray, T_route_tray,
                                                  T_from(tray_standoff, tray_rotation), n=12)
                        q_tray_off = tray_off_path[-1]
                        q_tray_on, tray_on_ok = arm_b.ik(T_from(tray_point, tray_rotation), q_route_tray)
                        if not tray_on_ok:
                            raise RuntimeError("robot B: IK failed for nut tray slot %d" % (nut_index + 1))
                        await move(drive_b, arm_b, tray_off_path, 2.0, sign)
                        await move(drive_b, arm_b, [q_tray_off, q_tray_on], 1.2, sign)
                        carried_nut(nut_index, False)
                        loose_nut(nut_index, True, tray_nut_positions[nut_index], tray_rotation)
                        log("[1/4 loosen] nut %d: placed in tray slot %d at (%.2f, %.2f, %.3f)" %
                            (st["nut"], nut_index + 1, tray_point[0], tray_point[1], tray_point[2]))
                        await move(drive_b, arm_b, [q_tray_on, q_tray_off], 1.2, sign)
                        await move(drive_b, arm_b, tray_off_path[::-1], 2.0, sign)
                        await move(drive_b, arm_b, [tray_off_path[0], q_route_nut], 1.0, sign)

                        rec = {"nut": st["nut"], "index": nut_index, "in": p_in, "route": route_from_nut,
                               "turn": [q_route_nut, q_route_tray], "tray_off": tray_off_path,
                               "tray_in": [q_tray_off, q_tray_on], "back": [tray_off_path[0], q_route_nut], "to_next": None}
                        if st["nut"] != order[-1]:
                            next_nut = order[order.index(st["nut"]) + 1] - 1
                            T_next_off = T_from(nut_world[next_nut] - back, R_hub)
                            to_next = cart_path(arm_b, q_route_nut, T_route_nut, T_next_off, n=12)
                            await move(drive_b, arm_b, to_next, 2.0, sign)
                            rec["to_next"] = to_next
                            q_current, T_current = to_next[-1], T_next_off
                        else:
                            q_current = q_route_nut
                        B_REC["nuts"].append(rec)
                    if mount == "joint":
                        joint_enabled(CUR["stud_joint"], True)
                        await frames(2)
                    joint_enabled(CUR["nut_clamp"], False)                  # all 5 nuts off -> wheel is held by pilot + studs only
                    log("[1/4 loosen] all 5 nuts in the tray; nutrunner goes home")
                    home_path = [q_current, PB["last_to_home"][-2], np.zeros(6)]
                    await move(drive_b, arm_b, home_path, 5.0, sign)
                    B_REC["to_home"] = home_path
                    await sleep(0.5)
                    Tw2 = body_pose(CUR["wheel"])
                    if Tw2 is not None:
                        d = Tw2[:3, 3] - T_wheel_seated[:3, 3]
                        c = max(-1.0, min(1.0, float(Tw2[:3, 2] @ T_wheel_seated[:3, 2])))
                        tilt = float(np.degrees(np.arccos(c)))
                        log("wheel moved after nut removal: %.2f mm, tilt %.2f deg (hanging on pilot/studs)" % (np.linalg.norm(d) * 1000, tilt))
                        if np.linalg.norm(d) * 1000 > LIM.get("wheel_sag_mm", 3.0) or tilt > LIM.get("wheel_tilt_deg", 1.0):
                            if SAFETY:
                                raise _Protect("wheel is not seated (moved %.1f mm, tilted %.1f deg) -> not grasping it" % (np.linalg.norm(d) * 1000, tilt))
                            log("WARNING: wheel sagged/tilted more than a seated wheel should. Grasp may be poor.")
                        if np.linalg.norm(d) > 0.03:
                            log("WARNING: wheel did not stay on the studs. Re-run with --mount joint.")
                            return None
                        Tw_phys = Tw2
                        if not vision_used:                          # vision: keep planning from the camera measurement
                            Tw = Tw2
                else:
                    if B_PRE:                                        # looked at the wheel but skips the nuts: B goes home
                        for path, sec in reversed(B_PRE):
                            await move(drive_b, arm_b, path[::-1], sec, sign)
                    if mount == "joint":
                        joint_enabled(CUR["stud_joint"], True)
                        await frames(2)
                    joint_enabled(CUR["nut_clamp"], False)
                    await sleep(0.5)

                # =============================== 2. robot A: wheel off -> floor ===============================
                mon.phase = "A-remove"
                PA = plan_a(arm_a, Tw[:3, 3], float(cell(stage, "tip_below_tcp")), float(cell(stage, "stud_exposed", 0.045)),
                            R_hub @ rot_z(CUR.get("grip_roll", 0.0)), CUR.get("place_xy"), CUR.get("return_via"))
                w0 = Tw_phys[:3, 3].copy()                          # physics: to measure how far the wheel really moved

                def wheel_pos():
                    T = body_pose(CUR["wheel"])
                    return None if T is None else T[:3, 3].copy()

                def tcp_pose():
                    T6_now = body_pose(RA + "/link6")
                    return None if T6_now is None else T6_now @ arm_a.T6_tcp

                def tcp_to_wheel_now():
                    T_tcp, T_w = tcp_pose(), body_pose(CUR["wheel"])
                    return None if T_tcp is None or T_w is None else np.linalg.inv(T_tcp) @ T_w

                def pose_err(Ta, Tb):
                    c = max(-1.0, min(1.0, (np.trace(Ta[:3, :3].T @ Tb[:3, :3]) - 1.0) / 2.0))
                    return float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])), float(np.degrees(np.arccos(c)))

                def jaw_gap():
                    a, b = body_pose(GA + "/jaw_0"), body_pose(GA + "/jaw_2")
                    return None if a is None or b is None else float(np.linalg.norm(a[:3, 3] - b[:3, 3]))

                async def grasp_joint_on():
                    Tb, Tw3 = body_pose(GA + "/base"), body_pose(CUR["wheel"])
                    if Tb is None or Tw3 is None:
                        raise RuntimeError("could not read gripper/wheel pose for the grasp joint")
                    Tr = np.linalg.inv(Tb) @ Tw3
                    gj = UsdPhysics.FixedJoint(stage.GetPrimAtPath(CUR["grasp_joint"]))
                    w, x, y, z = R_to_quat(Tr[:3, :3])
                    with session_edit():
                        gj.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in Tr[:3, 3]]))
                        gj.GetLocalRot0Attr().Set(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))
                    await frames(2)
                    joint_enabled(CUR["grasp_joint"], True)
                    await frames(2)

                log("[2/4 remove] A move to pre-grasp")
                await move(drive_a, arm_a, PA["home_to_pre"], 5.0, sign)
                await jaws(JAW_OPEN, CUR.get("jaw_approach", JAW_APPROACH), 1.0)
                log("[2/4 remove] A approach: fingers pass over the tire")
                await move(drive_a, arm_a, PA["approach"], 3.5, sign)
                log("[2/4 remove] A close 4 jaws on the tread")
                g0 = jaw_gap()
                await jaws(CUR.get("jaw_approach", JAW_APPROACH), jaw_close, 2.0)
                await sleep(0.7)
                g1 = jaw_gap()
                if g0 is not None and g1 is not None:
                    log("jaw travel %.1f mm of %.1f mm commanded (stopping short = pads pressing on the tire)"
                        % ((g0 - g1) / 2 * 1000 + CUR.get("jaw_approach", JAW_APPROACH) * 1000, jaw_close * 1000))
                if GRASP_MODE == "joint":
                    await grasp_joint_on()
                tcp_to_wheel_remove = tcp_to_wheel_now()          # wheel pose in the gripper while it was still on the hub
                log("[2/4 remove] A pull the wheel along the studs")
                await move(drive_a, arm_a, PA["pull_studs"], 3.0, sign)
                w1 = wheel_pos()
                if w1 is not None:
                    d = float(np.linalg.norm(w1 - w0))
                    log("wheel travel along studs: %.1f mm" % (d * 1000))
                    if d < 0.03:
                        log("WARNING: wheel did not follow the gripper (grip failed). Try --grasp joint. Aborting.")
                        await jaws(jaw_close, CUR.get("jaw_approach", JAW_APPROACH), 1.5)
                        await move(drive_a, arm_a, PA["approach"][::-1], 3.0, sign)
                        return None
                joint_enabled(CUR["stud_joint"], False)
                await sleep(0.5)
                log("[2/4 remove] A withdraw (wheel carried by the gripper only)")
                await move(drive_a, arm_a, PA["pull_clear"], 3.5, sign)
                log("[2/4 remove] A carry to place position (tool turns down)")
                await move(drive_a, arm_a, PA["pre_to_above"], 6.0, sign)
                w2, T_tcp = wheel_pos(), tcp_pose()
                if w2 is not None and T_tcp is not None:
                    slip = float(np.linalg.norm(T_tcp[:3, 3] - w2))
                    log("wheel offset from TCP while carried: %.1f mm" % (slip * 1000))
                    if slip > 0.15:
                        log("WARNING: wheel was dropped during carry.")
                log("[2/4 remove] A lower to floor + release")
                await move(drive_a, arm_a, PA["lower"], 4.0, sign)
                await sleep(0.5)
                if GRASP_MODE == "joint":
                    joint_enabled(CUR["grasp_joint"], False)
                    await sleep(0.3)
                await jaws(jaw_close, JAW_OPEN, 2.0)
                await sleep(0.8)
                log("[2/4 remove] A retreat and go home")
                await move(drive_a, arm_a, PA["raise"], 3.0, sign)
                # empty gripper passes the car on the way home: close the jaws first (wide-open finger bars hit the body)
                await jaws(JAW_OPEN, jaw_close, 1.5)
                await move(drive_a, arm_a, PA["above_to_home"], 5.0, sign)
                w3 = wheel_pos()
                if w3 is not None:
                    log("wheel final position: (%.3f, %.3f, %.3f)  [expected about (%.2f, %.2f, 0.11)]" % (w3[0], w3[1], w3[2], *(CUR.get("place_xy") or PLACE_XY)))
                log("==== wheel is off (%.0f s since start). waiting %.1f s, then mounting in reverse order ====" % (simclock["t"] - t_cycle0, PAUSE_S))

                if not REASSEMBLE:
                    return "removed"
                await sleep(PAUSE_S)

                # =============================== 3. robot A (reverse of 2): floor -> hub ===============================
                mon.phase = "A-mount"
                log("[3/4 mount] A home -> above the wheel (reverse of 'go home')")
                await move(drive_a, arm_a, PA["above_to_home"][::-1], 5.0, sign)   # jaws still closed (passing the car)
                await jaws(jaw_close, JAW_OPEN, 1.5)
                await move(drive_a, arm_a, PA["raise"][::-1], 3.0, sign)
                log("[3/4 mount] A close jaws on the wheel lying on the floor")
                await jaws(JAW_OPEN, jaw_close, 2.0)
                await sleep(0.7)
                if GRASP_MODE == "joint":
                    await grasp_joint_on()
                T_floor = body_pose(CUR["wheel"])
                log("[3/4 mount] A lift the wheel (reverse of 'lower')")
                await move(drive_a, arm_a, PA["lower"][::-1], 4.0, sign)
                T_up = body_pose(CUR["wheel"])
                if T_floor is not None and T_up is not None:
                    lift = float(np.linalg.norm(T_up[:3, 3] - T_floor[:3, 3]))
                    log("wheel lifted %.1f mm off the floor" % (lift * 1000))
                    if lift < 0.03:
                        raise RuntimeError("wheel did not follow robot A when lifting from the floor (try --grasp joint)")
                log("[3/4 mount] A carry back to the hub (reverse of 'carry to place')")
                await move(drive_a, arm_a, PA["pre_to_above"][::-1], 6.0, sign)

                # The wheel may sit a few mm differently in the jaws than it did on the hub (it settles when released on
                # the floor). Correct the TCP target so that the CUR["wheel"], not the TCP, returns to the recorded pose.
                tw_now = tcp_to_wheel_now()
                C = np.eye(4)
                if tw_now is not None and tcp_to_wheel_remove is not None:
                    C = tcp_to_wheel_remove @ np.linalg.inv(tw_now)
                    e_p, e_r = pose_err(tcp_to_wheel_remove, tw_now)
                    log("wheel-in-gripper offset vs removal: %.1f mm, %.2f deg -> corrected in the mounting path" % (e_p * 1000, e_r))
                    if e_p > 0.03 or e_r > 5.0:
                        raise RuntimeError("wheel shifted too much in the gripper (%.1f mm, %.1f deg); not mounting" % (e_p * 1000, e_r))
                T_pre_c, T_clear_c, T_seat_c = PA["T_pre"] @ C, PA["T_clear"] @ C, PA["T_grasp"] @ C
                q_pre_c, ok = arm_a.ik(T_pre_c, PA["pull_clear"][-1])
                if not ok:
                    raise RuntimeError("robot A: IK failed for the corrected pre-mount pose")
                await move(drive_a, arm_a, [PA["pull_clear"][-1], q_pre_c], 1.0, sign)
                log("[3/4 mount] A bring the wheel to the stud tips (reverse of 'withdraw')")
                p_to_clear = cart_path(arm_a, q_pre_c, T_pre_c, T_clear_c, n=20)
                await move(drive_a, arm_a, p_to_clear, 3.5, sign)
                T_w_clear = body_pose(CUR["wheel"])
                if T_w_clear is not None:
                    axis = T_wheel_seated[:3, 2]                     # stud axis (wheel outer-face normal)
                    dp = T_w_clear[:3, 3] - T_wheel_seated[:3, 3]
                    radial = float(np.linalg.norm(dp - axis * (dp @ axis)))
                    tilt = float(np.degrees(np.arccos(max(-1.0, min(1.0, float(T_w_clear[:3, 2] @ axis))))))
                    log("alignment at stud tips: radial %.1f mm, tilt %.2f deg, %.1f mm out from seated"
                        % (radial * 1000, tilt, float(dp @ axis) * 1000))
                    if radial > 0.005 or tilt > 2.0:
                        raise RuntimeError("wheel is not lined up with the studs (radial %.1f mm, tilt %.1f deg)" % (radial * 1000, tilt))
                if mount == "joint":
                    joint_enabled(CUR["stud_joint"], True)                  # reverse of 'stud joint off'
                    await frames(2)
                await sleep(0.5)
                log("[3/4 mount] A push the wheel onto the studs (reverse of 'pull')")
                p_push = cart_path(arm_a, p_to_clear[-1], T_clear_c, T_seat_c, n=8)
                await move(drive_a, arm_a, p_push, 3.0, sign)
                await sleep(0.4)
                if GRASP_MODE == "joint":
                    joint_enabled(CUR["grasp_joint"], False)
                    await frames(2)
                log("[3/4 mount] A open jaws, back off, go home")
                await jaws(jaw_close, CUR.get("jaw_approach", JAW_APPROACH), 2.0)
                await sleep(0.5)
                T_mounted = body_pose(CUR["wheel"])
                if T_mounted is not None:
                    e_p, e_r = pose_err(T_wheel_seated, T_mounted)
                    log("wheel mount error vs original seated pose: %.1f mm, %.2f deg" % (e_p * 1000, e_r))
                    if e_p > 0.005 or e_r > 1.0:
                        raise RuntimeError("wheel did not seat at the original hub pose; stopping before tightening")
                await move(drive_a, arm_a, PA["approach"][::-1], 3.5, sign)
                await jaws(CUR.get("jaw_approach", JAW_APPROACH), JAW_OPEN, 1.0)
                await move(drive_a, arm_a, PA["home_to_pre"][::-1], 5.0, sign)

                # =============================== 4. robot B (reverse of 1): tray -> nuts, tighten ===============================
                if SKIP_NUTS:
                    joint_enabled(CUR["nut_clamp"], True)
                    await frames(2)
                    joint_enabled(CUR["stud_joint"], False)
                    log("[4/4 tighten] skipped (--skip-nuts)")
                else:
                    mon.phase = "B-tighten"
                    tighten_order = [r["nut"] for r in reversed(B_REC["nuts"])]
                    log("[4/4 tighten] B to the tray; tightening order %s (reverse of %s)" % (tighten_order, order))
                    await move(drive_b, arm_b, B_REC["to_home"][::-1], 5.0, sign)
                    recs = B_REC["nuts"]
                    for k in range(len(recs) - 1, -1, -1):
                        r = recs[k]
                        await move(drive_b, arm_b, r["back"][::-1], 1.0, sign)
                        await move(drive_b, arm_b, r["tray_off"], 2.0, sign)
                        await move(drive_b, arm_b, r["tray_in"], 1.2, sign)
                        loose_nut(r["index"], False)
                        carried_nut(r["index"], True)
                        log("[4/4 tighten] nut %d: picked from tray slot %d" % (r["nut"], r["index"] + 1))
                        await move(drive_b, arm_b, r["tray_in"][::-1], 1.2, sign)
                        await move(drive_b, arm_b, r["tray_off"][::-1], 2.0, sign)
                        await move(drive_b, arm_b, r["turn"][::-1], 1.0, sign)
                        await move(drive_b, arm_b, r["route"][::-1], 2.0, sign)
                        await move(drive_b, arm_b, r["in"], 1.5, sign)
                        spin_attr.Set(rpm * 6.0)                    # tighten (deg/s), opposite of loosening
                        await sleep(spin_s)
                        spin_attr.Set(0.0)
                        carried_nut(r["index"], False)
                        nut_visible(r["index"], True)
                        log("[4/4 tighten] nut %d: tightened (spindle %.1f s)" % (r["nut"], spin_s))
                        await move(drive_b, arm_b, r["in"][::-1], 1.2, sign)
                        if k > 0:
                            await move(drive_b, arm_b, recs[k - 1]["to_next"][::-1], 2.0, sign)
                    joint_enabled(CUR["nut_clamp"], True)                   # all 5 nuts on -> wheel clamped to the hub again
                    await frames(2)
                    joint_enabled(CUR["stud_joint"], False)
                    log("[4/4 tighten] all 5 nuts tightened; nut clamp engaged; nutrunner goes home")
                    for path, sec in reversed(B_REC["to_first"]):
                        await move(drive_b, arm_b, path[::-1], sec, sign)

                await sleep(0.5)
                T_end = body_pose(CUR["wheel"])
                if T_end is not None:
                    e_p, e_r = pose_err(T_wheel_seated, T_end)
                    log("final wheel pose vs start: %.1f mm, %.2f deg" % (e_p * 1000, e_r))
                return Tw                                           # the pose the robots planned with (vision, or physics fallback)

            # =============================== scan: carriages travel, robot B watches the car side for the next wheel ===============================
            async def scan_for_wheel(T_ref, layout_prior):
                """Robot B looks at the car side (camera SCAN_DIST from the wheel plane) while both carriages travel toward
                the rear. Every few cm it looks for a tire (nut_vision.find_tire). When it sees one that is not the wheel
                just serviced, it sets the stop point so that the robots stand at the same place relative to the new wheel
                as they did at the front wheel, decelerates there, and checks once more standing still.
                Returns (T_prior for the new wheel, found_by_camera)."""
                import cv2
                import nut_vision as nv
                import vision_capture as vc
                mon.phase = "scan"
                nl = np.asarray(json.loads(str(cell(stage, "nut_points_wheel_local"))), dtype=float)
                z_top = float(np.mean(nl[:, 2])) + 0.015
                cam = vc.WristCamera(stage, str(cell(stage, "camera_b", GB + "/Camera_RSD455/RSD455")), resolution=(640, 400))
                T_tcp_cam = vc.tcp_to_camera(stage, GB + "/base/tcp", cam)
                n_out = T_ref[:3, 2]
                T_aim = T_ref.copy()
                T_aim[1, 3] += SCAN_AIM_SHIFT * float(np.sign(TRACK_BASE_Y[1] - TRACK_BASE_Y[0]) or -1.0)
                T_scan = vc.observation_tcp(T_aim, z_top, T_tcp_cam, tool_rotation(n_out), SCAN_DIST)
                q_scan, ok = arm_b.ik(T_scan, np.array(SEED_B_NUT))
                y_ref = float(T_ref[1, 3])                              # rail direction = world Y
                d_layout = float(layout_prior[1, 3] - y_ref) if layout_prior is not None else None
                if not ok:
                    log("[scan] IK failed for the scan pose -> moving to the layout position")
                    return None, False
                p_scan = [np.zeros(6), np.array([q_scan[0], -30.0, -120.0, q_scan[3], q_scan[4], q_scan[5]]), q_scan]
                log("[scan] B looks at the car side (camera %.1f m from the wheel plane); carriages start toward the rear at %.2f m/s"
                    % (SCAN_DIST, SCAN_SPEED))
                await move(drive_b, arm_b, p_scan, 5.0, sign)
                await sleep(0.3)
                expect = SCAN_DIST + z_top - 0.10                       # camera -> tire sidewall (about 10 cm outside the hub)
                cam.open_stream()
                hits, last_dbg = [], None
                try:
                    pos, v, dt = track_pos[1], 0.0, 1.0 / 60.0
                    target, last_look, n_look = TRACK_LIM[0], pos, 0
                    t_scan0 = time.time()
                    while True:
                        rem = target - pos
                        if abs(rem) < 5e-4 and abs(v) < 5e-3:
                            break
                        v_des = np.sign(rem) * min(SCAN_SPEED, (2.0 * SCAN_ACCEL * abs(rem)) ** 0.5)
                        v += float(np.clip(v_des - v, -SCAN_ACCEL * dt, SCAN_ACCEL * dt))
                        pos = float(np.clip(pos + v * dt, TRACK_LIM[0], TRACK_LIM[1]))
                        set_track(pos, pos)
                        dt = max(await tick(), 1e-4)
                        if abs(pos - last_look) >= 0.04 and abs(rem) > 0.10:
                            last_look = pos
                            fr = cam.grab()
                            if fr is None:
                                continue
                            n_look += 1
                            res = nv.find_tire(fr[0], fr[1], cam.K, fr[2], expect, debug=True)
                            if not res["ok"]:
                                continue
                            u, vpx, r = res["pixel"]
                            inside = r < u < cam.res[0] - r and r < vpx < cam.res[1] - r
                            y_found = float(res["center_world"][1])
                            if not inside or abs(y_found - y_ref) < 0.8:   # cut off at the image edge, or the wheel we just did
                                continue
                            hits.append(y_found)
                            last_dbg = res["debug"]
                            new_target = float(np.clip(np.median(hits[-3:]) - y_ref, TRACK_LIM[0], TRACK_LIM[1]))
                            if len(hits) == 1:
                                log("[scan] tire seen at y = %.3f m (rim r = %.0f mm, %.2f m away) -> stop the carriages at %.3f m"
                                    % (y_found, res["radius_m"] * 1000, float(np.linalg.norm(res["center_world"] - fr[2][:3, 3])), new_target))
                            target = new_target
                    log("[scan] carriages stopped at %.3f m after %d looks (%.0f ms wall clock per look incl. sim)"
                        % (pos, n_look, (time.time() - t_scan0) * 1000 / max(n_look, 1)))
                    set_track(pos, pos)
                    for arm in (arm_a, arm_b):
                        arm.set_track(pos)
                    await sleep(0.8)
                    # standing still: one more look to confirm and correct
                    fr = None
                    for _ in range(10):
                        await tick()
                        fr = cam.grab()
                    res = nv.find_tire(fr[0], fr[1], cam.K, fr[2], expect, debug=True) if fr is not None else {"ok": False}
                    if fr is not None:
                        T_fk = arm_b.fk(q_scan) @ T_tcp_cam
                        log("[scan] camera physics pos %s fwd %s | FK pos %s fwd %s | tire pixel %s"
                            % (np.round(fr[2][:3, 3], 3), np.round(-fr[2][:3, 2], 3), np.round(T_fk[:3, 3], 3), np.round(-T_fk[:3, 2], 3),
                               None if not res.get("ok") else np.round(res["pixel"], 1)))
                finally:
                    cam.close_stream()
                found = bool(hits) and res.get("ok")
                if found:
                    last_dbg = res["debug"]
                    c = np.asarray(res["center_world"], dtype=float)
                    d_fin = float(c[1] - y_ref)
                    log("[scan] confirm standing still: tire centre (%.3f, %.3f, %.3f), carriage %.3f m, needed %.3f m (error %.1f mm)"
                        % (c[0], c[1], c[2], pos, d_fin, (pos - d_fin) * 1000))
                    if abs(pos - d_fin) > 0.003:
                        await track_move(d_fin, d_fin, (arm_a, arm_b))
                        log("[scan] corrected the carriage position to %.3f m" % d_fin)
                    T_prior = T_ref.copy()
                    T_prior[:3, 3] = c - n_out * 0.10                    # sidewall centre -> wheel centre (about 10 cm inside)
                else:
                    log("[scan] no tire found along the track -> FALLBACK: layout position of the rear wheel")
                    if d_layout is not None:
                        await track_move(d_layout, d_layout, (arm_a, arm_b))
                    T_prior = layout_prior
                try:
                    if last_dbg is not None:
                        d = LOG_DIR or os.path.dirname(os.path.abspath(globals().get("__file__", "."))) or "."
                        cv2.imwrite(os.path.join(d, "scan_debug.png"), last_dbg)
                        log("[scan] annotated image: " + os.path.join(d, "scan_debug.png"))
                except Exception:
                    pass
                await move(drive_b, arm_b, p_scan[::-1], 5.0, sign)
                return T_prior, found

            # =============================== run: front-left wheel, then scan + rear-left wheel ===============================
            Tw_front = None
            if DO_FRONT:
                CUR.clear()
                CUR.update(FRONT_CFG)
                t_w0 = simclock["t"]
                r = await service_wheel(None)
                if r is None:
                    mon.report()
                    log("DONE (stopped during the front wheel)")
                    return
                if isinstance(r, str):                              # --no-reassembly
                    await track_to_rear()
                    mon.report()
                    log("DONE  (removal only; --no-reassembly)")
                    return
                Tw_front = r
                log("cycle time %.0f s (simulation time, including the %.1f s pause, excluding the track move)" % (simclock["t"] - t_w0, PAUSE_S))
            rear_cfg = WHEEL_CFGS[1] if len(WHEEL_CFGS) > 1 else None
            if rear_cfg is not None and HAS_TRACK and TRACK_TO_REAR:
                layout_rear = np.array(rear_cfg["layout_prior"]) if rear_cfg.get("layout_prior") is not None else None
                T_ref = Tw_front if Tw_front is not None else _STATE.get("wheel_prior")
                T_rear_prior, found = await scan_for_wheel(T_ref, layout_rear)
                CUR.clear()
                CUR.update(rear_cfg)
                CUR["place_xy"] = (REAR_PLACE_REL[0], TRACK_BASE_Y[0] + track_pos[0] + REAR_PLACE_REL[1])
                CUR["grip_roll"], CUR["jaw_approach"], CUR["return_via"] = REAR_GRIP_ROLL, REAR_JAW_APPROACH, REAR_RETURN_VIA
                log("==== %s wheel: carriages at %.3f m; same vision + nut sequence as the front wheel ====" % (rear_cfg["name"], track_pos[0]))
                t_w0 = simclock["t"]
                r = await service_wheel(T_rear_prior)
                if r is None or isinstance(r, str):
                    mon.report()
                    log("DONE (stopped during the %s wheel)" % rear_cfg["name"])
                    return
                log("cycle time %.0f s (simulation time, %s wheel, including the %.1f s pause)" % (simclock["t"] - t_w0, rear_cfg["name"], PAUSE_S))
                CUR["label"] = ""
            else:
                await track_to_rear()
            mon.report()
            log("DONE  (%s; press STOP then PLAY to run again)" % (
                "front + rear-left wheels: nuts off -> wheel off -> wheel on -> nuts on" if rear_cfg is not None and TRACK_TO_REAR
                else "nuts off -> wheel off -> %.0f s -> wheel on -> nuts on" % PAUSE_S))
        except _Protect as e:
            spin_attr.Set(0.0)
            log("PROTECTIVE STOP: %s" % e)
            log("robots hold their current pose. Press STOP to reset.")
            if mon is not None:
                mon.report()
            log("DONE (stopped by protective stop)")
        except _Abort:
            if _STATE["run_id"] == my_id:
                reset_scene_state()
                log("stopped -> scene reset. Press PLAY to run again.")
        except Exception as e:
            import traceback
            log("ERROR: %s" % e)
            traceback.print_exc()
            if mon is not None:
                mon.report()
        finally:
            if mon is not None:
                mon.close()
            simclock["sub"] = None                             # 물리 스텝 구독 해제

    try:
        from omni.kit.async_engine import run_coroutine
        run_coroutine(main())
    except Exception:
        asyncio.ensure_future(main())


def snapshot_layout():
    """Remember the wheel pose as authored in the scene (= the cell layout drawing) before physics moves anything.
    Vision uses it only as a prior: where to point the camera and which way is 'up' for nut numbering."""
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    w = str(cell(stage, "wheel_path", "")) if stage else ""
    if w and stage.GetPrimAtPath(w).IsValid():
        _STATE["wheel_prior"] = mat(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(w)))


def install_auto_run():
    """run now if playing, and again every time the timeline goes STOP -> PLAY"""
    import builtins

    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
    if "wheel_prior" not in _STATE:
        snapshot_layout()
    state = {"was_stopped": tl.is_stopped()}

    def on_event(e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            state["was_stopped"] = True
        elif e.type == int(omni.timeline.TimelineEventType.PLAY):
            if state["was_stopped"]:
                state["was_stopped"] = False
                run()

    builtins._cell_demo_sub = tl.get_timeline_event_stream().create_subscription_to_pop(on_event, name="cell_demo_auto_run")
    if tl.is_playing():
        state["was_stopped"] = False
        run()
    else:
        print("[cell_demo] ready: press PLAY to start (auto re-runs after STOP -> PLAY)")


try:
    import omni.kit.app  # noqa: F401
    _IN_ISAAC = True
except ImportError:
    _IN_ISAAC = False

if _IN_ISAAC:
    install_auto_run()
