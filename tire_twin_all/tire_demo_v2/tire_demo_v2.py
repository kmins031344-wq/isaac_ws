# Tire removal demo v2 for Isaac Sim 5.x  (run in Script Editor while PLAYING, or via run_tire_demo_v2.py)
# Scene: tire_demo_scene_v2.usda  (RS080N + 4-jaw wheel gripper, Prius body on a scissor lift, 22 kg wheel on 5 studs, no nuts)
# Sequence: approach -> close 4 jaws on the tread -> pull the wheel off the studs -> lay it flat on the floor -> release -> home
#
# Grasp model (GRASP_MODE = "physics"): real contact. The 4 pads squeeze the tread (drive force) and the pad lips hook both
# tire shoulders, so the wheel is carried by contact/friction only. The studs are modelled by /World/stud_joint (a prismatic
# joint along the axle); it is released once the wheel has travelled past the stud length.
# GRASP_MODE = "joint" is a fallback that locks the wheel to the gripper with a FixedJoint after closing.
# Kinematics (FK/IK) are read from the USD joint frames.
import asyncio
import math
import time

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ROBOT = "/World/RS080N"
G = ROBOT + "/WheelGripper"            # overwritten from scene customData
WHEEL = "/World/Wheel_FL"
STUD_JOINT = "/World/stud_joint"
GRASP_JOINT = "/World/grasp_joint"
ARM_JOINTS = ["base_link/joint1", "link1/joint2", "link2/joint3", "link3/joint4", "link4/joint5", "link5/joint6"]

GRASP_MODE = "physics"                  # "physics" (contact only) or "joint" (FixedJoint fallback)
JAW_OPEN = 0.0
JAW_APPROACH = 0.012                    # jaws partly closed while entering the wheel arch (lips still clear the tire by 10 mm)
APPROACH = 0.42                         # pre-grasp distance along the wheel axis [m]
PLACE_XY = (1.00, 1.05)                 # where the wheel is laid down (world x, y)
PLACE_CLEAR = 0.004                     # finger-tip height above floor at release [m]
ABOVE_Z = 0.75                          # wheel-center height before lowering [m]
SPEED = 1.0                             # >1 faster, <1 slower

# IK seeds [deg] (model space), found offline
SEED_GRASP = [0.0, -10.0, -125.0, 0.0, 30.0, 0.0]
SEED_PLACE = [-45.0, 15.0, -90.0, 0.0, -75.0, 45.0]


def scene_param(stage, key, default=None):
    v = stage.GetPrimAtPath("/World").GetCustomDataByKey("demo:" + key)
    return default if v is None else v


# ----------------------------------------------------------------------------- math helpers
def mat(gf):
    return np.array(gf, dtype=float).T          # Gf (row-vector) -> numpy (column-vector)


def quat_to_R(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return tuple(q)


def T_from(p, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def rot_axis(axis, ang):
    a = np.asarray(axis, dtype=float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if abs(math.pi - th) < 1e-5:                # near 180 deg: use the dominant axis
        A = (R + np.eye(3)) / 2
        i = int(np.argmax(np.diag(A)))
        ax = A[:, i] / math.sqrt(max(A[i, i], 1e-12))
        return ax * th
    return v / (2 * math.sin(th)) * th


def smooth(s):
    return s * s * (3 - 2 * s)


# ----------------------------------------------------------------------------- kinematics from USD
class Arm:
    def __init__(self, stage, sign=1.0):
        self.sign = sign
        xc = UsdGeom.XformCache()
        self.T_base = mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(ROBOT + "/base_link")))
        self.L0, self.L1inv, self.axis, lo, hi = [], [], [], [], []
        for jp in ARM_JOINTS:
            j = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(ROBOT + "/" + jp))
            p0, r0 = j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get()
            p1, r1 = j.GetLocalPos1Attr().Get(), j.GetLocalRot1Attr().Get()
            self.L0.append(T_from(np.array(p0), quat_to_R(r0.GetReal(), *r0.GetImaginary())))
            self.L1inv.append(np.linalg.inv(T_from(np.array(p1), quat_to_R(r1.GetReal(), *r1.GetImaginary()))))
            self.axis.append({"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[j.GetAxisAttr().Get()])
            lo.append(j.GetLowerLimitAttr().Get())
            hi.append(j.GetUpperLimitAttr().Get())
        self.lo_cmd, self.hi_cmd = np.array(lo) + 1.0, np.array(hi) - 1.0     # 1 deg margin
        W6 = mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(ROBOT + "/link6")))
        self.T6_tcp = np.linalg.inv(W6) @ mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(G + "/base/tcp")))
        self.T6_gbase = np.linalg.inv(W6) @ mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(G + "/base")))
        self.rest_err = float(np.linalg.norm(self.fk6(np.zeros(6)) - W6))

    def limits(self):
        a, b = self.sign * self.lo_cmd, self.sign * self.hi_cmd
        return np.minimum(a, b), np.maximum(a, b)

    def fk6(self, q_deg):                       # link6 world pose; q in model space [deg]
        T = self.T_base.copy()
        for i in range(6):
            Rq = np.eye(4)
            Rq[:3, :3] = rot_axis(self.axis[i], math.radians(q_deg[i]))
            T = T @ self.L0[i] @ Rq @ self.L1inv[i]
        return T

    def fk(self, q_deg):
        return self.fk6(q_deg) @ self.T6_tcp

    def ik(self, T_target, seed, iters=300):
        q = np.array(seed, dtype=float)
        lo, hi = self.limits()
        err = None
        for _ in range(iters):
            T = self.fk(q)
            e = np.concatenate([T_target[:3, 3] - T[:3, 3], rotvec(T_target[:3, :3] @ T[:3, :3].T)])
            err = e
            if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-5:
                break
            J = np.zeros((6, 6))
            h = 1e-3
            for i in range(6):
                dq = q.copy()
                dq[i] += h
                Ti = self.fk(dq)
                J[:3, i] = (Ti[:3, 3] - T[:3, 3]) / h
                J[3:, i] = rotvec(Ti[:3, :3] @ T[:3, :3].T) / h
            step = J.T @ np.linalg.solve(J @ J.T + 1e-8 * np.eye(6), e)
            m = np.max(np.abs(step))
            if m > 15.0:
                step *= 15.0 / m
            q = np.clip(q + step, lo, hi)
        ok = np.linalg.norm(err[:3]) < 1e-4 and np.linalg.norm(err[3:]) < 1e-4
        return q, ok


def cart_path(arm, q_start, T_a, T_b, n=24):
    """straight-line TCP move with fixed orientation -> joint waypoints"""
    out, q = [np.array(q_start, dtype=float)], np.array(q_start, dtype=float)
    for k in range(1, n + 1):
        T = T_a.copy()
        T[:3, 3] = T_a[:3, 3] + (T_b[:3, 3] - T_a[:3, 3]) * k / n
        q, ok = arm.ik(T, q)
        if not ok:
            raise RuntimeError("IK failed on straight path at step %d" % k)
        out.append(q)
    return out


def plan(stage, sign=1.0):
    """returns (arm, dict of joint-space paths in model space [deg])"""
    global G
    G = str(scene_param(stage, "gripper_path", G))
    arm = Arm(stage, sign)
    wc = np.array(scene_param(stage, "wheel_center"))
    tip = float(scene_param(stage, "tip_below_tcp"))
    stud = float(scene_param(stage, "stud_exposed", 0.045))
    # tool axis = tcp local -Z.  Grasp: tool -> +X (wheel axis). Place: tool -> -Z (down)
    R_grasp = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)     # cols: tcp x=(0,-1,0) y=(0,0,1) z=(-1,0,0)
    R_place = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)      # cols: tcp x=(0,-1,0) y=(1,0,0) z=(0,0,1)
    T_grasp = T_from(wc, R_grasp)
    T_clear = T_from(wc - np.array([stud + 0.03, 0, 0]), R_grasp)           # wheel just past the stud tips
    T_pre = T_from(wc - np.array([APPROACH, 0, 0]), R_grasp)
    T_place = T_from(np.array([PLACE_XY[0], PLACE_XY[1], tip + PLACE_CLEAR]), R_place)
    T_above = T_from(np.array([PLACE_XY[0], PLACE_XY[1], ABOVE_Z]), R_place)

    q_pre, ok1 = arm.ik(T_pre, np.array(SEED_GRASP))
    q_above, ok2 = arm.ik(T_above, np.array(SEED_PLACE))
    if not (ok1 and ok2):
        raise RuntimeError("IK failed for pre-grasp/above-place (ok=%s,%s)" % (ok1, ok2))
    p_in = cart_path(arm, q_pre, T_pre, T_grasp)
    p_out_a = cart_path(arm, p_in[-1], T_grasp, T_clear, n=8)
    p_out_b = cart_path(arm, p_out_a[-1], T_clear, T_pre, n=20)
    p_down = cart_path(arm, q_above, T_above, T_place)
    home = np.zeros(6)
    via = np.array([0.0, -30.0, -120.0, 0.0, 60.0, 0.0])      # folded pose: keeps the gripper clear of the car while swinging down
    tuck = p_out_b[-1].copy()
    tuck[0] = q_above[0]                                       # pre-grasp arm shape, turned toward the place side
    paths = {
        "home_to_pre": [home, via, q_pre],
        "approach": p_in,
        "pull_studs": p_out_a,
        "pull_clear": p_out_b,
        "pre_to_above": [p_out_b[-1], tuck, q_above],
        "lower": p_down,
        "raise": p_down[::-1],
        "above_to_home": [q_above, tuck, home],
    }
    return arm, paths


def sample(path, s):
    """piecewise-linear joint interpolation, s in [0,1], parameterised by joint travel"""
    seg = [max(float(np.max(np.abs(path[i + 1] - path[i]))), 1e-9) for i in range(len(path) - 1)]
    x = min(max(s, 0.0), 1.0) * sum(seg)
    for i, L in enumerate(seg):
        if x <= L or i == len(seg) - 1:
            return path[i] + (path[i + 1] - path[i]) * min(x / L, 1.0)
        x -= L


# ----------------------------------------------------------------------------- Isaac Sim runtime
def run():
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from omni.physx import get_physx_interface

    stage = omni.usd.get_context().get_stage()
    px = get_physx_interface()
    app = omni.kit.app.get_app()
    arm_attr = [stage.GetPrimAtPath(ROBOT + "/" + jp).GetAttribute("drive:angular:physics:targetPosition") for jp in ARM_JOINTS]
    jaw_attr = [stage.GetPrimAtPath("%s/joints/jaw_%d_joint" % (G, i)).GetAttribute("drive:linear:physics:targetPosition") for i in range(4)]

    def log(msg):
        print("[tire_demo] " + msg)

    timeline = omni.timeline.get_timeline_interface()
    my_id = _STATE["run_id"] = _STATE["run_id"] + 1

    async def tick():
        """wait one frame; returns elapsed demo time (0 while paused). Aborts when stopped or superseded."""
        t0 = time.time()
        await app.next_update_async()
        if _STATE["run_id"] != my_id or timeline.is_stopped():
            raise _Abort()
        return (time.time() - t0) * SPEED if timeline.is_playing() else 0.0

    async def frames(n):
        for _ in range(n):
            await tick()

    async def sleep(sec):
        t = 0.0
        while t < sec:
            t += await tick()

    def set_arm(q_model, sign):
        for a, v in zip(arm_attr, q_model):
            a.Set(float(sign * v))

    def set_jaws(v):
        for a in jaw_attr:
            a.Set(float(v))

    def body_pose(path):
        """physics pose of a rigid body as 4x4, or None"""
        try:
            d = px.get_rigidbody_transformation(path)
            p, r = d["position"], d["rotation"]
            return T_from(np.array([p[0], p[1], p[2]]), quat_to_R(r[3], r[0], r[1], r[2]))
        except Exception:
            return None

    async def move(path, sec, sign):
        t = 0.0
        while True:
            s = min(t / sec, 1.0)
            set_arm(sample(path, smooth(s)), sign)
            if s >= 1.0:
                break
            t += await tick()
        await frames(20)

    async def jaws(a, b, sec):
        t = 0.0
        while True:
            s = min(t / sec, 1.0)
            set_jaws(a + (b - a) * smooth(s))
            if s >= 1.0:
                break
            t += await tick()
        await frames(20)

    def reset_scene_state():
        """put drives/joints back to the initial state so the next PLAY starts clean"""
        set_arm(np.zeros(6), 1.0)
        set_jaws(JAW_OPEN)
        set_joint_enabled(GRASP_JOINT, False)
        set_joint_enabled(STUD_JOINT, True)

    def set_joint_enabled(path, on):
        stage.GetPrimAtPath(path).GetAttribute("physics:jointEnabled").Set(bool(on))

    def wheel_pos():
        T = body_pose(WHEEL)
        return None if T is None else T[:3, 3].copy()

    def jaw_gap(i, j):
        A, B = body_pose("%s/jaw_%d" % (G, i)), body_pose("%s/jaw_%d" % (G, j))
        return None if A is None or B is None else float(np.linalg.norm(A[:3, 3] - B[:3, 3]))

    async def main():
        try:
            for _ in range(120):                      # PLAY event can arrive a few frames before the state flips
                if timeline.is_playing():
                    break
                await app.next_update_async()
            else:
                log("Timeline is not playing. Press PLAY.")
                return
            for a in arm_attr + jaw_attr:
                if not a or not a.IsValid():
                    log("Scene mismatch: open tire_demo_scene_v2.usda")
                    return
            jaw_close = float(scene_param(stage, "jaw_close", 0.05))
            mg, mw = scene_param(stage, "gripper_mass"), scene_param(stage, "wheel_mass")
            log("payload: gripper %.0f kg + wheel %.0f kg = %.0f kg (RS080N limit 80 kg), grasp mode = %s" % (mg, mw, mg + mw, GRASP_MODE))

            # ---- reset state (allows re-running) ----
            set_joint_enabled(GRASP_JOINT, False)
            await frames(2)
            set_joint_enabled(STUD_JOINT, True)
            set_jaws(JAW_OPEN)
            arm0 = Arm(stage, 1.0)
            log("FK check at rest pose: %.2e (should be ~0)" % arm0.rest_err)
            set_arm(np.zeros(6), 1.0)
            await sleep(1.0)

            # ---- joint sign calibration (small test move) ----
            sign = 1.0
            q_test = np.array([8.0, 8.0, 0, 0, 0, 0])
            set_arm(q_test, 1.0)
            await sleep(1.2)
            T6 = body_pose(ROBOT + "/link6")
            if T6 is not None:
                e_pos = np.linalg.norm(arm0.fk6(q_test)[:3, 3] - T6[:3, 3])
                e_neg = np.linalg.norm(arm0.fk6(-q_test)[:3, 3] - T6[:3, 3])
                sign = 1.0 if e_pos <= e_neg else -1.0
                log("sign calibration: err(+)=%.4f m, err(-)=%.4f m -> sign %+d" % (e_pos, e_neg, sign))
                if min(e_pos, e_neg) > 0.03:
                    log("WARNING: FK model does not match physics (>3 cm). Aborting.")
                    set_arm(np.zeros(6), 1.0)
                    return
            else:
                log("physics pose query unavailable -> assuming sign +1")
            set_arm(np.zeros(6), 1.0)
            await sleep(1.0)

            arm, P = plan(stage, sign)
            log("plan OK")
            w0 = wheel_pos()

            log("1/8 move to pre-grasp")
            await move(P["home_to_pre"], 5.0, sign)
            await jaws(JAW_OPEN, JAW_APPROACH, 1.0)
            log("2/8 approach: fingers pass over the tire")
            await move(P["approach"], 3.5, sign)
            T6 = body_pose(ROBOT + "/link6")
            if T6 is not None and w0 is not None:
                tcp = (T6 @ arm.T6_tcp)[:3, 3]
                log("TCP vs wheel center before closing: %.1f mm" % (np.linalg.norm(tcp - w0) * 1000))

            log("3/8 close 4 jaws on the tread")
            g_open = jaw_gap(0, 2)
            await jaws(JAW_APPROACH, jaw_close, 2.0)
            await sleep(0.7)
            g_close = jaw_gap(0, 2)
            if g_open is not None and g_close is not None:
                travel = (g_open - g_close) / 2 * 1000
                log("jaw travel %.1f mm of %.1f mm commanded (stopping short = pads are pressing on the tire)" % (travel, jaw_close * 1000))
            if GRASP_MODE == "joint":
                Tb, Tw = body_pose(G + "/base"), body_pose(WHEEL)
                if Tb is None or Tw is None:
                    Tb = arm.fk6(P["approach"][-1]) @ arm.T6_gbase
                    Tw = mat(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(WHEEL)))
                Tr = np.linalg.inv(Tb) @ Tw
                gj = UsdPhysics.FixedJoint(stage.GetPrimAtPath(GRASP_JOINT))
                gj.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in Tr[:3, 3]]))
                w, x, y, z = R_to_quat(Tr[:3, :3])
                gj.GetLocalRot0Attr().Set(Gf.Quatf(float(w), float(x), float(y), float(z)))
                await frames(2)
                set_joint_enabled(GRASP_JOINT, True)
                await frames(2)

            log("4/8 pull the wheel along the studs")
            await move(P["pull_studs"], 3.0, sign)
            w1 = wheel_pos()
            if w0 is not None and w1 is not None:
                d = float(np.linalg.norm(w1 - w0))
                log("wheel travel along studs: %.1f mm" % (d * 1000))
                if d < 0.03:
                    log("WARNING: wheel did not follow the gripper (grip failed). Try GRASP_MODE = 'joint'. Aborting.")
                    await jaws(jaw_close, JAW_APPROACH, 1.5)
                    await move(P["approach"][::-1], 3.0, sign)
                    return
            set_joint_enabled(STUD_JOINT, False)          # wheel is past the stud tips -> free
            await sleep(0.5)
            log("5/8 withdraw (wheel now carried by the gripper only)")
            await move(P["pull_clear"], 3.5, sign)
            log("6/8 carry to place position (tool turns down)")
            await move(P["pre_to_above"], 6.0, sign)
            w2 = wheel_pos()
            T6 = body_pose(ROBOT + "/link6")
            if w2 is not None and T6 is not None:
                slip = np.linalg.norm((T6 @ arm.T6_tcp)[:3, 3] - w2)
                log("wheel offset from TCP while carried: %.1f mm" % (slip * 1000))
                if slip > 0.15:
                    log("WARNING: wheel was dropped during carry.")
            log("7/8 lower to floor + release")
            await move(P["lower"], 4.0, sign)
            await sleep(0.5)
            if GRASP_MODE == "joint":
                set_joint_enabled(GRASP_JOINT, False)
                await sleep(0.3)
            await jaws(jaw_close, JAW_OPEN, 2.0)
            await sleep(0.8)
            log("8/8 retreat and go home")
            await move(P["raise"], 3.0, sign)
            await move(P["above_to_home"], 5.0, sign)
            w3 = wheel_pos()
            if w3 is not None:
                log("wheel final position: (%.3f, %.3f, %.3f)  [expected about (%.2f, %.2f, 0.11)]" % (w3[0], w3[1], w3[2], PLACE_XY[0], PLACE_XY[1]))
            log("DONE  (press STOP then PLAY to run again)")
        except _Abort:
            if _STATE["run_id"] == my_id:
                reset_scene_state()
                log("stopped -> scene reset. Press PLAY to run again.")
        except Exception as e:
            import traceback
            log("ERROR: %s" % e)
            traceback.print_exc()

    try:
        from omni.kit.async_engine import run_coroutine
        run_coroutine(main())
    except Exception:
        asyncio.ensure_future(main())


class _Abort(Exception):
    pass


_STATE = {"run_id": 0}


def install_auto_run():
    """run the demo now if playing, and again every time the timeline goes STOP -> PLAY"""
    import builtins

    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
    state = {"was_stopped": tl.is_stopped()}

    def on_event(e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            state["was_stopped"] = True
        elif e.type == int(omni.timeline.TimelineEventType.PLAY):
            if state["was_stopped"]:              # fresh start (not a resume from pause)
                state["was_stopped"] = False
                run()

    # keep the subscription alive; replacing it drops the one from an earlier execution of this script
    builtins._tire_demo_sub = tl.get_timeline_event_stream().create_subscription_to_pop(on_event, name="tire_demo_auto_run")
    if tl.is_playing():
        state["was_stopped"] = False
        run()
    else:
        print("[tire_demo] ready: press PLAY to start (auto re-runs after STOP -> PLAY)")


try:
    import omni.kit.app  # noqa: F401
    _IN_ISAAC = True
except ImportError:
    _IN_ISAAC = False

if _IN_ISAAC:
    install_auto_run()
