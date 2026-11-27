# Tire cell demo v3 for Isaac Sim 5.x  (run via run_cell_demo.py, or in Script Editor: PLAY then run)
# Scene: tire_cell_v3.usda
#   Robot B (RS080N + QST62-230CT nutrunner): visits the 5 nut positions in star order, spins the spindle, retreats
#   Robot A (RS080N + 4-jaw wheel gripper)  : grips the tread, pulls the wheel off the studs, lays it on the floor
#
# What is physical here: car body (dynamic, on lift pads), 4 suspension corners (spring/damper), hub pilot + 5 studs,
# the 22 kg wheel hanging on them by contact, the jaws squeezing the tire. What is NOT simulated: nut threads/torque
# (the 5 nuts are one FixedJoint "nut_clamp" released after the 5th spin), the nutrunner's 50 mm spindle travel.
#
# Motion is kinematic: joint position targets from the IK in cell_kinematics.py (no torque control).
# Extension points: MotionBackend (swap the IK / send targets to ROS 2), ContactMonitor (collision evidence), params in scene customData.
import asyncio
import json
import os
import time

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics

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

SEED_A_GRASP = [0.0, -10.0, -125.0, 0.0, 30.0, 0.0]
SEED_A_PLACE = [-45.0, 15.0, -90.0, 0.0, -75.0, 45.0]
SEED_B_NUT = [-37.5, 15.0, -103.8, 85.9, 82.6, -60.9]

R_TOOL_X = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)      # tool axis (tcp -Z) -> world +X
R_TOOL_DOWN = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)    # tool axis -> world -Z


def cell(stage, key, default=None):
    v = stage.GetPrimAtPath("/World").GetCustomDataByKey("cell:" + key)
    return default if v is None else v


# ----------------------------------------------------------------------------- planning (pure; testable offline)
def plan_a(arm, wheel_center, tip_below_tcp, stud_exposed):
    wc = np.array(wheel_center, dtype=float)
    T_grasp = T_from(wc, R_TOOL_X)
    T_clear = T_from(wc - np.array([stud_exposed + 0.03, 0, 0]), R_TOOL_X)
    T_pre = T_from(wc - np.array([APPROACH_A, 0, 0]), R_TOOL_X)
    T_place = T_from(np.array([PLACE_XY[0], PLACE_XY[1], tip_below_tcp + PLACE_CLEAR]), R_TOOL_DOWN)
    T_above = T_from(np.array([PLACE_XY[0], PLACE_XY[1], ABOVE_Z]), R_TOOL_DOWN)
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
            "pre_to_above": [p_out_b[-1], tuck, q_above], "lower": p_down, "raise": p_down[::-1], "above_to_home": [q_above, tuck, home]}


def plan_b(arm, nut_points_world, order):
    """order: 1-based nut numbers. Returns home->first stand-off, per-nut (in, out, transfer to next), last->home."""
    pts = [np.array(nut_points_world[i - 1], dtype=float) for i in order]
    back = np.array([APPROACH_B, 0, 0])
    q0, ok = arm.ik(T_from(pts[0] - back, R_TOOL_X), np.array(SEED_B_NUT))
    if not ok:
        raise RuntimeError("robot B: IK failed for the first nut stand-off")
    home = np.zeros(6)
    via = np.array([q0[0], -30.0, -120.0, q0[3], q0[4], q0[5]])
    steps, q = [], q0
    for k, p in enumerate(pts):
        T_off, T_on = T_from(p - back, R_TOOL_X), T_from(p, R_TOOL_X)
        p_in = cart_path(arm, q, T_off, T_on, n=8)
        p_out = p_in[::-1]
        nxt = None
        if k + 1 < len(pts):
            nxt = cart_path(arm, p_out[-1], T_off, T_from(pts[k + 1] - back, R_TOOL_X), n=6)
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

    def log(msg):
        print("[cell_demo] " + msg)

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
            drive.command(sample(path, smooth(s)), sign)
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

    def joint_enabled(path, on):
        p = stage.GetPrimAtPath(path)
        if p.IsValid():
            p.GetAttribute("physics:jointEnabled").Set(bool(on))

    def nut_visible(i, on):
        p = stage.GetPrimAtPath("%s/nuts/nut_%d" % (CORNER, i))
        if p.IsValid():
            img = UsdGeom.Imageable(p)
            img.MakeVisible() if on else img.MakeInvisible()

    def reset_scene_state():
        drive_a.command(np.zeros(6), 1.0)
        drive_b.command(np.zeros(6), 1.0)
        set_track(0.0, 0.0)
        set_jaws(JAW_OPEN)
        spin_attr.Set(0.0)
        joint_enabled(GRASP_JOINT, False)
        joint_enabled(STUD_JOINT, False)
        joint_enabled(NUT_CLAMP, True)
        for i in range(5):
            nut_visible(i, True)

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
                        self.pairs[key] = {"t": simclock["t"] - self.t0, "phase": self.phase, "n": 1, "fmax": f}
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
            if tool and b.startswith(CAR + "/"):
                return "TOOL-BODY (scratch risk)"
            if tool and b.startswith(CORNER + "/"):
                return "TOOL-HUB/BRAKE (scratch risk)"
            if "/Nutrunner/" in a and b.startswith(WHEEL + "/"):
                return "NUTRUNNER-WHEEL (scratch risk)"
            if tool and b.startswith(WHEEL + "/"):
                soft = a.endswith("/pad") or "/lip_" in a
                return "gripper pad-tire (expected)" if (soft and "/tire_" in b) else "GRIPPER-WHEEL metal contact (scratch risk)"
            if (c0.startswith(WHEEL + "/") and c1.startswith(CAR + "/")) or (c1.startswith(WHEEL + "/") and c0.startswith(CAR + "/")):
                return "WHEEL-BODY (scratch risk)"
            if (c0.startswith(WHEEL + "/") and c1.startswith(CORNER + "/")) or (c1.startswith(WHEEL + "/") and c0.startswith(CORNER + "/")):
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

            Tw = body_pose(WHEEL)
            if Tw is None:
                Tw = mat(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(WHEEL)))
            log("wheel center measured: (%.4f, %.4f, %.4f)" % tuple(Tw[:3, 3]))

            # =============================== robot B: nuts ===============================
            if not SKIP_NUTS:
                mon.phase = "nutrunner"
                nut_local = json.loads(str(cell(stage, "nut_points_wheel_local")))
                order = json.loads(str(cell(stage, "nut_order", "[1,3,5,2,4]")))
                nut_world = [(Tw @ np.array([p[0], p[1], p[2], 1.0]))[:3] for p in nut_local]
                PB = plan_b(arm_b, nut_world, order)
                rpm, spin_s = float(cell(stage, "spin_rpm", 330.0)), float(cell(stage, "spin_seconds", 2.0))
                log("B 1/3 nutrunner to the wheel (order %s, %.0f r/min)" % (order, rpm))
                await move(drive_b, arm_b, PB["home_to_first"], 5.0, sign)
                for st in PB["steps"]:
                    await move(drive_b, arm_b, st["in"], 1.5, sign)
                    spin_attr.Set(-rpm * 6.0)                   # loosen (deg/s)
                    await sleep(spin_s)
                    spin_attr.Set(0.0)
                    nut_visible(st["nut"] - 1, False)            # nut is taken away inside the socket
                    log("B 2/3 nut %d: spindle spun %.1f s, retract" % (st["nut"], spin_s))
                    await move(drive_b, arm_b, st["out"], 1.2, sign)
                    if st["next"] is not None:
                        await move(drive_b, arm_b, st["next"], 1.0, sign)
                if mount == "joint":
                    joint_enabled(STUD_JOINT, True)
                    await frames(2)
                joint_enabled(NUT_CLAMP, False)                  # all 5 nuts off -> wheel is held by pilot + studs only
                log("B 3/3 all nuts off; nutrunner goes home")
                await move(drive_b, arm_b, PB["last_to_home"], 5.0, sign)
                await sleep(0.5)
                Tw2 = body_pose(WHEEL)
                if Tw2 is not None:
                    d = Tw2[:3, 3] - Tw[:3, 3]
                    c = max(-1.0, min(1.0, float(Tw2[:3, 2] @ Tw[:3, 2])))
                    tilt = float(np.degrees(np.arccos(c)))
                    log("wheel moved after nut removal: %.2f mm, tilt %.2f deg (hanging on pilot/studs)" % (np.linalg.norm(d) * 1000, tilt))
                    Rrel = Tw2[:3, :3] @ Tw[:3, :3].T
                    ax = np.array([Rrel[2, 1] - Rrel[1, 2], Rrel[0, 2] - Rrel[2, 0], Rrel[1, 0] - Rrel[0, 1]])
                    ax = ax / (np.linalg.norm(ax) + 1e-12)
                    log("  detail: displacement (%.2f, %.2f, %.2f) mm world, rotation axis (%.2f, %.2f, %.2f), outer-face normal now (%.3f, %.3f, %.3f)"
                        % (d[0] * 1000, d[1] * 1000, d[2] * 1000, ax[0], ax[1], ax[2], Tw2[0, 2], Tw2[1, 2], Tw2[2, 2]))
                    if np.linalg.norm(d) * 1000 > LIM.get("wheel_sag_mm", 3.0) or tilt > LIM.get("wheel_tilt_deg", 1.0):
                        if SAFETY:
                            raise _Protect("wheel is not seated (moved %.1f mm, tilted %.1f deg) -> not grasping it" % (np.linalg.norm(d) * 1000, tilt))
                        log("WARNING: wheel sagged/tilted more than a seated wheel should. Grasp may be poor.")
                    if np.linalg.norm(d) > 0.03:
                        log("WARNING: wheel did not stay on the studs. Re-run with --mount joint.")
                        return
                    Tw = Tw2
            else:
                if mount == "joint":
                    joint_enabled(STUD_JOINT, True)
                    await frames(2)
                joint_enabled(NUT_CLAMP, False)
                await sleep(0.5)

            # =============================== robot A: wheel ===============================
            mon.phase = "gripper"
            PA = plan_a(arm_a, Tw[:3, 3], float(cell(stage, "tip_below_tcp")), float(cell(stage, "stud_exposed", 0.045)))
            w0 = Tw[:3, 3].copy()

            def wheel_pos():
                T = body_pose(WHEEL)
                return None if T is None else T[:3, 3].copy()

            def jaw_gap():
                a, b = body_pose(GA + "/jaw_0"), body_pose(GA + "/jaw_2")
                return None if a is None or b is None else float(np.linalg.norm(a[:3, 3] - b[:3, 3]))

            log("A 1/8 move to pre-grasp")
            await move(drive_a, arm_a, PA["home_to_pre"], 5.0, sign)
            await jaws(JAW_OPEN, JAW_APPROACH, 1.0)
            log("A 2/8 approach: fingers pass over the tire")
            await move(drive_a, arm_a, PA["approach"], 3.5, sign)
            log("A 3/8 close 4 jaws on the tread")
            g0 = jaw_gap()
            await jaws(JAW_APPROACH, jaw_close, 2.0)
            await sleep(0.7)
            g1 = jaw_gap()
            if g0 is not None and g1 is not None:
                log("jaw travel %.1f mm of %.1f mm commanded (stopping short = pads pressing on the tire)"
                    % ((g0 - g1) / 2 * 1000 + JAW_APPROACH * 1000, jaw_close * 1000))
            if GRASP_MODE == "joint":
                Tb, Tw3 = body_pose(GA + "/base"), body_pose(WHEEL)
                if Tb is None or Tw3 is None:
                    Tb, Tw3 = arm_a.fk6(PA["approach"][-1]) @ arm_a.T6_tool, Tw
                Tr = np.linalg.inv(Tb) @ Tw3
                gj = UsdPhysics.FixedJoint(stage.GetPrimAtPath(GRASP_JOINT))
                gj.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in Tr[:3, 3]]))
                w, x, y, z = R_to_quat(Tr[:3, :3])
                gj.GetLocalRot0Attr().Set(Gf.Quatf(float(w), float(x), float(y), float(z)))
                await frames(2)
                joint_enabled(GRASP_JOINT, True)
                await frames(2)
            log("A 4/8 pull the wheel along the studs")
            await move(drive_a, arm_a, PA["pull_studs"], 3.0, sign)
            w1 = wheel_pos()
            if w1 is not None:
                d = float(np.linalg.norm(w1 - w0))
                log("wheel travel along studs: %.1f mm" % (d * 1000))
                if d < 0.03:
                    log("WARNING: wheel did not follow the gripper (grip failed). Try --grasp joint. Aborting.")
                    await jaws(jaw_close, JAW_APPROACH, 1.5)
                    await move(drive_a, arm_a, PA["approach"][::-1], 3.0, sign)
                    return
            joint_enabled(STUD_JOINT, False)
            await sleep(0.5)
            log("A 5/8 withdraw (wheel carried by the gripper only)")
            await move(drive_a, arm_a, PA["pull_clear"], 3.5, sign)
            log("A 6/8 carry to place position (tool turns down)")
            await move(drive_a, arm_a, PA["pre_to_above"], 6.0, sign)
            w2, T6 = wheel_pos(), body_pose(RA + "/link6")
            if w2 is not None and T6 is not None:
                slip = float(np.linalg.norm((T6 @ arm_a.T6_tcp)[:3, 3] - w2))
                log("wheel offset from TCP while carried: %.1f mm" % (slip * 1000))
                if slip > 0.15:
                    log("WARNING: wheel was dropped during carry.")
            log("A 7/8 lower to floor + release")
            await move(drive_a, arm_a, PA["lower"], 4.0, sign)
            await sleep(0.5)
            if GRASP_MODE == "joint":
                joint_enabled(GRASP_JOINT, False)
                await sleep(0.3)
            await jaws(jaw_close, JAW_OPEN, 2.0)
            await sleep(0.8)
            log("A 8/8 retreat and go home")
            await move(drive_a, arm_a, PA["raise"], 3.0, sign)
            await move(drive_a, arm_a, PA["above_to_home"], 5.0, sign)
            w3 = wheel_pos()
            if w3 is not None:
                log("wheel final position: (%.3f, %.3f, %.3f)  [expected about (%.2f, %.2f, 0.11)]" % (w3[0], w3[1], w3[2], PLACE_XY[0], PLACE_XY[1]))
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
            mon.report()
            log("DONE  (press STOP then PLAY to run again)")
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


def install_auto_run():
    """run now if playing, and again every time the timeline goes STOP -> PLAY"""
    import builtins

    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
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
