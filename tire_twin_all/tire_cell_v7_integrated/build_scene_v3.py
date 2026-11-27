"""
타이어 교체 셀 v4 씬 생성 (풀린 너트 유지판)

  로봇 A (RS080N + 4조 휠 그리퍼)  : 휠 탈거
  로봇 B (RS080N + QST62-230CT 너트러너) : 너트 5개 위치에서 스핀들 회전 (나사 체결은 미구현)
  차량: 차체 = 동적 강체(스프링 상 질량), 리프트 패드 위에 접촉으로 얹힘
        코너 4개 = 스프링 하 질량(허브·디스크·캘리퍼·너클) + 서스펜션(직선 관절 + 스프링/댐퍼)
        FL 휠 = 22 kg 강체. 허브 파일럿·스터드에 '접촉'으로 걸려 있고, 너트는 고정 관절(nut_clamp)로 표현

값은 전부 params.json 에서 읽는다.
실행: python build_scene_v3.py <출력폴더> <car_demo 저장소 경로>
"""
import json
import math
import os
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
import nut_tray as nt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else HERE
CAR_REPO = sys.argv[2] if len(sys.argv) > 2 else None
_argv = sys.argv
sys.argv = [sys.argv[0], OUT_DIR]
sys.path.insert(0, HERE)
import build_scene_v2 as b2  # noqa: E402  (형상 도우미 재사용)
import camera_mount  # noqa: E402  (손목 카메라 RealSense D455)
import track_mount  # noqa: E402  (리니어 트랙 1개 + 캐리지 2개)

sys.argv = _argv
P = json.load(open(os.path.join(HERE, "params.json"), encoding="utf-8"))
W, C, NR = P["wheel"], P["car"], P["nutrunner"]

ROBOT_FILE = "rs080n_pzv160_flat.usda"
SCENE = os.path.join(OUT_DIR, "tire_cell_v3.usda")
GRIP_FILE = os.path.join(OUT_DIR, "wheel_gripper_4jaw.usda")
NUT_FILE = os.path.join(OUT_DIR, "nutrunner_qst62.usda")
CAR_FILE = os.path.join(OUT_DIR, "car", "prius_body.usda")

TIRE_R, HW = W["tire_radius"], W["tire_width"] / 2
PCD_R, BORE_R = W["pcd_radius"], W["hub_bore_radius"]
PAD_Z0, PAD_Z1 = b2.PAD_Z0, b2.PAD_Z1
WHEEL_C = Gf.Vec3d(1.85, 0.0, 1.00)
DROOP = C["suspension_droop"]
CAR_WHEELS, CAR_WHEEL_Z = b2.CAR_WHEELS, b2.CAR_WHEEL_Z
b2.DROOP = DROOP
TIGHT = 0.0004           # 좁은 틈 부위 contactOffset [m]


# ------------------------------------------------------------------ 도우미
def phys_material(stage, name):
    m = P["materials"][name]
    mat = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/" + name)
    a = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    a.CreateStaticFrictionAttr(m["static"])
    a.CreateDynamicFrictionAttr(m["dynamic"])
    a.CreateRestitutionAttr(m["restitution"])
    return mat


def collide(prim, pm, hull=False, tight=False):
    UsdPhysics.CollisionAPI.Apply(prim)
    if hull:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    if pm is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
    if tight:
        prim.AddAppliedSchema("PhysxCollisionAPI")
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(TIGHT)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)


def hull(stage, path, pts, pm, tight=False):
    """점 집합의 볼록 껍질 충돌체 (보이지 않음). 면 정보는 PhysX 가 다시 계산하므로 간단히 넣는다."""
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pts])
    n = len(pts)
    m.CreateFaceVertexCountsAttr([3] * (n - 2))
    m.CreateFaceVertexIndicesAttr([i for k in range(1, n - 1) for i in (0, k, k + 1)])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreatePurposeAttr("guide")
    collide(m.GetPrim(), pm, hull=True, tight=tight)
    return m


def wedge(stage, path, r_in, r_out, a_c, a_half, z0, z1, pm, center=(0.0, 0.0), tight=False):
    """각도 a_c±a_half [deg] 의 쐐기. 안쪽 현의 중점이 r_in, 바깥 평면의 중점이 r_out 이 되도록 꼭짓점을 잡는다."""
    c = math.cos(math.radians(a_half))
    pts = []
    for r in (r_in / c, r_out / c):
        for s in (-1, 1):
            a = math.radians(a_c + s * a_half)
            for z in (z0, z1):
                pts.append((center[0] + r * math.cos(a), center[1] + r * math.sin(a), z))
    return hull(stage, path, pts, pm, tight)


def col_cyl(stage, path, r, z0, z1, pm, center=(0.0, 0.0), tight=False, n=None):
    """원기둥 충돌체를 n각 기둥 볼록 껍질로 만든다 (꼭짓점이 반지름 r 위 = 원기둥 안쪽에 내접).
    Cylinder 프림 충돌은 2차 실행에서 허브 장착면/파일럿이 휠을 받치지 못했으므로 쓰지 않는다."""
    if n is None:
        n = 12 if r < 0.01 else 32
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n
        for z in (z0, z1):
            pts.append((center[0] + r * math.cos(a), center[1] + r * math.sin(a), z))
    return hull(stage, path, pts, pm, tight)


def contact_report(prim):
    prim.AddAppliedSchema("PhysxContactReportAPI")
    prim.CreateAttribute("physxContactReport:threshold", Sdf.ValueTypeNames.Float).Set(0.0)


def lug_xy(i):
    a = math.radians(b2.lug_angle(i))
    return PCD_R * math.cos(a), PCD_R * math.sin(a)


# ------------------------------------------------------------------ 휠 충돌체 (휠 로컬, +Z = 바깥)
def wheel_colliders(stage, root, pm_rubber, pm_steel):
    Cc = root + "/colliders"
    for k in range(24):                                   # 타이어+림: 속이 빈 링 (허브·디스크와 겹치지 않게)
        wedge(stage, f"{Cc}/tire_{k}", 0.199, TIRE_R, 15.0 * k, 7.5, -HW, HW, pm_rubber)
    for k in range(16):                                   # 허브 보어 / 장착 패드
        wedge(stage, f"{Cc}/bore_{k}", BORE_R + W["bore_clearance_sim"], 0.0485, 22.5 * k, 11.25, PAD_Z0, PAD_Z1, pm_steel, tight=True)
        wedge(stage, f"{Cc}/pad_outer_{k}", 0.066, 0.092, 22.5 * k, 11.25, PAD_Z0, PAD_Z1, pm_steel, tight=True)
    for i in range(W["lugs"]):                            # 러그 구멍 사이 살 + 스터드 구멍 링
        a = b2.lug_angle(i)
        wedge(stage, f"{Cc}/web_{i}", 0.0485, 0.066, a + 36.0, 22.0, PAD_Z0, PAD_Z1, pm_steel, tight=True)
        for k in range(8):
            wedge(stage, f"{Cc}/hole_{i}_{k}", W["stud_hole_radius"], 0.0138, 45.0 * k, 22.5, PAD_Z0, PAD_Z1, pm_steel, center=lug_xy(i), tight=True)
    for k in range(10):                                   # 스포크: 외형 큐브에 그대로 충돌 적용
        collide(stage.GetPrimAtPath(f"{root}/spoke_{k}"), pm_steel)


def nuts(stage, root):
    nutm = b2.material(stage, "lug_nut", (0.6, 0.6, 0.62), 1.0, 0.25)
    for i in range(W["lugs"]):
        b2.lathe(stage, f"{root}/nut_{i}", [(0.0, PAD_Z1 - 0.004), (0.0121, PAD_Z1 - 0.004), (0.0121, PAD_Z1 + 0.014), (0.009, PAD_Z1 + 0.020), (0.0, PAD_Z1 + 0.020)],
                 nutm, n=6, center=lug_xy(i))


def loose_nuts(stage, spindle_path, socket_tip_z):
    """트레이에 놓인 너트와 소켓에 붙어 이동할 표시용 너트 형상을 만든다."""
    loose_root = UsdGeom.Xform.Define(stage, "/World/LooseNuts")
    loose_root.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    nutm = b2.material(stage, "lug_nut_loose", (0.64, 0.64, 0.66), 1.0, 0.25)
    for i in range(W["lugs"]):
        root = UsdGeom.Xform.Define(stage, f"/World/LooseNuts/nut_{i}")
        root.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        root.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))
        b2.lathe(stage, f"/World/LooseNuts/nut_{i}/mesh",
                 [(0.0, 0.0), (0.0121, 0.0), (0.0121, 0.014),
                  (0.009, 0.020), (0.0, 0.020)],
                 nutm, n=6)
        root.GetPrim().SetCustomDataByKey("loose_nut:index", i + 1)
        UsdGeom.Imageable(root.GetPrim()).MakeInvisible()

        carried = UsdGeom.Xform.Define(stage, f"{spindle_path}/carried_nut_{i}")
        carried.AddTranslateOp().Set(Gf.Vec3d(0, 0, socket_tip_z))
        b2.lathe(stage, f"{spindle_path}/carried_nut_{i}/mesh",
                 [(0.0, 0.0), (0.0121, 0.0), (0.0121, 0.014),
                  (0.009, 0.020), (0.0, 0.020)],
                 nutm, n=6)
        carried.GetPrim().SetCustomDataByKey("loose_nut:index", i + 1)
        UsdGeom.Imageable(carried.GetPrim()).MakeInvisible()

# ------------------------------------------------------------------ 코너(스프링 하): 허브·브레이크·너클 (로컬 +Z = 바깥)
def build_corner(stage, root, up, pm_steel, with_colliders):
    steel = b2.material(stage, "steel", (0.45, 0.45, 0.47), 1.0, 0.45)
    cast = b2.material(stage, "cast_iron", (0.2, 0.2, 0.21), 0.6, 0.7)
    red = b2.material(stage, "caliper_red", (0.75, 0.05, 0.05), 0.2, 0.4)
    studm = b2.material(stage, "stud", (0.8, 0.72, 0.35), 1.0, 0.3)
    black = b2.material(stage, "susp_black", (0.06, 0.06, 0.06), 0.3, 0.6)
    b2.cyl(stage, root + "/disc_hat_face", 0.078, 0.008, (0, 0, PAD_Z0 - 0.004), steel)
    b2.lathe(stage, root + "/disc_hat_wall", [(0.068, -0.012), (0.078, -0.012), (0.078, PAD_Z0 - 0.008), (0.068, PAD_Z0 - 0.008)], steel)
    b2.lathe(stage, root + "/disc_plate_out", [(0.068, -0.014), (0.150, -0.014), (0.150, -0.005), (0.068, -0.005)], steel, n=72)
    b2.lathe(stage, root + "/disc_plate_in", [(0.085, -0.031), (0.150, -0.031), (0.150, -0.022), (0.085, -0.022)], steel, n=72)
    b2.lathe(stage, root + "/disc_vanes", [(0.090, -0.022), (0.146, -0.022), (0.146, -0.014), (0.090, -0.014)], cast, n=36)
    b2.lathe(stage, root + "/hub_pilot", [(0.022, PAD_Z0), (BORE_R, PAD_Z0), (BORE_R, PAD_Z0 + W["pilot_length"]), (0.022, PAD_Z0 + W["pilot_length"])], steel)
    b2.lathe(stage, root + "/axle_nut", [(0.0, PAD_Z0), (0.018, PAD_Z0), (0.018, PAD_Z0 + 0.010), (0.0, PAD_Z0 + 0.010)], cast, n=6)
    for i in range(W["lugs"]):
        x, y = lug_xy(i)
        b2.cyl(stage, f"{root}/stud_{i}", W["stud_radius"], W["stud_exposed"] + 0.004, (x, y, PAD_Z0 - 0.004 + (W["stud_exposed"] + 0.004) / 2), studm)
    a0 = 20.0 if up > 0 else 200.0
    cal = [b2.sector(stage, root + "/caliper_body", 0.100, 0.176, a0, a0 + 56.0, -0.052, 0.010, red, n=8),
           b2.sector(stage, root + "/caliper_bridge", 0.150, 0.180, a0 + 6.0, a0 + 50.0, -0.050, 0.008, red, n=6)]
    kn = b2.box(stage, root + "/knuckle", (0.20, 0.09, 0.05), (0.0, 0.0, -0.062), cast)
    b2.cyl(stage, root + "/wheel_bearing", 0.055, 0.05, (0, 0, -0.05), cast)
    strut = b2.cyl(stage, root + "/strut", 0.028, 0.40, (up * 0.28, 0.0, -0.165), black, axis="X")     # 타이어 안쪽면(-0.1125)보다 안쪽
    ss = b2.lathe(stage, root + "/spring_seat", [(0.03, 0.0), (0.075, 0.0), (0.075, 0.012), (0.03, 0.012)], black, n=24)
    sx = UsdGeom.Xformable(ss.GetPrim())
    sx.AddTranslateOp().Set(Gf.Vec3d(up * 0.26, 0.0, -0.165))
    sx.AddRotateYOp().Set(90.0)
    b2.box(stage, root + "/lower_arm", (0.045, 0.07, 0.34), (-up * 0.105, 0.0, -0.24), black, rot_y=up * 8.0)
    b2.cyl(stage, root + "/tie_rod", 0.009, 0.34, (up * 0.02, 0.11, -0.24), steel)
    b2.cyl(stage, root + "/drive_shaft", 0.016, 0.42, (0, 0, -0.29), steel)
    b2.lathe(stage, root + "/cv_boot", [(0.016, -0.20), (0.045, -0.10), (0.045, -0.085), (0.016, -0.085)], black, n=24)
    if with_colliders:
        Cc = root + "/colliders"
        col_cyl(stage, Cc + "/hat", 0.078, -0.012, PAD_Z0, pm_steel, tight=True)                  # 휠 장착면
        col_cyl(stage, Cc + "/disc", 0.150, -0.031, -0.005, pm_steel)
        col_cyl(stage, Cc + "/pilot", BORE_R, PAD_Z0, PAD_Z0 + W["pilot_length"], pm_steel, tight=True)
        for i in range(W["lugs"]):
            col_cyl(stage, f"{Cc}/stud_{i}", W["stud_radius"], PAD_Z0, PAD_Z0 + W["stud_exposed"], pm_steel, center=lug_xy(i), tight=True)
        for c in cal:
            collide(c.GetPrim(), pm_steel, hull=True)
        collide(kn.GetPrim(), pm_steel)
        collide(strut.GetPrim(), pm_steel)


# ------------------------------------------------------------------ 너트러너 (QST62-230CT 치수 기반)
def build_nutrunner(path_file):
    if os.path.exists(path_file):
        os.remove(path_file)
    stage = Usd.Stage.CreateNew(path_file)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    R = "/NutrunnerQST62"
    L = R + "/Looks"
    root = UsdGeom.Xform.Define(stage, R)
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    for k in ("model", "torque_min", "torque_max", "speed_rpm", "travel", "mass", "gear_ratio"):
        root.GetPrim().SetCustomDataByKey("spec:" + k, NR[k])
    root.GetPrim().SetCustomDataByKey("source", "parametric model from Atlas Copco QST62-230CT published data (not vendor CAD)")
    yel = b2.material(stage, "ac_yellow", (0.93, 0.72, 0.05), 0.3, 0.45, root=L)
    dark = b2.material(stage, "ac_dark", (0.10, 0.10, 0.11), 0.5, 0.5, root=L)
    steel = b2.material(stage, "ac_steel", (0.62, 0.63, 0.65), 1.0, 0.25, root=L)
    z1 = -0.020
    z2 = z1 - NR["length_body"]
    z3 = z2 - NR["length_front"]
    sock_len = 0.078
    z4 = z3 - sock_len
    B = R + "/base"
    base = UsdGeom.Xform.Define(stage, B)
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(NR["mass"] + NR["bracket_mass"] - 1.2)
    b2.cyl(stage, B + "/adapter_plate", 0.10, 0.020, (0, 0, -0.010), dark)
    b2.box(stage, B + "/bracket", (0.11, 0.11, 0.09), (0, 0, z1 - 0.045), dark)
    b2.box(stage, B + "/bracket_clamp", (0.11, 0.11, 0.05), (0, 0, z1 - 0.30), dark)
    # 뒤에서부터: 커넥터 / 모터 / 기어+트랜스듀서 / 앞부분(스프링 스핀들 하우징)
    b2.lathe(stage, B + "/motor", [(0.0, z1 - 0.002), (0.031, z1 - 0.002), (0.031, z1 - 0.26), (0.0, z1 - 0.26)], yel, n=40)
    b2.lathe(stage, B + "/gear_housing", [(0.0, z1 - 0.26), (0.034, z1 - 0.26), (0.034, z1 - 0.40), (0.029, z2), (0.0, z2)], dark, n=40)
    b2.lathe(stage, B + "/transducer_ring", [(0.034, z1 - 0.33), (0.037, z1 - 0.33), (0.037, z1 - 0.37), (0.034, z1 - 0.37)], yel, n=40)
    b2.lathe(stage, B + "/front_housing", [(0.0, z2), (0.024, z2), (0.024, z2 - 0.09), (0.019, z2 - 0.10), (0.0, z2 - 0.10)], steel, n=32)
    b2.cyl(stage, B + "/cable_connector", 0.016, 0.07, (0.045, 0, z1 - 0.05), dark, axis="X")
    cc = col_cyl(stage, B + "/col_body", 0.036, z2, z1, None)
    cf = col_cyl(stage, B + "/col_front", 0.024, z2 - 0.10, z2, None)
    del cc, cf
    tcp = UsdGeom.Xform.Define(stage, B + "/tcp")                       # 소켓 입구 중심
    tcp.AddTranslateOp().Set(Gf.Vec3d(0, 0, z4))
    # 스핀들 + 21 mm 딥 소켓 (회전체). 소켓은 속이 비어 있어 충돌체 없음
    S = R + "/spindle"
    sp = UsdGeom.Xform.Define(stage, S)
    UsdPhysics.RigidBodyAPI.Apply(sp.GetPrim())
    ma = UsdPhysics.MassAPI.Apply(sp.GetPrim())
    ma.CreateMassAttr(1.2)
    ma.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, (z2 + z4) / 2))
    ma.CreateDiagonalInertiaAttr(Gf.Vec3f(0.006, 0.006, 0.0002))
    b2.lathe(stage, S + "/spindle_shaft", [(0.0, z2 - 0.10), (0.011, z2 - 0.10), (0.011, z3), (0.0, z3)], steel, n=24)
    b2.box(stage, S + "/square_drive", (0.019, 0.019, 0.02), (0, 0, z3 + 0.006), steel)
    b2.lathe(stage, S + "/socket", [(0.0125, z4), (0.015, z4), (0.015, z3 - 0.012), (0.013, z3), (0.0, z3), (0.0, z3 - 0.02), (0.0125, z3 - 0.02)], dark, n=6)
    b2.box(stage, S + "/socket_mark", (0.004, 0.002, sock_len * 0.6), (0.0152, 0, (z3 + z4) / 2), yel)       # 회전이 보이도록 표시
    j = UsdPhysics.RevoluteJoint.Define(stage, R + "/joints/spindle_joint")
    j.CreateBody0Rel().SetTargets([base.GetPath()])
    j.CreateBody1Rel().SetTargets([sp.GetPath()])
    j.CreateAxisAttr("Z")
    j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")              # 속도 제어: stiffness 0, 목표 속도 추종
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(50.0)
    d.CreateMaxForceAttr(NR["torque_max"])                              # 최대 토크 230 N·m
    d.CreateTargetVelocityAttr(0.0)
    j.GetPrim().AddAppliedSchema("PhysxJointAPI")
    j.GetPrim().CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float).Set(NR["speed_rpm"] * 6.0)   # 330 r/min = 1980 deg/s
    stage.GetRootLayer().Save()
    b2.MATS.clear()
    return -z4


# ------------------------------------------------------------------ 로봇 + 공구 결합
def add_robot(stage, name, tool_file, tool_name, base_xy=(0.0, 0.0), yaw_deg=0.0):
    R = "/World/" + name
    robot = UsdGeom.Xform.Define(stage, R).GetPrim()
    robot.GetReferences().AddReference("./" + ROBOT_FILE, "/World/RS080N")
    xf = UsdGeom.Xformable(robot)
    q = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg).GetQuat()
    for op in xf.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            op.Set(Gf.Vec3d(base_xy[0], base_xy[1], 0.0))
        elif op.GetOpName() == "xformOp:orient":
            if op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
                op.Set(Gf.Quatd(q.GetReal(), q.GetImaginary()))
            else:
                op.Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
    rj = stage.OverridePrim(R + "/root_joint")                          # world 고정 관절도 같은 자리로
    rj.GetAttribute("physics:localPos0").Set(Gf.Vec3f(base_xy[0], base_xy[1], 0.0))
    rj.GetAttribute("physics:localRot0").Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
    robot.AddAppliedSchema("PhysxArticulationAPI")
    robot.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(P["sim"]["articulation_position_iterations"])
    robot.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(P["sim"]["articulation_velocity_iterations"])
    stage.OverridePrim(R + "/Gripper").SetActive(False)                 # 합본 파일에 들어 있던 예전 그리퍼
    # 관절 최대 토크: 에셋 기본값(60,000 N·m)은 1.4 t 차량도 끌고 갈 만큼 비현실적 -> 가반하중 기준 추정값으로 제한
    for jp, tq in zip(["base_link/joint1", "link1/joint2", "link2/joint3", "link3/joint4", "link4/joint5", "link5/joint6"], P["robot"]["joint_max_torque"]):
        stage.OverridePrim(R + "/" + jp).GetAttribute("drive:angular:physics:maxForce").Set(float(tq))
    T = R + "/" + tool_name
    tool = UsdGeom.Xform.Define(stage, T)
    tool.GetPrim().GetReferences().AddReference("./" + os.path.basename(tool_file))
    tool.GetPrim().RemoveAPI(UsdPhysics.ArticulationRootAPI)
    xc = UsdGeom.XformCache()
    link6 = stage.GetPrimAtPath(R + "/link6")
    flip = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))  # 공구 -Z -> 플랜지 +Z
    tool.ClearXformOpOrder()
    tool.AddTransformOp().Set(flip * xc.GetLocalToWorldTransform(link6) * xc.GetLocalToWorldTransform(robot).GetInverse())
    fj = UsdPhysics.FixedJoint.Define(stage, T + "/flange_mount_joint")
    fj.CreateBody0Rel().SetTargets([link6.GetPath()])
    fj.CreateBody1Rel().SetTargets([Sdf.Path(T + "/base")])
    fj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot0Attr(Gf.Quatf(0, 1, 0, 0))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    links = [Sdf.Path(f"{R}/{n}") for n in ["world", "base_link", "link1", "link2", "link3", "link4", "link5", "link6"]]
    bodies = [p.GetPath() for p in Usd.PrimRange(tool.GetPrim()) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for b in bodies:
        UsdPhysics.FilteredPairsAPI.Apply(stage.OverridePrim(b)).CreateFilteredPairsRel().SetTargets(links)
        contact_report(stage.OverridePrim(b))
    return T, bodies


# ------------------------------------------------------------------ 씬
def main():
    b2.build_gripper(GRIP_FILE)
    tcp_nr = build_nutrunner(NUT_FILE)
    if CAR_REPO:
        b2.convert_car(CAR_REPO, CAR_FILE)
    if os.path.exists(SCENE):
        os.remove(SCENE)
    stage = Usd.Stage.CreateNew(SCENE)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr(P["sim"]["gravity"])
    sc.GetPrim().AddAppliedSchema("PhysxSceneAPI")
    sc.GetPrim().CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(P["sim"]["time_steps_per_second"])
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000)
    dl = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    dl.CreateIntensityAttr(2500)
    dl.AddRotateXYZOp().Set(Gf.Vec3f(-40, 25, 0))
    pm = {k: phys_material(stage, k) for k in P["materials"]}
    floor_m = b2.material(stage, "floor", (0.42, 0.43, 0.45), 0.0, 0.8)
    collide(b2.box(stage, "/World/Ground", (16, 16, 0.2), (2.0, -1.0, -0.1), floor_m).GetPrim(), pm["floor"])

    # ---------- 로봇 2대 ----------
    GA, bodies_a = add_robot(stage, "RobotA", GRIP_FILE, "WheelGripper")
    GB, bodies_b = add_robot(stage, "RobotB", NUT_FILE, "Nutrunner", P["robot_b_base"]["xy"], P["robot_b_base"]["yaw_deg"])
    track_mount.add_track(stage, P)                                      # 로봇 2대를 한 레일 위에 (직선 관절)
    camera_mount.add_cell_cameras(stage)                                 # 공구마다 D455 1대 (외형만, 충돌 없음)

    # ---------- 너트 보관 트레이: 로봇 B가 차를 보는 오른쪽 ----------
    tray = UsdGeom.Xform.Define(stage, "/World/NutTray")
    tray_center = np.array([0.85, -1.65, 0.30])          # 0.75 -> 0.85: 리니어 트랙(레일 베드 x <= 0.45) 과 겹치지 않게
    tray_yaw = float(P["robot_b_base"]["yaw_deg"])
    tray.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in tray_center]))
    tray.AddRotateZOp().Set(tray_yaw)
    tray.GetPrim().GetReferences().AddReference("./nut_tray.usda")
    tilt = math.radians(nt.TILT_DEG)
    plate_center_local = np.array([
        -0.012 - nt.PLATE[1] / 2 * math.cos(tilt),
        0.0,
        -0.035 - nt.PLATE[1] / 2 * math.sin(tilt),
    ])
    yaw = math.radians(tray_yaw)
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                   [math.sin(yaw), math.cos(yaw), 0.0],
                   [0.0, 0.0, 1.0]])
    tray_surface_center = tray_center + Rz @ plate_center_local
    tray_frame = nt.frame(tray_surface_center, tray_yaw)
    tray_slots = [nt.peg_base(tray_frame, 2, col) + tray_frame[3] * nt.NUT_SEAT
                  for col in range(1, 6)]
    tray_nut_positions = [nt.peg_base(tray_frame, 2, col) for col in range(1, 6)]
    tray_tool_rotation = nt.tool_rotation(tray_yaw, nt.TILT_DEG)

    # ---------- 차체 (동적 강체, 차 좌표계 = Z up, 앞 = -Y. world 로는 Z축 180도 회전) ----------
    t = (WHEEL_C[0] + CAR_WHEELS["FL"][0], WHEEL_C[1] + CAR_WHEELS["FL"][1], WHEEL_C[2] - CAR_WHEEL_Z + DROOP)
    car = UsdGeom.Xform.Define(stage, "/World/Car")
    car.AddTranslateOp().Set(Gf.Vec3d(*t))
    car.AddRotateZOp().Set(180.0)
    cp = car.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(cp)
    sprung = C["curb_mass"] - 2 * (C["unsprung_front"] + C["unsprung_rear"]) - 4 * W["mass"]
    ma = UsdPhysics.MassAPI.Apply(cp)
    ma.CreateMassAttr(sprung)
    ma.CreateCenterOfMassAttr(Gf.Vec3f(*C["com_car_frame"]))
    ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*C["inertia_diag"]))
    ma.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
    cp.AddAppliedSchema("PhysxRigidBodyAPI")
    cp.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    cp.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(1.0)
    contact_report(cp)
    n_plates = 0
    if os.path.exists(CAR_FILE):
        body = UsdGeom.Xform.Define(stage, "/World/Car/Body")
        body.GetPrim().GetReferences().AddReference("./car/" + os.path.basename(CAR_FILE))
        cstage = Usd.Stage.Open(CAR_FILE)
        for name in ("Hybrid", "Hybrid_Interior", "Windows"):
            # 동적 강체에는 삼각형 메시 충돌을 쓸 수 없으므로 원본 충돌은 끄고, 외판 삼각형마다 얇은 볼록판을 만든다
            stage.OverridePrim(f"/World/Car/Body/{name}").CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)
            if name == "Hybrid_Interior":
                continue
            m = UsdGeom.Mesh(cstage.GetPrimAtPath("/PriusBody/" + name))
            pts = np.array(m.GetPointsAttr().Get(), dtype=float)
            cnt, idx = m.GetFaceVertexCountsAttr().Get(), m.GetFaceVertexIndicesAttr().Get()
            k = 0
            for n in cnt:
                f = [idx[k + i] for i in range(n)]
                k += n
                for i in range(1, n - 1):
                    A, B, Cv = pts[f[0]], pts[f[i]], pts[f[i + 1]]
                    nrm = np.cross(B - A, Cv - A)
                    area = np.linalg.norm(nrm) / 2
                    if area < 2e-5:
                        continue
                    nrm = nrm / (2 * area) * 0.003                      # 두께 6 mm
                    hull(stage, f"/World/Car/colliders/{name}_{n_plates}", [A + nrm, B + nrm, Cv + nrm, A - nrm, B - nrm, Cv - nrm], pm["body_paint"])
                    n_plates += 1
    # 리프트 패드에 얹히는 잭 포인트 (차체 하면)
    cc_y = 0.0
    jack_z = 0.115
    padm = b2.material(stage, "lift_pad", (0.05, 0.05, 0.05), 0.0, 0.9)
    for i, (jx, jy) in enumerate(((-0.62, -0.75), (-0.62, 0.75), (0.62, -0.75), (0.62, 0.75))):
        jb = b2.box(stage, f"/World/Car/jack_point_{i}", (0.20, 0.16, 0.06), (jx, cc_y + jy, jack_z + 0.03), padm)
        collide(jb.GetPrim(), pm["lift_pad"])

    # ---------- 코너 4개 (스프링 하 질량) + 서스펜션 ----------
    q_near = Gf.Rotation(Gf.Vec3d(0, 1, 0), -90)       # 로컬 +Z -> world -X
    q_far = Gf.Rotation(Gf.Vec3d(0, 1, 0), 90)
    R_car = Gf.Rotation(Gf.Vec3d(0, 0, 1), 180)
    corner_paths = {}
    for name, (cx, cy) in CAR_WHEELS.items():
        pos = b2.car_to_world(cx, cy, CAR_WHEEL_Z - DROOP, t)
        near = cx > 0
        rot = q_near if near else q_far
        K = f"/World/Corner_{name}"
        corner_paths[name] = K
        k = UsdGeom.Xform.Define(stage, K)
        k.AddTranslateOp().Set(pos)
        k.AddOrientOp().Set(Gf.Quatf(rot.GetQuat()))
        kp = k.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(kp)
        front = cy < 0
        m_uns = C["unsprung_front"] if front else C["unsprung_rear"]
        target = name == "FL"
        UsdPhysics.MassAPI.Apply(kp).CreateMassAttr(m_uns + (0.0 if target else W["mass"]))
        build_corner(stage, K, 1.0 if near else -1.0, pm["steel"], with_colliders=target)
        if not target:                                    # 나머지 3개: 휠이 체결된 상태로 같은 강체에 포함
            Wk = K + "/wheel"
            UsdGeom.Xform.Define(stage, Wk)
            b2.build_wheel(stage, Wk, with_nuts=True, collision=False)
            col = b2.lathe(stage, Wk + "/collision", [(0.0, -HW), (TIRE_R - 0.006, -HW), (TIRE_R, -HW + 0.006), (TIRE_R, HW - 0.006), (TIRE_R - 0.006, HW), (0.0, HW)], None, n=32)
            col.CreatePurposeAttr("guide")
            collide(col.GetPrim(), pm["tire_rubber"], hull=True)
        # 서스펜션: 차체 기준 상하 직선 관절 + 스프링/댐퍼. 0 = 완전히 늘어난 위치(리프트 상태), + = 압축
        sj = UsdPhysics.PrismaticJoint.Define(stage, f"/World/Car/suspension_{name}")
        sj.CreateBody0Rel().SetTargets([cp.GetPath()])
        sj.CreateBody1Rel().SetTargets([kp.GetPath()])
        sj.CreateAxisAttr("Z")
        sj.CreateLowerLimitAttr(0.0)
        sj.CreateUpperLimitAttr(C["bump_travel"])
        sj.CreateLocalPos0Attr(Gf.Vec3f(cx, cy, CAR_WHEEL_Z - DROOP))
        sj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        sj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        sj.CreateLocalRot1Attr(Gf.Quatf((R_car * rot.GetInverse()).GetQuat()))
        d = UsdPhysics.DriveAPI.Apply(sj.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(C["wheel_rate"])
        d.CreateDampingAttr(C["wheel_damping"])
        d.CreateTargetPositionAttr(-C["suspension_preload_travel"])     # 늘어남 스토퍼 쪽으로 예압
        contact_report(kp)
        kp.AddAppliedSchema("PhysxRigidBodyAPI")
        kp.CreateAttribute("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)

    # ---------- 탈거 대상 휠 (FL) ----------
    Wp = "/World/Wheel_FL"
    wheel = UsdGeom.Xform.Define(stage, Wp)
    wheel.AddTranslateOp().Set(WHEEL_C)
    wheel.AddOrientOp().Set(Gf.Quatf(q_near.GetQuat()))
    wp = wheel.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(wp)
    ma = UsdPhysics.MassAPI.Apply(wp)
    ma.CreateMassAttr(W["mass"])
    ma.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*W["inertia_diag"]))
    ma.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
    wp.AddAppliedSchema("PhysxRigidBodyAPI")
    wp.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    wp.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(1.0)
    # 잠들지 않게: 너트 고정이 풀린 직후 바퀴가 '정지 상태'로 떠 있지 않고 바로 파일럿/스터드에 얹히도록
    wp.CreateAttribute("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    b2.build_wheel(stage, Wp, with_nuts=False, collision=False)
    wheel_colliders(stage, Wp, pm["tire_rubber"], pm["steel"])
    contact_report(wp)
    KF = corner_paths["FL"]
    UsdGeom.Xform.Define(stage, KF + "/nuts")
    nuts(stage, KF + "/nuts")
    loose_nuts(stage, GB + "/spindle", -tcp_nr)
    # 너트 5개의 체결 = 휠-허브 고정 관절. 너트러너가 5개를 모두 돌린 뒤 스크립트가 해제
    nj = UsdPhysics.FixedJoint.Define(stage, "/World/nut_clamp_joint")
    nj.CreateBody0Rel().SetTargets([Sdf.Path(KF)])
    nj.CreateBody1Rel().SetTargets([wp.GetPath()])
    for a in (nj.CreateLocalPos0Attr, nj.CreateLocalPos1Attr):
        a(Gf.Vec3f(0, 0, 0))
    for a in (nj.CreateLocalRot0Attr, nj.CreateLocalRot1Attr):
        a(Gf.Quatf(1, 0, 0, 0))
    # 예비: 접촉 방식이 불안정할 때 쓰는 스터드 구속 (차축 방향 직선 관절). 기본은 꺼짐
    sj = UsdPhysics.PrismaticJoint.Define(stage, "/World/stud_joint")
    sj.CreateJointEnabledAttr(False)
    sj.CreateBody0Rel().SetTargets([Sdf.Path(KF)])
    sj.CreateBody1Rel().SetTargets([wp.GetPath()])
    sj.CreateAxisAttr("Z")
    sj.CreateLowerLimitAttr(0.0)
    sj.CreateUpperLimitAttr(0.30)
    for a in (sj.CreateLocalPos0Attr, sj.CreateLocalPos1Attr):
        a(Gf.Vec3f(0, 0, 0))
    for a in (sj.CreateLocalRot0Attr, sj.CreateLocalRot1Attr):
        a(Gf.Quatf(1, 0, 0, 0))
    d = UsdPhysics.DriveAPI.Apply(sj.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(3000.0)
    # 예비: 접촉 파지가 실패할 때 쓰는 파지 고정 관절. 기본은 꺼짐
    gj = UsdPhysics.FixedJoint.Define(stage, "/World/grasp_joint")
    gj.CreateJointEnabledAttr(False)
    gj.CreateExcludeFromArticulationAttr(True)
    gj.CreateBody0Rel().SetTargets([Sdf.Path(GA + "/base")])
    gj.CreateBody1Rel().SetTargets([wp.GetPath()])
    for a in (gj.CreateLocalPos0Attr, gj.CreateLocalPos1Attr):
        a(Gf.Vec3f(0, 0, 0))
    for a in (gj.CreateLocalRot0Attr, gj.CreateLocalRot1Attr):
        a(Gf.Quatf(1, 0, 0, 0))

    # ---------- 시저 리프트 (정적. 패드와 런웨이에 충돌) ----------
    lift_blue = b2.material(stage, "lift_blue", (0.08, 0.22, 0.55), 0.5, 0.45)
    lift_yel = b2.material(stage, "lift_yellow", (0.9, 0.68, 0.05), 0.4, 0.5)
    chrome = b2.material(stage, "chrome", (0.8, 0.8, 0.82), 1.0, 0.15)
    cc = b2.car_to_world(0.0, cc_y, 0.0, t)
    top = t[2] + jack_z                                  # 패드 윗면 = 잭 포인트 아랫면
    for i, dx in enumerate((-0.62, 0.62)):
        x = cc[0] + dx
        collide(b2.box(stage, f"/World/Lift/base_{i}", (0.42, 2.3, 0.05), (x, cc[1], 0.025), lift_blue).GetPrim(), pm["steel"])
        collide(b2.box(stage, f"/World/Lift/runway_{i}", (0.42, 2.1, 0.07), (x, cc[1], top - 0.03 - 0.035), lift_blue).GetPrim(), pm["steel"])
        for j, dy in enumerate((-0.75, 0.75)):
            collide(b2.box(stage, f"/World/Lift/pad_{i}_{j}", (0.16, 0.12, 0.03), (x, cc[1] + dy, top - 0.015), padm).GetPrim(), pm["lift_pad"])
        h = top - 0.10 - 0.05
        Ls = math.hypot(1.7, h)
        ang = math.degrees(math.atan2(h, 1.7))
        for j, s in enumerate((1, -1)):
            b2.box(stage, f"/World/Lift/leg_{i}_{j}", (0.05, Ls, 0.09), (x + 0.13 * s, cc[1], 0.05 + h / 2), lift_yel, rot_x=s * ang)
        b2.cyl(stage, f"/World/Lift/pivot_{i}", 0.02, 0.36, (x, cc[1], 0.05 + h / 2), chrome, axis="X")

    # ---------- 스크립트가 읽는 값 ----------
    wprim = world.GetPrim()
    nut_local = [[float(v) for v in (*lug_xy(i), PAD_Z1 + 0.005)] for i in range(W["lugs"])]
    for k, v in {
        "cell:wheel_path": Wp, "cell:corner_path": KF, "cell:car_path": "/World/Car",
        "cell:robot_a": "/World/RobotA", "cell:robot_b": "/World/RobotB", "cell:gripper_path": GA, "cell:nutrunner_path": GB,
        "cell:wheel_center": Gf.Vec3d(WHEEL_C), "cell:tire_width": W["tire_width"], "cell:jaw_close": b2.STROKE,
        "cell:tip_below_tcp": b2.Z_TCP - b2.Z_TIP, "cell:stud_exposed": W["stud_exposed"],
        "cell:nut_points_wheel_local": json.dumps(nut_local), "cell:nut_order": json.dumps(P["demo"]["nut_order"]),
        "cell:nut_tray_center": json.dumps(tray_center.tolist()),
        "cell:nut_tray_slots": json.dumps([p.tolist() for p in tray_slots]),
        "cell:nut_tray_nut_positions": json.dumps([p.tolist() for p in tray_nut_positions]),
        "cell:nut_tray_tool_rotation": json.dumps(tray_tool_rotation.tolist()),
        "cell:spin_rpm": NR["speed_rpm"], "cell:spin_seconds": P["demo"]["spin_seconds"], "cell:wheel_mount_mode": P["demo"]["wheel_mount_mode"],
        "cell:gripper_mass": P["gripper"]["base_mass"] + 4 * P["gripper"]["jaw_mass"], "cell:wheel_mass": W["mass"],
        "cell:nutrunner_mass": NR["mass"] + NR["bracket_mass"], "cell:params_json": json.dumps(P, ensure_ascii=False),
        "cell:protective_stop": json.dumps(P["demo"]["protective_stop"]),
    }.items():
        wprim.SetCustomDataByKey(k, v)
    stage.GetRootLayer().Save()
    print("saved", SCENE)
    print("car: sprung %.0f kg + unsprung %.0f kg + wheels %.0f kg = %.0f kg, body collision plates %d"
          % (sprung, 2 * (C["unsprung_front"] + C["unsprung_rear"]), 4 * W["mass"], C["curb_mass"], n_plates))
    print("nutrunner TCP %.0f mm from flange, mass %.1f kg; payload B = %.1f kg" % (tcp_nr * 1000, NR["mass"], NR["mass"] + NR["bracket_mass"]))


if __name__ == "__main__":
    main()
