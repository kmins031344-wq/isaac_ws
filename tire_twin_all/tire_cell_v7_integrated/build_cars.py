"""
차량 에셋 생성 (준중형 / 중형 / 준대형 / 대형)

  cars/<이름>.usda        : 차량 에셋 (defaultPrim /Vehicle). 차 좌표계 = Z up, 앞 = -Y, 왼쪽 = +X, z 0 = 공차 상태의 지면
  cars/<이름>_test.usda   : 에셋 확인용 씬 (바닥 + 받침대 4개 위에 차를 올린 상태, 바퀴는 떠 있음)
  cars/cars_lineup.usda   : 4대를 나란히 놓은 확인용 씬

제원은 car_specs.json (출처 표기 포함) 에서 읽는다. 차체 외형은 실차 CAD 가 아니라
범용 승용차 메시(osrf/car_demo, Apache-2.0)를 각 차의 전장/전폭/전고/축거/타이어 지름에 맞춰 늘린 것이다.

강체 구성 (차 1대 = 13개)
  Body                 스프링 상 질량 (공차중량 - 스프링 하 - 휠)
  Knuckle_{FL,FR,RL,RR} 너클 + 캘리퍼 + 스트럿 (회전하지 않음)   -- 서스펜션 직선 관절(스프링/댐퍼)로 Body 에 연결
  Hub_{..}             허브 + 디스크 + 스터드 (회전)            -- 축 회전 관절로 Knuckle 에 연결 (기본: 브레이크 잠금)
  Wheel_{..}           휠 + 타이어                             -- 너트 체결 = 고정 관절(nut_clamp)로 Hub 에 연결

실행: python build_cars.py <출력폴더> [car_demo 저장소]
"""
import json
import math
import os
import shutil
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else HERE
_argv = sys.argv
sys.argv = [sys.argv[0], OUT_DIR]
sys.path.insert(0, HERE)
import build_scene_v2 as b2  # noqa: E402
import car_body  # noqa: E402

sys.argv = _argv
P = json.load(open(os.path.join(HERE, "params.json"), encoding="utf-8"))
SPECS = json.load(open(os.path.join(HERE, "car_specs.json"), encoding="utf-8"))
COMMON = SPECS["_common"]
SRC_BODY = os.path.join(OUT_DIR, "car", "prius_body.usda")
CARS_DIR = os.path.join(OUT_DIR, "cars")
V = "/Vehicle"
LOOKS = V + "/Looks"
TIGHT = 0.0004
RHO_IRON, G = 7200.0, 9.81
DROOP = P["car"]["suspension_droop"]
# 원본 메시의 기준 치수 (차 좌표, m)
SRC = {"y_front": -1.41, "y_rear": 1.45, "arch": 0.40, "z_axle": 0.30, "tire_r": 0.317}


def mat(stage, name, color, metallic=0.0, rough=0.5, opacity=1.0):
    return b2.material(stage, name, color, metallic, rough, opacity, root=LOOKS)


def phys_material(stage, name):
    m = P["materials"][name]
    pm = UsdShade.Material.Define(stage, f"{V}/PhysicsMaterials/{name}")
    api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
    api.CreateStaticFrictionAttr(m["static"])
    api.CreateDynamicFrictionAttr(m["dynamic"])
    api.CreateRestitutionAttr(m["restitution"])
    return pm


def collide(prim, pm, hull=False, tight=False):
    UsdPhysics.CollisionAPI.Apply(prim)
    if hull:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
    if tight:
        prim.AddAppliedSchema("PhysxCollisionAPI")
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(TIGHT)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)


def hull(stage, path, pts, pm, tight=False):
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
    c = math.cos(math.radians(a_half))
    pts = []
    for r in (r_in / c, r_out / c):
        for s in (-1, 1):
            a = math.radians(a_c + s * a_half)
            for z in (z0, z1):
                pts.append((center[0] + r * math.cos(a), center[1] + r * math.sin(a), z))
    return hull(stage, path, pts, pm, tight)


def col_cyl(stage, path, r, z0, z1, pm, center=(0.0, 0.0), tight=False, n=None):
    n = n or (12 if r < 0.01 else 32)
    pts = [(center[0] + r * math.cos(2 * math.pi * k / n), center[1] + r * math.sin(2 * math.pi * k / n), z) for k in range(n) for z in (z0, z1)]
    return hull(stage, path, pts, pm, tight)


def rigid(stage, path, pos, quat, mass, inertia=None):
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddOrientOp().Set(Gf.Quatf(quat))
    p = x.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    ma = UsdPhysics.MassAPI.Apply(p)
    ma.CreateMassAttr(float(mass))
    if inertia is not None:
        ma.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
        ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(v) for v in inertia]))
        ma.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
    p.AddAppliedSchema("PhysxRigidBodyAPI")
    p.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    p.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(1.0)
    p.CreateAttribute("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    p.AddAppliedSchema("PhysxContactReportAPI")
    p.CreateAttribute("physxContactReport:threshold", Sdf.ValueTypeNames.Float).Set(0.0)
    return p


# ------------------------------------------------------------------ 치수 계산
def axle_geom(ax):
    """축(앞/뒤)별 휠·브레이크 치수 (휠 로컬 좌표: 원점 = 휠 중심, +Z = 차 바깥쪽)"""
    w, asp, inch = ax["tire"]
    g = {"HW": w / 2000.0, "RIM_R": inch * 0.0254 / 2}
    g["R"] = g["RIM_R"] + w / 1000.0 * asp / 100.0
    g["PAD0"] = ax["offset_mm"] / 1000.0                     # 휠 장착면 = 오프셋(ET)
    g["PAD1"] = g["PAD0"] + 0.025
    g["BORE_R"] = COMMON["hub_bore_mm"] / 2000.0
    g["PCD_R"] = COMMON["pcd_mm"] / 2000.0
    g["DISC_R"], g["DISC_T"] = ax["disc_d_mm"] / 2000.0, ax["disc_t_mm"] / 1000.0
    g["DISC_Z1"] = g["PAD0"] - 0.0475                        # 디스크 바깥면 (허브면에서 모자 깊이 47.5 mm 안쪽)
    g["DISC_Z0"] = g["DISC_Z1"] - g["DISC_T"]
    g["STUD"] = COMMON["stud_exposed_mm"] / 1000.0
    g["tire_label"] = f"{w}/{asp}R{inch}"
    # 질량: 디스크는 치수에서 계산 (주철, 벤틸레이티드는 속이 비어 0.72배), 나머지는 가정
    disc = RHO_IRON * math.pi * (g["DISC_R"] ** 2 - 0.068 ** 2) * g["DISC_T"] * (0.72 if ax["vented"] else 1.0)
    hat = RHO_IRON * (math.pi * 0.078 ** 2 * 0.008 + math.pi * (0.078 ** 2 - 0.068 ** 2) * 0.04)
    g["disc_mass"] = disc + hat
    g["wheel_mass"] = ax["wheel_mass"]
    m_t = 0.46 * g["wheel_mass"]                             # 타이어 : 휠 = 46 : 54 (가정)
    izz = m_t * (0.88 * g["R"]) ** 2 + (g["wheel_mass"] - m_t) * (0.75 * g["RIM_R"]) ** 2
    ixx = izz / 2 + g["wheel_mass"] * (2 * g["HW"]) ** 2 / 12
    g["wheel_inertia"] = (ixx, ixx, izz)
    return g


def lug_xy(g, i):
    a = math.radians(b2.lug_angle(i))
    return g["PCD_R"] * math.cos(a), g["PCD_R"] * math.sin(a)


# ------------------------------------------------------------------ 휠 (외형 + 충돌)
def build_wheel(stage, root, g, pm_rubber, pm_steel, spokes=(10, 0.024)):
    rubber = mat(stage, "tire_rubber", (0.03, 0.03, 0.03), 0.0, 0.85)
    alloy = mat(stage, "alloy", (0.78, 0.79, 0.81), 1.0, 0.28)
    dark = mat(stage, "alloy_dark", (0.22, 0.22, 0.24), 1.0, 0.4)
    R, HW, RR, P0, P1, BR = g["R"], g["HW"], g["RIM_R"], g["PAD0"], g["PAD1"], g["BORE_R"]
    side = [(RR, -(HW - 0.0165)), (RR + 0.02, -(HW - 0.0045)), ((RR + R) / 2, -HW), (R - 0.02, -(HW - 0.0035)), (R - 0.007, -(HW - 0.0125)), (R, -(HW - 0.0245))]
    tread = []
    for gz in (-0.4 * HW, 0.0, 0.4 * HW):
        tread += [(R, gz - 0.005), (R - 0.007, gz - 0.004), (R - 0.007, gz + 0.004), (R, gz + 0.005)]
    b2.lathe(stage, root + "/tire", side + tread + [(r, -z) for r, z in reversed(side)], rubber, n=72)
    a, b, c = HW - 0.0125, HW - 0.0195, HW - 0.0225
    b2.lathe(stage, root + "/rim_barrel", [(RR - 0.018, -a), (RR + 0.012, -a), (RR + 0.012, -b), (RR + 0.0006, -c), (RR + 0.0006, c), (RR + 0.012, b), (RR + 0.012, a),
                                           (RR - 0.018, a), (RR - 0.013, HW - 0.0275), (RR - 0.013, -(HW - 0.0275))], alloy, n=72)
    b2.lathe(stage, root + "/pad_inner", [(BR + 0.0002, P0), (0.049, P0), (0.049, P1), (BR + 0.0002, P1)], alloy)
    b2.lathe(stage, root + "/pad_outer", [(0.0655, P0), (0.092, P0), (0.092, P1 + 0.004), (0.0655, P1)], alloy)
    for i in range(COMMON["lug_count"]):
        ang = b2.lug_angle(i)
        b2.sector(stage, f"{root}/pad_web_{i}", 0.0485, 0.066, ang + 9.0, ang + 63.0, P0, P1, alloy)
        b2.lathe(stage, f"{root}/lug_seat_{i}", [(0.0072, P0), (0.0138, P0), (0.0138, P1), (0.0095, P1), (0.0072, P1 - 0.006)], dark, n=24, center=lug_xy(g, i))
    r_a, r_b, z_a, z_b = 0.088, RR - 0.012, P1 - 0.008, HW - 0.03
    L, tilt = math.hypot(r_b - r_a, z_b - z_a), -math.degrees(math.atan2(z_b - z_a, r_b - r_a))
    Cc = root + "/colliders"
    ns, sw = spokes
    for k in range(ns):
        ang = b2.lug_angle(0) + 360.0 / ns * (k + 0.5) + ((6.0 if k % 2 else -6.0) if ns == 10 else 0.0)
        rm = (r_a + r_b) / 2
        sp = b2.box(stage, f"{root}/spoke_{k}", (L, sw, 0.018), (rm * math.cos(math.radians(ang)), rm * math.sin(math.radians(ang)), (z_a + z_b) / 2), alloy, rot_z=ang, rot_y=tilt)
        collide(sp.GetPrim(), pm_steel)
    for k in range(24):                                   # 타이어+림: 속이 빈 링
        wedge(stage, f"{Cc}/tire_{k}", RR - 0.017, R, 15.0 * k, 7.5, -HW, HW, pm_rubber)
    for k in range(16):                                   # 허브 보어 / 장착 패드
        wedge(stage, f"{Cc}/bore_{k}", BR + P["wheel"]["bore_clearance_sim"], 0.0485, 22.5 * k, 11.25, P0, P1, pm_steel, tight=True)
        wedge(stage, f"{Cc}/pad_outer_{k}", 0.066, 0.092, 22.5 * k, 11.25, P0, P1, pm_steel, tight=True)
    for i in range(COMMON["lug_count"]):                  # 러그 구멍 사이 살 + 스터드 구멍 링
        wedge(stage, f"{Cc}/web_{i}", 0.0485, 0.066, b2.lug_angle(i) + 36.0, 22.0, P0, P1, pm_steel, tight=True)
        for k in range(8):
            wedge(stage, f"{Cc}/hole_{i}_{k}", P["wheel"]["stud_hole_radius"], 0.0138, 45.0 * k, 22.5, P0, P1, pm_steel, center=lug_xy(g, i), tight=True)


# ------------------------------------------------------------------ 허브 (회전부: 허브 + 디스크 + 스터드 + 너트)
def build_hub(stage, root, g, pm_steel, vented):
    steel = mat(stage, "steel", (0.45, 0.45, 0.47), 1.0, 0.45)
    cast = mat(stage, "cast_iron", (0.2, 0.2, 0.21), 0.6, 0.7)
    studm = mat(stage, "stud", (0.8, 0.72, 0.35), 1.0, 0.3)
    nutm = mat(stage, "lug_nut", (0.6, 0.6, 0.62), 1.0, 0.25)
    P0, P1, BR, DR, Z0, Z1 = g["PAD0"], g["PAD1"], g["BORE_R"], g["DISC_R"], g["DISC_Z0"], g["DISC_Z1"]
    b2.cyl(stage, root + "/disc_hat_face", 0.078, 0.008, (0, 0, P0 - 0.004), steel)
    b2.lathe(stage, root + "/disc_hat_wall", [(0.068, Z1), (0.078, Z1), (0.078, P0 - 0.008), (0.068, P0 - 0.008)], steel)
    if vented:
        t = g["DISC_T"] * 0.32
        b2.lathe(stage, root + "/disc_plate_out", [(0.068, Z1 - t), (DR, Z1 - t), (DR, Z1), (0.068, Z1)], steel, n=72)
        b2.lathe(stage, root + "/disc_plate_in", [(0.085, Z0), (DR, Z0), (DR, Z0 + t), (0.085, Z0 + t)], steel, n=72)
        b2.lathe(stage, root + "/disc_vanes", [(0.090, Z0 + t), (DR - 0.004, Z0 + t), (DR - 0.004, Z1 - t), (0.090, Z1 - t)], cast, n=36)
    else:
        b2.lathe(stage, root + "/disc_plate", [(0.068, Z0), (DR, Z0), (DR, Z1), (0.068, Z1)], steel, n=72)
    b2.lathe(stage, root + "/hub_pilot", [(0.022, P0), (BR, P0), (BR, P0 + P["wheel"]["pilot_length"]), (0.022, P0 + P["wheel"]["pilot_length"])], steel)
    b2.lathe(stage, root + "/axle_nut", [(0.0, P0), (0.018, P0), (0.018, P0 + 0.010), (0.0, P0 + 0.010)], cast, n=6)
    b2.cyl(stage, root + "/hub_flange", 0.070, 0.012, (0, 0, Z1 - 0.006), cast)
    b2.cyl(stage, root + "/spindle", 0.022, 0.07, (0, 0, Z0 - 0.02), steel)
    Cc = root + "/colliders"
    col_cyl(stage, Cc + "/hat", 0.078, Z1, P0, pm_steel, tight=True)                               # 휠 장착면
    col_cyl(stage, Cc + "/disc", DR, Z0, Z1, pm_steel)
    col_cyl(stage, Cc + "/pilot", BR, P0, P0 + P["wheel"]["pilot_length"], pm_steel, tight=True)
    tq = COMMON["nut_torque_Nm"]
    hexr = COMMON["nut_hex_mm"] / 2000.0 / math.cos(math.radians(30))
    for i in range(COMMON["lug_count"]):
        x, y = lug_xy(g, i)
        b2.cyl(stage, f"{root}/stud_{i}", P["wheel"]["stud_radius"], g["STUD"] + 0.004, (x, y, P0 - 0.004 + (g["STUD"] + 0.004) / 2), studm)
        col_cyl(stage, f"{Cc}/stud_{i}", P["wheel"]["stud_radius"], P0, P0 + g["STUD"], pm_steel, center=(x, y), tight=True)
        n = b2.lathe(stage, f"{root}/nuts/nut_{i}", [(0.0, P1 - 0.004), (hexr, P1 - 0.004), (hexr, P1 + 0.014), (0.009, P1 + 0.020), (0.0, P1 + 0.020)], nutm, n=6, center=(x, y))
        np_ = n.GetPrim()
        for k, v in (("thread", COMMON["thread"]), ("hex_mm", COMMON["nut_hex_mm"]), ("torque_min_Nm", tq[0]), ("torque_max_Nm", tq[1]),
                     ("tighten_order", [1, 3, 5, 2, 4].index(i + 1) + 1), ("tightened", True)):
            np_.SetCustomDataByKey("nut:" + k, v)


# ------------------------------------------------------------------ 너클 (비회전부: 너클 + 캘리퍼 + 스트럿 + 링크)
def build_knuckle(stage, root, g, up, pm_steel, driven):
    cast = mat(stage, "cast_iron", (0.2, 0.2, 0.21), 0.6, 0.7)
    red = mat(stage, "caliper_red", (0.75, 0.05, 0.05), 0.2, 0.4)
    black = mat(stage, "susp_black", (0.06, 0.06, 0.06), 0.3, 0.6)
    steel = mat(stage, "steel", (0.45, 0.45, 0.47), 1.0, 0.45)
    DR, Z0, Z1, HW = g["DISC_R"], g["DISC_Z0"], g["DISC_Z1"], g["HW"]
    zk = Z0 - 0.045                                           # 너클 중심 (디스크 안쪽)
    zs = min(-HW - 0.05, zk - 0.06)                           # 스트럿: 타이어 안쪽면보다 안쪽
    a0 = 20.0 if up > 0 else 200.0
    cal = [b2.sector(stage, root + "/caliper_body", DR - 0.05, DR + 0.026, a0, a0 + 56.0, Z0 - 0.021, Z1 + 0.015, red, n=8),
           b2.sector(stage, root + "/caliper_bridge", DR, DR + 0.030, a0 + 6.0, a0 + 50.0, Z0 - 0.019, Z1 + 0.013, red, n=6)]
    kn = b2.box(stage, root + "/knuckle", (0.20, 0.09, 0.05), (0.0, 0.0, zk), cast)
    b2.cyl(stage, root + "/wheel_bearing", 0.055, 0.05, (0, 0, zk + 0.012), cast)
    strut = b2.cyl(stage, root + "/strut", 0.028, 0.40, (up * 0.28, 0.0, zs), black, axis="X")
    ss = b2.lathe(stage, root + "/spring_seat", [(0.03, 0.0), (0.075, 0.0), (0.075, 0.012), (0.03, 0.012)], black, n=24)
    sx = UsdGeom.Xformable(ss.GetPrim())
    sx.AddTranslateOp().Set(Gf.Vec3d(up * 0.26, 0.0, zs))
    sx.AddRotateYOp().Set(90.0)
    b2.box(stage, root + "/lower_arm", (0.045, 0.07, 0.34), (-up * 0.105, 0.0, zk - 0.18), black, rot_y=up * 8.0)
    b2.cyl(stage, root + "/tie_rod", 0.009, 0.34, (up * 0.02, 0.11, zk - 0.18), steel)
    if driven:
        b2.cyl(stage, root + "/drive_shaft", 0.016, 0.42, (0, 0, zk - 0.23), steel)
        b2.lathe(stage, root + "/cv_boot", [(0.016, zk - 0.14), (0.045, zk - 0.04), (0.045, zk - 0.025), (0.016, zk - 0.025)], black, n=24)
    for c in cal:
        collide(c.GetPrim(), pm_steel, hull=True)
    collide(kn.GetPrim(), pm_steel)
    collide(strut.GetPrim(), pm_steel)


# ------------------------------------------------------------------ 차체 메시를 제원에 맞춰 늘리기
def body_mapper(S, gf, gr, src_pts):
    half_src = float(np.abs(src_pts[src_pts[:, 2] < 0.85][:, 0]).max())           # 미러 제외 반폭
    y_min, y_max, z_min, z_max = src_pts[:, 1].min(), src_pts[:, 1].max(), src_pts[:, 2].min(), src_pts[:, 2].max()
    wb, over = S["wheelbase"], S["length"] - S["wheelbase"]
    fo, ro = over * S["front_overhang_fraction"], over * (1 - S["front_overhang_fraction"])
    a = SRC["arch"]
    kf, kr = 1.08 * gf["R"] / SRC["tire_r"], 1.08 * gr["R"] / SRC["tire_r"]     # 휠 아치는 타이어 지름 비율 + 8 % 여유
    ys = [y_min, SRC["y_front"] - a, SRC["y_front"] + a, SRC["y_rear"] - a, SRC["y_rear"] + a, y_max]
    yt = [-wb / 2 - fo, -wb / 2 - a * kf, -wb / 2 + a * kf, wb / 2 - a * kr, wb / 2 + a * kr, wb / 2 + ro]
    R_big = max(gf["R"], gr["R"])
    z_ax = R_big - 0.012                                       # 공차 상태 축 높이 (타이어 눌림 12 mm 가정)
    zs = [z_min, SRC["z_axle"], SRC["z_axle"] + 0.40, z_max]
    zt = [S["ground_clearance"], z_ax, z_ax + 0.40 * R_big / SRC["tire_r"] + 0.045, S["height"]]
    sx = S["width"] / 2 / half_src

    def f(pts):
        q = np.array(pts, dtype=float)
        return np.c_[q[:, 0] * sx, np.interp(q[:, 1], ys, yt), np.interp(q[:, 2], zs, zt)]
    return f


def build_car(key):
    S = SPECS[key]
    gf, gr = axle_geom(S["front"]), axle_geom(S["rear"])
    path = os.path.join(CARS_DIR, key + ".usda")
    if os.path.exists(path):
        os.remove(path)
    b2.MATS.clear()
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, V)
    stage.SetDefaultPrim(root.GetPrim())
    pm = {k: phys_material(stage, k) for k in ("tire_rubber", "steel", "body_paint", "lift_pad")}

    # ---------- 질량 배분 ----------
    corners = {"FL": (1, -1), "FR": (-1, -1), "RL": (1, 1), "RR": (-1, 1)}
    info = {}
    for name, (sx_, sy_) in corners.items():
        front = sy_ < 0
        g = gf if front else gr
        driven = (S["drive"] == "FWD") == front
        hub_m = g["disc_mass"] + 3.5 + (1.5 if driven else 0.0)             # 디스크 + 허브/베어링 (+ 등속조인트 몫)
        kn_m = (5.5 if front else 4.5) + (4.5 if front else 3.0) + 4.0      # 너클 + 캘리퍼 + 링크·스트럿 하단 몫 (가정)
        info[name] = {"g": g, "front": front, "driven": driven, "hub_m": hub_m, "kn_m": kn_m,
                      "x": sx_ * (S["track_front"] if front else S["track_rear"]) / 2, "y": sy_ * S["wheelbase"] / 2}
    unsprung = sum(i["hub_m"] + i["kn_m"] + i["g"]["wheel_mass"] for i in info.values())
    sprung = S["curb_mass"] - unsprung
    Lc, Wc, Hc = S["length"], S["width"], S["height"]
    inertia = [0.88 * sprung * (Lc ** 2 + Hc ** 2) / 12, 0.88 * sprung * (Wc ** 2 + Hc ** 2) / 12, 0.88 * sprung * (Lc ** 2 + Wc ** 2) / 12]
    com = (0.0, -S["wheelbase"] / 2 + (1 - S["front_weight_fraction"]) * S["wheelbase"], 0.38 * Hc)

    # ---------- 차체 ----------
    Bp = V + "/Body"
    bp = rigid(stage, Bp, (0, 0, 0), Gf.Quatf(1, 0, 0, 0), sprung)
    ma = UsdPhysics.MassAPI(bp)
    ma.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    ma.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
    # 차체 외형: 차종별 절차적 모델 (car_body.py). 충돌은 같은 형상을 성기게 만든 뒤 사각형마다 두께 6 mm 볼록판으로
    st = car_body.STYLES[key]
    bd = car_body.Body(S, key, max(gf["R"], gr["R"]), max(gf["HW"], gr["HW"]), DROOP)
    pts, faces, stations = bd.loft(step=0.03)
    P3 = [tuple(float(v) for v in p) for p in pts]
    vis = {"paint": mat(stage, "car_paint", st["paint"], 0.55, 0.28), "glass": mat(stage, "car_glass", (0.02, 0.025, 0.03), 0.0, 0.06),
           "trim": mat(stage, "car_trim", st["trim"], st["trim_metal"], 0.25), "liner": mat(stage, "wheel_house", (0.02, 0.02, 0.02), 0.0, 0.9)}
    for kind in ("paint", "glass"):
        q = faces[kind]
        b2.mesh(stage, f"{Bp}/geom/{kind}", P3, [4] * len(q), [i for f in q for i in f], vis[kind])
    tp, tq_ = bd.trim_strip(stations, car_body.N_SIDE)
    b2.mesh(stage, f"{Bp}/geom/window_trim", tp, [4] * len(tq_), [i for f in tq_ for i in f], vis["trim"])
    lp, lq = bd.liners()
    b2.mesh(stage, f"{Bp}/geom/wheel_house", lp, [4] * len(lq), [i for f in lq for i in f], vis["liner"])
    for di, (mname, dp, dq) in enumerate(bd.decals()):
        if mname == "trim":
            dm = vis["trim"]
        else:
            col, met, rough = car_body.DECAL_COLORS[mname]
            dm = mat(stage, "face_" + mname, col, met, rough)
        b2.mesh(stage, f"{Bp}/geom/face_{di:02d}_{mname}", dp, [4] * len(dq), [i for f in dq for i in f], dm)
    for sgn in (1, -1):                                   # 사이드 미러
        zs_ = bd.shoulder(bd.s_cowl + 0.34)
        b2.box(stage, f"{Bp}/geom/mirror_{'L' if sgn > 0 else 'R'}", (0.15, 0.07, 0.10), (sgn * (bd.w + 0.055), bd.y0 + bd.s_cowl + 0.34, zs_ + 0.07), vis["paint"])
    sp_, sq_, handles = bd.seams()
    b2.mesh(stage, f"{Bp}/geom/door_seams", sp_, [4] * len(sq_), [i for f in sq_ for i in f], vis["liner"])
    for hi, (sgn, hs, hz) in enumerate(handles):
        b2.box(stage, f"{Bp}/geom/door_handle_{hi}", (0.02, 0.17, 0.028), (sgn * (bd.half_width(hs) - 0.012), bd.y0 + hs, hz), vis["trim"])
    cpts, cfaces, _ = bd.loft(step=0.14, n_side=3)
    n_plates = 0
    for kind in ("paint", "glass"):
        for f in cfaces[kind]:
            A, B, Cv, D = [cpts[i] for i in f]
            nrm = np.cross(Cv - A, D - B)
            ln = np.linalg.norm(nrm)
            if ln < 4e-5:
                continue
            nrm = nrm / ln * 0.003
            hull(stage, f"{Bp}/colliders/plate_{n_plates}", [A + nrm, B + nrm, Cv + nrm, D + nrm, A - nrm, B - nrm, Cv - nrm, D - nrm], pm["body_paint"])
            n_plates += 1
    body_pts = {"Hybrid": pts, "Windows": pts[:1]}
    padm = mat(stage, "jack_point", (0.05, 0.05, 0.05), 0.0, 0.9)
    jz = S["ground_clearance"] - 0.01
    jacks = [(sx_ * (Wc / 2 - 0.28), sy_ * (S["wheelbase"] / 2 - 0.42)) for sx_ in (1, -1) for sy_ in (-1, 1)]
    for i, (jx, jy) in enumerate(jacks):
        jb = b2.box(stage, f"{Bp}/jack_point_{i}", (0.20, 0.16, 0.04), (jx, jy, jz), padm)
        collide(jb.GetPrim(), pm["lift_pad"])

    # ---------- 코너 4개 ----------
    UsdGeom.Scope.Define(stage, V + "/Joints")
    z_ax = max(gf["R"], gr["R"]) - 0.012
    tq = COMMON["nut_torque_Nm"]
    d_thread = float(COMMON["thread"][1:].split("x")[0]) / 1000.0
    clamp = [t / (COMMON["nut_factor_K"] * d_thread) for t in tq]           # F = T / (K d), 너트 1개
    for name, i in info.items():
        g = i["g"]
        left = i["x"] > 0
        rot = Gf.Rotation(Gf.Vec3d(0, 1, 0), 90.0 if left else -90.0)      # 로컬 +Z -> 차 바깥쪽
        q = Gf.Quatf(rot.GetQuat())
        up = -1.0 if left else 1.0                                          # 로컬 X 중 위쪽 방향
        pos = (i["x"], i["y"], g["R"] - 0.012 - DROOP + (z_ax - (g["R"] - 0.012)))
        K, Hb, Wh = f"{V}/Knuckle_{name}", f"{V}/Hub_{name}", f"{V}/Wheel_{name}"
        kp = rigid(stage, K, pos, q, i["kn_m"])
        build_knuckle(stage, K, g, up, pm["steel"], i["driven"])
        hp = rigid(stage, Hb, pos, q, i["hub_m"])
        build_hub(stage, Hb, g, pm["steel"], S["front" if i["front"] else "rear"]["vented"])
        wp = rigid(stage, Wh, pos, q, g["wheel_mass"], g["wheel_inertia"])
        build_wheel(stage, Wh, g, pm["tire_rubber"], pm["steel"], car_body.STYLES[key]["spokes"])
        wp.SetCustomDataByKey("wheel:tire", g["tire_label"])
        # 서스펜션: 0 = 완전히 늘어난 위치(리프트 상태), + = 압축
        sj = UsdPhysics.PrismaticJoint.Define(stage, f"{V}/Joints/suspension_{name}")
        sj.CreateBody0Rel().SetTargets([Sdf.Path(Bp)])
        sj.CreateBody1Rel().SetTargets([Sdf.Path(K)])
        sj.CreateAxisAttr("Z")
        sj.CreateLowerLimitAttr(0.0)
        sj.CreateUpperLimitAttr(P["car"]["bump_travel"])
        sj.CreateLocalPos0Attr(Gf.Vec3f(*pos))
        sj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        sj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        sj.CreateLocalRot1Attr(Gf.Quatf(rot.GetInverse().GetQuat()))
        # 휠 레이트: 공차 하중에서 droop 만큼 눌리도록 (정적 처짐 기준, derived)
        load = S["curb_mass"] * (S["front_weight_fraction"] if i["front"] else 1 - S["front_weight_fraction"]) / 2 - (i["hub_m"] + i["kn_m"] + g["wheel_mass"])
        rate = load * G / (DROOP + P["car"]["suspension_preload_travel"])
        d = UsdPhysics.DriveAPI.Apply(sj.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(rate)
        d.CreateDampingAttr(2 * 0.3 * math.sqrt(rate * load))               # 감쇠비 0.3 (가정)
        d.CreateTargetPositionAttr(-P["car"]["suspension_preload_travel"])
        # 축: 너클-허브 회전 관절. 기본은 브레이크 잠금(위치 유지 구동). stiffness 를 0 으로 하면 자유 회전
        aj = UsdPhysics.RevoluteJoint.Define(stage, f"{V}/Joints/axle_{name}")
        aj.CreateBody0Rel().SetTargets([Sdf.Path(K)])
        aj.CreateBody1Rel().SetTargets([Sdf.Path(Hb)])
        aj.CreateAxisAttr("Z")
        for fn in (aj.CreateLocalPos0Attr, aj.CreateLocalPos1Attr):
            fn(Gf.Vec3f(0, 0, 0))
        for fn in (aj.CreateLocalRot0Attr, aj.CreateLocalRot1Attr):
            fn(Gf.Quatf(1, 0, 0, 0))
        ad = UsdPhysics.DriveAPI.Apply(aj.GetPrim(), "angular")
        ad.CreateTypeAttr("force")
        ad.CreateStiffnessAttr(2000.0)                                       # N·m/deg: 브레이크 잠금 (가정)
        ad.CreateDampingAttr(20.0)
        ad.CreateTargetPositionAttr(0.0)
        ad.CreateMaxForceAttr(3000.0)                                        # 브레이크가 버티는 최대 토크 [N·m] (가정)
        aj.GetPrim().SetCustomDataByKey("axle:brake_locked", True)
        # 너트 5개 체결 = 허브-휠 고정 관절
        nj = UsdPhysics.FixedJoint.Define(stage, f"{V}/Joints/nut_clamp_{name}")
        nj.CreateBody0Rel().SetTargets([Sdf.Path(Hb)])
        nj.CreateBody1Rel().SetTargets([Sdf.Path(Wh)])
        for fn in (nj.CreateLocalPos0Attr, nj.CreateLocalPos1Attr):
            fn(Gf.Vec3f(0, 0, 0))
        for fn in (nj.CreateLocalRot0Attr, nj.CreateLocalRot1Attr):
            fn(Gf.Quatf(1, 0, 0, 0))
        for k, v in (("torque_min_Nm", tq[0]), ("torque_max_Nm", tq[1]), ("nuts", COMMON["lug_count"]), ("thread", COMMON["thread"]),
                     ("clamp_force_per_nut_N_min", round(clamp[0])), ("clamp_force_per_nut_N_max", round(clamp[1])),
                     ("tighten_order", "1-3-5-2-4 (별 모양)")):
            nj.GetPrim().SetCustomDataByKey("nut_clamp:" + k, v)
        i.update(pos=pos, rate=rate, load=load)

    summary = {
        "key": key, "label": S["label"], "curb_mass": S["curb_mass"], "sprung_mass": round(sprung, 1), "unsprung_total": round(unsprung, 1),
        "com": [round(v, 3) for v in com], "inertia": [round(v) for v in inertia], "body_collision_plates": n_plates,
        "jack_points": [[round(v, 3) for v in j] for j in jacks], "jack_bottom_z": round(jz - 0.02, 3), "droop": DROOP,
        "corners": {n: {"tire": i["g"]["tire_label"], "tire_radius": round(i["g"]["R"], 4), "wheel_mass": i["g"]["wheel_mass"],
                        "hub_disc_mass": round(i["hub_m"], 2), "disc_mass": round(i["g"]["disc_mass"], 2), "knuckle_mass": i["kn_m"],
                        "disc_d_mm": i["g"]["DISC_R"] * 2000, "center": [round(v, 4) for v in i["pos"]],
                        "wheel_rate_N_per_m": round(i["rate"]), "corner_sprung_load_kg": round(i["load"], 1)} for n, i in info.items()},
        "nut": {"count": 5, "thread": COMMON["thread"], "torque_Nm": tq, "clamp_force_per_nut_N": [round(c) for c in clamp]},
        "sources": S["_source"],
    }
    rp = root.GetPrim()
    rp.SetCustomDataByKey("vehicle:spec", json.dumps(S, ensure_ascii=False))
    rp.SetCustomDataByKey("vehicle:summary", json.dumps(summary, ensure_ascii=False))
    stage.GetRootLayer().Save()
    b2.MATS.clear()
    # 간섭 점검: 차체 외판 점이 타이어 원통 안에 들어오는지 (droop 위치, 완전 압축 위치)
    clear = {}
    ext = np.concatenate([body_pts["Hybrid"], body_pts["Windows"]])
    for n, i in info.items():
        g = i["g"]
        for tag, dz in (("droop", 0.0), ("bump", P["car"]["bump_travel"])):
            c = np.array(i["pos"]) + (0, 0, dz)
            inx = np.abs(ext[:, 0] - c[0]) < g["HW"]
            r = np.hypot(ext[inx, 1] - c[1], ext[inx, 2] - c[2])
            clear[f"{n}_{tag}"] = round(float(r.min() - g["R"]) * 1000) if len(r) else None
    summary["tire_to_body_min_gap_mm"] = clear
    return summary


def test_scene(path, items, on_ground=False):
    """items = [(key, summary, x 위치)] : 바닥 + 받침대 위에 차를 올린 확인용 씬. on_ground=True 면 받침대 없이 바퀴로 바닥에 세운다"""
    if os.path.exists(path):
        os.remove(path)
    b2.MATS.clear()
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    w = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(w.GetPrim())
    sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr(P["sim"]["gravity"])
    sc.GetPrim().AddAppliedSchema("PhysxSceneAPI")
    sc.GetPrim().CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(P["sim"]["time_steps_per_second"])
    gm = b2.material(stage, "floor", (0.4, 0.42, 0.44), 0.0, 0.5)
    sm = b2.material(stage, "stand", (0.85, 0.5, 0.05), 0.3, 0.5)
    cx = sum(x for _, _, x in items) / len(items)
    sx_, sy_ = (40.0, 30.0) if on_ground else (8.0 * len(items) + 6, 12.0)
    fl = b2.box(stage, "/World/GroundPlane", (sx_, sy_, 0.1), (cx, 0, -0.05), gm)
    UsdPhysics.CollisionAPI.Apply(fl.GetPrim())
    if on_ground:                                          # 1 m 격자선 (외형만)
        lm = b2.material(stage, "grid_line", (0.62, 0.64, 0.66), 0.0, 0.6)
        pts, cnt, idx = [], [], []
        def quad(x0, y0, x1, y1):
            n = len(pts)
            pts.extend([(x0, y0, 0.001), (x1, y0, 0.001), (x1, y1, 0.001), (x0, y1, 0.001)])
            cnt.append(4)
            idx.extend([n, n + 1, n + 2, n + 3])
        for k in range(int(sx_) + 1):
            x = cx - sx_ / 2 + k
            quad(x - 0.01, -sy_ / 2, x + 0.01, sy_ / 2)
        for k in range(int(sy_) + 1):
            y = -sy_ / 2 + k
            quad(cx - sx_ / 2, y - 0.01, cx + sx_ / 2, y + 0.01)
        b2.mesh(stage, "/World/GroundPlane_grid", pts, cnt, idx, lm)
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(900.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun.CreateIntensityAttr(2000.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(40, 20, 0))
    LIFT = (DROOP + 0.014) if on_ground else 0.45         # 바닥에 세울 때: 늘어난 서스펜션 기준으로 타이어가 바닥에 막 닿는 높이
    for key, s, x in items:
        car = UsdGeom.Xform.Define(stage, f"/World/{key}")
        car.AddTranslateOp().Set(Gf.Vec3d(x, 0, LIFT))
        car.GetPrim().GetReferences().AddReference(f"./{key}.usda")
        if on_ground:
            continue
        top = LIFT + s["jack_bottom_z"]
        for i, (jx, jy) in enumerate(s["jack_points"]):
            st = b2.box(stage, f"/World/Stands_{key}/stand_{i}", (0.26, 0.26, top), (x + jx, jy, top / 2), sm)
            UsdPhysics.CollisionAPI.Apply(st.GetPrim())
    stage.GetRootLayer().Save()
    b2.MATS.clear()


if __name__ == "__main__":
    os.makedirs(CARS_DIR, exist_ok=True)
    keys = [k for k in SPECS if not k.startswith("_")]
    out = {}
    for k in keys:
        out[k] = build_car(k)
        test_scene(os.path.join(CARS_DIR, k + "_test.usda"), [(k, out[k], 0.0)])
    test_scene(os.path.join(CARS_DIR, "cars_lineup.usda"), [(k, out[k], 3.2 * i) for i, k in enumerate(keys)])
    test_scene(os.path.join(CARS_DIR, "cars_ground.usda"), [(k, out[k], 3.2 * i) for i, k in enumerate(keys)], on_ground=True)
    json.dump(out, open(os.path.join(CARS_DIR, "cars_summary.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    for k, s in out.items():
        print(k, "| curb", s["curb_mass"], "sprung", s["sprung_mass"], "unsprung", s["unsprung_total"], "| plates", s["body_collision_plates"], "| gap mm", s["tire_to_body_min_gap_mm"])
