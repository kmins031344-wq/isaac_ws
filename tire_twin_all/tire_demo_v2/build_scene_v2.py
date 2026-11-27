"""
타이어 탈거 시연 씬 v2 생성

구성
  - 로봇: Kawasaki RS080N (Isaac Sim 정식 에셋, rs080n_pzv160_flat.usda 에서 참조)
  - 차체: osrf/car_demo 의 Prius 차체 메시 (Apache-2.0, 가입 없이 GitHub 에서 받음) -> car/prius_body.usda
  - 휠/타이어, 허브·브레이크 디스크·캘리퍼·스터드, 4조 휠 그리퍼, 리프트: 치수 기반 파라메트릭 모델 (원본 CAD 아님)

물리
  - 휠: 22 kg 강체, 관성 지정, 타이어 마찰 재질
  - 스터드 구속: 차축 방향으로만 미끄러지는 prismatic joint (/World/stud_joint). 스터드 길이만큼 빠지면 스크립트가 해제
  - 파지: 조 4개가 트레드를 실제 접촉으로 조이고, 패드 양쪽 턱이 타이어 어깨를 걸어 잡음 (FixedJoint 미사용)

실행: python build_scene_v2.py <출력폴더> <car_demo 저장소 경로>
"""
import math
import os
import shutil
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else "."
CAR_REPO = sys.argv[2] if len(sys.argv) > 2 else None
ROBOT_FILE = "rs080n_pzv160_flat.usda"
SCENE = os.path.join(OUT_DIR, "tire_demo_scene_v2.usda")
GRIP_FILE = os.path.join(OUT_DIR, "wheel_gripper_4jaw.usda")
CAR_FILE = os.path.join(OUT_DIR, "car", "prius_body.usda")

# ------------------------------------------------------------------ 치수
# 휠: 17인치 5x114.3 (현대·기아 준중형/중형급 기준), 225/45R17
TIRE_R, TIRE_W, RIM_R = 0.316, 0.225, 0.2159
HW = TIRE_W / 2
WHEEL_MASS = 22.0                       # 휠+타이어 (17~22 kg 범위의 무거운 쪽)
WHEEL_INERTIA = (0.60, 0.60, 1.06)      # kg m^2 (타이어 9.5 kg @r0.28 + 휠 12.5 kg @r0.16 근사)
PCD_R, N_LUG = 0.05715, 5
BORE_R = 0.03355                        # 허브 보어 67.1 mm
PAD_Z0, PAD_Z1 = 0.0425, 0.0675         # 휠 장착면(뒤) / 디스크 바깥면 (휠 로컬 z, +z = 바깥)
STUD_EXPOSED = 0.045                    # 허브면에서 스터드 돌출 길이

# 그리퍼 (커스텀 4조 외경 파지, 개념 설계)
STROKE = 0.050
SQUEEZE = 0.006                         # 닫힘 목표가 타이어 표면 안쪽으로 들어가는 양 -> 조임력
PAD_FACE_X = 0.027                      # jaw 원점(바 중심)에서 패드 접촉면까지
LIP_DEPTH, LIP_T, LIP_GAP = 0.022, 0.012, 0.006
R_OPEN = TIRE_R - SQUEEZE + STROKE + PAD_FACE_X        # 열림 시 바 중심 반경
Z_CARR_BOT = -0.132
Z_TIRE_OUT = Z_CARR_BOT - 0.018 - LIP_T - LIP_GAP      # 파지 시 타이어 바깥면 z (그리퍼 좌표)
Z_TCP = Z_TIRE_OUT - HW
Z_TIP = Z_TCP - HW - LIP_GAP - LIP_T
BASE_MASS, JAW_MASS = 16.0, 2.0
JAW_STIFF, JAW_DAMP, JAW_MAXF = 1.0e5, 1.0e4, 1500.0
JAW_ANGLES = (45.0, 135.0, 225.0, 315.0)

# 배치 (world: 로봇 베이스 원점, 정면 +X)
WHEEL_C = Gf.Vec3d(1.85, 0.0, 1.00)
DROOP = 0.09                            # 리프트에 올리면 서스펜션이 늘어나 휠이 차체 대비 내려감
CAR_WHEELS = {"FL": (0.767, -1.41), "FR": (-0.767, -1.41), "RL": (0.793, 1.45), "RR": (-0.793, 1.45)}   # 차 좌표 (URDF)
CAR_WHEEL_Z = 0.30

MATS = {}


# ------------------------------------------------------------------ 공용 도우미
def material(stage, name, color, metallic=0.0, rough=0.5, opacity=1.0, root="/World/Looks"):
    key = root + "/" + name
    if key in MATS:
        return MATS[key]
    m = UsdShade.Material.Define(stage, key)
    s = UsdShade.Shader.Define(stage, key + "/shader")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    if opacity < 1.0:
        s.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    MATS[key] = (m, color)
    return MATS[key]


def paint(prim, mat):
    m, color = mat
    UsdGeom.Gprim(prim).CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(m)


def collide(prim, phys_mat=None, hull=False):
    UsdPhysics.CollisionAPI.Apply(prim)
    if hull:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    if phys_mat is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics")


def mesh(stage, path, pts, counts, idx, mat):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    if mat:
        paint(m.GetPrim(), mat)
    return m


def lathe(stage, path, profile, mat, n=64, center=(0.0, 0.0), phase=0.0):
    """닫힌 단면 [(r, z)...] 을 (center 를 지나는) Z축 둘레로 회전"""
    pts, counts, idx = [], [], []
    m = len(profile)
    for k in range(n):
        a = 2 * math.pi * k / n + phase
        for r, z in profile:
            pts.append((center[0] + r * math.cos(a), center[1] + r * math.sin(a), z))
    for k in range(n):
        k2 = (k + 1) % n
        for i in range(m):
            i2 = (i + 1) % m
            idx += [k * m + i, k2 * m + i, k2 * m + i2, k * m + i2]
            counts.append(4)
    return mesh(stage, path, pts, counts, idx, mat)


def sector(stage, path, r0, r1, a0, a1, z0, z1, mat, n=10):
    """환형 부채꼴 블록 (각도 deg)"""
    pts, counts, idx = [], [], []
    for k in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * k / n)
        c, s = math.cos(a), math.sin(a)
        pts += [(r0 * c, r0 * s, z0), (r1 * c, r1 * s, z0), (r1 * c, r1 * s, z1), (r0 * c, r0 * s, z1)]
    for k in range(n):
        b, d = 4 * k, 4 * (k + 1)
        for i in range(4):
            i2 = (i + 1) % 4
            idx += [b + i, d + i, d + i2, b + i2]
            counts.append(4)
    idx += [0, 1, 2, 3, 4 * n, 4 * n + 3, 4 * n + 2, 4 * n + 1]
    counts += [4, 4]
    return mesh(stage, path, pts, counts, idx, mat)


def box(stage, path, size, center, mat, rot_z=0.0, rot_y=0.0, rot_x=0.0):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    if rot_z:
        xf.AddRotateZOp().Set(rot_z)
    if rot_y:
        xf.AddRotateYOp().Set(rot_y)
    if rot_x:
        xf.AddRotateXOp().Set(rot_x)
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    if mat:
        paint(c.GetPrim(), mat)
    return c


def cyl(stage, path, r, h, center, mat, axis="Z"):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(r)
    c.CreateHeightAttr(h)
    c.CreateAxisAttr(axis)
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*center))
    if mat:
        paint(c.GetPrim(), mat)
    return c


def lug_angle(i):
    return 90.0 + 360.0 * i / N_LUG


# ------------------------------------------------------------------ 휠 (로컬 +Z = 바깥면)
def build_wheel(stage, root, with_nuts, collision, phys_mat=None, looks="/World/Looks"):
    rubber = material(stage, "tire_rubber", (0.03, 0.03, 0.03), 0.0, 0.85, root=looks)
    alloy = material(stage, "alloy", (0.78, 0.79, 0.81), 1.0, 0.28, root=looks)
    dark = material(stage, "alloy_dark", (0.22, 0.22, 0.24), 1.0, 0.4, root=looks)
    nutm = material(stage, "lug_nut", (0.6, 0.6, 0.62), 1.0, 0.25, root=looks)
    R = TIRE_R
    # 타이어 단면: 비드 -> 사이드월 -> 숄더 -> 트레드(원주 홈 3줄) -> 반대쪽
    side = [(RIM_R, -0.096), (0.236, -0.108), (0.270, -HW), (0.296, -0.109), (0.309, -0.100), (R, -0.088)]
    tread = []
    for gz in (-0.045, 0.0, 0.045):
        tread += [(R, gz - 0.005), (R - 0.007, gz - 0.004), (R - 0.007, gz + 0.004), (R, gz + 0.005)]
    prof = side + tread + [(r, -z) for r, z in reversed(side)]
    lathe(stage, root + "/tire", prof, rubber, n=72)
    # 림 배럴 + 플랜지
    lathe(stage, root + "/rim_barrel", [(0.198, -0.100), (0.228, -0.100), (0.228, -0.093), (0.2165, -0.090), (0.2165, 0.090),
                                        (0.228, 0.093), (0.228, 0.100), (0.198, 0.100), (0.203, 0.085), (0.203, -0.085)], alloy, n=72)
    # 장착 패드: 안쪽 링 + 바깥 링 + 러그 구멍 사이 채움 + 러그 시트(구멍 뚫린 링)
    lathe(stage, root + "/pad_inner", [(BORE_R + 0.0002, PAD_Z0), (0.049, PAD_Z0), (0.049, PAD_Z1), (BORE_R + 0.0002, PAD_Z1)], alloy)
    lathe(stage, root + "/pad_outer", [(0.0655, PAD_Z0), (0.092, PAD_Z0), (0.092, PAD_Z1 + 0.004), (0.0655, PAD_Z1)], alloy)
    for i in range(N_LUG):
        a = lug_angle(i)
        sector(stage, f"{root}/pad_web_{i}", 0.0485, 0.066, a + 9.0, a + 72.0 - 9.0, PAD_Z0, PAD_Z1, alloy)
        cx, cy = PCD_R * math.cos(math.radians(a)), PCD_R * math.sin(math.radians(a))
        lathe(stage, f"{root}/lug_seat_{i}", [(0.0072, PAD_Z0), (0.0138, PAD_Z0), (0.0138, PAD_Z1), (0.0095, PAD_Z1), (0.0072, PAD_Z1 - 0.006)],
              dark, n=24, center=(cx, cy))
        if with_nuts:
            lathe(stage, f"{root}/lug_nut_{i}", [(0.0, PAD_Z1 - 0.004), (0.0121, PAD_Z1 - 0.004), (0.0121, PAD_Z1 + 0.014), (0.009, PAD_Z1 + 0.020), (0.0, PAD_Z1 + 0.020)],
                  nutm, n=6, center=(cx, cy))
    # 스포크 10개 (패드 -> 림, 바깥쪽으로 살짝 올라감)
    r_a, r_b, z_a, z_b = 0.088, 0.204, PAD_Z1 - 0.008, 0.082
    L = math.hypot(r_b - r_a, z_b - z_a)
    tilt = -math.degrees(math.atan2(z_b - z_a, r_b - r_a))
    for k in range(10):
        a = lug_angle(0) + 36.0 * k + 18.0 + (6.0 if k % 2 else -6.0)
        rm = (r_a + r_b) / 2
        box(stage, f"{root}/spoke_{k}", (L, 0.024, 0.018), (rm * math.cos(math.radians(a)), rm * math.sin(math.radians(a)), (z_a + z_b) / 2),
            alloy, rot_z=a, rot_y=tilt)
    if collision:
        col = lathe(stage, root + "/collision", [(0.0, -HW), (R - 0.006, -HW), (R, -HW + 0.006), (R, HW - 0.006), (R - 0.006, HW), (0.0, HW)], None, n=32)
        col.CreatePurposeAttr("guide")
        collide(col.GetPrim(), phys_mat, hull=True)


# ------------------------------------------------------------------ 허브/브레이크 (로컬 +Z = 바깥, 원점 = 휠 중심)
def build_hub(stage, root, up, with_wheel_on):
    """up: 로컬 X 가 world 위(+1)/아래(-1)"""
    steel = material(stage, "steel", (0.45, 0.45, 0.47), 1.0, 0.45)
    cast = material(stage, "cast_iron", (0.2, 0.2, 0.21), 0.6, 0.7)
    red = material(stage, "caliper_red", (0.75, 0.05, 0.05), 0.2, 0.4)
    stud = material(stage, "stud", (0.8, 0.72, 0.35), 1.0, 0.3)
    black = material(stage, "susp_black", (0.06, 0.06, 0.06), 0.3, 0.6)
    # 브레이크 디스크: 햇 + 벤틸레이티드 마찰면(판 2장 + 코어)
    cyl(stage, root + "/disc_hat_face", 0.078, 0.008, (0, 0, PAD_Z0 - 0.004), steel)
    lathe(stage, root + "/disc_hat_wall", [(0.068, -0.012), (0.078, -0.012), (0.078, PAD_Z0 - 0.008), (0.068, PAD_Z0 - 0.008)], steel)
    lathe(stage, root + "/disc_plate_out", [(0.068, -0.014), (0.150, -0.014), (0.150, -0.005), (0.068, -0.005)], steel, n=72)
    lathe(stage, root + "/disc_plate_in", [(0.085, -0.031), (0.150, -0.031), (0.150, -0.022), (0.085, -0.022)], steel, n=72)
    lathe(stage, root + "/disc_vanes", [(0.090, -0.022), (0.146, -0.022), (0.146, -0.014), (0.090, -0.014)], cast, n=36)
    # 허브 파일럿 + 액슬 너트
    lathe(stage, root + "/hub_pilot", [(0.022, PAD_Z0), (BORE_R - 0.0003, PAD_Z0), (BORE_R - 0.0003, PAD_Z0 + 0.012), (0.022, PAD_Z0 + 0.012)], steel)
    lathe(stage, root + "/axle_nut", [(0.0, PAD_Z0), (0.018, PAD_Z0), (0.018, PAD_Z0 + 0.010), (0.0, PAD_Z0 + 0.010)], cast, n=6)
    # 스터드 M12 x 5
    for i in range(N_LUG):
        a = math.radians(lug_angle(i))
        cyl(stage, f"{root}/stud_{i}", 0.006, STUD_EXPOSED + 0.004, (PCD_R * math.cos(a), PCD_R * math.sin(a), PAD_Z0 - 0.004 + (STUD_EXPOSED + 0.004) / 2), stud)
    # 캘리퍼 (디스크 뒤쪽 위)
    a0 = 20.0 if up > 0 else 200.0
    sector(stage, root + "/caliper_body", 0.100, 0.176, a0, a0 + 56.0, -0.052, 0.010, red, n=8)
    sector(stage, root + "/caliper_bridge", 0.150, 0.180, a0 + 6.0, a0 + 50.0, -0.050, 0.008, red, n=6)
    # 너클, 스트럿, 로어암, 타이로드, 드라이브샤프트
    box(stage, root + "/knuckle", (0.20, 0.09, 0.05), (0.0, 0.0, -0.062), cast)
    cyl(stage, root + "/wheel_bearing", 0.055, 0.05, (0, 0, -0.05), cast)
    cyl(stage, root + "/strut", 0.028, 0.40, (up * 0.28, 0.0, -0.12), black, axis="X")
    lathe(stage, root + "/spring_seat", [(0.03, 0.0), (0.075, 0.0), (0.075, 0.012), (0.03, 0.012)], black, n=24)
    box(stage, root + "/lower_arm", (0.045, 0.07, 0.34), (-up * 0.105, 0.0, -0.22), black, rot_y=up * 8.0)
    cyl(stage, root + "/tie_rod", 0.009, 0.34, (up * 0.02, 0.11, -0.22), steel)
    cyl(stage, root + "/drive_shaft", 0.016, 0.42, (0, 0, -0.29), steel)
    lathe(stage, root + "/cv_boot", [(0.016, -0.20), (0.045, -0.10), (0.045, -0.085), (0.016, -0.085)], black, n=24)
    # spring_seat 을 스트럿 위쪽으로 옮김
    ss = UsdGeom.Xformable(stage.GetPrimAtPath(root + "/spring_seat"))
    ss.AddTranslateOp().Set(Gf.Vec3d(up * 0.26, 0.0, -0.12))
    ss.AddRotateYOp().Set(90.0)


# ------------------------------------------------------------------ 4조 휠 그리퍼
def build_gripper(path_file):
    if os.path.exists(path_file):
        os.remove(path_file)
    stage = Usd.Stage.CreateNew(path_file)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    R = "/WheelGripper4Jaw"
    L = R + "/Looks"
    root = UsdGeom.Xform.Define(stage, R)
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("source", "parametric concept design (not vendor CAD)")
    body = material(stage, "body_blue", (0.10, 0.28, 0.55), 0.6, 0.4, root=L)
    alu = material(stage, "aluminium", (0.72, 0.73, 0.75), 1.0, 0.35, root=L)
    rail = material(stage, "rail_steel", (0.35, 0.35, 0.37), 1.0, 0.3, root=L)
    dark = material(stage, "housing_dark", (0.12, 0.12, 0.13), 0.5, 0.5, root=L)
    pad_m = material(stage, "pad_rubber", (0.05, 0.05, 0.05), 0.0, 0.9, root=L)
    pm = UsdShade.Material.Define(stage, L + "/PadPhysics")
    p = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
    p.CreateStaticFrictionAttr(0.9)
    p.CreateDynamicFrictionAttr(0.8)
    p.CreateRestitutionAttr(0.0)

    base = UsdGeom.Xform.Define(stage, R + "/base")
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(BASE_MASS)
    B = R + "/base"
    collide(cyl(stage, B + "/adapter_plate", 0.10, 0.015, (0, 0, -0.0075), dark).GetPrim())
    hub = lathe(stage, B + "/center_housing", [(0.0, -0.015), (0.135, -0.015), (0.135, -0.085), (0.0, -0.085)], body, n=8, phase=math.pi / 8)
    collide(hub.GetPrim(), hull=True)
    lathe(stage, B + "/housing_cover", [(0.0, -0.085), (0.11, -0.085), (0.10, -0.094), (0.0, -0.094)], dark, n=32)
    cyl(stage, B + "/servo_motor", 0.042, 0.12, (0.0, -0.165, -0.05), dark, axis="Y")
    box(stage, B + "/servo_gearbox", (0.10, 0.06, 0.07), (0.0, -0.10, -0.05), dark)
    for i, ang in enumerate(JAW_ANGLES):
        ca, sa = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        r_end = R_OPEN + 0.068
        rm, ln = (0.08 + r_end) / 2, r_end - 0.08
        collide(box(stage, f"{B}/arm_{i}", (ln, 0.07, 0.06), (rm * ca, rm * sa, -0.055), alu, rot_z=ang).GetPrim())
        box(stage, f"{B}/rail_{i}", (ln - 0.08, 0.03, 0.012), ((rm + 0.03) * ca, (rm + 0.03) * sa, -0.091), rail, rot_z=ang)
        box(stage, f"{B}/arm_cap_{i}", (0.012, 0.08, 0.07), ((r_end + 0.006) * ca, (r_end + 0.006) * sa, -0.058), dark, rot_z=ang)
        box(stage, f"{B}/screw_cover_{i}", (ln - 0.06, 0.036, 0.01), (rm * ca, rm * sa, -0.020), dark, rot_z=ang)
    tcp = UsdGeom.Xform.Define(stage, B + "/tcp")
    tcp.AddTranslateOp().Set(Gf.Vec3d(0, 0, Z_TCP))

    chan_w = TIRE_W + 2 * LIP_GAP
    for i, ang in enumerate(JAW_ANGLES):
        ca, sa = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        J = f"{R}/jaw_{i}"
        jaw = UsdGeom.Xform.Define(stage, J)
        jaw.AddTranslateOp().Set(Gf.Vec3d(R_OPEN * ca, R_OPEN * sa, 0))
        jaw.AddRotateZOp().Set(ang + 180.0)                      # 로컬 +X = 안쪽(닫힘 방향)
        UsdPhysics.RigidBodyAPI.Apply(jaw.GetPrim())
        UsdPhysics.MassAPI.Apply(jaw.GetPrim()).CreateMassAttr(JAW_MASS)
        collide(box(stage, J + "/carriage", (0.09, 0.09, 0.035), (0, 0, -0.1145), rail).GetPrim())
        bar_len = Z_CARR_BOT - Z_TIP
        collide(box(stage, J + "/finger_bar", (0.03, 0.07, bar_len), (0, 0, Z_CARR_BOT - bar_len / 2), alu).GetPrim())
        box(stage, J + "/finger_rib", (0.045, 0.012, 0.12), (-0.035, 0, Z_CARR_BOT - 0.055), alu, rot_y=-18.0)
        collide(box(stage, J + "/pad", (0.012, 0.09, chan_w), (0.021, 0, Z_TCP), pad_m).GetPrim(), pm)
        lx = PAD_FACE_X + LIP_DEPTH / 2
        collide(box(stage, J + "/lip_outer", (LIP_DEPTH, 0.09, LIP_T), (lx, 0, Z_TCP + chan_w / 2 + LIP_T / 2), alu).GetPrim(), pm)
        collide(box(stage, J + "/lip_inner", (LIP_DEPTH, 0.09, LIP_T), (lx, 0, Z_TCP - chan_w / 2 - LIP_T / 2), alu).GetPrim(), pm)
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{R}/joints/jaw_{i}_joint")
        j.CreateBody0Rel().SetTargets([base.GetPath()])
        j.CreateBody1Rel().SetTargets([jaw.GetPath()])
        j.CreateAxisAttr("X")
        j.CreateLowerLimitAttr(0.0)
        j.CreateUpperLimitAttr(STROKE)
        a = math.radians(ang + 180.0)
        j.CreateLocalPos0Attr(Gf.Vec3f(R_OPEN * ca, R_OPEN * sa, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(a / 2), 0, 0, math.sin(a / 2)))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(JAW_STIFF)
        d.CreateDampingAttr(JAW_DAMP)
        d.CreateMaxForceAttr(JAW_MAXF)
        d.CreateTargetPositionAttr(0.0)
    stage.GetRootLayer().Save()
    MATS.clear()


# ------------------------------------------------------------------ 차체 메시 (OBJ -> USD)
def convert_car(repo, out_file):
    src = os.path.join(repo, "prius_description", "meshes")
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    for t in ("Hybrid.png", "Hybrid_Interior.png"):
        shutil.copy(os.path.join(src, t), os.path.join(os.path.dirname(out_file), t))
    V, VT, faces, cur = [], [], {}, None
    for line in open(os.path.join(src, "hybrid_body.obj"), errors="ignore"):
        p = line.split()
        if not p:
            continue
        if p[0] == "v":
            V.append((float(p[1]) * 0.01, float(p[2]) * 0.01, float(p[3]) * 0.01))      # cm -> m
        elif p[0] == "vt":
            VT.append((float(p[1]), float(p[2])))
        elif p[0] == "usemtl":
            cur = p[1]
        elif p[0] == "f":
            faces.setdefault(cur, []).append([tuple(int(x) if x else 0 for x in (t.split("/") + ["", ""])[:3]) for t in p[1:]])
    if os.path.exists(out_file):
        os.remove(out_file)
    stage = Usd.Stage.CreateNew(out_file)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/PriusBody")
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("source", "osrf/car_demo prius_description/meshes/hybrid_body.obj (Apache-2.0)")
    for name, fl in faces.items():
        used = sorted({v[0] for f in fl for v in f})
        remap = {vi: k for k, vi in enumerate(used)}
        m = UsdGeom.Mesh.Define(stage, "/PriusBody/" + name)
        m.CreatePointsAttr([Gf.Vec3f(*V[vi - 1]) for vi in used])
        m.CreateFaceVertexCountsAttr([len(f) for f in fl])
        m.CreateFaceVertexIndicesAttr([remap[v[0]] for f in fl for v in f])
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set([Gf.Vec2f(*VT[v[1] - 1]) if v[1] else Gf.Vec2f(0, 0) for f in fl for v in f])
        mat = UsdShade.Material.Define(stage, "/PriusBody/Looks/" + name)
        sh = UsdShade.Shader.Define(stage, f"/PriusBody/Looks/{name}/shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        if name == "Windows":
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.05, 0.07, 0.09))
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.45)
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05)
            m.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.07, 0.09)])
            m.CreateDisplayOpacityAttr([0.45])
        else:
            rd = UsdShade.Shader.Define(stage, f"/PriusBody/Looks/{name}/st_reader")
            rd.CreateIdAttr("UsdPrimvarReader_float2")
            rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            tx = UsdShade.Shader.Define(stage, f"/PriusBody/Looks/{name}/texture")
            tx.CreateIdAttr("UsdUVTexture")
            tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./" + name + ".png")
            tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
            tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
            tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
            tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.2)
            m.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.57, 0.6)])
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        # 정적 충돌: 메시 삼각형 그대로 사용 (그리퍼/휠이 차체에 닿는지 확인용)
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("none")
    stage.GetRootLayer().Save()


# ------------------------------------------------------------------ 씬
def car_to_world(x, y, z, t):
    return Gf.Vec3d(-x + t[0], -y + t[1], z + t[2])          # 차를 Z축 180도 돌려 좌측이 로봇 쪽(-X)


def main():
    build_gripper(GRIP_FILE)
    have_car = bool(CAR_REPO)
    if have_car:
        convert_car(CAR_REPO, CAR_FILE)
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
    sc.CreateGravityMagnitudeAttr(9.81)
    sc.GetPrim().AddAppliedSchema("PhysxSceneAPI")
    sc.GetPrim().CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(120)
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000)
    dl = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    dl.CreateIntensityAttr(2500)
    dl.AddRotateXYZOp().Set(Gf.Vec3f(-40, 25, 0))

    floor_m = material(stage, "floor", (0.42, 0.43, 0.45), 0.0, 0.8)
    collide(box(stage, "/World/Ground", (16, 16, 0.2), (2.0, -1.0, -0.1), floor_m).GetPrim())
    tire_pm = UsdShade.Material.Define(stage, "/World/Looks/TirePhysics")
    p = UsdPhysics.MaterialAPI.Apply(tire_pm.GetPrim())
    p.CreateStaticFrictionAttr(0.9)
    p.CreateDynamicFrictionAttr(0.8)
    p.CreateRestitutionAttr(0.05)

    # ---------- 로봇 + 그리퍼 ----------
    robot = UsdGeom.Xform.Define(stage, "/World/RS080N").GetPrim()
    robot.GetReferences().AddReference("./" + ROBOT_FILE, "/World/RS080N")
    robot.AddAppliedSchema("PhysxArticulationAPI")
    robot.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    robot.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(4)
    stage.OverridePrim("/World/RS080N/Gripper").SetActive(False)
    G = "/World/RS080N/WheelGripper"
    grip = UsdGeom.Xform.Define(stage, G)
    grip.GetPrim().GetReferences().AddReference("./" + os.path.basename(GRIP_FILE))
    grip.GetPrim().RemoveAPI(UsdPhysics.ArticulationRootAPI)
    xc = UsdGeom.XformCache()
    link6 = stage.GetPrimAtPath("/World/RS080N/link6")
    flip = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))
    grip.ClearXformOpOrder()
    grip.AddTransformOp().Set(flip * xc.GetLocalToWorldTransform(link6) * xc.GetLocalToWorldTransform(robot).GetInverse())
    fj = UsdPhysics.FixedJoint.Define(stage, G + "/flange_mount_joint")
    fj.CreateBody0Rel().SetTargets([link6.GetPath()])
    fj.CreateBody1Rel().SetTargets([Sdf.Path(G + "/base")])
    fj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot0Attr(Gf.Quatf(0, 1, 0, 0))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    links = [Sdf.Path(f"/World/RS080N/{n}") for n in ["world", "base_link", "link1", "link2", "link3", "link4", "link5", "link6"]]
    gbodies = [Sdf.Path(G + "/base")] + [Sdf.Path(f"{G}/jaw_{i}") for i in range(4)]
    for b in gbodies:
        UsdPhysics.FilteredPairsAPI.Apply(stage.OverridePrim(b)).CreateFilteredPairsRel().SetTargets(links)

    # ---------- 차량 ----------
    t = (WHEEL_C[0] + CAR_WHEELS["FL"][0], WHEEL_C[1] + CAR_WHEELS["FL"][1], WHEEL_C[2] - CAR_WHEEL_Z + DROOP)
    UsdGeom.Xform.Define(stage, "/World/Car")
    if have_car:
        body = UsdGeom.Xform.Define(stage, "/World/Car/Body")
        body.GetPrim().GetReferences().AddReference("./car/" + os.path.basename(CAR_FILE))
        body.AddTranslateOp().Set(Gf.Vec3d(*t))
        body.AddRotateZOp().Set(180.0)
    q_near = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90).GetQuat())     # 로컬 +Z -> world -X
    q_far = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 1, 0), 90).GetQuat())
    for name, (cx, cy) in CAR_WHEELS.items():
        pos = car_to_world(cx, cy, CAR_WHEEL_Z - DROOP, t)
        near = cx > 0
        hub = UsdGeom.Xform.Define(stage, f"/World/Car/Hub_{name}")
        hub.AddTranslateOp().Set(pos)
        hub.AddOrientOp().Set(q_near if near else q_far)
        build_hub(stage, f"/World/Car/Hub_{name}", 1.0 if near else -1.0, name != "FL")
        if name != "FL":
            w = UsdGeom.Xform.Define(stage, f"/World/Car/Wheel_{name}")
            w.AddTranslateOp().Set(pos)
            w.AddOrientOp().Set(q_near if near else q_far)
            build_wheel(stage, f"/World/Car/Wheel_{name}", with_nuts=True, collision=False)

    # ---------- 탈거 대상 휠 (FL, 너트 없음) ----------
    W = "/World/Wheel_FL"
    wheel = UsdGeom.Xform.Define(stage, W)
    wheel.AddTranslateOp().Set(WHEEL_C)
    wheel.AddOrientOp().Set(q_near)
    wp = wheel.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(wp)
    ma = UsdPhysics.MassAPI.Apply(wp)
    ma.CreateMassAttr(WHEEL_MASS)
    ma.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*WHEEL_INERTIA))
    ma.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
    wp.AddAppliedSchema("PhysxRigidBodyAPI")
    wp.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    wp.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(1.0)
    build_wheel(stage, W, with_nuts=False, collision=True, phys_mat=tire_pm)
    # 스터드 구속: 차축 방향으로만 미끄러짐 (0 = 허브에 밀착, + = 바깥으로 빠짐). 감쇠 = 스터드/허브 마찰 근사
    sj = UsdPhysics.PrismaticJoint.Define(stage, "/World/stud_joint")
    sj.CreateBody1Rel().SetTargets([wheel.GetPath()])
    sj.CreateAxisAttr("Z")
    sj.CreateLowerLimitAttr(0.0)
    sj.CreateUpperLimitAttr(0.30)
    sj.CreateLocalPos0Attr(Gf.Vec3f(WHEEL_C))
    sj.CreateLocalRot0Attr(q_near)
    sj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    sj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    d = UsdPhysics.DriveAPI.Apply(sj.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(3000.0)
    d.CreateTargetVelocityAttr(0.0)
    # 예비용 파지 joint (GRASP_MODE="joint" 일 때만 사용)
    gj = UsdPhysics.FixedJoint.Define(stage, "/World/grasp_joint")
    gj.CreateJointEnabledAttr(False)
    gj.CreateExcludeFromArticulationAttr(True)
    gj.CreateBody0Rel().SetTargets([Sdf.Path(G + "/base")])
    gj.CreateBody1Rel().SetTargets([wheel.GetPath()])
    gj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    gj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    gj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    gj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    # ---------- 시저 리프트 (정적, 외형만) ----------
    lift_blue = material(stage, "lift_blue", (0.08, 0.22, 0.55), 0.5, 0.45)
    lift_yel = material(stage, "lift_yellow", (0.9, 0.68, 0.05), 0.4, 0.5)
    padm = material(stage, "lift_pad", (0.05, 0.05, 0.05), 0.0, 0.9)
    chrome = material(stage, "chrome", (0.8, 0.8, 0.82), 1.0, 0.15)
    cc = car_to_world(0.0, 0.0, 0.0, t)
    body_bottom = t[2] + 0.145
    top = body_bottom - 0.03
    for i, dx in enumerate((-0.62, 0.62)):
        x = cc[0] + dx
        box(stage, f"/World/Lift/base_{i}", (0.42, 2.3, 0.05), (x, cc[1], 0.025), lift_blue)
        box(stage, f"/World/Lift/runway_{i}", (0.42, 2.1, 0.07), (x, cc[1], top - 0.035), lift_blue)
        for j, dy in enumerate((-0.75, 0.75)):
            box(stage, f"/World/Lift/pad_{i}_{j}", (0.16, 0.12, 0.03), (x, cc[1] + dy, top + 0.015), padm)
        h = top - 0.07 - 0.05
        Ls = math.hypot(1.7, h)
        ang = math.degrees(math.atan2(h, 1.7))
        for j, s in enumerate((1, -1)):
            box(stage, f"/World/Lift/leg_{i}_{j}", (0.05, Ls, 0.09), (x + 0.13 * s, cc[1], 0.05 + h / 2), lift_yel, rot_x=s * ang)
        cyl(stage, f"/World/Lift/pivot_{i}", 0.02, 0.36, (x, cc[1], 0.05 + h / 2), chrome, axis="X")
        hc = UsdGeom.Xformable(cyl(stage, f"/World/Lift/hydraulic_{i}", 0.035, 0.75, (0, 0, 0), chrome, axis="Y").GetPrim())
        hc.ClearXformOpOrder()
        hc.AddTranslateOp().Set(Gf.Vec3d(x, cc[1] - 0.35, 0.05 + h * 0.36))
        hc.AddRotateXOp().Set(ang * 0.8)

    # 스크립트가 읽는 값
    wprim = world.GetPrim()
    for k, v in {
        "demo:wheel_center": Gf.Vec3d(WHEEL_C), "demo:tire_width": TIRE_W, "demo:gripper_path": G,
        "demo:jaw_close": STROKE, "demo:tip_below_tcp": Z_TCP - Z_TIP, "demo:stud_exposed": STUD_EXPOSED,
        "demo:gripper_mass": BASE_MASS + 4 * JAW_MASS, "demo:wheel_mass": WHEEL_MASS, "demo:tcp_from_flange": -Z_TCP,
    }.items():
        wprim.SetCustomDataByKey(k, v)
    stage.GetRootLayer().Save()
    print("saved", SCENE)
    print("gripper: open pad dia %.0f mm, closed target %.0f mm (tire %.0f), lip clearance on approach %.0f mm, TCP %.1f mm from flange, tip %.1f mm"
          % (2 * (R_OPEN - PAD_FACE_X) * 1000, 2 * (TIRE_R - SQUEEZE) * 1000, 2 * TIRE_R * 1000,
             (R_OPEN - PAD_FACE_X - LIP_DEPTH - TIRE_R) * 1000, -Z_TCP * 1000, -Z_TIP * 1000))
    # 하중 검토 (RS080N: 가반 80 kg, JT4/JT5 336 N·m, JT6 194 N·m)
    mg, mw = BASE_MASS + 4 * JAW_MASS, WHEEL_MASS
    arm_g, arm_w = 0.165 + 0.10, 0.165 - Z_TCP
    M5 = 9.81 * (mg * arm_g + mw * arm_w)
    print("load: gripper %.0f kg + wheel %.0f kg = %.0f kg (limit 80) | JT5 static moment %.0f N·m (limit 336)" % (mg, mw, mg + mw, M5))


if __name__ == "__main__":
    main()
