"""
리니어 트랙 1개(차 길이 방향, world Y)에 로봇 2대를 올린다. 캐리지 2개, 로봇마다 직선 관절 1개.

  /World/Track                      레일 외형 (고정, 충돌 없음)
  /World/RobotX/rail_anchor         월드에 고정된 관절 뿌리 (기존 root_joint 가 이걸 잡는다)
  /World/RobotX/track_joint         직선 관절: rail_anchor -> RobotX/world, 축 = world Y, 위치 구동
  /World/RobotX/world/carriage      캐리지 외형 (로봇과 같이 움직임)

로봇 베이스는 캐리지 윗면 높이(params.json track.carriage_height)만큼 올라가고, 둘 다 x = track.x 선 위에 놓인다.
구동 목표: track_joint 의 drive:linear:physics:targetPosition [m] (0 = 앞바퀴 작업 위치, 음수 = 차 뒤쪽).

  build_scene_v3.py 가 씬을 만들 때 자동으로 호출한다.
  이미 만들어진 씬에 넣기만 할 때:  python track_mount.py [tire_cell_v3.usda]
"""
import json
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROBOTS = ("/World/RobotA", "/World/RobotB")
TRACK = "/World/Track"
BED_HALF_W, RAIL_X, CARRIAGE = 0.45, 0.32, (0.90, 0.76)      # 레일 베드 반폭, 레일 위치, 캐리지 (x, y) 크기 [m]


def _mat(stage, name, rgb, metallic, rough):
    path = TRACK + "/Looks/" + name
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _box(stage, path, size, center, mat):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.ClearXformOpOrder()
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
    return c


def _base_pose(stage, R):
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(R))
    t, yaw = Gf.Vec3d(0), 0.0
    for op in xf.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            t = Gf.Vec3d(op.Get())
        elif op.GetOpName() == "xformOp:orient":
            q = op.Get()
            yaw = 2.0 * math.degrees(math.atan2(q.GetImaginary()[2], q.GetReal()))
    return t, yaw


def _set_base_pose(stage, R, pos, yaw_deg):
    q = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg).GetQuat()
    for op in UsdGeom.Xformable(stage.GetPrimAtPath(R)).GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            op.Set(Gf.Vec3d(*pos))
        elif op.GetOpName() == "xformOp:orient":
            if op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
                op.Set(Gf.Quatd(q.GetReal(), q.GetImaginary()))
            else:
                op.Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
    return q


def add_track(stage, P):
    T = P["track"]
    H, lo, hi = float(T["carriage_height"]), float(T["travel"][0]), float(T["travel"][1])
    world = stage.GetPrimAtPath("/World")
    if stage.GetPrimAtPath(TRACK):
        stage.RemovePrim(TRACK)
    UsdGeom.Xform.Define(stage, TRACK)
    steel = _mat(stage, "track_steel", (0.55, 0.56, 0.58), 1.0, 0.3)
    bed = _mat(stage, "track_bed", (0.16, 0.17, 0.19), 0.6, 0.5)
    yel = _mat(stage, "track_yellow", (0.92, 0.70, 0.05), 0.3, 0.5)
    car = _mat(stage, "track_carriage", (0.25, 0.27, 0.30), 0.8, 0.4)

    ys, joints = [], []
    for R in ROBOTS:
        t, yaw = _base_pose(stage, R)
        pos = (float(T["x"]), t[1], H)
        q = _set_base_pose(stage, R, pos, yaw)
        qf = Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary()))
        qinv = Gf.Quatf(q.GetInverse().GetReal(), Gf.Vec3f(q.GetInverse().GetImaginary()))
        ys.append(pos[1])
        # 관절 뿌리: 월드에 고정. 기존 root_joint(고정 관절)의 body1 을 로봇 world 링크에서 이 프림으로 바꾼다
        A = R + "/rail_anchor"
        if stage.GetPrimAtPath(A):
            stage.RemovePrim(A)
        anchor = UsdGeom.Xform.Define(stage, A).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(anchor)
        ma = UsdPhysics.MassAPI.Apply(anchor)
        ma.CreateMassAttr(10.0)
        ma.CreateDiagonalInertiaAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        rj = stage.OverridePrim(R + "/root_joint")
        rj.GetRelationship("physics:body1").SetTargets([Sdf.Path(A)])
        rj.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*pos))
        rj.GetAttribute("physics:localRot0").Set(qf)
        # 캐리지 = 로봇의 world 링크 (질량 0 이던 더미 링크에 캐리지 질량을 준다)
        wl = stage.OverridePrim(R + "/world")
        mw = UsdPhysics.MassAPI.Apply(wl)
        mw.CreateMassAttr(float(T["carriage_mass"]))
        mw.CreateDiagonalInertiaAttr(Gf.Vec3f(8.0, 10.0, 16.0))
        # 직선 관절: 로봇이 yaw 만큼 돌아가 있으므로 관절 프레임을 -yaw 로 되돌려 축을 world Y 에 맞춘다
        J = R + "/track_joint"
        if stage.GetPrimAtPath(J):
            stage.RemovePrim(J)
        pj = UsdPhysics.PrismaticJoint.Define(stage, J)
        pj.CreateBody0Rel().SetTargets([Sdf.Path(A)])
        pj.CreateBody1Rel().SetTargets([Sdf.Path(R + "/world")])
        pj.CreateAxisAttr("Y")
        pj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
        pj.CreateLocalRot0Attr(qinv)
        pj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        pj.CreateLocalRot1Attr(qinv)
        pj.CreateLowerLimitAttr(lo)
        pj.CreateUpperLimitAttr(hi)
        d = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(T["drive_stiffness"]))
        d.CreateDampingAttr(float(T["drive_damping"]))
        d.CreateMaxForceAttr(float(T["drive_max_force"]))
        d.CreateTargetPositionAttr(0.0)
        pj.GetPrim().AddAppliedSchema("PhysxJointAPI")
        pj.GetPrim().CreateAttribute("physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float).Set(float(T["max_speed"]) * 1.5)
        joints.append(J)
        # 캐리지 외형 (world 링크의 자식 -> 같이 움직임. 레일 방향에 맞게 -yaw)
        Cg = R + "/world/carriage"
        if stage.GetPrimAtPath(Cg):
            stage.RemovePrim(Cg)
        cx = UsdGeom.Xform.Define(stage, Cg)
        cx.AddRotateZOp().Set(-yaw)
        _box(stage, Cg + "/plate", (CARRIAGE[0], CARRIAGE[1], 0.04), (0, 0, -0.02), car)
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            _box(stage, Cg + "/block_%d" % i, (0.09, 0.14, 0.05), (sx * RAIL_X, sy * 0.26, -0.055), yel)
        _box(stage, Cg + "/motor", (0.14, 0.22, 0.12), (0.37, 0.0, 0.06), bed)

    # 레일 외형: 두 로봇이 갈 수 있는 범위 전체 + 캐리지 반길이 + 여유
    y0, y1 = min(ys) + lo - CARRIAGE[1] / 2 - 0.15, max(ys) + hi + CARRIAGE[1] / 2 + 0.15
    L, yc, x = y1 - y0, (y0 + y1) / 2, float(T["x"])
    _box(stage, TRACK + "/bed", (2 * BED_HALF_W, L, 0.03), (x, yc, 0.015), bed)
    for i, s in enumerate((1, -1)):
        _box(stage, TRACK + "/rail_%d" % i, (0.045, L - 0.06, 0.035), (x + s * RAIL_X, yc, 0.0475), steel)
    _box(stage, TRACK + "/rack", (0.03, L - 0.06, 0.025), (x + 0.20, yc, 0.0425), steel)
    for i, y in enumerate((y0, y1)):
        _box(stage, TRACK + "/end_stop_%d" % i, (2 * BED_HALF_W, 0.05, 0.10), (x, y, 0.05), yel)
    for k, v in {"cell:track_joint_a": joints[0], "cell:track_joint_b": joints[1], "cell:track_travel": json.dumps([lo, hi]),
                 "cell:track_min_gap": float(T["min_gap"]), "cell:track_max_speed": float(T["max_speed"]),
                 "cell:track_base_y": json.dumps(ys), "cell:track_length": L}.items():
        world.SetCustomDataByKey(k, v)
    return {"joints": joints, "rail_y": (y0, y1), "length": L, "base_y": ys}


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    scene = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "tire_cell_v3.usda")
    P = json.load(open(os.path.join(here, "params.json"), encoding="utf-8"))
    st = Usd.Stage.Open(scene)
    # 이미 만들어진 씬에서는 로봇 B 를 params 의 위치로 먼저 옮긴다 (x 는 add_track 이 레일 선으로 맞춤)
    tb, _ = _base_pose(st, ROBOTS[1])
    _set_base_pose(st, ROBOTS[1], (P["robot_b_base"]["xy"][0], P["robot_b_base"]["xy"][1], tb[2]), P["robot_b_base"]["yaw_deg"])
    r = add_track(st, P)
    st.GetRootLayer().Save()
    print("track: rail y %.2f .. %.2f (%.2f m), carriages at y=%s, joints %s" % (r["rail_y"][0], r["rail_y"][1], r["length"], r["base_y"], r["joints"]))
    print("saved", scene)
