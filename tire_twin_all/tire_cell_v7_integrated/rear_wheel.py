"""
rear_wheel.py - 열려 있는 씬에서 왼쪽 뒷바퀴(RL)를 앞바퀴(FL)와 같은 '탈거 가능한 휠'로 바꾸고, 뒷바퀴 작업용 너트 트레이를 하나 더 둔다.

  씬 파일(.usda)은 건드리지 않는다. 바꾼 내용은 세션 레이어(메모리)에만 들어가므로 Isaac 을 닫으면 사라진다.
  run_cell_demo.py 가 PLAY 전에 add_removable_wheel(stage) 를 부른다.

바뀌는 것 (모두 build_scene_v3.py 가 FL 에 하는 것과 같은 형상·값)
  - Corner_RL/wheel (허브 강체에 붙어 있던 외형 휠) 비활성화, 허브 질량에서 휠 22 kg 빼기
  - Corner_RL 에 충돌체 추가: 휠 장착면, 디스크, 파일럿, 스터드 5개, 캘리퍼, 너클, 스트럿
  - Corner_RL/nuts/nut_0..4 (체결된 너트 외형)
  - /World/Wheel_RL : 22 kg 강체 휠 (FL 과 같은 외형·충돌체)
  - /World/nut_clamp_joint_RL (켜짐), /World/stud_joint_RL, /World/grasp_joint_RL (꺼짐)
  - /World/NutTray_RL : 뒷바퀴 위치에서 로봇 B 기준으로 앞 트레이와 같은 자리에 놓은 트레이
"""
import json
import os
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))


def _builders():
    """build_scene_v3 의 형상 함수들 (import 해도 씬을 만들지 않음)"""
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    argv = sys.argv
    sys.argv = [argv[0], HERE]                                  # build_scene_v3 가 argv[1] 을 출력 폴더로 읽음 (파일은 안 씀)
    try:
        import build_scene_v3 as b3
    finally:
        sys.argv = argv
    return b3


def add_removable_wheel(stage, corner_name="RL", tray_shift=None):
    """corner_name 의 휠을 탈거 가능하게 만든다. tray_shift: 앞 트레이에서 옮길 world 벡터 (None = 두 코너의 y 차이).
    돌려줌: dict (경로들, 트레이 슬롯 등). cell_demo 가 그대로 쓴다."""
    b3 = _builders()
    b2, W = b3.b2, b3.W
    K = "/World/Corner_" + corner_name
    Wp = "/World/Wheel_" + corner_name
    kp = stage.GetPrimAtPath(K)
    if not kp.IsValid():
        raise RuntimeError("corner not found: " + K)
    world = stage.GetPrimAtPath("/World")
    cell = world.GetCustomData().get("cell", {})
    GA = cell["gripper_path"]
    old_target = stage.GetEditTarget()
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    try:
        pm = {n: UsdShade.Material(stage.GetPrimAtPath("/World/PhysicsMaterials/" + n)) for n in ("tire_rubber", "steel")}
        # ---- 허브: 붙어 있던 휠 떼기, 질량, 충돌체
        stage.GetPrimAtPath(K + "/wheel").SetActive(False)
        m_uns = b3.C["unsprung_rear"] if corner_name.startswith("R") else b3.C["unsprung_front"]
        UsdPhysics.MassAPI.Apply(kp).GetMassAttr().Set(float(m_uns))
        Cc = K + "/colliders"
        PAD_Z0 = b3.PAD_Z0
        b3.col_cyl(stage, Cc + "/hat", 0.078, -0.012, PAD_Z0, pm["steel"], tight=True)
        b3.col_cyl(stage, Cc + "/disc", 0.150, -0.031, -0.005, pm["steel"])
        b3.col_cyl(stage, Cc + "/pilot", b3.BORE_R, PAD_Z0, PAD_Z0 + W["pilot_length"], pm["steel"], tight=True)
        for i in range(W["lugs"]):
            b3.col_cyl(stage, f"{Cc}/stud_{i}", W["stud_radius"], PAD_Z0, PAD_Z0 + W["stud_exposed"], pm["steel"], center=b3.lug_xy(i), tight=True)
        for n in ("caliper_body", "caliper_bridge"):
            b3.collide(stage.GetPrimAtPath(K + "/" + n), pm["steel"], hull=True)
        for n in ("knuckle", "strut"):
            b3.collide(stage.GetPrimAtPath(K + "/" + n), pm["steel"])
        UsdGeom.Xform.Define(stage, K + "/nuts")
        b3.nuts(stage, K + "/nuts")
        # ---- 휠 강체 (허브와 같은 자세)
        xc = UsdGeom.XformCache()
        Tk = xc.GetLocalToWorldTransform(kp)
        Tk.Orthonormalize()
        wheel = UsdGeom.Xform.Define(stage, Wp)
        wheel.AddTranslateOp().Set(Tk.ExtractTranslation())
        wheel.AddOrientOp().Set(Gf.Quatf(Tk.ExtractRotationQuat()))
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
        wp.CreateAttribute("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        b2.build_wheel(stage, Wp, with_nuts=False, collision=False)
        b3.wheel_colliders(stage, Wp, pm["tire_rubber"], pm["steel"])
        b3.contact_report(wp)
        # ---- 관절 (FL 과 같은 구성, 이름 끝에 _RL)
        def joint(kind, name, b0, b1, enabled):
            j = kind.Define(stage, "/World/%s_%s" % (name, corner_name))
            j.CreateJointEnabledAttr(bool(enabled))
            j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
            j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
            for a in (j.CreateLocalPos0Attr, j.CreateLocalPos1Attr):
                a(Gf.Vec3f(0, 0, 0))
            for a in (j.CreateLocalRot0Attr, j.CreateLocalRot1Attr):
                a(Gf.Quatf(1, 0, 0, 0))
            return j
        joint(UsdPhysics.FixedJoint, "nut_clamp_joint", K, Wp, True)
        sj = joint(UsdPhysics.PrismaticJoint, "stud_joint", K, Wp, False)
        sj.CreateAxisAttr("Z")
        sj.CreateLowerLimitAttr(0.0)
        sj.CreateUpperLimitAttr(0.30)
        d = UsdPhysics.DriveAPI.Apply(sj.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(3000.0)
        gj = joint(UsdPhysics.FixedJoint, "grasp_joint", GA + "/base", Wp, False)
        gj.CreateExcludeFromArticulationAttr(True)
        # ---- 뒷바퀴용 너트 트레이: 앞 트레이를 평행 이동한 복사본
        if tray_shift is None:
            y0 = xc.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Corner_FL")).ExtractTranslation()[1]
            tray_shift = (0.0, float(Tk.ExtractTranslation()[1] - y0), 0.0)
        tray_shift = np.asarray(tray_shift, dtype=float)
        src = UsdGeom.Xformable(stage.GetPrimAtPath("/World/NutTray"))
        tray = UsdGeom.Xform.Define(stage, "/World/NutTray_" + corner_name)
        tray.GetPrim().GetReferences().AddReference(os.path.join(HERE, "nut_tray.usda"))      # 세션 레이어라 절대 경로
        tray.AddTransformOp().Set(src.GetLocalTransformation() * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*tray_shift)))
        slots = [list(np.array(p) + tray_shift) for p in json.loads(cell["nut_tray_slots"])]
        nut_pos = [list(np.array(p) + tray_shift) for p in json.loads(cell["nut_tray_nut_positions"])]
    finally:
        stage.SetEditTarget(old_target)
    return {"name": corner_name, "wheel": Wp, "corner": K, "nut_clamp": "/World/nut_clamp_joint_" + corner_name,
            "stud_joint": "/World/stud_joint_" + corner_name, "grasp_joint": "/World/grasp_joint_" + corner_name,
            "tray_slots": slots, "tray_nut_positions": nut_pos, "tray_shift": tray_shift.tolist(),
            "suspension": "/World/Car/suspension_" + corner_name}
