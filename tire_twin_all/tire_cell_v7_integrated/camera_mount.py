"""
RealSense D455 (sensors/rsd455/rsd455.usd) 를 로봇 2대의 공구에 장착한다.

  로봇 A 휠 그리퍼  : 센터 하우징 +X 쪽 옆면 (조 사이 빈 자리), 공구 진입 방향(-Z)을 봄
  로봇 B 너트러너   : 브래킷 클램프 -X 쪽 옆면 (케이블 커넥터 반대편), 소켓 방향(-Z)을 봄

카메라(에셋의 RSD455 강체, 0.103 kg)는 공구 Xform 아래(base 와 나란히)에 두고 고정 관절(Camera_RSD455_mount_joint)로 base 에 묶는다.
IMU 센서가 부모 강체를 읽기 때문에 강체로 남겨 둔다. 외형만 있고 충돌체는 없다.

  build_scene_v3.py 가 씬을 만들 때 자동으로 호출한다.
  이미 만들어진 씬에 붙이기만 할 때:  python camera_mount.py [tire_cell_v3.usda]
"""
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

CAM_FILE = "./sensors/rsd455/rsd455.usd"         # 씬 파일 기준 상대 경로
CAM_NAME = "Camera_RSD455"
# 에셋 기준 프레임(/Root): +X = 보는 방향, +Z = 영상 위쪽, 앞면 x=0, 뒷면 x=-0.026, 폭 y ±0.062, 높이 z ±0.0145
CAM_HALF_H = 0.0145
CAM_DEPTH = 0.026

# 공구 base 프레임(플랜지 면 z=0, -Z = 작업 방향) 에서의 장착 위치
#   rot: 카메라 +X -> 공구 -Z, 영상 위쪽 -> 바깥쪽(반경 방향)
#   bracket: (크기, 중심) 카메라와 공구 사이 연결판 (외형만)
MOUNTS = {
    "gripper": dict(
        pos=(0.125 + 0.0155 + CAM_HALF_H, 0.0, -0.015 - CAM_DEPTH),
        rot_y=90.0, rot_z=0.0,
        bracket=((0.0155, 0.06, CAM_DEPTH), (0.125 + 0.0155 / 2, 0.0, -0.015 - CAM_DEPTH / 2)),
    ),
    "nutrunner": dict(
        pos=(-(0.055 + 0.004 + CAM_HALF_H), 0.0, -0.320 - CAM_DEPTH / 2),
        rot_y=90.0, rot_z=180.0,
        bracket=((0.004, 0.06, CAM_DEPTH), (-(0.055 + 0.002), 0.0, -0.320)),
    ),
}


def _bracket_material(stage, root):
    path = root + "/Looks/cam_bracket"
    mat = UsdShade.Material.Get(stage, path)
    if mat:
        return mat
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.15, 0.15, 0.16))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.6)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def add_camera(stage, tool_path, kind):
    """tool_path: /World/RobotX/<공구>, kind: 'gripper' | 'nutrunner'. 카메라 Root 프림 경로를 돌려준다."""
    m = MOUNTS[kind]
    base = tool_path + "/base"
    C = tool_path + "/" + CAM_NAME          # base(강체) 의 자식이면 강체 중첩이 되므로 공구 Xform 바로 아래에 둔다
    for p in (C, C + "_mount_joint"):
        if stage.GetPrimAtPath(p):
            stage.RemovePrim(p)
    mount = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), m["rot_y"]) * Gf.Rotation(Gf.Vec3d(0, 0, 1), m["rot_z"]))
    mount.SetTranslateOnly(Gf.Vec3d(*m["pos"]))                       # 공구 base 프레임 기준 카메라 자세
    base_local = UsdGeom.Xformable(stage.GetPrimAtPath(base)).GetLocalTransformation()
    cam = UsdGeom.Xform.Define(stage, C)
    cam.GetPrim().GetReferences().AddReference(CAM_FILE)
    cam.ClearXformOpOrder()
    cam.AddTransformOp().Set(mount * base_local)
    # 에셋의 RSD455 는 강체(0.103 kg)이고 IMU 가 이 강체를 읽는다 -> 강체로 두고 공구 base 에 고정 관절로 붙인다
    body = C + "/RSD455"
    xc = UsdGeom.XformCache()
    rel = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(body)) * xc.GetLocalToWorldTransform(stage.GetPrimAtPath(base)).GetInverse()
    rel.Orthonormalize()
    fj = UsdPhysics.FixedJoint.Define(stage, C + "_mount_joint")
    fj.CreateBody0Rel().SetTargets([Sdf.Path(base)])
    fj.CreateBody1Rel().SetTargets([Sdf.Path(body)])
    fj.CreateLocalPos0Attr(Gf.Vec3f(rel.ExtractTranslation()))
    fj.CreateLocalRot0Attr(Gf.Quatf(rel.ExtractRotationQuat()))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    links = [Sdf.Path(tool_path.rsplit("/", 1)[0] + "/" + n) for n in ("link5", "link6")]
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(body)).CreateFilteredPairsRel().SetTargets(links + [Sdf.Path(base)])
    size, center = m["bracket"]
    br = UsdGeom.Cube.Define(stage, base + "/" + CAM_NAME + "_bracket")
    br.CreateSizeAttr(1.0)
    br.AddTranslateOp().Set(Gf.Vec3d(*center))
    br.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(br.GetPrim()).Bind(_bracket_material(stage, tool_path))
    cam.GetPrim().SetCustomDataByKey("source", "Intel RealSense D455 (rsd455.usd), visual only, no collision")
    return C


def add_cell_cameras(stage):
    world = stage.GetPrimAtPath("/World")
    paths = {}
    for key, tool, kind in (("cell:camera_a", "/World/RobotA/WheelGripper", "gripper"),
                            ("cell:camera_b", "/World/RobotB/Nutrunner", "nutrunner")):
        c = add_camera(stage, tool, kind)
        world.SetCustomDataByKey(key, c + "/RSD455")
        paths[kind] = c
    return paths


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    scene = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "tire_cell_v3.usda")
    st = Usd.Stage.Open(scene)
    for k, v in add_cell_cameras(st).items():
        print("camera", k, v)
    st.GetRootLayer().Save()
    print("saved", scene)
