"""
vision_capture.py - Isaac Sim 안에서 손목 카메라 한 장을 '필요할 때만' 찍는다 (Replicator annotator).

  렌더 프로덕트를 촬영 순간에 만들고, 찍고 나면 지운다. 그래서 촬영하지 않는 동안에는 이 카메라 때문에 느려지지 않는다.
  (ROS 2 토픽을 켜면 ros2_cameras.py 가 따로 계속 렌더링한다. 시연이 버벅이면 --no-ros 로 끄고 이 촬영만 쓰면 된다.)

  컬러와 깊이를 같은 카메라 프림에서 찍으므로 둘은 픽셀 단위로 정렬돼 있다 (RealSense 의 align_to_color 와 같은 상태).
  깊이 = distance_to_image_plane (광축 방향 거리, m).
  카메라 자세 = 카메라 강체(RSD455)의 물리 자세 x USD 에 적힌 강체->카메라 고정 변환. 실기에서는 로봇 FK x 핸드-아이 보정값이다.
"""
import numpy as np
from pxr import UsdGeom

from cell_kinematics import T_from, mat, quat_to_R
from nut_vision import Intrinsics


class WristCamera:
    def __init__(self, stage, body_path, cam_name="Camera_OmniVision_OV9782_Color", resolution=(1280, 800)):
        self.stage = stage
        self.body_path = body_path
        self.cam_path = body_path + "/" + cam_name
        cam = UsdGeom.Camera(stage.GetPrimAtPath(self.cam_path))
        if not cam:
            raise RuntimeError("camera prim not found: " + self.cam_path)
        self.res = (int(resolution[0]), int(resolution[1]))
        self.K = Intrinsics.from_usd(cam.GetFocalLengthAttr().Get(), cam.GetHorizontalApertureAttr().Get(), self.res[0], self.res[1],
                                     cam.GetHorizontalApertureOffsetAttr().Get() or 0.0, cam.GetVerticalApertureOffsetAttr().Get() or 0.0)
        xc = UsdGeom.XformCache()
        W_body = mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(body_path)))
        W_cam = mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(self.cam_path)))
        self.T_body_cam = np.linalg.inv(W_body) @ W_cam          # 고정 (장착 브래킷)
        self.T_body_cam[:3, :3] /= np.linalg.norm(self.T_body_cam[:3, :3], axis=0)

    def pose(self):
        from omni.physx import get_physx_interface
        d = get_physx_interface().get_rigidbody_transformation(self.body_path)
        p, r = d["position"], d["rotation"]
        return T_from(np.array([p[0], p[1], p[2]], dtype=float), quat_to_R(r[3], r[0], r[1], r[2])) @ self.T_body_cam

    async def capture(self, step, n_frames=1, warmup=6, max_wait=60):
        """step: 한 프레임 진행하는 코루틴 함수 (cell_demo 의 tick: 보호 정지 감시가 계속 돈다).
        n_frames 장 깊이를 평균한다. 돌려줌: rgb (H,W,3 uint8), depth (H,W float32), T_world_cam (촬영 중 평균 아님: 마지막 프레임)"""
        import omni.replicator.core as rep

        rp = rep.create.render_product(self.cam_path, self.res)
        a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        a_rgb.attach([rp])
        a_dep.attach([rp])
        rgb, depths = None, []
        try:
            for i in range(max_wait):
                await step()
                if i < warmup:
                    continue
                d = a_dep.get_data()
                c = a_rgb.get_data()
                if d is None or c is None or np.size(d) == 0 or np.size(c) == 0:
                    continue
                d = np.asarray(d, dtype=np.float32).reshape(self.res[1], self.res[0])
                if np.count_nonzero(np.isfinite(d) & (d > 0)) < d.size * 0.2:
                    continue
                depths.append(d)
                rgb = np.asarray(c).reshape(self.res[1], self.res[0], -1)[..., :3].astype(np.uint8)
                if len(depths) >= n_frames:
                    break
            if not depths:
                raise RuntimeError("camera returned no depth after %d frames" % max_wait)
            T = self.pose()
        finally:
            for a in (a_rgb, a_dep):
                try:
                    a.detach([rp])
                except Exception:
                    try:
                        a.detach()
                    except Exception:
                        pass
            try:
                rp.destroy()
            except Exception:
                pass
        stack = np.stack(depths)
        good = np.isfinite(stack) & (stack > 0)
        cnt = good.sum(axis=0)
        depth = (np.where(good, stack, 0.0).sum(axis=0) / np.maximum(cnt, 1)).astype(np.float32)     # 무효 픽셀은 0
        return rgb, depth, T

    # ---- 연속 촬영 (트랙 이동 중 스캔): 렌더 프로덕트 하나를 열어 두고 필요할 때마다 최신 프레임을 읽는다
    def open_stream(self):
        import omni.replicator.core as rep

        self._rp = rep.create.render_product(self.cam_path, self.res)
        self._ann = [rep.AnnotatorRegistry.get_annotator(n) for n in ("rgb", "distance_to_image_plane")]
        for a in self._ann:
            a.attach([self._rp])

    def grab(self):
        """최신 프레임 (rgb, depth, T_world_cam) 또는 아직 없으면 None. 렌더가 물리보다 1프레임쯤 늦을 수 있다."""
        c, d = (a.get_data() for a in self._ann)
        if d is None or c is None or np.size(d) == 0 or np.size(c) == 0:
            return None
        d = np.asarray(d, dtype=np.float32).reshape(self.res[1], self.res[0])
        if np.count_nonzero(np.isfinite(d) & (d > 0)) < d.size * 0.2:
            return None
        return np.asarray(c).reshape(self.res[1], self.res[0], -1)[..., :3].astype(np.uint8), d, self.pose()

    def close_stream(self):
        for a in getattr(self, "_ann", []):
            try:
                a.detach([self._rp])
            except Exception:
                try:
                    a.detach()
                except Exception:
                    pass
        try:
            self._rp.destroy()
        except Exception:
            pass
        self._ann = []


# ----------------------------------------------------------------------------- 관측 자세 / 시험용 도우미 (순수 함수)
def tool_rotation_for_axis(n_out, R_ref):
    """R_ref (공구 z 축 = 휠 바깥 법선이 되는 기준 자세) 를 최소 회전으로 돌려 공구 z 축을 n_out 에 맞춘다"""
    a = R_ref[:, 2] / np.linalg.norm(R_ref[:, 2])
    b = np.asarray(n_out, dtype=float) / np.linalg.norm(n_out)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-12:
        return R_ref.copy()
    Kx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return (np.eye(3) + Kx + Kx @ Kx / (1 + c)) @ R_ref


def observation_tcp(T_wheel_prior, z_top_local, T_tcp_cam, R_tool, dist=0.55):
    """카메라 광축이 (사전) 휠 축과 겹치고, 너트 윗면 평면에서 dist [m] 떨어지도록 하는 TCP 자세"""
    n = T_wheel_prior[:3, 2]
    c = (T_wheel_prior @ np.array([0.0, 0.0, z_top_local, 1.0]))[:3]
    p_cam = c + n * dist
    return T_from(p_cam - R_tool @ T_tcp_cam[:3, 3], R_tool)


def tcp_to_camera(stage, tcp_path, cam: WristCamera):
    xc = UsdGeom.XformCache()
    W_tcp = mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(tcp_path)))
    W_cam = mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(cam.cam_path)))
    T = np.linalg.inv(W_tcp) @ W_cam
    T[:3, :3] /= np.linalg.norm(T[:3, :3], axis=0)
    return T


def shift_car(stage, offset, prims=("/World/Car", "/World/Corner_FL", "/World/Corner_FR", "/World/Corner_RL", "/World/Corner_RR", "/World/Wheel_FL", "/World/Wheel_RL")):
    """시험용: 차(차체·코너 4개·휠)를 offset [m] 만큼 옮긴다. 정지 상태에서 호출 (관절 프레임은 몸체 기준이라 그대로 맞음)"""
    from pxr import Gf, Usd
    old = stage.GetEditTarget()
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))      # 세션 레이어: 씬 파일은 안 바뀜, rear_wheel.py 가 만든 휠도 옮겨짐
    try:
        for p in prims:
            prim = stage.GetPrimAtPath(p)
            if not prim.IsValid():
                continue
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(op.Get() + Gf.Vec3d(*[float(v) for v in offset]))
                    break
    finally:
        stage.SetEditTarget(old)


FIXED_CORNER_PARTS = ("caliper_body", "caliper_bridge", "knuckle", "strut", "spring_seat", "lower_arm", "tie_rod", "cv_boot", "drive_shaft")


def rotate_wheel(stage, deg, corner="/World/Corner_FL", wheel="/World/Wheel_FL", suspension="/World/Car/suspension_FL"):
    """시험용: 휠 + 허브(스터드·너트 포함)를 차축 둘레로 deg 만큼 돌린다. 정지 상태에서 호출.
    - 휠과 허브를 같이 돌리므로 스터드-구멍, 너트 고정 관절은 그대로 맞는다.
    - 서스펜션 관절의 허브 쪽 좌표틀을 반대로 돌려 차와의 연결 자세는 그대로 둔다.
    - 캘리퍼·너클·스트럿 등 실제로는 안 도는 부품은 반대로 돌려 제자리에 둔다 (같은 강체라 충돌체도 같이 제자리)."""
    from pxr import Gf, Usd, UsdPhysics

    if abs(deg) < 1e-9:
        return
    old = stage.GetEditTarget()
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    try:
        _rotate_wheel(stage, deg, corner, wheel, suspension, Gf, UsdPhysics)
    finally:
        stage.SetEditTarget(old)


def _rotate_wheel(stage, deg, corner, wheel, suspension, Gf, UsdPhysics):
    q = Gf.Rotation(Gf.Vec3d(0, 0, 1), float(deg)).GetQuat()                 # 몸체 로컬 z = 차축
    for p in (corner, wheel):
        for op in UsdGeom.Xformable(stage.GetPrimAtPath(p)).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                o = op.Get()
                op.Set(Gf.Quatf(Gf.Quatd(o.GetReal(), Gf.Vec3d(o.GetImaginary())) * q))
                break
        else:
            raise RuntimeError("no orient op on " + p)
    j = UsdPhysics.Joint(stage.GetPrimAtPath(suspension))
    inv = Gf.Rotation(Gf.Vec3d(0, 0, 1), -float(deg))
    r1 = j.GetLocalRot1Attr().Get()
    j.GetLocalRot1Attr().Set(Gf.Quatf(inv.GetQuat() * Gf.Quatd(r1.GetReal(), Gf.Vec3d(r1.GetImaginary()))))
    j.GetLocalPos1Attr().Set(Gf.Vec3f(inv.TransformDir(Gf.Vec3d(j.GetLocalPos1Attr().Get()))))
    for name in FIXED_CORNER_PARTS:
        prim = stage.GetPrimAtPath(corner + "/" + name)
        if not prim.IsValid():
            continue
        x = UsdGeom.Xformable(prim)
        ops = x.GetOrderedXformOps()
        spin = x.AddRotateZOp(opSuffix="unspin")
        spin.Set(-float(deg))
        x.SetXformOpOrder([spin] + ops)                                       # 맨 앞 = 허브 원점 기준 회전
