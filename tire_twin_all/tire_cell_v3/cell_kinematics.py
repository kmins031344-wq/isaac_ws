"""
cell_kinematics.py - 로봇 기구학 (Isaac Sim 없이도 import 가능: numpy + pxr 만 사용)

  Arm        : USD 관절 프레임에서 FK 모델을 만들고, 감쇠 최소제곱(DLS) 수치 IK 를 푼다
  cart_path  : TCP 직선 이동을 관절 경유점으로 변환
  sample     : 관절 경유점 보간

확장 지점
  - IK 를 Lula / cuRobo / MoveIt 등으로 바꾸려면 Arm.ik 와 같은 시그니처(T_target, seed) -> (q_deg, ok) 를 가진 객체로 교체
  - 관절 한계·축·링크 프레임은 전부 USD 에서 읽으므로 로봇 기종(RS025N 등)을 바꿔도 이 파일은 그대로 쓴다
"""
import math

import numpy as np
from pxr import UsdGeom, UsdPhysics

ARM_JOINTS = ["base_link/joint1", "link1/joint2", "link2/joint3", "link3/joint4", "link4/joint5", "link5/joint6"]


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
    def __init__(self, stage, robot_path, tool_path, sign=1.0):
        """robot_path: 로봇 루트, tool_path: 공구 루트(하위에 base, base/tcp 가 있어야 함). 공구 축 = tcp 로컬 -Z"""
        self.sign = sign
        self.robot_path, self.tool_path = robot_path, tool_path
        xc = UsdGeom.XformCache()
        self.T_base = mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(robot_path + "/base_link")))
        self.L0, self.L1inv, self.axis, lo, hi, vmax = [], [], [], [], [], []
        for jp in ARM_JOINTS:
            prim = stage.GetPrimAtPath(robot_path + "/" + jp)
            j = UsdPhysics.RevoluteJoint(prim)
            p0, r0 = j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get()
            p1, r1 = j.GetLocalPos1Attr().Get(), j.GetLocalRot1Attr().Get()
            self.L0.append(T_from(np.array(p0), quat_to_R(r0.GetReal(), *r0.GetImaginary())))
            self.L1inv.append(np.linalg.inv(T_from(np.array(p1), quat_to_R(r1.GetReal(), *r1.GetImaginary()))))
            self.axis.append({"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[j.GetAxisAttr().Get()])
            lo.append(j.GetLowerLimitAttr().Get())
            hi.append(j.GetUpperLimitAttr().Get())
            v = prim.GetAttribute("physxJoint:maxJointVelocity")
            vmax.append(float(v.Get()) if v and v.Get() is not None else 180.0)
        self.lo_cmd, self.hi_cmd = np.array(lo) + 1.0, np.array(hi) - 1.0     # 1 deg margin
        self.vmax = np.array(vmax)                                            # deg/s (제조사 최대 속도)
        W6 = mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(robot_path + "/link6")))
        self.T6_tcp = np.linalg.inv(W6) @ mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(tool_path + "/base/tcp")))
        self.T6_tool = np.linalg.inv(W6) @ mat(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(tool_path + "/base")))
        self.rest_err = float(np.linalg.norm(self.fk6(np.zeros(6)) - W6))

    def set_track(self, d, direction=(0.0, 1.0, 0.0)):
        """리니어 트랙 위치 d [m] 를 반영 (베이스가 direction 으로 d 만큼 평행 이동). FK/IK 가 이 위치 기준으로 계산된다"""
        if not hasattr(self, "T_base0"):
            self.T_base0 = self.T_base.copy()
        self.track = float(d)
        self.T_base = self.T_base0.copy()
        self.T_base[:3, 3] += float(d) * np.asarray(direction, dtype=float)

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


def sample(path, s):
    """piecewise-linear joint interpolation, s in [0,1], parameterised by joint travel"""
    seg = [max(float(np.max(np.abs(path[i + 1] - path[i]))), 1e-9) for i in range(len(path) - 1)]
    x = min(max(s, 0.0), 1.0) * sum(seg)
    for i, L in enumerate(seg):
        if x <= L or i == len(seg) - 1:
            return path[i] + (path[i + 1] - path[i]) * min(x / L, 1.0)
        x -= L


def min_duration(arm, path, margin=1.5):
    """관절 최대 속도를 넘지 않는 최소 이동 시간 [s] (smoothstep 의 최대 속도비 1.5 반영)"""
    t = 0.0
    for i in range(len(path) - 1):
        t += float(np.max(np.abs(path[i + 1] - path[i]) / arm.vmax))
    return t * 1.5 * margin
