"""너트 보관 트레이: 45도로 기울인 판에 M12 볼트(더미 스터드)를 엇갈리게 세운 것. 치수/좌표 계산 (USD·로봇 계획이 같이 쓴다)"""
import math

import numpy as np

ROWS, COLS, PITCH_U, PITCH_V = 5, 8, 0.055, 0.048
PLATE = (0.50, 0.30, 0.012)          # 폭, 경사 방향 길이, 두께 [m]
PEG_R, PEG_H = 0.006, 0.035          # M12, 판 위로 35 mm
TILT_DEG = 45.0
NUT_SEAT = 0.005                     # 소켓 끝(tcp)이 멈추는 높이: 판 위 5 mm (휠 너트에서와 같은 값)
USE = [(2, c) for c in (1, 2, 3, 4, 5)]      # 이번 바퀴 너트 5개를 끼울 볼트 (가운데 줄)


def frame(center, yaw_deg, tilt_deg=TILT_DEG):
    """반환: C, u(판 가로), v(경사 위쪽), n(판 바깥 법선 = 볼트 방향). 공구는 -n 방향으로 접근"""
    y, t = math.radians(yaw_deg), math.radians(tilt_deg)
    n = np.array([-math.cos(y) * math.sin(t), -math.sin(y) * math.sin(t), math.cos(t)])
    u = np.array([-math.sin(y), math.cos(y), 0.0])
    v = np.cross(n, u)
    return np.array(center, dtype=float), u, v / np.linalg.norm(v), n


def peg_base(fr, r, c):
    C, u, v, n = fr
    return C + u * ((c - (COLS - 1) / 2) * PITCH_U + (PITCH_U / 4 if r % 2 else -PITCH_U / 4)) + v * ((r - (ROWS - 1) / 2) * PITCH_V)


def tool_rotation(yaw_deg, tilt_deg=TILT_DEG, roll_deg=0.0):
    """tcp 회전행렬 (world): 공구 축(tcp -Z)이 판 안쪽(-n)을 향한다. roll = 공구 축 둘레 회전"""
    R0 = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)          # 공구 축 -> world +X
    y, t, r = math.radians(yaw_deg), math.radians(tilt_deg), math.radians(roll_deg)
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
    Rr = np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])
    return Rz @ Ry @ R0 @ Rr
