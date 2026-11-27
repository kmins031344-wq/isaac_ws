"""
nut_vision.py - 손목 카메라(컬러 + 정렬된 깊이) 한 장으로 휠 자세와 너트 5개 위치를 찾는다. numpy + OpenCV 만 쓴다.

  Isaac Sim 없이 import 된다 (실기 RealSense, ROS 2 노드에서도 같은 함수를 그대로 쓸 수 있음).

처리 순서
  1. ROI       : 배치도상의 휠 위치(사전값)를 영상에 투영해서 그 주변만 자른다.
  2. 너트 후보 : 깊이에서 '주변보다 튀어나온 작은 덩어리'를 찾는다 (white top-hat).
                 너트는 장착면보다 20 mm 솟아 있고 폭이 24 mm 라서, 길쭉한 스포크·반사광과 크기·모양으로 구분된다.
  3. 패턴 맞춤 : 후보 3개씩 골라 PCD 원(반지름 57.15 mm)에 맞는 조합을 찾는다 (RANSAC). 그 원 위에서
                 너트 배치(72도 간격)와 맞는 후보만 남긴다. 너트가 3개만 보여도 나머지 위치가 나온다.
  4. 허브 축   : 너트 사이 장착면의 깊이 점 수천 개에 평면을 맞춘다. 평면 법선이 허브 축이다.
  5. 휠 자세   : 축 + 중심 + 회전각 -> 4x4 휠 자세. 너트 위치 = 휠 자세 x 너트 로컬 좌표.

좌표 규약
  카메라 프레임은 USD 카메라 규약 (+X 오른쪽, +Y 위, -Z 보는 방향).
  깊이는 광축 방향 거리 [m] (RealSense depth / Isaac distance_to_image_plane 와 같음), 0 또는 inf = 무효.
"""
import itertools
import math

import cv2
import numpy as np


# ----------------------------------------------------------------------------- 카메라
class Intrinsics:
    def __init__(self, fx, fy, cx, cy, width, height):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.width, self.height = int(width), int(height)

    @classmethod
    def from_usd(cls, focal_length, h_aperture, width, height, h_offset=0.0, v_offset=0.0):
        """USD 카메라 속성 -> 픽셀 단위. Isaac 렌더러는 정사각 픽셀이라 fy = fx.
        OpenCV 규약(픽셀 i 의 중심 = 정수 i)이라 주점은 width/2 - 0.5. (이 0.5 를 빼먹으면 0.55 m 에서 0.4 mm 치우침)"""
        fx = width * focal_length / h_aperture
        return cls(fx, fx, width / 2.0 - 0.5 + h_offset * fx / focal_length, height / 2.0 - 0.5 - v_offset * fx / focal_length, width, height)

    def project(self, p_cam):
        """카메라 프레임 점 (..., 3) -> 픽셀 (u, v)"""
        p = np.asarray(p_cam, dtype=float)
        z = -p[..., 2]
        return np.stack([self.cx + self.fx * p[..., 0] / z, self.cy - self.fy * p[..., 1] / z], axis=-1)

    def backproject(self, u, v, depth):
        u, v, d = np.asarray(u, float), np.asarray(v, float), np.asarray(depth, float)
        return np.stack([(u - self.cx) / self.fx * d, -(v - self.cy) / self.fy * d, -d], axis=-1)


def _apply(T, p):
    p = np.asarray(p, dtype=float)
    return p @ T[:3, :3].T + T[:3, 3]


# ----------------------------------------------------------------------------- 파라미터
DEFAULTS = dict(
    roi_radius=0.16,          # 휠 중심 주변 ROI 반지름 [m] (사전값 오차 포함)
    depth_window=0.15,        # 사전 깊이 ± 이 범위 밖은 무시 [m] (소켓·차체·배경 제거)
    tophat_kernel=0.045,      # top-hat 커널 지름 [m]: 너트(24 mm)보다 크고 장착면(지름 184 mm)보다 작게
    min_prominence=0.008,     # 주변보다 이만큼 튀어나와야 후보 [m] (너트 20 mm, 장착면 테두리 4 mm)
    top_band=0.004,           # 덩어리 최고점에서 이만큼 아래까지를 너트 윗면으로 본다 [m]
    blob_diam=(0.012, 0.034), # 후보 덩어리의 등가 지름 범위 [m]
    min_fill=0.55,            # 덩어리 면적 / 최소 외접원 면적 (길쭉한 스포크 제거)
    circle_tol=0.004,         # PCD 원 반지름 허용 오차 [m]
    match_tol=0.005,          # 패턴 점과 후보 거리 허용치 [m]
    min_nuts=3,               # 이보다 적게 맞으면 실패
    pad_band=(0.037, 0.064),  # 장착면 평면 맞춤에 쓸 반경 범위 [m] (보어 33.5 mm 밖, 스포크 시작 88 mm 안)
    pad_nut_clear=0.017,      # 너트 중심에서 이 거리 안은 평면 맞춤에서 제외 [m]
    pad_below_top=0.020,      # 장착면이 너트 윗면보다 낮은 거리 [m] (build_scene_v2: 너트 윗면 PAD_Z1+0.020)
    max_rms=0.002,            # 패턴 잔차 RMS 상한 [m]
)


# ----------------------------------------------------------------------------- 기하 도우미
def _plane_fit(P):
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    return c, vt[2]


def _plane_fit_robust(P, iters=3, tol=0.0015):
    keep = np.ones(len(P), bool)
    c, n = _plane_fit(P)
    for _ in range(iters):
        d = np.abs((P - c) @ n)
        keep = d < max(tol, 2.5 * np.median(d[keep]) / 0.6745 if keep.any() else tol)
        if keep.sum() < 10:
            break
        c, n = _plane_fit(P[keep])
    return c, n, keep


def _circle3(a, b, c):
    """2D 세 점을 지나는 원 (중심, 반지름). 일직선이면 None"""
    d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if abs(d) < 1e-12:
        return None
    sa, sb, sc = a @ a, b @ b, c @ c
    ux = (sa * (b[1] - c[1]) + sb * (c[1] - a[1]) + sc * (a[1] - b[1])) / d
    uy = (sa * (c[0] - b[0]) + sb * (a[0] - c[0]) + sc * (b[0] - a[0])) / d
    o = np.array([ux, uy])
    return o, float(np.linalg.norm(a - o))


def _rigid2d(src, dst):
    """dst ~ R(phi) src + t, 최소제곱 (2D Procrustes)"""
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    s, d = src - ms, dst - md
    phi = math.atan2(float(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0])), float(np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1])))
    R = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
    return phi, md - R @ ms


def _rot2(phi):
    return np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])


# ----------------------------------------------------------------------------- 본체
def find_wheel(depth, K, T_world_cam, T_wheel_prior, nut_local, rgb=None, params=None, debug=False):
    """
    depth         : (H, W) 광축 방향 깊이 [m], 컬러와 정렬된 것
    K             : Intrinsics
    T_world_cam   : 4x4 카메라 자세 (USD 카메라 프레임 -> world)
    T_wheel_prior : 4x4 배치도상의 휠 자세 (ROI·회전각 기준). 휠 로컬 +Z = 바깥면 법선
    nut_local     : (N, 3) 휠 로컬 좌표의 너트 목표점 (소켓 TCP 가 가는 점)
    nut_local 의 z 는 '너트 윗면 - 15 mm' 이다. 윗면 높이 = nut_local z + top_above_target (params 로 바꿀 수 있음)

    돌려줌 dict: ok, reason, T_wheel, nuts_world (N,3), detected (너트 번호 -> 측정점), rms, n_found, debug (BGR 영상)
    """
    P = dict(DEFAULTS)
    P.update(params or {})
    nut_local = np.asarray(nut_local, dtype=float)
    top_above_target = P.get("top_above_target", 0.015)
    pattern = nut_local[:, :2]                                  # 휠 로컬 xy (너트 배치)
    pcd = float(np.mean(np.linalg.norm(pattern, axis=1)))
    z_top_local = float(np.mean(nut_local[:, 2])) + top_above_target
    out = {"ok": False, "reason": "", "T_wheel": None, "nuts_world": None, "detected": {}, "rms": None, "n_found": 0, "debug": None}

    T_cam_world = np.linalg.inv(T_world_cam)
    depth = np.asarray(depth, dtype=np.float32)
    H, W = depth.shape
    # ---- 1. ROI: 사전 휠 중심(너트 윗면 평면)을 영상에 투영
    c_prior_w = _apply(T_wheel_prior, [0.0, 0.0, z_top_local])
    c_prior_c = _apply(T_cam_world, c_prior_w)
    z0 = -float(c_prior_c[2])
    if z0 <= 0.05:
        out["reason"] = "wheel prior is behind the camera"
        return out
    u0, v0 = K.project(c_prior_c)
    r_px = int(P["roi_radius"] * K.fx / z0)
    x0, x1 = max(0, int(u0) - r_px), min(W, int(u0) + r_px)
    y0, y1 = max(0, int(v0) - r_px), min(H, int(v0) + r_px)
    if x1 - x0 < 20 or y1 - y0 < 20:
        out["reason"] = "wheel prior projects outside the image"
        return out
    D = depth[y0:y1, x0:x1].copy()
    valid = np.isfinite(D) & (D > 0) & (np.abs(D - z0) < P["depth_window"])
    # ---- 2. 너트 후보: 높이 = 사전 깊이 - 깊이 (카메라 쪽으로 튀어나올수록 큼), 무효 픽셀은 가장 낮게
    Hgt = np.where(valid, z0 - D, -P["depth_window"]).astype(np.float32)
    k = max(5, int(P["tophat_kernel"] * K.fx / z0) | 1)
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    tophat = cv2.morphologyEx(Hgt, cv2.MORPH_TOPHAT, ker)
    mask = ((tophat > P["min_prominence"]) & valid).astype(np.uint8)
    n_lab, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    vv, uu = np.mgrid[y0:y1, x0:x1]
    cands, rejected = [], []
    px_area = (z0 / K.fx) ** 2                                  # 픽셀 1개의 면적 [m^2] (근사)
    for i in range(1, n_lab):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 6:
            continue
        sel = lab == i
        diam = 2.0 * math.sqrt(area * px_area / math.pi)
        pts = np.column_stack(np.nonzero(sel)[::-1]).astype(np.float32)
        (_, _), rad = cv2.minEnclosingCircle(pts)
        fill = area / max(math.pi * rad * rad, 1.0)
        if not (P["blob_diam"][0] <= diam <= P["blob_diam"][1]) or fill < P["min_fill"]:
            rejected.append((sel, "size %.0f mm fill %.2f" % (diam * 1000, fill)))
            continue
        top = sel & (Hgt > Hgt[sel].max() - P["top_band"])
        pc = K.backproject(uu[top], vv[top], D[top])            # 윗면 픽셀을 3D 로 올려서 평균 (원근 보정)
        cands.append(_apply(T_world_cam, pc.mean(axis=0)))
    out["n_candidates"] = len(cands)
    # ---- 3. 패턴 맞춤 (RANSAC: 3개로 원을 만들고 PCD 와 맞는지 확인)
    if len(cands) < P["min_nuts"]:
        out["reason"] = "only %d nut candidates" % len(cands)
        out["debug"] = _draw(rgb, depth, K, T_cam_world, x0, y0, rejected, lab, cands, None, None) if debug else None
        return out
    C = np.array(cands)
    Rp = T_wheel_prior[:3, :3]
    n_prior = Rp[:, 2]
    best = None
    for tri in itertools.combinations(range(len(C)), 3):
        o3, nrm = _plane_fit(C[list(tri)])
        if abs(nrm @ n_prior) < math.cos(math.radians(15)):     # 사전 축과 15도 넘게 다르면 버림
            continue
        nrm = nrm if nrm @ n_prior > 0 else -nrm
        ex = Rp[:, 0] - (Rp[:, 0] @ nrm) * nrm
        ex /= np.linalg.norm(ex)
        ey = np.cross(nrm, ex)
        B = np.column_stack([ex, ey])
        q = (C - o3) @ B
        circ = _circle3(*q[list(tri)])
        if circ is None or abs(circ[1] - pcd) > P["circle_tol"]:
            continue
        o2 = circ[0]
        h = (C - o3) @ nrm
        # 원 위 후보들의 각도 -> 사전 회전각 기준으로 패턴에 대응
        rel = q - o2
        on_circle = (np.abs(np.linalg.norm(rel, axis=1) - pcd) < P["match_tol"]) & (np.abs(h) < P["match_tol"])
        score = int(on_circle.sum())
        if best is None or score > best[0]:
            best = (score, o3, nrm, B, o2, on_circle)
    if best is None or best[0] < P["min_nuts"]:
        out["reason"] = "no %d candidates lie on the %.1f mm bolt circle" % (P["min_nuts"], pcd * 2000)
        out["debug"] = _draw(rgb, depth, K, T_cam_world, x0, y0, rejected, lab, cands, None, None) if debug else None
        return out
    _, o3, nrm, B, o2, on_circle = best
    # 대응: 패턴 회전각 phi 를 사전값(0) 근처에서 추정하고 가장 가까운 패턴 점에 붙인다 (대칭이라 +-36도 안에서 유일)
    q = (C[on_circle] - o3) @ B - o2
    ang_meas = np.arctan2(q[:, 1], q[:, 0])
    ang_pat = np.arctan2(pattern[:, 1], pattern[:, 0])
    step = 2 * math.pi / len(pattern)
    phi = float(np.angle(np.mean(np.exp(1j * len(pattern) * (ang_meas[:, None] - ang_pat[None, :])[:, 0]))) / len(pattern))
    # ---- 4. 허브 축 다듬기: 너트 사이 장착면 점들에 평면 맞춤
    idx_c = np.flatnonzero(on_circle)
    for _ in range(2):
        center_w = o3 + B @ o2
        Pm = K.backproject(uu[valid], vv[valid], D[valid])
        Pw = _apply(T_world_cam, Pm)
        rel = Pw - center_w
        hgt = rel @ nrm
        rad = np.linalg.norm(rel - np.outer(hgt, nrm), axis=1)
        near_nut = np.zeros(len(Pw), bool)
        for c in C[idx_c]:
            near_nut |= np.linalg.norm(Pw - c, axis=1) < P["pad_nut_clear"]
        h_top = float(np.median((C[idx_c] - center_w) @ nrm))
        sel = (rad > P["pad_band"][0]) & (rad < P["pad_band"][1]) & ~near_nut & (np.abs(hgt - (h_top - P["pad_below_top"])) < 0.004)
        if sel.sum() < 200:
            break
        pc, pn, keep = _plane_fit_robust(Pw[sel])
        nrm = pn if pn @ nrm > 0 else -pn
        out["pad_points"] = int(keep.sum())
        # 새 축으로 평면 좌표계를 다시 만들고 중심을 다시 맞춘다
        ex = Rp[:, 0] - (Rp[:, 0] @ nrm) * nrm
        ex /= np.linalg.norm(ex)
        B = np.column_stack([ex, np.cross(nrm, ex)])
        top_plane = pc + nrm * P["pad_below_top"]               # 너트 윗면 평면 위의 한 점
        o3 = top_plane
        q_all = (C[idx_c] - o3) @ B
        o2 = _circle_fit_fixed(q_all, pcd, (center_w - o3) @ B)
    # ---- 5. 패턴 강체 맞춤 (2D 회전 + 이동, 대응은 가장 가까운 패턴 점)
    q = (C[idx_c] - o3) @ B
    for _ in range(3):
        pat_now = (_rot2(phi) @ pattern.T).T + o2
        assign = [int(np.argmin(np.linalg.norm(pat_now - p, axis=1))) for p in q]
        if len(set(assign)) < len(assign):
            break
        phi, o2 = _rigid2d(pattern[assign], q)
    pat_now = (_rot2(phi) @ pattern.T).T + o2
    res = np.linalg.norm(pat_now[assign] - q, axis=1)
    rms = float(np.sqrt(np.mean(res ** 2)))
    keep_pts = res < P["match_tol"]
    if keep_pts.sum() < P["min_nuts"]:
        out["reason"] = "only %d nuts fit the pattern" % keep_pts.sum()
        return out
    # 휠 자세: 로컬 x 축 = ex 를 phi 만큼 돌린 것, 원점 = 너트 윗면 중심에서 축 방향으로 z_top_local 만큼 안쪽
    R2 = _rot2(phi)
    ex_w, ey_w = B @ R2[:, 0], B @ R2[:, 1]
    T = np.eye(4)
    T[:3, :3] = np.column_stack([ex_w, ey_w, nrm])
    T[:3, 3] = o3 + B @ o2 - nrm * z_top_local
    nuts_w = _apply(T, nut_local)
    out.update(ok=rms <= P["max_rms"], reason="" if rms <= P["max_rms"] else "pattern rms %.1f mm too large" % (rms * 1000),
               T_wheel=T, nuts_world=nuts_w, rms=rms, n_found=int(keep_pts.sum()),
               detected={int(a): C[idx_c][j] for j, a in enumerate(assign) if keep_pts[j]})
    if debug:
        out["debug"] = _draw(rgb, depth, K, T_cam_world, x0, y0, rejected, lab, cands, T, nut_local, z_top_local - float(np.mean(nut_local[:, 2])))
    return out


def _circle_fit_fixed(q, r, o_init, iters=10):
    """반지름 r 고정 원의 중심 (Gauss-Newton)"""
    o = np.array(o_init, dtype=float)
    for _ in range(iters):
        d = q - o
        L = np.linalg.norm(d, axis=1)
        J = -d / L[:, None]
        e = L - r
        step = np.linalg.lstsq(J, -e, rcond=None)[0]
        o += step
        if np.linalg.norm(step) < 1e-7:
            break
    return o


# ----------------------------------------------------------------------------- 타이어 찾기 (트랙 이동 중 차 옆면을 보며)
def find_tire(rgb, depth, K, T_world_cam, expect_dist, rim_radius=0.2159, tire_radius=0.316, debug=False):
    """차 옆면 영상에서 휠 하나를 찾는다: 밝은 림 원(HoughCircles) + 바깥쪽 검은 고무 띠 + 깊이로 실제 크기 확인.
    expect_dist: 카메라에서 휠 면까지 대략 거리 [m] (반지름 범위를 정하는 데만 씀).
    돌려줌: dict(ok, center_world (타이어 옆면 높이의 휠 중심), radius_m, score, debug) 또는 ok=False"""
    out = {"ok": False, "center_world": None, "radius_m": None, "score": 0.0, "debug": None, "n_circles": 0}
    gray = cv2.medianBlur(cv2.cvtColor(np.ascontiguousarray(rgb[..., :3]).astype(np.uint8), cv2.COLOR_RGB2GRAY), 5)
    r0 = rim_radius * K.fx / expect_dist
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=r0 * 1.5, param1=120, param2=30,
                               minRadius=int(r0 * 0.7), maxRadius=int(r0 * 1.4))
    H, W = gray.shape
    best = None
    if circles is not None:
        out["n_circles"] = int(circles.shape[1])
        for u, v, r in circles[0]:
            R = int(1.4 * r) + 2                                    # 원 주변 창만 계산 (속도)
            x0, x1, y0, y1 = max(0, int(u) - R), min(W, int(u) + R), max(0, int(v) - R), min(H, int(v) + R)
            vv, uu = np.mgrid[y0:y1, x0:x1]
            rr = np.hypot(uu - u, vv - v)
            g, dw = gray[y0:y1, x0:x1], depth[y0:y1, x0:x1]
            rim = (rr > 0.88 * r) & (rr < 1.0 * r)                  # 림 플랜지 (밝은 합금)
            tire = (rr > 1.12 * r) & (rr < 1.38 * r)                # 타이어 옆면 (검은 고무)
            if tire.sum() < 50 or rim.sum() < 50:
                continue
            contrast = float(g[rim].mean() - g[tire].mean())
            dt = dw[tire]
            dt = dt[np.isfinite(dt) & (dt > 0)]
            if contrast < 40 or dt.size < 50:
                continue
            z = float(np.median(dt))
            r_m = float(r) * z / K.fx
            if abs(r_m - rim_radius) > 0.25 * rim_radius or abs(z - expect_dist) > 0.5 * expect_dist:
                continue
            if best is None or contrast > best[0]:
                best = (contrast, u, v, r, z, r_m)
    if best is not None:
        contrast, u, v, r, z, r_m = best
        c_cam = K.backproject(u, v, z)
        out.update(ok=True, center_world=_apply(T_world_cam, c_cam), radius_m=r_m, score=contrast, pixel=(float(u), float(v), float(r)))
    if debug:
        img = cv2.cvtColor(np.ascontiguousarray(rgb[..., :3]).astype(np.uint8), cv2.COLOR_RGB2BGR)
        if circles is not None:
            for u, v, r in circles[0]:
                cv2.circle(img, (int(u), int(v)), int(r), (0, 220, 255), 1, cv2.LINE_AA)
        if best is not None:
            cv2.circle(img, (int(best[1]), int(best[2])), int(best[3]), (60, 255, 60), 3, cv2.LINE_AA)
            cv2.drawMarker(img, (int(best[1]), int(best[2])), (60, 255, 60), cv2.MARKER_CROSS, 30, 3, cv2.LINE_AA)
            cv2.putText(img, "tire  r=%.0f mm  d=%.2f m" % (best[5] * 1000, best[4]), (int(best[1]) - 80, int(best[2] - best[3]) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 255, 60), 2, cv2.LINE_AA)
        out["debug"] = img
    return out


# ----------------------------------------------------------------------------- 시각화 (발표·디버그용)
def _draw(rgb, depth, K, T_cam_world, x0, y0, rejected, lab, cands, T_wheel, nut_local, top_offset=0.015):
    if rgb is not None:
        img = cv2.cvtColor(np.ascontiguousarray(rgb[..., :3]).astype(np.uint8), cv2.COLOR_RGB2BGR)
    else:
        d = np.nan_to_num(depth, nan=0, posinf=0)
        img = cv2.applyColorMap(cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8), cv2.COLORMAP_JET)
    for sel, _ in rejected:                                     # 버린 후보: 빨간 점
        ys, xs = np.nonzero(sel)
        img[ys + y0, xs + x0] = (0.5 * img[ys + y0, xs + x0] + (0, 0, 127)).astype(np.uint8)
    for c in cands:                                             # 후보: 노란 원
        u, v = K.project(_apply(T_cam_world, c))
        cv2.circle(img, (int(round(u)), int(round(v))), 9, (0, 220, 255), 1, cv2.LINE_AA)
    if T_wheel is not None:
        tops = _apply(T_wheel, nut_local + np.array([0, 0, top_offset]))
        for i, p in enumerate(tops):                            # 맞춘 패턴: 초록 십자 + 번호
            u, v = K.project(_apply(T_cam_world, p))
            cv2.drawMarker(img, (int(round(u)), int(round(v))), (60, 255, 60), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
            cv2.putText(img, str(i + 1), (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 255, 60), 2, cv2.LINE_AA)
        c = _apply(T_wheel, [0, 0, float(np.mean(nut_local[:, 2])) + top_offset])
        a = _apply(T_wheel, [0, 0, float(np.mean(nut_local[:, 2])) + top_offset + 0.08])
        uc, vc = K.project(_apply(T_cam_world, c))
        ua, va = K.project(_apply(T_cam_world, a))
        cv2.circle(img, (int(uc), int(vc)), 4, (255, 120, 0), -1, cv2.LINE_AA)
        cv2.arrowedLine(img, (int(uc), int(vc)), (int(ua), int(va)), (255, 120, 0), 2, cv2.LINE_AA)
    h_roi = max(1, lab.shape[0]) if lab is not None else 1
    w_roi = max(1, lab.shape[1]) if lab is not None else 1
    cv2.rectangle(img, (x0, y0), (x0 + w_roi, y0 + h_roi), (200, 200, 200), 1, cv2.LINE_AA)
    legend = [("ROI (layout prior)", (200, 200, 200)), ("rejected blob (spoke/edge)", (0, 0, 200)),
              ("nut candidate", (0, 220, 255)), ("fitted nut (number)", (60, 255, 60)), ("hub centre + axis", (255, 120, 0))]
    for i, (txt, col) in enumerate(legend):
        y = 22 + 20 * i
        cv2.rectangle(img, (8, y - 11), (20, y + 1), col, -1)
        cv2.putText(img, txt, (26, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, (26, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img


# ----------------------------------------------------------------------------- 자체 시험 (Isaac 없이: 합성 깊이 영상)
def _selftest():
    """너트 5개 + 장착면 + 스포크 형상을 깊이 영상으로 그려서 알고리즘만 시험한다."""
    rng = np.random.default_rng(0)
    W_, H_ = 1280, 800
    K = Intrinsics.from_usd(1.93, 3.896, W_, H_)
    pcd, pad_z1 = 0.05715, 0.0675
    nut_local = np.array([[pcd * math.cos(math.radians(90 + 72 * i)), pcd * math.sin(math.radians(90 + 72 * i)), pad_z1 + 0.005] for i in range(5)])
    # 실제 휠: 사전값에서 12 mm / 8 mm / -5 mm 이동, 0.6 도 기울어짐, 4 도 회전
    from_axis = lambda ax, a: cv2.Rodrigues(np.asarray(ax, float) * math.radians(a))[0]
    T_prior = np.eye(4)
    T_true = np.eye(4)
    T_true[:3, :3] = from_axis([1, 0, 0], 0.6) @ from_axis([0, 0, 1], 4.0)
    T_true[:3, 3] = [0.012, 0.008, -0.005]
    T_cam = np.eye(4)
    T_cam[:3, 3] = [0.0, 0.0, pad_z1 + 0.55]                    # 휠 바깥 0.55 m 에서 -Z 로 봄
    # 레이 캐스팅 대신: 각 픽셀의 광선과 휠 로컬 높이맵 교점 (작은 기울기라 2회 반복으로 충분)
    vv, uu = np.mgrid[0:H_, 0:W_].astype(float)
    win = (slice(H_ // 2 - 220, H_ // 2 + 220), slice(W_ // 2 - 220, W_ // 2 + 220))   # 휠 중심부만 그린다 (속도)
    vv, uu = vv[win], uu[win]
    ray = np.stack([(uu - K.cx) / K.fx, -(vv - K.cy) / K.fy, -np.ones_like(uu)], axis=-1)
    Tinv = np.linalg.inv(T_true)
    o_l = _apply(Tinv, T_cam[:3, 3])
    r_l = ray @ (Tinv[:3, :3] @ T_cam[:3, :3]).T

    def height(x, y):
        r = np.hypot(x, y)
        h = np.full(x.shape, -0.005)                            # 디스크
        h = np.where((r > 0.0336) & (r < 0.092), pad_z1, h)     # 장착면
        a = np.degrees(np.arctan2(y, x))
        for k in range(10):                                     # 스포크
            ak = 90 + 36 * k + 18
            d = np.abs(((a - ak + 180) % 360) - 180)
            h = np.where((r > 0.088) & (r < 0.2) & (d * np.pi / 180 * r < 0.012), pad_z1 - 0.008 + (r - 0.088) * 0.2, h)
        for p in nut_local:
            h = np.where(np.hypot(x - p[0], y - p[1]) < 0.0105, pad_z1 + 0.020, h)
        return h

    # 광선 행진: 위에서부터 0.5 mm 씩 내려가며 처음으로 높이맵 아래로 들어가는 지점
    t = np.full(uu.shape, np.nan)
    for z in np.arange(pad_z1 + 0.025, -0.01, -0.0005):
        tz = (o_l[2] - z) / -r_l[..., 2]
        x, y = o_l[0] + tz * r_l[..., 0], o_l[1] + tz * r_l[..., 1]
        t = np.where(np.isnan(t) & (height(x, y) >= z), tz, t)
    depth = np.zeros((H_, W_), np.float32)
    depth[win] = np.nan_to_num(t, nan=0.0)                     # ray z = -1 -> 깊이 = t
    depth += rng.normal(0, 0.001, depth.shape).astype(np.float32)
    res = find_wheel(depth, K, T_cam, T_prior, nut_local, debug=True)
    err = np.linalg.norm(res["nuts_world"] - _apply(T_true, nut_local), axis=1) if res["nuts_world"] is not None else None
    tilt = math.degrees(math.acos(min(1.0, float(res["T_wheel"][:3, 2] @ T_true[:3, 2])))) if res["T_wheel"] is not None else None
    print("selftest ok=%s found=%d rms=%s nut error mm=%s axis error deg=%s %s" % (
        res["ok"], res["n_found"], None if res["rms"] is None else round(res["rms"] * 1000, 2),
        None if err is None else np.round(err * 1000, 2), None if tilt is None else round(tilt, 3), res["reason"]))
    return res


if __name__ == "__main__":
    import sys
    r = _selftest()
    if len(sys.argv) > 1 and r["debug"] is not None:
        cv2.imwrite(sys.argv[1], r["debug"])
