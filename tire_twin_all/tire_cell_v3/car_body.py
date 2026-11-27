"""
차종별 차체 생성 (제원 치수 + 디자인 특징으로 만드는 절차적 모델, 실차 CAD 아님)

차 좌표계: Z up, 앞 = -Y, 왼쪽 = +X, z 0 = 공차 상태의 지면. s = 차 앞끝에서 뒤로 잰 거리 [m].
단면(반쪽)을 s 방향으로 이어 붙여 차체를 만들고, 휠 아치는 원 밖으로 밀어내서 뚫는다.
그릴·램프 같은 얼굴 요소는 앞/뒤 곡면을 따라가는 얇은 판(데칼)으로 붙인다.
"""
import math

import numpy as np

# ------------------------------------------------------------------ 차종별 디자인 값
#  profile: (s, 높이) 지붕선 제어점. s 는 m, 음수면 뒤끝 기준
#  front/rear: 평면에서 본 모서리 둥글림 (b = 모서리가 뒤로 물러나는 깊이, p = 클수록 각진 얼굴)
#  decals: (앞/뒤, 재질, [(x/w, z) 네 꼭짓점: 안쪽아래, 바깥아래, 바깥위, 안쪽위], 좌우대칭 여부)
STYLES = {
    "avante_cn7": {   # 낮고 날카로운 쐐기형, 패스트백, 그릴과 이어진 헤드램프, H 형 리어램프
        "paint": (0.20, 0.30, 0.46), "trim": (0.05, 0.05, 0.05), "trim_metal": 0.2,
        "h_nose": 0.66, "cowl": (0.50, 0.92), "roof_front": 0.82, "roof_rear": (-1.50, 1.375), "deck": (-0.44, 1.05), "h_tail": 0.99,
        "front": (0.40, 2.5), "rear": (0.26, 3.2), "tumble": 0.34, "spokes": (10, 0.022),
        "decals": [
            ("F", "grille", [(0.0, 0.31), (0.56, 0.31), (0.66, 0.55), (0.0, 0.55)], True),
            ("F", "lamp", [(0.50, 0.555), (0.93, 0.60), (0.93, 0.645), (0.50, 0.615)], True),
            ("F", "grille", [(0.62, 0.24), (0.88, 0.27), (0.88, 0.40), (0.68, 0.36)], True),
            ("F", "plate", [(0.0, 0.36), (0.20, 0.36), (0.20, 0.45), (0.0, 0.45)], True),
            ("R", "tail", [(0.0, 0.905), (0.93, 0.905), (0.93, 0.935), (0.0, 0.935)], True),
            ("R", "tail", [(0.74, 0.72), (0.93, 0.76), (0.93, 0.905), (0.80, 0.905)], True),
            ("R", "dark", [(0.0, 0.25), (0.70, 0.25), (0.62, 0.36), (0.0, 0.36)], True),
            ("R", "plate", [(0.0, 0.50), (0.20, 0.50), (0.20, 0.60), (0.0, 0.60)], True),
        ]},
    "k5_dl3": {       # 긴 보닛 + 패스트백, 넓은 타이거 노즈 그릴, 심장박동 모양 주간등, 지붕선 크롬 몰딩
        "paint": (0.47, 0.50, 0.52), "trim": (0.85, 0.85, 0.87), "trim_metal": 1.0,
        "h_nose": 0.68, "cowl": (0.56, 0.94), "roof_front": 0.84, "roof_rear": (-1.62, 1.40), "deck": (-0.36, 1.06), "h_tail": 1.01,
        "front": (0.38, 2.6), "rear": (0.30, 3.0), "tumble": 0.36, "spokes": (5, 0.050),
        "decals": [
            ("F", "grille", [(0.0, 0.47), (0.46, 0.47), (0.56, 0.60), (0.0, 0.585)], True),
            ("F", "lamp", [(0.52, 0.575), (0.92, 0.60), (0.92, 0.64), (0.58, 0.625)], True),
            ("F", "amber", [(0.55, 0.50), (0.70, 0.555), (0.70, 0.575), (0.55, 0.52)], True),
            ("F", "amber", [(0.70, 0.555), (0.84, 0.50), (0.84, 0.52), (0.70, 0.575)], True),
            ("F", "grille", [(0.0, 0.22), (0.74, 0.22), (0.66, 0.38), (0.0, 0.38)], True),
            ("F", "plate", [(0.0, 0.26), (0.20, 0.26), (0.20, 0.35), (0.0, 0.35)], True),
            ("R", "tail", [(0.0, 0.89), (0.80, 0.89), (0.80, 0.915), (0.0, 0.915)], True),
            ("R", "tail", [(0.78, 0.80), (0.94, 0.76), (0.94, 0.915), (0.78, 0.915)], True),
            ("R", "dark", [(0.0, 0.24), (0.76, 0.24), (0.70, 0.37), (0.0, 0.37)], True),
            ("R", "trim", [(0.40, 0.27), (0.66, 0.27), (0.64, 0.33), (0.40, 0.33)], True),
            ("R", "plate", [(0.0, 0.62), (0.20, 0.62), (0.20, 0.72), (0.0, 0.72)], True),
        ]},
    "grandeur_gn7": {  # 각지고 높은 얼굴, 끊김 없는 수평 주간등, 큰 사각 그릴, 긴 지붕의 3박스, 일자 리어램프
        "paint": (0.90, 0.90, 0.88), "trim": (0.85, 0.85, 0.87), "trim_metal": 1.0,
        "h_nose": 0.79, "cowl": (0.56, 1.01), "roof_front": 0.80, "roof_rear": (-1.36, 1.43), "deck": (-0.58, 1.11), "h_tail": 1.07,
        "front": (0.24, 3.8), "rear": (0.20, 4.2), "tumble": 0.26, "spokes": (18, 0.014),
        "decals": [
            ("F", "lamp", [(0.0, 0.735), (0.97, 0.735), (0.97, 0.755), (0.0, 0.755)], True),
            ("F", "grille", [(0.0, 0.27), (0.80, 0.27), (0.80, 0.70), (0.0, 0.70)], True),
            ("F", "lamp", [(0.62, 0.50), (0.78, 0.50), (0.78, 0.64), (0.62, 0.64)], True),
            ("F", "plate", [(0.0, 0.34), (0.20, 0.34), (0.20, 0.43), (0.0, 0.43)], True),
            ("R", "tail", [(0.0, 0.90), (0.97, 0.90), (0.97, 0.922), (0.0, 0.922)], True),
            ("R", "amber", [(0.60, 0.42), (0.86, 0.42), (0.86, 0.45), (0.60, 0.45)], True),
            ("R", "dark", [(0.0, 0.24), (0.84, 0.24), (0.84, 0.34), (0.0, 0.34)], True),
            ("R", "plate", [(0.0, 0.50), (0.20, 0.50), (0.20, 0.60), (0.0, 0.60)], True),
        ]},
    "k9_rj": {        # 후륜구동 비례(짧은 앞 오버행, 긴 보닛), 큰 크롬 그릴, 세로형 램프, 정통 3박스
        "paint": (0.035, 0.035, 0.05), "trim": (0.88, 0.88, 0.90), "trim_metal": 1.0,
        "h_nose": 0.80, "cowl": (0.84, 1.03), "roof_front": 0.80, "roof_rear": (-1.42, 1.46), "deck": (-0.66, 1.11), "h_tail": 1.05,
        "front": (0.30, 3.0), "rear": (0.26, 3.6), "tumble": 0.28, "spokes": (14, 0.018),
        "decals": [
            ("F", "trim", [(0.0, 0.385), (0.40, 0.385), (0.43, 0.745), (0.0, 0.745)], True),
            ("F", "grille", [(0.0, 0.41), (0.36, 0.41), (0.39, 0.72), (0.0, 0.72)], True),
            ("F", "lamp", [(0.50, 0.60), (0.90, 0.63), (0.90, 0.73), (0.50, 0.71)], True),
            ("F", "grille", [(0.50, 0.24), (0.86, 0.26), (0.86, 0.36), (0.50, 0.36)], True),
            ("F", "trim", [(0.0, 0.25), (0.44, 0.25), (0.44, 0.27), (0.0, 0.27)], True),
            ("R", "tail", [(0.60, 0.66), (0.93, 0.70), (0.93, 0.90), (0.66, 0.90)], True),
            ("R", "trim", [(0.0, 0.845), (0.66, 0.845), (0.66, 0.865), (0.0, 0.865)], True),
            ("R", "trim", [(0.0, 0.26), (0.80, 0.26), (0.80, 0.28), (0.0, 0.28)], True),
            ("R", "plate", [(0.0, 0.56), (0.20, 0.56), (0.20, 0.66), (0.0, 0.66)], True),
        ]},
}
DECAL_COLORS = {"grille": ((0.02, 0.02, 0.02), 0.6, 0.45), "lamp": ((0.95, 0.96, 1.0), 0.0, 0.08), "tail": ((0.75, 0.02, 0.02), 0.0, 0.15),
                "amber": ((1.0, 0.55, 0.05), 0.0, 0.15), "dark": ((0.03, 0.03, 0.03), 0.2, 0.6), "plate": ((0.93, 0.93, 0.90), 0.0, 0.5)}
N_SIDE = 9          # 옆면 세로 분할 수


def _smooth(y, n):
    k = np.exp(-0.5 * (np.arange(-3 * n, 3 * n + 1) / n) ** 2)
    return np.convolve(np.pad(y, 3 * n, mode="edge"), k / k.sum(), mode="valid")


class Body:
    def __init__(self, S, key, tire_r, tire_hw, droop):
        self.S, self.st = S, STYLES[key]
        st = self.st
        self.L, self.w, self.H, self.gc = S["length"], S["width"] / 2, S["height"], S["ground_clearance"]
        self.FO = (S["length"] - S["wheelbase"]) * S["front_overhang_fraction"]
        self.y0 = -S["wheelbase"] / 2 - self.FO
        self.s_fa, self.s_ra = self.FO, self.FO + S["wheelbase"]
        self.s_cowl = self.s_fa + st["cowl"][0]
        self.s_rf = self.s_cowl + st["roof_front"]
        self.s_rr, self.s_deck = self.L + st["roof_rear"][0], self.L + st["deck"][0]
        cp = [(0.0, st["h_nose"]), (0.10, st["h_nose"] + 0.035), (self.s_cowl, st["cowl"][1]), (self.s_rf, self.H - 0.012), ((self.s_rf + self.s_rr) / 2 - 0.15, self.H),
              (self.s_rr, st["roof_rear"][1]), (self.s_deck, st["deck"][1]), (self.L - 0.06, st["h_tail"] + 0.012), (self.L, st["h_tail"])]
        self._sg = np.arange(0.0, self.L + 1e-6, 0.005)
        self._top = _smooth(np.interp(self._sg, [c[0] for c in cp], [c[1] for c in cp]), 6)
        self._belt = (self.s_cowl, st["cowl"][1] - 0.035, self.s_deck, st["deck"][1] - 0.02)
        self.x_in = min(S["track_front"], S["track_rear"]) / 2 - tire_hw - 0.05           # 휠 하우스 안쪽 벽
        z_ax = tire_r - 0.012
        self.arch_r = tire_r + 0.07
        self.arches = [(-S["wheelbase"] / 2, z_ax + 0.01), (S["wheelbase"] / 2, z_ax + 0.01)]     # 완전 압축(범프)에서도 타이어가 닿지 않는 위치

    # ---- 형상 함수
    def top(self, s):
        return float(np.interp(s, self._sg, self._top))

    def bottom(self, s):
        f, r = max(0.0, 1 - s / 0.55), max(0.0, 1 - (self.L - s) / 0.75)
        return self.gc + 0.01 + 0.12 * f ** 2 + 0.19 * r ** 2

    def half_width(self, s):
        (bf, pf), (br, pr) = self.st["front"], self.st["rear"]
        k = 1.0
        if s < bf:
            k = min(k, (max(s, 0.0) / bf) ** (1 / pf))
        if self.L - s < br:
            k = min(k, (max(self.L - s, 0.0) / br) ** (1 / pr))
        return self.w * k

    def face_y(self, end, x):
        """앞('F')/뒤('R') 곡면의 y (차 좌표). x = 좌우 위치"""
        f = min(abs(x) / self.w, 1.0)
        if end == "F":
            return self.y0 + self.st["front"][0] * f ** self.st["front"][1]
        return self.y0 + self.L - self.st["rear"][0] * f ** self.st["rear"][1]

    def shoulder(self, s):
        s0, z0, s1, z1 = self._belt
        return min(float(np.interp(s, [s0, s1], [z0, z1])), self.top(s) - 0.02)

    def section(self, s, n_side):
        """반쪽 단면 (x >= 0) 점 목록 [(x, z)]: 바닥 중앙 -> 옆면 -> 지붕 중앙"""
        w, zb, zt, zs = self.half_width(s), self.bottom(s), self.top(s), self.shoulder(s)
        k = w / self.w
        pts = [(0.0, zb), (min(self.x_in * k, max(w - 0.09, 0.0)), zb), (max(w - 0.07, 0.0), zb), (w, zb + 0.07)]
        for i in range(1, n_side + 1):
            t = i / n_side
            pts.append((w - 0.022 * t ** 2 * k, zb + 0.07 + (zs - zb - 0.07) * t))
        gh = zt - zs
        xe = max(w - 0.045 * k - self.st["tumble"] * gh, 0.0)
        pts += [(xe, zt - min(0.035, 0.3 * gh)), (0.62 * xe, zt - 0.006), (0.0, zt)]
        return pts

    def stations(self, step):
        (bf, pf), (br, pr) = self.st["front"], self.st["rear"]
        fr = [0.0, 0.2, 0.35, 0.5, 0.6, 0.7, 0.78, 0.85, 0.9, 0.95, 0.98, 1.0] if step < 0.06 else [0.0, 0.4, 0.7, 0.9, 1.0]
        s = [bf * f ** pf for f in fr] + [self.L - br * f ** pr for f in fr]
        s += list(np.arange(max(bf, 0.0) + step, self.L - br, step)) + [self.s_cowl, self.s_rf, self.s_rr, self.s_deck]
        for cy, _ in self.arches:                       # 아치 가장자리가 매끈하도록 촘촘히
            sc = cy - self.y0
            s += list(np.arange(sc - self.arch_r - 0.03, sc + self.arch_r + 0.031, min(step, 0.03) if step < 0.06 else step / 2))
        s = sorted(set(round(float(v), 4) for v in s if 0.0 <= v <= self.L))
        out = [s[0]]
        for v in s[1:]:
            if v - out[-1] > 0.004:
                out.append(v)
        return out

    def _arch_push(self, P):
        """휠 아치 원 안에 들어온 옆면 점을 원 위로 밀어낸다. 반환: 밀린 점 표시"""
        moved = np.zeros(len(P), dtype=bool)
        for cy, cz in self.arches:
            dy, dz = P[:, 1] - cy, P[:, 2] - cz
            r = np.hypot(dy, dz)
            inside = (r < self.arch_r) & (np.abs(P[:, 0]) >= self.x_in - 1e-6)
            for i in np.where(inside)[0]:
                zb = self.bottom(P[i, 1] - self.y0)
                a = math.atan2(dz[i], dy[i]) if r[i] > 1e-6 else math.pi / 2
                ny, nz = cy + self.arch_r * math.cos(a), cz + self.arch_r * math.sin(a)
                if nz < zb:                              # 아래쪽은 바닥 높이에서 원과 만나는 점으로
                    nz = zb
                    ny = cy + math.copysign(math.sqrt(max(self.arch_r ** 2 - (zb - cz) ** 2, 0.0)), dy[i] if abs(dy[i]) > 1e-6 else 1.0)
                P[i, 1], P[i, 2] = ny, nz
                moved[i] = True
        return moved

    def loft(self, step=0.03, n_side=N_SIDE):
        """반환: points (N,3), faces {paint, glass, trim} = 사각형 인덱스 목록"""
        S = self.stations(step)
        m = len(self.section(0.5, n_side))
        half = []
        for s in S:
            half.append([(x, self.y0 + s, z) for x, z in self.section(s, n_side)])
        H = np.array(half, dtype=float)                                   # (ns, m, 3)
        flat = H.reshape(-1, 3).copy()
        side_rows = np.zeros((len(S), m), dtype=bool)
        side_rows[:, 1:4 + n_side] = True
        idx = np.where(side_rows.reshape(-1))[0]
        sub = flat[idx]
        mv = self._arch_push(sub)
        flat[idx] = sub
        moved = np.zeros(len(flat), dtype=bool)
        moved[idx] = mv
        pts = np.concatenate([flat, flat * [-1, 1, 1]])
        n = len(flat)
        faces = {"paint": [], "glass": [], "trim": []}
        i_belt, i_edge = 3 + n_side, 4 + n_side                           # 벨트(옆면 마지막) / 지붕 가장자리
        s_b = (self.s_rf + self.s_rr) / 2 + 0.05                          # B 필러
        for i in range(len(S) - 1):
            sm = (S[i] + S[i + 1]) / 2
            gh = self.top(sm) - self.shoulder(sm)
            for j in range(m - 1):
                a, b, c, d = i * m + j, (i + 1) * m + j, (i + 1) * m + j + 1, i * m + j + 1
                if moved[a] and moved[b] and moved[c] and moved[d]:
                    continue                                              # 휠 아치 구멍
                kind = "paint"
                if j == i_belt and gh > 0.20:                             # 옆 유리 (필러 제외)
                    pillar = abs(sm - s_b) < 0.045 or sm < self.s_cowl + 0.30 or sm > self.s_rr + 0.10
                    kind = "paint" if pillar else "glass"
                elif j >= i_edge and (self.s_cowl + 0.03 < sm < self.s_rf - 0.02 or self.s_rr + 0.02 < sm < self.s_deck - 0.03):
                    kind = "glass"                                        # 앞/뒤 유리
                faces[kind].append((a, b, c, d))
                faces[kind].append((n + a, n + d, n + c, n + b))
        return pts, faces, S

    def trim_strip(self, S, n_side):
        """창문 위 몰딩: 지붕 가장자리를 따라가는 얇은 띠"""
        pts, quads = [], []
        ss = [s for s in S if self.s_cowl + 0.32 <= s <= self.s_rr + 0.10]
        for sgn in (1, -1):
            base = len(pts)
            for s in ss:
                x, z = self.section(s, n_side)[4 + n_side]
                pts += [(sgn * (x + 0.004), self.y0 + s, z - 0.006), (sgn * (x + 0.004), self.y0 + s, z + 0.010)]
            for i in range(len(ss) - 1):
                quads.append((base + 2 * i, base + 2 * i + 2, base + 2 * i + 3, base + 2 * i + 1))
        return pts, quads

    def decals(self):
        out = []
        for end, matname, corners, mirror in self.st["decals"]:
            for sgn in ((1, -1) if mirror else (1,)):
                pts, quads = [], []
                nx, nz = 10, 2
                for iu in range(nx + 1):
                    u = iu / nx
                    for iv in range(nz + 1):
                        v = iv / nz
                        (x0, z0), (x1, z1), (x2, z2), (x3, z3) = corners
                        xf = (x0 * (1 - u) + x1 * u) * (1 - v) + (x3 * (1 - u) + x2 * u) * v
                        z = (z0 * (1 - u) + z1 * u) * (1 - v) + (z3 * (1 - u) + z2 * u) * v
                        x = xf * self.w
                        # 차체는 낮은 곳에서 폭이 줄어드므로 그만큼 안쪽으로, 그리고 표면에서 8 mm 띄운다
                        y = self.face_y(end, x) + (-0.008 if end == "F" else 0.008)
                        pts.append((sgn * (x + 0.006 * xf), y, z))
                for iu in range(nx):
                    for iv in range(nz):
                        a = iu * (nz + 1) + iv
                        quads.append((a, a + nz + 1, a + nz + 2, a + 1))
                out.append((matname, pts, quads))
        return out

    def liners(self):
        """휠 하우스 안쪽 (검은 반원통 + 안쪽 벽): 아치 구멍으로 속이 들여다보이지 않게"""
        pts, quads = [], []
        for cy, cz in self.arches:
            for sgn in (1, -1):
                base = len(pts)
                n = 16
                for k in range(n + 1):
                    a = math.pi * k / n
                    y, z = cy + (self.arch_r + 0.004) * math.cos(a), cz + (self.arch_r + 0.004) * math.sin(a)
                    pts += [(sgn * self.x_in, y, z), (sgn * (self.w - 0.012), y, z), (sgn * self.x_in, y, cz - 0.02)]
                for k in range(n):
                    a = base + 3 * k
                    quads.append((a, a + 3, a + 4, a + 1))
                    quads.append((a + 2, a + 5, a + 3, a))
        return pts, quads

    def seams(self, n_side=N_SIDE):
        """문 경계선 (옆면을 따라가는 가는 띠 3줄) 과 문 손잡이 위치"""
        s_b = (self.s_rf + self.s_rr) / 2 + 0.05
        cuts = [self.s_cowl + 0.02, s_b, min(self.s_rr + 0.16, self.s_ra - 0.05)]
        pts, quads = [], []
        for sgn in (1, -1):
            for sc in cuts:
                sec = self.section(sc, n_side)[4:4 + n_side]
                base = len(pts)
                for x, z in sec:
                    pts += [(sgn * (x + 0.003), self.y0 + sc - 0.004, z), (sgn * (x + 0.003), self.y0 + sc + 0.004, z)]
                for i in range(len(sec) - 1):
                    quads.append((base + 2 * i, base + 2 * i + 1, base + 2 * i + 3, base + 2 * i + 2))
        handles = [(sgn, sc - 0.16, self.shoulder(sc - 0.16) - 0.075) for sgn in (1, -1) for sc in cuts[1:]]
        return pts, quads, handles
