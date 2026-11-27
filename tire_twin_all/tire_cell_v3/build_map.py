"""
정비소 맵 생성 (전부 치수 기반 더미, 외형만 - 충돌 없음)

  garage_map.usda        : 건물·랙·공구함·펜스·바닥선
  tire_cell_v3_map.usda  : tire_cell_v3.usda(로봇 2대 + 차량 + 리프트) 위에 맵을 얹은 씬

배치 기준 (world, m): 로봇 A = 원점, 로봇 B = (0.55, -1.25), 차량 = x 1.6~3.6 / y -3.8~0.9, 휠 내려놓는 자리 = (1.0, 1.05)
AMR 은 사용하지 않음. 나중에 넣을 수 있도록 주행로(x = -2.9 ~ -1.9)만 바닥에 점선으로 표시.

실행: python build_map.py <출력폴더>
"""
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else HERE
_argv = sys.argv
sys.argv = [sys.argv[0], OUT_DIR]
sys.path.insert(0, HERE)
import build_scene_v2 as b2  # noqa: E402

sys.argv = _argv
MAP = os.path.join(OUT_DIR, "garage_map.usda")
SCENE = os.path.join(OUT_DIR, "tire_cell_v3_map.usda")
L = "/Garage/Looks"

X0, X1, Y0, Y1, H = -6.5, 11.0, -10.0, 8.5, 4.6        # 실내 범위와 벽 높이
WT = 0.2                                              # 벽 두께


def mat(stage, name, color, metallic=0.0, rough=0.6, opacity=1.0):
    return b2.material(stage, name, color, metallic, rough, opacity, root=L)


def tire(stage, path, pos, axis="Z"):
    """보관용 타이어 (외형만): 지름 632, 폭 225"""
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if axis == "Y":
        x.AddRotateXOp().Set(90.0)
    elif axis == "X":
        x.AddRotateYOp().Set(90.0)
    b2.lathe(stage, path + "/rubber", [(0.216, -0.096), (0.236, -0.108), (0.270, -0.1125), (0.300, -0.108), (0.316, -0.09), (0.316, 0.09),
                                       (0.300, 0.108), (0.270, 0.1125), (0.236, 0.108), (0.216, 0.096)], mat(stage, "tire", (0.035, 0.035, 0.035), 0.0, 0.9), n=28)


def rack(stage, root, origin, length, levels, label_color, with_rim):
    """타이어 랙: 가로 length, 깊이 0.75. 타이어를 세워서(축 = 랙 길이 방향) 줄지어 보관"""
    ox, oy = origin
    steel = mat(stage, "rack_steel", (0.15, 0.2, 0.5), 0.6, 0.5)
    beam = mat(stage, "rack_beam", (0.9, 0.45, 0.05), 0.4, 0.5)
    alloy = mat(stage, "rack_rim", (0.75, 0.76, 0.78), 1.0, 0.3)
    depth, top = 0.75, 0.25 + 0.80 * levels
    for i, px in enumerate((0.0, length / 2, length)):
        for j, py in enumerate((0.0, depth)):
            b2.box(stage, f"{root}/post_{i}_{j}", (0.06, 0.06, top), (ox + px, oy + py, top / 2), steel)
    b2.box(stage, f"{root}/sign", (length * 0.5, 0.02, 0.18), (ox + length / 2, oy - 0.02, top + 0.12), mat(stage, "sign_" + root.split("/")[-1], label_color, 0.0, 0.6))
    n = int((length - 0.2) // 0.26)
    for lv in range(levels):
        z = 0.25 + 0.80 * lv
        for j, py in enumerate((0.12, depth - 0.12)):
            b2.box(stage, f"{root}/beam_{lv}_{j}", (length, 0.05, 0.08), (ox + length / 2, oy + py, z), beam)
        for k in range(n):
            tp = f"{root}/tire_{lv}_{k}"
            tire(stage, tp, (ox + 0.2 + 0.26 * k, oy + depth / 2, z + 0.04 + 0.316 - 0.035), axis="X")
            if with_rim:
                b2.lathe(stage, tp + "/rim", [(0.02, -0.01), (0.215, -0.01), (0.215, 0.01), (0.02, 0.01)], alloy, n=20)


def build_map():
    if os.path.exists(MAP):
        os.remove(MAP)
    b2.MATS.clear()
    stage = Usd.Stage.CreateNew(MAP)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    G = "/Garage"
    root = UsdGeom.Xform.Define(stage, G)
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("source", "parametric dummy garage (visual only, no collision)")
    floor = mat(stage, "floor_epoxy", (0.36, 0.38, 0.40), 0.0, 0.35)
    wall = mat(stage, "wall", (0.82, 0.83, 0.80), 0.0, 0.8)
    wall_lo = mat(stage, "wall_dado", (0.25, 0.32, 0.42), 0.0, 0.6)
    col = mat(stage, "column", (0.6, 0.6, 0.6), 0.0, 0.7)
    yellow = mat(stage, "line_yellow", (0.95, 0.78, 0.05), 0.0, 0.5)
    white = mat(stage, "line_white", (0.92, 0.92, 0.92), 0.0, 0.5)
    blue = mat(stage, "line_blue", (0.1, 0.45, 0.85), 0.0, 0.5)
    red = mat(stage, "tool_red", (0.72, 0.06, 0.06), 0.3, 0.4)
    dark = mat(stage, "dark", (0.12, 0.12, 0.13), 0.3, 0.5)
    steel = mat(stage, "steel", (0.55, 0.56, 0.58), 0.9, 0.35)
    wood = mat(stage, "bench_top", (0.55, 0.38, 0.2), 0.0, 0.6)
    glass = mat(stage, "panel_clear", (0.7, 0.85, 0.9), 0.0, 0.1, opacity=0.25)
    cab = mat(stage, "cabinet", (0.86, 0.86, 0.84), 0.2, 0.5)
    shutter = mat(stage, "shutter", (0.68, 0.7, 0.72), 0.7, 0.4)

    # ---------- 바닥 + 벽 ----------
    cxm, cym = (X0 + X1) / 2, (Y0 + Y1) / 2
    b2.box(stage, G + "/floor", (X1 - X0, Y1 - Y0, 0.004), (cxm, cym, 0.002), floor)
    # 뒷벽(x = X1), 오른쪽 벽(y = Y1): 전체 높이. 아래 1.2 m 는 짙은 색 띠
    for name, size, c in (("wall_back", (WT, Y1 - Y0 + 2 * WT, H), (X1 + WT / 2, cym, H / 2)),
                          ("wall_right", (X1 - X0, WT, H), (cxm, Y1 + WT / 2, H / 2))):
        b2.box(stage, f"{G}/{name}", size, c, wall)
    b2.box(stage, G + "/dado_back", (0.01, Y1 - Y0, 1.2), (X1 - 0.005, cym, 0.6), wall_lo)
    b2.box(stage, G + "/dado_right", (X1 - X0, 0.01, 1.2), (cxm, Y1 - 0.005, 0.6), wall_lo)
    # 왼쪽 벽(y = Y0): 차량 출입 셔터 (폭 3.6, 높이 3.2) 가 있는 벽
    dx0, dx1, dh = 0.8, 4.4, 3.2
    b2.box(stage, G + "/wall_left_a", (dx0 - X0, WT, H), ((X0 + dx0) / 2, Y0 - WT / 2, H / 2), wall)
    b2.box(stage, G + "/wall_left_b", (X1 - dx1, WT, H), ((dx1 + X1) / 2, Y0 - WT / 2, H / 2), wall)
    b2.box(stage, G + "/wall_left_top", (dx1 - dx0, WT, H - dh), ((dx0 + dx1) / 2, Y0 - WT / 2, (H + dh) / 2), wall)
    b2.box(stage, G + "/shutter_box", (dx1 - dx0 + 0.2, 0.35, 0.35), ((dx0 + dx1) / 2, Y0 + 0.18, dh + 0.18), shutter)
    for k in range(6):                                   # 말려 올라간 셔터 살 일부
        b2.box(stage, f"{G}/shutter_slat_{k}", (dx1 - dx0, 0.03, 0.09), ((dx0 + dx1) / 2, Y0 - 0.02, dh - 0.05 - 0.1 * k), shutter)
    for i, x in enumerate((dx0, dx1)):
        b2.box(stage, f"{G}/shutter_rail_{i}", (0.08, 0.1, dh), (x, Y0 - 0.02, dh / 2), dark)
    # 앞쪽(x = X0): 시야 확보용 낮은 벽 + 유리 칸막이
    b2.box(stage, G + "/wall_front_low", (WT, Y1 - Y0, 1.0), (X0 - WT / 2, cym, 0.5), wall_lo)
    b2.box(stage, G + "/wall_front_glass", (0.02, Y1 - Y0, 1.6), (X0 - WT / 2, cym, 1.8), glass)
    # 기둥
    k = 0
    for x in (X0 + 0.15, cxm, X1 - 0.15):
        for y in (Y0 + 0.15, Y1 - 0.15):
            if abs(x - cxm) < 0.1 and y < 0:
                continue                                  # 셔터 앞은 비움
            b2.box(stage, f"{G}/column_{k}", (0.3, 0.3, H), (x, y, H / 2), col)
            k += 1

    # ---------- 바닥 표시 ----------
    def line(name, x0, y0, x1, y1, m, w=0.1):
        b2.box(stage, f"{G}/lines/{name}", (abs(x1 - x0) + (w if x0 == x1 else 0), abs(y1 - y0) + (w if y0 == y1 else 0), 0.003),
               ((x0 + x1) / 2, (y0 + y1) / 2, 0.0055), m)
    # 작업 베이 (리프트/차량)
    bx0, bx1, by0, by1 = 1.45, 4.2, -4.3, 1.5
    line("bay_a", bx0, by0, bx1, by0, yellow); line("bay_b", bx0, by1, bx1, by1, yellow)
    line("bay_c", bx1, by0, bx1, by1, yellow); line("bay_d", bx0, by0, bx0, by1, yellow)
    # 차량 진입로
    line("lane_l", 1.2, Y0, 1.2, by0, white, 0.08); line("lane_r", 4.0, Y0, 4.0, by0, white, 0.08)
    # 휠 내려놓는 자리
    px, py, ps = 1.0, 1.05, 0.5
    line("place_a", px - ps, py - ps, px + ps, py - ps, blue, 0.06); line("place_b", px - ps, py + ps, px + ps, py + ps, blue, 0.06)
    line("place_c", px - ps, py - ps, px - ps, py + ps, blue, 0.06); line("place_d", px + ps, py - ps, px + ps, py + ps, blue, 0.06)
    # AMR 주행로 (예약, 미사용): 점선
    for i in range(int((Y1 - Y0) / 0.62)):
        y = Y0 + 0.6 + 0.62 * i
        if y + 0.35 > Y1:
            break
        for j, x in enumerate((-2.9, -1.9)):
            b2.box(stage, f"{G}/lines/amr_{j}_{i}", (0.06, 0.35, 0.003), (x, y + 0.175, 0.0055), blue)

    # ---------- 로봇 제어기 (안전 펜스 없음) ----------
    fx = -1.55
    # 로봇 제어기 2대 + 조작반
    for i, y in enumerate((-1.6, 1.4)):
        b2.box(stage, f"{G}/controller_{i}", (0.55, 0.75, 1.25), (fx - 0.45, y, 0.625), cab)
        b2.box(stage, f"{G}/controller_{i}_door", (0.01, 0.6, 1.0), (fx - 0.45 - 0.28, y, 0.65), dark)
    b2.box(stage, G + "/hmi_stand", (0.08, 0.08, 1.1), (fx - 0.25, 0.75, 0.55), dark)
    b2.box(stage, G + "/hmi_panel", (0.06, 0.4, 0.3), (fx - 0.25, 0.75, 1.25), dark, rot_y=-20.0)
    b2.cyl(stage, G + "/estop", 0.04, 0.03, (fx - 0.25, 0.5, 1.12), red)

    # ---------- 타이어 랙 (오른쪽 벽 쪽) ----------
    rack(stage, G + "/rack_new", (2.6, Y1 - 1.0), 4.2, 3, (0.1, 0.55, 0.2), with_rim=False)       # 신규 타이어
    rack(stage, G + "/rack_used", (-2.4, Y1 - 1.0), 3.4, 2, (0.6, 0.15, 0.1), with_rim=True)      # 탈거 휠/폐타이어
    # 바닥에 쌓아 둔 타이어
    for k in range(4):
        tire(stage, f"{G}/tire_stack_{k}", (X1 - 0.8, 2.6, 0.1125 + 0.225 * k))

    # ---------- 공구함, 작업대, 장비 (뒷벽 쪽) ----------
    tx, ty = X1 - 0.45, -1.0
    b2.box(stage, G + "/toolchest/body", (0.55, 1.1, 0.95), (tx, ty, 0.55), red)
    b2.box(stage, G + "/toolchest/top", (0.5, 1.1, 0.45), (tx, ty, 1.25), red)
    for k in range(6):
        b2.box(stage, f"{G}/toolchest/drawer_{k}", (0.01, 1.0, 0.11), (tx - 0.28, ty, 0.2 + 0.14 * k), dark)
        b2.box(stage, f"{G}/toolchest/handle_{k}", (0.015, 0.7, 0.015), (tx - 0.29, ty, 0.24 + 0.14 * k), steel)
    for i, (dx, dy) in enumerate(((-0.2, -0.45), (-0.2, 0.45), (0.2, -0.45), (0.2, 0.45))):
        b2.cyl(stage, f"{G}/toolchest/caster_{i}", 0.04, 0.03, (tx + dx, ty + dy, 0.04), dark, axis="Y")
    wx, wy = X1 - 0.45, 1.6
    b2.box(stage, G + "/workbench/top", (0.75, 2.0, 0.05), (wx, wy, 0.88), wood)
    for i, (dx, dy) in enumerate(((-0.3, -0.9), (-0.3, 0.9), (0.3, -0.9), (0.3, 0.9))):
        b2.box(stage, f"{G}/workbench/leg_{i}", (0.06, 0.06, 0.86), (wx + dx, wy + dy, 0.43), steel)
    b2.box(stage, G + "/workbench/shelf", (0.7, 1.9, 0.03), (wx, wy, 0.3), steel)
    b2.box(stage, G + "/workbench/pegboard", (0.03, 2.0, 0.9), (X1 - 0.03, wy, 1.6), cab)
    b2.box(stage, G + "/workbench/vise", (0.15, 0.12, 0.14), (wx - 0.2, wy - 0.7, 0.98), dark)
    # 에어 컴프레서, 드럼, 소화기, 폐유통
    b2.cyl(stage, G + "/compressor/tank", 0.25, 1.1, (X1 - 0.5, -3.6, 0.45), red, axis="Y")
    b2.box(stage, G + "/compressor/motor", (0.3, 0.45, 0.3), (X1 - 0.5, -3.6, 0.86), dark)
    for i, y in enumerate((-4.0, -3.2)):
        b2.box(stage, f"{G}/compressor/foot_{i}", (0.4, 0.06, 0.2), (X1 - 0.5, y, 0.1), dark)
    for i, (x, y, c) in enumerate(((X1 - 0.45, -5.2, (0.1, 0.3, 0.6)), (X1 - 0.45, -5.9, (0.15, 0.15, 0.15)))):
        b2.cyl(stage, f"{G}/drum_{i}", 0.29, 0.88, (x, y, 0.44), mat(stage, f"drum_{i}", c, 0.5, 0.5))
    b2.cyl(stage, G + "/extinguisher", 0.07, 0.5, (X0 + 0.25, Y1 - 0.4, 0.3), red)
    b2.box(stage, G + "/extinguisher_sign", (0.01, 0.25, 0.25), (X0 + 0.02, Y1 - 0.4, 1.5), red)
    # 휠 밸런서 (더미)
    b2.box(stage, G + "/balancer/body", (0.6, 0.9, 0.95), (X1 - 0.8, -2.4, 0.475), cab)
    b2.cyl(stage, G + "/balancer/shaft", 0.02, 0.35, (X1 - 1.2, -2.4, 0.8), steel, axis="X")
    b2.box(stage, G + "/balancer/hood", (0.5, 0.8, 0.04), (X1 - 0.9, -2.4, 1.25), dark, rot_y=25.0)
    # 벽 안내판
    b2.box(stage, G + "/sign_bay", (0.02, 1.6, 0.5), (X1 - 0.02, -2.0, 3.2), blue)
    b2.box(stage, G + "/sign_safety", (1.4, 0.02, 0.5), (0.0, Y1 - 0.02, 3.0), yellow)

    # 천장 구조물/천장등 없음 (위가 열린 맵, 조명은 셀 씬의 돔/태양광 사용)
    stage.GetRootLayer().Save()
    b2.MATS.clear()


def build_scene():
    if os.path.exists(SCENE):
        os.remove(SCENE)
    stage = Usd.Stage.CreateNew(SCENE)
    stage.GetRootLayer().subLayerPaths.append("./tire_cell_v3.usda")     # 로봇·차량·리프트·물리 설정은 그대로
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    m = UsdGeom.Xform.Define(stage, "/World/Map")
    m.GetPrim().GetReferences().AddReference("./garage_map.usda")
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    build_map()
    build_scene()
    print("saved", MAP, "and", SCENE)
