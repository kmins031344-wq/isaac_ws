"""
너트 보관 트레이 모형 (외형만, 로봇 동작 없음)
  nut_tray.usda        : 트레이 에셋 (defaultPrim /NutTray). 리프트 옆면에 붙는 브래킷 + 45도 기울인 판 + M12 볼트 40개(엇갈림 배열)
  nut_tray_scene.usda  : 스크린샷용 씬 (바닥 + 리프트 일부 + 트레이, 너트 5개가 끼워진 상태)
실행: python build_tray.py <출력폴더>
"""
import math
import os
import sys

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else HERE
_argv = sys.argv
sys.argv = [sys.argv[0], OUT_DIR]
sys.path.insert(0, HERE)
import build_scene_v2 as b2  # noqa: E402
import nut_tray as nt  # noqa: E402

sys.argv = _argv
T = "/NutTray"
L = T + "/Looks"


def mat(stage, name, color, metallic=0.0, rough=0.5):
    return b2.material(stage, name, color, metallic, rough, 1.0, root=L)


def build_tray(path, nuts_on=((2, 1), (2, 2), (2, 3), (2, 4), (2, 5))):
    if os.path.exists(path):
        os.remove(path)
    b2.MATS.clear()
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, T)
    stage.SetDefaultPrim(root.GetPrim())
    rp = root.GetPrim()
    for k, v in (("pegs", nt.ROWS * nt.COLS), ("peg_thread", "M12x1.5"), ("peg_height_mm", nt.PEG_H * 1000), ("pitch_mm", "%g x %g" % (nt.PITCH_U * 1000, nt.PITCH_V * 1000)),
                 ("tilt_deg", nt.TILT_DEG), ("plate_mm", "%g x %g x %g" % tuple(v * 1000 for v in nt.PLATE)), ("note", "visual model only (no robot motion). Origin = 브래킷이 리프트 옆면에 닿는 면의 위쪽 가운데, -X 가 바깥(로봇 쪽)")):
        rp.SetCustomDataByKey("tray:" + k, v)
    alu = mat(stage, "tray_aluminium", (0.80, 0.81, 0.82), 1.0, 0.38)
    green = mat(stage, "bracket_green", (0.45, 0.80, 0.10), 0.2, 0.5)
    steel = mat(stage, "peg_steel", (0.62, 0.63, 0.65), 1.0, 0.3)
    dark = mat(stage, "bolt_dark", (0.12, 0.12, 0.13), 0.8, 0.4)
    nutm = mat(stage, "lug_nut", (0.72, 0.72, 0.74), 1.0, 0.22)
    W, D, TH = nt.PLATE
    t = math.radians(nt.TILT_DEG)
    # 브래킷: 리프트 옆면에 붙는 세로판 + 기울어진 받침판 + 삼각 보강판 2개
    b2.box(stage, T + "/bracket/wall_plate", (0.012, W + 0.04, 0.16), (-0.006, 0, -0.08), green)
    for i, y in enumerate((-(W / 2 - 0.05), W / 2 - 0.05)):
        b2.cyl(stage, f"{T}/bracket/wall_bolt_{i}", 0.009, 0.008, (-0.016, y, -0.03), dark, axis="X")
        g = b2.mesh(stage, f"{T}/bracket/gusset_{i}", [(-0.012, y - 0.004, -0.07), (-0.012, y - 0.004, -0.16), (-0.095, y - 0.004, -0.16),
                                                       (-0.012, y + 0.004, -0.07), (-0.012, y + 0.004, -0.16), (-0.095, y + 0.004, -0.16)],
                    [3, 3, 4, 4, 4], [0, 2, 1, 3, 4, 5, 0, 1, 4, 3, 1, 2, 5, 4, 2, 0, 3, 5], green)     # 판 아래쪽 보강판
        del g
    # 판: 위쪽 가장자리가 브래킷에 붙고 바깥쪽(-X) 아래로 45도 내려간다
    P = T + "/plate"
    px = UsdGeom.Xform.Define(stage, P)
    px.AddTranslateOp().Set(Gf.Vec3d(-0.012 - (D / 2) * math.cos(t), 0.0, -0.035 - (D / 2) * math.sin(t)))
    px.AddRotateYOp().Set(-nt.TILT_DEG)                    # 판 로컬: x = 경사 위쪽, y = 가로, z = 볼트 방향
    b2.box(stage, P + "/backing", (D + 0.02, W + 0.02, 0.010), (0, 0, -TH - 0.005), green)
    b2.box(stage, P + "/plate", (D, W, TH), (0, 0, -TH / 2), alu)
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        b2.cyl(stage, f"{P}/corner_bolt_{i}", 0.006, 0.005, (sx * (D / 2 - 0.018), sy * (W / 2 - 0.018), 0.0025), dark)
    peg_prof = [(0.0, 0.0), (nt.PEG_R, 0.0), (nt.PEG_R, nt.PEG_H - 0.006), (nt.PEG_R - 0.003, nt.PEG_H), (0.0, nt.PEG_H)]      # 끝 모따기
    nut_prof = [(0.0, 0.0), (0.0121, 0.0), (0.0121, 0.016), (0.0095, 0.024), (0.0, 0.024)]
    for r in range(nt.ROWS):
        for c in range(nt.COLS):
            x = (r - (nt.ROWS - 1) / 2) * nt.PITCH_V
            y = (c - (nt.COLS - 1) / 2) * nt.PITCH_U + (nt.PITCH_U / 4 if r % 2 else -nt.PITCH_U / 4)
            b2.lathe(stage, f"{P}/pegs/peg_{r}_{c}", peg_prof, steel, n=16, center=(x, y))
            b2.lathe(stage, f"{P}/pegs/washer_{r}_{c}", [(0.0, 0.0), (0.010, 0.0), (0.010, 0.0015), (0.0, 0.0015)], dark, n=16, center=(x, y))
            if (r, c) in nuts_on:
                b2.lathe(stage, f"{P}/nuts/nut_{r}_{c}", nut_prof, nutm, n=6, center=(x, y))
    stage.GetRootLayer().Save()
    b2.MATS.clear()


def build_scene(path):
    if os.path.exists(path):
        os.remove(path)
    b2.MATS.clear()
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    w = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(w.GetPrim())
    floor = b2.material(stage, "floor", (0.55, 0.55, 0.53), 0.0, 0.6)
    lg = b2.material(stage, "lift_green", (0.25, 0.70, 0.12), 0.2, 0.5)
    deck = b2.material(stage, "lift_deck", (0.42, 0.62, 0.36), 0.5, 0.45)
    bump = b2.material(stage, "deck_bump", (0.36, 0.55, 0.31), 0.5, 0.4)
    g = b2.box(stage, "/World/GroundPlane", (8.0, 8.0, 0.1), (0, 0, -0.05), floor)
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    # 리프트 플랫폼 일부 (옆면 x = 0, 윗면 z = 0.26). 윗면에 미끄럼 방지 돌기
    H = 0.26
    b2.box(stage, "/World/LiftPlatform/body", (1.2, 2.4, H), (0.6, 0, H / 2), lg)
    b2.box(stage, "/World/LiftPlatform/deck", (1.2, 2.4, 0.004), (0.6, 0, H + 0.002), deck)
    k = 0
    for i in range(22):
        for j in range(44):
            b2.box(stage, f"/World/LiftPlatform/bumps/b_{k}", (0.026, 0.008, 0.002), (0.04 + 0.052 * i + (0.026 if j % 2 else 0), -1.16 + 0.054 * j, H + 0.005), bump, rot_z=45.0 if (i + j) % 2 else -45.0)
            k += 1
    tray = UsdGeom.Xform.Define(stage, "/World/NutTray")
    tray.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, H - 0.01))
    tray.GetPrim().GetReferences().AddReference("./nut_tray.usda")
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(900.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun.CreateIntensityAttr(2200.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, -25, 0))
    stage.GetRootLayer().Save()
    b2.MATS.clear()


if __name__ == "__main__":
    build_tray(os.path.join(OUT_DIR, "nut_tray.usda"))
    build_scene(os.path.join(OUT_DIR, "nut_tray_scene.usda"))
    print("saved nut_tray.usda, nut_tray_scene.usda")
