"""
비전만 빠르게 시험 (전체 사이클 없이): 로봇 B 를 관측 자세로 보내 한 장 찍고, 찾은 너트를 실제(물리) 위치와 비교한다.

  cd ~/isaacsim && ./python.sh <이 폴더>/test_vision_isaac.py --headless [--car-offset 0,0.03,-0.01] [--noise 1.0] [--res 1280x800]

결과: 너트별 오차 [mm], 축 오차 [deg], 처리 시간, vision_debug.png (이 폴더)
"""
import argparse
import os
import sys

sys.stdout.reconfigure(line_buffering=True)          # 로그가 Isaac 종료 때 버퍼에 남아 사라지지 않게
import time

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--no-map", action="store_true")
ap.add_argument("--car-offset", default="0,0,0")
ap.add_argument("--wheel-rotate", type=float, default=0.0)
ap.add_argument("--noise", type=float, default=0.0, help="깊이 노이즈 표준편차 [mm]")
ap.add_argument("--frames", type=int, default=1)
ap.add_argument("--res", default="1280x800")
ap.add_argument("--dist", type=float, default=0.55)
args, _ = ap.parse_known_args()

HERE = os.path.dirname(os.path.abspath(__file__))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless})
sys.path.insert(0, HERE)
import json  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdGeom  # noqa: E402

import nut_vision as nv  # noqa: E402
import vision_capture as vc  # noqa: E402
from cell_kinematics import ARM_JOINTS, Arm, T_from, mat, quat_to_R, sample, smooth  # noqa: E402

scene = os.path.join(HERE, "tire_cell_v3.usda" if args.no_map else "tire_cell_v3_map.usda")
ctx = omni.usd.get_context()
ctx.open_stage(scene)
for _ in range(30):
    app.update()
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
stage = ctx.get_stage()
cell = stage.GetPrimAtPath("/World").GetCustomData()["cell"]
RB, GB, WHEEL = cell["robot_b"], cell["nutrunner_path"], cell["wheel_path"]
nut_local = np.array(json.loads(cell["nut_points_wheel_local"]))
T_prior = mat(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(WHEEL)))     # 배치도 값 (차를 옮기기 전)
off = [float(v) for v in args.car_offset.split(",")]
if any(off):
    vc.shift_car(stage, off)
    print("[test] car shifted by", off)
if args.wheel_rotate:
    vc.rotate_wheel(stage, args.wheel_rotate)
    print("[test] wheel + hub rotated %.1f deg" % args.wheel_rotate)

w, h = (int(v) for v in args.res.split("x"))
cam = vc.WristCamera(stage, GB + "/Camera_RSD455/RSD455", resolution=(w, h))
T_tcp_cam = vc.tcp_to_camera(stage, GB + "/base/tcp", cam)
arm = Arm(stage, RB, GB)
drive = [stage.GetPrimAtPath(RB + "/" + j).GetAttribute("drive:angular:physics:targetPosition") for j in ARM_JOINTS]
R_TOOL_X = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
z_top = float(np.mean(nut_local[:, 2])) + 0.015
T_obs = vc.observation_tcp(T_prior, z_top, T_tcp_cam, R_TOOL_X, args.dist)
q_obs, ok = arm.ik(T_obs, np.array([-37.5, 15.0, -103.8, 85.9, 82.6, -60.9]))
print("[test] observation IK ok=%s q=%s" % (ok, np.round(q_obs, 1)))

from omni.physx import get_physx_interface  # noqa: E402

px = get_physx_interface()


def body_pose(path):
    d = px.get_rigidbody_transformation(path)
    p, r = d["position"], d["rotation"]
    return T_from(np.array(p[:3], dtype=float), quat_to_R(r[3], r[0], r[1], r[2]))


def updates(n):
    for _ in range(n):
        app.update()


omni.timeline.get_timeline_interface().play()
updates(120)
# 관절 부호 확인 (cell_demo 와 같은 방법)
sign = 1.0
for s in (1.0, -1.0):
    for a, v in zip(drive, s * np.array([8.0, 8.0, 0, 0, 0, 0])):
        a.Set(float(v))
    updates(150)
    if np.linalg.norm(arm.fk6(np.array([8.0, 8.0, 0, 0, 0, 0]))[:3, 3] - body_pose(RB + "/link6")[:3, 3]) < 0.01:
        sign = s
        break
print("[test] joint sign", sign)
via = np.array([q_obs[0], -30.0, -120.0, q_obs[3], q_obs[4], q_obs[5]])
path = [np.zeros(6), via, q_obs]
for k in range(301):
    q = sample(path, smooth(k / 300))
    for a, v in zip(drive, q):
        a.Set(float(sign * v))
    app.update()
updates(120)
T_cam_fk = arm.fk(q_obs) @ T_tcp_cam
print("[test] camera pose: physics vs FK %.2f mm" % (np.linalg.norm(cam.pose()[:3, 3] - T_cam_fk[:3, 3]) * 1000))

from omni.kit.async_engine import run_coroutine  # noqa: E402

t0 = time.time()
import omni.kit.app  # noqa: E402

task = run_coroutine(cam.capture(omni.kit.app.get_app().next_update_async, n_frames=args.frames))
while not task.done():
    app.update()
rgb, depth, T_wc = task.result()
t1 = time.time()
if args.noise > 0:
    depth = depth + np.random.default_rng(1).normal(0, args.noise / 1000, depth.shape).astype(np.float32) * (depth > 0)
res = nv.find_wheel(depth, cam.K, T_wc, T_prior, nut_local, rgb=rgb, debug=True)
t2 = time.time()
T_gt = body_pose(WHEEL)
gt_nuts = (T_gt @ np.c_[nut_local, np.ones(5)].T).T[:, :3]
print("[test] capture %.2f s (wall, incl. render warm-up), vision %.0f ms" % (t1 - t0, (t2 - t1) * 1000))
if res["T_wheel"] is not None:
    _c = T_prior[:3, :3].T @ res["T_wheel"][:3, :3]
    _g = T_prior[:3, :3].T @ T_gt[:3, :3]
    print("[test] wheel rotation: vision %.2f deg, actual %.2f deg" % (np.degrees(np.arctan2(_c[1, 0], _c[0, 0])), np.degrees(np.arctan2(_g[1, 0], _g[0, 0]))))
print("[test] candidates %s, found %d, ok=%s %s" % (res.get("n_candidates"), res["n_found"], res["ok"], res["reason"]))
print("[test] prior vs actual wheel: %.1f mm" % (np.linalg.norm(T_prior[:3, 3] - T_gt[:3, 3]) * 1000))
if res["T_wheel"] is not None:
    e = np.linalg.norm(res["nuts_world"] - gt_nuts, axis=1) * 1000
    ax = np.degrees(np.arccos(min(1.0, float(res["T_wheel"][:3, 2] @ T_gt[:3, 2]))))
    print("[test] nut error mm:", np.round(e, 2), " max %.2f" % e.max())
    print("[test] center error in wheel frame (x, y, axial) mm:", np.round(T_gt[:3, :3].T @ (res["T_wheel"][:3, 3] - T_gt[:3, 3]) * 1000, 2))
    print("[test] axis error %.3f deg, center error %.2f mm, rms %.2f mm, pad points %s" % (
        ax, np.linalg.norm(res["T_wheel"][:3, 3] - T_gt[:3, 3]) * 1000, res["rms"] * 1000, res.get("pad_points")))
    print("[test] fixed-coordinate (prior) nut error mm:", np.round(np.linalg.norm((T_prior @ np.c_[nut_local, np.ones(5)].T).T[:, :3] - gt_nuts, axis=1) * 1000, 1))
if res["debug"] is not None:
    cv2.imwrite(os.path.join(HERE, "vision_debug.png"), res["debug"])
cv2.imwrite(os.path.join(HERE, "vision_rgb.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
print("[test] DONE")
app.close()
