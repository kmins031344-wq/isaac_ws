"""
타이어 탈거 시연 - 터미널에서 한 번에 실행

  cd ~/isaacsim
  ./python.sh ~/cobot3_ws/isaac_assets/run_tire_demo_v2.py

Isaac Sim 창을 띄우고 tire_demo_scene_v2.usda 를 연 뒤 Play 하고 시연 동작(tire_demo_v2.py)을 실행한다.
옵션: --speed 1.5 (빠르게), --headless (창 없이), --exit (끝나면 자동 종료), --grasp joint (접촉 파지 실패 시 고정 관절 방식)
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=None)
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--headless", action="store_true")
ap.add_argument("--exit", action="store_true")
ap.add_argument("--grasp", choices=["physics", "joint"], default="physics")
args, _ = ap.parse_known_args()

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = args.scene or os.path.join(HERE, "tire_demo_scene_v2.usda")
if not os.path.exists(SCENE):
    sys.exit("scene not found: " + SCENE)

try:
    from isaacsim import SimulationApp          # Isaac Sim 4.5+
except ImportError:
    from omni.isaac.kit import SimulationApp
app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900})

import builtins
import omni.timeline
import omni.usd

ctx = omni.usd.get_context()
ctx.open_stage(SCENE)
for _ in range(30):
    app.update()
while ctx.get_stage_loading_status()[2] > 0:    # 로딩 중인 파일 수
    app.update()
print("[run_tire_demo] scene loaded:", SCENE)

# 보기 좋은 카메라 위치
try:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[-2.4, 3.6, 2.5], target=[1.4, 0.1, 0.9])
except Exception as e:
    print("[run_tire_demo] camera not set:", e)

omni.timeline.get_timeline_interface().play()
for _ in range(60):
    app.update()

# 시연 동작 시작 (tire_demo_v2.py 가 import 되면서 비동기로 실행됨)
done = {"flag": False}
_print = builtins.print


def _tee(*a, **k):
    _print(*a, **k)
    if a and isinstance(a[0], str) and a[0].startswith("[tire_demo]") and ("DONE" in a[0] or "ERROR" in a[0] or "Abort" in a[0]):
        done["flag"] = True


builtins.print = _tee
sys.path.insert(0, HERE)
import tire_demo_v2 as tire_demo  # noqa: E402

tire_demo.SPEED = args.speed
tire_demo.GRASP_MODE = args.grasp

while app.is_running():
    app.update()
    if done["flag"] and args.exit:
        for _ in range(120):
            app.update()
        break
builtins.print = _print
app.close()
