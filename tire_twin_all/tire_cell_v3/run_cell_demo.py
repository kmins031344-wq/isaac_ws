"""
타이어 교체 셀 v3 시연 - 터미널에서 한 번에 실행

  bash run.sh            (또는)   cd ~/isaacsim && ./python.sh <이 폴더>/run_cell_demo.py

옵션
  --speed 1.5        동작 속도 배율 (관절 최대 속도는 넘지 않음)
  --mount joint      휠이 스터드에 '접촉'으로 걸려 있지 못할 때: 차축 방향 구속 관절로 대체
  --grasp joint      접촉 파지가 실패할 때: 조를 닫은 뒤 고정 관절로 대체
  --skip-nuts        너트러너(로봇 B) 단계를 건너뜀
  --no-map           정비소 맵 없이 셀만 실행
  --no-safety        보호 정지 끄기 (차체 이동·충돌이 감지돼도 계속 진행)
  --no-track-move    앞바퀴 작업 뒤 트랙을 타고 뒷바퀴로 이동하는 단계를 생략
  --no-ros           손목 카메라 ROS 2 토픽을 내보내지 않음 (기본: 내보냄, ros2_cameras.py 참고)
  --ros-res 640x400  ROS 2 로 내보내는 카메라 해상도 (D455 비율 16:10. 1280x800 도 가능)
  --headless / --exit
Stop 후 Play 를 누르면 처음부터 다시 실행된다. 끝나면 contact_log.csv 가 이 폴더에 저장된다.
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=None)
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--mount", choices=["contact", "joint"], default=None)
ap.add_argument("--grasp", choices=["physics", "joint"], default="physics")
ap.add_argument("--skip-nuts", action="store_true")
ap.add_argument("--no-safety", action="store_true")
ap.add_argument("--no-map", action="store_true")
ap.add_argument("--no-track-move", action="store_true")
ap.add_argument("--no-ros", action="store_true")
ap.add_argument("--ros-res", default="640x400")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--exit", action="store_true")
args, _ = ap.parse_known_args()

HERE = os.path.dirname(os.path.abspath(__file__))
_map = os.path.join(HERE, "tire_cell_v3_map.usda")      # 정비소 맵이 얹힌 씬 (있으면 기본으로 사용)
SCENE = args.scene or (_map if (os.path.exists(_map) and not args.no_map) else os.path.join(HERE, "tire_cell_v3.usda"))
if not os.path.exists(SCENE):
    sys.exit("scene not found: " + SCENE)

try:
    from isaacsim import SimulationApp          # Isaac Sim 4.5+
except ImportError:
    from omni.isaac.kit import SimulationApp
app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900})
sys.path.insert(0, HERE)
if not args.no_ros:
    import ros2_cameras
    ros2_cameras.enable_bridge()
    for _ in range(10):                                  # 확장이 다 올라온 뒤에 씬을 연다
        app.update()

import builtins
import omni.timeline
import omni.usd

ctx = omni.usd.get_context()
ctx.open_stage(SCENE)
for _ in range(30):
    app.update()
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
print("[run_cell_demo] scene loaded:", SCENE)
if not args.no_ros:
    try:
        _w, _h = (int(v) for v in args.ros_res.lower().split("x"))
        ros2_cameras.setup(ctx.get_stage(), _w, _h)
        for _ in range(5):
            app.update()
    except Exception as e:
        print("[run_cell_demo] ROS 2 camera topics NOT started:", e)
try:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[-3.0, -5.4, 3.4], target=[2.0, -0.3, 0.9])
except Exception as e:
    print("[run_cell_demo] camera not set:", e)

done = {"flag": False}
_print = builtins.print


def _tee(*a, **k):
    _print(*a, **k)
    if a and isinstance(a[0], str) and a[0].startswith("[cell_demo]") and ("DONE" in a[0] or "ERROR" in a[0] or "Abort" in a[0]):
        done["flag"] = True


builtins.print = _tee
sys.path.insert(0, HERE)
import cell_demo  # noqa: E402   (PLAY 이벤트에 맞춰 자동 실행되도록 등록됨)

cell_demo.SPEED = args.speed
cell_demo.GRASP_MODE = args.grasp
cell_demo.MOUNT_MODE = args.mount
cell_demo.SKIP_NUTS = args.skip_nuts
cell_demo.SAFETY = not args.no_safety
cell_demo.TRACK_TO_REAR = not args.no_track_move
cell_demo.LOG_DIR = HERE

omni.timeline.get_timeline_interface().play()
while app.is_running():
    app.update()
    if done["flag"] and args.exit:
        for _ in range(120):
            app.update()
        break
builtins.print = _print
app.close()
