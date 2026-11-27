"""
타이어 교체 셀 v6 시연 - 한 템포: 너트 5개 풀어 트레이 -> 휠 탈거(바닥) -> 2초 대기 -> 역순으로 휠 장착 -> 트레이 너트 조임

  bash run.sh            (또는)   cd ~/isaacsim && ./python.sh <이 폴더>/run_cell_demo.py

옵션
  --speed 1.5        동작 속도 배율 (관절 최대 속도는 넘지 않음)
  --mount joint      휠이 스터드에 '접촉'으로 걸려 있지 못할 때: 차축 방향 구속 관절로 대체
  --grasp joint      접촉 파지가 실패할 때: 조를 닫은 뒤 고정 관절로 대체
  --skip-nuts        너트러너(로봇 B) 단계를 건너뜀 (풀기·조이기 둘 다)
  --pause 2.0        휠을 바닥에 놓은 뒤 역순 장착 전 대기 시간 [s]
  --no-reassembly    휠을 바닥에 놓고 끝냄 (v4 동작)
  --no-map           정비소 맵 없이 셀만 실행
  --no-safety        보호 정지 끄기 (차체 이동·충돌이 감지돼도 계속 진행)
  --no-track-move    앞바퀴 작업 뒤 트랙을 타고 뒷바퀴로 이동하는 단계를 생략
  --no-ros           손목 카메라 ROS 2 토픽을 내보내지 않음 (기본: 내보냄, ros2_cameras.py 참고)
  --ros-res 640x400  ROS 2 로 내보내는 카메라 해상도 (D455 비율 16:10. 1280x800 도 가능)
  --no-vision        손목 카메라 비전을 끄고 예전처럼 휠의 물리 자세로 너트 위치를 계산
  --vision-res 1280x800  비전 촬영 해상도 (촬영하는 순간에만 렌더링)
  --vision-noise 1.0 시뮬레이션 깊이에 가우시안 노이즈 [mm] 추가 (실제 D455 처럼)
  --no-rear          앞바퀴 뒤에 뒷바퀴(왼쪽 뒤) 작업을 하지 않음 (예전처럼 트랙만 뒷바퀴 위치로 이동)
  --rear-only        앞바퀴는 건너뛰고 바로 스캔 -> 뒷바퀴 작업 (시험용)
  --rear-wheel-rotate 15  뒷바퀴+허브를 [deg] 돌려 놓고 시작
  --wheel-rotate 20  휠+허브를 차축 둘레로 [deg] 돌려 놓고 시작 (너트 위치가 배치도와 달라짐: 비전 시연용)
  --car-offset 0,0.03,-0.01  차(차체·코너·휠)를 배치도 위치에서 [m] 만큼 옮겨 놓고 시작 (비전 시연용: 고정 좌표면 빗나감)
  --headless / --exit
Stop 후 Play 를 누르면 처음부터 다시 실행된다. 끝나면 contact_log.csv 가 이 폴더에 저장된다.
"""
import argparse
import os
import sys

sys.stdout.reconfigure(line_buffering=True)          # 로그가 Isaac 종료 때 버퍼에 남아 사라지지 않게

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=None)
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--mount", choices=["contact", "joint"], default=None)
ap.add_argument("--grasp", choices=["physics", "joint"], default="physics")
ap.add_argument("--skip-nuts", action="store_true")
ap.add_argument("--pause", type=float, default=2.0)
ap.add_argument("--no-reassembly", action="store_true")
ap.add_argument("--no-safety", action="store_true")
ap.add_argument("--no-map", action="store_true")
ap.add_argument("--no-track-move", action="store_true")
ap.add_argument("--no-ros", action="store_true")
ap.add_argument("--ros-res", default="640x400")
ap.add_argument("--no-vision", action="store_true")
ap.add_argument("--vision-res", default="1280x800")
ap.add_argument("--vision-noise", type=float, default=0.0)
ap.add_argument("--car-offset", default="0,0,0")
ap.add_argument("--wheel-rotate", type=float, default=0.0)
ap.add_argument("--no-rear", action="store_true")
ap.add_argument("--rear-only", action="store_true")
ap.add_argument("--rear-wheel-rotate", type=float, default=0.0)
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
cell_demo.PAUSE_S = args.pause
cell_demo.REASSEMBLE = not args.no_reassembly
cell_demo.SAFETY = not args.no_safety
cell_demo.TRACK_TO_REAR = not args.no_track_move
cell_demo.LOG_DIR = HERE
cell_demo.VISION = not args.no_vision
cell_demo.VISION_RES = tuple(int(v) for v in args.vision_res.lower().split("x"))
cell_demo.VISION_NOISE_MM = args.vision_noise
cell_demo.VISION_ROS = not args.no_ros
cell_demo.DO_FRONT = not args.rear_only
if not args.no_rear and not args.no_track_move:            # 왼쪽 뒷바퀴를 앞바퀴처럼 탈거 가능하게 (세션 레이어, 씬 파일은 안 바뀜)
    try:
        import numpy as _np
        from pxr import UsdGeom as _UsdGeom

        import rear_wheel
        _rear = rear_wheel.add_removable_wheel(ctx.get_stage(), "RL")
        _rear["layout_prior"] = _np.array(_UsdGeom.XformCache().GetLocalToWorldTransform(ctx.get_stage().GetPrimAtPath(_rear["wheel"]))).T.tolist()
        cell_demo.REAR = _rear
        print("[run_cell_demo] rear-left wheel made removable: %s, nut tray %s" % (_rear["wheel"], "/World/NutTray_RL"))
    except Exception as e:
        print("[run_cell_demo] rear wheel NOT prepared (%s) -> front wheel only" % e)
_off = [float(v) for v in args.car_offset.split(",")]
if any(_off):                                            # cell_demo 를 import 할 때 배치도 휠 위치를 이미 기억했다 -> 그 뒤에 옮김
    import vision_capture
    vision_capture.shift_car(ctx.get_stage(), _off)
    print("[run_cell_demo] car shifted by (%.3f, %.3f, %.3f) m from the layout" % tuple(_off))
if args.wheel_rotate:
    import vision_capture
    vision_capture.rotate_wheel(ctx.get_stage(), args.wheel_rotate)
    print("[run_cell_demo] wheel + hub rotated %.1f deg about the axle (nuts are no longer where the layout says)" % args.wheel_rotate)
if args.rear_wheel_rotate and cell_demo.REAR is not None:
    import vision_capture
    vision_capture.rotate_wheel(ctx.get_stage(), args.rear_wheel_rotate, corner=cell_demo.REAR["corner"], wheel=cell_demo.REAR["wheel"],
                                suspension=cell_demo.REAR["suspension"])
    print("[run_cell_demo] rear wheel + hub rotated %.1f deg" % args.rear_wheel_rotate)

omni.timeline.get_timeline_interface().play()
while app.is_running():
    app.update()
    if done["flag"] and args.exit:
        for _ in range(120):
            app.update()
        break
builtins.print = _print
app.close()
