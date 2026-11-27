# -*- coding: utf-8 -*-
"""씬 하나를 Isaac Sim 으로 여는 간단한 실행기 (보기/스크린샷용, 로봇 동작 없음).  사용:  bash view.sh <이름>"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCENES = {
    "cell": "tire_cell_v3.usda",                 # 로봇 2대 + 차 + 리프트
    "map": "tire_cell_v3_map.usda",              # 위 + 정비소 맵
    "cars": "cars/cars_ground.usda",             # 차량 4종, 바닥 위
    "cars_lift": "cars/cars_lineup.usda",        # 차량 4종, 받침대 위
    "avante": "cars/avante_cn7_test.usda",
    "k5": "cars/k5_dl3_test.usda",
    "grandeur": "cars/grandeur_gn7_test.usda",
    "k9": "cars/k9_rj_test.usda",
    "tray": "nut_tray_scene.usda",               # 너트 보관 트레이
    "lift": "lift_scene.usda",                   # 리프트만
    "nutrunner": "nutrunner_robot_scene.usda",   # 너트러너 로봇만
    "gripper": "gripper_robot_scene.usda",       # 그리퍼 로봇만
}
name = sys.argv[1] if len(sys.argv) > 1 else ""
path = os.path.join(HERE, SCENES.get(name, name))
if not name or not os.path.exists(path):
    print("사용법: bash view.sh <이름>\n이름: " + ", ".join(SCENES))
    sys.exit(1)
try:
    from isaacsim import SimulationApp          # Isaac Sim 4.5+
except ImportError:
    from omni.isaac.kit import SimulationApp
app = SimulationApp({"headless": False, "width": 1600, "height": 900})
import omni.usd  # noqa: E402

omni.usd.get_context().open_stage(path)
print("[view_scene] opened", path)
while app.is_running():
    app.update()
app.close()
