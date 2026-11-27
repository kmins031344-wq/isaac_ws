# 타이어 교체 디지털트윈 - 버전 묶음 (2026-11-27)

Isaac Sim 5.1 필요. 각 버전 폴더는 파일끼리 상대 경로로 연결돼 있어서 폴더째 어디에 두어도 됩니다.
프로젝트 전체 설명은 저장소 최상위 [README](../README.md) 를 보세요.

## 폴더

| 폴더 | 내용 |
|---|---|
| `tire_cell_v7_integrated/` | **최신 통합본.** 비전 너트 검출 + 분해·너트 보관·재조립 + 카메라·ROS 2·트랙. [README](tire_cell_v7_integrated/README.md) |
| `tire_cell_v3/` | 통합 전 본 (분해만 + 카메라·ROS 2·트랙). [README](tire_cell_v3/README.md) |
| `tire_demo_v2/` | 초기 시연 (로봇 1대가 휠만 빼서 내려놓는 버전). [README](tire_demo_v2/README.md) |
| `../docs/` | 발표용 정리 PDF, 확인용 그림 (저장소 최상위로 옮김) |

v4(너트 트레이 보관), v5(재조립), v6 는 v7 에 합쳐져서 폴더가 따로 없습니다. v4 설명은 `tire_cell_v7_integrated/README_V4.md` 에 남아 있습니다.

## `tire_cell_v7_integrated/` 에 들어 있는 것

| 항목 | 파일 |
|---|---|
| 그리퍼 로봇 (Kawasaki RS080N + 4조 휠 그리퍼) | `rs080n_pzv160_flat.usda`, `wheel_gripper_4jaw.usda`, 보기: `gripper_robot_scene.usda` |
| 너트러너 로봇 (RS080N + Atlas Copco QST62-230CT 치수 모델) | `nutrunner_qst62.usda`, 보기: `nutrunner_robot_scene.usda` |
| 로봇 셀 (로봇 2대 + 차 + 리프트 + 물리) | `tire_cell_v3.usda`, 설정값 `params.json` |
| 정비소 맵 | `garage_map.usda`, 셀과 합친 씬 `tire_cell_v3_map.usda` |
| 리프트 (자체 제작 더미) | 셀 씬 안에 포함, 보기: `lift_scene.usda` |
| 너트 보관 트레이 | `nut_tray.usda`, 보기: `nut_tray_scene.usda` |
| 차량 4종 (아반떼 CN7, K5, 그랜저 GN7, K9) | `cars/*.usda`, 제원 `car_specs.json`, 설명 `CAR_ASSETS.md` |
| 리니어 트랙, 손목 카메라 D455, ROS 2 | `track_mount.py`, `camera_mount.py`, `ros2_cameras.py` |
| 비전 너트 검출 | `nut_vision.py`, `vision_capture.py`, `vision_ros.py`, 단독 시험 `test_vision_isaac.py` |
| 시연 코드 | `cell_demo.py`, `cell_kinematics.py`, `run_cell_demo.py`, `run.sh` |
| 다시 만드는 스크립트 | `build_scene_v3.py`, `build_map.py`, `build_cars.py`, `car_body.py`, `build_tray.py`, `nut_tray.py` |

셀 씬 파일 이름이 `tire_cell_v3*.usda` 인 것은 v3 에서 만든 씬을 계속 이어 쓰기 때문입니다.

## 실행

```bash
cd tire_cell_v7_integrated
bash run.sh              # 전체 시연 (비전 → 분해 → 재조립 → 트랙 이동, 카메라 ROS 2 토픽 포함)
bash run.sh --no-ros     # ROS 토픽 없이 (더 빠름)
bash view.sh             # 볼 수 있는 씬 이름 목록
bash view.sh tray        # 예: 너트 트레이 보기.  cars / lift / nutrunner / gripper / map ...
```

`view.sh` 가 안 되면 Isaac Sim 을 켜고 `Ctrl + O` 로 `.usda` 파일을 직접 여세요.
Isaac Sim 이 `~/isaacsim` 이 아닌 곳에 있으면 앞에 `ISAACSIM_PATH=/경로/isaacsim` 을 붙입니다.

이전 버전은 `cd tire_cell_v3 && bash run.sh`, `cd tire_demo_v2 && bash run.sh` 입니다.

## 확인 상태

- `tire_cell_v7_integrated`: `--no-ros` 전체 사이클 Isaac Sim 실행 확인 (최종 휠 자세 오차 0.6 mm / 0.01°, 322 s). ROS 토픽을 켠 전체 사이클은 미확인.
- `tire_demo_v2`: Isaac Sim 에서 실행 확인됨.
- `tire_cell_v3`: 토크 제한·보호 정지를 넣은 본은 실행 결과를 확인하지 못함.
- 차량 4종, 트레이, 맵, `view.sh`: Isaac Sim 실행 검증 전.
