# 타이어 교체 디지털트윈 - 전체 묶음 (2026-11-27)

Isaac Sim 5.1 필요. 폴더째 아무 곳에나 두고 쓰면 됩니다 (파일끼리 상대 경로로 연결).

## 폴더

| 폴더 | 내용 |
|---|---|
| `tire_cell_v7_integrated/` | **최신 통합본.** 조립·분해·너트 보관 + 카메라·ROS 2·트랙. 이 폴더의 README 참고 |
| `tire_cell_v3/` | 통합 전 본 (카메라·ROS 2·트랙만). 아래 표의 모든 것이 여기 들어 있습니다 |
| `tire_demo_v2/` | 이전 시연 (로봇 1대가 휠만 빼서 내려놓는 버전) |
| `docs/` | 발표용 정리 PDF, 확인용 그림 |

## `tire_cell_v3/` 에 들어 있는 것

| 항목 | 파일 |
|---|---|
| 그리퍼 로봇 (Kawasaki RS080N + 4조 휠 그리퍼) | `rs080n_pzv160_flat.usda`, `wheel_gripper_4jaw.usda`, 보기: `gripper_robot_scene.usda` |
| 너트러너 로봇 (RS080N + Atlas Copco QST62-230CT 치수 모델) | `nutrunner_qst62.usda`, 보기: `nutrunner_robot_scene.usda` |
| 로봇 셀 (로봇 2대 + 차 + 리프트 + 물리) | `tire_cell_v3.usda`, 설정값 `params.json` |
| 정비소 맵 | `garage_map.usda`, 셀과 합친 씬 `tire_cell_v3_map.usda` |
| 리프트 (자체 제작 더미) | 셀 씬 안에 포함, 보기: `lift_scene.usda` |
| 너트 보관 트레이 (모형만) | `nut_tray.usda`, 보기: `nut_tray_scene.usda` |
| 차량 4종 (아반떼 CN7, K5, 그랜저 GN7, K9) | `cars/*.usda`, 제원 `car_specs.json`, 설명 `CAR_ASSETS.md` |
| 시연 코드 | `cell_demo.py`, `cell_kinematics.py`, `run_cell_demo.py`, `run.sh` |
| 다시 만드는 스크립트 | `build_scene_v3.py`, `build_map.py`, `build_cars.py`, `car_body.py`, `build_tray.py`, `nut_tray.py` |

## 실행

```bash
cd tire_cell_v3
bash run.sh            # 시연 (로봇 2대, 정비소 맵 포함)
bash view.sh           # 볼 수 있는 씬 이름 목록
bash view.sh tray      # 예: 너트 트레이 보기.  cars / lift / nutrunner / gripper / map ...
```

`view.sh` 가 안 되면 Isaac Sim 을 켜고 `Ctrl + O` 로 `.usda` 파일을 직접 여세요.
Isaac Sim 이 `~/isaacsim` 이 아닌 곳에 있으면 앞에 `ISAACSIM_PATH=/경로/isaacsim` 을 붙입니다.

이전 시연은 `cd tire_demo_v2 && bash run.sh` 입니다.

## 확인 상태

- `tire_demo_v2`: Isaac Sim 에서 실행 확인됨.
- `tire_cell_v3` 시연: 토크 제한·보호 정지를 넣은 최신본은 아직 실행 결과를 확인하지 못함.
- 차량 4종, 트레이, 맵, `view.sh`: Isaac Sim 실행 검증 전.
