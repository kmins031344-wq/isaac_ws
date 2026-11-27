# 다른 PC 에서 실행하기 (2026-11-27 묶음)

> 통합 전 버전입니다. 최신본은 `../tire_cell_v7_integrated/` 입니다.

파일끼리 상대 경로로 연결돼 있어서 폴더째 어디에 두어도 됩니다. Isaac Sim 5.1 이 필요합니다.

```bash
cd tire_cell_v3
bash run.sh              # 시연 실행 (로봇 2대가 너트 풀고 휠을 빼서 내려놓음, 정비소 맵 포함)
bash view.sh cars        # 씬 보기만 (스크린샷용). 이름 목록:  bash view.sh
```

Isaac Sim 이 `~/isaacsim` 이 아닌 곳에 있으면 앞에 `ISAACSIM_PATH=/경로/isaacsim` 을 붙입니다.
`view.sh` 가 안 되면 Isaac Sim 을 먼저 켜고 `Ctrl + O` 로 아래 파일을 직접 엽니다.

| `view.sh` 이름 | 파일 | 내용 |
|---|---|---|
| `map` | `tire_cell_v3_map.usda` | 로봇 2대 + 차 + 리프트 + 정비소 맵 |
| `cell` | `tire_cell_v3.usda` | 위에서 맵 제외 |
| `cars` | `cars/cars_ground.usda` | 차량 4종 (아반떼 CN7, K5, 그랜저 GN7, K9), 바닥 위 |
| `cars_lift` | `cars/cars_lineup.usda` | 차량 4종, 받침대 위 |
| `avante` / `k5` / `grandeur` / `k9` | `cars/<이름>_test.usda` | 차량 1대씩 |
| `tray` | `nut_tray_scene.usda` | 너트 보관 트레이 (모형만) |
| `lift` | `lift_scene.usda` | 리프트만 |
| `nutrunner` | `nutrunner_robot_scene.usda` | 너트러너 로봇만 |
| `gripper` | `gripper_robot_scene.usda` | 휠 그리퍼 로봇만 |

차량 에셋 설명과 제원 출처는 `CAR_ASSETS.md`, 다시 만드는 스크립트는 `build_scene_v3.py`(셀), `build_map.py`(맵), `build_cars.py`(차량), `build_tray.py`(트레이) 입니다.
`view.sh` 와 차량·트레이·맵 씬은 Isaac Sim 에서 실행 검증을 하지 않았습니다.

---

# 타이어 교체 셀 v3 (Isaac Sim 5.1)

로봇 2대가 리프트 위 차량의 앞바퀴를 탈거하는 시연입니다.
- **로봇 B** (RS080N + Atlas Copco QST62-230CT 너트러너): 너트 5개 위치를 대각선 순서(1→3→5→2→4)로 돌며 스핀들을 회전시키고 빠집니다.
- **로봇 A** (RS080N + 4조 휠 그리퍼): 트레드를 잡아 바퀴를 스터드에서 빼고 바닥에 눕혀 놓습니다.

## 실행

```bash
bash run.sh                 # 기본: 너트를 푼 뒤 바퀴는 스터드 구속 관절(차축 방향으로만 이동)로 지지
bash run.sh --mount contact # 실험용: 허브 파일럿·스터드 '접촉'만으로 지지 (현재 바퀴가 4.9도 기울어지는 문제가 있음)
bash run.sh --grasp joint   # 그리퍼가 바퀴를 놓치면
bash run.sh --skip-nuts     # 너트러너 단계 생략
```
Stop → Play 를 누르면 처음부터 다시 실행됩니다. 대기·궤적 시간은 물리 스텝 시간으로 재므로 카메라 렌더링 등으로 화면이 느려져도 동작이 어긋나지 않습니다. 끝나면 이 폴더에 `contact_log.csv` 가 생깁니다.

## 정비소 맵
`bash run.sh` 는 `tire_cell_v3_map.usda` (셀 + 정비소 맵)를 엽니다. 맵 없이 셀만 보려면 `bash run.sh --no-map`.
맵(`garage_map.usda`)은 전부 더미 형상이고 **충돌이 없습니다** (외형만). 크기 17.5 m x 18.5 m, 천장 구조물 없음(위가 열림). 벽, 셔터, 기둥, 타이어 랙 2개(신규/탈거), 공구함, 작업대, 컴프레서, 로봇 제어기, 바닥 구획선이 있습니다.
AMR 은 쓰지 않으며, 나중에 넣을 수 있도록 왼쪽에 주행로(파란 점선)만 표시해 두었습니다. 리프트는 Isaac Sim 에 차량용 리프트 에셋이 확인되지 않아 더미를 그대로 씁니다.

## 이 버전은 Isaac Sim 에서 실행 검증이 되지 않았습니다
경로(도달 범위, 관절 한계, 간섭)는 계산으로 확인했지만, 물리 거동은 직접 시험해야 합니다. 터미널의 `[cell_demo]` 줄이 판단 근거입니다.

| 로그 | 의미 |
|---|---|
| `wheel moved after nut removal` | 너트를 푼 뒤 바퀴가 처진 양. 0.3 mm 안팎이 정상, 30 mm 이상이면 `--mount joint` |
| `jaw travel` | 조가 타이어에 닿아 멈췄는지 |
| `wheel travel along studs` | 바퀴가 그리퍼를 따라 빠졌는지 (30 mm 미만이면 `--grasp joint`) |
| `contact summary` | 공구-차체, 금속-휠 접촉 여부와 최대 힘 → 긁힘 가능성 |
| `car body max displacement / tilt` | 차체가 흔들린 양 |

## 물리 적용 범위

| 대상 | 충돌 | 질량 | 비고 |
|---|---|---|---|
| 차체 | 외판 삼각형 570개를 6 mm 두께 볼록판으로 | 1,210 kg (스프링 상) | 동적 강체. 리프트 패드 4곳에 접촉으로 얹힘 |
| 코너 4개 (허브·디스크·캘리퍼·너클) | FL 은 부품별, 나머지는 휠 포함 단순 형상 | 앞 23 kg, 뒤 18 kg (+휠 22 kg) | 서스펜션: 상하 직선 관절 + 스프링 25 N/mm, 감쇠 2,500 N·s/m |
| FL 휠 | 타이어 링 24조각, 허브 보어, 스터드 구멍 5개, 스포크 | 22 kg, 관성 지정 | 허브 파일럿과 스터드에 접촉으로 걸림 |
| 너트 5개 | 없음 | 없음 | 체결은 고정 관절 1개(`nut_clamp_joint`)로 표현. 나사·토크는 미구현 |
| 그리퍼 | 패드·턱·핑거·암 | 24 kg (추정) | 조 4개 직선 관절, 조임력 약 600 N/조 |
| 너트러너 | 몸통 | 12.2 kg (공구 10.2 + 브래킷 2.0) | 스핀들 회전 관절: 최대 230 N·m, 330 r/min. 소켓은 충돌 없음, 50 mm 이동은 미구현 |
| 리프트 | 패드, 런웨이, 베이스 | 고정 | |

모든 값은 `params.json` 에 있고, 값마다 출처를 적었습니다 (spec = 제조사 자료, asset = Isaac Sim 에셋, assumed = 가정, derived = 계산).
**가정값이 많습니다.** 특히 차량 질량 배분·서스펜션·마찰계수·그리퍼 질량은 실측이나 자료로 교체해야 합니다.

### 현실과 다른 점
- 허브 보어 틈을 0.3 mm 로 키웠습니다 (실제 약 0.05 mm). 시뮬레이션 접촉 안정용입니다.
- 브레이크는 잠긴 것으로 보고 허브를 너클에 고정했습니다 (바퀴가 돌지 않음).
- 로봇 관절 게인과 최대 힘은 Isaac Sim 에셋 값입니다. 실제 모터 토크는 공개 자료가 없어 넣지 못했습니다.
- 너트러너 반력(최대 230 N·m)은 발생하지 않습니다. RS080N 손목 JT6 허용 토크가 194 N·m 이므로 실제 적용 시 반력 바가 필요합니다.
- 셀 시연의 차체는 Prius 메시입니다. 차량 4종(CN7, K5, K9, 그랜저) 에셋은 `cars/` 에 따로 있지만 아직 셀 시연에 연결하지 않았습니다.

## 파일

| 파일 | 내용 |
|---|---|
| `params.json` | 물리값 전체 (수정 후 `build_scene_v3.py` 로 씬 재생성) |
| `tire_cell_v3.usda` | 셀 씬 (로봇 2대, 차량, 리프트, 물리) |
| `tire_cell_v3_map.usda`, `garage_map.usda`, `build_map.py` | 정비소 맵을 얹은 씬, 맵, 맵 생성 스크립트 |
| `cell_kinematics.py` | FK/IK (USD 관절 프레임에서 모델 생성, 수치 IK) |
| `cell_demo.py` | 시연 순서, 접촉 기록 |
| `run_cell_demo.py`, `run.sh` | 실행 |
| `wheel_gripper_4jaw.usda`, `nutrunner_qst62.usda` | 엔드이펙터 (치수 기반 모델, 제조사 CAD 아님) |
| `rs080n_pzv160_flat.usda` | RS080N (Isaac Sim 정식 에셋에서 추출) |
| `car/` | 차체 메시 (osrf/car_demo, Apache-2.0) |
| `build_scene_v3.py`, `build_scene_v2.py` | 씬 생성 스크립트 |

## 리니어 트랙 (1개, 캐리지 2개)
로봇 2대가 차 길이 방향(world Y) 레일 하나를 같이 씁니다 (`track_mount.py`, 값은 `params.json` 의 `track`).

| 항목 | 값 |
|---|---|
| 레일 | x = 0 선, y = -4.98 ~ 0.93 m (길이 5.91 m). 외형만 있고 충돌 없음 |
| 캐리지 | 로봇마다 1개, 윗면 높이 0.12 m (로봇 베이스가 그만큼 올라감), 질량 120 kg |
| 기준 위치 (트랙 0) | 로봇 A y = 0 (앞바퀴 앞), 로봇 B y = -1.25 (A 보다 차 뒤쪽). B 는 이전 (0.55, -1.25) 에서 레일 선 위 (0, -1.25) 로 옮김 |
| 이동 범위 | 각 로봇 -3.2 ~ +0.4 m (음수 = 차 뒤쪽), 최대 1.0 m/s |
| 최소 간격 | 캐리지 중심 사이 1.0 m (이보다 가까워지는 이동은 코드가 거부) |
| 관절 | `/World/RobotA/track_joint`, `/World/RobotB/track_joint` (직선 관절, 위치 구동 `drive:linear:physics:targetPosition` [m]) |

- 시연은 앞바퀴 작업이 끝나면 두 로봇이 트랙을 타고 뒷바퀴 위치(-2.86 m)로 이동합니다 (`T 1/2`, `T 2/2` 로그). 생략: `bash run.sh --no-track-move`.
- 코드에서는 `cell_demo.py` 의 `track_move(da, db, arms)` 가 두 캐리지를 움직이고, `Arm.set_track(d)` 가 FK/IK 기준 위치를 옮깁니다.
- 뒷바퀴는 씬에서 아직 분리되지 않는 단순 형상이라, 뒷바퀴 탈거 동작은 없습니다 (이동만).
- 높이·질량·속도·구동 게인은 가정값입니다 (특정 주행축 제품 아님).

## 손목 카메라 (Intel RealSense D455)
두 로봇 공구에 D455 (`sensors/rsd455/rsd455.usd`) 를 1대씩 달았습니다. 둘 다 공구 진입 방향(-Z)을 봅니다.

| 로봇 | 장착 위치 | 카메라 프림 |
|---|---|---|
| A 휠 그리퍼 | 센터 하우징 +X 옆면 (조 사이), 플랜지에서 15~41 mm 아래 | `/World/RobotA/WheelGripper/Camera_RSD455/RSD455` |
| B 너트러너 | 브래킷 클램프 -X 옆면 (케이블 반대편), 플랜지에서 307~333 mm 아래 | `/World/RobotB/Nutrunner/Camera_RSD455/RSD455` |

- 그 안의 `Camera_OmniVision_OV9782_Color / _Left / _Right`, `Camera_Pseudo_Depth`, `Imu_Sensor` 를 그대로 씁니다. 경로는 `/World` 의 customData `cell:camera_a`, `cell:camera_b` 에도 적혀 있습니다.
- 카메라는 0.103 kg 강체이고 고정 관절(`Camera_RSD455_mount_joint`)로 공구 base 에 붙어 함께 움직입니다 (IMU 가 이 강체를 읽음). 외형만 있고 충돌체는 없습니다.
- 위치는 `camera_mount.py` 의 `MOUNTS` 에서 바꿉니다. 바꾼 뒤 `python camera_mount.py` (Isaac Sim python) 로 기존 씬에 다시 붙이거나, `build_scene_v3.py` 로 씬을 새로 만들면 자동으로 붙습니다.

### ROS 2 토픽 (기본으로 켜짐)
`bash run.sh` 를 실행하면 두 카메라 영상이 ROS 2 토픽으로 나옵니다 (`ros2_cameras.py`, Isaac Sim ROS 2 브리지). 끄려면 `bash run.sh --no-ros`.

| 토픽 | 형식 |
|---|---|
| `/robot_a/camera/color/image_raw`, `/robot_b/...` | `sensor_msgs/Image` rgb8, 기본 640x400 (`--ros-res 1280x800` 로 변경) |
| `/robot_a/camera/color/camera_info`, `/robot_b/...` | 내부 파라미터 (fx = fy ≈ 317 @ 640x400) |
| `/robot_a/camera/depth/image_raw`, `/robot_b/...` | `sensor_msgs/Image` 32FC1, 단위 m |
| `/robot_a/camera/depth/camera_info`, `/robot_b/...` | |
| `/tf` | `World` → `robot_a_color`, `robot_a_depth`, `robot_b_color`, `robot_b_depth` (ROS 광학 좌표계: +Z 가 보는 방향, +Y 가 영상 아래) |
| `/clock` | 시뮬레이션 시간. 받는 쪽은 `use_sim_time:=true` |

확인 (다른 터미널, 같은 ROS_DOMAIN_ID):
```bash
source /opt/ros/jazzy/setup.bash      # = ros_set
ros2 topic list
ros2 topic hz /robot_b/camera/color/image_raw
ros2 run rqt_image_view rqt_image_view
```
- `run.sh` 가 Isaac Sim 내장 Jazzy 라이브러리 경로를 직접 등록하므로 `isaac_ros` 를 따로 실행할 필요가 없습니다. `ROS_DOMAIN_ID` 는 환경 변수(`~/.bashrc`)를 따르고, 설정돼 있지 않으면 136 을 씁니다. Isaac Sim 을 띄우는 터미널에서는 `ros_set`(시스템 ROS source)을 하지 않는 편이 안전합니다.
- 영상 4개를 매 프레임 렌더링하므로 실제 시간 기준으로는 시뮬레이션이 느려집니다 (시험 PC 에서 영상 약 6~7 Hz). 동작 타이밍은 물리 시간으로 재므로 결과는 같습니다.
- 깊이 영상은 D455 에셋의 `Camera_Pseudo_Depth` (렌더러 깊이) 입니다. 실제 스테레오 깊이의 노이즈·구멍은 없습니다.

## 확장 지점
- **IK 교체**: `cell_kinematics.Arm.ik(T_target, seed) -> (q, ok)` 와 같은 형태면 Lula, cuRobo, MoveIt 등으로 바꿀 수 있습니다.
- **구동 방식 교체**: `cell_demo.py` 의 `UsdDriveBackend.command(q, sign)` 만 바꾸면 ROS 2 토픽/액션이나 토크 제어로 넘어갈 수 있습니다.
- **로봇 기종 교체**: 관절 축·한계·속도를 USD 에서 읽으므로 RS025N 등으로 바꿔도 기구학 코드는 그대로입니다.
- **차종 추가**: `params.json` 의 `car`, `wheel` 값과 차체 메시만 바꾸면 됩니다.
- **목표 좌표**: 지금은 시뮬레이션의 바퀴 자세를 직접 읽습니다 (`body_pose(WHEEL)`). 손목 카메라(D455) 인식 결과로 바꾸는 자리입니다.
