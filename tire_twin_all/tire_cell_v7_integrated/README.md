# 타이어 교체 셀 v7 통합본 (2026-11-27)

두 갈래 작업을 합친 폴더입니다.
- **조립·분해·너트 보관** (`tire_cell_v6_reverse`): 너트 5개를 풀어 트레이에 보관 → 휠 탈거 → 2초 대기 → 휠 재장착 → 트레이 너트를 회수해 조임
- **카메라·ROS 2·트랙** (`tire_cell_v3` 수정본): 손목 카메라 D455 2대, ROS 2 토픽(도메인은 `ROS_DOMAIN_ID`, 없으면 136), 리니어 트랙 1개, 시뮬레이션 시간 기준 타이밍

```bash
cd tire_twin_all/tire_cell_v7_integrated      # 저장소 최상위 기준
bash run.sh                    # 전체: 분해 → 재조립 → 트랙으로 뒷바퀴 위치 이동, 카메라 토픽 포함
bash run.sh --no-ros           # 카메라 토픽 없이 (더 빠름)
bash run.sh --no-reassembly    # 휠을 바닥에 놓고 끝 (재조립 생략)
bash run.sh --no-track-move    # 마지막 트랙 이동 생략
```

합치면서 바뀐 점
- 로봇 B 는 트랙 위 (0, -1.25, 0.12) 에 있습니다 (v6 에서는 (0.55, -1.25, 0)). 로봇 A 도 캐리지 높이 0.12 m 만큼 올라갔습니다.
- 너트 트레이를 x 0.75 → 0.85 로 10 cm 옮겼습니다 (레일과 4 cm 겹쳐서).
- v6 시연 코드의 대기·사이클 시간을 벽시계에서 물리 시간으로 바꿨습니다. `cycle time` 은 시뮬레이션 시간입니다.
- 트랙 이동은 재조립이 끝난 뒤 맨 마지막에 합니다.

## 비전으로 너트 찾기 (2026-11-27 추가, 기본 켜짐)

너트 좌표를 휠의 물리 자세에서 바로 가져오던 것을, **로봇 B 손목 D455 로 한 번 보고 찾는 방식**으로 바꿨습니다.

1. 로봇 B 가 관측 자세로 갑니다. 배치도상의 휠 축 위에서 카메라가 너트 윗면 0.55 m 앞에 오는 자세입니다.
2. 컬러와 깊이를 한 장 찍습니다. 렌더링은 촬영하는 순간에만 켭니다 (1280x800, 약 0.3 s).
3. `nut_vision.py` 가 처리합니다 (약 0.1 s).
   - 깊이 top-hat 으로 튀어나온 작은 덩어리를 찾고, 크기·모양으로 스포크를 거릅니다.
   - PCD 114.3 원과 72° 배치에 맞는 후보만 남깁니다 (RANSAC).
   - 장착면 점 수천 개에 평면을 맞춰 허브 축을 구합니다.
   - 휠 자세 4x4 를 구하고, 너트 5개 위치는 휠 자세 × 너트 로컬 좌표로 계산합니다.
4. 두 로봇 모두 이 측정값으로 경로를 계획합니다: 로봇 B 는 너트, 로봇 A 는 휠 중심과 축. 그 뒤에는 비전을 다시 돌리지 않습니다 (보고 나서 움직이기).
5. 다음 경우에는 로그에 `FALLBACK` 을 남기고 예전처럼 물리 자세를 씁니다.
   - 너트가 3개 미만으로 맞음
   - 패턴 잔차 RMS 가 2 mm 를 넘음
   - 결과가 배치도에서 8 cm 넘게 떨어짐
   - 축이 5° 넘게 기울어짐

```bash
bash run.sh --no-ros                                   # 비전 켜짐 (기본)
bash run.sh --no-ros --car-offset 0.01,0.03,0 --vision-noise 1.0   # 차를 배치도에서 옮겨 놓고 + 깊이 노이즈: 비전 효과 시연
bash run.sh --no-ros --wheel-rotate 20 --vision-noise 1.0         # 휠+허브를 20도 돌려 놓고: 너트가 배치도 위치에 없음
bash run.sh --no-vision                                # 예전 방식
cd ~/isaacsim && ./python.sh <이 폴더>/test_vision_isaac.py --headless --no-map --car-offset 0,0.03,0   # 비전만 40초 시험
python3 nut_vision.py out.png                          # Isaac 없이 알고리즘 자체 시험 (합성 깊이)
```

실행할 때마다 `vision_debug.png` 에 검출 결과 영상이 저장됩니다. 초록 십자는 맞춘 너트와 번호, 파란 점과 화살표는 허브 중심과 축, 빨간 부분은 걸러낸 스포크입니다.
로그의 `check vs simulation truth` 줄은 시뮬레이션 정답과 비교한 평가용 값이고, 계획에는 쓰지 않습니다.

측정 결과 (Isaac Sim 5.1, 화면 없이)

| 조건 | 너트 오차 최대 | 축 오차 | 고정 좌표였다면 |
|---|---|---|---|
| 차 배치도 위치 | 0.11 mm | 0.000° | 0 mm |
| 차 (1, 3, 0) cm 이동 + 깊이 노이즈 1 mm, 전체 사이클 | 0.15 mm | 0.025° | 31.6 mm |
| 차 (2, -4, 1.5) cm 이동 + 노이즈 1.5 mm (너트 4개만 검출) | 0.29 mm | 0.037° | 44.7 mm |
| 휠 +20° 회전 + 노이즈 1 mm (비전이 찾은 회전 19.98°) | 0.05 mm | 0.032° | 20.0 mm |
| 휠 −15° 회전 + 노이즈 1 mm (비전이 찾은 회전 −14.92°) | 0.12 mm | 0.019° | 14.9 mm |
| 휠 +20° 회전 + 노이즈 1 mm, 전체 사이클 | 0.19 mm | 0.015° | 19.9 mm |

마지막 줄은 휠 위치를 4.5 cm 바꿔서 비전 단독으로 시험한 결과입니다 (`test_vision_isaac.py`). 이렇게 Z 방향으로 옮긴 조건에서 전체 사이클은 돌려 보지 않았습니다.

`--wheel-rotate` 는 휠과 허브(스터드·너트)를 같이 돌리고, 캘리퍼·스트럿 등 안 도는 부품은 제자리에 둡니다 (`vision_capture.rotate_wheel`).
휠 +20° 전체 사이클: 탈거 76.7 mm → 재장착 오차 2.7 mm → 최종 0.6 mm / 0.01°, 보호 정지 없음, 322 s.

전체 사이클 (비전 + 차 1/3 cm 이동 + 노이즈 1 mm): 풀기 → 탈거 76.7 mm → 재장착 오차 2.7 mm → 조임 후 최종 휠 자세 오차 0.5 mm / 0.01° → 트랙 이동. 보호 정지 없음, 사이클 322 s (비전 단계 추가 후에도 같음).

한계와 다음 단계
- 시뮬레이션 깊이는 오차가 없습니다. 실제 D455 는 금속 반사 때문에 너트 윗면 깊이가 빠질 수 있으니 `--vision-noise` 로 견고성을 확인하고, 실기에서는 프레임을 여러 장 평균하세요 (`VISION_FRAMES`).
- 카메라 자세는 카메라 강체의 물리 자세를 씁니다 (= 완벽한 핸드-아이 보정). 실기에서는 FK × 핸드-아이 보정 행렬로 바꿔야 합니다 (`vision_capture.WristCamera.pose`).
- 너트 번호는 휠이 배치도 대비 ±36° 안에서 돌아가 있다고 가정하고 붙입니다. 조임 순서(별 모양)는 번호와 상관없이 맞습니다.
- 로봇 A 손목 카메라는 아직 쓰지 않습니다. 너트를 푼 뒤 A 로 한 번 더 보면 휠 처짐까지 반영할 수 있습니다.

---

Isaac Sim 5.1 실행 확인 (`--no-ros`, 화면 없이): 너트 5개 트레이 보관 → 휠 탈거 76.5 mm → 재장착 오차 2.5 mm → 너트 5개 조임 후 최종 휠 자세 오차 0.6 mm / 0.01° → 트랙 -2.860 m 이동 오차 0.17 mm. 보호 정지 없음, 사이클 322 s(시뮬레이션 시간).
ROS 토픽을 켠 상태의 전체 사이클은 아직 돌려 보지 않았습니다.

---

# (이력) 타이어 교체 셀 v5: 분해 후 재조립

v7 에 합쳐진 v5 (`tire_cell_v5_assembly/`, 이 저장소에는 없음) 설명입니다. v5 는 너트를 풀고 휠을 바닥에 내려놓는 동작에 이어, 같은 휠을 다시 허브에 장착하고 트레이의 너트를 회수해 별 모양 순서로 조이는 버전입니다. Isaac Sim 5.1이 필요합니다.

```bash
cd tire_twin_all/tire_cell_v7_integrated      # 저장소 최상위 기준
bash run.sh
```

Isaac Sim이 기본 경로에 없으면 `ISAACSIM_PATH=/경로/isaacsim bash run.sh`로 실행합니다. `--grasp joint`는 바닥 휠 픽업/운반이 물리 접촉만으로 되지 않을 때 사용하는 고정 조인트 대체 모드이고, `--skip-nuts`는 너트 조작을 건너뛰되 휠 재장착은 계속합니다.

## v5 조립 순서와 모델 범위

1. 로봇 A가 바닥의 휠을 잡고 원래 측정한 허브 자세로 되돌립니다.
2. 로봇 B가 트레이 슬롯에서 너트를 하나씩 회수해 휠에 조입니다.
3. 다섯 너트가 모두 돌아오면 너트 고정 조인트를 다시 활성화합니다.

너트의 나사산과 실제 체결 토크는 물리적으로 계산하지 않습니다. 트레이/소켓의 너트는 시각화 형상이고, 다섯 너트의 체결 상태는 `nut_clamp_joint` 하나로 표현합니다. 따라서 Isaac Sim에서의 실제 동작은 별도로 실행해 확인해야 합니다.

## 씬/물리 베이스라인 (v3 기반)

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

# 씬 베이스라인 설명 (Isaac Sim 5.1)

기존 씬과 로봇 모델 설명입니다. v5에서는 아래 분해 동작 뒤에 바닥 휠의 재장착과 너트 회수가 추가됩니다.
- **로봇 B** (RS080N + Atlas Copco QST62-230CT 너트러너): 너트 5개를 별 모양 순서(1→3→5→2→4)로 풀고, 재조립 때 같은 순서로 조입니다.
- **로봇 A** (RS080N + 4조 휠 그리퍼): 휠을 허브에서 빼서 바닥에 놓은 다음 다시 들어 원래 허브 위치에 장착합니다.

## 실행

```bash
bash run.sh                 # 기본: 휠 분해 후 재장착, 트레이 너트 회수·조임까지 실행
bash run.sh --mount contact # 실험용: 허브 파일럿·스터드 '접촉'만으로 지지 (현재 바퀴가 4.9도 기울어지는 문제가 있음)
bash run.sh --grasp joint   # 그리퍼가 휠을 놓치면 고정 조인트 파지로 실행
bash run.sh --skip-nuts     # 너트 작업만 건너뛰고 휠 분해·재장착 실행
```
Stop → Play 를 누르면 처음부터 다시 실행됩니다. 대기·궤적 시간은 물리 스텝 시간으로 재므로 카메라 렌더링 등으로 화면이 느려져도 동작이 어긋나지 않습니다. 끝나면 이 폴더에 `contact_log.csv` 가 생깁니다.

## 정비소 맵
`bash run.sh` 는 `tire_cell_v3_map.usda` (셀 + 정비소 맵)를 엽니다. 맵 없이 셀만 보려면 `bash run.sh --no-map`.
맵(`garage_map.usda`)은 전부 더미 형상이고 **충돌이 없습니다** (외형만). 크기 17.5 m x 18.5 m, 천장 구조물 없음(위가 열림). 벽, 셔터, 기둥, 타이어 랙 2개(신규/탈거), 공구함, 작업대, 컴프레서, 로봇 제어기, 바닥 구획선이 있습니다.
AMR 은 쓰지 않으며, 나중에 넣을 수 있도록 왼쪽에 주행로(파란 점선)만 표시해 두었습니다. 리프트는 Isaac Sim 에 차량용 리프트 에셋이 확인되지 않아 더미를 그대로 씁니다.

## 실행 로그 읽는 법
v7 에서 `--no-ros` 전체 사이클은 Isaac Sim 실행으로 확인했습니다 (맨 위 결과 참고). 다른 옵션 조합에서는 터미널의 `[cell_demo]` 줄로 판단합니다.

| 로그 | 의미 |
|---|---|
| `wheel moved after nut removal` | 너트를 푼 뒤 바퀴가 처진 양. 0.3 mm 안팎이 정상, 30 mm 이상이면 `--mount joint` |
| `jaw travel` | 조가 타이어에 닿아 멈췄는지 |
| `wheel travel along studs` | 바퀴가 그리퍼를 따라 빠졌는지 (30 mm 미만이면 `--grasp joint`) |
| `A assembly wheel pickup` / `wheel mount error` | 바닥 휠이 들렸는지, 원래 허브 자세로 돌아왔는지 |
| `B assembly ... tightened` | 트레이 너트를 회수해 휠 위치에 조였는지 |
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
