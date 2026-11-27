# 시스템 아키텍처

> 최신본 `tire_twin_all/tire_cell_v7_integrated/` 기준입니다. 세부 수치와 표는 그 폴더의 [README](../tire_twin_all/tire_cell_v7_integrated/README.md) 에 있습니다.

## 1. 셀 구성

| 구성 | 내용 |
|---|---|
| 로봇 A (그리퍼) | Kawasaki RS080N + 4조 휠 그리퍼. 휠을 허브에서 빼서 바닥에 놓고, 다시 들어 원래 허브 위치에 장착 |
| 로봇 B (너트러너) | Kawasaki RS080N + Atlas Copco QST62-230CT 치수 모델. 너트 5개를 별 모양 순서(1→3→5→2→4)로 풀고 조임 |
| 손목 카메라 | Intel RealSense D455 × 2 (로봇마다 1대). 지금 비전에는 로봇 B 카메라만 사용 |
| 리니어 트랙 | 차 길이 방향 레일 1개, 캐리지 2개. 앞바퀴 → 뒷바퀴 위치(-2.86 m) 이동 |
| 차량·리프트 | 차체(Prius 메시) + 코너 4개 서스펜션, 리프트는 자체 제작 더미 |
| 너트 트레이 | 풀린 너트 5개 보관 |
| 정비소 맵 | 17.5 m × 18.5 m 더미 형상 (충돌 없음, 외형만) |

차량 4종(아반떼 CN7, K5, 그랜저 GN7, K9) 에셋은 `cars/` 에 있지만 아직 셀 시연에는 연결하지 않았습니다.

## 2. 작업 순서

```text
[비전] 로봇 B 관측 자세 → D455 컬러+깊이 1장 → 너트 5개 위치·허브 축 측정
   ↓
[분해] 로봇 B: 너트 5개 풀기 → 트레이 슬롯에 보관
       로봇 A: 휠 잡기 → 스터드에서 빼기 → 바닥에 놓기
   ↓  2초 대기
[조립] 로봇 A: 바닥 휠 픽업 → 측정한 허브 자세로 재장착
       로봇 B: 트레이에서 너트 회수 → 별 모양 순서로 조임
   ↓
[이동] 두 로봇이 트랙을 타고 뒷바퀴 위치로 이동
```

모든 대기·궤적 시간은 물리 시간 기준이라 렌더링이 느려져도 동작이 어긋나지 않습니다.

## 3. 비전 너트 검출

`nut_vision.py` (처리 약 0.1 s), 촬영은 `vision_capture.py` (1280×800, 약 0.3 s).

1. 깊이 top-hat 으로 튀어나온 작은 덩어리를 찾고 크기·모양으로 스포크를 거름
2. PCD 114.3 원과 72° 배치에 맞는 후보만 남김 (RANSAC)
3. 장착면 점에 평면을 맞춰 허브 축을 구함
4. 휠 자세 4×4 → 너트 5개 위치 계산. 두 로봇 모두 이 값으로 경로를 계획 (보고 나서 움직이기)

너트 3개 미만 검출, 잔차 RMS 2 mm 초과, 배치도에서 8 cm 초과, 축 기울기 5° 초과면 `FALLBACK` 으로 물리 자세를 사용합니다.
측정 결과: 차 위치를 몇 cm 옮기거나 휠을 ±20° 돌려도 너트 오차 최대 0.3 mm 이하.

## 4. 코드 구조

| 파일 | 역할 |
|---|---|
| `run.sh`, `run_cell_demo.py` | 실행 진입점 (Isaac Sim 실행, 씬 열기, Play) |
| `cell_demo.py` | 시연 순서, 접촉 기록, 트랙 이동 (`track_move`) |
| `cell_kinematics.py` | FK/IK (USD 관절 프레임에서 모델 생성, 수치 IK) |
| `nut_vision.py`, `vision_capture.py`, `vision_ros.py` | 비전 |
| `ros2_cameras.py` | 카메라 → ROS 2 토픽 |
| `track_mount.py`, `camera_mount.py` | 트랙, 카메라 장착 |
| `params.json` | 물리값 전체 (값마다 출처: spec / asset / assumed / derived) |
| `build_*.py` | 씬·맵·차량·트레이 재생성 |

확장 지점: IK 는 `Arm.ik(T_target, seed) -> (q, ok)` 형태면 Lula·cuRobo·MoveIt 으로, 구동은 `UsdDriveBackend.command(q, sign)` 만 바꾸면 ROS 2 액션·토크 제어로 교체할 수 있습니다.

## 5. ROS 2 토픽

| 토픽 | 형식 |
|---|---|
| `/robot_a/camera/color/image_raw`, `/robot_b/...` | `sensor_msgs/Image` rgb8, 기본 640×400 |
| `/robot_a/camera/color/camera_info`, `/robot_b/...` | 내부 파라미터 |
| `/robot_a/camera/depth/image_raw`, `/robot_b/...` | `sensor_msgs/Image` 32FC1, 단위 m |
| `/tf` | `World` → 각 카메라 광학 좌표계 |
| `/clock` | 시뮬레이션 시간 (`use_sim_time:=true`) |

## 6. 모델 한계

- 너트 나사산·체결 토크는 계산하지 않고 고정 관절 1개(`nut_clamp_joint`)로 표현
- 너트러너 반력(최대 230 N·m) 미구현. 실제로는 RS080N 손목 허용 토크(194 N·m) 때문에 반력 바 필요
- 시뮬레이션 깊이는 노이즈가 없음. 실기에서는 금속 반사로 깊이가 빠질 수 있음
- 카메라 자세는 물리 자세 그대로 사용 (완벽한 핸드-아이 보정 가정)
- 차량 질량 배분·서스펜션·마찰·그리퍼 질량 등 가정값이 많음 (`params.json` 의 `assumed`)
