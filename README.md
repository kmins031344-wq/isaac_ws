# 타이어 교체 디지털트윈 (Isaac Sim)

정비소에서 하는 **타이어(휠) 교체 작업을 로봇 2대가 자동으로 수행하는 셀**을 NVIDIA Isaac Sim 5.1 로 만든 디지털트윈입니다.

![정비소 맵](docs/map_preview.png)

## 무엇을 하나

차를 리프트에 올린 상태에서 로봇 두 대가 나눠 일합니다.

| 로봇 | 구성 | 하는 일 |
|---|---|---|
| **로봇 A (그리퍼)** | Kawasaki RS080N + 4조 휠 그리퍼 | 휠을 잡아 허브에서 빼고, 다시 끼움 |
| **로봇 B (너트러너)** | Kawasaki RS080N + Atlas Copco QST62-230CT 치수 모델 | 휠 너트 5개를 풀어 트레이에 보관하고, 다시 꺼내 별 모양 순서로 조임 |

작업 순서 (최신 `tire_cell_v7_integrated` 기준)

1. 로봇 B 손목 카메라(RealSense D455)로 휠을 한 번 촬영해 **너트 5개 위치와 허브 축을 비전으로 측정**
2. 로봇 B 가 너트 5개를 풀어 너트 트레이에 보관
3. 로봇 A 가 휠을 탈거해 바닥에 내려놓음
4. 로봇 A 가 휠을 다시 허브에 장착
5. 로봇 B 가 트레이에서 너트를 회수해 조임
6. 리니어 트랙으로 뒷바퀴 위치까지 이동

카메라 영상은 ROS 2 토픽으로 내보낼 수 있습니다.

### 측정 결과 (Isaac Sim 5.1, 화면 없이 실행)

- 전체 사이클 약 322 s (시뮬레이션 시간), 보호 정지 없음
- 재조립 후 최종 휠 자세 오차 약 0.6 mm / 0.01°
- 비전 너트 위치 오차 최대 0.3 mm 이하 (차를 몇 cm 옮기거나 휠을 ±20° 돌려도)

자세한 수치와 한계는 [tire_cell_v7_integrated/README.md](tire_twin_all/tire_cell_v7_integrated/README.md) 를 보세요.

## 폴더 구조

```
isaac_ws/
├── docs/                          발표용 정리 PDF, 확인용 그림
│   └── camera_check/              손목 카메라 확인 캡처
└── tire_twin_all/
    ├── tire_cell_v7_integrated/   최신 통합본 (조립·분해·너트 보관 + 비전 + ROS 2 + 트랙)
    ├── tire_cell_v3/              통합 전 본 (카메라·ROS 2·트랙)
    └── tire_demo_v2/              초기 시연 (로봇 1대가 휠만 빼서 내려놓음)
```

주요 에셋 (각 버전 폴더 안)

- 로봇: `rs080n_pzv160_flat.usda`, `wheel_gripper_4jaw.usda`, `nutrunner_qst62.usda`
- 셀·맵: `tire_cell_v3.usda`, `garage_map.usda`, `tire_cell_v3_map.usda`
- 차량 4종 (아반떼 CN7, K5, 그랜저 GN7, K9): `cars/*.usda`, 제원 `car_specs.json`
- 리프트, 너트 트레이: `lift_scene.usda`, `nut_tray.usda`
- 씬을 다시 만드는 스크립트: `build_*.py`

## 실행

Isaac Sim 5.1 이 `~/isaacsim` 에 설치되어 있어야 합니다. 다른 경로면 앞에 `ISAACSIM_PATH=/경로/isaacsim` 을 붙이세요.

```bash
cd tire_twin_all/tire_cell_v7_integrated
bash run.sh                 # 전체 시연 (비전 + 카메라 ROS 2 토픽 포함)
bash run.sh --no-ros        # ROS 토픽 없이 (더 빠름)
bash run.sh --no-vision     # 비전 없이 물리 자세로 너트 좌표 사용
bash view.sh                # 볼 수 있는 씬 목록 (cars / lift / tray / map ...)
```

`view.sh` 가 안 되면 Isaac Sim 에서 `Ctrl + O` 로 `.usda` 파일을 직접 여세요.

## 확인 상태

- `tire_demo_v2`: Isaac Sim 에서 실행 확인
- `tire_cell_v7_integrated`: `--no-ros` 전체 사이클 실행 확인. ROS 토픽을 켠 전체 사이클은 아직 미확인
- 차량 4종, 트레이, 맵, `view.sh`: Isaac Sim 실행 검증 전

## 문서

- [docs/타이어교체_디지털트윈_정리.pdf](docs/타이어교체_디지털트윈_정리.pdf): 발표용 정리 자료
- [tire_twin_all/README.md](tire_twin_all/README.md): 버전별 파일 목록
