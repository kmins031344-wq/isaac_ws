# 시작 가이드 (Getting Started)

> 저장소를 clone 받아 Isaac Sim 에서 타이어 교체 시연을 실행하기까지의 절차입니다. 위에서부터 순서대로 따라 하세요.

## 0. 준비물

| 항목 | 기준 |
|---|---|
| OS | Ubuntu 24.04 |
| 시뮬레이터 | Isaac Sim 5.1 (`~/isaacsim`) |
| GPU | NVIDIA RTX |
| ROS 2 | Jazzy (토픽 확인용, 선택) |
| 기타 | Git, GitHub 계정 (push 하려면 저장소 권한 필요) |

인터넷 연결은 필요 없습니다. 로봇·센서·차체 에셋이 저장소에 들어 있습니다.

---

## 1. 저장소 받기

```bash
cd ~
git clone https://github.com/kmins031344-wq/isaac_ws.git
cd isaac_ws
```

각 버전 폴더는 파일끼리 상대 경로로 연결돼 있어서 어디에 clone 해도 됩니다.

---

## 2. Isaac Sim 경로 확인

`run.sh` 는 기본으로 `~/isaacsim` 을 찾습니다.

```bash
ls ~/isaacsim/isaac-sim.sh
```

다른 곳에 설치했다면 실행할 때 앞에 경로를 붙입니다.

```bash
ISAACSIM_PATH=/경로/isaacsim bash run.sh
```

---

## 3. 시연 실행

```bash
cd tire_twin_all/tire_cell_v7_integrated
bash run.sh                    # 전체: 비전 → 분해 → 재조립 → 트랙으로 뒷바퀴 위치 이동, 카메라 토픽 포함
bash run.sh --no-ros           # 카메라 토픽 없이 (더 빠름)
bash run.sh --no-reassembly    # 휠을 바닥에 놓고 끝 (재조립 생략)
bash run.sh --no-track-move    # 마지막 트랙 이동 생략
bash run.sh --no-vision        # 비전 없이 물리 자세로 너트 좌표 사용
bash run.sh --no-map           # 정비소 맵 없이 셀만
```

- Isaac Sim 창이 뜨고 씬 열기 → Play → 시연까지 자동으로 진행됩니다.
- **Stop(■) 후 Play(▶)** 를 누르면 처음부터 다시 실행됩니다.
- 끝나면 폴더에 `contact_log.csv`, `vision_debug.png` 가 생깁니다.

비전 견고성 시연 (차·휠을 배치도에서 어긋나게 놓기):

```bash
bash run.sh --no-ros --car-offset 0.01,0.03,0 --vision-noise 1.0
bash run.sh --no-ros --wheel-rotate 20 --vision-noise 1.0
```

### 실행이 잘 됐는지 보는 법

터미널의 `[cell_demo]` 줄을 확인합니다.

| 로그 | 의미 |
|---|---|
| `wheel moved after nut removal` | 너트를 푼 뒤 바퀴가 처진 양. 0.3 mm 안팎이 정상 |
| `wheel travel along studs` | 바퀴가 그리퍼를 따라 빠졌는지 (30 mm 미만이면 `--grasp joint`) |
| `wheel mount error` | 재장착 후 원래 허브 자세와의 차이 |
| `FALLBACK` | 비전 검출 실패로 물리 자세를 대신 사용함 |
| `contact summary` | 공구-차체, 금속-휠 접촉 여부와 최대 힘 |

---

## 4. 씬만 보기

```bash
bash view.sh             # 볼 수 있는 씬 이름 목록
bash view.sh cars        # 차량 4종
bash view.sh tray        # 너트 트레이
```

이름: `map`, `cell`, `cars`, `cars_lift`, `avante`, `k5`, `grandeur`, `k9`, `tray`, `lift`, `nutrunner`, `gripper`.
`view.sh` 가 안 되면 Isaac Sim 을 켜고 `Ctrl + O` 로 `.usda` 파일을 직접 엽니다.

---

## 5. ROS 2 토픽 확인 (선택)

`bash run.sh` (`--no-ros` 없이) 를 실행하면 두 손목 카메라 영상이 ROS 2 토픽으로 나옵니다. `ROS_DOMAIN_ID` 는 `~/.bashrc` 값을 따르고, 없으면 136 입니다.

Isaac Sim 을 띄운 터미널이 아닌 **다른 터미널**에서, 같은 `ROS_DOMAIN_ID` 로:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic hz /robot_b/camera/color/image_raw
ros2 run rqt_image_view rqt_image_view
```

Isaac Sim 을 띄우는 터미널에서는 시스템 ROS 를 source 하지 않는 편이 안전합니다. 토픽 목록은 [`architecture.md`](architecture.md#ros-2-토픽) 참고.

---

## 6. 작업 시작

작업 브랜치를 만들고 PR 을 보내는 절차는 [`branching.md`](branching.md) 를 따르세요.
