# 타이어 탈거 시연 (Isaac Sim)

RS080N 로봇이 리프트 위 차량의 앞바퀴(너트 없음)를 4조 그리퍼로 잡아 빼서 바닥에 내려놓는 시연입니다.

## 실행

```bash
cd tire_twin_all/tire_demo_v2      # 저장소 최상위 기준
bash run.sh
```

- Isaac Sim 창이 뜨고 씬 열기 → Play → 시연까지 자동으로 진행됩니다 (약 45초).
- **Stop(■) 후 Play(▶)를 누르면 처음부터 다시 실행됩니다.**
- Isaac Sim 이 `~/isaacsim` 이 아닌 곳에 설치돼 있으면: `ISAACSIM_PATH=/설치/경로 bash run.sh`

옵션
- `--grasp joint` : 접촉 파지가 실패할 때 쓰는 예비 방식 (조를 닫은 뒤 고정 관절로 묶음)
- `--speed 1.5`   : 동작 속도 배율
- `--exit`        : 시연이 끝나면 자동 종료

## 필요 환경
- Isaac Sim 5.x (5.1.0 에서 작성). 인터넷 연결은 필요 없습니다 (로봇 에셋 포함).
- NVIDIA RTX GPU

## 파일
| 파일 | 내용 |
|---|---|
| `run.sh` | 실행 스크립트 (Isaac Sim 위치 자동 탐색) |
| `run_tire_demo_v2.py` | Isaac Sim 실행 + 씬 열기 + Play |
| `tire_demo_v2.py` | 시연 동작 (FK/IK, 순서 제어). Script Editor 에서 직접 실행해도 됨 |
| `tire_demo_scene_v2.usda` | 시연 씬 |
| `rs080n_pzv160_flat.usda` | Kawasaki RS080N (Isaac Sim 정식 에셋에서 추출) |
| `wheel_gripper_4jaw.usda` | 4조 휠 그리퍼 (치수 기반 개념 설계, 제조사 CAD 아님) |
| `car/` | 차체 메시 (osrf/car_demo Prius, Apache-2.0) |
| `build_scene_v2.py` | 씬 생성 스크립트 (참고용) |

## 수치
- 휠+타이어 22 kg (17인치, 5x114.3), 그리퍼 24 kg(추정) → 합계 46 kg / RS080N 가반하중 80 kg
- 손목(JT5) 정적 모멘트 약 159 N·m / 허용 336 N·m

## 문제가 생기면
터미널의 `[tire_demo]` 로 시작하는 줄을 확인하세요.
- `jaw travel` : 조가 타이어에 닿아 멈췄는지
- `wheel travel along studs` : 바퀴가 그리퍼를 따라 빠졌는지 (30 mm 미만이면 파지 실패 → `--grasp joint`)
- `wheel offset from TCP while carried` : 운반 중 미끄러짐
