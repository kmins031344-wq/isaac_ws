# 2026-11-27 비전 검출 영상 및 전체 사이클 실행 확인

## 확인한 것

- `tire_cell_v7_integrated` 에서 `bash run.sh` (카메라 ROS 2 토픽 켬) 로 전체 사이클 실행: 비전 → 너트 풀기·트레이 보관 → 휠 탈거 → 재장착 → 너트 조임 → 트랙 이동
- 비전 너트 검출 결과 영상 `vision_debug.png` 확인

## 문서 반영

- 루트 README, `tire_twin_all/README.md`, v7 README, `docs/history/README.md` 의 "ROS 토픽 켠 전체 사이클 미확인" 을 확인으로 변경
- `docs/architecture.md` 에 비전 검출 결과 영상 추가

## 남은 확인

- ROS 토픽 켠 실행의 수치(재장착 오차, 사이클 시간)는 따로 기록하지 않았다. 문서의 수치는 `--no-ros` 측정값이다.
