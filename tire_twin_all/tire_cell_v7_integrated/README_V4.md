# (이력) 타이어 교체 셀 v4

> 참고용 이력 문서입니다. v4 기능(너트 트레이 보관)은 `tire_cell_v7_integrated/` 에 합쳐졌고, `tire_cell_v4_nut_drop/` 폴더는 이 저장소에 없습니다. 실행은 이 폴더의 `README.md` 를 따르세요.

원본 `tire_cell_v3/`를 그대로 두고, 풀린 너트를 트레이에 보관하는 새 버전입니다.

## 동작

로봇 B의 오른쪽이면서 차체 바깥 바닥에 놓인 트레이로 이동합니다. 별 모양 순서로 너트를 풀고, 풀린 너트는 소켓에 붙어 로봇을 따라가다가 트레이 슬롯에 차례로 놓입니다.
원본 체결 시각은 유지되며, 모든 너트를 풀면 기존 고정 관절을 해제해 휠 탈거로 이어집니다.

```bash
cd tire_twin_all/tire_cell_v7_integrated
bash run.sh
```

씬을 다시 만들려면:

```bash
python3 build_scene_v3.py .
```

## 주의

- 풀린 너트는 시각화용 형상으로 소켓을 따라 이동하고 슬롯에 놓입니다. 나사산과 실제 강체 파지/접촉 물리는 구현되어 있지 않습니다.
- 트레이 위치와 슬롯 좌표는 `build_scene_v3.py`의 `cell:nut_tray_center`, `cell:nut_tray_slots`에서 바꿀 수 있습니다.
