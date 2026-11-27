# 2026-11-27 저장소 생성 및 문서 정리

## 한 일

- `tire_twin_all/` (v2, v3, v7 통합본) 을 저장소 `main` 에 업로드
- `docs/readme-update` 브랜치에서 문서 정리
  - 루트 README 작성 (프로젝트 소개, 실행 환경, 구조, 현재 상태)
  - `tire_twin_all/docs/` → 최상위 `docs/` 로 이동
  - `docs/` 에 시작 가이드, 아키텍처, 브랜치 규칙, 작업 기록 추가
  - 하위 README 의 개인 PC 절대 경로(`~/Desktop/...`, `/home/rokey/Downloads/...`)와 zip 안내를 저장소 상대 경로로 변경
  - `tire_twin_all/README.md` 의 최신본 안내를 v3 → v7 로 수정

## 결정 사항

- 최신 작업은 `tire_cell_v7_integrated` 에서 이어간다. v4·v5·v6 폴더는 v7 에 합쳐져 저장소에 없다 (v4 설명은 `README_V4.md` 에 이력으로 남김).
- 셀 씬 파일 이름 `tire_cell_v3*.usda` 는 바꾸지 않는다 (코드가 이 이름을 참조).
- `ROS_DOMAIN_ID` 는 PC 별 `~/.bashrc` 값을 따른다 (`run.sh` 기본값 136).

## 참고

- v7 상세 설명과 측정 결과: [`tire_cell_v7_integrated/README.md`](../../tire_twin_all/tire_cell_v7_integrated/README.md)
