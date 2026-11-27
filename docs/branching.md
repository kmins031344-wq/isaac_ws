# 브랜치 전략 (Branching Strategy)

## 기본 원칙

- `main`: 항상 실행 가능한 상태만 유지한다. 직접 커밋하지 않고 PR 로만 병합한다.
- 작업 브랜치: `{유형}/{짧은-설명}` 형식으로 만든다. 예: `docs/readme-update`

## 브랜치 이름

| 유형 | 용도 | 예 |
|---|---|---|
| `feat` | 기능 추가 (동작, 비전, 씬) | `feat/rear-wheel-removal` |
| `fix` | 버그 수정 | `fix/gripper-slip` |
| `docs` | 문서 | `docs/readme-update` |
| `exp` | 실험·측정 | `exp/vision-noise` |

## 커밋 메시지

`유형: 설명` 형식. 유형은 `feat`, `fix`, `docs`, `test`, `refactor`, `chore`.

예: `docs: 루트 README 에 프로젝트 설명 추가`

## 작업 절차

```bash
git switch main
git pull
git switch -c feat/작업-이름
# ... 작업 ...
git add -A
git commit -m "feat: 설명"
git push            # push.autoSetupRemote 설정 시 첫 push 도 이것으로 충분
```

1. GitHub 에서 `main` 으로 PR 을 만든다. 설명에 바뀐 점과 Isaac Sim 실행 확인 여부를 적는다.
2. 머지 후 로컬을 정리한다.

```bash
git switch main
git pull
git branch -d feat/작업-이름
```

## 올리지 않는 것

`.gitignore` 에 있는 `__pycache__/`, `*.pyc`, `build/`, `install/`, `log/`, `.claude/`, 모델 가중치(`*.onnx`, `*.pt`, `*.pth`), `Collected_*/`.
GitHub 는 100 MB 넘는 파일을 받지 않으므로 큰 에셋은 따로 관리한다.
