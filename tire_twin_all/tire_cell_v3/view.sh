#!/usr/bin/env bash
# 씬 보기 (스크린샷용).  사용법:  bash view.sh cars   |  이름 목록은  bash view.sh
# Isaac Sim 설치 위치가 ~/isaacsim 이 아니면:  ISAACSIM_PATH=/경로/isaacsim bash view.sh cars
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for d in "$ISAACSIM_PATH" "$HOME/isaacsim" "$HOME/isaac-sim" "$HOME/IsaacSim" "$HOME/.local/share/ov/pkg/isaac-sim-5.1.0" "/isaac-sim" "/opt/isaacsim"; do
  if [ -n "$d" ] && [ -x "$d/python.sh" ]; then ISAAC="$d"; break; fi
done
if [ -z "$ISAAC" ]; then
  if command -v isaacsim >/dev/null 2>&1; then exec python "$HERE/view_scene.py" "$@"; fi
  echo "Isaac Sim 의 python.sh 를 찾지 못했습니다.  ISAACSIM_PATH=/경로/isaacsim bash view.sh <이름>"
  exit 1
fi
cd "$ISAAC" && exec ./python.sh "$HERE/view_scene.py" "$@"
