#!/usr/bin/env bash
# 타이어 탈거 시연 실행.  사용법:  ./run.sh [--speed 1.5] [--grasp joint] [--exit]
# Isaac Sim 설치 위치가 ~/isaacsim 이 아니면:  ISAACSIM_PATH=/경로/isaacsim ./run.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for d in "$ISAACSIM_PATH" "$HOME/isaacsim" "$HOME/isaac-sim" "$HOME/IsaacSim" "$HOME/.local/share/ov/pkg/isaac-sim-5.1.0" "/isaac-sim" "/opt/isaacsim"; do
  if [ -n "$d" ] && [ -x "$d/python.sh" ]; then ISAAC="$d"; break; fi
done
if [ -z "$ISAAC" ]; then
  if command -v isaacsim >/dev/null 2>&1; then          # pip 로 설치한 경우
    exec python "$HERE/run_tire_demo_v2.py" "$@"
  fi
  echo "Isaac Sim 의 python.sh 를 찾지 못했습니다."
  echo "설치 폴더를 지정해서 다시 실행하세요:  ISAACSIM_PATH=/경로/isaacsim ./run.sh"
  exit 1
fi
echo "Isaac Sim: $ISAAC"
cd "$ISAAC" && exec ./python.sh "$HERE/run_tire_demo_v2.py" "$@"
