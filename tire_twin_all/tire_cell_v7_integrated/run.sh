#!/usr/bin/env bash
# 타이어 교체 셀 시연 실행 (너트 풀기 -> 휠 탈거 -> 2초 -> 역순 장착·조임 -> 트랙으로 뒷바퀴 이동).
# 사용법:  ./run.sh [--no-vision] [--vision-noise 1.0] [--car-offset 0,0.03,0] [--speed 1.5] [--mount joint] [--grasp joint] [--skip-nuts] [--pause 2.0] [--no-reassembly] [--no-track-move] [--no-ros] [--ros-res 640x400] [--exit]
# Isaac Sim 설치 위치가 ~/isaacsim 이 아니면:  ISAACSIM_PATH=/경로/isaacsim ./run.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for d in "$ISAACSIM_PATH" "$HOME/isaacsim" "$HOME/isaac-sim" "$HOME/IsaacSim" "$HOME/.local/share/ov/pkg/isaac-sim-5.1.0" "/isaac-sim" "/opt/isaacsim"; do
  if [ -n "$d" ] && [ -x "$d/python.sh" ]; then ISAAC="$d"; break; fi
done
if [ -z "$ISAAC" ]; then
  if command -v isaacsim >/dev/null 2>&1; then          # pip 로 설치한 경우
    exec python "$HERE/run_cell_demo.py" "$@"
  fi
  echo "Isaac Sim 의 python.sh 를 찾지 못했습니다."
  echo "설치 폴더를 지정해서 다시 실행하세요:  ISAACSIM_PATH=/경로/isaacsim ./run.sh"
  exit 1
fi
echo "Isaac Sim: $ISAAC"
# ROS 2 브리지 환경 (~/.bashrc 의 isaac_ros 와 같은 일을 자동으로 한다: 내장 Jazzy 라이브러리 경로 등록)
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-136}            # 이 PC 에 배정된 도메인. 다른 PC 는 ~/.bashrc 의 값을 따른다
ROS_LIB="$ISAAC/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib"
if [ -d "$ROS_LIB" ]; then
  case ":$LD_LIBRARY_PATH:" in
    *":$ROS_LIB:"*) ;;
    *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ROS_LIB" ;;
  esac
fi
echo "ROS 2: ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  ${FASTRTPS_DEFAULT_PROFILES_FILE:+DDS profile=$FASTRTPS_DEFAULT_PROFILES_FILE}"
cd "$ISAAC" && exec ./python.sh "$HERE/run_cell_demo.py" "$@"
