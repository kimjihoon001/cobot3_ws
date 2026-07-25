#!/usr/bin/env bash
# 통합 시뮬 진단 캡처 (MM 수확·placing / IW 주행·도킹 / Nav2 공통)
#
# 목적: 문제 순간의 전체 ROS 토픽을 한 번에 저장해 오프라인 분석한다.
#   예) IW 베드/도킹 충돌, MM 파지·바스켓 작업영역, 슬롯 선택, 맵/AMCL 등.
#
# 사용법:
#   1) 통합 런치가 이미 떠 있는 상태에서 실행
#   2) 확인하려는 동작 직전에 시작
#   3) 문제 구간을 지난 뒤 몇 초 더 녹화하고 Ctrl-C
#
# 저장 위치: diagnostics/bags/sim_diag_<타임스탬프>/

set -euo pipefail

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-108}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

# ros2 가 없으면 ROS·워크스페이스를 자동 source 한다(터미널에 안 잡혀 빈 bag 나오는 것 방지).
if ! command -v ros2 >/dev/null 2>&1; then
  for s in /opt/ros/*/setup.bash "$HOME/cobot3_ws/install/setup.bash"; do
    [ -f "$s" ] && source "$s"
  done
fi

STAMP=$(date +%Y%m%d_%H%M%S)
OUT="$(dirname "$0")/bags/sim_diag_${STAMP}"
mkdir -p "$(dirname "$OUT")"

echo "[capture] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW=${RMW_IMPLEMENTATION}"

# 사전 점검: 토픽이 실제로 보이는지 확인. 없으면 녹화해봐야 빈 bag만 나온다.
echo "[capture] 토픽 확인 중..."
N=$(timeout 8 ros2 topic list 2>/dev/null | wc -l || true)
echo "[capture] 감지된 전체 토픽: ${N}개"
if [ "${N:-0}" -le 2 ]; then
  echo "[capture] ✗ 토픽이 거의 안 보입니다(=런치 미기동 또는 domain/RMW 불일치)."
  echo "          통합 런치가 떠 있는지, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}/RMW=${RMW_IMPLEMENTATION} 가"
  echo "          런치 터미널과 같은지 확인 후 다시 실행하세요."
  exit 1
fi
echo "[capture] 저장: ${OUT}"
echo "[capture] Ctrl-C 로 종료. 확인하려는 구간을 지난 뒤 몇 초 더 녹화하고 종료하세요."

# 전체 토픽 녹화(-a). 정규식/네임스페이스 매칭 이슈를 피하려고 전부 담는다.
exec ros2 bag record -a -o "${OUT}"
