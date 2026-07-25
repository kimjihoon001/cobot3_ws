#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-/home/rokey/cobot3_ws}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${1:-${WORKSPACE}/debug_bags/forklift_${STAMP}}"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-108}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "$(dirname "${OUTPUT}")"

TOPICS=(
  /clock
  /forklift_0/joint_command
  /forklift_0/joint_states
  /forklift_0/pose
  /forklift/handoff_state
  /forklift/status
  /forklift/clear
  /forklift/task_complete
  /forklift/pallet_on_iw
  /forklift/amr_docked
  /forklift/reset_mission
  /forklift/start_cycle
  /iwhub_0/deck_geometry
  /handoff/tray_ready
  /rosout
)

echo "[forklift rosbag] output: ${OUTPUT}"
echo "[forklift rosbag] 녹화 중입니다. 문제가 보인 직후 Ctrl+C를 누르세요."
echo "[forklift rosbag] 종료 후 Claude Code 전달용 review.md를 자동 생성합니다."

set +e
ros2 bag record \
  --storage sqlite3 \
  --max-cache-size 52428800 \
  --include-unpublished-topics \
  -o "${OUTPUT}" \
  "${TOPICS[@]}"
RECORD_STATUS=$?
set -e

if [[ ! -f "${OUTPUT}/metadata.yaml" ]]; then
  echo "[forklift rosbag] bag이 생성되지 않았습니다: ${OUTPUT}" >&2
  exit "${RECORD_STATUS:-1}"
fi

python3 "${WORKSPACE}/scripts/analyze_forklift_debug_bag.py" \
  "${OUTPUT}" \
  --output "${OUTPUT}/review.md" \
  --json-output "${OUTPUT}/analysis.json"

echo
echo "[forklift rosbag] 분석 완료"
echo "  bag:    ${OUTPUT}"
echo "  review: ${OUTPUT}/review.md"
echo "  json:   ${OUTPUT}/analysis.json"
