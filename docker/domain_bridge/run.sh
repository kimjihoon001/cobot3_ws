#!/usr/bin/env bash
# domain_bridge(Humble, 108↔109 중계) 컨테이너 빌드+실행.
#
# 최초 1회 또는 config.yaml/Dockerfile 수정 후: ./run.sh --build
# 평소 실행: ./run.sh
set -euo pipefail
cd "$(dirname "$0")"

if [[ "${1:-}" == "--build" ]]; then
  docker build -t mm_domain_bridge .
fi

# 인스턴스 하나만 — 두 개가 같이 돌면 /clock·TF를 중복으로 이어줘서
# TF_OLD_DATA / "jump back in time" 이 난다(2026-07-27 확인). --name 고정 +
# 실행 중 검사로 중복 기동을 막는다. 기존 걸 여기서 함부로 안 끈다 —
# 남길 인스턴스는 사람이 고른다.
CONTAINER_NAME=mm_domain_bridge
existing=$(docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format "{{.ID}}\t{{.Status}}")
if [[ -n "$existing" ]]; then
  echo "이미 ${CONTAINER_NAME} 컨테이너가 있습니다(중복 실행 방지로 여기서 멈춤):"
  echo "$existing"
  echo "이걸 정말 새로 띄우려면 먼저 확인 후: docker stop ${CONTAINER_NAME}"
  exit 1
fi

# --network host: 도메인 108(Isaac)·109(이 워크스테이션) 트래픽 모두 실제 LAN
# 인터페이스(10.10.0.x)로 나가야 해서 컨테이너 격리 네트워크를 쓰면 안 된다.
docker run --rm --network host --name "$CONTAINER_NAME" \
  -e ROS_LOCALHOST_ONLY=0 \
  -v /root/.ros/DEFAULT_FASTRTPS_PROFILES.xml:/root/.ros/DEFAULT_FASTRTPS_PROFILES.xml:ro \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/DEFAULT_FASTRTPS_PROFILES.xml \
  mm_domain_bridge
