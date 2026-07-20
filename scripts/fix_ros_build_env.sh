#!/usr/bin/env bash
# 커스텀 msg(smartfarm_interfaces) colcon build에 필요한 .venv 파이썬 패키지를
# ROS2 배포판(Humble/Jazzy 등)에 맞춰 자동으로 감지·설치한다.
# empy는 배포판별로 필요한 버전이 다르다(Jazzy 이상=4.x, 그 이전=3.3.4) —
# 버전이 맞지 않으면 rosidl_adapter가 다른 에러로 깨진다.
# 참고: https://github.com/ros2/rosidl/issues/779
set -euo pipefail

cd "$(dirname "$0")/.."   # cobot3_ws 루트

if [ -z "${ROS_DISTRO:-}" ]; then
  ROS_DISTRO=$(ls /opt/ros 2>/dev/null | head -n1)
fi
if [ -z "$ROS_DISTRO" ]; then
  echo "[fix_ros_build_env] ROS2 배포판을 못 찾았습니다. /opt/ros/<distro>/setup.bash 를 source 하거나 ROS2를 설치하세요." >&2
  exit 1
fi

if [ ! -x ".venv/bin/pip" ]; then
  echo "[fix_ros_build_env] .venv 가 없습니다. README '사전 준비'대로 python3 -m venv .venv 를 먼저 만드세요." >&2
  exit 1
fi

PIP=".venv/bin/pip"

echo "[fix_ros_build_env] ROS_DISTRO=$ROS_DISTRO"

# Jazzy(및 그 이후 rolling)만 신버전 empy(4.x) 호환. Humble 등 이전 배포판은 3.3.4 고정.
case "$ROS_DISTRO" in
  jazzy|rolling) REQUIRED_EMPY="" ;;
  *) REQUIRED_EMPY="3.3.4" ;;
esac

$PIP show catkin_pkg >/dev/null 2>&1 || { echo "[fix_ros_build_env] catkin_pkg 설치"; $PIP install -q catkin_pkg; }
$PIP show lark        >/dev/null 2>&1 || { echo "[fix_ros_build_env] lark 설치"; $PIP install -q lark; }

CURRENT_EMPY=$($PIP show empy 2>/dev/null | awk '/^Version:/{print $2}')

if [ -n "$REQUIRED_EMPY" ] && [ "$CURRENT_EMPY" != "$REQUIRED_EMPY" ]; then
  echo "[fix_ros_build_env] empy ${CURRENT_EMPY:-없음} -> $REQUIRED_EMPY (배포판 $ROS_DISTRO 호환 버전으로 교체)"
  $PIP install -q "empy==$REQUIRED_EMPY"
elif [ -z "$REQUIRED_EMPY" ] && [ -z "$CURRENT_EMPY" ]; then
  echo "[fix_ros_build_env] empy 설치 (최신)"
  $PIP install -q empy
else
  echo "[fix_ros_build_env] empy $CURRENT_EMPY 유지 (배포판 $ROS_DISTRO 와 호환)"
fi

echo "[fix_ros_build_env] 완료 — colcon build 진행하면 됩니다."
