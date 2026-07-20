#!/usr/bin/env bash
# 개인 작업 트리 → 진짜 프로젝트 트리(~/cobot3_ws/isaacpjt) 동기화.
#
# 흐름: 여기(basic/jihoonkim/isaacpjt)서 작업 → 검증 → ./sync_to_project.sh → 커밋/push.
# 코어만 보낸다 (아카이브성 제외: temp/, tests/, spikes 01~05, 구식 ros 액션 스택,
# RESULTS.md·SATURDAY.txt 문서). --delete 는 절대 안 쓴다(CLAUDE.md — USD 날림 방지).
# 대상에서 파일을 "빼야" 할 땐 수동으로 지울 것.
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)"
DST="$HOME/cobot3_ws/isaacpjt"

# main.py = 환경/골격, mm·iw·fork = 로봇 드라이버(플래그로 선택), robot_base = 공통 베이스
rsync -av \
    "$SRC/main.py" "$SRC/mm.py" "$SRC/iw.py" "$SRC/fork.py" \
    "$SRC/robot_base.py" "$SRC/README.md" "$DST/"
rsync -av --exclude __pycache__ \
    "$SRC/pjt_config" "$SRC/pjt_utils" "$SRC/scene" "$SRC/tools" "$DST/"
rsync -av --exclude __pycache__ --exclude stub_harvester.py \
    "$SRC/robots" "$DST/"
mkdir -p "$DST/ros"
rsync -av "$SRC/ros/__init__.py" "$SRC/ros/robot_bridge.py" "$DST/ros/"
rsync -av --exclude __pycache__ "$SRC/assets" "$DST/"   # assets/tomato(과실 USD) 포함

echo
echo "== 동기화 완료: $DST  (검증: cd $DST && isaac_python main.py --quiet) =="
