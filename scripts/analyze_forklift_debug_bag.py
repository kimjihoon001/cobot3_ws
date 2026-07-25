#!/usr/bin/env python3
"""Forklift/IW rosbag을 읽어 Claude Code 전달용 검토 패킷을 생성한다."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


HANDOFF_TOPIC = "/forklift/handoff_state"
COMMAND_TOPIC = "/forklift_0/joint_command"
STATE_TOPIC = "/forklift_0/joint_states"
POSE_TOPIC = "/forklift_0/pose"
STATUS_TOPIC = "/forklift/status"

# Isaac가 게시하는 피드백 스트림. ROS 측 명령/상태 토픽이 bag 종료까지 계속
# 게시되는데 이 스트림들만 먼저 끊기면 Isaac 단절/종료로 판정한다.
ISAAC_FEEDBACK_TOPICS = (
    STATE_TOPIC,
    POSE_TOPIC,
    HANDOFF_TOPIC,
    "/iwhub_0/deck_geometry",
    "/clock",
)
# 상태 문자열이 아래 키워드를 포함하면 일반 오류가 아니라 상태 단절로 승격한다.
DISCONNECT_KEYWORDS = (
    "연결이 끊겼",
    "연결 끊",
    "disconnect",
    "timeout",
    "타임아웃",
)
# runtime_observation.txt에서 Isaac 종료/무효화를 나타내는 마커.
ISAAC_SHUTDOWN_MARKERS = (
    "Simulation App Shutting Down",
    "Physics Simulation View is not created",
)


def yaw_from_pose(message: Any) -> float:
    q = message.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def finite_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def joint_values(message: Any) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    names = list(getattr(message, "name", []))
    for index, name in enumerate(names):
        values: dict[str, float] = {}
        for field in ("position", "velocity", "effort"):
            sequence = getattr(message, field, [])
            if index < len(sequence):
                value = finite_float(sequence[index])
                if value is not None:
                    values[field] = value
        result[str(name)] = values
    return result


def value_for(
    joints: dict[str, dict[str, float]],
    name: str,
    field: str,
) -> float | None:
    return joints.get(name, {}).get(field)


def git_text(workspace: Path, *args: str) -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=workspace,
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        return result.stdout.strip()
    except Exception as exc:
        return f"<git command failed: {exc}>"


def event_line(seconds: float, kind: str, detail: str) -> str:
    return f"- `{seconds:9.3f}s` **{kind}** — {detail}"


def analyze(bag_path: Path, workspace: Path) -> dict[str, Any]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }
    message_types: dict[str, Any] = {}
    for topic, type_name in topic_types.items():
        try:
            message_types[topic] = get_message(type_name)
        except Exception:
            pass

    counts: Counter[str] = Counter()
    decode_errors: Counter[str] = Counter()
    timeline: list[str] = []
    first_ns: int | None = None
    last_ns: int | None = None
    first_topic_ns: dict[str, int] = {}
    last_topic_ns: dict[str, int] = {}

    last_command_signature = None
    last_owner_signature = None
    last_alignment_signature = None
    last_status = None
    status_samples: list[tuple[float, str]] = []
    attach_request_times: list[float] = []
    fork_owner_times: list[float] = []
    alignment_samples: list[dict[str, Any]] = []
    handoff_samples: list[dict[str, Any]] = []
    lift_commands: list[tuple[float, float]] = []
    lift_states: list[tuple[float, float]] = []
    poses: list[tuple[float, float, float, float]] = []

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        counts[topic] += 1
        first_ns = timestamp_ns if first_ns is None else min(first_ns, timestamp_ns)
        last_ns = timestamp_ns if last_ns is None else max(last_ns, timestamp_ns)
        first_topic_ns.setdefault(topic, timestamp_ns)
        last_topic_ns[topic] = max(last_topic_ns.get(topic, timestamp_ns), timestamp_ns)
        if topic not in message_types:
            continue
        try:
            message = deserialize_message(raw, message_types[topic])
        except Exception:
            decode_errors[topic] += 1
            continue
        seconds = (timestamp_ns - first_ns) / 1e9

        if topic == COMMAND_TOPIC:
            joints = joint_values(message)
            lift = value_for(joints, "lift_joint", "position")
            steer = value_for(joints, "back_wheel_swivel", "position")
            drive = value_for(joints, "back_wheel_drive", "velocity")
            attach = value_for(joints, "pallet_attach", "position")
            deck = value_for(joints, "pallet_deck_attach", "position")
            lock = value_for(joints, "iw_dock_lock", "position")
            pallet = value_for(joints, "pallet_id", "position")
            if lift is not None:
                lift_commands.append((seconds, lift))
            if attach is not None and attach >= 0.5:
                attach_request_times.append(seconds)
            signature = tuple(
                None if value is None else round(value, 4)
                for value in (lift, steer, drive, attach, deck, lock, pallet)
            )
            if signature != last_command_signature:
                timeline.append(event_line(
                    seconds,
                    "COMMAND",
                    "lift={}, steer={}, drive={}, fork_attach={}, "
                    "deck_attach={}, dock_lock={}, pallet={}".format(
                        *[
                            "None" if value is None else f"{value:.4f}"
                            for value in (lift, steer, drive, attach, deck, lock, pallet)
                        ]
                    ),
                ))
                last_command_signature = signature

        elif topic == STATE_TOPIC:
            joints = joint_values(message)
            lift = value_for(joints, "lift_joint", "position")
            if lift is not None:
                lift_states.append((seconds, lift))

        elif topic == POSE_TOPIC:
            position = message.pose.position
            yaw = yaw_from_pose(message)
            poses.append((seconds, float(position.x), float(position.y), yaw))

        elif topic == HANDOFF_TOPIC:
            try:
                payload = json.loads(message.data)
            except (TypeError, json.JSONDecodeError):
                decode_errors[topic] += 1
                continue
            payload["_time"] = seconds
            handoff_samples.append(payload)
            owner_signature = (
                payload.get("owner"),
                payload.get("pallet_id"),
                payload.get("dock_locked"),
                payload.get("deck_collision_filtered"),
                payload.get("fork_collision_filtered"),
            )
            if owner_signature != last_owner_signature:
                timeline.append(event_line(
                    seconds,
                    "HANDOFF",
                    "owner={}, pallet={}, dock_locked={}, deck_filter={}, "
                    "fork_filter={}".format(*owner_signature),
                ))
                last_owner_signature = owner_signature
            if payload.get("owner") == "fork":
                fork_owner_times.append(seconds)

            alignment = payload.get("pickup_alignment")
            if isinstance(alignment, dict):
                sample = dict(alignment)
                sample["_time"] = seconds
                alignment_samples.append(sample)
                alignment_signature = (
                    sample.get("valid"),
                    sample.get("reason"),
                    round(finite_float(sample.get("z_correction")) or 0.0, 3),
                )
                if alignment_signature != last_alignment_signature:
                    timeline.append(event_line(
                        seconds,
                        "PICKUP GATE",
                        "valid={}, reason={}, lateral={}m, yaw={}deg, "
                        "overlap={}m, z_correction={}m, lift={}m".format(
                            sample.get("valid"),
                            sample.get("reason"),
                            sample.get("lateral_error"),
                            sample.get("yaw_error_deg"),
                            sample.get("insertion_overlap"),
                            sample.get("z_correction"),
                            sample.get("lift_actual"),
                        ),
                    ))
                    last_alignment_signature = alignment_signature

        elif topic == STATUS_TOPIC:
            status = str(message.data)
            status_samples.append((seconds, status))
            if status != last_status:
                timeline.append(event_line(seconds, "STATUS", status))
                last_status = status

    duration = (
        0.0 if first_ns is None or last_ns is None
        else (last_ns - first_ns) / 1e9
    )

    runtime_observation_path = bag_path / "runtime_observation.txt"
    runtime_observation = (
        runtime_observation_path.read_text(encoding="utf-8").strip()
        if runtime_observation_path.exists()
        else ""
    )

    findings: list[dict[str, str]] = []
    if counts[COMMAND_TOPIC] == 0:
        findings.append({
            "severity": "BLOCKER",
            "finding": "지게차 명령 토픽이 녹화되지 않았습니다.",
        })
    if counts[STATE_TOPIC] == 0:
        findings.append({
            "severity": "BLOCKER",
            "finding": "실제 lift_joint 피드백이 녹화되지 않았습니다.",
        })
    if counts[HANDOFF_TOPIC] == 0:
        findings.append({
            "severity": "BLOCKER",
            "finding": "Isaac handoff 상태가 녹화되지 않았습니다.",
        })
    def is_disconnect(text: str) -> bool:
        return any(keyword in text for keyword in DISCONNECT_KEYWORDS)

    error_statuses = [
        (seconds, status)
        for seconds, status in status_samples
        if "ERROR" in status.upper()
        or "실패" in status
        or is_disconnect(status)
    ]

    # 명시적 치명 오류 1: 상태 문자열로 보고된 ROS 상태 스트림 단절.
    disconnect_statuses = [
        (seconds, status)
        for seconds, status in error_statuses
        if is_disconnect(status)
    ]
    if disconnect_statuses:
        seconds, status = disconnect_statuses[0]
        findings.append({
            "severity": "BLOCKER",
            "finding": (
                f"{seconds:.3f}s에 ROS 상태 스트림이 단절됐습니다: {status}"
            ),
        })

    # 명시적 치명 오류 2: 데이터 기반 Isaac 피드백 스트림 단절. 명령/상태 토픽은
    # bag 종료까지 게시되는데 Isaac 피드백 스트림만 유의미하게(스트림 주기의 5배
    # 또는 0.5초 이상) 먼저 멈추면 상태 단절로 승격한다. 상태 문자열이 없어도
    # 데이터만으로 잡히므로 "OK"로 떨어지지 않는다.
    feedback_gaps: list[tuple[str, float]] = []
    if last_ns is not None:
        for topic in ISAAC_FEEDBACK_TOPICS:
            count = counts.get(topic, 0)
            if count < 2 or topic not in last_topic_ns:
                continue
            span = (last_topic_ns[topic] - first_topic_ns[topic]) / 1e9
            period = span / (count - 1)
            gap = (last_ns - last_topic_ns[topic]) / 1e9
            if gap > max(0.5, 5.0 * period):
                feedback_gaps.append((topic, gap))
    if feedback_gaps:
        feedback_gaps.sort(key=lambda item: item[1], reverse=True)
        detail = ", ".join(f"{topic} {gap:.2f}s" for topic, gap in feedback_gaps)
        findings.append({
            "severity": "BLOCKER",
            "finding": (
                "ROS 상태 단절(데이터 기반): 명령/상태 토픽은 bag 종료까지 "
                f"게시됐지만 Isaac 피드백 스트림이 먼저 멈췄습니다 — {detail}."
            ),
        })

    # 명시적 치명 오류 3: runtime_observation에 기록된 Isaac 종료/무효화.
    shutdown_hits = [
        marker for marker in ISAAC_SHUTDOWN_MARKERS if marker in runtime_observation
    ]
    if shutdown_hits:
        findings.append({
            "severity": "BLOCKER",
            "finding": (
                "Isaac 시뮬레이션 종료/무효화가 런타임 관찰에 기록됐습니다: "
                + ", ".join(shutdown_hits)
            ),
        })

    # 단절이 아닌 일반 상태 오류(있다면)도 치명으로 보고한다.
    other_error_statuses = [
        (seconds, status)
        for seconds, status in error_statuses
        if not is_disconnect(status)
    ]
    if other_error_statuses:
        seconds, status = other_error_statuses[0]
        findings.append({
            "severity": "BLOCKER",
            "finding": (
                f"{seconds:.3f}s에 작업 상태 오류가 발생했습니다: {status}"
            ),
        })
    if attach_request_times and not fork_owner_times:
        findings.append({
            "severity": "ERROR",
            "finding": "fork_attach 요청은 있었지만 owner=fork 전환이 한 번도 없었습니다.",
        })
    if alignment_samples:
        invalid = [sample for sample in alignment_samples if not sample.get("valid")]
        reasons = Counter(str(sample.get("reason")) for sample in invalid)
        if invalid:
            findings.append({
                "severity": "ERROR",
                "finding": "Pickup Gate 불합격: " + ", ".join(
                    f"{reason} {count}회" for reason, count in reasons.items()
                ),
            })
        owner_before_valid = False
        valid_times = [
            float(sample["_time"])
            for sample in alignment_samples
            if sample.get("valid")
        ]
        if fork_owner_times and (
            not valid_times or min(fork_owner_times) < min(valid_times)
        ):
            owner_before_valid = True
        if owner_before_valid:
            findings.append({
                "severity": "BLOCKER",
                "finding": "실측 정렬 통과 전에 팔레트 소유권이 fork로 전환됐습니다.",
            })
    elif attach_request_times:
        findings.append({
            "severity": "ERROR",
            "finding": "결합 요청은 있었지만 pickup_alignment 실측값이 없습니다.",
        })

    tilts = [
        finite_float(sample.get("iw_tilt_deg"))
        for sample in handoff_samples
    ]
    tilts = [value for value in tilts if value is not None]
    if tilts and max(tilts) > 5.0:
        findings.append({
            "severity": "ERROR",
            "finding": f"IW 최대 기울기 {max(tilts):.2f}°로 5° 한계를 넘었습니다.",
        })

    rise_errors = []
    carry_errors = []
    for sample in handoff_samples:
        actual = finite_float(sample.get("pallet_rise"))
        expected = finite_float(sample.get("expected_rise"))
        carry = finite_float(sample.get("carry_pose_error"))
        if actual is not None and expected is not None:
            rise_errors.append(abs(actual - expected))
        if carry is not None:
            carry_errors.append(carry)
    if rise_errors and max(rise_errors) > 0.015:
        findings.append({
            "severity": "ERROR",
            "finding": f"팔레트 상승 오차 최대 {max(rise_errors):.4f}m입니다.",
        })
    if carry_errors and max(carry_errors) > 0.015:
        findings.append({
            "severity": "ERROR",
            "finding": f"팔레트 추종 오차 최대 {max(carry_errors):.4f}m입니다.",
        })
    if (
        (error_statuses or feedback_gaps)
        and fork_owner_times
        and any(sample.get("valid") for sample in alignment_samples)
        and (not rise_errors or max(rise_errors) <= 0.015)
        and (not carry_errors or max(carry_errors) <= 0.015)
    ):
        findings.append({
            "severity": "EVIDENCE",
            "finding": (
                "팔레트 결합·상승·포크 추종은 정상 범위였습니다. "
                "최초 확인된 실패는 후진 중 Isaac 피드백 스트림 단절이므로 "
                "Z/삽입 상수를 다시 조절할 근거가 없습니다."
            ),
        })
    if not findings:
        findings.append({
            "severity": "OK",
            "finding": "기록된 상태에서 자동 판정된 오류가 없습니다.",
        })

    return {
        "bag": str(bag_path.resolve()),
        "duration_sec": duration,
        "topic_types": topic_types,
        "topic_counts": dict(sorted(counts.items())),
        "decode_errors": dict(sorted(decode_errors.items())),
        "findings": findings,
        "runtime_observation": runtime_observation,
        "timeline": timeline,
        "metrics": {
            "attach_request_count": len(attach_request_times),
            "fork_owner_sample_count": len(fork_owner_times),
            "alignment_sample_count": len(alignment_samples),
            "max_iw_tilt_deg": max(tilts) if tilts else None,
            "max_rise_error_m": max(rise_errors) if rise_errors else None,
            "max_carry_error_m": max(carry_errors) if carry_errors else None,
            "lift_command_min_m": min((value for _, value in lift_commands), default=None),
            "lift_command_max_m": max((value for _, value in lift_commands), default=None),
            "lift_state_min_m": min((value for _, value in lift_states), default=None),
            "lift_state_max_m": max((value for _, value in lift_states), default=None),
            "pose_start": list(poses[0][1:]) if poses else None,
            "pose_end": list(poses[-1][1:]) if poses else None,
            "terminal_status": status_samples[-1][1] if status_samples else None,
            "isaac_feedback_gap_sec": (
                max((gap for _, gap in feedback_gaps), default=None)
            ),
            "isaac_shutdown_detected": bool(shutdown_hits),
        },
        "git": {
            "head": git_text(workspace, "rev-parse", "HEAD"),
            "branch": git_text(workspace, "branch", "--show-current"),
            "status": git_text(workspace, "status", "--short"),
            "diff_stat": git_text(workspace, "diff", "--stat"),
        },
    }


def markdown(report: dict[str, Any]) -> str:
    metrics = report["metrics"]
    lines = [
        "# Forklift/IW ROS bag 검토 패킷",
        "",
        "이 문서는 실제 ROS bag을 메시지 타임스탬프 순서로 읽어 자동 생성했습니다.",
        "Claude Code는 먼저 이 문서와 bag을 읽고 원인을 판정한 뒤 코드를 수정하세요.",
        "",
        "## 재현 데이터",
        "",
        f"- Bag: `{report['bag']}`",
        f"- Duration: `{report['duration_sec']:.3f}s`",
        f"- Git branch: `{report['git']['branch']}`",
        f"- Git HEAD: `{report['git']['head']}`",
        "",
        "## 자동 판정",
        "",
    ]
    lines.extend(
        f"- **{item['severity']}** — {item['finding']}"
        for item in report["findings"]
    )
    lines.extend([
        "",
        "## 핵심 수치",
        "",
        f"- attach 요청 수: `{metrics['attach_request_count']}`",
        f"- owner=fork 표본 수: `{metrics['fork_owner_sample_count']}`",
        f"- pickup alignment 표본 수: `{metrics['alignment_sample_count']}`",
        f"- IW 최대 tilt: `{metrics['max_iw_tilt_deg']}` deg",
        f"- pallet rise 최대 오차: `{metrics['max_rise_error_m']}` m",
        f"- carry pose 최대 오차: `{metrics['max_carry_error_m']}` m",
        f"- lift 명령 범위: `{metrics['lift_command_min_m']}` ~ "
        f"`{metrics['lift_command_max_m']}` m",
        f"- lift 실제 범위: `{metrics['lift_state_min_m']}` ~ "
        f"`{metrics['lift_state_max_m']}` m",
        f"- forklift pose 시작: `{metrics['pose_start']}`",
        f"- forklift pose 종료: `{metrics['pose_end']}`",
        f"- 마지막 작업 상태: `{metrics['terminal_status']}`",
        f"- Isaac 피드백 스트림 단절 gap: `{metrics['isaac_feedback_gap_sec']}` s",
        f"- Isaac 종료 마커 감지: `{metrics['isaac_shutdown_detected']}`",
        "",
        "## Isaac 런타임 관찰",
        "",
        "```text",
        report["runtime_observation"] or "<별도 런타임 관찰 기록 없음>",
        "```",
        "",
        "## 이벤트 타임라인",
        "",
    ])
    lines.extend(report["timeline"] or ["- 유효한 이벤트가 없습니다."])
    lines.extend([
        "",
        "## 토픽/메시지 수",
        "",
        "| Topic | Type | Count |",
        "|---|---|---:|",
    ])
    for topic, type_name in sorted(report["topic_types"].items()):
        lines.append(
            f"| `{topic}` | `{type_name}` | "
            f"{report['topic_counts'].get(topic, 0)} |"
        )
    lines.extend([
        "",
        "## Claude Code 검토 요청",
        "",
        "1. `pickup_alignment`의 lateral/yaw/insertion/height 중 최초 실패 축을 "
        "타임라인에서 확정하세요.",
        "2. `/forklift_0/joint_command`와 실제 `/forklift_0/joint_states`의 "
        "`lift_joint` 차이를 같은 시각으로 비교하세요.",
        "3. 실측 정렬이 통과하기 전에 owner가 `fork`로 바뀌는 경로가 있는지 "
        "확인하세요.",
        "4. 팔레트가 들리지 않았다면 `pallet_rise`와 `expected_rise`, "
        "`carry_pose_error`를 근거로 좌표 문제와 소유권 문제를 구분하세요.",
        "5. 이번 캡처에서 결합·상승·추종이 정상이라면 Z나 삽입 상수를 다시 "
        "바꾸지 말고, 후진 시점의 Isaac 종료/physics view 무효화 경로를 먼저 "
        "추적하세요.",
        "6. 추측으로 상수를 다시 조절하지 말고, bag에서 확인된 최초 불일치만 "
        "최소 수정하세요.",
        "7. 기존 사용자 변경을 되돌리거나 전체 파일을 교체하지 마세요.",
        "",
        "## 현재 워킹트리",
        "",
        "```text",
        report["git"]["status"] or "<clean>",
        "```",
        "",
        "## 현재 diff 통계",
        "",
        "```text",
        report["git"]["diff_stat"] or "<no diff>",
        "```",
        "",
    ])
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path("/home/rokey/cobot3_ws"),
    )
    args = parser.parse_args()

    result = analyze(args.bag, args.workspace)
    output = args.output or args.bag / "review.md"
    json_output = args.json_output or args.bag / "analysis.json"
    output.write_text(markdown(result), encoding="utf-8")
    json_output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(output)


if __name__ == "__main__":
    main()
