#!/usr/bin/env python3
"""현재 MM 수확 파라미터의 TCP 측면 경로를 그린다."""

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle
import numpy as np


FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(FONT_PATH)
plt.rcParams["font.family"] = font_manager.FontProperties(
    fname=FONT_PATH).get_name()
plt.rcParams["axes.unicode_minus"] = False

# 토마토 중심 기준 측면 좌표 [수평, 높이] (m).
fruit = np.array([0.0, 0.0])
fruit_radius = 0.034
safe = np.array([-0.270, -0.075])
circ_start = np.array([-0.170, -0.109])
tcp_goal = fruit.copy()

# P1에서 과실 쪽 수평 접선으로 출발하고 P3를 통과하는 원.
start_z = circ_start[1]
chord_x = -circ_start[0]
circle_center = np.array([
    circ_start[0],
    (start_z * start_z - chord_x * chord_x) / (2.0 * start_z),
])
circle_radius = np.linalg.norm(circ_start - circle_center)
theta_start = np.arctan2(
    circ_start[1] - circle_center[1],
    circ_start[0] - circle_center[0],
)
theta_goal = np.arctan2(
    tcp_goal[1] - circle_center[1],
    tcp_goal[0] - circle_center[0],
)
theta = np.linspace(theta_start, theta_goal, 100)
arc = circle_center + circle_radius * np.column_stack(
    [np.cos(theta), np.sin(theta)])
circ_interim = arc[len(arc) // 2]

fig, axes = plt.subplots(1, 2, figsize=(16, 7))
fig.suptitle(
    "수정 후 수확 TCP 경로 — 최종 HarvestTCP = 토마토 중심",
    fontsize=21,
    fontweight="bold",
)


def setup_axis(ax, xlim, ylim, title):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.28)
    ax.axhline(0.0, color="#718096", lw=1.1, ls="--")
    ax.axvline(0.0, color="#718096", lw=1.1, ls="--")
    ax.set_xlabel("토마토 중심 기준 수평 거리 (m)   로봇 ← 0 → 베드")
    ax.set_ylabel("토마토 중심 기준 높이 Z (m)")
    ax.set_title(title, fontsize=15, fontweight="bold")
    ax.add_patch(Circle(fruit, fruit_radius, color="#ef4444", ec="#991b1b", lw=2.5))
    ax.text(0.0, 0.0, "토마토\n중심", ha="center", va="center",
            color="white", fontweight="bold", fontsize=11)


def annotate(ax, point, text, color, offset):
    ax.scatter(*point, s=95, color=color, edgecolor="white", linewidth=1.5, zorder=8)
    ax.annotate(
        text,
        point,
        xytext=offset,
        textcoords="offset points",
        fontsize=10,
        bbox=dict(boxstyle="round,pad=0.35", fc="white", ec=color, lw=1.5),
        arrowprops=dict(arrowstyle="->", color=color, lw=1.5),
    )


ax = axes[0]
setup_axis(ax, (-0.31, 0.07), (-0.15, 0.08), "전체 접근 경로")
ax.plot(
    [safe[0], circ_start[0]],
    [safe[1], circ_start[1]],
    color="#2563eb",
    lw=5,
    label="LIN: 안전점 → CIRC 시작점",
)
ax.plot(
    arc[:, 0],
    arc[:, 1],
    color="#f59e0b",
    lw=5,
    label="CIRC: P1 → P2 → P3 전체가 단일 원호",
)
annotate(ax, safe, "P0 안전 TCP\n(-27.0, -7.5) cm", "#2563eb", (-110, 28))
annotate(ax, circ_start, "P1 CIRC 시작 TCP\n(-17.0, -10.9) cm", "#0f766e", (-25, -62))
annotate(
    ax,
    circ_interim,
    f"P2 원호 보조 TCP\n({circ_interim[0]*100:.1f}, "
    f"{circ_interim[1]*100:.1f}) cm",
    "#d97706",
    (-95, 30),
)
annotate(ax, tcp_goal, "P3 최종 TCP\n(0, 0) = 토마토 중심", "#7c3aed", (18, 48))
ax.legend(loc="upper left", fontsize=10)

ax = axes[1]
setup_axis(ax, (-0.20, 0.07), (-0.135, 0.075), "토마토 주변 확대")
ax.plot(
    arc[:, 0],
    arc[:, 1],
    color="#f59e0b",
    lw=5,
)
annotate(ax, circ_start, "P1 시작", "#0f766e", (-45, -45))
annotate(ax, circ_interim, "P2 = 원호 위 보조점", "#d97706", (-95, 30))
annotate(ax, tcp_goal, "HarvestTCP\n토마토 중심과 일치", "#7c3aed", (15, 42))
ax.text(
    -0.185,
    0.060,
    "P1에서 수평 접선으로 진입\nP1 → P2 → P3 전체가 하나의 CIRC 원호",
    fontsize=11,
    color="#166534",
    va="top",
    bbox=dict(boxstyle="round,pad=0.5", fc="#f0fdf4", ec="#16a34a", lw=1.6),
)
ax.annotate(
    "",
    xy=(0.0, -fruit_radius),
    xytext=(0.0, fruit_radius),
    arrowprops=dict(arrowstyle="<->", color="#991b1b", lw=1.6),
)
ax.text(0.008, 0.0, "지름 6.8 cm", color="#991b1b", va="center", fontsize=10)

fig.text(
    0.5,
    0.02,
    "파란색: LIN  |  주황색: CIRC 요청 경로  |  최종점에서는 HarvestTCP의 XYZ가 토마토 중심 XYZ와 동일",
    ha="center",
    fontsize=11,
)
fig.tight_layout(rect=(0, 0.055, 1, 0.93))
fig.savefig("harvest_tcp_path_center_aligned.png", dpi=180, bbox_inches="tight")
