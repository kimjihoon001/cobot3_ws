# nav2_bringup 소스/파라미터를 런타임에 문자열 패치할 때 쓰는 버전 호환 헬퍼.
#
# nav2_bringup의 navigation_launch.py는 배포판/버전에 따라 behavior_server Node
# 정의가 두 형식으로 갈린다: 옛 버전은 remappings=remappings)까지만 있어 cmd_vel이
# 그대로 나가고, 일부 최신 버전(Jazzy, 2026-07 확인)은 이미 자체적으로
# remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]로 리맵해서 낸다. 후자를
# "형식이 다름 = 에러"로 처리하면 이미 원하는 상태인데도 launch가 죽는다.

import os
import re
import tempfile

# Humble 표기 → Jazzy 이상 표기. 값에 '/' 가 들어가는 플러그인은 navfn 과 behaviors 뿐이고
# (costmap·controller·smoother 플러그인은 Humble 도 이미 '::'), 키가 정확히 `plugin` 인
# 줄만 건드리므로 map 경로·BT xml 경로 같은 다른 '/' 는 안 다친다.
_PLUGIN_LINE = re.compile(r'(plugin:\s*")(\w+)/(\w+)(")')


def convert_plugin_names_for_distro(params_file: str) -> str:
    """distro 에 맞게 플러그인 표기를 고친 nav2 파라미터 파일 경로를 돌려준다.

    nav2 는 Iron 에서 플러그인 클래스 이름을 `pkg/Class` → `pkg::Class` 로 바꿨다.
    표기가 틀리면 pluginlib 이 못 찾아 해당 서버(planner_server 등)가 아예 안 뜨고,
    그 lifecycle 실패가 costmap 등 나머지 navigation bringup 전체를 중단시킨다.
    파일을 두 벌 두면 한쪽만 고치는 사고가 나므로, 원본 하나를 두고 실행 시점에
    변환한다. Humble이면 원본 그대로 돌려준다(이미 이 표기라 손댈 게 없다).
    """
    if os.environ.get("ROS_DISTRO", "") == "humble":
        return params_file
    with open(params_file, encoding="utf-8") as f:
        text = f.read()
    fixed = _PLUGIN_LINE.sub(r"\1\2::\3\4", text)
    if fixed == text:
        return params_file
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False,
                                      encoding="utf-8")
    tmp.write(fixed)
    tmp.close()
    return tmp.name

_BEHAVIOR_SERVER_ANCHOR = (
    "                executable='behavior_server',\n"
    "                name='behavior_server',\n"
    "                output='screen',\n"
    "                respawn=use_respawn,\n"
    "                respawn_delay=2.0,\n"
    "                parameters=[configured_params],\n"
    "                arguments=['--ros-args', '--log-level', log_level],\n"
    "                remappings=remappings),")

_BEHAVIOR_SERVER_ALREADY_REMAPPED = _BEHAVIOR_SERVER_ANCHOR.replace(
    "                remappings=remappings),",
    "                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],\n            ),")


def remap_behavior_server_cmd_vel(navigation_src: str) -> str:
    """behavior_server의 cmd_vel을 cmd_vel_nav로 리맵한 navigation_launch.py 소스를 돌려준다.

    이미 리맵돼 있으면 그대로 돌려주고, 두 형식 다 아니면 예외를 던진다(nav2_bringup
    버전이 또 바뀌어 패치 지점을 못 찾는 경우 조용히 무시하지 않기 위함).
    """
    if _BEHAVIOR_SERVER_ANCHOR not in navigation_src:
        if _BEHAVIOR_SERVER_ALREADY_REMAPPED not in navigation_src:
            raise RuntimeError(
                "nav2_bringup behavior_server 정의 형식이 예상과 다릅니다")
        return navigation_src
    return navigation_src.replace(
        _BEHAVIOR_SERVER_ANCHOR,
        _BEHAVIOR_SERVER_ANCHOR.replace(
            "                remappings=remappings),",
            "                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),"))


def remove_docking_server(navigation_src: str) -> str:
    """Nav2 충전 docking_server를 실행/관리 목록에서 함께 제거한다."""
    lifecycle_entry = "        'docking_server',\n"
    if lifecycle_entry not in navigation_src:
        return navigation_src
    fixed = navigation_src.replace(lifecycle_entry, "", 1)
    patterns = (
        r"            Node\(\n                package='opennav_docking',\n                executable='opennav_docking',\n                name='docking_server',\n.*?            \),\n",
        r"                    ComposableNode\(\n                        package='opennav_docking',\n                        plugin='opennav_docking::DockingServer',\n                        name='docking_server',\n.*?                    \),\n",
    )
    for pattern in patterns:
        fixed, count = re.subn(pattern, "", fixed, count=1, flags=re.DOTALL)
        if count != 1:
            raise RuntimeError("nav2_bringup docking_server 정의 형식이 예상과 다릅니다")
    return fixed
