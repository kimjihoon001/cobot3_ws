# -*- coding: utf-8 -*-
"""모니터링 UI용 고정 감시 카메라 4대 (--cctv).

장소별 CCTV가 아니라 공정 순서대로 배치한다. MM 전방 D455가 1번 화면이고
나머지가 수확 → 인계 → 적재로 이어진다.

  /cctv/greenhouse  MM이 수확해 IW 팔레트에 넣는 구간
  /cctv/unloading   IW ↔ 지게차 팔레트 인계
  /cctv/storage     지게차가 랙에 적재
  /cctv/overview    전체 조감도 (orthographic) — 격자에는 안 넣고 필요할 때만 확대

넷 다 RGB만 뽑는다. 사람이 보기만 하는 화면이라 depth 렌더는 GPU만 먹는다.
좌표는 온실/창고 치수에서 유도하므로 씬 크기를 바꿔도 따라온다.
"""
from pxr import Gf, Usd, UsdGeom

from scene.warehouse import RACK_DEPTH

ROOT = "/World/MonitorCams"

# iwhub_control/lanes.py의 DOCK y와 같아야 한다. IW는 온실 쪽(Y<13)에 서고
# 지게차는 창고 안(Y>13)에서 나오므로, 인계 장면은 두 방을 가르는 벽의
# 입구를 통해서만 한 화면에 담긴다.
IW_DOCK_Y = 10.84885

# mm_moveit/launch/nav2_harvest_bringup.launch.py의 fixed_goal_x/y와 같아야 한다.
# MM이 여기 정차해서 딴다. 이 지점을 안 보면 통로를 지나가는 장면만 남는다.
HARVEST_XY = (-0.54, -8.19)

# 화면 칸이 16:9라 4:3으로 뽑으면 좌우에 검은 여백이 크게 남는다. 16:9로
# 맞추면 칸을 꽉 채우고 렌더 픽셀도 25% 줄어든다.
RES = (640, 360)

# 광각 CCTV 느낌. USD 기본값(focalLength 50 + aperture 20.955)은 화각이
# 23°라 감시 화면에는 지나치게 좁다. 12mm면 약 82°.
CCTV_FOCAL_LENGTH = 12.0
CCTV_APERTURE = 20.955


def _look_at(eye, target, up_hint) -> Gf.Matrix4d:
    """USD 카메라(로컬 -Z를 본다)를 eye에서 target으로 향하게 하는 변환.

    up_hint가 시선과 평행하면 축이 무너진다. 바로 내려다보는 조감도는
    월드 +Z가 시선과 겹치므로 화면 세로축이 될 방향을 직접 넘긴다.
    """
    z = (Gf.Vec3d(*eye) - Gf.Vec3d(*target)).GetNormalized()
    x = Gf.Cross(Gf.Vec3d(*up_hint), z).GetNormalized()
    y = Gf.Cross(z, x)
    m = Gf.Matrix4d(1.0)
    m.SetRotateOnly(Gf.Matrix3d(x[0], x[1], x[2],
                                y[0], y[1], y[2],
                                z[0], z[1], z[2]))
    m.SetTranslateOnly(Gf.Vec3d(*eye))
    return m


def _add_camera(stage: Usd.Stage, path: str, eye, target,
                up_hint=(0.0, 0.0, 1.0), ortho_width: float | None = None,
                res=RES) -> str:
    cam = UsdGeom.Camera.Define(stage, path)
    UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(
        _look_at(eye, target, up_hint))
    if ortho_width is None:
        cam.CreateProjectionAttr("perspective")
        cam.CreateFocalLengthAttr(CCTV_FOCAL_LENGTH)
        aperture = CCTV_APERTURE
    else:
        # orthographic aperture는 월드 단위의 10배로 적는다(USD 규약).
        cam.CreateProjectionAttr("orthographic")
        aperture = ortho_width * 10.0
    # 세로 aperture를 렌더 해상도 비율에 맞춰야 화면이 늘어나지 않는다.
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateVerticalApertureAttr(aperture * res[1] / res[0])
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200.0))
    return path


def spawn(stage: Usd.Stage, cfg, publish: bool = True, log=print) -> list:
    """고정 카메라 4대를 씬에 박고 ROS2 RGB 발행까지 건다."""
    g, wh = cfg.greenhouse, cfg.warehouse
    UsdGeom.Xform.Define(stage, ROOT)

    # 창고는 온실 뒤(+Y)에 붙어 있다. 입구가 온실 쪽, 랙이 뒷벽이다.
    wh_front = g.length / 2.0
    wh_back = wh_front + wh.depth
    half_w = g.width / 2.0

    # 랙은 창고 뒷벽 앞에 선다(warehouse.spawn과 같은 식).
    rack_y = wh_front + wh.depth / 2.0 + (wh.depth / 2.0 - RACK_DEPTH / 2.0 - 0.10)

    cams = [
        # 수확 작업: MM 로봇팔과 IW 위 팔레트가 같이 식별돼야 하므로 멀리서
        # 전경을 담지 않고 작업 구역에 붙인다. 온실 높이가 4.5m뿐이라
        # 먼 모서리에 두면 부감각이 12°까지 떨어져 평면처럼 보인다.
        #
        # 이랑 사이 통로(lanes.py VLANES의 x=0) 끝에 세워 통로를 따라 보게 한다.
        # 옆(+X 벽)에서 잡으면 시선이 x=1.45 이랑을 넘어야 하는데 줄기가 1.8m라
        # 캐노피에 아래쪽이 잘린다. 통로축이면 사이를 그대로 통과한다.
        (_add_camera(stage, f"{ROOT}/Greenhouse",
                     eye=(0.0, -g.length / 2.0 + 0.8, 2.6),
                     target=(HARVEST_XY[0], HARVEST_XY[1], 1.1)),
         "/cctv/greenhouse", RES),
        # 인계: 창고 안에서 입구를 통해 온실 쪽 IW를 본다. 시선이 입구 폭
        # 안을 지나야 벽에 안 가리므로 카메라 X를 크게 잡으면 안 된다.
        (_add_camera(stage, f"{ROOT}/Unloading",
                     eye=(3.0, wh_front + 2.5, g.height - 0.7),
                     target=(0.0, IW_DOCK_Y + 0.65, 0.8)),
         "/cctv/unloading", RES),
        # 적재: 랙 정면을 비스듬히. 지게차 진입 경로와 랙이 같이 들어온다.
        (_add_camera(stage, f"{ROOT}/Storage",
                     eye=(half_w - 2.05, wh_front + 2.0, g.height - 0.6),
                     target=(0.0, rack_y, 1.2)),
         "/cctv/storage", RES),
    ]

    # 조감도: 온실+창고 전체를 위에서. 긴 축(Y)이 화면 가로로 오게 돌려야
    # 16:9 화면에 낭비 없이 담긴다.
    center_y = (-g.length / 2.0 + wh_back) / 2.0
    cams.append(
        (_add_camera(stage, f"{ROOT}/Overview",
                     eye=(0.0, center_y, 40.0),
                     target=(0.0, center_y, 0.0),
                     up_hint=(-1.0, 0.0, 0.0),
                     ortho_width=g.length + wh.depth + 2.0),
         "/cctv/overview", RES),
    )

    log(f"[MonitorCam] 고정 카메라 {len(cams)}대: {ROOT}")
    if publish:
        from ros import robot_bridge as RB
        for index, (prim, topic, (width, height)) in enumerate(cams):
            RB.build_rgb_camera(stage, f"/World/RosMonitorCam_{index}",
                                prim, topic, width, height, log=log)
    return cams
