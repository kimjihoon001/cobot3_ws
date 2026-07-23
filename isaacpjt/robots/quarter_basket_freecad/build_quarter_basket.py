#!/usr/bin/env python3
"""FreeCAD 1.1.x — UR10e 동축 3중 1/4구 스쿱 그리퍼 (아이스크림 디셔식).

세 껍질이 **모두 같은 +Z축(= UR 툴/J6, 6축)을 중심으로 제자리 회전**한다.
nested(동축 슬리브 반지름이 서로 다름)라 회전해도 안 부딪힌다. 단위 mm.

★★ 축 규약 = Isaac / UR 툴 프레임 (프로젝트 EE CAD `+Z접근·+Y위·+X옆` 과 동일) ★★
    +Z = 접근축 = 그리퍼가 뻗는 방향 = 세 스쿱의 공통 회전축(J6).
    +Y = 위(up). 수확자세 월드 위쪽 = 줄기가 나오는 쪽 → 여기만 열린 슬릿.
    +X = 옆(lateral).
    원점 = UR 툴 플랜지 접촉면(EN ISO 9409-1-50-4-M6). 그리퍼는 +Z 로 뻗는다.
    과실 중심 C = (0, 0, FRUIT_CENTER_Z).  중력(월드)은 툴프레임 −Y 방향.
    방위각 φ(XY평면, +Z축 중심): +X=0°, +Y(위)=90°, −X=180°, −Y(아래)=270°.

각 껍질 = 구껍질의 방위각 90° 조각(1/4구 = "gore"). +Z축 중심으로 회전해 방위각을 쓴다.
    · 닫힘(크래들): 세 조각이 아래 270°[135°~405°]를 타일링, 위 [45°,135°] 슬릿만 비움.
        - ① 안쪽 receiver : [135,225]  (−X~아래)
        - ② 중간 receiver : [225,315]  (아래 −Y 중심)
        - ③ 바깥 cutter   : [315,45]   (아래~슬릿쪽), 레딩엣지(45°)에 날.
    · 열림(받기/배출): 세 조각을 왼쪽 [135,225]로 모아 아래(−Y)를 연다 → 과실 진입/낙하.
    · 절단: ③ 이 닫힘 뒤 조금 더 회전, 날이 +Y 슬릿의 줄기를 스윕 절단.
      ①② 로 먼저 받쳐 놓고 ③ 이 자르므로 절단 순간에도 과실 낙하 없음.

실행:  FreeCADCmd build_quarter_basket.py
       또는  ./FreeCAD-1.1.1.AppImage --console build_quarter_basket.py
환경변수:  BASKET_POSE=closed|open (기본 closed),  FRUIT_REF=1 (참고 과실구, STEP 미포함),
          BASKET_CAD_DIR=<소스폴더>(콘솔 실행시 __file__ 없을 때 폴백), BASKET_CAD_OUT=<출력>
"""

import json
import math
import os
from pathlib import Path

import FreeCAD as App
import Part

try:
    import Mesh
except ImportError:
    Mesh = None


# FreeCAD 콘솔은 __file__ 을 안 줄 수 있음 → 폴백.
if "__file__" in globals():
    HERE = Path(__file__).resolve().parent
else:
    HERE = Path(os.environ.get("BASKET_CAD_DIR", Path.home())).resolve()
OUT = Path(os.environ.get("BASKET_CAD_OUT", HERE / "generated")).resolve()
POSE = os.environ.get("BASKET_POSE", "closed").lower()
SHOW_FRUIT = os.environ.get("FRUIT_REF", "0") == "1"

# ── 지오메트리 (mm) ────────────────────────────────────────────────────────
FRUIT_DIAMETER = 80.0            # 대과 토마토 목표. ★ Isaac 충돌구 반지름과 맞출 것.
FRUIT_R = FRUIT_DIAMETER / 2.0
FRUIT_CENTER_Z = 120.0           # 플랜지 → 과실중심 접근축(+Z) 거리 (프로젝트 grasp_reach≈115)
C = App.Vector(0.0, 0.0, FRUIT_CENTER_Z)

WALL = 3.0                       # 껍질 두께
GAP = 2.0                        # nested 껍질 사이 반지름 간극(회전 간섭 방지)
CLEAR = 4.0                      # 과실 ~ ① 내면 여유
FRUIT_SHAFT_CLEARANCE = 2.0      # 과실 뒤면 ~ 회전 슬리브/웹 최소 축방향 여유
R1_IN = FRUIT_R + CLEAR                  # ① 안쪽 receiver 내경 = 44
R2_IN = R1_IN + WALL + GAP               # ② 중간 receiver 내경 = 49
R3_IN = R2_IN + WALL + GAP               # ③ 바깥 cutter   내경 = 54

AZ_SPAN = 90.0                   # 각 스쿱 방위각 폭 = 1/4구
CLOSED_AZ = {"inner": 135.0, "middle": 225.0, "outer": 315.0}   # 닫힘 시작 방위각
OPEN_DELTA = {"inner": 0.0, "middle": -90.0, "outer": -180.0}   # 열림(배출) 회전 델타
CUT_DELTA = 50.0                 # ③ 절단 회전(닫힘에서 +): 날이 45°→~95° 로 +Y 줄기 스윕

# 동축 슬리브(모두 +Z축) — 안/중간/바깥 로터 구동 슬리브. 서로 안 겹치게 반지름 스택.
INNER_SLEEVE = (3.0, 6.5)
MIDDLE_SLEEVE = (7.2, 10.0)
OUTER_SLEEVE = (10.7, 14.0)
SLEEVE_START_Z = 18.0
HOUSING_END_Z = 58.0

STEM_RADIUS = 5.0                # 줄기 반지름(③ 셸 모서리 절단 기준)

# UR10e ISO 9409-1-50-4-M6 어댑터
ADAPTER_DIAMETER = 80.0
ADAPTER_THICKNESS = 14.0
FLANGE_PCD = 50.0
M6_CLEARANCE = 6.6
M6_COUNTERBORE = 11.0
DOWEL_RADIUS = 29.6
DOWEL_SLOT_WIDTH = 6.2

ZAXIS = App.Vector(0, 0, 1)
ORIGIN = App.Vector(0, 0, 0)


def bounds_box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, App.Vector(x0, y0, z0))


def tube(r0, r1, z0, z1):
    outer = Part.makeCylinder(r1, z1 - z0, App.Vector(0, 0, z0))
    inner = Part.makeCylinder(r0, z1 - z0 + 2, App.Vector(0, 0, z0 - 1))
    return outer.cut(inner)


def azimuth_sector(radius, z0, z1, az_start, az_span=AZ_SPAN):
    """+Z축 중심 방위각 [az_start, az_start+az_span] 파이 섹터 솔리드(z0..z1)."""
    sec = Part.makeCylinder(radius, z1 - z0, App.Vector(0, 0, z0), ZAXIS, az_span)
    sec.rotate(App.Vector(0, 0, z0), ZAXIS, az_start)
    return sec


def scoop_gore(inner_r, az_start, axial_bore_r):
    """C 중심 구껍질(inner_r..inner_r+WALL)의 방위각 90° 조각 = 1/4구 gore.
    +Z축 중심으로 회전하면 방위각을 스윕한다(아이스크림 디셔 스쿱).
    뒤쪽 구면 캡에는 동축 슬리브가 통과하는 보어를 둔다."""
    r_out = inner_r + WALL
    shell = Part.makeSphere(r_out, C).cut(Part.makeSphere(inner_r, C))
    wedge = azimuth_sector(r_out + 2.0, FRUIT_CENTER_Z - r_out - 2.0,
                           FRUIT_CENTER_Z + r_out + 2.0, az_start)
    gore = shell.common(wedge)
    # Nested inner shafts pass through the polar cap of every outer gore.
    # Cutting only through the rear cap preserves the receiving surface.
    bore_z0 = FRUIT_CENTER_Z - r_out - 1.0
    rear_cap_bore = Part.makeCylinder(
        axial_bore_r, WALL + 2.0, App.Vector(0, 0, bore_z0)
    )
    return gore.cut(rear_cap_bore)


def rotor_connector(inner_r, sleeve, az_start):
    """gore 를 동축 슬리브에 연결하는 뒤판(웹) — 과실 뒤(−Z 극 뒤)라 간섭 없음."""
    r0, r1 = sleeve
    r_out = inner_r + WALL
    # 구껍질의 뒤쪽 캡 내부에 웹을 붙인다. 예전 +5 mm 위치는 과실 내부로
    # 들어갔으므로, inner-sphere 극점보다 1 mm 뒤로 이동한다.
    web_z = FRUIT_CENTER_Z - inner_r - 1.0
    fruit_near_z = FRUIT_CENTER_Z - FRUIT_R
    sleeve_end_z = web_z + 3.0
    if sleeve_end_z > fruit_near_z - FRUIT_SHAFT_CLEARANCE:
        raise RuntimeError(
            "rotor connector enters fruit envelope: %.2f > %.2f"
            % (sleeve_end_z, fruit_near_z - FRUIT_SHAFT_CLEARANCE)
        )
    rho = math.sqrt(max(0.0, r_out ** 2 - (FRUIT_CENTER_Z - web_z) ** 2)) + 0.5
    sleeve_solid = tube(r0, r1, SLEEVE_START_Z, sleeve_end_z)
    web = azimuth_sector(rho, web_z, web_z + 3.0, az_start).cut(
        Part.makeCylinder(r0, 5.0, App.Vector(0, 0, web_z - 1.0)))
    return sleeve_solid.fuse(web).removeSplitter()


def rotating_scoop(inner_r, sleeve, az_start):
    return scoop_gore(inner_r, az_start, sleeve[0]).fuse(
        rotor_connector(inner_r, sleeve, az_start)).removeSplitter()


def ur_adapter_and_housing():
    adapter = Part.makeCylinder(ADAPTER_DIAMETER / 2.0, ADAPTER_THICKNESS)
    for deg in (45, 135, 225, 315):
        a = math.radians(deg)
        x, y = FLANGE_PCD / 2.0 * math.cos(a), FLANGE_PCD / 2.0 * math.sin(a)
        hole = Part.makeCylinder(M6_CLEARANCE / 2.0, ADAPTER_THICKNESS + 2.0,
                                 App.Vector(x, y, -1))
        cbore = Part.makeCylinder(M6_COUNTERBORE / 2.0, 6.0,
                                  App.Vector(x, y, ADAPTER_THICKNESS - 6.0))
        adapter = adapter.cut(hole.fuse(cbore))
    sr = DOWEL_SLOT_WIDTH / 2.0                 # Ø6 위치결정핀 방사형 슬롯(과구속 방지)
    slot = Part.makeCylinder(sr, ADAPTER_THICKNESS + 2.0,
                             App.Vector(DOWEL_RADIUS - 1.5, 0, -1)).fuse(
        Part.makeCylinder(sr, ADAPTER_THICKNESS + 2.0,
                          App.Vector(DOWEL_RADIUS + 1.5, 0, -1))).fuse(
        bounds_box(DOWEL_RADIUS - 1.5, DOWEL_RADIUS + 1.5, -sr, sr, -1,
                   ADAPTER_THICKNESS + 1.0))
    adapter = adapter.cut(slot)
    # 고정 베어링/기어 하우징 — 세 동축 슬리브를 감싼다(힌지 기둥 없음).
    housing = tube(OUTER_SLEEVE[1] + 1.0, OUTER_SLEEVE[1] + 20.0,
                   ADAPTER_THICKNESS, HOUSING_END_Z)
    return adapter.fuse(housing).removeSplitter()


def fruit_reference():
    return Part.makeSphere(FRUIT_R, C)


def posed(shape, key):
    """BASKET_POSE=open 이면 +Z축 중심으로 OPEN_DELTA 만큼 회전(배출 자세)."""
    if POSE == "open":
        shape = shape.copy()
        shape.rotate(ORIGIN, ZAXIS, OPEN_DELTA[key])
    return shape


def rotated_copy(shape, angle):
    result = shape.copy()
    result.rotate(ORIGIN, ZAXIS, angle)
    return result


def assert_no_interference(named_shapes, tolerance=1.0e-4):
    """Fail generation if any two solids have a non-zero overlap volume."""
    names = list(named_shapes)
    for i, name_a in enumerate(names):
        for name_b in names[i + 1:]:
            overlap = named_shapes[name_a].common(named_shapes[name_b])
            volume = 0.0 if overlap.isNull() else overlap.Volume
            if volume > tolerance:
                raise RuntimeError(
                    "interference: %s vs %s = %.6f mm^3"
                    % (name_a, name_b, volume)
                )


def add_shape(doc, name, label, shape, color, transparency=0):
    if shape.isNull() or not shape.isValid() or shape.Volume <= 0:
        raise RuntimeError("{} is not a valid solid".format(name))
    obj = doc.addObject("Part::Feature", name)
    obj.Label = label
    obj.Shape = shape
    obj.ViewObject.ShapeColor = color
    obj.ViewObject.LineColor = (0.12, 0.12, 0.12)
    if transparency:
        obj.ViewObject.Transparency = transparency
    return obj


def add_joint_metadata(obj, joint_name, closed, other):
    obj.addProperty("App::PropertyString", "JointName", "Kinematics")
    obj.addProperty("App::PropertyString", "JointAxis", "Kinematics")
    obj.addProperty("App::PropertyAngle", "ClosedAngle", "Kinematics")
    obj.addProperty("App::PropertyAngle", "OpenOrCutAngle", "Kinematics")
    obj.JointName = joint_name
    obj.JointAxis = "tool0 +Z (J6)"
    obj.ClosedAngle = closed
    obj.OpenOrCutAngle = other


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if "CoaxialQuarterScoopGripper" in App.listDocuments():
        App.closeDocument("CoaxialQuarterScoopGripper")
    doc = App.newDocument("CoaxialQuarterScoopGripper")

    frame = ur_adapter_and_housing()
    inner = rotating_scoop(R1_IN, INNER_SLEEVE, CLOSED_AZ["inner"])
    middle = rotating_scoop(R2_IN, MIDDLE_SLEEVE, CLOSED_AZ["middle"])
    outer = rotating_scoop(R3_IN, OUTER_SLEEVE, CLOSED_AZ["outer"])
    fruit = fruit_reference()

    # Closed pose: all three rotors, fixed housing and reference tomato.
    assert_no_interference({
        "fixed_housing": frame,
        "receiver_1": inner,
        "receiver_2": middle,
        "cutter_3": outer,
        "fruit_envelope": fruit,
    })
    # Open pose: all three gores stack azimuthally but remain radially nested.
    assert_no_interference({
        "fixed_housing": frame,
        "receiver_1_open": rotated_copy(inner, OPEN_DELTA["inner"]),
        "receiver_2_open": rotated_copy(middle, OPEN_DELTA["middle"]),
        "cutter_3_open": rotated_copy(outer, OPEN_DELTA["outer"]),
        "fruit_envelope": fruit,
    })

    base = add_shape(doc, "UR10eAdapter", "UR10e 어댑터+고정 베어링 하우징",
                     frame, (0.48, 0.50, 0.53))
    s1 = add_shape(doc, "ScoopQuarter1", "① 안쪽 receiver (Z회전)",
                   posed(inner, "inner"), (0.16, 0.58, 0.24))
    s2 = add_shape(doc, "ScoopQuarter2", "② 중간 receiver (Z회전)",
                   posed(middle, "middle"), (0.15, 0.44, 0.75))
    s3 = add_shape(doc, "CutterQuarter3", "③ 바깥 셸 모서리 cutter (Z회전)",
                   posed(outer, "outer"), (0.90, 0.52, 0.12))

    add_joint_metadata(s1, "scoop_quarter_1_joint", 0.0, OPEN_DELTA["inner"])
    add_joint_metadata(s2, "scoop_quarter_2_joint", 0.0, OPEN_DELTA["middle"])
    add_joint_metadata(s3, "cutter_quarter_3_joint", 0.0, CUT_DELTA)
    s3.addProperty("App::PropertyString", "CuttingEdge", "Kinematics")
    s3.CuttingEdge = "leading meridian edge at closed azimuth 45 deg"

    if SHOW_FRUIT:
        add_shape(doc, "FruitReference", "참고용 과실구", fruit,
                  (0.85, 0.25, 0.20), transparency=60)

    params = doc.addObject("App::FeaturePython", "DesignParameters")
    for name, value in (("FruitDiameter", FRUIT_DIAMETER),
                        ("FruitCenterFromTool0", FRUIT_CENTER_Z),
                        ("InnerReceiverR", R1_IN), ("MiddleReceiverR", R2_IN),
                        ("OuterCutterR", R3_IN), ("ShellWall", WALL)):
        params.addProperty("App::PropertyLength", name, "Dimensions")
        setattr(params, name, value)
    params.addProperty("App::PropertyString", "Mechanism", "Design")
    params.Mechanism = "3 coaxial 1/4-sphere gores rotating about tool0/J6 +Z"

    doc.recompute()
    fcstd = OUT / "coaxial_quarter_scoop_gripper.FCStd"
    doc.saveAs(str(fcstd))

    objs = [base, s1, s2, s3]
    Part.export(objs, str(OUT / "coaxial_quarter_scoop_gripper.step"))
    for obj in objs:
        Part.export([obj], str(OUT / (obj.Name + ".step")))
        if Mesh is not None:
            Mesh.export([obj], str(OUT / (obj.Name + ".stl")))

    dims = {
        "units": "mm",
        "axis_convention": "Isaac/UR tool: +Z=approach=rotation axis(J6), +Y=up(stem/slit), +X=lateral",
        "mechanism": "3 coaxial 1/4-sphere gores, in-place rotation about +Z (ice-cream scoop)",
        "fruit_diameter": FRUIT_DIAMETER,
        "fruit_center_z": FRUIT_CENTER_Z,
        "scoop_inner_radii": {"inner": R1_IN, "middle": R2_IN, "outer": R3_IN},
        "wall": WALL, "gap": GAP, "az_span_deg": AZ_SPAN,
        "fruit_to_shaft_clearance": FRUIT_SHAFT_CLEARANCE,
        "closed_azimuth_deg": CLOSED_AZ,
        "open_delta_deg": OPEN_DELTA,
        "cut_delta_deg": CUT_DELTA,
        "cutting_edge": "outer quarter leading meridian; no separate blade",
        "open_top_slit_deg": [45.0, 135.0],
        "all_joint_axes": "+Z (tool0/J6, coaxial in-place rotation)",
        "robot_interface": "EN ISO 9409-1-50-4-M6",
    }
    (OUT / "dimensions.json").write_text(
        json.dumps(dims, indent=2, ensure_ascii=False), encoding="utf-8")
    print("[quarter-scoop] pose=%s → %s" % (POSE, fcstd))
    print("[quarter-scoop] 3 x +Z coaxial gores; outer leading edge cuts stem.")
    print("[quarter-scoop] interference check: closed/open/fruit = PASS")


if __name__ == "__main__":
    main()
