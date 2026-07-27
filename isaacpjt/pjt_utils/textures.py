# -*- coding: utf-8 -*-
"""PNG 텍스처 머티리얼 헬퍼 — 수확 씬 시각 품질용 (Isaac Sim 5.1 / RTX).

`ripeness.bind_matte_material` 은 displayColor(단색)를 읽는 무광 재질을 만든다.
이 모듈은 같은 자리에 **이미지 텍스처**를 물린 UsdPreviewSurface 를 만든다.

전제 — 대상 메시에 `primvars:st`(UV) 가 있어야 한다.
  - aoc 배경 식물: dae 원본이 TEXCOORD 를 갖고 있어 변환된 usd 에도 st 가 있다(확인함).
  - 수확 대상 토마토: st 가 **없다**(points/normals 뿐). `add_spherical_uv` 로 만든다.

⚠ YOLO 데이터셋 생성(`tomatest/03_generate_from_scene.py`)은 /World/Plants 하위
메시의 바인딩을 `UnbindAllBindings()` 후 displayColor 단색재질로 다시 물린다.
즉 여기서 텍스처를 입혀도 **데이터셋 이미지는 종전대로 단색**이다. 라벨 근거인
displayColor 도 그대로 두므로(텍스처는 재질만 바꾼다) SDG 경로는 영향받지 않는다.
"""
import math

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

# 텍스처 재질을 만들 때 쓰는 공통 반사값. 잎/과피 모두 젖은 광택이 아니라
# 미세하게만 반사한다 — matte 재질(roughness 0.65)과 같은 계열로 맞춘 값이다.
_ROUGHNESS = 0.55
_SPECULAR = 0.15


def bind_texture(stage, prim_path: str, mat_path: str, texture_path: str,
                 *, cutout: bool = False, roughness: float = _ROUGHNESS) -> None:
    """prim_path 에 texture_path 를 diffuse 로 물린 UsdPreviewSurface 를 바인딩.

    mat_path 재질은 한 번만 만들고 이후 호출은 재사용한다 (그루 180개 ×메시 6개가
    같은 재질 6개를 공유 — 재질을 인스턴스마다 만들면 RTX 컴파일이 폭발한다).

    cutout=True 면 텍스처 알파를 opacity 로 물리고 opacityThreshold 를 준다.
    잎/꽃은 사각 판에 알파로 모양을 낸 카드라 이게 없으면 **초록 사각형**으로 렌더된다.
    threshold>0 은 blend 가 아니라 cutout 이라 정렬 비용도 없다.
    """
    if not stage.GetPrimAtPath(mat_path):
        mat = UsdShade.Material.Define(stage, mat_path)
        surf = UsdShade.Shader.Define(stage, mat_path + "/Surface")
        surf.CreateIdAttr("UsdPreviewSurface")
        surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        surf.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(_SPECULAR)

        # st 리더 — varname 은 스펙상 token 이지만 Kit RTX 구현이 string 을 기대한다
        # (ripeness.bind_matte_material 의 displayColor 리더와 같은 함정).
        reader = UsdShade.Shader.Define(stage, mat_path + "/StReader")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
        st_out = reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        tex = UsdShade.Shader.Define(stage, mat_path + "/Diffuse")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
        # 줄기는 UV v 가 0~29 로 타일링돼 있고 꽃/잎도 u 가 0~5 다 → repeat 필수.
        # clamp 로 두면 타일 경계 픽셀이 길게 늘어난다.
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
        rgb_out = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

        surf.CreateInput(
            "diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb_out)
        if cutout:
            a_out = tex.CreateOutput("a", Sdf.ValueTypeNames.Float)
            surf.CreateInput(
                "opacity", Sdf.ValueTypeNames.Float).ConnectToSource(a_out)
            surf.CreateInput(
                "opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.5)
        mat.CreateSurfaceOutput().ConnectToSource(
            surf.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    else:
        mat = UsdShade.Material(stage.GetPrimAtPath(mat_path))

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    # 변환된 usd 메시에는 흰색 DefaultMaterial 이 직접 바인딩돼 있다. 그냥 Bind 하면
    # 그게 이겨서 텍스처가 안 보인다 → 먼저 떼고 강하게 묶는다.
    binding.UnbindAllBindings()
    binding.Bind(mat, UsdShade.Tokens.strongerThanDescendants)


def add_spherical_uv(stage, prim_path: str) -> int:
    """st 가 없는 메시에 구면 투영 UV 를 만들어 넣는다. 넣은 메시 수 반환.

    수확 대상 토마토 usd 는 points/normals 만 있고 UV 가 없다. 토마토는 거의 구라
    구면 투영이면 충분하다 — u=경도, v=위도(정규화 z).

    한계: u 가 0↔1 로 되감기는 자오선에 이음매(seam)가 한 줄 생긴다. 과피
    텍스처(AG15frt*)가 저주파 얼룩이라 실질적으로 안 보이지만, 선명한 무늬
    텍스처로 바꾸면 드러난다. 그때는 메시에 UV 를 구워 오는 게 맞다.
    """
    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        api = UsdGeom.PrimvarsAPI(prim)
        if api.HasPrimvar("st"):
            continue
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not pts:
            continue
        zs = [p[2] for p in pts]
        zmin, zmax = min(zs), max(zs)
        span = (zmax - zmin) or 1.0
        st = [Gf.Vec2f(0.5 + math.atan2(p[1], p[0]) / (2.0 * math.pi),
                       (p[2] - zmin) / span) for p in pts]
        pv = api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                               UsdGeom.Tokens.vertex)
        pv.Set(st)
        count += 1
    return count
