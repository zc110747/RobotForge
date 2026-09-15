"""`backend.loaders.asset_loader` —— GeometryAsset 扩展点契约（提示词 §44 / §65）。

## 这个文件在守什么

§65 要求「只验证 `RobotModel → GeometryAsset → AssetLoader`」，
即**这条缝存在且真的可用**，但**不实现**任何真 parser。

"缝存在且真的可用"必须能失败。最容易的失效方式是：
`GeometryAssetRegistry` 是个空壳类（注册进去了但 `resolve` 行为不变）。
所以本文件的核心是 `TestSeamIsReal` ——
它断言"注册前后行为**不同**"。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from backend.loaders import (
    AssetError,
    AssetLoader,
    AssetLoaderRegistrationError,
    GeometryAsset,
    GeometryAssetRegistry,
    UnsupportedAssetError,
    default_asset_registry,
    get_asset_registry,
    register_asset_loader,
)
from backend.loaders.asset_loader import AssetLoadError
from backend.model import robot_model as RM


class ToyMeshLoader(AssetLoader):
    """合成 mesh loader：认 `.toymesh`，内容是若干行 `x y z`。"""

    extensions = ("toymesh",)
    calls: list[str] = []

    def load(self, source, *, name: str = "") -> GeometryAsset:
        ToyMeshLoader.calls.append(str(source))
        path = Path(source)
        verts = [
            [float(t) for t in line.split()]
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.startswith("#")
        ]
        return GeometryAsset(
            name=name or path.name,
            kind="mesh",
            format="toymesh",
            data={"vertices": verts},
            metadata={"vertex_count": len(verts)},
        )


@pytest.fixture(autouse=True)
def _reset_calls():
    ToyMeshLoader.calls = []
    yield
    ToyMeshLoader.calls = []


def _write_mesh(path: Path, n: int = 3) -> Path:
    lines = ["# toy mesh"] + [f"{i} 0 0" for i in range(n)]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


# ---------------------------------------------------------------------------
# 抽象的形状
# ---------------------------------------------------------------------------


class TestAbstractionShape:
    def test_asset_loader_is_abstract(self):
        with pytest.raises(TypeError):
            AssetLoader()  # type: ignore[abstract]

    def test_geometry_asset_is_frozen(self):
        a = GeometryAsset(name="n", kind="mesh", format="stl")
        with pytest.raises(Exception):
            a.name = "other"  # type: ignore[misc]

    def test_to_dict_omits_data(self):
        """`to_dict()` 不得含 `data` —— 它可能被塞进 WS 帧（§24 的理由）。"""
        a = GeometryAsset(
            name="n", kind="mesh", format="stl", data={"big": [1] * 1000}
        )
        d = a.to_dict()
        assert "data" not in d
        assert d["kind"] == "mesh" and d["format"] == "stl"

    def test_error_hierarchy(self):
        """`UnsupportedAssetError` = "不知道怎么读"；
        `AssetLoadError` = "读的时候坏了"。都继承 `AssetError`。"""
        assert issubclass(UnsupportedAssetError, AssetError)
        assert issubclass(AssetLoadError, AssetError)
        # 两者不可互相替代 —— 修复动作不同
        assert not issubclass(UnsupportedAssetError, AssetLoadError)
        assert not issubclass(AssetLoadError, UnsupportedAssetError)


# ---------------------------------------------------------------------------
# v0.1 的"什么都没有"状态
# ---------------------------------------------------------------------------


class TestV01IsEmpty:
    def test_default_registry_is_empty_in_v0_1(self):
        """§65：v0.1 不实现 STL/OBJ/GLTF/STEP parser ⇒ 默认注册表为空。"""
        assert default_asset_registry().extensions() == []
        assert len(default_asset_registry()) == 0

    def test_native_geometry_resolves_to_none(self):
        """原生 geom（无外部资源）必须返回 `None`，**不抛异常**。

        这是 v0.1 的**正常路径**：box/cylinder/sphere 本来就没有 asset。
        让它抛异常会逼所有调用方写 try/except。
        """
        reg = default_asset_registry()
        ref = RM.GeometryRef(type="cylinder", size=[0.02, 0.03])
        assert ref.asset is None
        assert reg.resolve(ref.asset) is None

    def test_empty_string_asset_resolves_to_none(self):
        assert default_asset_registry().resolve("") is None

    def test_missing_loader_error_names_v0_1_reality(self):
        """报错必须说清"v0.1 一个 parser 都没实现"，否则用户会以为文件有问题。"""
        with pytest.raises(UnsupportedAssetError) as ei:
            default_asset_registry().resolve("wheel.stl")
        msg = str(ei.value)
        assert "stl" in msg
        assert "v0.1" in msg

    def test_all_future_formats_are_unsupported_in_v0_1(self):
        """§44 列出的五种未来格式，v0.1 必须**全部**不支持。"""
        reg = default_asset_registry()
        for name in ("a.stl", "a.obj", "a.gltf", "a.glb", "a.step"):
            with pytest.raises(UnsupportedAssetError):
                reg.resolve(name)

    def test_registry_accepts_being_empty_without_crashing(self):
        reg = GeometryAssetRegistry()
        assert reg.extensions() == []
        assert "stl" not in reg
        assert len(reg) == 0
        assert reg.resolve(None) is None


# ---------------------------------------------------------------------------
# ★ 缝是真的：注册前后行为不同
# ---------------------------------------------------------------------------


class TestSeamIsReal:
    """Phase 8 的核心断言（§65）。

    这一组必须能失败。若 `GeometryAssetRegistry` 退化成空壳
    （注册了但 resolve 不查表），`test_registering_changes_behaviour` 会红。
    """

    def test_registering_changes_behaviour(self, tmp_path):
        """★ 注册前抛错、注册后成功 —— 同一个引用，两种结果。"""
        mesh = _write_mesh(tmp_path / "wheel.toymesh")

        empty = GeometryAssetRegistry()
        with pytest.raises(UnsupportedAssetError):
            empty.resolve(str(mesh))

        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())
        asset = reg.resolve(str(mesh))
        assert isinstance(asset, GeometryAsset)
        assert asset.metadata["vertex_count"] == 3

    def test_only_the_registered_extension_works(self, tmp_path):
        """注册 `toymesh` 不能让 `.stl` 也变可用（防"只要非空就全放行"）。"""
        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())
        assert "toymesh" in reg
        with pytest.raises(UnsupportedAssetError):
            reg.resolve("wheel.stl")

    def test_resolve_dispatches_to_the_loader(self, tmp_path):
        mesh = _write_mesh(tmp_path / "arm.toymesh")
        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())
        reg.resolve(str(mesh))
        assert ToyMeshLoader.calls == [str(mesh)]

    def test_geometry_ref_with_asset_resolves(self, tmp_path):
        """从 `GeometryRef.asset` 这个**真实字段**出发，不手搓字符串。"""
        mesh = _write_mesh(tmp_path / "link.toymesh", n=5)
        ref = RM.GeometryRef(type="mesh", size=[], asset=str(mesh))
        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())
        asset = reg.resolve(ref.asset, name="link-0")
        assert asset.name == "link-0"
        assert asset.metadata["vertex_count"] == 5

    def test_name_defaults_to_the_reference(self, tmp_path):
        """未显式给 `name` 时，loader 拿到的 `name` 是**完整的引用字符串**。

        这是刻意的：`asset` 是"引用"，可能是个相对路径（`meshes/wheel.stl`），
        而 loader 才知道该怎么把它变成展示名。
        平台层**不**替它 `Path(...).name` —— 那会丢掉目录信息。
        """
        mesh = _write_mesh(tmp_path / "plain.toymesh")
        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())
        # 传显式 name
        assert reg.resolve(str(mesh), name="plain").name == "plain"
        # 不传 name ⇒ loader 收到完整引用，由它自己决定展示名
        assert reg.resolve(str(mesh)).name == str(mesh)

    def test_robot_model_geometry_can_carry_an_asset_reference(self):
        """`GeometryRef.asset` 字段必须真的能被赋值并保留（否则缝不存在）。"""
        ref = RM.GeometryRef(type="mesh", size=[], asset="wheel.stl")
        assert ref.asset == "wheel.stl"
        assert ref.to_dict()["asset"] == "wheel.stl"

    def test_mjcf_loader_keeps_asset_none_in_v0_1(self, mini_arm_model):
        """v0.1 从 MJCF 加载出来的一切 `asset` 都必须是 `None`。

        这条与上面的"缝存在"配对：缝开着，但 v0.1 没人用它 ——
        若哪天有人偷偷给某个 geom 填了 asset 却没实现 loader，
        这条会红，而不会在运行期变成"几何静默消失"。
        """
        for link in mini_arm_model.links:
            for g in (*link.visual, *link.collision):
                assert g.asset is None, f"{link.id} 的几何带了 asset={g.asset!r}"


# ---------------------------------------------------------------------------
# 注册表的错误处理
# ---------------------------------------------------------------------------


class TestRegistrationRules:
    def test_empty_extensions_rejected(self):
        class Bad(AssetLoader):
            extensions = ()

            def load(self, source, *, name=""):  # pragma: no cover
                raise AssertionError

        with pytest.raises(AssetLoaderRegistrationError, match="非空"):
            GeometryAssetRegistry().register(Bad())

    def test_uppercase_extension_rejected(self):
        class Bad(AssetLoader):
            extensions = ("STL",)

            def load(self, source, *, name=""):  # pragma: no cover
                raise AssertionError

        with pytest.raises(AssetLoaderRegistrationError, match="全小写"):
            GeometryAssetRegistry().register(Bad())

    def test_leading_dot_rejected(self):
        class Bad(AssetLoader):
            extensions = (".stl",)

            def load(self, source, *, name=""):  # pragma: no cover
                raise AssertionError

        with pytest.raises(AssetLoaderRegistrationError, match="前导点"):
            GeometryAssetRegistry().register(Bad())

    def test_duplicate_extension_rejected(self):
        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())

        class Also(AssetLoader):
            extensions = ("toymesh",)

            def load(self, source, *, name=""):  # pragma: no cover
                raise AssertionError

        with pytest.raises(AssetLoaderRegistrationError, match="重复占用"):
            reg.register(Also())

    def test_multi_extension_loader_registers_all(self):
        class Multi(AssetLoader):
            extensions = ("aaa", "bbb")

            def load(self, source, *, name=""):
                return GeometryAsset(name=name, kind="uri", format="x")

        reg = GeometryAssetRegistry()
        reg.register(Multi())
        assert reg.extensions() == ["aaa", "bbb"]
        reg.resolve("f.aaa")
        reg.resolve("f.bbb")

    def test_partial_registration_does_not_happen(self):
        """冲突时必须**一个都不注册**（避免"一半新一半旧"的注册表）。"""
        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())

        class Clash(AssetLoader):
            extensions = ("fresh", "toymesh")  # 第二个冲突

            def load(self, source, *, name=""):  # pragma: no cover
                raise AssertionError

        with pytest.raises(AssetLoaderRegistrationError):
            reg.register(Clash())
        assert "fresh" not in reg, "冲突时不该留下部分注册"
        assert reg.extensions() == ["toymesh"]

    def test_unregister(self):
        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())
        reg.unregister("toymesh")
        assert reg.extensions() == []


# ---------------------------------------------------------------------------
# 扩展名解析的边界（`.stl` 这种前导点路径）
# ---------------------------------------------------------------------------


class TestExtensionParsing:
    def test_plain_filename(self, tmp_path):
        mesh = _write_mesh(tmp_path / "a.toymesh")
        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())
        assert reg.resolve(str(mesh)) is not None

    def test_dotfile_only(self):
        """`Path('.zzz').suffix` 是**空串** —— pathlib 把前导点当隐藏文件名。

        这条守住"resolve 不能只用 `Path(...).suffix` 一种写法"：
        若只取 suffix，`.zzz` 会解析出空扩展名，
        报错会变成"没有处理扩展名 '' 的 loader" —— 完全无法定位。
        """
        reg = GeometryAssetRegistry()
        reg.register(_StubLoader())
        with pytest.raises(UnsupportedAssetError) as ei:
            reg.resolve(".zzz")
        assert "'zzz'" in str(ei.value), f"应解析出 'zzz'，实际：{ei.value}"

    def test_bare_extension_argument(self):
        reg = GeometryAssetRegistry()
        reg.register(_StubLoader())
        # 直接给扩展名（无点）也应能命中
        assert "stub" in reg
        reg.resolve("stub")

    def test_multi_dot_filename_uses_last_segment(self):
        reg = GeometryAssetRegistry()
        reg.register(_StubLoader())
        # 关键：不能把整串当扩展名
        reg.resolve("a.b.stub")

    def test_uppercase_filename_matches_lowercase_registration(self, tmp_path):
        mesh = _write_mesh(tmp_path / "WHEEL.TOYMESH")
        reg = GeometryAssetRegistry()
        reg.register(ToyMeshLoader())
        assert reg.resolve(str(mesh)) is not None


class _StubLoader(AssetLoader):
    """认 `.stub`，返回 uri 形态（不碰文件系统，便于测解析）。"""

    extensions = ("stub",)

    def load(self, source, *, name=""):
        return GeometryAsset(name=name or str(source), kind="uri", format="stub")


# ---------------------------------------------------------------------------
# 装配
# ---------------------------------------------------------------------------


class TestAssembly:
    def test_default_registry_is_fresh(self):
        a = default_asset_registry()
        b = default_asset_registry()
        assert a is not b
        a.register(ToyMeshLoader())
        assert b.extensions() == []

    def test_get_asset_registry_is_singleton(self):
        assert get_asset_registry() is get_asset_registry()

    def test_register_asset_loader_uses_the_global(self):
        """全局注册后必须能从全局查到（并清理干净）。"""
        reg = get_asset_registry()
        try:
            register_asset_loader(ToyMeshLoader())
            assert "toymesh" in get_asset_registry()
        finally:
            reg.unregister("toymesh")
        assert "toymesh" not in get_asset_registry()

    def test_asset_module_has_no_hardcoded_format_names(self):
        """`asset_loader.py` 不得把具体格式名写成**字符串字面量**。

        （同 `test_loader_registry.py` 的判据：查 STRING token，
        因为模块名/类名里出现 "stl" 是合法的。）
        """
        import tokenize

        src = (
            Path(__file__).resolve().parent.parent
            / "backend"
            / "loaders"
            / "asset_loader.py"
        ).read_text(encoding="utf-8")
        strings = [
            tok.string.lower().strip("\"'")
            for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
            if tok.type == tokenize.STRING
        ]
        assert strings, "没有扫到字符串字面量 —— 扫描器失效"
        for forbidden in ("stl", "obj", "gltf", "glb", "step"):
            assert forbidden not in strings, f"asset_loader.py 硬编码了 {forbidden!r}"
