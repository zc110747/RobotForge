"""`backendl.loaders.registry` —— 格式分派契约（提示词 §43 / §64）。

## 这个文件在守什么

`RobotModelLoader` 抽象 + `LoaderRegistry` 分派，共同保证：

> **加一个模型格式 = 加一个 Loader 模块 + 一次 register，
> Core（Runtime / api / model）一行不改。**

这条主张**可以失败**，所以值得测。最容易的失效方式是
"分派看起来存在，但实际还硬编码着 MJCF"——那时本文件的
`TestSecondFormatFlowsThroughRuntime` 会红（合成格式加载不了），
而其他测试可能全绿。

## 与 `test_mjcf_loader.py` 的分工

- `test_mjcf_loader.py`：MJCF **内容** → RobotModel 是否正确（翻译保真）
- 本文件：**分派机制**是否正确（谁能被加载、错误怎么报、是否可扩展）

两者都不测对方。所以本文件里大量使用**合成格式**（`toyfmt`），
刻意不用 MJCF —— 否则"分派正确"会被 MJCF 自己的正确性掩盖。
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from backend.loaders import (
    LoaderError,
    LoaderRegistrationError,
    LoaderRegistry,
    LoaderReport,
    RobotModelLoader,
    UnknownFormatError,
    default_loader_registry,
    get_loader_registry,
)
from backend.loaders.mjcf_loader import MJCFLoader
from backend.model import robot_model as RM


class ToyLoader(RobotModelLoader):
    """合成第二格式：JSON 描述的两杆机器人。与 MJCF 无任何关系。"""

    format_name = "toyfmt"

    #: 记录 load 被调用的次数 —— 用来证明"分派真的调到了这个 loader"
    calls: list[str] = []

    def load(self, source):
        ToyLoader.calls.append(str(source))
        spec = json.loads(Path(source).read_text(encoding="utf-8"))
        links = tuple(
            RM.Link(id=l["id"], name=l["id"], parent_joint=l.get("parent_joint"))
            for l in spec["links"]
        )
        joints = tuple(
            RM.Joint(
                id=j["id"],
                name=j["id"],
                type="revolute",
                parent_link=j["parent"],
                child_link=j["child"],
                axis=RM.Vector3.of(j["axis"]),
                limits=RM.JointLimits(
                    position_min=j["min"], position_max=j["max"]
                ),
            )
            for j in spec["joints"]
        )
        frames = [
            RM.Frame(id=f["id"], name=f["id"], parent=f["parent"])
            for f in spec.get("frames", [])
        ]
        # ⚠️ 合成模型要过**与真实模型同一套** Validator，三条隐性要求：
        #   ① base_frame 必须**存在于 frames**（frames 是独立注册表，不是 links）
        #   ② 恰好一个 Link 的 parent_joint 为 None ⇒ 子 link 必须写 parent_joint
        #   ③ revolute 关节要有完整 JointLimits(position_min/max)
        # 不满足时 report.ok 为假 —— "能加载"≠"结构合法"。
        model = RM.RobotModel(
            metadata=RM.RobotMetadata(id=spec["id"], name=spec["id"], version="0.0.0"),
            links=links,
            joints=joints,
            frames=frames,
            root_link=spec["root"],
            base_frame=spec["base_frame"],
        )
        return model, LoaderReport(source="<toy>", format="toyfmt")


@pytest.fixture(autouse=True)
def _reset_toy_calls():
    ToyLoader.calls = []
    yield
    ToyLoader.calls = []


def _write_toy_package(root: Path, pkg_id: str = "toybot") -> Path:
    """在 root 下写一个用 toyfmt 的完整机器人包，返回包目录。"""
    pkg = root / pkg_id
    (pkg / "model").mkdir(parents=True, exist_ok=True)
    (pkg / "model" / "robot.toyfmt").write_text(
        json.dumps(
            {
                "id": pkg_id,
                "root": "base",
                "base_frame": "base",
                "links": [
                    {"id": "base"},
                    {"id": "arm", "parent_joint": "j1"},
                ],
                "joints": [
                    {
                        "id": "j1",
                        "parent": "base",
                        "child": "arm",
                        "axis": [0, 0, 1],
                        "min": -1.57,
                        "max": 1.57,
                    }
                ],
                "frames": [{"id": "base", "parent": "base"}],
            }
        ),
        encoding="utf-8",
    )
    (pkg / "manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "id": pkg_id,
                "name": "Toy Bot",
                "version": "0.1.0",
                "model": {"format": "toyfmt", "file": "model/robot.toyfmt"},
                "capabilities": {"fk": True},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return pkg


# ---------------------------------------------------------------------------
# 注册表本身
# ---------------------------------------------------------------------------


class TestRegistryContract:
    def test_builtin_registry_knows_mjcf(self):
        reg = default_loader_registry()
        assert reg.formats() == ["mjcf"]
        assert isinstance(reg.get("mjcf"), MJCFLoader)

    def test_format_name_comes_from_the_loader_not_the_caller(self):
        """注册名必须取自 `loader.format_name` —— 传参会让两者可能不一致。"""
        reg = LoaderRegistry()
        got = reg.register(ToyLoader())
        assert reg.formats() == ["toyfmt"], "注册名应等于 loader.format_name"
        assert got is not None and isinstance(got, ToyLoader)

    def test_register_returns_the_loader_for_decorator_use(self):
        reg = LoaderRegistry()

        @reg.register
        class L(ToyLoader):
            format_name = "deco"

        assert "deco" in reg

    def test_unknown_format_error_is_actionable(self):
        reg = LoaderRegistry()
        with pytest.raises(UnknownFormatError) as ei:
            reg.get("urdf")
        msg = str(ei.value)
        assert "urdf" in msg
        # 必须告诉用户"现在有哪些"，否则排查没有线索
        assert "当前已注册" in msg

    def test_unknown_format_error_is_a_loader_error(self):
        """`UnknownFormatError` 必须继承 `LoaderError`。

        调用方（API 层）按 `LoaderError` 统一处理"模型层失败"，
        若它落在继承体系之外，就会漏掉这条分支、变成未捕获的 500。
        """
        assert issubclass(UnknownFormatError, LoaderError)

    def test_lookup_is_case_insensitive(self):
        reg = default_loader_registry()
        assert isinstance(reg.get("MJCF"), MJCFLoader)

    def test_duplicate_registration_raises_with_both_names(self):
        reg = default_loader_registry()
        with pytest.raises(LoaderRegistrationError) as ei:
            reg.register(MJCFLoader())
        msg = str(ei.value)
        assert "mjcf" in msg and "MJCFLoader" in msg

    def test_empty_format_name_rejected(self):
        class Bad(RobotModelLoader):
            format_name = ""

            def load(self, source):  # pragma: no cover - 不该被调用
                raise AssertionError

        with pytest.raises(LoaderRegistrationError, match="非空字符串"):
            LoaderRegistry().register(Bad())

    def test_uppercase_format_name_rejected(self):
        class Bad(RobotModelLoader):
            format_name = "ToyFMT"

            def load(self, source):  # pragma: no cover
                raise AssertionError

        with pytest.raises(LoaderRegistrationError, match="全小写"):
            LoaderRegistry().register(Bad())

    def test_unregister_removes_the_format(self):
        reg = LoaderRegistry()
        reg.register(ToyLoader())
        reg.unregister("toyfmt")
        assert reg.formats() == []
        with pytest.raises(UnknownFormatError):
            reg.get("toyfmt")


# ---------------------------------------------------------------------------
# `can_load`：自动分派判据
# ---------------------------------------------------------------------------


class TestCanLoad:
    def test_matches_own_extension(self):
        assert ToyLoader().can_load("/x/y.toyfmt") is True

    def test_does_not_match_other_extension(self):
        assert ToyLoader().can_load("/x/y.mjcf") is False

    def test_matches_xml_string_for_mjcf(self):
        """XML 文本没有扩展名，`can_load` 要看内容前缀。"""
        assert MJCFLoader().can_load("<mujoco/>") is True

    def test_rejects_non_path_non_str(self):
        assert ToyLoader().can_load(12345) is False
        assert ToyLoader().can_load(None) is False

    def test_accepts_pathlib_path(self):
        assert ToyLoader().can_load(Path("/x/y.toyfmt")) is True


# ---------------------------------------------------------------------------
# ★ 可扩展性：第二个格式能穿过整条链路
# ---------------------------------------------------------------------------


class TestSecondFormatFlowsThroughRuntime:
    """Phase 7 的核心断言（§64）。

    如果 `RobotRuntime` 或 `api/registry.py` 里还硬编码着 `MJCFLoader`，
    这组测试会失败 —— 因为它们用的是**完全不是 MJCF** 的格式。
    """

    def test_package_records_the_declared_format(self, tmp_path):
        from backend.api import registry as api_reg

        _write_toy_package(tmp_path)
        reg = default_loader_registry()
        reg.register(ToyLoader())
        api_reg.use_loader_registry(reg)
        try:
            pkg = api_reg.get_package("toybot", tmp_path)
            assert pkg.model_format == "toyfmt"
        finally:
            api_reg.use_loader_registry(None)

    def test_load_model_dispatches_to_the_registered_loader(self, tmp_path):
        from backend.api import registry as api_reg

        _write_toy_package(tmp_path)
        reg = default_loader_registry()
        reg.register(ToyLoader())
        api_reg.use_loader_registry(reg)
        try:
            pkg = api_reg.get_package("toybot", tmp_path)
            assert ToyLoader.calls == [], "发现阶段不该加载模型"
            model, report = pkg.load_model()
            assert ToyLoader.calls, "load_model 必须调到注册的 toyfmt loader"
            assert model.metadata.id == "toybot"
            assert [j.id for j in model.joints] == ["j1"]
        finally:
            api_reg.use_loader_registry(None)

    def test_runtime_loads_a_non_mjcf_robot(self, tmp_path):
        """整条 Runtime 链路（发现 → 加载 → 持有）对第二个格式成立。"""
        import asyncio

        from backend.api import registry as api_reg
        from backend.runtime import RobotRuntime

        _write_toy_package(tmp_path)
        reg = default_loader_registry()
        reg.register(ToyLoader())
        api_reg.use_loader_registry(reg)
        try:
            rt = RobotRuntime(packages_dir=tmp_path)
            asyncio.run(rt.start())
            try:
                assert list(rt.models()) == ["toybot"]
            finally:
                asyncio.run(rt.stop())
        finally:
            api_reg.use_loader_registry(None)

    def test_unregistered_format_is_rejected_at_discovery(self, tmp_path):
        """没有 loader 的格式必须在**发现阶段**就报错，且信息可操作。

        用裸 `LoaderRegistry()`（空表）⇒ 连 mjcf 都没有 ⇒
        报错应提到"已注册：（空）"，让人一眼看出是装配问题。
        """
        from backend.api import registry as api_reg

        _write_toy_package(tmp_path)
        from backend.loaders import LoaderRegistry as _LR

        api_reg.use_loader_registry(_LR())
        try:
            with pytest.raises(api_reg.RobotPackageError) as ei:
                api_reg.discover_packages(tmp_path)
            msg = str(ei.value)
            assert "toyfmt" in msg
            assert "已注册" in msg
        finally:
            api_reg.use_loader_registry(None)

    def test_use_loader_registry_none_restores_default(self, tmp_path):
        """注入必须可撤销 —— 否则一个测试会污染后续所有测试。"""
        from backend.api import registry as api_reg

        api_reg.use_loader_registry(None)
        assert "mjcf" in api_reg._loader_registry()
        # 真实包仍能被发现（证明恢复到了可用的默认表）
        pkgs = api_reg.discover_packages()
        assert any(p.id == "mini_arm" for p in pkgs)


# ---------------------------------------------------------------------------
# 装配：懒加载与全局状态
# ---------------------------------------------------------------------------


class TestAssembly:
    def test_default_registry_is_fresh_each_call(self):
        """`default_loader_registry()` 每次返回**新**实例（测试要能隔离）。"""
        a = default_loader_registry()
        b = default_loader_registry()
        assert a is not b
        a.register(ToyLoader())
        assert "toyfmt" not in b

    def test_get_loader_registry_is_singleton(self):
        assert get_loader_registry() is get_loader_registry()

    def test_registry_module_has_no_hardcoded_format_literals(self):
        """分派模块里**不能**把格式名当字面量硬编码（§69 规则 2 的同族要求）。

        ⚠️ 判据要精确：不能简单地"源码里没有 mjcf 这个子串" ——
        `from .mjcf_loader import MJCFLoader` 里**合法地**包含 "mjcf"
        （那是**模块名**，不是硬编码的格式名）。
        真正要禁止的是"用**字符串**比较格式名"，即：

        ```python
        if fmt != "mjcf": ...        # ✗ 这就是被替换掉的那个 if
        reg.get("mjcf")              # ✗ 硬编码的查表键
        ```

        所以判据必须落在 **STRING 字面量** 上。
        这也正是"剥掉注释与字符串"这个工具在此**不能直接套用**的原因 ——
        本次要查的就是字符串。用 `tokenize` 直接挑出 STRING token 来看。
        """
        import tokenize

        src = (
            Path(__file__).resolve().parent.parent
            / "backend"
            / "loaders"
            / "registry.py"
        ).read_text(encoding="utf-8")
        strings = [
            tok.string
            for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
            if tok.type == tokenize.STRING
        ]
        # 正向证据：确认真的扫到了字符串（否则"没有格式名字面量"是空的结论）
        assert strings, "没有扫到任何字符串字面量 —— 扫描器失效"

        lower = [s.lower().strip("\"'") for s in strings]
        for forbidden in ("mjcf", "urdf", "stl", "obj", "gltf", "glb", "step"):
            assert (
                forbidden not in lower
            ), f"registry.py 把格式名 {forbidden!r} 硬编码成了字符串字面量"
