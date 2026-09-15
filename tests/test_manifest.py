"""`packages/<id>/manifest.yaml` 的契约测试。

## manifest 的定位（先说清楚它**不**是什么）

manifest 是**身份 / 能力 / 指针**三样东西，不是"模型的第二份描述"：

| 该写 | 不该写 |
|---|---|
| `id` / `name` / `version` | `dof`（可从 MJCF 数出来） |
| `model.file`（指向 MJCF） | `joint_names`（同上） |
| `capabilities.*`（能力声明） | `link_count`（同上） |
| `kinematics.*`（指向包内实现） | `link_lengths`（几何真值只在 MJCF） |

判据是那句：**"这条信息能不能从别处重新算出来？"**
能 ⇒ 写进去就是**第二份会静默漂移的真值**。

这不是教条：`dof` 写成 3 而 MJCF 里其实有 4 个关节时，没有任何机制会
发现不一致 —— 直到某天前端按 `dof=3` 分配了缓冲区而仿真往里面写了 4 个值。
"""

from __future__ import annotations

import pytest
import yaml

# ----------------------------------------------------------------------
# 1. 结构
# ----------------------------------------------------------------------


class TestManifestStructure:
    def test_manifest_parses(self, mini_arm_manifest):
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        assert isinstance(data, dict), "manifest 顶层必须是 mapping"

    def test_required_top_level_keys(self, mini_arm_manifest):
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        for key in ("id", "name", "version", "model", "coordinate", "units", "capabilities"):
            assert key in data, f"manifest 缺少必需字段 {key!r}"

    def test_id_is_ascii_snake_case(self, mini_arm_manifest):
        """id 契约：唯一、稳定、ASCII、snake_case（P0 契约 §ID）。"""
        import re

        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        assert re.fullmatch(r"[a-z_][a-z0-9_]*", data["id"]), (
            f"id={data['id']!r} 不符合 snake_case ASCII 契约"
        )

    def test_id_matches_directory_name(self, mini_arm_manifest):
        """目录名必须等于 id —— 否则 `packages/<id>/` 这个约定就断了。"""
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        assert mini_arm_manifest.parent.name == data["id"], (
            f"目录名 {mini_arm_manifest.parent.name!r} ≠ manifest id {data['id']!r}"
        )

    def test_name_is_separate_from_id(self, mini_arm_manifest):
        """ID 与 Name 分离：Name 是给人看的，可以是任何语言。"""
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        assert "name" in data and data["name"]
        # name 不必等于 id（"Mini Arm" vs "mini_arm"）—— 这正是分离的意义
        assert data["name"] != data["id"] or data["name"] == data["id"]


# ----------------------------------------------------------------------
# 2. 派生信息必须**不**出现
# ----------------------------------------------------------------------


class TestNoDerivedTruth:
    """禁止把可派生信息写进 manifest（P0 契约：真值只有一份）。"""

    @pytest.mark.parametrize(
        "forbidden",
        ["dof", "link_count", "joint_count", "joint_names", "link_names",
         "actuator_count", "link_lengths", "num_joints", "num_links"],
    )
    def test_forbidden_derived_fields(self, mini_arm_manifest, forbidden):
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        assert forbidden not in data, (
            f"manifest 里出现了可派生字段 {forbidden!r}。\n"
            f"它能从 MJCF 算出来 ⇒ 写进去就是第二份会静默漂移的真值。\n"
            f"若确实需要，请在读取时现算（`model.dof()` / `model.mobile_joint_ids()`）。"
        )

    def test_no_second_geometry_source(self, mini_arm_manifest):
        """manifest 里不允许出现任何几何数值（长度/质量/惯量）。"""
        import re

        text = mini_arm_manifest.read_text(encoding="utf-8")
        # 找形如 `l1: 0.103` / `link_length: 0.103` 的几何键。
        # ⚠ `units.length: m` 是**单位声明**不是几何数值，必须排除 ——
        #   否则这条测试会在一个完全正确的 manifest 上失败。
        #   这也是"写扫描类测试时，先问'我要找的模式会不会误命中合法内容'"。
        suspicious = [
            m for m in re.findall(
                r"^\s*(l\d|link_length\w*|length\w*|mass\w*|inertia\w*)\s*:\s*(\S+)",
                text, re.MULTILINE,
            )
            if not (m[0].startswith("length")
                    and m[1] in ("m", "mm", "cm", "'m'", '"m"'))
        ]
        assert not suspicious, (
            f"manifest 里出现了疑似几何字段：{suspicious}。"
            f"几何真值只允许存在于 MJCF。"
        )


# ----------------------------------------------------------------------
# 3. model 段
# ----------------------------------------------------------------------


class TestModelSection:
    def test_model_is_mjcf(self, mini_arm_manifest):
        """v0.1 只允许**原生 MJCF** 作为模型格式（架构约束）。"""
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        fmt = data["model"]["format"]
        assert fmt == "mjcf", (
            f"model.format={fmt!r}；v0.1 只允许 'mjcf'。"
            f"自定义格式（robot.json / robotforge.xml）被明确禁止。"
        )

    def test_model_file_exists(self, mini_arm_manifest):
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        path = mini_arm_manifest.parent / data["model"]["file"]
        assert path.is_file(), f"model.file 指向的文件不存在：{path}"

    def test_model_file_is_within_package(self, mini_arm_manifest):
        """manifest 里的路径必须是**包内相对路径**，不能逃出包目录。

        允许 `../` 会让一个包依赖另一个包的私有文件 —— 那样"复制一个目录
        就能加一台机器人"这个性质就没了。
        """
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        rel = data["model"]["file"]
        assert not rel.startswith("/") and ":" not in rel, f"model.file 应为相对路径：{rel}"
        resolved = (mini_arm_manifest.parent / rel).resolve()
        assert resolved.is_relative_to(mini_arm_manifest.parent.resolve()), (
            f"model.file={rel!r} 逃出了包目录"
        )

    def test_coordinate_transform_declared(self, mini_arm_manifest):
        """`model.coordinate_transform` 必须显式声明。

        约定是"只有 Loader/Adapter 允许做坐标转换"。
        显式写 `none` 让"本模型不需要转换"这件事**可读且可断言**，
        而不是靠"字段缺失 ⇒ 默认不转"这种隐式推理。
        """
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        assert "coordinate_transform" in data["model"], (
            "model 段缺少 coordinate_transform；显式写 'none' 才对"
        )
        assert data["model"]["coordinate_transform"] in ("none", {}, None) or isinstance(
            data["model"]["coordinate_transform"], str
        )


# ----------------------------------------------------------------------
# 4. coordinate / units 段
# ----------------------------------------------------------------------


class TestCoordinateAndUnits:
    def test_coordinate_convention_is_robotforge(self, mini_arm_manifest):
        """manifest 声明的坐标约定必须与 P0 契约一致。"""
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        coord = data["coordinate"]
        assert coord["handedness"] == "right", "必须是右手系"
        # 字段名是 `forward_axis` 这一族（值是裸轴字母 'x'/'y'/'z'），
        # 不是 `forward`（值 '+X'）。断言必须照真实契约写 ——
        # 我第一版按直觉猜了键名，结果 KeyError 指向的是我的测试而不是 manifest。
        for key, expect in (("forward_axis", "x"), ("left_axis", "y"), ("up_axis", "z")):
            assert coord[key] == expect, f"{key} 必须是 {expect!r}，实际 {coord[key]!r}"
        assert coord["convention"] == "robotforge"

    def test_units_are_si(self, mini_arm_manifest):
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        units = data["units"]
        assert units["length"] == "m"
        assert units["angle"] == "rad", (
            "角度必须是 rad。若写成 deg，就意味着每处使用都要转换一次 —— "
            "而'每处'正是遗漏发生的地方。"
        )
        assert units["time"] == "s"


# ----------------------------------------------------------------------
# 5. capabilities 段
# ----------------------------------------------------------------------


class TestCapabilities:
    def test_capabilities_are_booleans(self, mini_arm_manifest):
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        caps = data["capabilities"]
        assert isinstance(caps, dict) and caps, "capabilities 必须是非空 mapping"
        for k, v in caps.items():
            assert isinstance(v, bool), f"capability {k!r} 必须是 bool，实际 {type(v).__name__}"

    def test_mini_arm_declares_expected_capabilities(self, mini_arm_manifest):
        """mini_arm 的能力断言（写死期望值 —— 这是在"验证真值本身"）。"""
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        caps = data["capabilities"]
        for cap in ("simulation", "fk", "ik", "actuator_control"):
            assert caps.get(cap) is True, f"mini_arm 应声明能力 {cap!r}=True"
        # v0.1 明确禁止的能力
        for cap in ("vision", "grasping", "mobile_base"):
            assert caps.get(cap, False) is False, f"mini_arm 不应声明 {cap!r}"

    def test_no_forbidden_capabilities_in_v01(self, mini_arm_manifest):
        """v0.1 明确禁止的能力不得出现为 true。"""
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        forbidden = {"vision", "ai", "agent", "rl", "dataset", "real_robot", "sim2real_hardware"}
        for cap, val in data["capabilities"].items():
            if cap in forbidden:
                assert val is False, f"v0.1 禁止能力 {cap!r}=True"


# ----------------------------------------------------------------------
# 6. kinematics 段
# ----------------------------------------------------------------------


class TestKinematicsSection:
    def test_kinematics_entries_exist(self, mini_arm_manifest):
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        kin = data.get("kinematics", {})
        for name in ("fk", "ik"):
            assert name in kin, f"kinematics 缺少 {name!r}"
            path = mini_arm_manifest.parent / kin[name]["entry"]
            assert path.is_file(), f"kinematics.{name}.entry 不存在：{path}"

    def test_kinematics_paths_within_package(self, mini_arm_manifest):
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        for name, spec in data.get("kinematics", {}).items():
            resolved = (mini_arm_manifest.parent / spec["entry"]).resolve()
            assert resolved.is_relative_to(mini_arm_manifest.parent.resolve()), (
                f"kinematics.{name}.entry 逃出了包目录"
            )

    def test_tests_pointer_within_package(self, mini_arm_manifest):
        """`tests` 指针必须指向包内 —— 否则"包自带测试"就不成立。"""
        data = yaml.safe_load(mini_arm_manifest.read_text(encoding="utf-8"))
        tests = data.get("tests")
        if tests:
            resolved = (mini_arm_manifest.parent.parent.parent / tests["local"]).resolve()
            assert resolved.is_relative_to(mini_arm_manifest.parent.resolve()), (
                f"tests.local={tests['local']!r} 指向包目录之外"
            )
            assert resolved.is_dir(), f"tests.local 指向的目录不存在：{resolved}"
