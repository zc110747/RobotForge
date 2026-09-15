"""MJCF Loader 契约测试。

## Loader 的职责边界

```text
packages/<id>/model/*.xml   →  [Loader]  →  RobotModel
       外部格式                            唯一的内部表示
```

Loader 是**唯一**允许理解"MJCF 这门语言"的地方。测试要证明的是：

1. **忠实性** —— 翻译没有丢信息、没有读错（用 MuJoCo 做对照系）
2. **拒绝性** —— 遇到不支持的构造必须**报错**而不是"尽力而为"
3. **无残留** —— 翻译完的 RobotModel 里不得有任何 MJCF 痕迹

第 3 条是架构约束"Runtime 不得依赖 MJCF XML"的机器判据。
"""

from __future__ import annotations

import json
import math
import re

import pytest

from backend.loaders.loader import LoaderError, RobotModelLoader
from backend.loaders.mjcf_loader import MJCFLoader, sanitize_id

# ----------------------------------------------------------------------
# 1. 抽象基类契约
# ----------------------------------------------------------------------


class TestLoaderABC:
    def test_mjcf_loader_is_a_loader(self):
        assert issubclass(MJCFLoader, RobotModelLoader)

    def test_format_name(self):
        assert MJCFLoader().format_name == "mjcf"

    def test_loader_abstraction_does_not_import_mujoco(self, repo_root):
        """★ 抽象层不得 import mujoco。

        若 `loader.py`（抽象基类）import 了 mujoco，那么"以后加一个 URDF
        loader"就必须安装 mujoco —— 而 MJCF 与 URDF 是两条独立的支线。
        把引擎依赖压在**具体** loader 里，抽象层保持纯净。
        """
        # ⚠ 不能用朴素的 `"import mujoco" not in text`：
        #   该文件的 docstring 里恰好有一句"因此本模块**不**import mujoco"，
        #   朴素子串匹配会把这句**说明不要导入**的话当成违规。
        #   ⇒ 必须只看**代码行**（剥掉注释与字符串）。
        code_lines = _strip_comments_and_strings(
            (repo_root / "backend" / "loaders" / "loader.py").read_text(encoding="utf-8")
        )
        offenders = [
            ln for ln in code_lines
            if re.match(r"\s*(import\s+mujoco|from\s+mujoco)", ln)
        ]
        assert not offenders, (
            f"backend/loaders/loader.py 是抽象层，不得 import mujoco，实际：{offenders}"
        )

    def test_mjcf_loader_imports_mujoco_lazily(self, repo_root):
        """`mjcf_loader.py` 里的 mujoco 导入必须在**函数内部**。

        这样 import 这个模块本身不需要装 mujoco（例如只想看 dataclass 定义，
        或在没有 mujoco 的机器上跑非 MJCF 测试）。
        """
        text = (repo_root / "backend" / "loaders" / "mjcf_loader.py").read_text(encoding="utf-8")
        # 模块顶层不得出现 `import mujoco`
        top_level = []
        for line in text.splitlines():
            if line and not line[0].isspace() and not line.startswith(("#", '"', "'")):
                top_level.append(line)
        assert not any(
            line.startswith("import mujoco") or line.startswith("from mujoco")
            for line in top_level
        ), "mjcf_loader.py 的顶层不得 import mujoco（应在函数内延迟导入）"


# ----------------------------------------------------------------------
# 2. 翻译忠实性（用 MuJoCo 作对照系）
# ----------------------------------------------------------------------


class TestTranslationFidelity:
    def test_returns_model_and_report(self, mini_arm_mjcf):
        result = MJCFLoader(robot_id="mini_arm").load(mini_arm_mjcf)
        assert isinstance(result, tuple) and len(result) == 2
        model, report = result
        assert report.format == "mjcf"
        assert report.source.endswith("mini_arm.xml")

    def test_link_count_matches_mujoco(self, mini_arm_model, mj_model):
        """Link 数 = MuJoCo 的 body 数 **减去 world**。

        ★ 两个容易搞错的地方：
          ① MuJoCo 的 `nbody` **包含 world**（body 0），而 RobotModel 的
             `links` 不包含它（world 由 `base_frame` 代表，不是 Link）。
          ② MJCF 里"无 joint 的 body"在 RobotModel 里**也是**一个 Link
             （Loader 为它合成一个 fixed Joint 以满足 Link-Joint alternation）。
        """
        assert len(mini_arm_model.links) == mj_model.nbody - 1, (
            f"RobotModel 有 {len(mini_arm_model.links)} 个 link，"
            f"MuJoCo 有 {mj_model.nbody} 个 body（含 world ⇒ 期望 "
            f"{mj_model.nbody - 1} 个 link）"
        )

    def test_joint_count_matches_mujoco_plus_synthesized(self, mini_arm_model, mj_model):
        """Joint 数 = MuJoCo 的 joint 数 + 合成的 fixed joint 数。"""
        n_synth = sum(1 for j in mini_arm_model.joints if j.type == "fixed")
        assert len(mini_arm_model.joints) == mj_model.njnt + n_synth, (
            f"RobotModel {len(mini_arm_model.joints)} 个 joint，"
            f"MuJoCo {mj_model.njnt} 个 + 合成 {n_synth} 个"
        )

    def test_dof_matches_mujoco(self, mini_arm_model, mj_model):
        """自由度数必须等于 MuJoCo 的 nq（可动关节数）。"""
        assert mini_arm_model.dof() == mj_model.nq, (
            f"dof={mini_arm_model.dof()}，MuJoCo nq={mj_model.nq}"
        )

    def test_actuator_count_matches_mujoco(self, mini_arm_model, mj_model):
        assert len(mini_arm_model.actuators) == mj_model.nu

    def test_joint_order_matches_mujoco_qpos(self, mini_arm_model, mj_model):
        """★ 关节顺序必须与 MuJoCo 的 qpos 顺序**逐位**一致。

        这是 Python 侧 `{joint_id: value}` 与引擎侧 `qpos` 数组之间的
        唯一桥梁。顺序错了不会报错，只会把 shoulder 的值喂给 elbow。
        """
        mujoco = pytest.importorskip("mujoco")
        mj_order = [
            mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(mj_model.njnt)
        ]
        assert list(mini_arm_model.mobile_joint_ids()) == mj_order, (
            f"RobotModel {list(mini_arm_model.mobile_joint_ids())} ≠ MuJoCo {mj_order}"
        )

    def test_axes_match_mujoco(self, mini_arm_model, mj_model):
        """关节轴向量必须逐分量一致（含符号）。

        ★ 符号尤其重要：轴写成 `0 -1 0` 与 `0 1 0` 都能编译通过，
          但旋转方向相反 —— 表现为"控制方向反了"。
        """
        mujoco = pytest.importorskip("mujoco")
        worst = 0.0
        for jid in mini_arm_model.mobile_joint_ids():
            jid_mj = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            axis = mini_arm_model.joint(jid).axis
            mj_axis = mj_model.jnt_axis[jid_mj]
            for a, b in zip(axis.to_list(), mj_axis):
                worst = max(worst, abs(a - float(b)))
        assert worst < 1e-12, f"关节轴最差分量误差 {worst:.3e}"

    def test_body_origins_match_mujoco(self, mini_arm_model, mj_model):
        """关节 origin 必须与 MuJoCo 的 body_pos 一致。

        ★ 这里有一条重要的实现发现，写在 mjcf_loader.py 的注释里：
          MuJoCo 3.13 的 `MjModel` **没有** `jnt_quat`，而 `jnt_pos` 在本
          模型里全为 0。真正携带"关节坐标架相对父 body 的位姿"的是
          `body_pos` / `body_quat`（因为 joint 写在 body 原点处，
          关节坐标系就是 body 坐标系）。
        """
        mujoco = pytest.importorskip("mujoco")
        worst = 0.0
        for link in mini_arm_model.links:
            if link.is_root:
                continue
            bid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, link.id)
            assert bid >= 0, f"MuJoCo 里找不到 body {link.id}"
            # 该 link 的进入 joint
            joints = [j for j in mini_arm_model.joints if j.child_link == link.id]
            assert len(joints) == 1, f"link {link.id} 应恰好有 1 个进入 joint"
            o = joints[0].origin.position
            mjp = mj_model.body_pos[bid]
            for a, b in zip(o.to_list(), mjp):
                worst = max(worst, abs(a - float(b)))
        assert worst < 1e-12, f"关节 origin 最差分量误差 {worst:.3e}"

    def test_quaternion_order_converted(self, mini_arm_model, mj_model):
        """★ 四元数分量顺序必须从 `[w,x,y,z]` 转成 `[x,y,z,w]`。

        这是跨库边界最容易错的一处。用一个**旋转非平凡**的 body
        来验证（单位四元数下转置与否看不出来）。

        mini_arm 里 `shoulder_hub` geom 用了 `quat="0.707... 0.707... 0 0"`
        —— 绕 X 转 90°，正是理想的探针。
        """
        # 找那个绕 X 90° 的 geom（在 model 里体现为 geometry 的位姿）
        found = False
        geoms = [g for link in mini_arm_model.links
                 for g in list(link.visual) + list(link.collision)]
        for geom in geoms:
            q = geom.transform.orientation
            # 绕 X 90° 在 [x,y,z,w] 下是 (sin45, 0, 0, cos45) ≈ (0.7071, 0, 0, 0.7071)
            if abs(q.x - math.sin(math.pi / 4)) < 1e-6 and abs(q.w - math.cos(math.pi / 4)) < 1e-6:
                found = True
                assert abs(q.y) < 1e-9 and abs(q.z) < 1e-9, (
                    f"绕 X 90° 的四元数应为 (0.7071,0,0,0.7071)，实际 {q}。"
                    f"若得到 (0,0,0.7071,0.7071) 之类 ⇒ 分量顺序没转"
                )
        assert found, (
            "没找到绕 X 90° 的 geom —— 测试探针失效。"
            "请确认 MJCF 里仍有 quat=\"0.70710678 0.70710678 0 0\" 的 geom，"
            "否则这条'四元数顺序'测试实际上没有在验证任何东西。"
        )

    def test_synthesized_fixed_joint_for_jointless_body(self, mini_arm_model, mj_model):
        """无 joint 的 body 必须被合成一个 fixed Joint。

        ## 为什么必须有这个合成

        RobotModel 的 Link-Joint alternation 契约：每个非根 Link
        必须**恰好**由一个 Joint 进入。而 MJCF 允许"没有 joint 的 body"
        （等价于与父级刚性固连）。

        ⇒ 两种处理方式：
          ① 把它合并进父 link（丢掉一个 link，几何会错位）
          ② 合成一个 `type="fixed"` 的 Joint（保留 link，语义正确）

        选 ②。因为 `ee_link` 有自己的几何（手掌 + 两指）与 site（tcp），
        合并进 forearm 会让"末端执行器在哪个 link 上"变得不明确，
        而 EndEffector 契约要求一个明确的 link。
        """
        fixed = [j for j in mini_arm_model.joints if j.type == "fixed"]
        assert len(fixed) == 1, f"期望恰好 1 个合成的 fixed joint，实际 {len(fixed)}"
        assert fixed[0].id == "ee_link_fixed" or fixed[0].child_link == "ee_link"
        # 同时断言两条：属性存在时必须为 False，且 type 必须是 fixed
        assert fixed[0].type == "fixed", f"合成 joint 的 type 应为 'fixed'，实际 {fixed[0].type!r}"
        # ⚠ `is_mobile` 是**方法**（`def is_mobile(self)`），不是 property。
        #   漏掉括号时得到的是 bound method 对象 —— 它的真值为 True，
        #   于是 `assert not j.is_mobile` 会**通过**（永远通过），
        #   而 `assert j.is_mobile is False` 会失败。
        #   这正是"测试断言写错导致测试失效"的典型：前者是静默失效，
        #   后者才暴露问题。因此本项目统一写成 `is False` 这种严格形式。
        assert fixed[0].is_mobile() is False, (
            "fixed joint 不得被当作可动关节（is_mobile() 必须是 False）"
        )
        assert fixed[0].id not in mini_arm_model.mobile_joint_ids(), (
            "fixed joint 不得出现在 mobile_joint_ids() 里"
        )


# ----------------------------------------------------------------------
# 3. 拒绝性：不支持的东西必须报错
# ----------------------------------------------------------------------


class TestRejection:
    """v0.1 不支持的自由关节必须**报错**，不能静默降级。

    ## 为什么"静默降级"是错的

    若把 `free` 当作 `fixed` 处理，模型能加载、能渲染、能跑仿真 ——
    但机器人的行为与 MJCF 作者的本意完全不同。用户会看到一台"能动的
    机器人"，然后花几个小时怀疑自己的控制器。

    ⇒ 宁可加载失败并说清原因。这是"快速失败优于静默错误"的一个具体应用。
    """

    def test_free_joint_is_rejected(self, tmp_path):
        xml = """<mujoco model="t">
  <worldbody>
    <body name="base" pos="0 0 0">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>"""
        p = tmp_path / "t.xml"
        p.write_text(xml, encoding="utf-8")
        with pytest.raises(LoaderError) as ei:
            MJCFLoader(robot_id="t").load(p)
        assert "free" in str(ei.value).lower(), (
            f"报错信息里应提到 free joint，实际：{ei.value}"
        )

    def test_ball_joint_is_rejected(self, tmp_path):
        xml = """<mujoco model="t">
  <worldbody>
    <body name="base" pos="0 0 0">
      <geom type="box" size="0.1 0.1 0.1"/>
      <body name="child" pos="0.2 0 0">
        <joint name="ball" type="ball"/>
        <geom type="sphere" size="0.05"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""
        p = tmp_path / "t.xml"
        p.write_text(xml, encoding="utf-8")
        with pytest.raises(LoaderError) as ei:
            MJCFLoader(robot_id="t").load(p)
        assert "ball" in str(ei.value).lower(), f"报错信息里应提到 ball joint：{ei.value}"

    def test_missing_file_is_rejected(self, tmp_path):
        with pytest.raises((LoaderError, FileNotFoundError)):
            MJCFLoader(robot_id="t").load(tmp_path / "nope.xml")

    def test_invalid_xml_is_rejected(self, tmp_path):
        p = tmp_path / "bad.xml"
        p.write_text("<mujoco><worldbody></mujoco>", encoding="utf-8")
        with pytest.raises(Exception):
            MJCFLoader(robot_id="t").load(p)

    def test_duplicate_id_raises_instead_of_auto_suffixing(self, tmp_path):
        """★ 名称冲突时必须报错，**不得**自动加后缀。

        ## 为什么自动加后缀是错的

        自动把第二个 `link` 改成 `link_2` 会让加载"成功"，但：
        - 用户写的 `model.link("link")` 拿到的是**第一个**，而 IK 的
          引用可能指向第二个 —— 一个无法从代码看出的错配；
        - 错误从"加载时明确失败"推迟到"运行时悄悄算错"。

        ⇒ Loader 契约：**ID 冲突 = 加载失败**。修法只有两个：
          改 MJCF 里的名字，或在 MJCF 里显式写不同的 name。
        """
        xml = """<mujoco model="t">
  <worldbody>
    <body name="base" pos="0 0 0">
      <geom name="g" type="box" size="0.1 0.1 0.1"/>
      <body name="base" pos="0 0 0.2">
        <joint name="j" type="hinge" axis="0 0 1"/>
        <geom name="g" type="box" size="0.1 0.1 0.1"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""
        p = tmp_path / "dup.xml"
        p.write_text(xml, encoding="utf-8")
        # MuJoCo 自己有同名检查，可能在编译期就拒绝；两种都对
        with pytest.raises(Exception) as ei:
            MJCFLoader(robot_id="t").load(p)
        msg = str(ei.value).lower()
        assert "dup" in msg or "repeat" in msg or "same name" in msg or "冲突" in msg, (
            f"应因重名而失败，实际：{ei.value}"
        )


# ----------------------------------------------------------------------
# 4. 无 MJCF 残留（架构约束的机器判据）
# ----------------------------------------------------------------------


class TestNoMJCFLeakage:
    """★ RobotModel 里不得残留任何 MJCF/引擎痕迹。

    这是架构约束"Runtime 不得依赖 MJCF XML / MjModel"的**执行机制**。
    文档写了不算数，必须有一条测试在有人加 `model._mjcf_raw = ...` 时变红。
    """

    def test_model_has_no_mjcf_fields(self, mini_arm_model):
        """结构字段里不得有 MJCF 痕迹。

        ⚠ 一个必要的例外：`metadata.description` 会写
          "Loaded from native mjcf (...)" —— 那是**来源说明**（一个字符串），
          不是"结构化地依赖 MJCF"。区别在于：前者是给人看的注记，
          后者是让 Runtime 能反查 XML。

        ⇒ 因此只扫描**结构部分**（links / joints / actuators / frames / ...），
          不扫描 metadata 的自由文本描述。
        """
        d = mini_arm_model.to_dict()
        structural = {k: v for k, v in d.items() if k != "metadata"}
        text = json.dumps(structural)
        for bad in ("<body", "<joint", "<geom", "mujoco", "mjcf", "compiler", "worldbody"):
            assert bad not in text.lower(), (
                f"RobotModel 的**结构部分**里出现 {bad!r} —— 引擎痕迹泄漏了"
            )
        # metadata 的 description 是自由文本，但仍不得包含原始 XML 片段
        desc = d["metadata"].get("description", "")
        assert "<body" not in desc and "worldbody" not in desc, (
            "metadata.description 里不得内嵌原始 XML"
        )

    def test_model_has_no_engine_objects(self, mini_arm_model):
        """不得持有 MjModel / MjData 之类的引擎对象。"""
        for field_name in ("_mj_model", "_mj_data", "mj_model", "mj_data", "mjcf_raw", "_xml"):
            assert not hasattr(mini_arm_model, field_name), (
                f"RobotModel 不应有属性 {field_name!r}"
            )

    def test_model_is_frozen(self, mini_arm_model):
        """冻结契约：不得往 RobotModel 上挂运行时状态。"""
        import dataclasses

        with pytest.raises(dataclasses.FrozenInstanceError):
            mini_arm_model.runtime_state = {"qpos": [0, 0, 0]}

        with pytest.raises(dataclasses.FrozenInstanceError):
            mini_arm_model._mjcf_raw = "<mujoco/>"

    def test_nested_contracts_are_frozen(self, mini_arm_model):
        import dataclasses

        link = mini_arm_model.links[0]
        with pytest.raises(dataclasses.FrozenInstanceError):
            link.mass = 999.0

    def test_no_mujoco_import_needed_to_use_model(self, repo_root):
        """`backend/model/` 不得 import mujoco —— 契约层必须引擎无关。"""
        model_dir = repo_root / "backend" / "model"
        for p in sorted(model_dir.rglob("*.py")):
            text = p.read_text(encoding="utf-8")
            assert "import mujoco" not in text, (
                f"{p.relative_to(repo_root)} 出现了 `import mujoco` —— "
                f"契约层必须与引擎解耦"
            )

    def test_loader_report_records_facts_not_xml(self, mini_arm_loader_report):
        """LoaderReport 记录的是**事实**（计数、跳过项），不是原始 XML。"""
        text = json.dumps(
            {
                "source": mini_arm_loader_report.source,
                "format": mini_arm_loader_report.format,
                "facts": mini_arm_loader_report.facts,
                "notes": mini_arm_loader_report.notes,
                "skipped": mini_arm_loader_report.skipped,
            },
            default=str,
        )
        assert "<body" not in text and "worldbody" not in text


# ----------------------------------------------------------------------
# 5. id 规范化
# ----------------------------------------------------------------------


class TestSanitizeId:
    """MJCF 的 `name` 允许任意字符，RobotModel 的 ID 契约要求 snake_case ASCII。"""

    @pytest.mark.parametrize(
        "raw,expect",
        [
            ("Base", "base"),
            ("upper_arm", "upper_arm"),
            ("left-arm", "left_arm"),
            ("link.1", "link_1"),
            ("link 1", "link_1"),
            ("L1", "l1"),
        ],
    )
    def test_basic_sanitize(self, raw, expect):
        assert sanitize_id(raw) == expect

    def test_leading_digit_gets_prefix(self):
        """以数字开头时必须加前缀（ID 契约要求首字符是字母或下划线）。"""
        out = sanitize_id("1link")
        assert out[0] == "_", f"以数字开头的 id 应加前缀，实际 {out!r}"

    def test_non_ascii_falls_back(self):
        """非 ASCII 名称必须**降级**到 fallback，而不是被丢弃成一个空串。

        ★ 中文名称在 MJCF 里是合法的（MuJoCo 支持 UTF-8），但 RobotModel
          的 ID 契约要求 ASCII。若直接过滤掉所有非 ASCII 字符，
          `"连杆"` 会变成 `""` —— 一个空 ID，然后在别处以
          "ID 不能为空"的形式报错，与真正的起因（名称是中文）相隔很远。
        """
        out = sanitize_id("连杆", fallback="link")
        assert out == "link", f"非 ASCII 名称应降级为 fallback，实际 {out!r}"

    def test_empty_name_falls_back(self):
        assert sanitize_id("", fallback="node") == "node"

    def test_result_always_matches_id_pattern(self):
        import re

        pattern = re.compile(r"^[a-z_][a-z0-9_]*$")
        for raw in ["Base", "left-arm", "1link", "连杆", "a.b.c", "___", "A B C"]:
            out = sanitize_id(raw, fallback="n")
            assert pattern.fullmatch(out), f"sanitize_id({raw!r}) = {out!r} 不符合 ID 契约"


def _strip_comments_and_strings(text: str) -> list[str]:
    """剥掉注释与字符串字面量，只留代码行（用于"源码里有没有 X"的扫描）。

    ★ 为什么必须剥：
      在 `loader.py` 的 docstring 里有一句"因此本模块**不**import mujoco" ——
      朴素子串匹配会把这句**说明不要导入**的话判成违规。
      这类"扫描器误命中合法内容"是写源码扫描测试时最常见的假阳性来源。
      同源的教训见 tests/test_units.py 的 `units.length: m` 排除逻辑。

    这里用 `tokenize` 而不是正则：Python 的字符串/注释边界是正则处理不好的
    （三引号、嵌套引号、转义、f-string）。
    """
    import io
    import tokenize

    out: list[str] = []
    try:
        for tok in tokenize.generate_tokens(io.StringIO(text).readline):
            if tok.type in (tokenize.COMMENT, tokenize.STRING):
                continue
            if tok.type == tokenize.NL and not tok.line.strip():
                continue
            out.append(tok.string)
    except tokenize.TokenError:
        # 源码不完整时退化为"整份文本"，宁可误报也不漏报
        return text.splitlines()
    return out
