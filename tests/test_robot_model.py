"""`RobotModel` 契约测试 —— **架构的核心**。

## RobotModel 是什么

它是唯一允许存在的内部机器人表示（Canonical Internal Representation）：

```text
        MJCF ─┐
        URDF ─┼─→ [Loader] ─→ RobotModel ─→ Runtime ─→ Backend
        (未来)─┘                  ↑
                          所有消费者只依赖它
```

**没有任何消费者可以直接看 MJCF / URDF / MjModel / Object3D。**
本文件就是这条约束的执行机制。

## 为什么用 `frozen=True` 而不是"约定不要改"

因为"约定"在 deadline 面前会失效。`frozen=True` 让
`model.runtime_state = ...` **物理上**抛 `FrozenInstanceError`。

最终用户拿到的控制权反而更大（前端可以放心共享同一个 model 对象，
因为没人能改它），但**违反契约要先让测试变红**这一层保护是硬的。
"""

from __future__ import annotations

import dataclasses
import json

import pytest

from backend.model.robot_model import (
    ACTUATOR_TYPES,
    GEOMETRY_TYPES,
    JOINT_TYPES,
    MOBILE_JOINT_TYPES,
    ROBOTFORGE_COORDINATE,
    ROBOTFORGE_UNITS,
    Link,
    RobotModel,
)

# ----------------------------------------------------------------------
# 1. 常量
# ----------------------------------------------------------------------


class TestModuleConstants:
    def test_joint_types_are_closed(self):
        """关节类型是**封闭集合**（三元素）。"""
        assert set(JOINT_TYPES) == {"fixed", "revolute", "prismatic"}

    def test_mobile_joint_types_is_subset(self):
        assert MOBILE_JOINT_TYPES <= set(JOINT_TYPES)
        assert "fixed" not in MOBILE_JOINT_TYPES, (
            "fixed 不是可动关节 —— 它不产生自由度"
        )

    def test_actuator_types_closed(self):
        assert "position" in ACTUATOR_TYPES
        for t in ACTUATOR_TYPES:
            assert isinstance(t, str)

    def test_coordinate_convention_matches_p0_contract(self):
        """模块常量必须与 P0 契约逐字段一致。

        ⚠ 这两个常量是**普通 dict**（"约定的规范形态"），不是 dataclass 实例。
          它们代表"RobotForge 平台认定的正确约定"，供 Loader 在把外部
          模型翻译成 RobotModel 时做比对；而 `CoordinateConvention`
          dataclass 是**模型自带**的那一份（用于校验模型是否声明了别的约定）。
        """
        assert isinstance(ROBOTFORGE_COORDINATE, dict)
        assert ROBOTFORGE_COORDINATE["handedness"] == "right"
        assert ROBOTFORGE_COORDINATE["forward_axis"] == "x"
        assert ROBOTFORGE_COORDINATE["left_axis"] == "y"
        assert ROBOTFORGE_COORDINATE["up_axis"] == "z"
        assert ROBOTFORGE_COORDINATE["convention"] == "robotforge"

    def test_units_convention_is_si(self):
        assert isinstance(ROBOTFORGE_UNITS, dict)
        assert ROBOTFORGE_UNITS["length"] == "m"
        assert ROBOTFORGE_UNITS["angle"] == "rad"
        assert ROBOTFORGE_UNITS["time"] == "s"

    def test_model_declares_p0_conventions(self, mini_arm_model):
        """模型**自身**声明的约定必须等于平台约定。

        这是"模型的 coordinate 段不能乱写"的判据：Loader 会把模型声明的
        约定与平台常量比对，不一致就拒绝（否则一个写着 Z-up 的模型
        会被当成 Y-up 静默使用）。
        """
        c = mini_arm_model.coordinate
        assert c.is_robotforge(), f"模型声明了非 RobotForge 坐标约定：{c}"
        u = mini_arm_model.units
        assert u.is_si(), f"模型声明了非 SI 单位：{u}"


# ----------------------------------------------------------------------
# 2. 查找 API
# ----------------------------------------------------------------------


class TestLookup:
    def test_link_lookup(self, mini_arm_model):
        link = mini_arm_model.link("shoulder_link")
        assert isinstance(link, Link)
        assert link.id == "shoulder_link"

    def test_joint_lookup(self, mini_arm_model):
        j = mini_arm_model.joint("shoulder")
        assert j.id == "shoulder"
        assert j.type == "revolute"

    def test_missing_link_raises_keyerror(self, mini_arm_model):
        """★ 查找不存在的 id 必须抛 `KeyError`，**不得**返回 `None`。

        ## 为什么返回 None 是错的

        返回 `None` 会把"拼错了 id"推迟到下一行才爆 —— 而且往往爆在一个
        与起因无关的地方（`AttributeError: 'NoneType' has no attribute 'axis'`）。
        更糟的是若调用方写了 `if link is not None:` 就会**静默跳过**一步。

        ⇒ 契约：找不到就立刻、明确地失败。
        """
        with pytest.raises(KeyError):
            mini_arm_model.link("no_such_link")
        with pytest.raises(KeyError):
            mini_arm_model.joint("no_such_joint")
        with pytest.raises(KeyError):
            mini_arm_model.frame("no_such_frame")
        with pytest.raises(KeyError):
            mini_arm_model.site("no_such_site")
        with pytest.raises(KeyError):
            mini_arm_model.actuator("no_such_actuator")

    def test_keyerror_message_is_helpful(self, mini_arm_model):
        """报错信息应列出可用的 id（否则调试要从头数起）。"""
        with pytest.raises(KeyError) as ei:
            mini_arm_model.joint("sholder")  # 拼错一个字母
        msg = str(ei.value)
        assert "sholder" in msg, "报错信息应包含查不到的那个 id"
        # 最好还能提示相近的候选
        assert "shoulder" in msg or "可用" in msg or "available" in msg.lower()

    def test_has_methods(self, mini_arm_model):
        assert mini_arm_model.has_link("base")
        assert not mini_arm_model.has_link("nope")
        assert mini_arm_model.has_joint("elbow")
        assert not mini_arm_model.has_joint("nope")

    def test_id_listing_methods(self, mini_arm_model):
        assert list(mini_arm_model.link_ids()) == [l.id for l in mini_arm_model.links]
        assert list(mini_arm_model.joint_ids()) == [j.id for j in mini_arm_model.joints]

    def test_mobile_joint_ids_excludes_fixed(self, mini_arm_model):
        """`mobile_joint_ids()` 必须排除 fixed joint，且保持声明顺序。"""
        mobile = list(mini_arm_model.mobile_joint_ids())
        assert mobile == ["base_yaw", "shoulder", "elbow"]
        assert "ee_link_fixed" not in mobile

    def test_dof_matches_mobile_joint_count(self, mini_arm_model):
        assert mini_arm_model.dof() == len(list(mini_arm_model.mobile_joint_ids()))

    def test_default_end_effector(self, mini_arm_model):
        ee = mini_arm_model.default_end_effector()
        assert ee is not None
        assert ee.site == "tcp"


# ----------------------------------------------------------------------
# 3. 冻结契约（架构约束的执行机制）
# ----------------------------------------------------------------------


class TestFrozenContract:
    """★ 本文件最重要的类。"""

    def test_robot_model_is_frozen(self, mini_arm_model):
        with pytest.raises(dataclasses.FrozenInstanceError):
            mini_arm_model.root_link = "other"

    def test_cannot_attach_runtime_state(self, mini_arm_model):
        """★ "RobotModel 不得含运行时状态"这一条必须由**运行时**强制。

        `model.qpos = [...]` / `model.current_pose = ...` 这类字段是
        "把运行时状态塞进模型"的典型形式。它们会：
        - 让同一个 RobotModel 不能被多个 Runtime 并发使用；
        - 让"模型"与"某一时刻的状态"概念混淆，UI 刷新时读到过期值。
        """
        for attr in ("qpos", "qvel", "current_pose", "runtime_state", "state"):
            with pytest.raises(dataclasses.FrozenInstanceError):
                setattr(mini_arm_model, attr, [0.0, 0.0, 0.0])

    def test_all_nested_contracts_are_frozen(self, mini_arm_model):
        """**所有**嵌套契约对象都必须冻结（不能只有顶层冻结）。"""
        frozen_types = set()
        for link in mini_arm_model.links:
            frozen_types.add(type(link))
        for j in mini_arm_model.joints:
            frozen_types.add(type(j))
        for a in mini_arm_model.actuators:
            frozen_types.add(type(a))

        for t in frozen_types:
            assert dataclasses.is_dataclass(t), f"{t.__name__} 不是 dataclass"
            assert t.__dataclass_params__.frozen, (
                f"{t.__name__} 不是 frozen —— 顶层冻结但嵌套可变等于没冻结"
            )

    def test_cannot_mutate_nested_via_attribute(self, mini_arm_model):
        with pytest.raises(dataclasses.FrozenInstanceError):
            mini_arm_model.links[0].id = "hacked"

    def test_lists_attempt_mutation_does_not_silently_corrupt(self, mini_arm_model):
        """列表字段（如 `links`）是可变 list —— 这是**已知的**设计取舍。

        为什么要专门写一条测试把它记录下来：

        ```text
        frozen=True 只阻止"重新绑定属性"（obj.links = [...]），
        不阻止"就地修改列表"（obj.links.append(...)）。
        ```

        ⇒ 契约的执行是**两层**的：
          ① frozen=True 挡住属性重绑定（测试在上面几条）
          ② validator 在加载时校验结构合法性（test_robot_model 的校验组）

        本测试把"列表是活的"这个事实**显式记录**下来，避免有人以为
        `frozen=True` 就等于深不可变，从而放松 ②。
        """
        original_len = len(mini_arm_model.links)
        try:
            mini_arm_model.links.append("junk")  # type: ignore[arg-type]
            assert len(mini_arm_model.links) == original_len + 1, (
                "如果这里失败，说明列表已被改成了 tuple —— "
                "那是另一种（更严格的）设计，请同步更新本条测试"
            )
        finally:
            mini_arm_model.links.pop()

    def test_hashability_reflects_frozen_intent(self, mini_arm_model):
        """冻结对象应尽量可哈希（但含 list 字段的顶层对象不可哈希）。

        这条件测试记录的是**真实状态**，不是理想状态：
        `RobotModel.links` 是 list ⇒ dataclass 的 `__hash__` 被置为 None。
        记录它，避免有人在别处 `set([model])` 然后困惑。
        """
        # 单个 Vector3 / Quaternion 是可哈希的（纯 float 字段）
        from backend.model.types import Vector3

        assert isinstance(hash(Vector3(1, 2, 3)), int)
        # RobotModel 本身不可哈希（因为含 list）—— 记录这个事实
        with pytest.raises(TypeError):
            hash(mini_arm_model)


# ----------------------------------------------------------------------
# 4. 序列化契约
# ----------------------------------------------------------------------


class TestSerialization:
    def test_to_dict_is_json_serializable(self, mini_arm_model):
        """`to_dict()` 必须能直接 `json.dumps`（供 WebSocket 用）。

        ★ 这条测试是"契约对象里不存 numpy"这条规则的**执行机制**：
          存 `np.float32` 时 `json.dumps` 会抛 TypeError。
        """
        d = mini_arm_model.to_dict()
        text = json.dumps(d)  # 不抛就说明没有 numpy 标量
        assert len(text) > 100

    def test_to_dict_has_expected_top_level_keys(self, mini_arm_model):
        d = mini_arm_model.to_dict()
        for key in (
            "metadata", "coordinate", "units", "root_link", "base_frame",
            "links", "joints", "actuators", "frames", "sites",
            "end_effectors", "capabilities",
        ):
            assert key in d, f"to_dict() 缺少顶层字段 {key!r}"

    def test_round_trip_preserves_counts(self, mini_arm_model):
        """`to_dict()` 的计数必须与对象本身一致。"""
        d = mini_arm_model.to_dict()
        assert len(d["links"]) == len(mini_arm_model.links)
        assert len(d["joints"]) == len(mini_arm_model.joints)
        assert len(d["actuators"]) == len(mini_arm_model.actuators)
        assert len(d["frames"]) == len(mini_arm_model.frames)

    def test_all_numbers_are_python_floats(self, mini_arm_model):
        """递归确认所有数值都是纯 Python `float`/`int`（不是 numpy 类型）。"""
        def walk(node, path=""):
            if isinstance(node, dict):
                for k, v in node.items():
                    walk(v, f"{path}.{k}")
            elif isinstance(node, list):
                for i, v in enumerate(node):
                    walk(v, f"{path}[{i}]")
            elif isinstance(node, float):
                assert type(node) is float, (
                    f"{path} 的类型是 {type(node)}，不是纯 float"
                )
            elif isinstance(node, bool):
                pass
            elif isinstance(node, int):
                assert type(node) is int, f"{path} 的类型是 {type(node)}，不是纯 int"

        walk(mini_arm_model.to_dict())

    def test_summary_line_is_human_readable(self, mini_arm_model):
        line = mini_arm_model.summary_line()
        assert isinstance(line, str) and line
        assert "mini_arm" in line


# ----------------------------------------------------------------------
# 5. Link-Joint Tree 契约
# ----------------------------------------------------------------------


class TestLinkJointTree:
    def test_every_non_root_link_entered_by_exactly_one_joint(self, mini_arm_model):
        """★ Link-Joint alternation：每个非根 link **恰好**被一个 joint 进入。

        这条契约是"树"的定义性质之一。它保证了：
        - FK 可以沿唯一的父路径回溯（无需处理"多个父"的歧义）；
        - `link.parent_joint` 是**单值**的。

        `ee_link` 就是靠"Loader 为无 joint 的 body 合成 fixed joint"
        来满足这条契约的。
        """
        for link in mini_arm_model.links:
            entering = [j for j in mini_arm_model.joints if j.child_link == link.id]
            # ⚠ `is_root` 是**方法**（`def is_root(self)`）。漏括号会得到
            #   bound method（真值恒为 True）⇒ 所有非根 link 都走进
            #   "根"分支 ⇒ 报出"shoulder_link 不应被 joint 进入"这种反向错误。
            if link.is_root():
                assert not entering, (
                    f"根 link {link.id!r} 不应被任何 joint 进入，实际有 {len(entering)} 个"
                )
            else:
                assert len(entering) == 1, (
                    f"link {link.id!r} 应恰好被 1 个 joint 进入，实际 {len(entering)} 个"
                )

    def test_no_orphan_links(self, mini_arm_model):
        """除根之外，每个 link 都必须能从根出发到达（无孤儿）。"""
        reachable = {mini_arm_model.root_link}
        changed = True
        while changed:
            changed = False
            for j in mini_arm_model.joints:
                if j.parent_link in reachable and j.child_link not in reachable:
                    reachable.add(j.child_link)
                    changed = True
        orphans = [l.id for l in mini_arm_model.links if l.id not in reachable]
        assert not orphans, f"存在孤立 link（从根不可达）：{orphans}"

    def test_no_cycles(self, mini_arm_model):
        """从每个 link 沿 parent 上溯，必须在有限步内到达根。"""
        for link in mini_arm_model.links:
            seen = set()
            cur = link.id
            steps = 0
            while cur != mini_arm_model.root_link:
                assert cur not in seen, f"link {link.id!r} 的父链上出现环"
                seen.add(cur)
                entering = [j for j in mini_arm_model.joints if j.child_link == cur]
                if not entering:
                    break
                cur = entering[0].parent_link
                steps += 1
                assert steps < len(mini_arm_model.links) + 1, "父链过长 ⇒ 疑似环"

    def test_no_multi_parent(self, mini_arm_model):
        """一个 link 不能有多个父（由"恰好一个进入 joint"隐含，显式再断言一次）。"""
        child_to_joints = {}
        for j in mini_arm_model.joints:
            child_to_joints.setdefault(j.child_link, []).append(j.id)
        for child, js in child_to_joints.items():
            assert len(js) == 1, f"link {child!r} 有多个父 joint：{js}"

    def test_parent_joint_and_child_joints_are_consistent(self, mini_arm_model):
        """`link.parent_joint` / `link.child_joints` 必须与 joints 表互洽。

        ★ 这是"两份描述同一件事的数据必须一致"的检查。不一致会出现
          "从 parent 方向遍历"与"从 child 方向遍历"得到不同树的情形。
        """
        for link in mini_arm_model.links:
            # child_joints
            expect_children = sorted(j.id for j in mini_arm_model.joints if j.parent_link == link.id)
            assert sorted(link.child_joints) == expect_children, (
                f"link {link.id!r} 的 child_joints={sorted(link.child_joints)} "
                f"≠ 由 joints 表推出的 {expect_children}"
            )
            # parent_joint
            entering = [j.id for j in mini_arm_model.joints if j.child_link == link.id]
            if link.is_root():
                assert link.parent_joint is None
            else:
                assert link.parent_joint == entering[0], (
                    f"link {link.id!r} 的 parent_joint={link.parent_joint!r} "
                    f"≠ {entering[0]!r}"
                )

    def test_root_link_is_declared(self, mini_arm_model):
        assert mini_arm_model.root_link == "base"
        assert mini_arm_model.has_link(mini_arm_model.root_link)
        assert mini_arm_model.link(mini_arm_model.root_link).is_root()

    def test_joint_origin_is_explicit(self, mini_arm_model):
        """★ 每个 joint 都必须有**显式** origin（即使是 fixed）。

        ## 为什么不允许"省略 origin 表示原点"

        因为省略会带来两种读法："就是原点" vs "忘了写"。
        而这两种读法在**大部分模型上表现得一样**（origin 恰好是原点），
        只在某些模型上产生一个"连杆位置整体偏了"的现象。
        ⇒ 显式 `Transform.identity()` 让意图可读。
        """
        for j in mini_arm_model.joints:
            assert j.origin is not None, f"joint {j.id!r} 缺少 origin"
            assert j.origin.position is not None
            assert j.origin.orientation is not None

    def test_mobile_joint_axis_is_unit_length(self, mini_arm_model):
        """可动关节的轴必须是单位向量。

        ★ 非单位轴不会报错，只会让"关节转 1 rad"变成"转 2.5 rad" ——
          一个看起来像"控制增益不对"的错误。
        """
        for j in mini_arm_model.joints:
            if not j.is_mobile():
                continue
            assert j.axis.is_normalized(tol=1e-9), (
                f"关节 {j.id!r} 的轴 {j.axis} 不是单位向量（模长 {j.axis.norm()}）"
            )

    def test_fixed_joint_axis_is_still_declared(self, mini_arm_model):
        """★ fixed joint 也必须有轴（契约要求 axis 永远存在）。

        理由：契约的一致性。若允许 fixed 的 axis 为 None，那么每个读
        `joint.axis` 的地方都要先判类型 —— 而"某些字段有时为 None"
        正是契约不清晰的标志。给 fixed 一个占位轴（惯例是 Z），
        读的人永远不需要判空。
        """
        for j in mini_arm_model.joints:
            assert j.axis is not None, f"joint {j.id!r} 缺少 axis"


# ----------------------------------------------------------------------
# 6. 能力驱动（供前端用）
# ----------------------------------------------------------------------


class TestCapabilities:
    def test_capabilities_present(self, mini_arm_model):
        caps = mini_arm_model.capabilities
        assert caps is not None
        assert caps.simulation is True
        assert caps.fk is True
        assert caps.ik is True

    def test_capabilities_serialize(self, mini_arm_model):
        d = mini_arm_model.to_dict()["capabilities"]
        assert d["ik"] is True
        assert isinstance(d["simulation"], bool)

    def test_frontend_has_no_hardcoded_robot_id(self, repo_root):
        """★ 前端不得出现 `if (robot.id === 'mini_arm')` 之类的硬编码分支。

        （架构约束：Core 与 Frontend 都不得含 robot-specific branch。
          v0.1 前端尚未实现时此测试自动通过，但一旦写了前端就会生效。
          `frontend/` 不存在时跳过。）
        """
        fe = repo_root / "frontend"
        if not fe.is_dir():
            pytest.skip("frontend/ 尚未创建（Phase 2）")
        import re

        pattern = re.compile(r"""(id|name|robotId)\s*===?\s*['"](mini_arm|mearm)['"]""")
        offenders = []
        for p in sorted(fe.rglob("*")):
            if not p.is_file() or p.suffix not in (".ts", ".tsx", ".js", ".jsx", ".vue"):
                continue
            if "node_modules" in p.parts:
                continue
            for lineno, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
                if pattern.search(line):
                    offenders.append(f"  {p.relative_to(repo_root)}:{lineno}  {line.strip()}")
        assert not offenders, (
            "前端出现按 robot id 硬编码的分支：\n" + "\n".join(offenders)
            + "\n\n应改为按 capability 决定（model.capabilities.ik 等）。"
        )

    def test_core_has_no_robot_specific_branches(self, repo_root):
        """★ backend/ 里不得出现按 robot id 的分支。

        这是架构约束"Core 不得包含 robot-specific branch"的机器判据。
        """
        import re

        import io
        import tokenize

        pattern = re.compile(
            r"""(robot_id|robotId|robot\.id|model\.id)\s*===?\s*['"](mini_arm|mearm)['"]"""
        )
        offenders = []
        for p in sorted((repo_root / "backend").rglob("*.py")):
            raw = p.read_text(encoding="utf-8")
            # ★ 必须剥掉注释与字符串：`robot_model.py` 的 docstring 里
            #   恰好有一个 **❌ 反例** 示例 `{robot.id === 'mini_arm' && ...}`，
            #   朴素的行扫描会把这个"教学用的反面例子"判成违规。
            #   写源码扫描测试时，"我的模式会不会命中合法内容"必须先想清楚。
            try:
                stripped = tokenize.untokenize(
                    tok for tok in tokenize.generate_tokens(io.StringIO(raw).readline)
                    if tok.type not in (tokenize.COMMENT, tokenize.STRING)
                )
            except tokenize.TokenError:
                stripped = raw
            for lineno, line in enumerate(stripped.splitlines(), 1):
                if pattern.search(line):
                    offenders.append(f"  {p.relative_to(repo_root)}:{lineno}  {line.strip()}")
        assert not offenders, (
            "backend/ 出现按 robot id 硬编码的分支：\n" + "\n".join(offenders)
            + "\n\n应改为按 capability / 结构特征判断。"
        )
