"""`backend/model/validator.py` 的契约测试。

## 这个文件为什么必须先做"反例注射"

`validator` 是全项目**唯一**一处"把错误提前到加载阶段"的关口。
它有三个典型的失效模式，而它们都表现为"测试全绿"：

```text
① 漏报  某条规则根本没实现       → 坏模型被放行，问题推迟到仿真里才爆
② 误报  规则太严                  → 合法模型被拒，"功能好好的却启动不了"
③ 虚绿  规则实现了但从未被触发    → 测试写了，却测的是一个恒真的条件
```

③ 最危险。所以本文件的写法是：**每一条规则，都用一个真实的坏模型去
触发它，并断言它报了预定的错误码**。断言的对象是 `code`，不是
"是否报错" —— 因为 `ok=False` 可能来自完全不相干的另一条规则。

## 规则码清单（实测，2026-09-15）

```text
树结构      tree.multiple_roots      多个根 / 多个 parent 为空的 link
            tree.cycle               关节树成环
            ref.link.parent_joint    link.parent_joint 指向不存在的关节
            ref.link.child_joint     link.child_joints 指向不存在的关节

标识        id.duplicate             同类型内 id 重复

关节        joint.type               不支持的关节类型（ball / free）
            joint.axis_not_normalized 轴不是单位向量
            joint.limits_inverted      position_min > position_max
            joint.no_position_limit    （warning）revolute 无限位

引用        ref.site.parent          site.parent 指向不存在的 link
            ref.end_effector.site    end_effector.site 指向不存在的 site

约定        coordinate.mismatch      坐标系声明不符合 RobotForge 约定
            units.mismatch           单位声明不全是 SI
```

## 与类型层（`backend/model/types.py`）的分工

**构造即拦截**的问题不归 validator 管 —— 它们连 `RobotModel` 都造不出来：

```text
Vector3(nan, 0, 0)      → ValueError: 必须是有限数；NaN/inf 会静默污染整条运动学链
Vector3(inf, 0, 0)      → 同上
```

因此本文件**不**测 NaN 传播：那是类型层的契约，已有 `test_coordinate.py`
覆盖。在这里重复测会给人"validator 负责 NaN"的错觉。
"""

from __future__ import annotations

import dataclasses

import pytest

from backend.model.robot_model import (
    CoordinateConvention,
    RobotModel,
    UnitConvention,
)
from backend.model.types import Quaternion, Transform, Vector3
from backend.model.validator import (
    RobotModelError,
    ValidationReport,
    assert_valid,
    summarize,
    validate_robot_model,
)

# ----------------------------------------------------------------------
# 小的注射工具
# ----------------------------------------------------------------------


def _replace_link(model: RobotModel, link_id: str, **kw) -> RobotModel:
    """返回一个'某个 link 被改掉'的模型副本（原模型不动）。"""
    return dataclasses.replace(
        model,
        links=[
            dataclasses.replace(l, **kw) if l.id == link_id else l
            for l in model.links
        ],
    )


def _replace_joint(model: RobotModel, joint_id: str, **kw) -> RobotModel:
    return dataclasses.replace(
        model,
        joints=[
            dataclasses.replace(j, **kw) if j.id == joint_id else j
            for j in model.joints
        ],
    )


def _codes(report: ValidationReport) -> set[str]:
    return {i.code for i in report.issues}


def _errors(report: ValidationReport) -> set[str]:
    return {i.code for i in report.errors}


def _warnings(report: ValidationReport) -> set[str]:
    return {i.code for i in report.warnings}


# ----------------------------------------------------------------------
# 基线：clean 模型必须干净通过
# ----------------------------------------------------------------------


class TestCleanModelPasses:
    """★ 最基础的"误报检查"：一个正确的模型必须**零问题**通过。

    ## 为什么这条要单独测

    一条过严的规则（比如"所有关节都必须有限位"）会让**当前正确**的
    `mini_arm` 被拒 —— 而表现形式是"服务起不来"，很容易被误诊成
    环境问题。所以先钉住"干净模型必须干净"。
    """

    def test_mini_arm_validates_clean(self, mini_arm_model):
        report = validate_robot_model(mini_arm_model)
        assert report.ok is True, (
            f"mini_arm 应当零问题通过，实际报了：\n{report.format()}"
        )
        assert report.errors == []
        assert report.warnings == []

    def test_report_carries_robot_id(self, mini_arm_model):
        report = validate_robot_model(mini_arm_model)
        assert report.robot_id == mini_arm_model.metadata.id

    def test_assert_valid_returns_the_model(self, mini_arm_model):
        """`assert_valid` 应支持链式调用：`assert_valid(model).summary_line()`。"""
        out = assert_valid(mini_arm_model)
        assert out is mini_arm_model

    def test_summarize_is_json_friendly(self, mini_arm_model):
        """`summarize` 的输出必须可 JSON 序列化（它要给 Web API 用）。"""
        import json

        s = summarize(validate_robot_model(mini_arm_model))
        json.dumps(s)  # 不抛异常即通过
        assert s["ok"] is True
        assert s["robot"] == mini_arm_model.metadata.id
        assert s["error_count"] == 0


# ----------------------------------------------------------------------
# 树结构
# ----------------------------------------------------------------------


class TestTreeRules:
    def test_multiple_roots_detected(self, mini_arm_model):
        """把 shoulder_link 的 parent_joint 置空 ⇒ 出现第二个根。"""
        broken = _replace_link(mini_arm_model, "shoulder_link", parent_joint=None)
        report = validate_robot_model(broken)
        assert "tree.multiple_roots" in _errors(report), (
            f"应报 tree.multiple_roots，实际：{_codes(report)}"
        )

    def test_cycle_detected(self, mini_arm_model):
        """把 base_yaw 的父 link 指到 forearm_link ⇒ 成环。

        ## 为什么用"改 parent 为下游 link"来造环

        这是**真实会发生的**错误：手写 MJCF 时把两段臂的父子顺序写反。
        此时从根出发遍历永远到不了 forearm_link 之后的部分，而一个只做
        "拓扑排序"的实现可能会静默截断，把后半条链丢掉 —— 表现为
        "机械臂少了半截"。
        """
        broken = _replace_joint(mini_arm_model, "base_yaw", parent_link="forearm_link")
        report = validate_robot_model(broken)
        assert "tree.cycle" in _errors(report), (
            f"应报 tree.cycle，实际：{_codes(report)}"
        )

    def test_dangling_parent_joint_detected(self, mini_arm_model):
        broken = _replace_link(mini_arm_model, "upper_arm", parent_joint="no_such_joint")
        report = validate_robot_model(broken)
        assert "ref.link.parent_joint" in _errors(report), (
            f"应报 ref.link.parent_joint，实际：{_codes(report)}"
        )

    def test_dangling_child_joint_detected(self, mini_arm_model):
        """★ `child_joints` 的反向悬空也必须查。

        `parent_joint` 与 `child_joints` 是同一关系的两个方向。只查一个
        方向是常见的疏漏 —— 而反向悬空会让"遍历子关节"静默少走一支。
        """
        broken = _replace_link(mini_arm_model, "upper_arm", child_joints=["nope"])
        report = validate_robot_model(broken)
        assert "ref.link.child_joint" in _errors(report), (
            f"应报 ref.link.child_joint，实际：{_codes(report)}"
        )

    def test_duplicate_link_id_detected(self, mini_arm_model):
        broken = dataclasses.replace(
            mini_arm_model, links=mini_arm_model.links + [mini_arm_model.link("base")]
        )
        report = validate_robot_model(broken)
        assert "id.duplicate" in _errors(report), (
            f"应报 id.duplicate，实际：{_codes(report)}"
        )


# ----------------------------------------------------------------------
# 关节规则
# ----------------------------------------------------------------------


class TestJointRules:
    def test_unsupported_joint_type_detected(self, mini_arm_model):
        """`ball` / `free` 关节必须被拒。

        ## 为什么这条重要

        RobotForge 的 `JOINT_TYPES` 只认 `fixed / revolute / prismatic`。
        MJCF 支持 `ball` 与 `free` —— 若把它们放进来，下游 FK 会用一个
        错误数量的自由度去算，而**不会报错**（只是位置不对）。
        """
        for bad_type in ("ball", "free"):
            broken = _replace_joint(mini_arm_model, "shoulder", type=bad_type)
            report = validate_robot_model(broken)
            assert "joint.type" in _errors(report), (
                f"type={bad_type} 应报 joint.type，实际：{_codes(report)}"
            )

    def test_non_unit_axis_detected(self, mini_arm_model):
        """轴必须是单位向量 —— 缩放过的轴会让转角与位移不成比例。"""
        broken = _replace_joint(
            mini_arm_model, "shoulder", axis=Vector3(0.0, 2.0, 0.0)
        )
        report = validate_robot_model(broken)
        assert "joint.axis_not_normalized" in _errors(report), (
            f"应报 joint.axis_not_normalized，实际：{_codes(report)}"
        )

    def test_inverted_limits_detected(self, mini_arm_model):
        lim = dataclasses.replace(
            mini_arm_model.joint("shoulder").limits,
            position_min=1.0,
            position_max=-1.0,
        )
        broken = _replace_joint(mini_arm_model, "shoulder", limits=lim)
        report = validate_robot_model(broken)
        assert "joint.limits_inverted" in _errors(report), (
            f"应报 joint.limits_inverted，实际：{_codes(report)}"
        )

    def test_missing_position_limit_is_warning_not_error(self, mini_arm_model):
        """★ 无限位是 **warning**，不是 error —— 这个分级是刻意的且有理由。

        ## 为什么不算错

        无限位在**运动学**上是良定义的（关节可以转任意角度），只是在
        **物理执行**上做不到。把它当 error 会让一批"仅为可视化/研究"
        的模型无法加载；把它当 warning 则既能在 UI 上提示，又不阻断。

        ⇒ 判据要落在"级别"上，而不只是"有没有报"。一条把 warning 误升级
          成 error 的改动会让这个模型无法加载，而"报了 issue"这一点
          完全不变 —— 只断言"报了"是测不出来的。
        """
        broken = _replace_joint(mini_arm_model, "shoulder", limits=None)
        report = validate_robot_model(broken)
        assert "joint.no_position_limit" in _warnings(report), (
            f"应作为 warning 报 joint.no_position_limit，实际 issues：{_codes(report)}"
        )
        assert "joint.no_position_limit" not in _errors(report), (
            "无限位不应升级为 error（那会让研究用模型无法加载）"
        )
        # 只有 warning ⇒ 整体仍应视为通过
        assert report.ok is True, (
            f"仅有 warning 时 ok 应为 True，实际 False：\n{report.format()}"
        )


# ----------------------------------------------------------------------
# 引用完整性
# ----------------------------------------------------------------------


class TestReferenceRules:
    def test_end_effector_bad_site_detected(self, mini_arm_model):
        ee = dataclasses.replace(mini_arm_model.end_effectors[0], site="no_such_site")
        broken = dataclasses.replace(mini_arm_model, end_effectors=[ee])
        report = validate_robot_model(broken)
        assert "ref.end_effector.site" in _errors(report), (
            f"应报 ref.end_effector.site，实际：{_codes(report)}"
        )

    def test_site_bad_parent_detected(self, mini_arm_model):
        """★ site 挂在**不存在的 link** 上 —— 这是几何与拓扑脱节。

        一个挂空的 site 在 FK 里无法求值。若只做"名字查得到即可"的浅
        检查，它会一路滑到渲染阶段才炸。
        """
        target_site = mini_arm_model.sites[0]
        broken_site = dataclasses.replace(target_site, parent="no_such_link")
        broken = dataclasses.replace(
            mini_arm_model,
            sites=[broken_site if s.id == target_site.id else s
                   for s in mini_arm_model.sites],
        )
        report = validate_robot_model(broken)
        assert "ref.site.parent" in _errors(report), (
            f"应报 ref.site.parent，实际：{_codes(report)}"
        )


# ----------------------------------------------------------------------
# 约定（坐标系 / 单位）
# ----------------------------------------------------------------------


class TestConventionRules:
    def test_coordinate_mismatch_detected(self, mini_arm_model):
        """坐标系声明不符 RobotForge 约定 ⇒ error。

        ## 为什么必须硬拦

        "+Y 朝前"和"+X 朝前"是**镜像**关系，不是旋转关系。混用会让
        所有位姿看起来"翻了个方向"，而且不会产生任何数值异常 ——
        是最难远程诊断的一类问题。所以必须在加载阶段拦住。
        """
        broken = dataclasses.replace(
            mini_arm_model,
            coordinate=CoordinateConvention(
                convention="robotforge",
                handedness="right",
                forward_axis="y",   # 应为 x
                left_axis="x",
                up_axis="z",
            ),
        )
        report = validate_robot_model(broken)
        assert "coordinate.mismatch" in _errors(report), (
            f"应报 coordinate.mismatch，实际：{_codes(report)}"
        )

    def test_left_handed_rejected(self, mini_arm_model):
        broken = dataclasses.replace(
            mini_arm_model,
            coordinate=CoordinateConvention(
                convention="robotforge",
                handedness="left",
                forward_axis="x",
                left_axis="y",
                up_axis="z",
            ),
        )
        report = validate_robot_model(broken)
        assert "coordinate.mismatch" in _errors(report)

    def test_units_mismatch_detected(self, mini_arm_model):
        """非 SI 单位 ⇒ error（mm / deg 都算）。

        ## 为什么 mm 也必须拦

        长度用 mm 是工程习惯，但它会让**所有**长度换成 mm 而角度仍是
        弧度 —— 混量纲的计算不会报错，只会给出一个 1000 倍小的结果。
        """
        broken = dataclasses.replace(
            mini_arm_model,
            units=UnitConvention(
                length="mm",   # 应为 m
                angle="rad",
                time="s",
                linear_velocity="m/s",
                angular_velocity="rad/s",
                force="N",
                torque="N*m",
            ),
        )
        report = validate_robot_model(broken)
        assert "units.mismatch" in _errors(report), (
            f"应报 units.mismatch，实际：{_codes(report)}"
        )


# ----------------------------------------------------------------------
# 报告对象本身
# ----------------------------------------------------------------------


class TestReportObject:
    def test_issues_carry_location(self, mini_arm_model):
        """★ 每条 issue 必须给出 `where` —— 否则使用者只能靠翻源码定位。

        报错信息里没位置，等价于要求使用者去读 validator 的实现才能
        知道哪里错了。实测格式是 `link[upper_arm]` / `joint[shoulder]`
        这类"类型[id]"形式，既机器可解析又人类可读。
        """
        broken = _replace_link(mini_arm_model, "upper_arm", parent_joint="nope")
        report = validate_robot_model(broken)
        for issue in report.issues:
            assert issue.where, f"issue {issue.code} 缺少 where"
            assert issue.message, f"issue {issue.code} 缺少 message"

    def test_format_is_human_readable(self, mini_arm_model):
        broken = _replace_link(mini_arm_model, "upper_arm", parent_joint="nope")
        text = validate_robot_model(broken).format()
        assert "upper_arm" in text, f"format() 应包含出错对象，实际：\n{text}"

    def test_raise_if_invalid_raises(self, mini_arm_model):
        broken = _replace_link(mini_arm_model, "upper_arm", parent_joint="nope")
        report = validate_robot_model(broken)
        with pytest.raises(RobotModelError):
            report.raise_if_invalid()

    def test_raise_if_invalid_noop_when_ok(self, mini_arm_model):
        report = validate_robot_model(mini_arm_model)
        assert report.raise_if_invalid() is report

    def test_assert_valid_raises_on_broken(self, mini_arm_model):
        broken = _replace_link(mini_arm_model, "upper_arm", parent_joint="nope")
        with pytest.raises(RobotModelError):
            assert_valid(broken)

    def test_error_and_warning_lists_are_disjoint(self, mini_arm_model):
        """★ `errors` 与 `warnings` 不得有交集。

        一条 issue 同时出现在两个列表里，会让任何基于计数的判断失真
        （比如 UI 显示"1 个错误、1 个警告"但实际只有一条）。
        """
        broken = _replace_joint(mini_arm_model, "shoulder", limits=None)
        report = validate_robot_model(broken)
        codes_e = {i.code for i in report.errors}
        codes_w = {i.code for i in report.warnings}
        assert not (codes_e & codes_w), f"同一码同时出现在两边：{codes_e & codes_w}"
        assert len(report.issues) == len(report.errors) + len(report.warnings)


# ----------------------------------------------------------------------
# 规则覆盖（元测试 + 完整反例表）
# ----------------------------------------------------------------------


def _all_counterexamples(model: RobotModel) -> dict[str, RobotModel]:
    """**每一条 error 规则**对应一个"精确坏掉的模型"。

    ## 为什么要把它们集中放在一个函数里

    这些注射用例有两个用途：

    1. 元测试 `test_every_error_rule_has_a_counterexample` 用它证明
       "**每条**规则都至少被触发过一次"—— 于是规则坏掉时必然有测试变红；
    2. 每个反例的**键**就是它应当触发的规则码 —— 元测试直接比对键集合，
       不需要猜某条规则在哪被触发。

    比"把反例散落在各个 Test*Rules 类里再猜覆盖"强的地方：反例表是
    **声明式**的，一眼能看出"哪条规则没有反例"，而不用去数 assert。

    ## 各反例的设计要点

    反过来读这张表也很有价值：它是 validator 契约的**负向规格说明**——
    一条规则上没有任何反例，就说明那条规则只是"写了但没生效"。
    """
    import dataclasses

    from backend.model.robot_model import (
        Actuator,
        EndEffector,
        Frame,
        GeometryRef,
        Inertial,
        Joint,
        JointLimits,
        MimicJoint,
        RobotCapabilities,
        RobotMetadata,
        Site,
    )

    out: dict[str, RobotModel] = {}

    def add(code: str, mm: RobotModel) -> None:
        assert code not in out, f"{code} 被注册了两次"
        out[code] = mm

    def link(link_id: str, **kw):
        return dataclasses.replace(model.link(link_id), **kw)

    def joint(joint_id: str, **kw):
        return dataclasses.replace(model.joint(joint_id), **kw)

    def with_links(*repl):
        by_id = {r.id: r for r in repl}
        return dataclasses.replace(
            model, links=[by_id.get(l.id, l) for l in model.links]
        )

    def with_joints(*repl):
        by_id = {r.id: r for r in repl}
        return dataclasses.replace(
            model, joints=[by_id.get(j.id, j) for j in model.joints]
        )

    # ---------------- metadata ----------------
    add(
        "metadata.empty",
        dataclasses.replace(
            model, metadata=RobotMetadata(id="ok_robot", name="", version="1")
        ),
    )
    add(
        "metadata.id_format",
        dataclasses.replace(
            model, metadata=RobotMetadata(id="Bad-Robot", name="n", version="1")
        ),
    )

    # ---------------- coordinate / units ----------------
    add(
        "coordinate.mismatch",
        dataclasses.replace(
            model,
            coordinate=CoordinateConvention(
                convention="robotforge",
                handedness="right",
                forward_axis="y",
                left_axis="x",
                up_axis="z",
            ),
        ),
    )
    add(
        "coordinate.axes_not_permutation",
        dataclasses.replace(
            model,
            coordinate=CoordinateConvention(
                convention="robotforge",
                handedness="right",
                forward_axis="x",
                left_axis="x",   # 与 forward 重复
                up_axis="z",
            ),
        ),
    )
    add(
        "units.mismatch",
        dataclasses.replace(
            model,
            units=UnitConvention(
                length="mm",
                angle="rad",
                time="s",
                linear_velocity="m/s",
                angular_velocity="rad/s",
                force="N",
                torque="N*m",
            ),
        ),
    )

    # ---------------- id ----------------
    add("id.duplicate", dataclasses.replace(model, links=model.links + [model.link("base")]))
    add(
        "id.format",
        dataclasses.replace(
            model,
            links=[
                dataclasses.replace(l, id="Bad_ID") if l.id == "base" else l
                for l in model.links
            ],
        ),
    )

    # ---------------- references ----------------
    add("ref.joint.parent_link", with_joints(joint("shoulder", parent_link="nope")))
    add("ref.joint.child_link", with_joints(joint("shoulder", child_link="nope")))
    add(
        "ref.joint.mimic",
        with_joints(joint("shoulder", mimic=MimicJoint(joint="nope", multiplier=1.0))),
    )
    add("ref.link.parent_joint", with_links(link("upper_arm", parent_joint="nope")))
    add("ref.link.child_joint", with_links(link("upper_arm", child_joints=["nope"])))
    add(
        "ref.actuator.joint",
        dataclasses.replace(
            model,
            actuators=[
                dataclasses.replace(model.actuators[0], joint="nope")
            ]
            + list(model.actuators[1:]),
        ),
    )
    add(
        "ref.frame.parent",
        dataclasses.replace(
            model,
            frames=[
                dataclasses.replace(f, parent="nope") if f.id == "tcp_frame" else f
                for f in model.frames
            ],
        ),
    )
    add(
        "ref.site.parent",
        dataclasses.replace(
            model,
            sites=[
                dataclasses.replace(s, parent="nope") if s.id == "tcp" else s
                for s in model.sites
            ],
        ),
    )
    add(
        "ref.end_effector.frame",
        dataclasses.replace(
            model,
            end_effectors=[
                dataclasses.replace(model.end_effectors[0], frame="nope")
            ],
        ),
    )
    add(
        "ref.end_effector.site",
        dataclasses.replace(
            model,
            end_effectors=[
                dataclasses.replace(model.end_effectors[0], site="nope")
            ],
        ),
    )

    # ---------------- tree ----------------
    add("tree.root_link_empty", dataclasses.replace(model, root_link=""))
    add("tree.root_link_missing", dataclasses.replace(model, root_link="nope"))
    add(
        "tree.multiple_roots",
        with_links(link("shoulder_link", parent_joint=None)),
    )
    add(
        "tree.root_link_not_root",
        # 声明 root_link="shoulder_link"，但它有 parent_joint（不是根）。
        # ⚠️ 不能改成 `replace(base, parent_joint="base_yaw")` —— 那会造出一个
        # **环**（base → base_yaw → base），报的是 tree.cycle 而不是本条。
        # 这正是"注射错了地方会得到一个假覆盖"的活例子：
        # test_every_counterexample_actually_fires 抓住了它。
        dataclasses.replace(model, root_link="shoulder_link"),
    )
    add(
        "tree.multi_parent",
        with_joints(joint("elbow", child_link="upper_arm")),
    )
    add(
        "tree.orphan_link",
        # 把 base_yaw 的 child 从 shoulder_link 改到 upper_arm：
        # shoulder_link 既非 root 也无 joint 指向它 ⇒ 孤立
        with_joints(joint("base_yaw", child_link="upper_arm")),
    )
    add("tree.base_frame_empty", dataclasses.replace(model, base_frame=""))
    add("tree.base_frame_missing", dataclasses.replace(model, base_frame="nope"))
    add(
        "tree.cycle",
        with_joints(joint("base_yaw", parent_link="forearm_link")),
    )

    # ---------------- joints ----------------
    add("joint.type", with_joints(joint("shoulder", type="ball")))
    add(
        "joint.axis_not_normalized",
        with_joints(joint("shoulder", axis=Vector3(0.0, 2.0, 0.0))),
    )
    # axis_zero：既要"模 < 1e-6"又不能触发 axis_not_normalized 的 continue
    # —— 看实现：两个 error 都会报（axis_not_normalized 在前，不中断）。
    add(
        "joint.axis_zero",
        with_joints(joint("shoulder", axis=Vector3(0.0, 0.0, 0.0))),
    )
    add(
        "joint.limits_inverted",
        with_joints(
            joint(
                "shoulder",
                limits=JointLimits(
                    position_min=1.0, position_max=-1.0,
                    velocity_max=1.0, effort_max=1.0,
                ),
            )
        ),
    )
    add(
        "joint.limits_velocity",
        with_joints(
            joint(
                "shoulder",
                limits=JointLimits(
                    position_min=-1.0, position_max=1.0,
                    velocity_max=0.0, effort_max=1.0,
                ),
            )
        ),
    )
    add(
        "joint.limits_effort",
        with_joints(
            joint(
                "shoulder",
                limits=JointLimits(
                    position_min=-1.0, position_max=1.0,
                    velocity_max=1.0, effort_max=0.0,
                ),
            )
        ),
    )

    # ---------------- actuators ----------------
    add(
        "actuator.type",
        dataclasses.replace(
            model,
            actuators=[dataclasses.replace(model.actuators[0], type="pneumatic")]
            + list(model.actuators[1:]),
        ),
    )
    add(
        "actuator.command_range_inverted",
        dataclasses.replace(
            model,
            actuators=[
                dataclasses.replace(
                    model.actuators[0], command_min=1.0, command_max=-1.0
                )
            ]
            + list(model.actuators[1:]),
        ),
    )

    # ---------------- frames ----------------
    add(
        "frame.cycle",
        dataclasses.replace(
            model,
            frames=[
                Frame(id="world", name="world", parent="base"),
                Frame(id="base", name="base", parent="world"),
            ]
            + [f for f in model.frames if f.id not in ("world", "base")],
        ),
    )

    # ---------------- end effectors ----------------
    add(
        "end_effector.frame_empty",
        dataclasses.replace(
            model,
            end_effectors=[
                dataclasses.replace(model.end_effectors[0], frame="", site=None)
            ],
        ),
    )

    # ---------------- transforms ----------------
    # 类型层（Vector3/Quaternion 的 __post_init__）已经挡住了 NaN 与非归一化，
    # 因此这两条规则只能被"绕过构造函数"的输入触发 —— 用 object.__setattr__
    # 直接改底层值，模拟 Loader 出错或从 JSON 反序列化时的漏检。
    nan_t = Transform.__new__(Transform)
    object.__setattr__(nan_t, "position", Vector3(0.0, 0.0, 0.0))
    object.__setattr__(nan_t, "orientation", Quaternion.identity())
    object.__setattr__(nan_t.position, "x", float("nan"))
    add(
        "transform.nan",
        with_joints(joint("elbow", origin=nan_t)),
    )

    bad_q = Quaternion.__new__(Quaternion)
    object.__setattr__(bad_q, "x", 0.0)
    object.__setattr__(bad_q, "y", 0.0)
    object.__setattr__(bad_q, "z", 0.0)
    object.__setattr__(bad_q, "w", 2.0)
    bad_qt = Transform.__new__(Transform)
    object.__setattr__(bad_qt, "position", Vector3.zero())
    object.__setattr__(bad_qt, "orientation", bad_q)
    add(
        "transform.quaternion_not_normalized",
        with_joints(joint("elbow", origin=bad_qt)),
    )

    # ---------------- geometry / inertial ----------------
    def with_geom(link_id: str, g: GeometryRef, index: int = 0):
        lk = model.link(link_id)
        old = list(lk.collision)
        old[index] = g
        return with_links(dataclasses.replace(lk, collision=old))

    base_g = model.link("base").collision[0]

    add(
        "geometry.type",
        with_geom("base", dataclasses.replace(base_g, type="torus")),
    )
    add(
        "geometry.size",
        with_geom("base", dataclasses.replace(base_g, type="sphere", size=(1.0, 1.0))),
    )
    add(
        "geometry.size_nonpositive",
        with_geom(
            "base",
            dataclasses.replace(base_g, size=tuple(-abs(v) for v in base_g.size)),
        ),
    )
    add(
        "geometry.mesh_no_asset",
        with_geom("base", GeometryRef(type="mesh", size=(), asset="")),
    )
    add(
        "geometry.rgba",
        with_geom("base", dataclasses.replace(base_g, rgba=(1.0, 0.0))),
    )
    add(
        "inertial.mass",
        with_links(
            dataclasses.replace(
                model.link("base"),
                inertial=dataclasses.replace(model.link("base").inertial, mass=-1.0),
            )
        ),
    )

    # ---------------- capabilities ----------------
    add(
        "capabilities.ee_mismatch",
        dataclasses.replace(
            model,
            end_effectors=[],
            capabilities=RobotCapabilities(
                simulation=True, fk=True, ik=True,
                actuator_control=True, end_effector=True,
            ),
        ),
    )
    add(
        "capabilities.actuator_mismatch",
        dataclasses.replace(
            model,
            actuators=[],
            capabilities=RobotCapabilities(
                simulation=True, fk=True, ik=True,
                actuator_control=True, end_effector=True,
            ),
        ),
    )

    return out


class TestRuleCoverage:
    """★ 元测试：validator 的**每一条** error 规则都必须有反例。

    ## 这条测试在防什么

    '反例注射'式的测试有一个隐蔽的失效模式：某条规则**没被任何测试
    触发**，于是它坏掉（被删、被 `if False` 掉、条件写反）也不会有人
    知道 —— 测试套件依然全绿。

    修法：把 validator 源码里的规则码全抽出来，与反例表的键集合做差集。
    差集非空即失败，并直接点名"哪条规则没有反例"。

    ## 实现方式

    用 `ast` 解析源码找 `report.error/warn(...)` 的字面量，比正则可靠
    （正则会把注释与文档字符串里的示例也算进来 —— 本项目吃过这个亏）。

    反向也查：反例表里的码必须都真实存在（否则是拼写错误，一个永远
    不会命中的"检查"）。
    """

    def test_every_error_rule_has_a_counterexample(self, mini_arm_model):
        declared = _declared_rules()
        cases = _all_counterexamples(mini_arm_model)

        errors = {c for c, lvl in declared.items() if lvl == "error"}
        missing = sorted(errors - set(cases))
        assert not missing, (
            "以下 error 规则没有任何反例 ⇒ 它们坏掉也不会被发现：\n"
            + "\n".join(f"  - {c}" for c in missing)
            + "\n请在 _all_counterexamples() 里补一个注射用例。"
        )

    def test_counterexample_table_has_no_typos(self, mini_arm_model):
        """★ 反向检查：反例表里的码必须都真实存在。

        一个拼错的码（`tree.cyle`）会让 `_all_counterexamples` 里那条用例
        "看起来有覆盖"，但实际永远不可能命中 —— 一个永不失败的检查。
        """
        declared = set(_declared_rules())
        cases = set(_all_counterexamples(mini_arm_model))
        unknown = sorted(cases - declared)
        assert not unknown, (
            "反例表里的这些码在 validator 里不存在（拼写错误？）：\n"
            + "\n".join(f"  - {c}" for c in unknown)
        )

    def test_every_counterexample_actually_fires(self, mini_arm_model):
        """★ 每个反例必须**真的触发**它声称的那条规则。

        ## 这条比"覆盖了"更强

        上一条只证明"有个用例注册在这个码下面"。而一个用例完全可能因为
        注射错了地方而触发**另一条**规则 —— 此时覆盖面是假的。

        所以这里逐条：跑一遍，断言目标码出现在 issues 里。
        """
        failures = []
        for code, broken in _all_counterexamples(mini_arm_model).items():
            report = validate_robot_model(broken)
            fired = _codes(report)
            if code not in fired:
                failures.append(
                    f"  - {code}: 注射后实际报的是 {sorted(fired) or '（无）'}"
                )
        assert not failures, (
            "以下反例没有触发它声称的规则 ⇒ 覆盖是假的：\n" + "\n".join(failures)
        )

    def test_all_warning_rules_have_counterexamples_too(self, mini_arm_model):
        """warning 规则同样要有反例 —— 它们也可能被静默删掉。

        与 error 的区别只在于严重度，不在于"是否值得测试"。一条被删掉的
        warning 会让 UI 少一个提示，而没有任何测试会红。
        """
        import dataclasses

        from backend.model.robot_model import JointLimits, MimicJoint

        declared = _declared_rules()
        warnings = {c for c, lvl in declared.items() if lvl == "warning"}
        assert warnings, "没有抽到任何 warning 规则 —— 抽取逻辑坏了"

        fired: set[str] = set()

        def fire(mm):
            fired.update(_codes(validate_robot_model(mm)))

        def with_joints(*repl):
            by_id = {r.id: r for r in repl}
            return dataclasses.replace(
                mini_arm_model,
                joints=[by_id.get(j.id, j) for j in mini_arm_model.joints],
            )

        def joint(joint_id, **kw):
            return dataclasses.replace(mini_arm_model.joint(joint_id), **kw)

        # joint.no_position_limit
        fire(with_joints(joint("shoulder", limits=None)))
        # joint.mimic_not_implemented
        fire(with_joints(joint("elbow", mimic=MimicJoint(joint="shoulder"))))
        # inertial.zero_mass
        fire(
            dataclasses.replace(
                mini_arm_model,
                links=[
                    dataclasses.replace(
                        l,
                        inertial=dataclasses.replace(l.inertial, mass=0.0),
                    )
                    if l.id == "base"
                    else l
                    for l in mini_arm_model.links
                ],
            )
        )
        # capabilities.ik_without_ee / capabilities.fk_no_dof
        from backend.model.robot_model import RobotCapabilities

        fire(
            dataclasses.replace(
                mini_arm_model,
                end_effectors=[],
                capabilities=RobotCapabilities(
                    simulation=True, fk=True, ik=True,
                    actuator_control=True, end_effector=False,
                ),
            )
        )
        fire(
            dataclasses.replace(
                mini_arm_model,
                joints=[
                    dataclasses.replace(j, type="fixed")
                    for j in mini_arm_model.joints
                ],
            )
        )

        missing = sorted(warnings - fired)
        assert not missing, (
            "以下 warning 规则没有反例：\n"
            + "\n".join(f"  - {c}" for c in missing)
        )


    def test_validator_never_raises_on_any_counterexample(self, mini_arm_model):
        """★★ validator **绝不能抛异常** —— 哪怕输入严重损坏。

        ## 这条测试是被一个真实 bug 逼出来的

        实测发现：把 `root_link` 改成一个不存在的名字，再让别的 link 变成
        根时，`_check_tree` 会走到

        ```python
        model.link(model.root_link).parent_joint   # KeyError!
        ```

        而不是报 `tree.root_link_not_root`。原因：上面 ① 已经报了
        `tree.root_link_missing`，但 ② 的 `if` 条件**没排除**这个情况，
        于是继续去 `model.link()` 一个不存在的 id。

        ## 为什么这类 bug 特别危险

        validator 是"错误 → 报告"的转换器。它自己抛异常时：

        - 调用方（Loader / Runtime）拿到的是 `KeyError: 没有 link 'nope'`，
          一个**与真实问题无关**的信息（真实问题是"root_link 名字写错"）；
        - 更糟：调用方往往会 `except Exception` 兜底，把模型加载失败报成
          "内部错误"，而真正的诊断信息（那 3 条 issue）从未生成。

        ⇒ 判据是"对**每一个**反例都不抛" —— 这比逐条断言某条规则报了什么
          更强，因为它覆盖了"多条规则同时被违反时的交互"。
        """
        import dataclasses

        broken_cases = _all_counterexamples(mini_arm_model)
        raised = []
        for code, broken in broken_cases.items():
            try:
                validate_robot_model(broken)
            except Exception as e:  # noqa: BLE001 — 就是要抓全部
                raised.append(f"  - {code}: {type(e).__name__}: {e}")
        assert not raised, (
            "validator 在以下损坏输入上抛了异常（它应该返回报告）：\n"
            + "\n".join(raised)
        )

        # 额外：一些"多条规则同时被违反"的复合损坏。
        composite = [
            dataclasses.replace(mini_arm_model, root_link="nope"),
            dataclasses.replace(mini_arm_model, root_link="", base_frame="nope"),
            dataclasses.replace(mini_arm_model, links=[], joints=[]),
            dataclasses.replace(
                mini_arm_model, links=[], joints=[], frames=[], sites=[],
                end_effectors=[], actuators=[],
            ),
        ]
        for i, mm in enumerate(composite):
            try:
                validate_robot_model(mm)
            except Exception as e:  # noqa: BLE001
                raise AssertionError(
                    f"复合损坏 #{i} 让 validator 抛了 {type(e).__name__}: {e}"
                ) from e

    def test_empty_model_reports_rather_than_crashes(self, mini_arm_model):
        """极端输入：一个**什么都是空的**模型。"""
        import dataclasses

        empty = dataclasses.replace(
            mini_arm_model,
            links=[],
            joints=[],
            frames=[],
            sites=[],
            end_effectors=[],
            actuators=[],
            root_link="",
            base_frame="",
        )
        report = validate_robot_model(empty)
        assert report.ok is False
        assert report.issues, "空模型必须报出问题，而不是给出空报告"
        assert report.format(), "format() 不能崩"


def _declared_rules() -> dict[str, str]:
    """从 validator 源码抽出全部规则码 → 级别。"""
    import ast
    import pathlib

    import backend.model.validator as V

    src = pathlib.Path(V.__file__).read_text(encoding="utf-8")
    tree = ast.parse(src)

    declared: dict[str, str] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not isinstance(func, ast.Attribute):
            continue
        if func.attr not in ("error", "warn"):
            continue
        if not node.args:
            continue
        arg = node.args[0]
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            declared[arg.value] = "error" if func.attr == "error" else "warning"

    assert declared, (
        "没能从 validator 源码里抽出任何规则码 —— 抽取逻辑坏了，"
        "所有基于它的元测试都已失去意义（这是最典型的'元测试自己先坏'）"
    )
    return declared
