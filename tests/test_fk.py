"""通用 FK / IK 接口契约测试。

## 与 `packages/mini_arm/tests/` 的分工

```text
packages/mini_arm/tests/test_kinematics_regression.py
    → mini_arm **这个型号**的 FK/IK 数值正确性（三路交叉验证、往返精度）

tests/test_fk.py + tests/test_ik.py（本文件）
    → **接口契约**：Core 的通用引擎长什么样、报什么错、返回什么形状
```

即：本文件关心"接口对不对"，包内测试关心"这个型号算得准不准"。

## v0.1 的现状（Phase 3 已落地）

Core 的通用 FK 引擎在 Phase 3 搬进了 `backend/kinematics/fk.py`。
在那之前，本文件用 `packages/mini_arm/kinematics/fk.py` 的
`forward_kinematics` 作为**原型**来验证接口设计 —— 它的算法
本来就与任何 Tree 型 RobotModel 无关，正是 Core 引擎的雏形。

⇒ 迁移后本文件直接 `import backend.kinematics.fk`，不再用按路径加载的
   临时手段。这也是"原型 → 正式"这条路走完的标志：引擎现在住在 Core，
   测试也按 Core 的地址去访问它。

★ 注意 `TestEngineGenerality` 里的两条测试现在**真的**在扫描 Core 源码
  （`generic_fk.__file__` 指向 `backend/kinematics/fk.py`）。迁移之前
  它扫的是包内文件 —— 也就是说，那时"Core 里没有机器人名"这个断言
  其实没有被验证过。迁移顺手补上了这个盲区。
"""

from __future__ import annotations

import math

import pytest

from backend.model.robot_model import RobotModel
from backend.model.types import Quaternion, Transform, Vector3

import backend.kinematics.fk as generic_fk


# ----------------------------------------------------------------------
# FK 接口契约
# ----------------------------------------------------------------------


class TestFKInterface:
    """通用 FK 引擎的接口形状。"""

    def test_accepts_robot_model_and_joint_dict(self, mini_arm_model):
        """入口签名：`(RobotModel, dict[str, float]) -> Transform`。"""
        out = generic_fk.forward_kinematics(mini_arm_model, {"shoulder": 0.1})
        assert isinstance(out, Transform)
        assert isinstance(out.position, Vector3)
        assert isinstance(out.orientation, Quaternion)

    def test_missing_joints_default_to_zero(self, mini_arm_model):
        """**未提供的关节按 0 处理**（契约，不是"报错"）。

        ## 为什么默认 0 而不是报错

        0 是"零位"这个有明确物理含义的值。而"缺一个关节就报错"会让
        UI 在初始化阶段（还没从后端收到完整状态）无法渲染任何东西 ——
        被迫先造一份全零的假状态，那反而更容易出错。
        """
        full = generic_fk.forward_kinematics(
            mini_arm_model, {"base_yaw": 0.0, "shoulder": 0.0, "elbow": 0.0}
        )
        partial = generic_fk.forward_kinematics(mini_arm_model, {})
        assert partial.position.approx_eq(full.position, tol=1e-15)
        assert partial.orientation.approx_eq(full.orientation)

    def test_extra_joint_ids_are_ignored(self, mini_arm_model):
        """字典里多出的键不影响结果（不报错）。

        理由：调用方可能持有一个"全平台通用"的关节字典（含夹爪等本型号
        没有的项）。对多余键报错会迫使每个调用方都先做过滤。
        """
        a = generic_fk.forward_kinematics(mini_arm_model, {"shoulder": 0.2})
        b = generic_fk.forward_kinematics(
            mini_arm_model, {"shoulder": 0.2, "no_such_joint": 9.9}
        )
        assert a.position.approx_eq(b.position, tol=1e-15)

    def test_deterministic(self, mini_arm_model):
        """同样输入必须给同样输出（无隐藏状态）。"""
        q = {"base_yaw": 0.3, "shoulder": -0.4, "elbow": 0.9}
        r1 = generic_fk.forward_kinematics(mini_arm_model, q)
        r2 = generic_fk.forward_kinematics(mini_arm_model, q)
        assert r1.position.approx_eq(r2.position, tol=0.0)
        assert r1.orientation.approx_eq(r2.orientation)

    def test_does_not_mutate_input(self, mini_arm_model):
        """不得修改传入的字典（否则调用方共享的 state 会被污染）。"""
        q = {"base_yaw": 0.1, "shoulder": 0.2, "elbow": 0.3}
        snapshot = dict(q)
        generic_fk.forward_kinematics(mini_arm_model, q)
        assert q == snapshot

    def test_does_not_mutate_model(self, mini_arm_model):
        """不得修改 model（契约：RobotModel 不可变）。"""
        before = mini_arm_model.summary_line()
        generic_fk.forward_kinematics(
            mini_arm_model, {"shoulder": 0.5, "elbow": 0.5}
        )
        assert mini_arm_model.summary_line() == before

    def test_output_is_finite(self, mini_arm_model):
        """输出必须全为有限值（NaN 会造成下游静默错误）。"""
        for q in [
            {"shoulder": 0.0},
            {"shoulder": math.pi / 2, "elbow": 3 * math.pi / 4},
            {"base_yaw": math.pi, "shoulder": -math.pi / 2, "elbow": -3 * math.pi / 4},
        ]:
            t = generic_fk.forward_kinematics(mini_arm_model, q)
            assert t.position.is_finite(), f"q={q} 产生了非有限位置 {t.position}"
            assert t.orientation.is_normalized(tol=1e-9), (
                f"q={q} 产生了非归一化四元数 {t.orientation}"
            )

    def test_returns_tcp_not_flange(self, mini_arm_model):
        """★ 输出必须是 **TCP**，不是法兰（ee_link 原点）。

        ## 为什么这条必须显式测

        二者相差 L_TOOL = 32 mm。若 FK 返回法兰而 IK 以 TCP 为目标，
        整个链条会有一个恒定的 32 mm 偏差 —— 看起来像标定问题。
        """
        t = generic_fk.forward_kinematics(
            mini_arm_model, {"base_yaw": 0.0, "shoulder": 0.0, "elbow": 0.0}
        )
        # 零位时 TCP 应在 x = L1 + L2 + L_TOOL = 0.200
        assert abs(t.position.x - 0.200) < 1e-9, (
            f"零位 TCP x = {t.position.x}，期望 0.200（L1+L2+L_TOOL）。"
            f"若为 0.168 说明返回的是法兰而不是 TCP。"
        )

    def test_link_transforms_covers_all_links(self, mini_arm_model):
        """`link_transforms` 必须覆盖**全部** link（供前端逐段渲染）。"""
        poses = generic_fk.link_transforms(mini_arm_model, {})
        assert set(poses.keys()) == {l.id for l in mini_arm_model.links}, (
            f"缺少 {set(l.id for l in mini_arm_model.links) - set(poses.keys())}"
        )

    def test_link_transforms_agrees_with_single_pose(self, mini_arm_model):
        """逐 link 与整体 FK 的结果必须一致（同一条链，无第二份实现）。"""
        q = {"base_yaw": 0.4, "shoulder": -0.7, "elbow": 1.1}
        poses = generic_fk.link_transforms(mini_arm_model, q)
        ee = mini_arm_model.default_end_effector()
        ee_link = mini_arm_model.site(ee.site).parent
        tcp = poses[ee_link].compose(mini_arm_model.site(ee.site).transform)
        direct = generic_fk.forward_kinematics(mini_arm_model, q)
        assert tcp.position.approx_eq(direct.position, tol=1e-15)


# ----------------------------------------------------------------------
# IK 接口契约
# ----------------------------------------------------------------------


@pytest.fixture(scope="session")
def ik_mod():
    """加载 mini_arm 的 IK 实现 —— **经 manifest 声明**，不硬编码路径。

    迁移前这里是 `root / "packages" / "mini_arm" / "kinematics" / "ik.py"`。
    那条硬编码与 `cli.py` 的加载逻辑是**第二份实现**：manifest 改了 entry，
    测试还会去加载旧路径（或反过来，测试通过了但 CLI 加载失败）。
    现在两者共用 `backend.cli._load_package_module()` —— 同一个入口，
    于是"manifest 的 entry 指向的文件真的能加载"这件事被测试覆盖了。

    IK 留在包内是**设计决定**而非技术债（见 mini_arm/manifest.yaml 里
    `ik.type` 的注释）：解析解能精确验证往返一致性。所以这里加载的是
    包内模块，而不是 Core —— 与上面的 `generic_fk` 恰好相反。
    """
    from backend.api.registry import get_package
    from backend.cli import _kinematics_entry, _load_package_module

    pkg = get_package("mini_arm")
    entry = _kinematics_entry(pkg, "ik")
    assert entry is not None, "mini_arm/manifest.yaml 必须声明 kinematics.ik.entry"
    return _load_package_module(pkg, entry, "ik")


class TestIKInterface:
    def test_solve_returns_solution_object(self, mini_arm_model, ik_mod):
        """`solve()` 返回 `IkSolution`，不是裸 dict。

        ## 为什么返回对象而不是 `dict[str, float]`

        因为"这是哪个分支"、"有没有触限"、"误差多大"都是调用方需要的
        **决策依据**。返回裸 dict 会迫使调用方自己重算 —— 而重算就是
        第二份实现的开始，它会与第一份漂移。
        """
        sol = ik_mod.solve(
            mini_arm_model, Transform(Vector3(0.12, 0.0, 0.15), Quaternion.identity())
        )
        assert hasattr(sol, "joint_positions")
        assert hasattr(sol, "branch")
        assert hasattr(sol, "position_error")
        assert hasattr(sol, "clamped")
        assert isinstance(sol.joint_positions, dict)
        assert sol.to_dict()["branch"] == sol.branch

    def test_solution_covers_all_mobile_joints(self, mini_arm_model, ik_mod):
        """解必须给出**所有**可动关节的值（不能只给部分）。"""
        sol = ik_mod.solve(
            mini_arm_model, Transform(Vector3(0.12, 0.0, 0.15), Quaternion.identity())
        )
        assert set(sol.joint_positions.keys()) == set(mini_arm_model.mobile_joint_ids()), (
            f"解只覆盖了 {set(sol.joint_positions.keys())}，"
            f"期望 {set(mini_arm_model.mobile_joint_ids())}"
        )

    def test_solve_all_returns_both_branches(self, mini_arm_model, ik_mod):
        sols = ik_mod.solve_all(
            mini_arm_model, Transform(Vector3(0.12, 0.0, 0.15), Quaternion.identity())
        )
        assert len(sols) == 2
        assert {s.branch for s in sols} == {"elbow_up", "elbow_down"}

    def test_solve_all_sorted_by_error(self, mini_arm_model, ik_mod):
        sols = ik_mod.solve_all(
            mini_arm_model, Transform(Vector3(0.12, 0.0, 0.15), Quaternion.identity())
        )
        errs = [s.position_error for s in sols]
        assert errs == sorted(errs)

    def test_unreachable_raises_with_details(self, mini_arm_model, ik_mod):
        """`UnreachableError` 必须携带诊断信息（距离 / 可达范围）。

        ## 为什么报错信息里要有数字

        因为"不可达"本身没有可操作性。开发者需要立刻知道"差多少、
        最小可达半径是多少"才能决定是改目标还是改臂长。
        只报"unreachable"会迫使每个人自己去算一遍。
        """
        with pytest.raises(ik_mod.UnreachableError) as ei:
            ik_mod.solve(
                mini_arm_model,
                Transform(Vector3(0.5, 0.0, 0.136), Quaternion.identity()),
            )
        exc = ei.value
        assert hasattr(exc, "distance") and exc.distance > 0
        assert hasattr(exc, "reach_max") and exc.reach_max > 0
        assert str(exc), "报错信息不能为空"

    def test_limit_violation_is_reported(self, mini_arm_model, ik_mod):
        """超出关节限位必须能区分于"不可达"。"""
        # 折叠位形：位置可达但姿态超限
        r_min = abs(0.103 - 0.097)
        with pytest.raises((ik_mod.LimitViolationError, ik_mod.IkError)):
            ik_mod.solve(
                mini_arm_model,
                Transform(Vector3(r_min, 0.0, 0.136), Quaternion.identity()),
                branch="elbow_up",
            )

    def test_clamp_marks_solution(self, mini_arm_model, ik_mod):
        """`clamp=True` 的解必须带 `clamped` 标记（不得静默夹紧）。"""
        r_min = abs(0.103 - 0.097)
        sol = ik_mod.solve(
            mini_arm_model,
            Transform(Vector3(r_min, 0.0, 0.136), Quaternion.identity()),
            branch="elbow_up",
            clamp=True,
        )
        assert sol.clamped is True
        assert sol.clamped_joints, "必须报告哪些关节被夹紧"

    def test_unknown_branch_raises(self, mini_arm_model, ik_mod):
        with pytest.raises(ik_mod.IkError):
            ik_mod.solve(
                mini_arm_model,
                Transform(Vector3(0.12, 0.0, 0.15), Quaternion.identity()),
                branch="elbow_sideways",
            )

    def test_solution_respects_limits(self, mini_arm_model, ik_mod):
        """默认（不 clamp）返回的解必须**全部**在限位内。

        ## 为什么目标点要挑，而不能随手写

        mini_arm 有两条**不同**的可行域边界，混淆它们会写出"以为在测 A、
        实际在测 B"的测试：

        ```text
        边界 1  位置可达     r ≤ r_max = L1 + L2      —— 超出即 UnreachableError
        边界 2  姿态可达     |elbow| ≤ 135°           —— 超出即"位置可达但姿态不可达"
        ```

        边界 2 **比边界 1 更紧**：`|elbow| = 135°` 对应
        `r_lim = sqrt(L1² + L2² + 2·L1·L2·cos 135°) ≈ 0.076737 m`。
        即 `r < 0.076737` 而 `r > r_min` 这个区间里的目标**位置可达但姿态不可达**。

        ⇒ 随手写 `(0.05, 0.15)`（r≈0.052）会撞上边界 2 而抛 IkError，
          那不是 bug，是**本型号真的做不到**。
        """
        targets = [
            (0.12, 0.15),   # r≈0.121
            (0.15, 0.20),   # r≈0.163
            (0.09, 0.17),   # r≈0.096
            (0.08, 0.16),   # r≈0.084，贴着边界 2（|elbow|≈130.7°）
        ]
        for x, z in targets:
            for sol in ik_mod.solve_all(
                mini_arm_model, Transform(Vector3(x, 0.0, z), Quaternion.identity())
            ):
                for jid, val in sol.joint_positions.items():
                    lim = mini_arm_model.joint(jid).limits
                    if lim is None or lim.position_min is None:
                        continue
                    assert lim.position_min - 1e-9 <= val <= lim.position_max + 1e-9, (
                        f"目标 ({x},0,{z}) 的解 {jid}={val} 超出 "
                        f"[{lim.position_min}, {lim.position_max}]"
                    )

    def test_attitude_feasibility_boundary_is_where_theory_says(
        self, mini_arm_model, ik_mod
    ):
        """★ 姿态不可达的边界必须落在 cos 定理算出的位置。

        ## 这条测试在防什么

        "位置可达但姿态不可达"是本项目**唯一**一处刻意区分"可达"两字的
        地方。如果 IK 的限位检查实现错了（比如把 `±135°` 写成 `±180°`、
        或者检查前做了错误的弧度/角度换算），它会静默地把一批**越限的
        解**当成合法解返回 —— 而下游 FK 会老老实实照它算出错的位置，
        最后表现为"机械臂在仿真里自己掰断了"。

        所以这里不测"某个点行不行"，而是测**边界落在哪**：

        ```text
        r > r_lim  ⇒  两个分支都必须可解
        r < r_lim  ⇒  两个分支都必须因姿态不可达而失败
        ```
        """
        # 从 model 里取真实几何与限位，不硬编码数字
        #
        # mini_arm 的平面 2R 链（**必须算到 TCP，不能只算到腕部**）：
        #
        #   shoulder --L1--> elbow --L2--> ee_link --L_TOOL--> tcp
        #                                  └────── L2_EFF ──────┘
        #
        # 只取 `tcp` site 的偏移 (L_TOOL=0.032) 会漏掉 `ee_link_fixed`
        # 那一段 (L2=0.065)，算出的 r_lim 偏小 —— 正是这条测试要防的
        # "用错几何"错误。
        elbow_origin = mini_arm_model.joint("elbow").origin
        l1 = abs(elbow_origin.position.x)  # shoulder → elbow
        assert l1 > 0, "上臂长度不应为 0（否则这条测试的前提失效）"

        ee_origin = mini_arm_model.joint("ee_link_fixed").origin
        l2 = abs(ee_origin.position.x)  # elbow → ee_link
        assert l2 > 0

        tcp_site = mini_arm_model.site("tcp")
        l_tool = abs(tcp_site.transform.position.x)  # ee_link → tcp
        assert l_tool > 0

        l2_eff = l2 + l_tool

        lim = mini_arm_model.joint("elbow").limits
        assert lim is not None and lim.position_max is not None
        theta_lim = lim.position_max  # 弧度

        r_lim = math.sqrt(l1 * l1 + l2_eff * l2_eff + 2 * l1 * l2_eff * math.cos(theta_lim))

        # 肩关节的世界 z = base_yaw 原点 z + shoulder 原点 z
        shoulder_z = (
            mini_arm_model.joint("base_yaw").origin.position.z
            + mini_arm_model.joint("shoulder").origin.position.z
        )
        assert shoulder_z > 0, "肩关节不应在原点（否则这条测试的前提失效）"

        eps = 5e-5  # 5e-5 > 数值噪声，< 边界处 |elbow| 的变化率对应的位移

        # 边界外侧：必须解得出来
        for r in (r_lim + eps, r_lim + 2 * eps, r_lim + 1e-3):
            T = Transform(Vector3(r, 0.0, shoulder_z), Quaternion.identity())
            sols = ik_mod.solve_all(mini_arm_model, T)
            assert len(sols) == 2, f"r={r} 应该有两个分支"
            worst = max(
                abs(s.joint_positions["elbow"]) for s in sols
            )
            assert worst <= theta_lim + 1e-9, (
                f"r={r} > r_lim={r_lim} 的解 |elbow|={worst} 超限，"
                f"说明限位检查放过了越限解"
            )

        # 边界内侧：两个分支都必须因姿态不可达而失败
        for r in (r_lim - eps, r_lim - 2 * eps, r_lim - 1e-3):
            T = Transform(Vector3(r, 0.0, shoulder_z), Quaternion.identity())
            with pytest.raises(ik_mod.IkError):
                ik_mod.solve_all(mini_arm_model, T)

    def test_reach_limits_are_consistent(self, mini_arm_model, ik_mod):
        lo, hi = ik_mod.reach_limits()
        assert 0 <= lo < hi
        assert abs(hi - (0.103 + 0.097)) < 1e-12


# ----------------------------------------------------------------------
# 通用性：FK 引擎不得包含型号特定逻辑
# ----------------------------------------------------------------------


class TestEngineGenerality:
    def test_fk_engine_has_no_robot_specific_branch(self):
        """★ 通用 FK 引擎**不得**出现 `if robot == "mini_arm"`。

        这是架构约束"Core 不得含 robot-specific branch"的机器判据。
        有了它，"加第二台机器人"才真的只需加一个目录。

        ## Phase 3 之后这条断言的分量变了

        迁移前 `generic_fk.__file__` 指向 `packages/mini_arm/kinematics/fk.py`
        —— 也就是说，之前这条"Core 里没有型号名"的断言，扫描的是**包内文件**，
        断言是关于 Core 的，证据却取自包。现在它扫的是
        `backend/kinematics/fk.py`，前提与结论终于对上了。

        下面这行断言把这个前提**钉住**：一旦有人把引擎搬回包内
        （或把 import 改回按路径加载），测试会立刻失败并说明原因，
        而不是悄悄退化成"扫了个包内文件也算通过"。
        """
        import io
        import re
        import tokenize
        from pathlib import Path

        engine_path = Path(generic_fk.__file__).resolve()
        assert engine_path == (
            Path(__file__).resolve().parent.parent / "backend" / "kinematics" / "fk.py"
        ), (
            f"本测试必须扫描 Core 引擎，实际扫的是 {engine_path}。"
            f"若你把引擎搬走了，请同步改这里 —— 否则这条断言会静默失去意义。"
        )

        src = engine_path.read_text(encoding="utf-8")
        stripped = tokenize.untokenize(
            tok for tok in tokenize.generate_tokens(io.StringIO(src).readline)
            if tok.type not in (tokenize.COMMENT, tokenize.STRING)
        )
        offenders = []
        for lineno, line in enumerate(stripped.splitlines(), 1):
            if re.search(r"""['"](mini_arm|mearm)['"]""", line):
                offenders.append(f"  line {lineno}: {line.strip()}")
        assert not offenders, (
            "通用 FK 引擎里出现了型号字面量：\n" + "\n".join(offenders)
        )

    def test_engine_reads_geometry_from_model(self, mini_arm_model):
        """★ 引擎的几何必须全部来自 model，不得硬编码。

        判据：**改一个关节的轴**，引擎的输出必须跟着变。
        若引擎把轴写死，改轴不会影响它 —— 那它就只对 mini_arm 有效。
        """
        import dataclasses

        # 构造一个"轴被改掉"的模型副本
        joints = []
        for j in mini_arm_model.joints:
            if j.id == "shoulder":
                j = dataclasses.replace(
                    j, axis=Vector3(0.0, -1.0, 0.0)  # 反向
                )
            joints.append(j)
        mutated = dataclasses.replace(mini_arm_model, joints=joints)

        q = {"base_yaw": 0.0, "shoulder": 0.6, "elbow": 0.0}
        orig = generic_fk.forward_kinematics(mini_arm_model, q)
        flipped = generic_fk.forward_kinematics(mutated, q)
        assert not orig.position.approx_eq(flipped.position, tol=1e-6), (
            "把 shoulder 的轴反向之后 FK 输出没变 ⇒ 引擎没有从 model 读轴，"
            "而是把轴硬编码了"
        )
        # 轴反向 ⇒ z 分量应符号相反（绕 +Y 与 -Y 的镜像）
        assert abs(orig.position.z - flipped.position.z) > 1e-3
