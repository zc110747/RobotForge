"""通用 IK 接口契约测试。

## 与 `tests/test_fk.py` 的分工

```text
tests/test_fk.py   → FK 的前向映射契约：入口签名、输入容忍度、输出含义、通用性
tests/test_ik.py   → IK 的**求逆**契约：解的形状、分支语义、三类失败的区分、限位
```

两者都与 `packages/mini_arm/tests/test_kinematics_regression.py` 不同 ——
后者测"mini_arm 这个型号算得准不准"，本文件测"**任何**符合契约的 IK
引擎是否满足接口约定"。

## IK 最容易被写坏的地方

IK 有**三种**不同的失败原因，而它们对调用方的含义完全不同：

```text
① UnreachableError        目标超出可达半径   → 改目标，别改代码
② LimitViolationError     位置可达但姿态不可达 → 目标在"死区"里，改目标
③ IkError（其他）          参数错 / 分支名错    → 调用方 bug，必须改代码
```

把它们混成一个 `false` 返回值是最常见的错误：调用方无法区分
"我算错了"和"这事物理上做不到"。所以本文件的核心就是**逐条钉住
这三类失败必须是可区分的**。

## 另一条要点：clamp 不能静默

`clamp=True` 是一个**明确要求"给我个次优解"**的开关 —— 它不是
"随便夹一下"。因此返回值必须带 `clamped` 标记与 `clamped_joints`
明细，否则调用方会以为拿到的解精确命中目标。
"""

from __future__ import annotations

import math

import pytest

from backend.model.types import Quaternion, Transform, Vector3

# ----------------------------------------------------------------------
# 载入 mini_arm 的 IK 原型（经 manifest 的 kinematics.ik.entry 加载）
# ----------------------------------------------------------------------


def _load_ik():
    """加载 mini_arm 的 IK 实现 —— 经 manifest 声明，与 `cli.py` 共用同一入口。

    迁移前这里是硬编码的 `root / "packages" / "mini_arm" / "kinematics" / "ik.py"`。
    那条路径与 manifest 的 `entry` 是**两份真值**：改了 manifest，测试仍去
    旧路径加载（或反过来）。现在走 `_load_package_module`，于是
    "manifest 的 entry 真的能加载"也被这些测试覆盖到了。
    """
    from backend.api.registry import get_package
    from backend.cli import _kinematics_entry, _load_package_module

    pkg = get_package("mini_arm")
    entry = _kinematics_entry(pkg, "ik")
    assert entry is not None, "mini_arm/manifest.yaml 必须声明 kinematics.ik.entry"
    return _load_package_module(pkg, entry, "ik")


@pytest.fixture(scope="session")
def ik_mod():
    return _load_ik()


# ----------------------------------------------------------------------
# 测试用的几何常量（都从 model 现算，不硬编码）
# ----------------------------------------------------------------------


def _geometry(model):
    """从 RobotModel 抽出平面 2R 的几何。**全部现算，不写死数字。**

    ```text
    shoulder --L1--> elbow --L2--> ee_link --L_TOOL--> tcp
                                   └────── L2_EFF ──────┘
    ```
    """
    l1 = abs(model.joint("elbow").origin.position.x)
    l2 = abs(model.joint("ee_link_fixed").origin.position.x)
    l_tool = abs(model.site("tcp").transform.position.x)
    shoulder_z = (
        model.joint("base_yaw").origin.position.z
        + model.joint("shoulder").origin.position.z
    )
    theta_lim = model.joint("elbow").limits.position_max
    return {
        "l1": l1,
        "l2": l2,
        "l_tool": l_tool,
        "l2_eff": l2 + l_tool,
        "shoulder_z": shoulder_z,
        "theta_lim": theta_lim,
    }


def _r_attitude_limit(geom):
    """姿态可达的最小半径（|elbow| = 限位 时由余弦定理给出）。"""
    return math.sqrt(
        geom["l1"] ** 2
        + geom["l2_eff"] ** 2
        + 2 * geom["l1"] * geom["l2_eff"] * math.cos(geom["theta_lim"])
    )


def _at(r, geom, y=0.0):
    """水平方向 r、肩高处的目标位姿。"""
    return Transform(Vector3(r, y, geom["shoulder_z"]), Quaternion.identity())


# ----------------------------------------------------------------------
# 解的形状
# ----------------------------------------------------------------------


class TestSolutionShape:
    """IK 返回的对象必须自带"这个解有多可信"的全部信息。"""

    def test_solve_returns_solution_object(self, mini_arm_model, ik_mod):
        """`solve()` 返回 `IkSolution`，不是裸 dict。

        ## 为什么返回对象而不是 `dict[str, float]`

        因为"这是哪个分支"、"有没有触限"、"误差多大"都是调用方需要的
        **决策依据**。返回裸 dict 会迫使调用方自己重算 —— 而重算就是
        第二份实现的开始，它会与第一份漂移。
        """
        sol = ik_mod.solve(mini_arm_model, _at(0.12, _geometry(mini_arm_model)))
        assert hasattr(sol, "joint_positions")
        assert hasattr(sol, "branch")
        assert hasattr(sol, "position_error")
        assert hasattr(sol, "clamped")
        assert isinstance(sol.joint_positions, dict)
        assert sol.to_dict()["branch"] == sol.branch

    def test_solution_covers_all_mobile_joints(self, mini_arm_model, ik_mod):
        """解必须给出**所有**可动关节的值（不能只给部分）。

        只给部分关节的解在应用到 runtime 时，缺失的关节会保留上一次的
        值 —— 表现为"机械臂每次运动都从奇怪的地方开始"。
        """
        sol = ik_mod.solve(mini_arm_model, _at(0.12, _geometry(mini_arm_model)))
        assert set(sol.joint_positions.keys()) == set(
            mini_arm_model.mobile_joint_ids()
        ), (
            f"解只覆盖了 {set(sol.joint_positions.keys())}，"
            f"期望 {set(mini_arm_model.mobile_joint_ids())}"
        )

    def test_solution_values_are_finite(self, mini_arm_model, ik_mod):
        """解必须是有限值 —— NaN 会让 FK 输出 NaN 而不报错。"""
        sol = ik_mod.solve(mini_arm_model, _at(0.13, _geometry(mini_arm_model)))
        for jid, val in sol.joint_positions.items():
            assert math.isfinite(val), f"{jid}={val} 不是有限值"

    def test_position_error_is_truthful(self, mini_arm_model, ik_mod):
        """★ 自报的 `position_error` 必须等于**独立重算**的 FK 误差。

        ## 这条在防什么

        `position_error` 是调用方判断"这个解能不能用"的唯一依据。
        如果它是个常数 0、或者只反映内部残差而不反映真实 FK 误差，
        调用方就会**无条件接受一个明显错误的解**。

        而"自报误差为 0"这件事本身无法自证 —— 除非用**另一条路径**
        （这里是把解代回 FK）独立算一遍。
        """
        ik = ik_mod
        fk = _load_fk()
        geom = _geometry(mini_arm_model)

        for r in (0.09, 0.12, 0.15, 0.19):
            target = _at(r, geom)
            for sol in ik.solve_all(mini_arm_model, target):
                achieved = fk.forward_kinematics(
                    mini_arm_model, sol.joint_positions
                )
                real_err = (achieved.position - target.position).norm()
                assert abs(real_err - sol.position_error) < 1e-12, (
                    f"r={r} {sol.branch}: 自报误差 {sol.position_error:.3e} "
                    f"≠ 回代 FK 实测 {real_err:.3e}"
                )


def _load_fk():
    """Core 通用 FK —— 用来**独立**回代校验包内 IK 的自报误差。

    ## Phase 3 之后这个函数变成一个直白的 import

    迁移前它按路径加载 `packages/mini_arm/kinematics/fk.py` 并调
    `forward_kinematics`。现在 Core 有了正式引擎，直接 import 即可 ——
    而且**更该用 Core**：这些测试的本质是"用一条独立路径回算，看自报的
    `position_error` 是不是真的"。

    用同一个包内的解析式 FK 回代会削弱这个论据（同一个包里两份实现，
    可能共享同一个错误假设，比如把 `L_TOOL` 漏掉）。用 Core 的链式乘法
    回代才是真正的**独立**校验：它不读包里的几何常量，只读 model。
    """
    import backend.kinematics.fk as core_fk

    return core_fk


# ----------------------------------------------------------------------
# 分支语义
# ----------------------------------------------------------------------


class TestBranches:
    """2R 平面臂有两个解（肘上 / 肘下），这是物理事实不是实现细节。"""

    def test_solve_all_returns_both_branches(self, mini_arm_model, ik_mod):
        sols = ik_mod.solve_all(mini_arm_model, _at(0.12, _geometry(mini_arm_model)))
        assert len(sols) == 2
        assert {s.branch for s in sols} == {"elbow_up", "elbow_down"}

    def test_both_branches_hit_the_same_target(self, mini_arm_model, ik_mod):
        """★ 两个分支必须**都命中同一个目标点**。

        ## 为什么不能只检查"返回了 2 个解"

        只数个数的话，一个简单 bug（两个分支算了同一套公式）也能通过。
        必须两个都代回 FK，都落在目标上 —— 才证明它们真的是**两个**
        不同的构型解，而不是同一个解写了两遍。
        """
        fk = _load_fk()
        geom = _geometry(mini_arm_model)
        for r in (0.09, 0.13, 0.17):
            target = _at(r, geom)
            sols = ik_mod.solve_all(mini_arm_model, target)
            for sol in sols:
                ach = fk.forward_kinematics(mini_arm_model, sol.joint_positions)
                err = (ach.position - target.position).norm()
                assert err < 1e-9, f"{sol.branch} r={r} 未命中目标，误差 {err:.3e}"
            # 两个分支的关节值必须真的不同
            assert sols[0].joint_positions != sols[1].joint_positions, (
                "两个分支给出了完全相同的关节值 ⇒ 分支逻辑没生效"
            )

    def test_branches_are_mirrored_in_elbow_sign(self, mini_arm_model, ik_mod):
        """两个分支的 elbow 必须**符号相反、绝对值相同**（镜像的代数特征）。"""
        sols = ik_mod.solve_all(mini_arm_model, _at(0.12, _geometry(mini_arm_model)))
        by = {s.branch: s for s in sols}
        e_up = by["elbow_up"].joint_positions["elbow"]
        e_dn = by["elbow_down"].joint_positions["elbow"]
        assert e_up > 0 > e_dn, f"elbow_up={e_up}, elbow_down={e_dn} 符号不对"
        assert abs(abs(e_up) - abs(e_dn)) < 1e-12

    def test_solve_all_sorted_by_error(self, mini_arm_model, ik_mod):
        """`solve_all` 的结果按误差升序 —— 调用方可以取 `[0]` 当最优解。"""
        sols = ik_mod.solve_all(mini_arm_model, _at(0.12, _geometry(mini_arm_model)))
        errs = [s.position_error for s in sols]
        assert errs == sorted(errs)

    def test_solve_all_is_deterministic(self, mini_arm_model, ik_mod):
        """同目标必须给同样的解序列（含顺序）。"""
        target = _at(0.12, _geometry(mini_arm_model))
        a = [s.to_dict() for s in ik_mod.solve_all(mini_arm_model, target)]
        b = [s.to_dict() for s in ik_mod.solve_all(mini_arm_model, target)]
        assert a == b

    def test_unknown_branch_raises(self, mini_arm_model, ik_mod):
        """分支名写错必须报错，而不是静默回退到默认分支。"""
        with pytest.raises(ik_mod.IkError):
            ik_mod.solve(
                mini_arm_model,
                _at(0.12, _geometry(mini_arm_model)),
                branch="elbow_sideways",
            )

    def test_explicit_branch_selects_that_branch(self, mini_arm_model, ik_mod):
        """显式传 `branch=` 必须真的返回该分支，而不是"随便给一个"。"""
        geom = _geometry(mini_arm_model)
        target = _at(0.12, geom)
        up = ik_mod.solve(mini_arm_model, target, branch="elbow_up")
        dn = ik_mod.solve(mini_arm_model, target, branch="elbow_down")
        assert up.branch == "elbow_up"
        assert dn.branch == "elbow_down"
        assert up.joint_positions["elbow"] > 0 > dn.joint_positions["elbow"]


# ----------------------------------------------------------------------
# 三类失败的区分（本文件的核心）
# ----------------------------------------------------------------------


class TestFailureTaxonomy:
    """★ IK 的三种失败必须**互相可区分**。

    混成一种是最常见的接口设计错误：调用方拿不到"该改目标还是该改代码"
    的信息，只能靠字符串匹配报错信息 —— 那是脆的。
    """

    def test_out_of_reach_is_distinguishable(self, mini_arm_model, ik_mod):
        """① 超出可达半径 → `UnreachableError`，且必须携带诊断数字。

        ## 为什么报错信息里要有数字

        "不可达"本身没有可操作性。开发者需要立刻知道"差多少、最小可达
        半径是多少"才能决定是改目标还是改臂长。只报 "unreachable"
        会迫使每个人自己再算一遍 —— 而每个人算的都可能不一样。
        """
        geom = _geometry(mini_arm_model)
        far = geom["l1"] + geom["l2_eff"] + 0.2  # 远超 0.200
        with pytest.raises(ik_mod.UnreachableError) as ei:
            ik_mod.solve(mini_arm_model, _at(far, geom))
        exc = ei.value
        assert hasattr(exc, "distance") and exc.distance > 0
        assert hasattr(exc, "reach_max") and exc.reach_max > 0
        assert exc.distance > exc.reach_max, "报出的距离应真的超限"
        assert str(exc), "报错信息不能为空"

    def test_unreachable_is_not_a_limit_violation(self, mini_arm_model, ik_mod):
        """① 不得被误报成 ② —— 否则调用方会去查关节限位，白费时间。"""
        geom = _geometry(mini_arm_model)
        with pytest.raises(ik_mod.UnreachableError):
            ik_mod.solve(mini_arm_model, _at(geom["l1"] + geom["l2_eff"] + 0.2, geom))
        # 反向：姿态不可达的点不得报 UnreachableError
        with pytest.raises(ik_mod.IkError) as ei:
            ik_mod.solve(mini_arm_model, _at(0.03, geom), branch="elbow_up")
        assert not isinstance(ei.value, ik_mod.UnreachableError), (
            "位置可达但姿态不可达，不应报 UnreachableError"
        )

    def test_limit_violation_is_distinguishable(self, mini_arm_model, ik_mod):
        """② 位置可达但姿态不可达 → 报限位类错误（不是 Unreachable）。"""
        geom = _geometry(mini_arm_model)
        r_dead = 0.03  # 远小于 r_lim，位置在环内，但需要 |elbow|≈163° > 135°
        assert r_dead > abs(geom["l1"] - geom["l2_eff"]), "该点应在 2R 环内"
        assert r_dead < _r_attitude_limit(geom), "该点应在姿态死区内"
        with pytest.raises(ik_mod.IkError):
            ik_mod.solve(mini_arm_model, _at(r_dead, geom), branch="elbow_up")

    def test_failure_messages_name_the_cause(self, mini_arm_model, ik_mod):
        """失败信息必须**说清原因**，可被日志/UI 直接展示。"""
        geom = _geometry(mini_arm_model)
        msgs = {}
        try:
            ik_mod.solve(mini_arm_model, _at(geom["l1"] + geom["l2_eff"] + 0.2, geom))
        except Exception as e:
            msgs["unreachable"] = str(e)
        try:
            ik_mod.solve(mini_arm_model, _at(0.03, geom))
        except Exception as e:
            msgs["infeasible"] = str(e)
        assert msgs["unreachable"] != msgs["infeasible"], (
            "两种失败的报错信息无法区分"
        )
        assert len(msgs["unreachable"]) > 10 and len(msgs["infeasible"]) > 10


# ----------------------------------------------------------------------
# 限位与 clamp
# ----------------------------------------------------------------------


class TestLimitsAndClamp:
    def test_solution_respects_limits(self, mini_arm_model, ik_mod):
        """默认（不 clamp）返回的解必须**全部**在限位内。

        ## 为什么目标点要挑，而不能随手写

        mini_arm 有两条**不同**的可行域边界，混淆它们会写出"以为在测 A、
        实际在测 B"的测试：

        ```text
        边界 1  位置可达     r ≤ r_max = L1 + L2_EFF   → 超出即 UnreachableError
        边界 2  姿态可达     |elbow| ≤ 135°            → 超出即"位置可达但姿态不可达"
        ```

        边界 2 **比边界 1 更紧**：`|elbow| = 135°` 对应 r ≈ 0.0767 m
        （由余弦定理现算，见 `_r_attitude_limit`）。即 `r_min < r < 0.0767`
        这个区间里的目标**位置可达但姿态不可达**。

        ⇒ 随手写 `(0.05, 0.15)`（r≈0.052）会撞上边界 2 而抛 IkError，
          那不是 bug，是**本型号真的做不到**。
        """
        geom = _geometry(mini_arm_model)
        r_lim = _r_attitude_limit(geom)
        targets = [
            (0.12, 0.15),
            (0.15, 0.20),
            (0.09, 0.17),
            (0.08, 0.16),
            (r_lim + 5e-4, geom["shoulder_z"]),  # 贴着姿态边界外侧
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
        """★ 姿态不可达的边界必须落在余弦定理算出的位置。

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
        geom = _geometry(mini_arm_model)
        r_lim = _r_attitude_limit(geom)

        # 边界外侧：必须解得出来，且解不越限
        for r in (r_lim + 5e-4, r_lim + 1e-3, r_lim + 1e-2):
            sols = ik_mod.solve_all(mini_arm_model, _at(r, geom))
            assert len(sols) == 2, f"r={r} 应该有两个分支"
            worst = max(abs(s.joint_positions["elbow"]) for s in sols)
            assert worst <= geom["theta_lim"] + 1e-9, (
                f"r={r} > r_lim={r_lim:.6f} 的解 |elbow|={worst} 超限，"
                f"说明限位检查放过了越限解"
            )

        # 边界内侧：两个分支都必须因姿态不可达而失败
        for r in (r_lim - 5e-4, r_lim - 1e-3, r_lim - 1e-2):
            assert r > abs(geom["l1"] - geom["l2_eff"]), (
                f"r={r} 应仍在 2R 环内（否则测的是 Unreachable 而非姿态）"
            )
            with pytest.raises(ik_mod.IkError) as ei:
                ik_mod.solve_all(mini_arm_model, _at(r, geom))
            assert not isinstance(ei.value, ik_mod.UnreachableError), (
                f"r={r} 位置在环内，应报姿态不可达而非 Unreachable"
            )

    def test_clamp_marks_solution(self, mini_arm_model, ik_mod):
        """`clamp=True` 的解必须带 `clamped` 标记（不得静默夹紧）。"""
        geom = _geometry(mini_arm_model)
        sol = ik_mod.solve(
            mini_arm_model, _at(0.03, geom), branch="elbow_up", clamp=True
        )
        assert sol.clamped is True
        assert sol.clamped_joints, "必须报告哪些关节被夹紧"

    def test_clamp_stays_within_limits(self, mini_arm_model, ik_mod):
        """夹紧后的解必须**真的**在限位内 —— 这是 clamp 的全部意义。"""
        geom = _geometry(mini_arm_model)
        sol = ik_mod.solve(
            mini_arm_model, _at(0.03, geom), branch="elbow_up", clamp=True
        )
        for jid, val in sol.joint_positions.items():
            lim = mini_arm_model.joint(jid).limits
            if lim is None or lim.position_min is None:
                continue
            assert lim.position_min - 1e-9 <= val <= lim.position_max + 1e-9, (
                f"clamp 后 {jid}={val} 仍在限位外"
            )

    def test_clamp_reports_honest_residual_error(self, mini_arm_model, ik_mod):
        """★ 夹紧必然产生误差，`position_error` 必须**如实报告**它。

        ## 为什么这条重要

        夹紧后的解**不可能**再精确命中目标（除非目标本来就在边界上）。
        如果实现把 `position_error` 清成 0，调用方会以为拿到了精确解，
        于是掩盖掉"目标在死区里"这个事实。实测该残差约 74 mm，是
        量级很大的偏差 —— 正好用来验证"它没有被悄悄抹掉"。
        """
        geom = _geometry(mini_arm_model)
        target = _at(0.03, geom)
        sol = ik_mod.solve(mini_arm_model, target, branch="elbow_up", clamp=True)
        assert sol.position_error > 1e-3, (
            f"夹紧到 135° 的解不可能零误差，却报了 {sol.position_error:.3e}"
        )

        # 且该误差必须与回代 FK 的实测一致
        fk = _load_fk()
        ach = fk.forward_kinematics(mini_arm_model, sol.joint_positions)
        real = (ach.position - target.position).norm()
        assert abs(real - sol.position_error) < 1e-12, (
            f"自报 {sol.position_error:.6e} ≠ 实测 {real:.6e}"
        )

    def test_solve_all_rejects_clamped_solutions_beyond_tolerance(
        self, mini_arm_model, ik_mod
    ):
        """★ `solve_all` 必须**拒绝**误差过大的 clamp 解。

        ## 两个入口的职责划分（刻意的不对称）

        ```text
        solve(..., clamp=True)     "我知道目标不可精确达到，给我最近的可行解"
                                   → 返回解 + clamped 标记 + 诚实的残差

        solve_all(...)             "给我所有**能精确命中**目标的解"
                                   → 做不到就不出现在结果里
        ```

        这不是不一致，而是**责任划分**：前者是"退让"，后者是"枚举"。
        如果把 clamp 解也塞进 `solve_all`，调用方遍历结果时会误以为
        每个解都精确命中 —— 那是最糟的静默错误。
        """
        geom = _geometry(mini_arm_model)
        # solve 能给出夹紧解
        sol = ik_mod.solve(mini_arm_model, _at(0.03, geom), clamp=True)
        assert sol.position_error > 1e-3

        # 但 solve_all 不得接受它
        with pytest.raises(ik_mod.IkError):
            ik_mod.solve_all(mini_arm_model, _at(0.03, geom))

    def test_reach_limits_are_consistent(self, mini_arm_model, ik_mod):
        """`reach_limits()` 必须与 model 里的几何一致（不含底座高度）。"""
        geom = _geometry(mini_arm_model)
        lo, hi = ik_mod.reach_limits()
        assert 0 <= lo < hi
        assert abs(hi - (geom["l1"] + geom["l2_eff"])) < 1e-12, (
            f"r_max={hi} 与 L1+L2_EFF={geom['l1'] + geom['l2_eff']} 不一致"
        )
        assert abs(lo - abs(geom["l1"] - geom["l2_eff"])) < 1e-12


# ----------------------------------------------------------------------
# 通用性
# ----------------------------------------------------------------------


class TestEngineAgainstModel:
    """★ IK 对 `RobotModel` 的**使用程度**必须是明确的、有测试钉住的。

    ## 现状（Phase 3 之后的定论）：包内 IK 是"型号算法"，且这是设计决定

    实测（2026-09-15，Phase 3 期间复核仍然成立）：

    ```text
    ik.py 中 'axis' 出现次数 = 0
    ik.py 中 'model.<属性>' 出现次数 = 0（只有 import 与类型标注用到了 RobotModel）
    改 shoulder 的 axis 后调用 solve() → 返回完全相同的关节角
    ```

    即：**IK 只用 `model` 做类型检查与回代自检，几何与限位全部来自
    模块常量**（`L1 / L2_EFF / YAW_LIMIT / SHOULDER_LIMIT / ELBOW_LIMIT`，
    而这些常量本身从 `fk.py` 复用，与 MJCF 数值一致）。

    ## Phase 3 的结论：这不是"待修的缺陷"，而是**刻意的边界**

    Phase 3 把通用 FK 提升进了 Core（`backend/kinematics/fk.py`）——
    因为"逐级相乘 Link-Joint 链"对**任何** Tree 型模型都成立。

    但 IK **没有**跟着提升，理由是形式上的：mini_arm 的逆解是
    "平面 2R 余弦定理 + `atan2` 定偏航 + 几何判肘部朝向"的**闭式解**，
    它依赖于这台机器的机构（两段平行臂、一个偏航轴）。
    一个通用 IK 只可能是**数值迭代** —— 而迭代会引入容差，
    让本项目的核心验收（`Joint → FK → Pose → IK → Joint'` 的精确往返）
    从"严格相等"退化成"误差小于 ε"。

    ⇒ 因此 `manifest.yaml` 里显式写着 `kinematics.ik.type: package`。
      这不是"还没做"，是**声明**：这台机器用包内解析解。

    ## 但现状必须被钉住，否则会安静地腐化

    "轴改了 IK 不动"依然是一个**危险信号**：它意味着如果有人把
    `packages/mini_arm/kinematics/` 当成 Core 用，就会得到一个"看起来
    通用、实际写死"的引擎。所以本类做三件事：

    1. **断言现状**（常量与 model 一致）—— 保证"写死"至少是**对**的；
    2. **断言声明**（`manifest.kinematics.ik.type == 'package'`）——
       让"实现是包内的"与"manifest 说是包内的"不可能悄悄脱钩；
    3. **断言边界**（解决不对 model 的轴/连杆长度做承诺）—— 一旦有人
       真的把 IK 提升为 Core 引擎，本测试会立即失败，提醒他同步改
       manifest 的 type 与这两条说明，而不是留下一份说谎的文档。
    """

    def test_constants_agree_with_model(self, mini_arm_model, ik_mod):
        """① IK 的写死常量必须与 model 里的几何/限位**逐项一致**。

        这是"允许写死"的唯一前提：写死的值必须是对的。一旦有人只改了
        MJCF 而忘了改常量，这里会立刻失败 —— 而不是等到机械臂在仿真里
        跑出错误轨迹。
        """
        geom = _geometry(mini_arm_model)

        assert abs(ik_mod.L1 - geom["l1"]) < 1e-12, (
            f"IK.L1={ik_mod.L1} ≠ model 的 shoulder→elbow={geom['l1']}"
        )
        assert abs(ik_mod.L2_EFF - geom["l2_eff"]) < 1e-12, (
            f"IK.L2_EFF={ik_mod.L2_EFF} ≠ model 的 elbow→tcp={geom['l2_eff']}"
        )
        assert abs(ik_mod.Z_BASE - geom["shoulder_z"]) < 1e-12, (
            f"IK.Z_BASE={ik_mod.Z_BASE} ≠ model 的肩高={geom['shoulder_z']}"
        )

        # 关节限位也必须一致
        for jid, const_name in (
            ("base_yaw", "YAW_LIMIT"),
            ("shoulder", "SHOULDER_LIMIT"),
            ("elbow", "ELBOW_LIMIT"),
        ):
            lim = mini_arm_model.joint(jid).limits
            assert lim is not None and lim.position_max is not None, (
                f"{jid} 应有对称限位"
            )
            const = getattr(ik_mod, const_name)
            assert abs(const - lim.position_max) < 1e-12, (
                f"IK.{const_name}={const} ≠ model 的 {jid} 限位={lim.position_max}"
            )

    def test_manifest_declares_ik_as_a_package_implementation(self):
        """①′ ★ manifest 必须声明 `ik.type: package`，与实现位置对账。

        ## 为什么这条断言值得单独存在

        `test_package_ik_is_a_2r_analytic_solver_not_a_generic_engine`
        断言的是**行为**（改轴不影响 IK）；本测试断言的是**声明**
        （manifest 承认这件事）。

        两者缺一不可，因为它们的失效方式不同：

        - 只测行为 ⇒ 有人把 IK 换成 Core 数值引擎，行为变了，测试失败，
          但他可能"修"成读轴就收工 —— 而 manifest 里仍写着 `package`，
          声明从此与实现对不上（`cli.py` 会按 manifest 去加载包内文件）。
        - 只测声明 ⇒ manifest 写着 `package`，但实现其实被换成了引擎。

        两条一起，"声明 = 实现"这个不变式才真正被钉住。
        这也是 `manifest.kinematics.ik.type` 这个字段在 v0.1 的**唯一消费者**
        （另一个消费者是 `cli.py` 的 `_resolve_capability`）。
        """
        from backend.api.registry import get_package

        pkg = get_package("mini_arm")
        kin = pkg.raw_manifest.get("kinematics")
        assert isinstance(kin, dict), "manifest 必须有 kinematics 段"
        ik_section = kin.get("ik")
        assert isinstance(ik_section, dict), "manifest 必须有 kinematics.ik 段"

        assert ik_section.get("type") == "package", (
            f"manifest 声明 kinematics.ik.type = {ik_section.get('type')!r}，"
            f"但经 Phase 3 复核，mini_arm 的 IK 是**包内解析解**（设计决定）。"
            f"若你真的把它换成了 Core 通用引擎，请同时："
            f"1) 把这里与 manifest 改成 'engine'；"
            f"2) 重写 test_package_ik_is_a_2r_analytic_solver_not_a_generic_engine；"
            f"3) 想清楚'精确往返'这个验收目标还成不成立。"
        )
        assert ik_section.get("entry") == "kinematics/ik.py", (
            "type=package 时 entry 必须指向包内实现文件"
        )

    def test_package_ik_is_a_2r_analytic_solver_not_a_generic_engine(
        self, mini_arm_model, ik_mod
    ):
        """② ★ 记录包内 IK 的**设计边界**：它是 2R 解析解，不消费 model 的轴。

        ## 这条测试的写法是一个承诺

        它断言"改轴不影响 IK"。在 Phase 3 之前，这被写成"v0.1 的已知缺陷"；
        Phase 3 复核后改判为：**这是刻意的边界**（见本类 docstring 的
        "Phase 3 的结论"）。改判的理由不是"懒得做"，而是形式上的 ——
        通用 IK 只能是数值迭代，会破坏精确往返验收。

        所以这条测试现在的职责是**守住这个边界**：

        - 有人把 IK 升级成真·通用引擎（读轴） ⇒ 本测试失败
          ⇒ 他必须来改这条测试，并在改动时读到上面那段理由
          ⇒ 他要么有更好的方案，要么会意识到自己正在牺牲什么
        - 没人碰 ⇒ 这个边界在文档/测试里明明白白，不会被误认为"已通用"

        比"写进 TODO 注释"强的地方：注释不会被 CI 检查，测试会。
        **而且边界是两个方向的** —— 它同时拒绝"假装通用"和"无意识通用化"。
        """
        import dataclasses

        geom = _geometry(mini_arm_model)
        target = _at(0.12, geom)

        joints = []
        for j in mini_arm_model.joints:
            if j.id == "shoulder":
                j = dataclasses.replace(j, axis=Vector3(0.0, -1.0, 0.0))
            joints.append(j)
        mutated = dataclasses.replace(mini_arm_model, joints=joints)

        base = ik_mod.solve(mini_arm_model, target, branch="elbow_up")
        flipped = ik_mod.solve(mutated, target, branch="elbow_up")
        assert base.joint_positions == flipped.joint_positions, (
            "包内 IK 开始消费 model 的 axis 了 —— 这说明它正在被改造成通用引擎。"
            "若这是**有意**的：请同步改 manifest 的 kinematics.ik.type、"
            "test_manifest_declares_ik_as_a_package_implementation，"
            "并重新论证'精确往返'这个 Phase 3 验收目标。"
            "若这是**无意**的：请撤回 —— 包内 IK 的定位是 2R 解析解。"
        )

    def test_ik_still_validates_against_the_model_it_is_given(
        self, mini_arm_model, ik_mod
    ):
        """③ ★ 回代自检必须用**传进来的** model，而不是又拿常量算一遍。

        ## 为什么这条不能用"改肘关节原点"来测

        实测发现 `mini_arm/kinematics/fk.py` 里其实有**两个**函数：

        ```text
        forward_kinematics(model, q)          解析式：L1 / L2 / L_TOOL 全是**常量**
                                              只用 model 取旋转轴 + 算四元数
        forward_kinematics_generic(model, q)  通用链式：逐级遍历 Link-Joint，
                                              几何**全部**来自 model
        ```

        ★ Phase 3 之后，第二个名字变成了**重导出**：它的实现搬到了
          `backend/kinematics/fk.py`（Core），包内只是
          `forward_kinematics_generic = core.forward_kinematics` 的别名。
          见 `packages/mini_arm/kinematics/fk.py` 末尾的"对 Core 的重导出"一节。
          本测试关心的性质（解析式几何来自常量、自检读 model 只为轴）**没有变**。

        而 `ik.py` 的自检调用的是**前者**（`forward_kinematics`，即解析式）。
        因此：

        - 改 model 的 `elbow` 原点 ⇒ 解析式 FK 不动 ⇒ 自检误差不动
        - 改 model 的 `shoulder` **axis** ⇒ `_axis_of(model, "shoulder")` 读到新轴
          ⇒ 四元数朝向变了 ⇒ `orientation_error` 变了，但 `position_error`
          仍不变（位置是用常量算的）

        ## 所以判据要落在**朝向**上，而不是位置

        自检里位置误差和朝向误差都被算了。用"改轴"来验证"自检真的用了
        model"，必须看 `orientation_error` —— 位置那一路本来就与 model 无关，
        用位置去测会得到一个**恒等同于 0** 的假检查。

        ## 而且必须改 `shoulder` 的轴，且**目标朝向不能是单位四元数**

        两个实测发现的陷阱（2026-09-15）：

        **陷阱 1 —— `base_yaw` 的轴在 φ=0 时不可观测。**
        `_q_revolute(axis, 0.0)` 对任何 axis 都返回单位四元数：

        ```text
        base_yaw 轴 = +X / +Y / +Z  →  orientation 全是 [0, 0.46529, 0, 0.885158]
        shoulder 轴 = +Y / -Y / +Z  →  orientation 分别变成
                                      [0,.465,0,.885] / [0,-.465,0,.885] / [0,0,.465,.885]
        ```

        **陷阱 2 —— 目标朝向为 `identity` 时，`orientation_error` 恒为
        `2·acos(|w|)`，与轴无关。** 因为 `_orientation_error` 用 `abs(dot)`
        处理双重覆盖，而 `dot(identity, q) = q.w`：

        ```text
        shoulder 轴 = +Y / -Y / +Z  且 target.orientation = identity()
            → ori_err 全是 0.9679245648053303   （只是 2·acos(0.885158)）
        ```

        ⇒ 用 identity 目标去测"改轴影响朝向误差"，会得到一个**恒真的假检查**：
          轴怎么改、朝向怎么变，`|w|` 都不变，于是断言"误差变了"永远失败、
          或者被人为放宽成"误差可能变"就永远通过。

        ⇒ 正确做法：目标朝向取**机械臂某个真实构型的朝向**（非退化）。
          此时实测：

        ```text
        shoulder 轴 = +Y  → ori_err = 0.000000000（精确命中）
        shoulder 轴 = +Z  → ori_err = 1.341019055
        shoulder 轴 = -Y  → ori_err = 1.935849130
        ```
        """
        import dataclasses

        geom = _geometry(mini_arm_model)
        # ★ 参考朝向必须取自**解析式** FK（包内 fk.py），而不是 Core 的链式 FK。
        #
        #   实测（Phase 3 迁移时踩到）：两种实现在数学上等价，但浮点运算的
        #   顺序不同（解析式直接按 L1/L2/L_TOOL 拼四元数；Core 逐级
        #   compose Transform），于是同一个关节角得到的四元数会差 ~3e-8 rad。
        #
        #   而 `base.orientation_error` 是 IK 用**解析式** FK 回代算出来的。
        #   所以只要参考值来自 Core，下面那条 `< 1e-9` 就会失败 ——
        #   哪怕 IK 的自检一点错都没有。
        #
        #   这不是"放宽容差"的时刻，而是"参考值与被测路径必须同源"。
        #   本测试要断言的是"IK 精确复现它自己那套几何约定"，
        #   因此参考值必须来自 IK 实际使用的那条路径。
        from backend.api.registry import get_package
        from backend.cli import _kinematics_entry, _load_package_module

        pkg = get_package("mini_arm")
        analytic_fk = _load_package_module(pkg, _kinematics_entry(pkg, "fk"), "fk")

        # 目标朝向取一个**非退化**的真实构型：否则 ori_err 只反映 |w|。
        ref_q = {"base_yaw": 0.0, "shoulder": -0.8878671629120187,
                 "elbow": 1.855791727717349}
        reference_ori = analytic_fk.forward_kinematics(mini_arm_model, ref_q).orientation
        target = Transform(
            Vector3(0.12, 0.0, geom["shoulder_z"]), reference_ori
        )

        # 把 shoulder 的轴从 +Y 改成 +Z：位置公式用的是 L1/L2 常量（不变），
        # 但朝向的四元数轴变了，因此 orientation_error 必然不同。
        joints = []
        for j in mini_arm_model.joints:
            if j.id == "shoulder":
                j = dataclasses.replace(j, axis=Vector3(0.0, 0.0, 1.0))
            joints.append(j)
        mutated = dataclasses.replace(mini_arm_model, joints=joints)

        base = ik_mod.solve(mini_arm_model, target, branch="elbow_up")
        changed = ik_mod.solve(mutated, target, branch="elbow_up")

        assert base.position_error < 1e-9, "原始 model 上自检应几乎零位置误差"
        assert base.orientation_error < 1e-9, (
            "参考构型自己应精确命中它自己的朝向（否则参考值取错了）"
        )
        # 位置误差**不应**变化（解析式 FK 的位置部分不读 model）——
        # 这本身就是上面那段说明的机器可验证版本。
        assert abs(changed.position_error - base.position_error) < 1e-12, (
            "解析式 FK 的位置居然受 model 的轴影响 ⇒ 说明实现变了，"
            "请重读本测试的说明并重新划定判据。"
        )
        # 朝向误差必须变 —— 这才是"自检真的用了传入 model"的证据。
        assert changed.orientation_error > 1.0, (
            f"把 shoulder 的轴从 +Y 改成 +Z 后，自检的朝向误差是 "
            f"{changed.orientation_error:.9f}，与原始的 {base.orientation_error:.9f} "
            f"没有实质差别 ⇒ 自检没有真的用传入的 model"
        )

    def test_base_yaw_axis_is_unobservable_at_zero_yaw(
        self, mini_arm_model, ik_mod
    ):
        """⑤ ★ 记录一个容易骗过测试的事实：`base_yaw` 的轴在 φ=0 时不可观测。

        ## 这条测试在防"假检查"

        `_q_revolute(axis, 0.0)` 对任何 `axis` 都返回单位四元数。因此在
        `base_yaw = 0` 的位形下，改 `base_yaw` 的轴**不会**改变 FK 输出。

        如果谁写了"改 base_yaw 轴 → 断言输出变化"的通用性测试，他会得到
        一个永远失败的测试（若不看这个事实），或者更糟 —— 若他为了让它
        通过而放宽成"输出可能变化"，就得到了一个**永远不会失败**的测试。

        本测试把这个事实**显式**写成断言，于是：

        - 谁要拿轴做判据，会先看到这里该用哪个关节；
        - 将来若 `_q_revolute` 改成"零角也带轴信息"（不可能，但若发生），
          这条会失败并提醒重新审视上面所有基于轴不可观测性的推理。
        """
        import dataclasses

        q = {"base_yaw": 0.0, "shoulder": 0.3, "elbow": 0.5}
        mutated_q = {"base_yaw": 0.0, "shoulder": 0.3, "elbow": 0.5}
        fk = _load_fk()

        base = fk.forward_kinematics(mini_arm_model, q)
        for ax in (Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)):
            joints = [
                dataclasses.replace(j, axis=ax) if j.id == "base_yaw" else j
                for j in mini_arm_model.joints
            ]
            mm = dataclasses.replace(mini_arm_model, joints=joints)
            out = fk.forward_kinematics(mm, mutated_q)
            assert out.orientation.approx_eq(base.orientation, tol=1e-15), (
                f"base_yaw 轴改成 {ax} 后朝向变了 —— 零角处轴变得可观测了，"
                f"请重新审视依赖此性质的分析"
            )
            assert out.position.approx_eq(base.position, tol=1e-15)

        # 而 base_yaw ≠ 0 时它是可观测的 —— 这才让上面的结论有边界。
        q2 = {"base_yaw": 0.7, "shoulder": 0.3, "elbow": 0.5}
        base2 = fk.forward_kinematics(mini_arm_model, q2)
        joints = [
            dataclasses.replace(j, axis=Vector3(1, 0, 0)) if j.id == "base_yaw" else j
            for j in mini_arm_model.joints
        ]
        mm2 = dataclasses.replace(mini_arm_model, joints=joints)
        out2 = fk.forward_kinematics(mm2, q2)
        assert not out2.orientation.approx_eq(base2.orientation, tol=1e-6), (
            "base_yaw≠0 时轴应可观测"
        )

    def test_position_error_uses_analytic_fk_geometry_from_constants(
        self, mini_arm_model, ik_mod
    ):
        """④ 钉住"位置自检用常量"这一事实，防止它被误当成通用性证据。

        ## 这条测试存在的理由

        上一条测试发现了一个容易误判的组合：**IK 的常量几何** +
        **解析式 FK 的常量几何** 让"改 model 几何 → 位置误差不变"。

        一个不谨慎的读者会把它读成"自检是假的"。实际上自检是**真的**，
        只是它验证的是"我算的关节角在那个写死的几何下能不能复现目标" ——
        这对 v0.1 是有效的自检（常量已由 ① 证明与 model 一致）。

        ## Phase 3 之后的重新表述

        迁移前的注释预测："Phase 3 把 IK 提升为 Core 引擎、几何改为从 model
        读取时，本条与上一条会**同时失败**。"

        **这个预测没有发生**，而且这是**有意**的：Phase 3 只提升了 FK
        （链式乘法对任何 Tree 模型都成立），IK 按设计留在包内做解析解
        （见本类 docstring 的"Phase 3 的结论"）。所以"常量几何"不是
        待修的过渡态，而是**终态**。

        ⇒ 于是这条测试的职责也变了：它不再是"等 Phase 3 来失败"的哨兵，
          而是**守住"常量 = model"这个不变式**。它和 ① 一起构成对
          "允许写死"的完整约束：

          ```text
          ①  写死的常量必须等于 model          ← 写死是对的
          ④  自检必须用写死的常量（而非 model）← 写死是刻意的
          ```

        第二条同样重要：如果哪天自检偷偷改成读 model，它就不再能
        发现"常量与 model 漂移"这类问题（因为两者用的是同一个源），
        ① 的防护也会随之失效。**自检的价值恰恰在于它的几何来源
        与 model 不同** —— 这才叫独立回代。
        """
        import dataclasses

        geom = _geometry(mini_arm_model)
        target = _at(0.12, geom)

        # 把整条臂的几何都改掉（所有关节原点 ×2）
        joints = []
        for j in mini_arm_model.joints:
            if j.origin.position.x > 0:
                new_origin = Transform(
                    Vector3(j.origin.position.x * 2, 0.0, j.origin.position.z),
                    j.origin.orientation,
                )
                j = dataclasses.replace(j, origin=new_origin)
            joints.append(j)
        doubled = dataclasses.replace(mini_arm_model, joints=joints)

        base = ik_mod.solve(mini_arm_model, target, branch="elbow_up")
        scaled = ik_mod.solve(doubled, target, branch="elbow_up")

        # 臂长翻倍后，同样的关节角显然到不了原目标 —— 但自检若用常量，
        # 它就**看不到**这件事，误差仍是 0。
        assert scaled.position_error < 1e-12, (
            "把臂长翻倍后位置自检误差变了 ⇒ 位置自检已改为读 model 几何。"
            "这不是 Phase 3 遗留的过渡态 —— 按设计，包内 IK 的自检"
            "**应当**用解析式常量几何（与 model 互为独立来源）。"
            "若你有意改成读 model，请先读本条与 ② 的说明，"
            "并确认 ① 的'常量 = model'防护不会因此失效。"
        )
