"""mini_arm 包内测试：**事实断言**（geometric facts）。

## 这个文件存在的理由

`tests/`（仓库顶层）测的是**架构契约** —— Loader 有没有正确翻译、
RobotModel 是否合法、坐标/单位有没有被偷偷转换。
本文件测的是**这个型号自己的几何事实** —— 常量对不对、轴选对没有、
tcp 有没有选成 flange。

二者必须分开，因为它们的"失败含义"不同：

```text
顶层测试失败 ⇒ 架构坏了（所有机器人都受影响）
包内测试失败 ⇒ 只是 mini_arm 这个型号的描述错了（架构是好的）
```

混在一起会让"换个型号"这件事变得不敢做 —— 而 RobotForge 的全部意义
就是让换型号变成数据变更而不是代码变更。

## 一条重要方法学（本文件里有一半测试是为它写的）

**判据本身也可能是错的。** 本项目的 `elbow_up/elbow_down` 语义曾被
一个错误的诊断判据误报为"50% 标反"，排查的终点是发现**诊断脚本**错了而非
IK 错了。因此本文件里所有几何判据都写成"**与 yaw 无关的平面量**"，
并额外用一条 `test_branch_semantics_is_yaw_invariant` 显式断言这一性质。

详见 `tests/test_ik.py` 顶部的"踩坑记录"，以及
`docs/robot-model.md` 关于"判据必须与被验证对象处在同一个坐标系"的说明。
"""

from __future__ import annotations

import math

import pytest

from backend.model.types import Quaternion, Transform, Vector3

# ⚠ 这里**不能**写 `from .conftest import ...`：
#   `packages/mini_arm/tests/` 不是 Python package（无 `__init__.py`，
#   且 `packages/` 整体刻意不是 package），相对 import 会抛
#   `ImportError: attempted relative import with no known parent package`。
#
#   conftest.py 里的模块加载器通过 **fixture** 暴露（见 `fk` / `ik`），
#   这是 pytest 在"非 package 目录"里共享工具的惯用做法。

# ----------------------------------------------------------------------
# 1. 常量与 MJCF 的一致性
# ----------------------------------------------------------------------


class TestConstants:
    """几何常量必须与 MJCF 逐项一致。

    这些常量是"闭式解析解"的前提。如果有人改了 MJCF 的连杆长度而忘了改
    这里，解析 FK 会开始给出错误结果，而链式 FK 仍然正确 ——
    两条路径**分歧**，正是 `test_fk_cross_check_generic_engine` 的失败条件。
    """

    def test_link_lengths_match_mjcf_bodies(self, mini_arm_model, fk):
        """L1 / L2 必须等于从 RobotModel 读出的关节原点间距。"""
        m = mini_arm_model

        # shoulder → elbow 的距离就是 L1。
        # 注意：joint.origin 是**相对父 link** 的位姿，所以 shoulder 的
        # origin 是 (0,0,SHOULDER_OFFSET) 而不是原点；elbow 的 origin 才是
        # (L1,0,0)。断言要按各自的语义写，不能一律要求为零。
        elbow_origin = m.joint("elbow").origin.position
        assert abs(fk.L1 - elbow_origin.x) < 1e-12, (
            f"L1={fk.L1} 与 MJCF 的 elbow 原点 x={elbow_origin.x} 不一致"
        )
        assert abs(elbow_origin.y) < 1e-12 and abs(elbow_origin.z) < 1e-12, (
            "elbow 原点应有纯 +X 偏移（连杆沿 +X 伸出）"
        )

        shoulder_origin = m.joint("shoulder").origin.position
        assert abs(shoulder_origin.x) < 1e-12 and abs(shoulder_origin.y) < 1e-12, (
            "shoulder 原点应无水平偏移（在立柱正上方）"
        )

        # base → base_yaw 与 base_yaw → shoulder 的高度
        assert abs(fk.BASE_HEIGHT - m.joint("base_yaw").origin.position.z) < 1e-12
        assert abs(fk.SHOULDER_OFFSET - shoulder_origin.z) < 1e-12

    def test_tool_offset_matches_tcp_site(self, mini_arm_model, fk):
        """L_TOOL 必须等于 tcp site 相对 ee_link 的偏移。

        ⚠ 这是最容易"静默漂移"的常量：改了 site 的 pos 而没改 L_TOOL，
        解析 FK 的误差恰好是一个常数 32 mm 量级的偏移 —— 看起来像标定误差。
        """
        tcp = mini_arm_model.site("tcp").transform.position
        assert abs(fk.L_TOOL - tcp.x) < 1e-12
        assert abs(tcp.y) < 1e-12 and abs(tcp.z) < 1e-12, (
            "tcp site 有非 X 分量偏移 ⇒ 解析 FK 的 L2+L_TOOL 简化不再成立"
        )

    def test_joint_order_matches_model(self, mini_arm_model, fk):
        """JOINT_ORDER 必须与 RobotModel 的可动关节顺序一致。

        这个顺序**同时**是 MuJoCo 的 qpos 顺序 —— 它是 Python 侧的
        `{joint_id: value}` 与 MuJoCo 侧 `qpos` 数组之间的唯一桥梁。
        顺序错了，Sim2Sim 会把 shoulder 的值喂给 elbow。
        """
        assert tuple(mini_arm_model.mobile_joint_ids()) == tuple(fk.JOINT_ORDER)


# ----------------------------------------------------------------------
# 2. 轴向：本型号最易配错的一处
# ----------------------------------------------------------------------


class TestJointAxes:
    """肩/肘必须是绕 +Y，不是绕 X。

    绕 Y ⇒ 连杆在 XZ 平面内摆动（前后上下）—— 平面 2R 需要的平面。
    绕 X ⇒ 连杆在 YZ 平面内摆动（左右）—— 工作空间与 FK/IK 推导不符，
            但**不会报错**，只会让机械臂朝错误方向动。

    这类错误不会产生 NaN、不会触发断言，只能靠显式断言轴向量拦住。
    """

    def test_base_yaw_is_z_axis(self, mini_arm_model):
        axis = mini_arm_model.joint("base_yaw").axis
        assert axis.approx_eq(Vector3(0.0, 0.0, 1.0), tol=1e-12), (
            f"base_yaw 轴应为 +Z，实际 {axis}"
        )

    def test_shoulder_and_elbow_are_y_axis(self, mini_arm_model):
        for jid in ("shoulder", "elbow"):
            axis = mini_arm_model.joint(jid).axis
            assert axis.approx_eq(Vector3(0.0, 1.0, 0.0), tol=1e-12), (
                f"{jid} 轴应为 +Y（使连杆在 XZ 平面内摆动），实际 {axis}。"
                f"若为 +X，机械臂会在 YZ 平面内左右摆动，与平面 2R 推导不符。"
            )

    def test_axes_are_orthogonal_to_link_direction(self, mini_arm_model):
        """肩/肘轴必须垂直于上臂方向（+X），否则不是纯俯仰。"""
        for jid in ("shoulder", "elbow"):
            axis = mini_arm_model.joint(jid).axis
            assert abs(axis.dot(Vector3(1.0, 0.0, 0.0))) < 1e-12


# ----------------------------------------------------------------------
# 3. EndEffector 的选择：tcp 而不是 flange
# ----------------------------------------------------------------------


class TestEndEffectorSelection:
    """必须选中 `tcp`，不能是 `ee_frame_site`。

    二者都是 site，都在 ee_link 上，名字都"像"末端 —— 但语义差 32 mm：

    ```text
    ee_frame_site  = 法兰面（ee_link 原点）
    tcp            = 工具中心点（法兰 + 工具长度）
    ```

    选错不会报错，只会让所有 IK 目标整体偏移 32 mm。
    这是"看起来像标定误差"的错误里最贵的一种。
    """

    def test_default_end_effector_uses_tcp_site(self, mini_arm_model):
        ee = mini_arm_model.default_end_effector()
        assert ee is not None, "模型没有 end_effector"
        assert ee.site == "tcp", (
            f"end_effector 用了 site={ee.site!r}，应为 'tcp'。"
            f"若选到 'ee_frame_site'，所有目标会整体差 L_TOOL=0.032 m。"
        )

    def test_ee_frame_is_registered(self, mini_arm_model):
        """EndEffector 引用的 frame 必须真的在 frames 注册表里。

        踩坑记录：第一版 Loader 造了 `EndEffector.frame` 的 id 却没把它
        加进 `frames` ⇒ validator 报 "引用了不存在的 frame"。
        契约是"引用的东西必须在注册表里"，两边都要维护。
        """
        ee = mini_arm_model.default_end_effector()
        assert ee.frame in mini_arm_model.frame_ids()

    def test_tcp_is_distinct_from_ee_frame(self, mini_arm_model):
        """tcp 与 ee_frame 必须不是同一个点 —— 否则这条测试没有意义。"""
        tcp = mini_arm_model.site("tcp").transform.position
        ee_frame = mini_arm_model.site("ee_frame_site").transform.position
        gap = (tcp - ee_frame).norm()
        assert abs(gap - 0.032) < 1e-12, (
            f"tcp 与 ee_frame_site 距离 {gap} m，期望 0.032 m"
        )


# ----------------------------------------------------------------------
# 4. 关节限位
# ----------------------------------------------------------------------


class TestJointLimits:
    def test_limits_match_mjcf_ranges(self, mini_arm_model):
        """限位必须从 MJCF 的 range 原样读入，且是 rad。

        ⚠ 双重转换陷阱：MJCF 的 `compiler angle="radian"` 让 MuJoCo 在
        **编译期**就完成了 deg→rad 转换，所以 `jnt_range` 读出来已经是 rad。
        如果 Loader 再乘一次 π/180，限位会缩小 57 倍 —— 而机械臂看起来
        "能动，只是范围很小"，不会报错。
        """
        expected = {
            "base_yaw": (-math.pi, math.pi),
            "shoulder": (-math.pi / 2, math.pi / 2),
            "elbow": (-3 * math.pi / 4, 3 * math.pi / 4),
        }
        for jid, (lo, hi) in expected.items():
            lim = mini_arm_model.joint(jid).limits
            assert lim.has_position_bounds, f"{jid} 没有位置限位"
            assert lim.position_min is not None and lim.position_max is not None
            assert abs(lim.position_min - lo) < 1e-9, (
                f"{jid} 下限 {lim.position_min} ≠ {lo}"
            )
            assert abs(lim.position_max - hi) < 1e-9, (
                f"{jid} 上限 {lim.position_max} ≠ {hi}"
            )

    def test_limits_are_in_radians_not_degrees(self, mini_arm_model):
        """显式断言"不是角度制"。"""
        lim = mini_arm_model.joint("shoulder").limits
        assert lim.position_max is not None
        assert lim.position_max < 10.0, (
            f"shoulder 上限 {lim.position_max} 看起来是角度制（≈90）"
            f"而不是弧度制（≈1.5708）—— 可能发生了重复的 deg→rad 转换"
        )


# ----------------------------------------------------------------------
# 5. 可达范围
# ----------------------------------------------------------------------


class TestReach:
    def test_reach_limits_from_link_lengths(self, ik):
        """可达半径 = |L1 - L2eff| .. L1 + L2eff。"""
        r_min, r_max = ik.reach_limits()
        assert abs(r_max - (0.103 + 0.097)) < 1e-12
        assert abs(r_min - abs(0.103 - 0.097)) < 1e-12

    def test_out_of_reach_raises(self, mini_arm_model, ik):
        """超出可达范围必须抛 UnreachableError，而不是返回夹紧解。"""
        with pytest.raises(ik.UnreachableError):
            ik.solve(mini_arm_model, Transform(Vector3(0.5, 0.0, 0.136), Quaternion.identity()))

    def test_dead_zone_raises(self, mini_arm_model, ik):
        """死区球内（比 |L1-L2eff| 还近）也必须抛。"""
        with pytest.raises(ik.UnreachableError):
            ik.solve(mini_arm_model, Transform(Vector3(0.001, 0.0, 0.136), Quaternion.identity()))


# ----------------------------------------------------------------------
# 6. 分支语义 —— 本文件最重要的一组
# ----------------------------------------------------------------------


class TestBranchSemantics:
    """`elbow_up` / `elbow_down` 必须与 yaw 无关。

    ## 为什么这组测试写成"竖直高度差"

    第一版诊断判据用了 `(tcp - shoulder)[[0, 2]] × (elbow - shoulder)[[0, 2]]`
    —— 即**世界 XZ 平面**里的二维叉积。当 `base_yaw = 0` 时它是对的；
    但 `yaw = 131°` 时，臂的平面本身已经绕 Z 转过了，把三维向量直接投影到
    世界 XZ 再做叉积，得到的是**另一个量**，与"在臂的自身平面里看肘在上还是下"
    不是一回事。

    结果：正确实现的 IK 被误报为"约一半分支标反"，排查方向完全跑偏。

    ⇒ 正确判据：**肘的世界 Z** 与 **肩→TCP 弦**在肘处的高度比较。
      "up/down" 本来就是竖直方向的语义，这是唯一与 yaw 无关的判据。
    """

    @staticmethod
    def _elbow_above_chord(model, q, fk):
        """肘高出 肩→TCP 弦 的量（m）。正 = up。与 yaw 无关。"""
        poses = fk.link_transforms(model, {
            "base_yaw": q[0], "shoulder": q[1], "elbow": q[2],
        })
        shoulder_world = Vector3(0.0, 0.0, fk.BASE_HEIGHT + fk.SHOULDER_OFFSET)
        elbow_world = poses["forearm_link"].position
        ee = model.default_end_effector()
        tcp_world = poses[model.site(ee.site).parent].compose(
            model.site(ee.site).transform
        ).position

        chord = tcp_world - shoulder_world
        chord_len2 = chord.dot(chord)
        if chord_len2 < 1e-18:
            return 0.0
        t = (elbow_world - shoulder_world).dot(chord) / chord_len2
        nearest = shoulder_world + chord * t
        return elbow_world.z - nearest.z

    def test_branch_matches_vertical_geometry(self, mini_arm_model, fk, ik):
        """对所有可达目标，branch 标签必须与竖直几何一致。"""
        checked = 0
        for i in range(60):
            for j in range(6):
                # 在球壳内规则采样，避免随机不可复现
                r = 0.05 + 0.14 * (i / 59.0)
                z = 0.136 + 0.16 * math.sin(2 * math.pi * j / 6.0)
                x, y = r, 0.0
                try:
                    sols = ik.solve_all(
                        mini_arm_model, Transform(Vector3(x, y, z), Quaternion.identity())
                    )
                except ik.IkError:
                    continue
                for sol in sols:
                    if sol.branch == "degenerate":
                        continue
                    q = [
                        sol.joint_positions.get("base_yaw", 0.0),
                        sol.joint_positions["shoulder"],
                        sol.joint_positions["elbow"],
                    ]
                    dz = self._elbow_above_chord(mini_arm_model, q, fk)
                    if abs(dz) < 1e-9:
                        continue
                    expected = "elbow_up" if dz > 0 else "elbow_down"
                    assert sol.branch == expected, (
                        f"目标 ({x:.4f},{y:.4f},{z:.4f}) 的 branch={sol.branch!r}，"
                        f"但肘高出弦 {dz * 1000:+.4f} mm ⇒ 应为 {expected!r}"
                    )
                    checked += 1
        assert checked > 100, f"只校验了 {checked} 个解，采样覆盖不足"

    def test_branch_semantics_is_yaw_invariant(self, mini_arm_model, fk, ik):
        """同一 (r, z)、不同 yaw 必须得到同样的分支标签。

        ★ 这一条是专门为"诊断判据用错坐标系"那个坑写的回归测试。
          判据若依赖 yaw，本测试立刻失败。
        """
        r, z = 0.13, 0.19
        for i in (0, 1, 2, 3, 4, 5, 6, 7):
            phi = 2 * math.pi * i / 8.0
            x, y = r * math.cos(phi), r * math.sin(phi)
            sols = ik.solve_all(
                mini_arm_model, Transform(Vector3(x, y, z), Quaternion.identity())
            )
            by_branch = {s.branch: s for s in sols}
            if "elbow_up" in by_branch:
                dz_up = self._elbow_above_chord(
                    mini_arm_model,
                    [by_branch["elbow_up"].joint_positions.get("base_yaw", 0.0),
                     by_branch["elbow_up"].joint_positions["shoulder"],
                     by_branch["elbow_up"].joint_positions["elbow"]],
                    fk,
                )
                assert dz_up > 0, (
                    f"yaw={math.degrees(phi):.1f}° 时 elbow_up 的肘低于弦 {dz_up * 1000:.4f} mm"
                    f" ⇒ 判据随 yaw 漂移了"
                )

    def test_two_branches_are_mirror_images(self, mini_arm_model, ik):
        """两分支的 shoulder 角关于目标方向角对称 ⇒ 肘一上一下。"""
        sols = ik.solve_all(
            mini_arm_model, Transform(Vector3(0.12, 0.0, 0.15), Quaternion.identity())
        )
        assert len(sols) == 2, f"期望 2 个解，实际 {len(sols)}"
        branches = {s.branch for s in sols}
        assert branches == {"elbow_up", "elbow_down"}
        # 上下两支的肘角符号必须相反
        up = next(s for s in sols if s.branch == "elbow_up")
        down = next(s for s in sols if s.branch == "elbow_down")
        assert up.joint_positions["elbow"] * down.joint_positions["elbow"] < 0


# ----------------------------------------------------------------------
# 7. 退化位形
# ----------------------------------------------------------------------


class TestDegenerate:
    """奇异位形的行为。

    ## 实测得到的事实（决定了这三条测试怎么写）

    在 `r = r_max`（完全伸展）附近扫描，`elbow` 的解是：

    ```text
      eps=r_max-r    |θ2|            两分支
      0              0.000000°       合并为 1 个，标注 [degenerate]
      1e-9           0.011464°       分开为 2 个
      1e-7           0.114643°       分开为 2 个
      1e-3           11.469115°      分开为 2 个
    ```

    ⇒ 只有**恰好** `d == r_max`（浮点意义上完全不小于）才判退化。
      `eps = 1e-9` 已经让 |θ2| = 0.011° ≫ `TOL_SINGULAR = 1e-6`（rad）⇒ 判非退化。

    这不是缺陷而是刻意的：`TOL_SINGULAR` 作为**角度阈值**（rad）时约等于
    0.0000573°，比 `eps = 1e-9 m` 引起的 0.011° 小两个数量级。
    换言之，"退化"的定义是"|θ2| 小到无法区分"，而不是"目标离边界很近" ——
    后者是可达性问题，由 `UnreachableError` 负责。

    ## 折叠位形（r = r_min）

    实测：两个分支都因 `elbow = ±180°` 超出 ±135° 限位而抛 `IkError`。
    这是**正确**行为 —— 位置可达但姿态不可达，必须报错而不是硬塞一个
    "差不多"的解。`clamp=True` 可以强制夹紧，但会带 `clamped` 标记。
    """

    def test_exact_full_extension_is_labelled_degenerate(self, mini_arm_model, ik):
        """恰好 r = r_max 时必须标注 degenerate，且只返回一个解。"""
        r_max = 0.103 + 0.097
        sols = ik.solve_all(
            mini_arm_model,
            Transform(Vector3(r_max, 0.0, 0.136), Quaternion.identity()),
        )
        assert len(sols) == 1, f"完全伸展应只有 1 个解（去重后），实际 {len(sols)}"
        assert "degenerate" in sols[0].branch, (
            f"完全伸展位形未标注 degenerate，实际 branch={sols[0].branch!r}"
        )

    def test_degenerate_solutions_are_deduplicated(self, mini_arm_model, ik):
        """退化时不能返回两份完全相同的解。"""
        r_max = 0.103 + 0.097
        sols = ik.solve_all(
            mini_arm_model,
            Transform(Vector3(r_max, 0.0, 0.136), Quaternion.identity()),
        )
        keys = [tuple(round(v, 9) for v in s.joint_positions.values()) for s in sols]
        assert len(keys) == len(set(keys)), f"存在重复解：{keys}"

    def test_near_extension_is_two_distinct_branches(self, mini_arm_model, ik):
        """离边界 1e-3 m 时必须是两个分开的分支。

        ★ 这条与上一条成对：它们一起把"退化"的门槛钉死在
          "恰好到达边界"这一点上，防止有人把 TOL_SINGULAR 改大之后
          把一大片正常位形误判为退化（那样 UI 会一直显示"奇异"）。
        """
        r = 0.103 + 0.097 - 1e-3
        sols = ik.solve_all(
            mini_arm_model, Transform(Vector3(r, 0.0, 0.136), Quaternion.identity())
        )
        assert len(sols) == 2, f"期望 2 个解，实际 {len(sols)}"
        assert {s.branch for s in sols} == {"elbow_up", "elbow_down"}

    def test_folded_configuration_violates_limits(self, mini_arm_model, ik):
        """完全折叠位形位置可达但姿态超限 ⇒ 必须抛 IkError。"""
        r_min = abs(0.103 - 0.097)
        with pytest.raises(ik.IkError):
            ik.solve_all(
                mini_arm_model,
                Transform(Vector3(r_min, 0.0, 0.136), Quaternion.identity()),
            )

    def test_folded_configuration_is_clampable(self, mini_arm_model, ik):
        """`clamp=True` 在**单分支** `solve()` 下允许强制夹紧，且诚实标记误差。

        ## 为什么这里用 `solve()` 而不是 `solve_all()`

        实测两者的分工是**刻意**的，二者对"可行"的定义不同：

        ```text
        solve(branch, clamp=True)  ⇒ 返回夹紧解，pos_err 0.0742 m，clamped=True
        solve_all(clamp=True)      ⇒ 抛 IkError("位置误差 0.074241 m 过大")
        ```

        这不是矛盾，而是两个接口各自的承诺：

        - `solve_all` 承诺"返回的解是可用的" ⇒ 74 mm 误差不算可用，
          必须抛错而不是把一个差 74 mm 的解混进候选集让上层去挑。
        - `solve` 承诺"按你指定的分支给一个解，并如实告诉你它有多准" ⇒
          调用方（可能在做避障搜索或轨迹插值）需要看到那个夹紧解本身。

        ⇒ 若把 `solve_all` 的 1e-3 容差当成"bug"去放宽，就会破坏它
          "候选集里没有坏解"这个承诺。这条测试把这个分工钉住。
        """
        r_min = abs(0.103 - 0.097)
        sol = ik.solve(
            mini_arm_model,
            Transform(Vector3(r_min, 0.0, 0.136), Quaternion.identity()),
            branch="elbow_up",
            clamp=True,
        )
        assert sol.clamped is True, "夹紧解必须标记 clamped=True"
        assert "elbow" in sol.clamped_joints, (
            f"应报告 elbow 被夹紧，实际 {sol.clamped_joints}"
        )
        # 关节角确实被夹到了限位
        assert abs(sol.joint_positions["elbow"] - 3 * math.pi / 4) < 1e-9
        # ★ 诚实性：误差必须是"实际回代 FK 算出来的"，不是 0
        assert sol.position_error > 0.05, (
            f"折叠位形夹紧后误差应约 0.074 m（夹到 ±135° 的后果），"
            f"实际 {sol.position_error} —— 若为 0 说明没有真的回代 FK 自检"
        )

    def test_solve_all_rejects_clamped_solutions_beyond_tolerance(
        self, mini_arm_model, ik
    ):
        """`solve_all` 必须拒绝误差超过 1e-3 m 的夹紧解。

        `solve_all` 的承诺是"候选集里没有坏解"。放宽这个容差会让 UI
        显示一个差 74 mm 的"可达解"，而用户看到的是机械臂停在错误位置。
        """
        r_min = abs(0.103 - 0.097)
        with pytest.raises(ik.IkError) as ei:
            ik.solve_all(
                mini_arm_model,
                Transform(Vector3(r_min, 0.0, 0.136), Quaternion.identity()),
                clamp=True,
            )
        assert "过大" in str(ei.value), (
            f"期望因位置误差过大而拒绝，实际报错：{ei.value}"
        )
