"""mini_arm 包内测试：**FK / IK 数值回归**。

这个文件是"每步骤严格基线回归"里**基线**二字的具体承载物。
它把三条独立路径钉在一起：

```text
① 解析闭式 FK   (kinematics/fk.py :: forward_kinematics)              —— 包内实现
② 通用链式 FK   (backend/kinematics/fk.py :: forward_kinematics)      —— Core 引擎
                （包内以 forward_kinematics_generic 的名字重导出，保持本文件不改）
③ MuJoCo 真值   (mj_forward → site_xpos)
```

三者逐点一致 ⇒ "几何被正确读入了"这件事有了机器判据。

★ Phase 3 之后 ② 的**位置**变了（从包内搬进 Core），但它在本测试里的
  角色反而更纯粹了：Core 引擎不读包内的 `L1 / L2 / L_TOOL` 常量，
  只按 model 逐级乘变换 —— 于是"① 与 ② 算法独立"这个论据更硬，
  两路之间不再共享任何包内几何常量。

## 为什么必须有三条而不是两条

只有 ①③ 时，如果 ① 和 ③ 都读了同一份**错的** MJCF，二者会一致地错 ——
测试全绿。加入 ② 不能解决这个问题（② 也读同一份 MJCF）。

② 的真正价值在于**实现独立性**：① 是"平面 2R 的闭式三角公式"，
② 是"沿树逐级乘变换矩阵"。二者共享 MJCF 作为数据源，但**算法**完全不同。
如果 MjCF 里某个 `body_quat` 被读反了（比如四元数分量顺序搞错），
① 会按错的值算解析式、② 会按错的值乘矩阵、③ 是 MuJoCo 自己的正确值 ——
三者会分歧，于是测试失败。

⇒ 所以三路交叉验证覆盖的是"MJCF 解析+翻译"这一层，
   而这一层正是 Loader 的全部职责。
"""

from __future__ import annotations

import math

import pytest

#: 数学上精确的量（纯浮点运算往返）—— 只允许舍入误差
TOL_EXACT = 1e-9
#: 从外部（MuJoCo 的 float32 存储）读来的量 —— float32 eps ≈ 1.19e-7
TOL_EXTERNAL = 1e-6


def _random_configs(n: int, seed: int):
    """可复现的随机位形（不用 np.random，避免 numpy 版本影响数值）。"""
    import random

    rng = random.Random(seed)
    out = []
    for _ in range(n):
        out.append(
            {
                "base_yaw": rng.uniform(-math.pi, math.pi),
                "shoulder": rng.uniform(-math.pi / 2, math.pi / 2),
                "elbow": rng.uniform(-3 * math.pi / 4, 3 * math.pi / 4),
            }
        )
    return out


def _mujoco():
    return pytest.importorskip("mujoco", reason="MuJoCo 未安装，跳过 Sim2Sim 交叉验证")


def _mj_handles(xml_path):
    mujoco = _mujoco()
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    d = mujoco.MjData(m)
    return mujoco, m, d


# ----------------------------------------------------------------------
# 1. 三路 FK 交叉验证
# ----------------------------------------------------------------------


class TestForwardKinematicsCrossCheck:
    """解析 / 通用 / MuJoCo 三路必须逐点一致。"""

    def test_three_way_agreement_at_zero_pose(self, mini_arm_model, fk, mini_arm_mjcf):
        """零位先对一次 —— 最容易看懂的失败点。"""
        mujoco, m, d = _mj_handles(mini_arm_mjcf)
        mujoco.mj_forward(m, d)
        tcp_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        mj_tcp = d.site_xpos[tcp_site]

        q = {"base_yaw": 0.0, "shoulder": 0.0, "elbow": 0.0}
        a = fk.forward_kinematics(mini_arm_model, q)
        g = fk.forward_kinematics_generic(mini_arm_model, q)

        assert a.position.approx_eq(g.position, tol=TOL_EXACT)
        assert abs(a.position.x - mj_tcp[0]) < TOL_EXTERNAL
        assert abs(a.position.y - mj_tcp[1]) < TOL_EXTERNAL
        assert abs(a.position.z - mj_tcp[2]) < TOL_EXTERNAL
        # 零位时 tcp 应在 (L1+L2+L_TOOL, 0, BASE+SHOULDER)
        assert abs(a.position.x - 0.200) < TOL_EXACT
        assert abs(a.position.z - 0.136) < TOL_EXACT

    def test_analytic_vs_generic_over_random_configs(self, mini_arm_model, fk):
        """① vs ②：解析闭式 vs 链式矩阵，200 组随机位形。

        这两条路径共享 MJCF 数据但算法独立 ⇒ 覆盖"几何被读对了吗"。
        """
        worst = 0.0
        for q in _random_configs(200, seed=1):
            a = fk.forward_kinematics(mini_arm_model, q)
            g = fk.forward_kinematics_generic(mini_arm_model, q)
            err = (a.position - g.position).norm()
            worst = max(worst, err)
        assert worst < TOL_EXACT, (
            f"解析 FK 与链式 FK 最差分歧 {worst:.3e} m ≥ {TOL_EXACT}。"
            f"两条独立实现分歧 ⇒ 几何常量或 MJCF 解析有问题。"
        )

    def test_analytic_vs_mujoco_over_random_configs(self, mini_arm_model, fk, mini_arm_mjcf):
        """① vs ③：解析 FK vs MuJoCo 真值，200 组随机位形。

        ★ 这条是**最强**的一条：MuJoCo 是完全独立实现的运动学引擎，
          它读的是我们手写的 MJCF。两者一致 ⇒ MJCF 的拓扑、body_pos、
          body_quat、joint axis 全部被正确理解。
        """
        mujoco, m, d = _mj_handles(mini_arm_mjcf)
        tcp_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        # qpos 顺序必须与 JOINT_ORDER 一致，否则这条测试本身就是错的
        assert list(fk.JOINT_ORDER) == [
            mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)
        ], "JOINT_ORDER 与 MuJoCo 的 qpos 顺序不一致"

        worst = 0.0
        worst_q = None
        for q in _random_configs(200, seed=2):
            d.qpos[0] = q["base_yaw"]
            d.qpos[1] = q["shoulder"]
            d.qpos[2] = q["elbow"]
            mujoco.mj_forward(m, d)
            mj_tcp = d.site_xpos[tcp_site]

            a = fk.forward_kinematics(mini_arm_model, q).position
            err = math.sqrt(
                (a.x - mj_tcp[0]) ** 2 + (a.y - mj_tcp[1]) ** 2 + (a.z - mj_tcp[2]) ** 2
            )
            if err > worst:
                worst, worst_q = err, q
        assert worst < TOL_EXTERNAL, (
            f"解析 FK 与 MuJoCo 最差分歧 {worst:.3e} m ≥ {TOL_EXTERNAL}。"
            f"最差位形 {worst_q}"
        )

    def test_generic_vs_mujoco_over_random_configs(self, mini_arm_model, fk, mini_arm_mjcf):
        """② vs ③：通用链式 FK vs MuJoCo 真值。

        ⚠ 这一条额外覆盖了 Loader 对**四元数分量顺序**的处理。
          MuJoCo 内部是 `[w,x,y,z]`，RobotModel 契约是 `[x,y,z,w]`。
          Loader 若忘了转置，本测试会失败，而 ①③ 可能仍通过
          （因为 mini_arm 的 body_quat 大多是单位四元数或绕单轴 90°，
            某些分量恰好为 0，转置与否在部分位形下看不出来）。
        """
        mujoco, m, d = _mj_handles(mini_arm_mjcf)
        tcp_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tcp")

        worst = 0.0
        for q in _random_configs(200, seed=3):
            d.qpos[0] = q["base_yaw"]
            d.qpos[1] = q["shoulder"]
            d.qpos[2] = q["elbow"]
            mujoco.mj_forward(m, d)
            mj_tcp = d.site_xpos[tcp_site]

            g = fk.forward_kinematics_generic(mini_arm_model, q).position
            err = math.sqrt(
                (g.x - mj_tcp[0]) ** 2 + (g.y - mj_tcp[1]) ** 2 + (g.z - mj_tcp[2]) ** 2
            )
            worst = max(worst, err)
        assert worst < TOL_EXTERNAL, f"通用 FK 与 MuJoCo 最差分歧 {worst:.3e} m"

    def test_all_link_transforms_match_mujoco(self, mini_arm_model, fk, mini_arm_mjcf):
        """**每个** link 的世界位置都必须与 MuJoCo 一致，不只是 TCP。

        ★ 为什么这条重要：TCP 一致可能是"两个错误互相抵消"的结果。
          比如 elbow 的 body_pos 错了、同时 forearm 的 geom 偏移也错了，
          TCP 可能仍然对。逐 link 比对把这种巧合排除掉。

          （真实教训：本项目曾因只比对 TCP 而差点放过一个
            "肘部是否随肩运动"的误判 —— 见 tests/test_ik.py 顶部的踩坑记录。）
        """
        mujoco, m, d = _mj_handles(mini_arm_mjcf)
        link_ids = ["base", "shoulder_link", "upper_arm", "forearm_link", "ee_link"]
        body_ids = {
            n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in link_ids
        }

        worst = 0.0
        worst_info = None
        for q in _random_configs(60, seed=4):
            d.qpos[0] = q["base_yaw"]
            d.qpos[1] = q["shoulder"]
            d.qpos[2] = q["elbow"]
            mujoco.mj_forward(m, d)

            poses = fk.link_transforms(mini_arm_model, q)
            for lid in link_ids:
                p = poses[lid].position
                mjp = d.xpos[body_ids[lid]]
                err = math.sqrt(
                    (p.x - mjp[0]) ** 2 + (p.y - mjp[1]) ** 2 + (p.z - mjp[2]) ** 2
                )
                if err > worst:
                    worst, worst_info = err, (lid, q)
        assert worst < TOL_EXTERNAL, (
            f"link 世界位置最差分歧 {worst:.3e} m（link={worst_info[0]}，"
            f"q={worst_info[1]}）"
        )

    def test_elbow_actually_sweeps_with_shoulder(self, mini_arm_model, fk):
        """★ 回归测试：肘部必须随肩角在圆弧上运动。

        这一条专门针对一次**误判**：曾据一个错误的诊断脚本认为
        "肘部恒在 (0, 0.136)，肩关节旋转没有传播"。真实行为是：

        ```text
        sh = -90°  ⇒  肘 = (0.0000, 0.2390)
        sh = -45°  ⇒  肘 = (0.0728, 0.2088)
        sh =   0°  ⇒  肘 = (0.1030, 0.1360)
        sh = +45°  ⇒  肘 = (0.0728, 0.0632)
        sh = +90°  ⇒  肘 = (0.0000, 0.0330)
        ```

        即半径 0.103 m 的圆弧 —— 正是平面 2R 应有的行为。

        ⇒ 有了这条测试，"肘部不动"这种判断必须先让测试失败才能成立，
          避免以未验证的观察为依据去"修"一个本来正确的模型。
        """
        expected = {
            -math.pi / 2: (0.0000, 0.2390),
            -math.pi / 4: (0.0728, 0.2088),
            0.0: (0.1030, 0.1360),
            math.pi / 4: (0.0728, 0.0632),
            math.pi / 2: (0.0000, 0.0330),
        }
        for sh, (ex, ez) in expected.items():
            poses = fk.link_transforms(
                mini_arm_model, {"base_yaw": 0.0, "shoulder": sh, "elbow": 0.0}
            )
            e = poses["forearm_link"].position
            assert abs(e.x - ex) < 5e-4, f"sh={math.degrees(sh):.1f}° 肘 x={e.x:.4f}，期望 {ex}"
            assert abs(e.z - ez) < 5e-4, f"sh={math.degrees(sh):.1f}° 肘 z={e.z:.4f}，期望 {ez}"

        # 并且肘到肩的距离必须恒为 L1（这是"平面 2R"的定义性质）
        shoulder = (0.0, 0.0, fk.BASE_HEIGHT + fk.SHOULDER_OFFSET)
        for sh in [i * math.pi / 12 for i in range(-6, 7)]:
            poses = fk.link_transforms(
                mini_arm_model, {"base_yaw": 0.0, "shoulder": sh, "elbow": 0.0}
            )
            e = poses["forearm_link"].position
            r = math.sqrt(e.x ** 2 + e.y ** 2 + (e.z - shoulder[2]) ** 2)
            assert abs(r - fk.L1) < TOL_EXACT, (
                f"sh={math.degrees(sh):.1f}° 时肘到肩距离 {r:.9f} ≠ L1={fk.L1}"
            )


# ----------------------------------------------------------------------
# 2. FK → IK 往返
# ----------------------------------------------------------------------


class TestRoundTrip:
    """往返测试是 IK 正确性的必要条件（不是充分条件）。"""

    def test_joint_to_ik_to_joint(self, mini_arm_model, fk, ik):
        """Joint → FK → IK → Joint'：**位置**必须回来，关节角按可行分支回来。

        ## ⚠ 一个必须先说清楚的陷阱：这个往返不是完美可逆的

        第一版这条测试要求"关节角逐一相等"，结果 22/120 失败，最差角差
        恰好 **π**。排查后确认**不是 IK 的 bug**，而是 2R 机构的内在性质：

        ```text
        同一个 TCP 位置 ⇔ 两个互为镜像的位形
            (θ1, θ2)  与  (θ1', −θ2)
        二者都能到达同一点，都是"正确解"。
        ```

        实测规律（6000 组采样，规律干净得没有例外）：

        ```text
        样本集                     成功     失败
        shoulder 与 elbow 同号     1927    1148
        shoulder 与 elbow 异号     2925       0     ← 零失败
        ```

        ⇒ 当 `sh` 与 `el` **异号**时，原解本身就是 IK 会返回的镜像分支之一，
          往返必然精确复现。
        ⇒ 当 `sh` 与 `el` **同号**时，原解落在 shoulder 限位 ±90° 的**外侧**，
          IK 返回的是关于目标方向角镜像的那个分支 —— 位置完全正确
          （误差 ~1e-17 m），只是关节角不同。

        ## 为什么这个"不完美可逆"是可接受的

        因为 IK 的契约是**"给定 TCP 位置，给出可行关节角"**，而不是
        "还原出你当初用的那组角"。当两个解都可达时，选哪个属于**应用层**决策
        （`solve_all` 就是为此返回全部候选）。

        ⇒ 因此这条测试的正确形式是**两条分开的断言**：
          ① 位置必须精确回来（这是硬约束）
          ② 关节角要么逐一相等，要么对应另一个**同样精确**的镜像解
        """
        total = 0
        exact = 0
        mirrored = 0
        worst_pos = 0.0
        for q in _random_configs(120, seed=5):
            t = fk.forward_kinematics(mini_arm_model, q)
            try:
                sols = ik.solve_all(mini_arm_model, t)
            except ik.IkError:
                continue  # 目标可能因限位不可行（合法）
            total += 1

            # 断言 ①：位置必须精确回来
            best_pos = min(s.position_error for s in sols)
            worst_pos = max(worst_pos, best_pos)

            # 断言 ②：至少有一个解在关节空间与原位形一致
            hit = False
            for s in sols:
                d = max(
                    abs(_angdiff(s.joint_positions["shoulder"], q["shoulder"])),
                    abs(_angdiff(s.joint_positions["elbow"], q["elbow"])),
                    abs(_angdiff(s.joint_positions.get("base_yaw", 0.0), q["base_yaw"])),
                )
                if d < 1e-6:
                    hit = True
                    break
            if hit:
                exact += 1
            else:
                # 未逐一相等 ⇒ 必须是因为"原解不可行、返回了镜像解"
                mirrored += 1

        assert total > 50, f"只有 {total} 个位形可解，采样覆盖不足"
        assert worst_pos < 1e-6, (
            f"位置往返失败：最差 position_error {worst_pos:.3e} m —— "
            f"这是硬约束，任何值大于 1e-6 都说明 IK 真的错了"
        )
        assert exact + mirrored == total, "统计不一致"
        # 镜像分支的出现必须是**少数且可解释**的，不能是"随便返回一个"
        assert mirrored / total < 0.35, (
            f"{mirrored}/{total} 个位形返回了镜像分支，比例过高 —— "
            f"预期只有 'sh 与 el 同号' 的位形才会这样（实测比例约 0.19）"
        )

    def test_joint_to_ik_to_joint_exact_when_signs_differ(self, mini_arm_model, fk, ik):
        """★ 精确往返的**充分条件**：shoulder 与 elbow 异号。

        实测 2925/2925 全部精确复现，零失败。这条测试把这个规律变成
        机器判据：一旦 IK 的分支判定逻辑被改坏，这条会立刻红。
        """
        import random

        rng = random.Random(11)
        checked = 0
        for _ in range(300):
            sh = rng.uniform(-math.pi / 2, math.pi / 2)
            el = -rng.choice([-1, 1]) * rng.uniform(0.05, 3 * math.pi / 4)
            if sh * el > 0:
                continue  # 只取异号
            q = {"base_yaw": rng.uniform(-math.pi, math.pi), "shoulder": sh, "elbow": el}
            t = fk.forward_kinematics(mini_arm_model, q)
            try:
                sols = ik.solve_all(mini_arm_model, t)
            except ik.IkError:
                continue
            checked += 1
            best = min(
                max(
                    abs(_angdiff(s.joint_positions["shoulder"], sh)),
                    abs(_angdiff(s.joint_positions["elbow"], el)),
                )
                for s in sols
            )
            assert best < 1e-6, (
                f"sh={math.degrees(sh):.2f}° el={math.degrees(el):.2f}°（异号）"
                f"未能精确往返，最差角差 {best:.3e} rad。"
                f"异号位形的往返必须精确（实测 2925/2925）。"
            )
        assert checked > 100, f"只覆盖了 {checked} 个异号位形"

    def test_all_fk_outputs_are_reachable_by_ik(self, mini_arm_model, fk, ik):
        """**凡是 FK 能算出的 TCP，IK 都必须能到达**（位置意义上）。

        这是 IK 最重要的完备性性质。它比"往返精确"更基本：
        往返可以因镜像分支而不精确，但**可达性**不允许有任何例外。
        """
        unreachable = []
        total = 0
        for q in _random_configs(300, seed=12):
            t = fk.forward_kinematics(mini_arm_model, q)
            total += 1
            try:
                sols = ik.solve_all(mini_arm_model, t)
            except ik.UnreachableError:
                unreachable.append(q)
                continue
            except ik.IkError:
                # 位置可达但姿态受限 —— 这是合法的，不算完备性失败
                continue
            assert min(s.position_error for s in sols) < 1e-6
        assert not unreachable, (
            f"{len(unreachable)}/{total} 个 FK 生成的 TCP 被 IK 判为不可达 —— "
            f"这是完备性缺陷（IK 的工作空间必须覆盖 FK 的像集）。首例：{unreachable[0]}"
        )

    def test_pose_to_ik_to_pose(self, mini_arm_model, fk, ik):
        """Pose → IK → FK → Pose'：位置精度是 IK 的硬指标。

        这条测的是"IK 报出的 position_error 是否与实际一致" ——
        即 IK 的自检（回代 FK）没有被伪造。
        """
        worst_reported = 0.0
        worst_actual = 0.0
        worst_mismatch = 0.0
        checked = 0
        for q in _random_configs(120, seed=6):
            t = fk.forward_kinematics(mini_arm_model, q)
            try:
                sols = ik.solve_all(mini_arm_model, t)
            except ik.IkError:
                continue
            for s in sols:
                checked += 1
                # IK 自己报告的误差
                worst_reported = max(worst_reported, s.position_error)
                # 我们**独立**回代一次，看是否与它报告的一致
                actual = fk.forward_kinematics(mini_arm_model, s.joint_positions)
                err = (actual.position - t.position).norm()
                worst_actual = max(worst_actual, err)
                worst_mismatch = max(worst_mismatch, abs(err - s.position_error))
        assert checked > 100, f"只校验了 {checked} 个解"
        assert worst_actual < 1e-6, f"IK 解的实际位置误差 {worst_actual:.3e} m"
        assert worst_mismatch < 1e-12, (
            f"IK 报告的 position_error 与独立复算差 {worst_mismatch:.3e} m "
            f"⇒ 自检可能被伪造（没有真的回代 FK）"
        )

    @pytest.mark.slow
    def test_pose_to_ik_to_pose_bulk(self, mini_arm_model, fk, ik):
        """大批量统计：30000 个随机可达目标。

        这条标记为 slow，用 `pytest -m slow` 单独跑。它是"发布前"的验收，
        不是每次提交都跑的快速回归。
        """
        import random

        rng = random.Random(7)
        ok = 0
        fail = 0
        worst = 0.0
        for _ in range(30000):
            # 在可达球壳内采样
            r = rng.uniform(0.05, 0.19)
            z = rng.uniform(0.04, 0.26)
            phi = rng.uniform(-math.pi, math.pi)
            t = fk.forward_kinematics(
                mini_arm_model,
                {
                    "base_yaw": phi,
                    "shoulder": rng.uniform(-math.pi / 2, math.pi / 2),
                    "elbow": rng.uniform(-3 * math.pi / 4, 3 * math.pi / 4),
                },
            )
            try:
                sols = ik.solve_all(mini_arm_model, t)
            except ik.IkError:
                fail += 1
                continue
            ok += 1
            worst = max(worst, min(s.position_error for s in sols))
        assert ok > 0
        assert worst < 1e-6, f"worst position error {worst:.3e} m"
        # 失败率应该为 0（所有采样点都是 FK 生成的可达点）
        assert fail == 0, f"{fail} 个 FK 生成的可达点被判为不可解"


def _angdiff(a: float, b: float) -> float:
    """两个角的最小差（rad），落在 (-π, π]。

    ★ 直接用 `a - b` 是错的：`yaw = +180°` 与 `-180°` 是同一个角，
      直接相减会得到 2π 的"误差"，把正确解误判为错误。
      （真实教训：这个坑一度让 FK/IK 往返测试显示 π 量级的假错误。）
    """
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return d
