"""MuJoCo 仿真后端测试（提示词 §50 / §62）。

## 这组测试在防什么

Phase 5 的失败模式和 Phase 4 完全不同。Phase 4 是"闭环可能空转"，
Phase 5 是**"引擎接对了但语义错了，而所有数字看起来都很合理"**：

| 风险 | 表现 | 本文件的对策 |
|---|---|---|
| 四元数顺序反了 | 位置全对、姿态全错；往返测试仍通过 | `TestQuaternionBoundary`：非对称四元数 + 方向钉住 |
| 关节顺序配错 | "发 shoulder 动 elbow" | `TestJointOrder`：`qposadr` 与 `mobile_joint_ids()` 对账 |
| 限幅写在引擎里 | 命令越界时关节停在限位，**看起来对** | `TestLimits`：断言 `clamped` 在**目标阶段**就记录 |
| 物理根本没跑 | 状态总是等于命令 | `TestPhysicsIsReal`：重力下垂 / 关节耦合 / 速度非零 |
| 换成另一台机器人就崩 | 硬编码 mini_arm | `TestEngineGenerality`：用**合成模型**驱动 |

## 真值只有一份

所有期望都从 `RobotModel`（`mobile_joint_ids` / `JointLimits`）与
`MjModel`（`jnt_qposadr` / `jnt_range`）**两边独立推导再比对** ——
不写第三份关节表。这正是本 Phase 要验证的"两个来源一致"。

## 为什么容差是 `1e-3` 而不是 `1e-12`

`MuJoCoBackend` 是**物理**后端，不是线性插值器。位置型执行器
（`<position kp=60 kv=4>`）在重力下垂与稳态误差下**不可能**精确到位：

```text
肩部命令 0.5 rad → 稳态 0.5023 rad（残差 2.3e-3）
  构成：重力下垂（上臂+前臂重心偏离转轴）
      + 位置伺服的比例稳态误差（kp 有限）
```

所以判据分两层（与 Phase 3 的"分类优于容差"同源）：

```text
① 物理量必须是**物理**的：残差必须小（否则说明没收敛），
   且必须**不随步数继续漂移**（否则说明在发散）
② 纯运动学的断言（FK 一致性、四元数换算）才用机器精度
```
"""

from __future__ import annotations

import asyncio
import math
import os
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco", reason="MuJoCo 未安装")

from backend.kinematics.fk import forward_kinematics  # noqa: E402
from backend.model.types import Quaternion, Transform, Vector3  # noqa: E402
from backend.runtime import RobotBackend, RobotCommand, RobotRuntime  # noqa: E402
from backend.runtime.backend import BackendError  # noqa: E402
from backend.simulation import (  # noqa: E402
    DEFAULT_MAX_SUBSTEPS,
    JointOrderError,
    MuJoCoBackend,
    assemble_state,
    clamp_targets_to_limits,
    mj_quat_to_xyzw,
    mujoco_backend_factory,
    ordered_joint_ids,
    positions_to_vector,
    vector_to_positions,
    xyzw_to_mj_quat,
)
from backend.simulation.simulation_backend import DEFAULT_STEP_SECONDS  # noqa: E402


def _run(coro):
    """跑协程。与 `test_runtime.py` 同样刻意**不用** pytest-asyncio。"""
    return asyncio.run(coro)


#: 物理量判据的容差（见模块 docstring 的双层原则）。
PHYSICS_TOL = 5e-3
#: 运动学/换算类判据的容差。
EXACT_TOL = 1e-12


@pytest.fixture
def backend(mini_arm_model) -> MuJoCoBackend:
    """一个已启动的 `MuJoCoBackend`（每个测试一个，避免状态串味）。"""
    b = MuJoCoBackend(mini_arm_model)
    _run(b.start())
    return b


@pytest.fixture
def sim_runtime():
    """`RobotRuntime` + `MuJoCoBackend` 的真实闭环（§62 的验收对象）。"""
    rt = RobotRuntime(backend_factory=mujoco_backend_factory)
    _run(rt.start())
    try:
        yield rt
    finally:
        _run(rt.stop())


# ===========================================================================
# 单一的四元数边界（§40 / §35）
# ===========================================================================


class TestQuaternionBoundary:
    """★ 这是 Phase 5 最容易被"互相抵消"掩盖的一处。"""

    #: 四个分量互不相等、非零、非 ±1 —— 任何顺序错误都会产生不同的数值。
    #: 用 `[1,0,0,0]` 这类特殊值测试是**无效**的（重排后仍是它自己）。
    ASYMMETRIC_WXYZ = (0.5, 0.1, 0.2, 0.3)

    def test_mj_to_xyzw_moves_w_to_the_end(self) -> None:
        """`[w,x,y,z]` → `[x,y,z,w]`：w 从**首**位移到**末**位。"""
        got = mj_quat_to_xyzw((0.5, 0.1, 0.2, 0.3))
        assert got == [0.1, 0.2, 0.3, 0.5]

    def test_xyzw_to_mj_moves_w_to_the_front(self) -> None:
        """反向：`[x,y,z,w]` → `[w,x,y,z]`。"""
        got = xyzw_to_mj_quat((0.1, 0.2, 0.3, 0.5))
        assert got == [0.5, 0.1, 0.2, 0.3]

    def test_round_trip_is_the_identity(self) -> None:
        """往返必须**逐位**回到原值（不能只是"落在同一旋转上"）。"""
        src = list(self.ASYMMETRIC_WXYZ)
        assert mj_quat_to_xyzw(xyzw_to_mj_quat(src)) == src
        assert xyzw_to_mj_quat(mj_quat_to_xyzw(src)) == src

    def test_identity_quaternion_is_distinguishable(self) -> None:
        """单位四元数：`[0,0,0,1]`（xyzw）↔ `[1,0,0,0]`（wxyz）。

        这条是**最廉价**的防呆：顺序写反的实施者在这里就会看到
        `[1,0,0,0]` 变成 `[0,0,0,1]`，而不是一个"看起来也对"的值。
        """
        assert mj_quat_to_xyzw((1.0, 0.0, 0.0, 0.0)) == [0.0, 0.0, 0.0, 1.0]
        assert xyzw_to_mj_quat(Quaternion.identity().to_list()) == [1.0, 0.0, 0.0, 0.0]

    def test_rotation_by_pi_over_two_about_y(self) -> None:
        """绕 Y 转 90°：`[x,y,z,w] = [0, √2/2, 0, √2/2]`。

        用**已知解析值**校验方向 —— 上一组用任意数只能验"顺序"，
        这一组验"顺序**与**含义都对"。两者缺一不可：
        顺序对而含义错（比如把共轭当成了本体）在任意数测试里看不出来。
        """
        q = Quaternion.from_axis_angle(Vector3.unit_y(), math.pi / 2).to_list()
        mj = xyzw_to_mj_quat(q)
        assert mj == pytest.approx([math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0], abs=EXACT_TOL)
        assert mj_quat_to_xyzw(mj) == pytest.approx(q, abs=EXACT_TOL)

    def test_conversion_is_confined_to_one_file(self) -> None:
        """全仓库只允许 `mujoco_backend.py` 出现 `[w,x,y,z]` 重排。

        ⚠️ 这段扫描必须能**失败**。它靠"匹配到别的文件的重排模式"报警；
        为了证明它不是恒真，先断言"被认可的那一个文件确实是匹配的"
        （正向证据），再断言"没有第二个文件"（负向）。
        """
        import re

        root = Path(__file__).resolve().parent.parent
        #: 载体 A：`[q0,q1,q2,q3]` 的下标重排
        sel_re = re.compile(r"\[\s*[a-zA-Z_.\[\]0-9]+\[1\]\s*,\s*[a-zA-Z_.\[\]0-9]+\[2\]")
        #: 载体 B：直接对元组/列表解包重排 `w, x, y, z = ...`
        unpack_re = re.compile(r"^\s*w\s*,\s*x\s*,\s*y\s*,\s*z\s*=", re.M)

        hits: list[str] = []
        allowed = root / "backend" / "simulation" / "mujoco_backend.py"
        for p in sorted((root / "backend").rglob("*.py")):
            text = p.read_text(encoding="utf-8")
            if sel_re.search(text) or unpack_re.search(text):
                hits.append(str(p.relative_to(root)).replace("\\", "/"))

        # 正向证据：本文件的换算函数必须**确实**用了重排
        assert hits, (
            "扫描器没有匹配到任何文件 —— 说明它失效了，"
            "而不是'没有越界'。请检查 sel_re / unpack_re 是否仍匹配 mj_quat_to_xyzw 的实现。"
        )
        assert hits == ["backend/simulation/mujoco_backend.py"], (
            f"四元数重排出现在 {hits}；应当只在 backend/simulation/mujoco_backend.py。"
            f"（多个点做同一件事 = 迟早有一个被改错，而错误会互相抵消。）"
        )
        assert allowed.is_file()

    def test_mujoco_quaternion_matches_core_fk(self, backend, mini_arm_model) -> None:
        """★ 交叉验证：MuJoCo 的 TCP 姿态与 Core FK **逐位**一致。

        这是整个 Phase 5 最有说服力的一条断言，因为它把
        "四元数边界换算正确"与"MJCF 几何与 RobotModel 几何对齐"
        同时验证了 —— 两个来源（`site_xmat` 与链式 FK）独立算出来，
        只可能在**两边都对**时一致。

        阈值用机器精度而不是 1e-3：这条路径**不含物理**，
        差异只来自浮点，所以"1e-6 的差异"就意味着实现错了。
        """
        _run(backend.reset())
        for _ in range(3):
            _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 0.37, "elbow": -0.22})))

        snap = backend.snapshot()
        mj_xyzw = mj_quat_to_xyzw(snap["mj_tcp_quaternion_wxyz"])
        core_xyzw = snap["core_fk_quaternion_xyzw"]

        assert max(abs(a - b) for a, b in zip(mj_xyzw, core_xyzw)) < 1e-9, (
            f"MuJoCo 姿态 {mj_xyzw} 与 Core FK {core_xyzw} 不一致"
        )
        assert max(abs(a - b) for a, b in zip(
            snap["mj_tcp_position"], snap["core_fk_position"])) < 1e-9


# ===========================================================================
# 关节顺序（§30 的契约）
# ===========================================================================


class TestJointOrder:
    def test_model_order_equals_qpos_address_order(self, mini_arm_model, mj_model) -> None:
        """★ `mobile_joint_ids()` 的顺序 == `jnt_qposadr` 升序。

        这条一旦不成立，"发 shoulder 动 elbow"就会静默发生 ——
        而且**所有基于自洽性的测试仍然会通过**（因为读写都用同一个错序）。
        """
        ids = ordered_joint_ids(mini_arm_model)
        addrs = []
        for jid in ids:
            j = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            assert j >= 0, f"MJCF 里没有关节 {jid!r}"
            addrs.append(int(mj_model.jnt_qposadr[j]))
        assert addrs == sorted(addrs), f"qposadr = {addrs} 不是升序（顺序不一致）"
        assert addrs == list(range(len(ids))), (
            f"v0.1 的铰链模型每个关节占 1 个 qpos 分量，"
            f"故 qposadr 应当就是 0..n-1，实际 {addrs}"
        )

    def test_backend_has_same_order_as_model(self, backend, mini_arm_model) -> None:
        assert backend._joint_ids == mini_arm_model.mobile_joint_ids()

    def test_start_rejects_mismatched_order(self, mini_arm_model) -> None:
        """顺序不一致时 `start()` 必须**拒绝启动**，而不是跑起来。

        构造方式：把 MJCF 里两个关节的声明顺序对调（XML 层改动），
        再让 RobotModel 保持原顺序 —— 这正是"改了 MJCF 忘了改别处"的形态。

        ⚠️ 这个测试的价值在于它证明了**断言存在**。若 `start()` 不检查，
        它会静默通过，而这个测试会红 —— 所以它不是恒真的。
        """
        src = Path(__file__).resolve().parent.parent / "packages" / "mini_arm" / "model" / "mini_arm.xml"
        swapped = _SwappedModelBase(mini_arm_model)
        assert swapped.mobile_joint_ids() == list(reversed(mini_arm_model.mobile_joint_ids()))
        b = MuJoCoBackend(swapped, source=str(src))
        with pytest.raises(BackendError, match="顺序|qposadr"):
            _run(b.start())

    def test_vector_conversion_round_trip(self, mini_arm_model) -> None:
        ids = ordered_joint_ids(mini_arm_model)
        values = [0.1 * (i + 1) for i in range(len(ids))]
        vec = positions_to_vector(mini_arm_model, dict(zip(ids, values)))
        assert vec == values
        assert vector_to_positions(mini_arm_model, vec) == dict(zip(ids, values))

    def test_positions_to_vector_rejects_missing_joint(self, mini_arm_model) -> None:
        """缺一个关节必须报错，**不能补 0**。

        补 0 会把"状态里少了 shoulder"表达成"shoulder 在 0 弧度" ——
        而后者是完全合法的位形，于是错误被完美隐藏。
        """
        with pytest.raises(JointOrderError, match="缺少关节"):
            positions_to_vector(mini_arm_model, {"shoulder": 0.1})

    def test_vector_to_positions_rejects_short_vector(self, mini_arm_model) -> None:
        with pytest.raises(JointOrderError, match="分量"):
            vector_to_positions(mini_arm_model, [0.1])


class _SwappedModelBase:
    """真子类：`mobile_joint_ids()` 反向，其余全部转发。

    ⚠️ 必须是**真子类**而不是"实例字典打补丁"的包装：`mobile_joint_ids`
    是定义在 `RobotModel` 类上的方法，实例字典里本来没有它。
    第一版用 `self.__dict__.update({k: getattr(inner, k) for k in dir(inner)})`
    把**绑定方法**拷进实例字典 —— 其中 `mobile_joint_ids` 指向的是
    inner 自己的那个方法（已绑定到 inner），于是"覆盖"根本没发生，
    测试恒不触发（表现为 `DID NOT RAISE`）。
    """

    def __init__(self, inner):
        self._inner = inner

    def mobile_joint_ids(self):  # noqa: D102
        return list(self._inner.mobile_joint_ids())[::-1]

    def __getattr__(self, name):
        return getattr(self._inner, name)


# ===========================================================================
# 限位 / 限幅
# ===========================================================================


class TestLimits:
    def test_clamp_uses_joint_limits_not_actuator_ctrlrange(self, mini_arm_model) -> None:
        """限幅的**依据**必须是关节限位。

        为什么这条要单独测：两个区间在本模型里数值相同，所以"用错了"
        在行为上看不出差别 —— 直到有人给 Actuator 加上映射
        （`ctrlrange` 变成 `-1..1`），那时整个臂就只剩 ±1 rad 的行程，
        而且**没有任何错误提示**。
        """
        joints = {j.id: j for j in mini_arm_model.joints}
        actuators = {a.joint: a for a in mini_arm_model.actuators if a.joint}
        for jid, joint in joints.items():
            if joint.limits is None or not joint.limits.has_position_bounds():
                continue
            act = actuators.get(jid)
            assert act is not None, f"关节 {jid!r} 没有执行器"
            # 本模型刻意让二者相等（见 MJCF 注释），但断言落在**关节**上
            assert clamp_targets_to_limits(mini_arm_model, {jid: 99.0})[1][jid] == (
                joint.limits.position_max
            )

    def test_out_of_range_target_is_clamped_and_reported(self, backend, mini_arm_model) -> None:
        """超限目标：`clamped` 必须记录，且值 == 模型上限。"""
        _run(backend.reset())
        jid = ordered_joint_ids(mini_arm_model)[1]
        upper = mini_arm_model.joint(jid).limits.position_max
        _run(backend.step(RobotCommand.of("mini_arm", {jid: 99.0})))
        assert backend.clamped == {jid: upper}

    def test_engine_receives_the_clamped_target(self, backend, mini_arm_model) -> None:
        """★ 关键：送进 `d.ctrl` 的必须是**夹紧后**的值。

        这一条能抓住"限幅写在引擎里"的错误实现：那种实现下
        `d.ctrl` 是 99.0（MuJoCo 会静默丢弃越界的 ctrl，
        于是机器人**纹丝不动**，而没有任何错误）。
        """
        _run(backend.reset())
        jid = ordered_joint_ids(mini_arm_model)[1]
        idx = backend._joint_ids.index(jid)
        upper = mini_arm_model.joint(jid).limits.position_max
        _run(backend.step(RobotCommand.of("mini_arm", {jid: 99.0})))
        assert backend._data.ctrl[idx] == pytest.approx(upper, abs=EXACT_TOL)

    def test_clamped_target_is_reached(self, backend, mini_arm_model) -> None:
        """限幅后关节**真的到得了**上限 —— 证明限位没被别的东西挡住。

        ⚠️ 这条曾经会失败，而失败的原因是**模型缺陷**不是代码缺陷：
        立柱与上臂自碰撞，把 shoulder 卡在 0.9367 而不是 1.5708。
        修法是在 MJCF 里显式 `exclude`（见 `mini_arm.xml` 的 contact 段）。
        保留这条断言，就是为了让那个缺陷不会悄悄回来。
        """
        jid = ordered_joint_ids(mini_arm_model)[1]
        limits = mini_arm_model.joint(jid).limits
        for upper in (limits.position_max, limits.position_min):
            _run(backend.reset())
            for _ in range(40):
                st = _run(backend.step(RobotCommand.of("mini_arm", {jid: upper})))
            assert abs(st.joint_positions[jid] - upper) < PHYSICS_TOL, (
                f"命令关节到 {upper}，稳态 {st.joint_positions[jid]}"
                f"（差 {abs(st.joint_positions[jid] - upper):.4f} rad）—— "
                f"很可能是自碰撞把关节挡住了，检查 MJCF 的 <contact><exclude>"
            )

    def test_in_range_target_is_not_clamped(self, backend, mini_arm_model) -> None:
        """合法目标不得被夹 —— 证明限幅不是"无差别夹紧"。"""
        _run(backend.reset())
        jid = ordered_joint_ids(mini_arm_model)[1]
        _run(backend.step(RobotCommand.of("mini_arm", {jid: 0.4})))
        assert backend.clamped == {}

    def test_unbounded_joint_is_not_clamped(self, mini_arm_model) -> None:
        """未声明限位的关节不得被伪造一个限幅。

        `JointLimits.position_min/max` 为 `None` 的语义是"**未声明**"
        （不是 0、也不是无穷）。给它夹一个"默认 ±π"是造出一条模型
        从未声明的约束，而它的后果（关节到不了某些角度）极难追溯。

        ⚠️ 构造手法：直接在**真模型**上追加一个 `limits=JointLimits()` 的关节。
        第一版用 `__dict__.update(dir(inner))` 的包装类，结果
        `mobile_joint_ids()` 与 `joints` 是类属性/实例属性混用，
        `__getattr__` 与实例字典打架 ⇒ `KeyError: 没有 joint 'free_spin'`。
        这正是"包装类冒充模型"这个手法本身的坑：**越像真模型越好**。
        """
        from backend.model.robot_model import Joint, JointLimits

        free = Joint(
            id="free_spin",
            name="free_spin",
            type="revolute",
            parent_link=mini_arm_model.root_link,
            child_link=mini_arm_model.root_link,
            origin=Transform.identity(),
            axis=Vector3.unit_z(),
            limits=JointLimits(),  # 无边界
        )
        patched = _ModelWithExtraJoint(mini_arm_model, free)
        # 正向证据：包装类真的把新关节接上了（否则下面的断言是空转）
        assert "free_spin" in patched.mobile_joint_ids()
        assert patched.joint("free_spin") is free

        out, clamped = clamp_targets_to_limits(patched, {"free_spin": 1000.0})
        assert out == {"free_spin": 1000.0}
        assert clamped == {}

    def test_none_limits_joint_is_not_clamped(self, backend) -> None:
        """`limits is None` 的路径也要覆盖（合成模型里的 `fixed` 关节是 None）。

        与上一条的区别：上一条是 `JointLimits()`（"声明了但没有边界"），
        本条是 `limits=None`（"完全没声明"）。两条都必须**不夹**，
        且都会 `return` —— 若实现里少了一个分支判断，
        这里会 `AttributeError: 'NoneType' has no attribute 'has_position_bounds'`。
        """
        from backend.model.robot_model import Joint, RobotModel, Transform, Vector3

        free = Joint(
            id="no_limits",
            name="no_limits",
            type="revolute",
            parent_link=backend._model.root_link,
            child_link=backend._model.root_link,
            origin=Transform.identity(),
            axis=Vector3.unit_z(),
            limits=None,
        )
        patched = _ModelWithExtraJoint(backend._model, free)
        out, clamped = clamp_targets_to_limits(patched, {"no_limits": -5.0})
        assert out == {"no_limits": -5.0}
        assert clamped == {}
        assert isinstance(backend._model, RobotModel)


class _ModelWithExtraJoint:
    """在模型上挂一个额外的关节，用于测 `limits=None` / 无边界 的路径。

    ⚠️ 与 `_SwappedModelBase` 同一类坑：必须**真子类化**，
    不能靠实例字典打补丁 —— 否则 `mobile_joint_ids` / `joint` 这些
    类方法不会被覆盖，`__getattr__` 也不会被调用（实例字典里已有名字）。
    """

    def __init__(self, inner, joint):
        self._inner = inner
        self._joint = joint
        self._joints = list(inner.joints) + [joint]

    @property
    def joints(self):
        return self._joints

    def joint(self, joint_id):
        if joint_id == self._joint.id:
            return self._joint
        return self._inner.joint(joint_id)

    def mobile_joint_ids(self):
        return [j.id for j in self._joints if j.is_mobile()]

    def __getattr__(self, name):
        return getattr(self._inner, name)


# ===========================================================================
# 物理是真的（不是线性插值器）
# ===========================================================================


class TestPhysicsIsReal:
    """★ 这一组证明"换成真物理"这件事**真的发生了**。

    否则 `MuJoCoBackend` 可能与 `MockBackend` 行为相同，
    而 §62 的验收会"通过"，却什么都没验证。
    """

    def test_gravity_causes_droop(self, mini_arm_model) -> None:
        """零位形**不是**平衡位形：重力把上臂压低。

        若这个残差为 0，说明仿真里没有重力（或 qpos 被直接写了值
        而不是被 `mj_step` 推出来的）—— 那是"假物理"。
        """
        b = MuJoCoBackend(mini_arm_model)
        _run(b.start())
        snap = b.snapshot()
        # 零位时上臂质心在 +X，重力矩把它往下压 ⇒ shoulder 变正（绕 +Y 是俯）
        assert abs(snap["positions"]["shoulder"]) > 1e-4, (
            "重力没有产生下垂 —— 检查 MuJoCo 的 gravity 与 mj_step"
        )
        assert abs(snap["positions"]["shoulder"]) < 1e-2, (
            f"下垂 {snap['positions']['shoulder']} rad 过大 —— 静平衡可能没收敛"
        )

    def test_state_has_nonzero_velocities_during_motion(self, backend) -> None:
        """运动中速度必须非零。零速度 = 状态是算出来的而不是推出来的。"""
        _run(backend.reset())
        st = _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 1.0})))
        assert max(abs(v) for v in st.joint_velocities.values()) > 1e-3

    def test_velocities_decay_at_rest(self, backend) -> None:
        """到位后速度必须衰减到 ~0 —— 证明"稳态"是真的稳态。"""
        _run(backend.reset())
        for _ in range(60):
            st = _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 0.4})))
        assert max(abs(v) for v in st.joint_velocities.values()) < 1e-3

    def test_joint_coupling_exists(self, backend) -> None:
        """★ 关节耦合：动一个近端关节，**未命令**的远端关节会被惯性带动。

        `MockBackend` 结构上不可能产生这个现象（它只改被命令的关节）。
        所以这条断言是一个**判据**，能区分"真物理"与"限速插值"。

        ⚠️ 判据是"**发生了变化**"而不是"变成了某个值"：
        末端的最终角度由惯性/重力/阻尼共同决定，没有闭式期望值。
        """
        _run(backend.reset())
        before = backend.snapshot()["positions"]["elbow"]
        for _ in range(10):
            st = _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 1.2})))
        after = st.joint_positions["elbow"]
        assert abs(after - before) > 1e-5, (
            f"命令 shoulder 后 elbow 完全没动（{before} → {after}）"
            f" —— 这看起来像 MockBackend 的线性插值，不是物理"
        )

    def test_target_smaller_than_gravity_droop_is_bounded(self, backend) -> None:
        """命令 0.0 时关节不会**越过**零位很多（重力只把它压到平衡点）。

        防的是"符号写反导致正反馈一发不可收"：那种实现下
        命令 0 会让关节角度指数增长，而"状态非零"这一条也会通过。
        """
        _run(backend.reset())
        for _ in range(60):
            st = _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 0.0})))
        assert abs(st.joint_positions["shoulder"]) < 0.05

    def test_no_nan_in_state(self, backend) -> None:
        """★ 数值稳定性：任何状态下都不得出现 NaN/Inf。

        这条是防回归的：`armature` 缺失时仿真会在**1 步内**溢出成 NaN
        （见 `mini_arm.xml` 里关于 ω = sqrt(kp/I) 的注释）。
        NaN 一旦出现会**静默传播**（比较全为 False，看起来像"没动"）。
        """
        _run(backend.reset())
        for target in (1.4, -1.4, 0.0, 0.8, -0.3):
            for _ in range(20):
                st = _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": target})))
            for jid, v in st.joint_positions.items():
                assert math.isfinite(v), f"{jid} = {v}（命令 {target} 后溢出）"
            for jid, v in st.joint_velocities.items():
                assert math.isfinite(v), f"d({jid})/dt = {v}（命令 {target} 后溢出）"


# ===========================================================================
# 命令语义
# ===========================================================================


class TestCommandSemantics:
    def test_send_command_does_not_advance_physics(self, backend) -> None:
        """`send_command` 只写目标，**不**推进物理（§28）。

        若它顺手推一步，那"命令 → 状态"的因果就被压缩成了一次调用，
        而 §61/§62 要求的"Command ≠ State"会退化成"回显"。
        """
        b0 = backend.snapshot()
        _run(backend.send_command(RobotCommand.of("mini_arm", {"shoulder": 0.9})))
        b1 = backend.snapshot()
        assert b1["positions"] == b0["positions"], "send_command 推进了物理"
        assert b1["targets"]["shoulder"] == pytest.approx(0.9)
        assert b1["command_count"] == b0["command_count"] + 1

    def test_untouched_joints_keep_their_target(self, backend) -> None:
        """未提及的关节"保持"（不是复位到 0）。

        ⚠️ 判据是**目标不变**（invariant），不是"位置到了某值"——
        物理会让它缓慢下垂，位置本来就会漂一点。
        """
        _run(backend.reset())
        for _ in range(20):
            _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 0.5})))
        before = backend.snapshot()["targets"]["elbow"]
        _run(backend.step(RobotCommand.of("mini_arm", {"base_yaw": 0.3})))
        assert backend.snapshot()["targets"]["elbow"] == before

    def test_unknown_joint_is_rejected(self, backend) -> None:
        with pytest.raises(ValueError, match="shulder|未知|没有"):
            _run(backend.step(RobotCommand.of("mini_arm", {"shulder": 0.1})))

    def test_command_before_start_raises(self, mini_arm_model) -> None:
        b = MuJoCoBackend(mini_arm_model)
        with pytest.raises(BackendError, match="未启动"):
            _run(b.send_command(RobotCommand.of("mini_arm", {"shoulder": 0.1})))

    def test_get_state_before_start_raises(self, mini_arm_model) -> None:
        b = MuJoCoBackend(mini_arm_model)
        with pytest.raises(BackendError, match="未启动"):
            _run(b.get_state())

    def test_reset_zeroes_joints_and_keeps_command_count(self, backend) -> None:
        """`reset()` 归零关节但**保留** `command_count`。

        与 `MockBackend` 同一契约：测试要靠 `command_count` 证明
        "命令真的到了 Backend"，而 reset 是最常被调用的操作。
        """
        for _ in range(15):
            _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 0.6})))
        count_before = backend.command_count
        _run(backend.reset())
        snap = backend.snapshot()
        for jid, v in snap["positions"].items():
            assert v == 0.0, f"reset 后 {jid} = {v}"
        assert snap["command_count"] == count_before
        assert snap["clamped"] == {}

    def test_reset_does_not_settle(self, backend) -> None:
        """`reset()` 后必须**立刻**是零位（不能"归零 + 再沉一会儿"）。

        理由：否则测试里分不清"复位没生效"与"刚复位还在下沉"。
        """
        for _ in range(15):
            _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 0.6})))
        _run(backend.reset())
        assert backend.snapshot()["positions"]["shoulder"] == 0.0

    def test_status_transitions(self, backend) -> None:
        assert backend.snapshot()["status"] in ("idle", "reset")
        st = _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 0.2})))
        assert st.status == "running"

    def test_start_and_stop_are_idempotent(self, mini_arm_model) -> None:
        b = MuJoCoBackend(mini_arm_model)
        _run(b.start())
        _run(b.start())
        assert b.snapshot()["started"] is True
        _run(b.stop())
        _run(b.stop())
        assert b.snapshot()["started"] is False


# ===========================================================================
# 与 Core / Runtime 的一致性
# ===========================================================================


class TestConsistencyWithCore:
    def test_state_pose_equals_core_fk(self, sim_runtime, mini_arm_model) -> None:
        """状态里的末端位姿 == Core FK 对**同一批关节角**的结果。

        这条防的是"Backend 自己发明了一套位姿计算"。同时它也证明
        `end_effector_pose` 与 `joint_positions` 说的是**同一时刻**的事
        （若位姿晚一帧或多算一步，这里就会差出可观测的量）。
        """
        st = _run(sim_runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.31, "elbow": 0.17})))
        ref = forward_kinematics(mini_arm_model, dict(st.joint_positions))
        assert st.end_effector_pose is not None
        assert max(
            abs(a - b)
            for a, b in zip(
                st.end_effector_pose.position.to_list(), ref.position.to_list()
            )
        ) < EXACT_TOL
        assert max(
            abs(a - b)
            for a, b in zip(
                st.end_effector_pose.orientation.to_list(), ref.orientation.to_list()
            )
        ) < EXACT_TOL

    def test_pose_varies_with_configuration(self, sim_runtime) -> None:
        """位姿必须随关节角变化（防"回显写死常量"）。"""
        _run(sim_runtime.reset())
        a = _run(sim_runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.1})))
        for _ in range(30):
            b = _run(sim_runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.9})))
        pa = a.end_effector_pose.position.to_list()
        pb = b.end_effector_pose.position.to_list()
        assert max(abs(x - y) for x, y in zip(pa, pb)) > 1e-2

    def test_timestamps_are_monotonic_simulation_time(self, sim_runtime) -> None:
        """`timestamp` 是**仿真时间**且单调递增。

        用仿真时间而不是墙钟：物理后端的"时间"必须与它推进的量一致，
        否则"0.02 s/步"这个契约无法用时间戳验证。
        """
        _run(sim_runtime.reset())
        ts = []
        for _ in range(5):
            st = _run(sim_runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.3})))
            ts.append(st.timestamp)
        assert ts == sorted(ts) and ts[-1] > ts[0]

    def test_step_advances_by_step_seconds(self, sim_runtime) -> None:
        """一次 `step()` 推进 ≈ `step_seconds` 的仿真时间。

        判据是**区间**而不是精确值：子步是整数，而 `timestep` 不一定整除
        `step_seconds`，所以实际推进量会略超一个子步。
        """
        _run(sim_runtime.reset())
        st = _run(sim_runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.3})))
        assert DEFAULT_STEP_SECONDS <= st.timestamp < DEFAULT_STEP_SECONDS * 1.2


# ===========================================================================
# 引擎无关的公共层
# ===========================================================================


class TestSimulationLayer:
    def test_assemble_state_fills_all_model_joints(self, mini_arm_model) -> None:
        st = assemble_state(mini_arm_model, {"shoulder": 0.5})
        assert set(st.joint_positions) == set(mini_arm_model.mobile_joint_ids())
        assert st.joint_positions["shoulder"] == 0.5
        assert st.joint_positions["elbow"] == 0.0
        assert st.robot == mini_arm_model.metadata.id

    def test_assemble_state_status_is_validated(self, mini_arm_model) -> None:
        with pytest.raises(ValueError, match="status"):
            assemble_state(mini_arm_model, {}, status="wat")

    def test_step_seconds_default_is_not_the_mjcf_timestep(self) -> None:
        """`step_seconds` 与 MJCF 的 `timestep` 是**两个**量。

        它们相等会让"一次 step = 一个积分步"，于是 `armature` 带来的
        稳定性余量被浪费掉，且调 `timestep` 会静默改变收敛速度。
        """
        assert DEFAULT_STEP_SECONDS > 0.002
        assert DEFAULT_MAX_SUBSTEPS * 0.002 >= DEFAULT_STEP_SECONDS


# ===========================================================================
# 引擎通用性（§69 规则 2：不得出现机器人型号分支）
# ===========================================================================


class TestEngineGenerality:
    """用**合成的第二个机器人**验证 Backend 不是为 mini_arm 写的。"""

    SYNTHETIC_XML = """<mujoco model="two_dof_toy">
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="base" pos="0 0 0">
      <geom name="g_base" type="box" size="0.02 0.02 0.02"/>
      <body name="link_a" pos="0 0 0.05">
        <joint name="pan" type="hinge" axis="0 0 1"
               range="-2.0 2.0" damping="0.05" armature="0.01"/>
        <geom name="g_a" type="capsule" size="0.01 0.03" pos="0.03 0 0"
              quat="0.70710678 0 0.70710678 0"/>
        <body name="link_b" pos="0.06 0 0">
          <joint name="lift" type="hinge" axis="0 1 0"
                 range="-1.0 1.0" damping="0.05" armature="0.01"/>
          <geom name="g_b" type="capsule" size="0.01 0.02" pos="0.02 0 0"
                quat="0.70710678 0 0.70710678 0"/>
          <site name="tcp" pos="0.05 0 0" size="0.003"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="pan_motor" joint="pan" kp="60" kv="4" ctrlrange="-2.0 2.0"/>
    <position name="lift_motor" joint="lift" kp="60" kv="4" ctrlrange="-1.0 1.0"/>
  </actuator>
</mujoco>
"""

    def _toy_model(self):
        from backend.loaders.mjcf_loader import MJCFLoader

        model, _ = MJCFLoader(robot_id="two_dof_toy").load(self.SYNTHETIC_XML)
        return model

    def test_backend_drives_a_second_robot(self) -> None:
        """★ 换成 2-DOF 玩具机器人，同一份 Backend 代码必须能跑。

        这条同时证明三件事：
        ① 关节数不是硬编码的（2 vs 3）；
        ② 关节名不是硬编码的（`pan` / `lift` vs `base_yaw` / `shoulder`）；
        ③ `nq/nv` 校验是按模型算的，不是按 mini_arm 算的。
        """
        model = self._toy_model()
        assert model.mobile_joint_ids() == ["pan", "lift"]

        b = MuJoCoBackend(model, source=self.SYNTHETIC_XML)
        _run(b.start())
        assert b.snapshot()["joints"] == ["pan", "lift"]

        st = _run(b.step(RobotCommand.of(model.metadata.id, {"pan": 0.5, "lift": -0.4})))
        assert set(st.joint_positions) == {"pan", "lift"}
        # 位置能到位（旋转轴不含重力矩，故残差只来自伺服）
        for _ in range(60):
            st = _run(b.step(RobotCommand.of(model.metadata.id, {"pan": 0.5, "lift": -0.4})))
        assert abs(st.joint_positions["pan"] - 0.5) < PHYSICS_TOL
        assert abs(st.joint_positions["lift"] + 0.4) < PHYSICS_TOL
        assert st.end_effector_pose is not None

    def test_runtime_with_synthetic_robot(self, tmp_path) -> None:
        """整条链路（发现 → 加载 → Runtime → Backend）对第二台机器人成立。"""
        pkg = tmp_path / "two_dof_toy"
        (pkg / "model").mkdir(parents=True)
        (pkg / "model" / "robot.xml").write_text(self.SYNTHETIC_XML, encoding="utf-8")
        (pkg / "manifest.yaml").write_text(
            "id: two_dof_toy\n"
            "name: Two DOF Toy\n"
            "version: 0.1.0\n"
            "model:\n  format: mjcf\n  file: model/robot.xml\n"
            "capabilities:\n  simulation: true\n  fk: true\n  ik: false\n"
            "  actuator_control: true\n  end_effector: true\n",
            encoding="utf-8",
        )

        rt = RobotRuntime(packages_dir=tmp_path, backend_factory=mujoco_backend_factory)
        _run(rt.start())
        try:
            assert list(rt.models()) == ["two_dof_toy"]
            assert type(rt.backend("two_dof_toy")).__name__ == "MuJoCoBackend"
            st = _run(rt.step(RobotCommand.of("two_dof_toy", {"pan": 0.3})))
            assert st.joint_positions["pan"] > 0.0
        finally:
            _run(rt.stop())


# ===========================================================================
# 架构约束
# ===========================================================================


class TestArchitecture:
    ROOT = Path(__file__).resolve().parent.parent

    @staticmethod
    def _code_only(path: Path) -> str:
        """剥离注释与字符串 —— 否则一句注释就能让扫描失败/通过。"""
        import io
        import tokenize

        src = path.read_text(encoding="utf-8")
        out = []
        for tok in tokenize.generate_tokens(io.StringIO(src).readline):
            if tok.type in (tokenize.COMMENT, tokenize.STRING):
                continue
            out.append(tok.string)
        return " ".join(out)

    def test_simulation_layer_has_no_robot_literals(self) -> None:
        """§69 规则 2：`backend/simulation/` 不得出现机器人型号字面量。"""
        offenders = []
        for p in sorted((self.ROOT / "backend" / "simulation").glob("*.py")):
            code = self._code_only(p)
            for lit in ("mini_arm", "mearm", "MeArm"):
                if lit in code:
                    offenders.append(f"{p.name}: {lit}")
        assert not offenders, f"仿真层出现型号字面量：{offenders}"

    def test_simulation_does_not_import_api(self) -> None:
        """Backend 只吃 `RobotModel`，不认识 packages 注册表。

        例外是 `api/registry.py` 的 `PACKAGES_DIR` 常量 —— 那是"到哪里找包"，
        不是"包里有谁"。本测试因此只禁 `get_package` / `discover_packages`
        这类**查询**接口。
        """
        offenders = []
        for p in sorted((self.ROOT / "backend" / "simulation").glob("*.py")):
            code = self._code_only(p)
            for bad in ("get_package", "discover_packages", "RobotPackage"):
                if bad in code:
                    offenders.append(f"{p.name}: {bad}")
        assert not offenders, f"仿真层依赖了注册表：{offenders}"

    def test_runtime_does_not_import_simulation(self) -> None:
        """依赖方向：`runtime` 不得 **import** `simulation`。

        若反过来，**默认 Backend 会变成 MuJoCo**，
        于是 Phase 4 的 MockBackend 路径（以及它的测试）全部失效，
        而且"Backend 可注入"这条契约会退化成"可注入但默认硬编码"。

        ⚠️ 判据必须是"**import 语句**"而不是"文本里出现 `simulation`"：
        `RobotBackend.is_simulation` 这个**字段名**里就有 simulation，
        而它是 §47 接口的一部分。用文本包含判会得到一个恒假的检查
        （永远有 offender）。这与 §4.7"扰动自检写空转"是同一类错误：
        **判据要测的语义，与它实际测到的东西不是一回事。**
        """
        offenders = []
        for p in sorted((self.ROOT / "backend" / "runtime").glob("*.py")):
            for ln, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
                s = line.strip()
                if not (s.startswith("import ") or s.startswith("from ")):
                    continue
                low = s.lower()
                if "simulation" in low or "mujoco" in low:
                    offenders.append(f"{p.name}:{ln}: {s}")
        assert not offenders, f"runtime 的 import 依赖了 simulation/mujoco：{offenders}"

    def test_backend_factory_signature_takes_only_a_model(self) -> None:
        """工厂必须**只吃 `RobotModel`**。

        若它需要 `packages_dir` 或 robot_id，`simulation` 就得认识注册表，
        而 §69 规则 10 的"换执行端"会从"换一个函数"变成"改两处调用链"。
        """
        import inspect

        sig = inspect.signature(mujoco_backend_factory)
        assert list(sig.parameters) == ["model"]

    def test_mujoco_is_imported_lazily(self) -> None:
        """`mujoco` 必须是**局部** import（在 `_require_mujoco()` 里）。

        理由：`backend/simulation/__init__.py` 会让 import 这个包本身就
        触发模块加载。若顶部 `import mujoco`，那么"没装 mujoco 的环境"
        在 import 阶段就崩，而正确行为是**启动 Backend 时**给一条
        可操作的错误（"pip install mujoco"）。
        """
        p = self.ROOT / "backend" / "simulation" / "mujoco_backend.py"
        text = p.read_text(encoding="utf-8")
        code = self._code_only(p)
        assert "import mujoco" in code, "扫描器失效：文件里应当有 `import mujoco`"
        top_level = [
            ln for ln in text.splitlines()
            if ln.startswith("import mujoco") or ln.startswith("from mujoco")
        ]
        assert not top_level, f"模块级 import mujoco：{top_level}"

    def test_backend_class_implements_the_interface(self, mini_arm_model) -> None:
        """`MuJoCoBackend` 必须是 `RobotBackend`（§47）。"""
        assert issubclass(MuJoCoBackend, RobotBackend)
        assert isinstance(MuJoCoBackend(mini_arm_model), RobotBackend)

    def test_factory_returns_a_backend(self, mini_arm_model) -> None:
        b = mujoco_backend_factory(mini_arm_model)
        assert isinstance(b, RobotBackend)
        assert type(b).__name__ == "MuJoCoBackend"


# ===========================================================================
# §69 规则 10：与 MockBackend 的互换性
# ===========================================================================


class TestInterchangeableWithMockBackend:
    def test_both_backends_share_the_full_interface(self) -> None:
        """两个 Backend 的公共方法集合必须一致。

        做法：扫 `RobotBackend` 的抽象方法 + 公共方法，
        断言两者都能 `hasattr`。这条能抓住"MuJoCoBackend 少实现了
        一个抽象方法但没报错"（不报错是因为漏了 `@abstractmethod`）。
        """
        from backend.runtime import MockBackend

        required = [
            m for m in ("start", "stop", "reset", "send_command", "get_state", "step")
        ]
        for name in required:
            assert callable(getattr(MuJoCoBackend, name, None)), f"MuJoCoBackend 缺 {name}"
            assert callable(getattr(MockBackend, name, None)), f"MockBackend 缺 {name}"

    def test_runtime_is_agnostic_to_which_backend(self, tmp_path, mini_arm_model) -> None:
        """同一段 Runtime 代码，两个 Backend 都能驱动（§69 规则 10）。

        判据是**不变量**：两次运行的 shape / 键集合相同，
        数值**允许不同**（那正是"换了执行端"的意义）。
        """
        from backend.runtime import MockBackend

        results = {}
        for label, factory in (
            ("mock", lambda m: MockBackend(m)),
            ("mujoco", mujoco_backend_factory),
        ):
            rt = RobotRuntime(backend_factory=factory)
            _run(rt.start())
            try:
                st = _run(rt.step(RobotCommand.of("mini_arm", {"shoulder": 0.5})))
                results[label] = st
            finally:
                _run(rt.stop())

        assert set(results["mock"].joint_positions) == set(results["mujoco"].joint_positions)
        assert results["mock"].robot == results["mujoco"].robot
        assert results["mock"].status == results["mujoco"].status
        # 数值**应当**不同：MockBackend 精确走 0.35，MuJoCo 被惯性拖着走。
        assert (
            abs(
                results["mock"].joint_positions["shoulder"]
                - results["mujoco"].joint_positions["shoulder"]
            )
            > 0.05
        ), "两个 Backend 产生了一模一样的单步状态 —— 说明其中一个没有真的在执行"


# ===========================================================================
# 诊断探针的元测试
# ===========================================================================


class TestSnapshotProbe:
    def test_tcp_gap_is_at_machine_precision(self, backend) -> None:
        """`tcp_position_gap` 是**几何对齐**探针，应当约等于机器精度。

        它不是精度指标。若它变成 1e-3 量级，说明 MJCF 里某条 `body pos`
        与 RobotModel 期待的数字不一致（改了 MJCF 忘了别处）。
        """
        _run(backend.reset())
        for _ in range(3):
            _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 0.27, "elbow": -0.13})))
        gap = backend.snapshot()["tcp_position_gap"]
        assert gap < 1e-9, f"TCP 位置差 {gap} —— MJCF 几何与 RobotModel 可能已漂移"

    def test_snapshot_does_not_mutate_state(self, backend) -> None:
        """`snapshot()` 必须无副作用（它是调试/测试入口，会被反复调用）。"""
        _run(backend.reset())
        _run(backend.step(RobotCommand.of("mini_arm", {"shoulder": 0.4})))
        a = backend.snapshot()
        b = backend.snapshot()
        assert a["positions"] == b["positions"]
        assert a["substeps"] == b["substeps"]

    def test_source_can_be_injected(self, mini_arm_model) -> None:
        """`source` 可注入 ⇒ 测试能用 XML 字符串驱动，不依赖仓库里的文件路径。"""
        xml = (
            Path(__file__).resolve().parent.parent
            / "packages" / "mini_arm" / "model" / "mini_arm.xml"
        ).read_text(encoding="utf-8")
        b = MuJoCoBackend(mini_arm_model, source=xml)
        _run(b.start())
        assert b.snapshot()["started"] is True

    def test_missing_source_gives_actionable_error(self, mini_arm_model) -> None:
        """找不到 MJCF 时必须报出"请显式传 source"，而不是一个 FileNotFoundError。"""
        from backend.model.robot_model import RobotMetadata

        stripped = _ModelWithMetadata(
            mini_arm_model,
            RobotMetadata(id="mini_arm", name="x", version="0.0.0", description=""),
        )
        b = MuJoCoBackend(stripped)
        with pytest.raises(BackendError, match="source"):
            _run(b.start())


class _ModelWithMetadata:
    """只换 `metadata` 的包装（同样是真子类，理由见 `_SwappedModelBase`）。"""

    def __init__(self, inner, metadata):
        self._inner = inner
        self._metadata = metadata

    @property
    def metadata(self):
        return self._metadata

    def __getattr__(self, name):
        return getattr(self._inner, name)
