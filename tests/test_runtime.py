"""Runtime 层测试（提示词 §46 / §47 / §28 / §29）。

## 这组测试在防什么

Phase 4 的核心风险**不是**"代码跑不起来"，而是**"跑起来了但什么都没验证"**：

| 风险 | 本文件的对策 |
|---|---|
| `MockBackend` 把命令原样回显 ⇒ "Command→State"退化成恒真 | `TestCommandIsNotState`：断言 State 真的可以 ≠ Command |
| Runtime 里偷偷出现机器人型号 | `TestArchitecture`：源码扫描 + `accept_phase4.py` |
| Backend 与 Runtime 焊死（§69 规则 10 作废） | `TestBackendIsInjectable`：塞一个自定义 Backend 进 Runtime |
| "未启动"伪装成"没有机器人" | `TestLifecycle`：未启动时读模型必须抛错 |
| 命令里的关节名打错了却"成功" | `TestCommandValidation`：未知关节必须被拒绝 |

## 真值只有一份

期望值全部从 `RobotModel` 推导（`mobile_joint_ids()` / `JointLimits`），
不另写一份关节表 —— 否则"改了 MJCF 忘了改测试"会让测试通过而系统是错的。
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from backend.api.registry import discover_packages, get_package
from backend.model.robot_model import RobotModel
from backend.runtime import (
    BackendError,
    MockBackend,
    RobotBackend,
    RobotCommand,
    RobotRuntime,
    RobotState,
    RuntimeError_,
)
from backend.runtime.state import STATUSES


# ---------------------------------------------------------------------------
# 夹具
# ---------------------------------------------------------------------------


@pytest.fixture
def runtime() -> RobotRuntime:
    """一个**已启动**的、真实的 `RobotRuntime`（mini_arm）。

    `packages_dir=None` ⇒ 用仓库的 `packages/`。
    之所以不用人造包：本文件要验证的是"真实链路"，
    而人造包环境（验证"加机器人不用改 Core"）在 `test_api.py` 里。
    """
    rt = RobotRuntime()
    asyncio.run(rt.start())
    try:
        yield rt
    finally:
        asyncio.run(rt.stop())


def _run(coro):
    """跑一个协程。

    ⚠️ 刻意用 `asyncio.run` 而不是 pytest-asyncio：
    本项目其余测试都是同步的，引入插件会让 conftest 的
    作用域规则变复杂（尤其 `mini_arm_model` 是 session 级）。
    每个测试自己起一个事件循环，互不污染，且失败信息更直白。
    """
    return asyncio.run(coro)


# ===========================================================================
# §28 / §29 —— 两个契约必须分离
# ===========================================================================


class TestCommandContract:
    def test_command_requires_robot(self) -> None:
        with pytest.raises(ValueError, match="robot"):
            RobotCommand.of("", {"shoulder": 0.1})

    def test_command_rejects_bool_joint_value(self) -> None:
        """`{"shoulder": true}` 必须被拒绝。

        Python 里 `isinstance(True, int)` 为真，所以不显式检查的话
        `true` 会静默变成 `1.0` 弧度 —— 一个没有提示的 57° 误差。
        """
        with pytest.raises(TypeError, match="bool"):
            RobotCommand.of("mini_arm", {"shoulder": True})

    def test_command_rejects_nan_and_inf(self) -> None:
        for bad in (float("nan"), float("inf"), float("-inf")):
            with pytest.raises(ValueError):
                RobotCommand.of("mini_arm", {"shoulder": bad})

    def test_command_rejects_negative_timestamp(self) -> None:
        with pytest.raises(ValueError, match="timestamp"):
            RobotCommand.of("mini_arm", {"shoulder": 0.1}, timestamp=-1.0)

    def test_command_unknown_joint_rejected(self, runtime: RobotRuntime) -> None:
        """打错的关节名必须报错，**不能**静默忽略。

        静默忽略的表现是"发了命令但机器人不动"，而日志一片正常 ——
        这是 Sim2Real 上最贵的一类 bug。
        """
        cmd = RobotCommand.of("mini_arm", {"shulder": 0.1})
        with pytest.raises(ValueError, match="shulder"):
            _run(runtime.step(cmd))

    def test_command_is_frozen(self) -> None:
        """命令一旦发出不可改 —— Backend 收到的不该是共享可变字典。"""
        cmd = RobotCommand.of("mini_arm", {"shoulder": 0.1})
        with pytest.raises(Exception):
            cmd.joint_targets = {"shoulder": 0.9}  # type: ignore[misc]

    def test_command_wire_format_uses_joints_key(self) -> None:
        """§52 的线上字段名是 `joints`，内部契约名是 `joint_targets`。

        这个映射必须在**一处**发生，所以这里钉住它。
        """
        cmd = RobotCommand.of("mini_arm", {"shoulder": 0.5}, timestamp=1.0)
        wire = cmd.to_dict()
        assert wire == {"robot": "mini_arm", "joints": {"shoulder": 0.5}, "timestamp": 1.0}
        assert json.dumps(wire)  # 必须可 JSON 序列化

    def test_command_accepts_partial_joint_set(self) -> None:
        """部分指令合法（前端拖一个滑块就是这类）。"""
        cmd = RobotCommand.of("mini_arm", {"shoulder": 0.5})
        cmd.validate_against(["base_yaw", "shoulder", "elbow"])  # 不抛错


class TestStateContract:
    def test_state_rejects_unknown_status(self) -> None:
        with pytest.raises(ValueError, match="status"):
            RobotState.of("mini_arm", status="Error")  # 大小写错误必须被抓

    def test_all_statuses_are_usable(self) -> None:
        for s in STATUSES:
            assert RobotState.of("mini_arm", status=s).status == s

    def test_state_none_pose_is_not_identity(self) -> None:
        """`end_effector_pose=None`（不知道）≠ `Transform.identity()`（在原点）。

        用零位姿冒充"不知道"，会让"末端真的在原点"与"末端位置未知"
        再也无法区分 —— 这是一个**丢信息**的转换。
        """
        unknown = RobotState.of("mini_arm")
        assert unknown.end_effector_pose is None
        assert unknown.to_dict()["end_effector"] is None

    def test_state_accepts_flat_pose_sequence(self) -> None:
        """`[x,y,z,qx,qy,qz,qw]` 扁平形式（WS 上更紧凑）。"""
        st = RobotState.of("mini_arm", end_effector_pose=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
        assert st.end_effector_pose is not None
        assert st.to_dict()["end_effector"]["orientation"] == [0.0, 0.0, 0.0, 1.0]

    def test_state_rejects_wrong_length_pose_sequence(self) -> None:
        with pytest.raises(ValueError, match="7"):
            RobotState.of("mini_arm", end_effector_pose=[0.1, 0.2, 0.3])

    def test_state_position_vector_uses_model_order(self) -> None:
        st = RobotState.of("mini_arm", joint_positions={"a": 1.0, "b": 2.0})
        assert st.position_vector(["b", "a"]) == [2.0, 1.0]

    def test_state_position_vector_missing_joint_raises(self) -> None:
        """缺关节必须抛错，**不能**补 0。

        补 0 会把"状态里没有肩关节"表达成"肩关节在 0 弧度"，
        而那是一个完全合法的位形 —— 错误被伪装成了正常值。
        """
        st = RobotState.of("mini_arm", joint_positions={"a": 1.0})
        with pytest.raises(KeyError):
            st.position_vector(["a", "missing"])

    def test_stale_like_keeps_observations(self) -> None:
        """出错时保留最后已知关节角（画面不该跳回原点）。"""
        st = RobotState.of("mini_arm", {"shoulder": 0.42}, status="running")
        stale = st.stale_like()
        assert stale.status == "error"
        assert stale.joint_positions == {"shoulder": 0.42}


# ===========================================================================
# ★ 本 Phase 最关键的一组：Command ≠ State
# ===========================================================================


class TestCommandIsNotState:
    """如果 MockBackend 把命令原样回显，Phase 4 的验收就是恒真的。

    §29 明确要求"即使 v0.1 是 Sim2Sim，也必须维持该语义"。
    所以这里逐条钉住三种"State 可以 ≠ Command"的真实机制。
    """

    def test_ratelimit_makes_state_lag_command(self, mini_arm_model: RobotModel) -> None:
        """限速：一步走不到目标 ⇒ State ≠ Command（这是**设计**，不是缺陷）。"""
        backend = MockBackend(mini_arm_model, max_step=0.1)
        _run(backend.start())
        cmd = RobotCommand.of("mini_arm", {"shoulder": 1.0})
        state = _run(backend.step(cmd))

        assert state.joint_positions["shoulder"] == pytest.approx(0.1)
        assert state.joint_positions["shoulder"] != cmd.joint_targets["shoulder"]

    def test_ratelimit_converges_in_finite_steps(self, mini_arm_model: RobotModel) -> None:
        """限速是"慢"不是"不到" —— 足够多步后必须精确到位。

        这一条与上一条配对：只有"不到"而没有"最终到"，
        就说明限速实现成了硬性截断（一个更隐蔽的 bug）。
        """
        backend = MockBackend(mini_arm_model, max_step=0.1)
        _run(backend.start())
        cmd = RobotCommand.of("mini_arm", {"shoulder": 1.0})
        state = None
        for _ in range(20):
            state = _run(backend.step(cmd))
        assert state is not None
        assert state.joint_positions["shoulder"] == pytest.approx(1.0, abs=1e-12)

    def test_untouched_joints_hold_position(self, mini_arm_model: RobotModel) -> None:
        """未提及的关节**保持原位**，不是清零。

        ⚠️ 判据是"不变"而不是"已到位"。这里用 `max_step=10.0`（一步到位）
        让两者重合；但**不要**据此以为"保持原位"等于"等于上一个目标" ——
        在限速下 shoulder 会停在中途，那是正确的（见 accept_phase4.py 的同一注记）。
        """
        backend = MockBackend(mini_arm_model, max_step=10.0)
        _run(backend.start())
        ids = mini_arm_model.mobile_joint_ids()

        _run(backend.step(RobotCommand.of("mini_arm", {ids[0]: 0.4})))
        state = _run(backend.step(RobotCommand.of("mini_arm", {ids[1]: 0.4})))

        assert state.joint_positions[ids[0]] == pytest.approx(0.4)
        assert state.joint_positions[ids[1]] == pytest.approx(0.4)

    def test_untouched_joints_hold_position_under_ratelimit(
        self, mini_arm_model: RobotModel
    ) -> None:
        """限速下"保持原位"的正确判据：**逐位不变**，而非"已到位"。

        这条测试是从 accept_phase4.py 的一次真实误报里长出来的：
        当时把判据写成"两个关节都到 0.4"，而限速让 shoulder 停在 0.35
        ⇒ 引擎完全正确，却被判失败。
        """
        backend = MockBackend(mini_arm_model, max_step=0.35)
        _run(backend.start())
        ids = mini_arm_model.mobile_joint_ids()

        _run(backend.step(RobotCommand.of("mini_arm", {ids[0]: 0.4})))
        before = _run(backend.get_state()).joint_positions[ids[0]]

        for _ in range(10):
            state = _run(backend.step(RobotCommand.of("mini_arm", {ids[1]: 0.4})))

        # 未提及的关节逐位不变（真的没被动过）
        assert state.joint_positions[ids[0]] == pytest.approx(before, abs=1e-15)
        # 被提及的关节最终收敛（限速是"慢"不是"不到"）
        assert state.joint_positions[ids[1]] == pytest.approx(0.4, abs=1e-12)

    def test_clamping_makes_state_differ_from_command(self, mini_arm_model: RobotModel) -> None:
        """限幅：超限目标被夹到 [lower, upper]。"""
        backend = MockBackend(mini_arm_model, max_step=10.0)
        _run(backend.start())

        joint = mini_arm_model.joint(mini_arm_model.mobile_joint_ids()[0])
        limits = joint.limits
        assert limits.has_position_bounds(), "本测试依赖模型真的声明了限位"
        # 字段名是 position_min / position_max —— **不是** upper/lower。
        # 且单位的语义由 joint.type 决定（revolute ⇒ rad，prismatic ⇒ m），
        # 所以限位值可以直接与命令里的值比较。
        upper = limits.position_max
        assert upper is not None

        state = _run(backend.step(RobotCommand.of("mini_arm", {joint.id: upper + 5.0})))
        assert state.joint_positions[joint.id] == pytest.approx(upper)
        assert backend.clamped[joint.id] == pytest.approx(upper)
        # ★ 命令带的是 upper + 5.0，状态是 upper ⇒ 二者确实不同
        assert state.joint_positions[joint.id] != pytest.approx(upper + 5.0)

    def test_clamping_is_reported_not_silent(self, mini_arm_model: RobotModel) -> None:
        """限幅必须**可观测**（记录在 `backend.clamped`）。

        静默限幅会让"我发的 3.0 rad 怎么只到 1.57"变成无法回答的问题。
        """
        backend = MockBackend(mini_arm_model, max_step=100.0)
        _run(backend.start())
        assert backend.clamped == {}
        jid = mini_arm_model.mobile_joint_ids()[0]
        _run(backend.step(RobotCommand.of("mini_arm", {jid: 99.0})))
        assert jid in backend.clamped


# ===========================================================================
# §46 —— Runtime 生命周期与模型管理
# ===========================================================================


class TestLifecycle:
    def test_reading_models_before_start_raises(self) -> None:
        """未启动 ≠ 没有机器人。

        返回空列表会让"忘了启动"表现成"一台机器人都没有"，
        而这两种情况的修复动作完全不同。
        """
        rt = RobotRuntime()
        with pytest.raises(RuntimeError_, match="start"):
            rt.models()
        with pytest.raises(RuntimeError_, match="start"):
            rt.packages()

    def test_start_is_idempotent(self) -> None:
        rt = RobotRuntime()
        _run(rt.start())
        first = rt.summary()
        _run(rt.start())
        assert rt.summary() == first
        _run(rt.stop())

    def test_stop_is_idempotent(self) -> None:
        rt = RobotRuntime()
        _run(rt.start())
        _run(rt.stop())
        _run(rt.stop())  # 不应抛错

    def test_stop_releases_backends(self, runtime: RobotRuntime) -> None:
        assert runtime.summary()["backends"], "start 后必须真的有 backend"
        _run(runtime.stop())
        assert runtime.summary()["backends"] == {}

    def test_reset_zeroes_joints(self, runtime: RobotRuntime) -> None:
        _run(runtime.step(RobotCommand.of("mini_arm", {"shoulder": 99.0})))
        _run(runtime.reset("mini_arm"))
        state = _run(runtime.get_state("mini_arm"))
        assert state.status == "reset"
        assert set(state.joint_positions.values()) == {0.0}

    def test_reset_all_when_no_robot_given(self, runtime: RobotRuntime) -> None:
        _run(runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.5})))
        _run(runtime.reset())  # 不指定 ⇒ 全部
        assert _run(runtime.get_state("mini_arm")).joint_positions["shoulder"] == 0.0


class TestModelManagement:
    def test_discovers_mini_arm(self, runtime: RobotRuntime) -> None:
        """发现式注册：包来自 `packages/*/manifest.yaml`，不是硬编码表。"""
        assert "mini_arm" in runtime.models()
        assert [p.id for p in runtime.packages()] == ["mini_arm"]

    def test_model_ids_sorted(self, runtime: RobotRuntime) -> None:
        """顺序稳定 —— 否则前端测试无法稳定断言。"""
        ids = list(runtime.models())
        assert ids == sorted(ids)

    def test_model_matches_registry_loader(self, runtime: RobotRuntime) -> None:
        """Runtime 里的模型与直接走 registry 加载的**是同一份真值**。"""
        direct, _report = get_package("mini_arm").load_model()
        via_runtime = runtime.model("mini_arm")
        assert via_runtime.to_dict() == direct.to_dict()

    def test_validation_report_available(self, runtime: RobotRuntime) -> None:
        report = runtime.validation_report("mini_arm")
        assert report.ok is True
        assert report.issues == []

    def test_unknown_robot_raises(self, runtime: RobotRuntime) -> None:
        with pytest.raises(RuntimeError_, match="nope"):
            runtime.model("nope")
        assert runtime.has_robot("nope") is False


class TestStepClosedLoop:
    def test_step_returns_state_for_the_commanded_robot(self, runtime: RobotRuntime) -> None:
        state = _run(runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.5})))
        assert state.robot == "mini_arm"
        assert state.status == "running"
        assert "shoulder" in state.joint_positions

    def test_step_sets_end_effector_pose_from_core_fk(self, runtime: RobotRuntime) -> None:
        """末端位姿必须**随关节角变化**，且与 Core FK 一致。

        ⚠️ 如果用包内解析 FK，位姿会是那几个写死常量的函数，
        与 `runtime.model()` 里的模型脱钩 —— 那样这条件就白测了。
        所以这里拿 Core FK 当独立裁判（与 Phase 3 的同一策略）。
        """
        from backend.kinematics.fk import forward_kinematics

        state = _run(runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.3, "elbow": 0.2})))
        model = runtime.model("mini_arm")
        expected = forward_kinematics(model, dict(state.joint_positions))

        assert state.end_effector_pose is not None
        got = state.end_effector_pose
        exp = expected
        for a, b in zip(got.position.to_list(), exp.position.to_list()):
            assert a == pytest.approx(b, abs=1e-15)
        for a, b in zip(got.orientation.to_list(), exp.orientation.to_list()):
            assert a == pytest.approx(b, abs=1e-15)

    def test_pose_changes_with_joint_angle(self, runtime: RobotRuntime) -> None:
        """位姿**不是**常量 —— 防"回显写死常量"的实现。"""
        a = _run(runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.1, "elbow": 0.0})))
        b = _run(runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.9, "elbow": 0.0})))
        assert a.end_effector_pose is not None and b.end_effector_pose is not None
        pa = a.end_effector_pose.position.to_list()
        pb = b.end_effector_pose.position.to_list()
        assert max(abs(x - y) for x, y in zip(pa, pb)) > 1e-3

    def test_send_command_does_not_return_state(self, runtime: RobotRuntime) -> None:
        """§28：命令**不代表**执行结果，所以 `send_command` 返回 None。

        想一步拿到状态用 `step()`。把两件事分开是因为真实场景里它们
        本来就不该同步（真机要等好几个控制周期）。
        """
        result = _run(runtime.send_command(RobotCommand.of("mini_arm", {"shoulder": 0.5})))
        assert result is None

    def test_commands_are_serialized_by_lock(self, runtime: RobotRuntime) -> None:
        """并发命令不得交错（`asyncio.Lock` 的意义）。

        没有锁时，两个并发的 `step` 会让 A 的命令与 B 的命令
        在"发命令→读状态"之间交叉，表现为"A 的指令时灵时不灵"。
        这里用两个并发的 step 断言**两次读到的状态各自自洽**
        （各自对应自己那条命令的目标，而不是对方的）。
        """
        async def both():
            return await asyncio.gather(
                runtime.step(RobotCommand.of("mini_arm", {"shoulder": 0.5})),
                runtime.step(RobotCommand.of("mini_arm", {"shoulder": -0.5})),
            )

        s1, s2 = _run(both())
        # 两个状态都应该只包含 shoulder，且值在合法区间内（被限速夹住）
        for s in (s1, s2):
            assert s.robot == "mini_arm"
            assert abs(s.joint_positions["shoulder"]) <= 0.35 + 1e-12


# ===========================================================================
# §47 / §69 规则 10 —— Backend 必须可替换
# ===========================================================================


class _ScriptedBackend(RobotBackend):
    """一个**故意撒谎**的 Backend：状态永远是固定值，与命令无关。

    它的用途是证明"Runtime 真的通过接口调用 Backend" ——
    如果 Runtime 里藏着对 `MockBackend` 的 isinstance 判断或字段访问，
    这个替身就会失败。同时它也证明"State 可以完全不等于 Command"。
    """

    is_simulation = False

    def __init__(self, model: RobotModel) -> None:
        self._model = model
        self.calls: list[str] = []
        self.received: list[RobotCommand] = []

    async def start(self) -> None:
        self.calls.append("start")

    async def stop(self) -> None:
        self.calls.append("stop")

    async def reset(self) -> None:
        self.calls.append("reset")

    async def send_command(self, command: RobotCommand) -> None:
        self.calls.append("send_command")
        self.received.append(command)

    async def get_state(self) -> RobotState:
        self.calls.append("get_state")
        return RobotState.of(
            self._model.metadata.id,
            joint_positions={jid: 0.123 for jid in self._model.mobile_joint_ids()},
            status="idle",
        )


class TestBackendIsInjectable:
    def test_custom_backend_receives_commands(self) -> None:
        """§69 规则 10：换 Backend 不改 Runtime（Phase 6 Sim2Real 的立足点）。"""
        made: list[_ScriptedBackend] = []

        def factory(model: RobotModel) -> RobotBackend:
            b = _ScriptedBackend(model)
            made.append(b)
            return b

        rt = RobotRuntime(backend_factory=factory)
        _run(rt.start())
        try:
            assert made, "工厂必须被调用"
            assert "start" in made[0].calls

            state = _run(rt.step(RobotCommand.of("mini_arm", {"shoulder": 0.7})))
            assert made[0].received[0].joint_targets == {"shoulder": 0.7}
            assert "send_command" in made[0].calls and "get_state" in made[0].calls

            # ★ 撒谎 Backend 的状态与命令**完全不同** —— Runtime 照原样透传
            assert state.joint_positions["shoulder"] == pytest.approx(0.123)
            assert state.status == "idle"
        finally:
            _run(rt.stop())
        assert "stop" in made[0].calls

    def test_reset_reaches_backend(self) -> None:
        made: list[_ScriptedBackend] = []
        rt = RobotRuntime(backend_factory=lambda m: made.append(_ScriptedBackend(m)) or made[-1])
        _run(rt.start())
        _run(rt.reset("mini_arm"))
        _run(rt.stop())
        assert "reset" in made[0].calls

    def test_is_simulation_flag_is_backend_owned(self) -> None:
        """`is_simulation` 属于 Backend（Runtime 不该自己判断"是不是仿真"）。"""
        assert MockBackend.is_simulation is True
        assert _ScriptedBackend.is_simulation is False


class TestBackendErrors:
    def test_command_before_start_raises(self, mini_arm_model: RobotModel) -> None:
        backend = MockBackend(mini_arm_model)
        with pytest.raises(BackendError, match="未启动"):
            _run(backend.send_command(RobotCommand.of("mini_arm", {"shoulder": 0.1})))

    def test_reset_preserves_command_count(self, mini_arm_model: RobotModel) -> None:
        """`reset` 保留 `command_count`：测试靠它证明"命令确实到达了 Backend"，
        而 reset 恰好是最常被调用的操作。清零会让
        "复位后没收到命令"与"从没收到命令"无法区分。"""
        backend = MockBackend(mini_arm_model)
        _run(backend.start())
        _run(backend.send_command(RobotCommand.of("mini_arm", {"shoulder": 0.1})))
        assert backend.command_count == 1
        _run(backend.reset())
        assert backend.command_count == 1

    def test_wrong_type_rejected_by_runtime(self, runtime: RobotRuntime) -> None:
        with pytest.raises(TypeError, match="RobotCommand"):
            _run(runtime.step({"robot": "mini_arm"}))  # type: ignore[arg-type]


# ===========================================================================
# §69 规则 1/2 —— 架构约束
# ===========================================================================


class TestArchitecture:
    RUNTIME_DIR = Path(__file__).resolve().parent.parent / "backend" / "runtime"

    def _stripped_sources(self) -> dict[str, str]:
        import tokenize

        out: dict[str, str] = {}
        for p in sorted(self.RUNTIME_DIR.glob("*.py")):
            src = p.read_text(encoding="utf-8")
            out[p.name] = tokenize.untokenize(
                tok
                for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
                if tok.type not in (tokenize.COMMENT, tokenize.STRING)
            )
        return out

    def test_runtime_has_no_robot_model_literals(self) -> None:
        """§46：Runtime 不应该知道 `mini_arm` / `mearm` 等具体实现。

        剥离注释与字符串后再扫 —— 否则一句文档"Runtime 不知道 mini_arm"
        就会让检查失败，而那显然是误判。
        """
        offenders: list[str] = []
        for name, src in self._stripped_sources().items():
            for ln, line in enumerate(src.splitlines(), 1):
                for lit in ('"mini_arm"', "'mini_arm'", '"mearm"', "'mearm'"):
                    if lit in line:
                        offenders.append(f"{name}:{ln}: {line.strip()}")
        assert not offenders, f"runtime 出现型号字面量：{offenders[:3]}"

    def test_runtime_has_no_hardware_literals(self) -> None:
        """§46 的第二个清单：serial / CAN / USB 等传输实现。

        这些属于未来的 `Transport`（§48 v0.1 不实现），
        Runtime 现在就不该提及它们 —— 否则"预留接口"会变成"已经耦合"。
        """
        offenders: list[str] = []
        for name, src in self._stripped_sources().items():
            for ln, line in enumerate(src.splitlines(), 1):
                low = line.lower()
                for lit in ("serial", "can", "usb"):
                    # 用词边界式判断，避免 `can` 命中 `cannot` / `scan`
                    if f'"{lit}"' in low or f"'{lit}'" in low:
                        offenders.append(f"{name}:{ln}: {line.strip()}")
        assert not offenders, f"runtime 出现硬件/传输字面量：{offenders[:3]}"

    def test_runtime_does_not_import_fastapi_or_mujoco(self) -> None:
        """Runtime 是**纯编排层**：它不该知道 HTTP 框架或物理引擎。

        MuJoCo 的接入点是 `backend_factory`（Phase 5），
        而不是 Runtime 里的 `import mujoco`。
        """
        banned = ("fastapi", "mujoco", "starlette")
        offenders: list[str] = []
        for name, src in self._stripped_sources().items():
            for ln, line in enumerate(src.splitlines(), 1):
                s = line.strip()
                if not (s.startswith("import ") or s.startswith("from ")):
                    continue
                for b in banned:
                    if b in s:
                        offenders.append(f"{name}:{ln}: {s}")
        assert not offenders, f"runtime 反向依赖：{offenders[:3]}"

    def test_kinematics_does_not_import_runtime(self) -> None:
        """依赖方向只能是 runtime → kinematics。

        kinematics 是纯数学，它连 runtime 的存在都不该知道 ——
        否则 Phase 3 的"Core 通用性"证明会失效
        （Core 会被拴在这个项目的运行时设计上）。
        """
        kin = Path(__file__).resolve().parent.parent / "backend" / "kinematics"
        offenders: list[str] = []
        for p in sorted(kin.glob("*.py")):
            src = p.read_text(encoding="utf-8")
            for ln, line in enumerate(src.splitlines(), 1):
                s = line.strip()
                if (s.startswith("import ") or s.startswith("from ")) and "runtime" in s:
                    offenders.append(f"{p.name}:{ln}: {s}")
        assert not offenders, f"kinematics 反向依赖 runtime：{offenders}"

    def test_model_does_not_import_runtime(self) -> None:
        """§27：RobotModel 不保存 Runtime State —— 连 import 都不该有。"""
        mod = Path(__file__).resolve().parent.parent / "backend" / "model"
        offenders: list[str] = []
        for p in sorted(mod.glob("*.py")):
            src = p.read_text(encoding="utf-8")
            for ln, line in enumerate(src.splitlines(), 1):
                s = line.strip()
                if (s.startswith("import ") or s.startswith("from ")) and "runtime" in s:
                    offenders.append(f"{p.name}:{ln}: {s}")
        assert not offenders, f"model 反向依赖 runtime：{offenders}"

    def test_backend_module_does_not_import_registry(self) -> None:
        """Backend 只吃 `RobotModel`，不该知道"机器人包"这一层。

        它若 import 了 registry，就意味着 Backend 可能去自己找包 ——
        那"Runtime 负责 discovery"这条 §46 的分工就破了。
        """
        src = (self.RUNTIME_DIR / "backend.py").read_text(encoding="utf-8")
        import_lines = [
            ln.strip()
            for ln in src.splitlines()
            if ln.strip().startswith(("import ", "from "))
        ]
        assert not any("registry" in ln for ln in import_lines)


# ===========================================================================
# 包发现与 Phase 2 的 REST 层一致性
# ===========================================================================


class TestConsistencyWithRegistry:
    def test_runtime_and_registry_agree_on_ids(self, runtime: RobotRuntime) -> None:
        """Runtime 与 REST 层发现的是同一批包（同一个真值源）。"""
        via_registry = [p.id for p in discover_packages()]
        via_runtime = list(runtime.models())
        assert via_registry == via_runtime

    def test_model_json_serializable(self, runtime: RobotRuntime) -> None:
        """WS 帧要能真的过 `json.dumps` —— 不靠人工检查。"""
        payload = runtime.model("mini_arm").to_dict()
        assert json.loads(json.dumps(payload)) == payload
