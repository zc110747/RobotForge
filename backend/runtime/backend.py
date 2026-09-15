"""Backend 抽象 —— 运动学与物理的执行端（提示词 §47）。

```python
class RobotBackend:
    async def start(self): ...
    async def stop(self): ...
    async def reset(self): ...
    async def send_command(self, command): ...
    async def get_state(self): ...
```

## 这个抽象存在的**唯一理由**

§47 列了 v0.1 的实现 `MuJoCoBackend` 和未来的 `RealRobotBackend`，
并说"统一：RobotCommand / RobotState"。§69 规则 10 把它写成 P0 规则：
**"Simulation 与 Real 使用统一 Backend Interface。"**

所以这个类的价值不是"抽象一下更优雅"，而是把下面这个问题
在**今天**就变成可回答的：

> 把 MuJoCo 换成一台真机，需要改几行 Runtime？

如果 Runtime 只依赖这五个 `async` 方法、只吃 `RobotCommand`、只吐 `RobotState`，
答案是**零行**。§63 的 Phase 6 验收就是在验证这件事
（用 `MockBackend` 冒充真机，Runtime 一行不动）。

## 为什么全是 `async`

不只是"以后要等网络"。`asyncio` 让**同一段 Runtime 代码**
在仿真（微秒级）和真机（毫秒级串口）上都成立，
而不需要为真机写第二套线程模型。
一旦签名是同步的，未来加 `await` 就是**破坏性变更**，
会波及 Runtime + WebSocket + 所有测试。

## `MockBackend` 为什么必须"有状态"且"会限幅"

见 `state.py` docstring 的同一段论证。补充一点：

`MockBackend` 是 Phase 4（本 Phase）唯一可执行的 Backend，
Phase 5 才有 MuJoCo。如果它把命令原样回显，
那么 Phase 4 的验收"Command 发送 → Backend 执行 → State 返回"
就退化成"字典 A 变成了字典 B"—— 它**结构上**通过，**信息上**什么都没有。

所以 MockBackend：

1. **有状态**：未提及的关节保持上一帧位置（不是清零）。
2. **限幅**：超出 `JointLimits` 的目标被夹到边界并记录
   （`clamp`），所以"命令 ≠ 状态"这个关键不等式在 Phase 4 就会被演练。
3. **限速**：单步最多走 `max_step_rad`，所以它需要多帧才收敛到位
   —— 这让"跟踪误差"成为可观测的量。

这三条都是**故意**的失败面制造：它们让 Phase 5 换上真物理时，
上层已经见过"命令与状态不一致"的正常情况。
"""

from __future__ import annotations

import abc
import time
from typing import Any

from ..model.robot_model import RobotModel
from .command import RobotCommand
from .state import RobotState

#: MockBackend 单步最大关节增量（rad，或棱柱关节的 m）。
#: 取 0.35 rad ≈ 20°，是一个"明显需要多帧才到位"的量。
#: 调小会让测试要跑很多帧；调大到 π 就退化成一步到位、失去限速意义。
DEFAULT_MAX_STEP = 0.35


class BackendError(RuntimeError):
    """Backend 层的失败（启动失败 / 未启动就收命令 / 内部异常）。

    刻意与 `ValueError`（命令内容非法）分开：
    前者是**环境**问题（没插硬件、模型编译失败），后者是**调用方**问题。
    API 层要把它们映射成不同的 WS 帧/状态码。
    """


class RobotBackend(abc.ABC):
    """所有执行端的统一接口（§47）。

    ## 子类必须保证的性质

    * `send_command` 之后 `get_state` 反映的是 **Actual**，
      允许与命令的 Desired 不同（这是契约，不是缺陷）。
    * `start` / `stop` / `reset` 可重复调用（幂等），
      因为 WS 客户端断线重连不会去同步服务器的生命周期状态。
    * 任何方法都不允许让异常逃逸成"连接崩掉"：
      Backend 抛 `BackendError`，由 Runtime 决定是转 `error` 帧还是重连。
    """

    #: 该 Backend 是否驱动真实物理。`MockBackend` 为 True
    #: （它*假装*是物理，所以它承担"状态可以滞后于命令"的义务）。
    is_simulation: bool = True

    @abc.abstractmethod
    async def start(self) -> None:
        """建立资源（编译模型 / 打开串口）。幂等。"""

    @abc.abstractmethod
    async def stop(self) -> None:
        """释放资源。幂等。"""

    @abc.abstractmethod
    async def reset(self) -> None:
        """回到初始状态（关节归零 / 仿真时间归零）。"""

    @abc.abstractmethod
    async def send_command(self, command: RobotCommand) -> None:
        """下发**目标**。返回 `None` —— 因为命令不代表执行结果（§28）。"""

    @abc.abstractmethod
    async def get_state(self) -> RobotState:
        """读取**当前观测状态**（§29）。"""

    # ---- 便捷组合（非抽象：子类不必实现）----

    async def step(self, command: RobotCommand) -> RobotState:
        """`send_command` + `get_state` 的组合。

        存在的理由是**顺序**：Runtime 的 Sim2Sim 闭环是
        "发命令 → 推进 → 读状态"，把它写成一步可以防止调用方
        不小心先读状态再发命令（那样读到的永远是上一帧）。
        """
        await self.send_command(command)
        return await self.get_state()


class MockBackend(RobotBackend):
    """Phase 4 的唯一 Backend：一个**有状态、限幅、限速**的替身。

    Phase 5 会加 `MuJoCoBackend`（真物理），两者**接口完全相同** ——
    这正是 §69 规则 10 要求的可替换性。

    ## 它不是"假数据"

    它按 `RobotModel` 的 `JointLimits` 和 `mobile_joint_ids()` 工作，
    所以它读的是**同一份真值**（RobotModel），
    而不是自己编一张关节表。将来换上 MuJoCo，
    被验证的"命令 → 状态 → WS → 前端"这条链**一模一样**。
    """

    def __init__(
        self,
        model: RobotModel,
        initial_positions: dict[str, float] | None = None,
        max_step: float = DEFAULT_MAX_STEP,
        clamp: bool = True,
    ) -> None:
        self._model = model
        self._max_step = float(max_step)
        self._clamp_enabled = bool(clamp)

        # ★ 关节顺序取自模型，**顺序即 FK/qpos 顺序**（§30 / robot_model 注释）
        self._joint_ids: list[str] = model.mobile_joint_ids()

        self._positions: dict[str, float] = {jid: 0.0 for jid in self._joint_ids}
        if initial_positions:
            unknown = sorted(set(initial_positions) - set(self._joint_ids))
            if unknown:
                raise ValueError(
                    f"initial_positions 含未知关节 {unknown}（可用 {self._joint_ids}）"
                )
            for jid, val in initial_positions.items():
                self._positions[jid] = float(val)

        self._velocities: dict[str, float] = {jid: 0.0 for jid in self._joint_ids}
        self._status = "idle"
        self._started = False
        self._t0 = time.monotonic()

        #: 被限幅的目标，供测试与诊断断言"命令确实被改过"。
        self.clamped: dict[str, float] = {}
        #: 累计收到的命令数（用于验证"命令真的到了 Backend"）。
        self.command_count = 0

    # ---- 生命周期 ----

    async def start(self) -> None:
        self._started = True
        self._status = "idle"
        self._t0 = time.monotonic()

    async def stop(self) -> None:
        self._started = False
        self._status = "idle"

    async def reset(self) -> None:
        """关节归零、速度清零。**不**清零 `command_count`**（那是统计量）。

        为什么保留 `command_count`：测试要用它证明"命令确实到达了 Backend"，
        而复位恰好是测试中最常调用的操作。清零会让"复位后没收到命令"
        与"从没收到命令"无法区分。
        """
        for jid in self._joint_ids:
            self._positions[jid] = 0.0
            self._velocities[jid] = 0.0
        self.clamped.clear()
        self._status = "reset"
        self._t0 = time.monotonic()

    # ---- 命令 ----

    async def send_command(self, command: RobotCommand) -> None:
        if not self._started:
            raise BackendError(
                "Backend 未启动就收到命令。Runtime 必须在 send_command 前 await start()。"
            )
        command.validate_against(self._joint_ids)
        self.command_count += 1

        for jid, target in command.joint_targets.items():
            if self._clamp_enabled:
                joint = self._model.joint(jid)
                limits = joint.limits
                if limits.has_position_bounds():
                    clamped, was_clamped = limits.clamps(target)
                    if was_clamped:
                        self.clamped[jid] = clamped
                        target = clamped

            # 限速：向目标走一步，而不是瞬移到位。
            current = self._positions[jid]
            delta = target - current
            step = self._max_step
            new = current + (step if delta > step else (-step if delta < -step else delta))
            self._velocities[jid] = (new - current)  # 单位时间 = 1 步，故数值上等于位移
            self._positions[jid] = new

        self._status = "running"

    # ---- 观测 ----

    async def get_state(self) -> RobotState:
        """产生 `RobotState`。**末端位姿由 Core FK 算出**（不读包内常量）。

        ⚠️ 这一点很重要：如果用包内的解析 FK，那么
        "Command → Backend → State" 这条验收就变成了
        "回显了包内那几个写死的常量"，任何关节配置都会得到同一个位姿。
        用 Core 链式 FK 则位姿**真的随关节角变化**。
        """
        # 延迟 import：`runtime` → `kinematics` 是合法方向
        # （§69 只禁止 kinematics → runtime 与 model → kinematics）。
        from ..kinematics.fk import forward_kinematics

        # ⚠️ `forward_kinematics(model, positions)` 只有两个参数：
        #    它自己按 §23 的 EndEffector 解析 TCP 帧。
        #    不要传第三个 `frame` 参数 —— 签名没有它。
        pose = None
        if self._model.default_end_effector() is not None:
            try:
                pose = forward_kinematics(self._model, dict(self._positions))
            except (KeyError, ValueError):
                # 模型声明了 EE 但链不完整 ⇒ 位姿未知。**不**用 identity 冒充
                # （见 state.py 里的同一论证）。
                pose = None

        return RobotState(
            robot=self._model.metadata.id,
            joint_positions=dict(self._positions),
            joint_velocities=dict(self._velocities),
            end_effector_pose=pose,
            status=self._status,
            timestamp=time.monotonic() - self._t0,
        )

    # ---- 诊断 ----

    def snapshot(self) -> dict[str, Any]:
        """无副作用的内部快照（测试/日志用）。故意**不**是 `get_state`：
        后者有 `async` 与时间戳语义，不适合调试打印。"""
        return {
            "started": self._started,
            "status": self._status,
            "joints": list(self._joint_ids),
            "positions": dict(self._positions),
            "clamped": dict(self.clamped),
            "command_count": self.command_count,
        }


__all__ = [
    "DEFAULT_MAX_STEP",
    "BackendError",
    "MockBackend",
    "RobotBackend",
]
