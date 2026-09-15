"""RobotRuntime —— Package → Loader → RobotModel → Backend 的持有者（提示词 §46）。

RobotRuntime 负责（§46 逐字）：

```text
Package discovery
Package loading
RobotModel management
RobotCommand
Backend management
RobotState
Runtime lifecycle
```

并且**不应该知道**：

```text
mini_arm / mearm / serial / CAN / USB
```

## 那 Runtime 到底做什么

它是一条**窄腰**：左边是"多台机器人、多种包"，
右边是"多种 Backend"，而 Runtime 本身对两边都不知情。
它只知道三件事：

1. 有个 `discover_packages()` 能列出包（`RobotPackage` 对象，不是路径字符串）
2. 有个 `RobotPackage.load_model()` 能给出 `RobotModel`
3. 有个 `RobotBackend` 能收 `RobotCommand` 吐 `RobotState`

## 为什么 Backend 由**注入的工厂**产生

如果 Runtime 里写 `MockBackend(model)`，那么 §69 规则 10
（Simulation 与 Real 统一 Backend Interface）就废了 ——
Runtime 与具体 Backend 焊死了。所以构造参数是
`backend_factory: Callable[[RobotModel], RobotBackend]`。

于是"换成 MuJoCo"在 Phase 5 是**换一个工厂函数**，
而不是改 Runtime。Phase 6 的 Sim2Real 验证（§63）
也正是靠这个缝来插入假真机。

## 多机器人：`dict[str, RobotModel]` 而不是"当前机器人"

`models()` 返回**全部**已加载的模型。§51 要求前端有
"Robot Selection"，而单选的前端仍然需要一个列表来选。
如果 Runtime 只持有"当前模型"，那么每次切换都要重新加载 + 重新校验，
且"列出可用机器人"与"当前是谁"这两个关注点会在 API 层被混在一起。

v0.1 的策略是**预加载全部**（mini_arm 只有一个包，成本可忽略）。
将来包多了要改成惰性加载，改的是 `_ensure_loaded`，
而 `models()` / `model(id)` 的契约不变。
"""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any, Callable, Iterable

from ..model.robot_model import RobotModel
from ..model.validator import ValidationReport
from ..api.registry import (
    PACKAGES_DIR,
    RobotPackage,
    RobotPackageError,
    discover_packages,
)
from .backend import BackendError, MockBackend, RobotBackend
from .command import RobotCommand
from .state import RobotState

#: Backend 工厂的签名：拿一个 RobotModel，给出一个（未启动的）Backend。
BackendFactory = Callable[[RobotModel], RobotBackend]


def default_backend_factory(model: RobotModel) -> RobotBackend:
    """v0.1 默认工厂 = `MockBackend`。

    Phase 5 会把它换成"`MuJoCoBackend`"，
    而因为这是个**函数**而不是 Runtime 内部的 `if`，
    替换方式是 `RobotRuntime(backend_factory=mujoco_factory)` —— Runtime 零改动。
    """
    return MockBackend(model)


class RuntimeError_(RuntimeError):
    """Runtime 层错误（未知机器人 / 生命周期顺序错误）。

    命名带下划线是为了**不遮蔽内建 `RuntimeError`** ——
    本模块里两处都要用（这里抛 `RuntimeError_`，而 `asyncio.CancelledError`
    在旧版本里继承自 `RuntimeError`）。用一个别名型子类既清晰又安全。
    """


class RobotRuntime:
    """运行时的唯一入口（§46）。

    ## 生命周期

    ```python
    rt = RobotRuntime()
    await rt.start()                      # discovery + loading + backend start
    await rt.step("mini_arm", RobotCommand.of("mini_arm", {"shoulder": 0.5}))
    state = await rt.get_state("mini_arm")
    await rt.stop()
    ```

    `start()` 之前读 `models()` 会**抛错**而不是返回空 ——
    空列表会让"还没启动"伪装成"一台机器人都没有"。
    """

    def __init__(
        self,
        packages_dir: Path | None = None,
        backend_factory: BackendFactory = default_backend_factory,
    ) -> None:
        self._packages_dir = packages_dir
        self._backend_factory = backend_factory

        self._packages: dict[str, RobotPackage] = {}
        self._models: dict[str, RobotModel] = {}
        self._reports: dict[str, ValidationReport] = {}
        self._backends: dict[str, RobotBackend] = {}

        self._started = False
        #: 串行化对 backend 的访问。理由见 `step()` 的 docstring。
        self._lock = asyncio.Lock()

    # ------------------------------------------------------------------
    # 生命周期（§46 Runtime lifecycle）
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """发现包 → 加载模型 → 校验 → 为每个模型建 Backend 并启动。幂等。

        ## 为什么校验只**报告**不**拒绝**

        `RobotPackage.load_model()` 返回 `(model, report)` 且
        `report.ok is False` 时仍然给 model（见 registry.py 的注释）。
        Runtime 沿用同一策略：**坏模型也进 `models()`**，
        但 `validation_report(id)` 会带着 issues。
        理由与 API 层一致 —— 前端需要"显示为什么画不出来"，
        而静默丢弃会让一台机器人凭空消失。
        """
        if self._started:
            return

        pkgs = discover_packages(self._packages_dir)  # 可能抛 RobotPackageError
        for pkg in pkgs:
            model, report = pkg.load_model()
            self._packages[pkg.id] = pkg
            self._models[pkg.id] = model
            self._reports[pkg.id] = report

        for robot_id, model in self._models.items():
            backend = self._backend_factory(model)
            await backend.start()
            self._backends[robot_id] = backend

        self._started = True

    async def stop(self) -> None:
        """停掉所有 Backend。幂等。

        逐个 try/except 而不是一次性 `gather`：一个 Backend 关闭失败
        **不能**阻止其它 Backend 被关闭（否则会漏掉资源释放）。
        """
        errors: list[str] = []
        for robot_id, backend in self._backends.items():
            try:
                await backend.stop()
            except Exception as exc:  # noqa: BLE001 - 关闭阶段要尽力而为
                errors.append(f"{robot_id}: {exc}")
        self._backends.clear()
        self._started = False
        if errors:
            # 抛在最后：既完成了全部清理，又不静默吞掉失败
            raise BackendError("停止 Backend 时有失败：" + "; ".join(errors))

    async def reset(self, robot_id: str | None = None) -> None:
        """复位一个或全部机器人。"""
        self._ensure_started()
        targets = [robot_id] if robot_id is not None else list(self._backends)
        for rid in targets:
            await self._backend(rid).reset()

    # ------------------------------------------------------------------
    # RobotModel management（§46）
    # ------------------------------------------------------------------

    def packages(self) -> list[RobotPackage]:
        """已发现的包（轻量，不含几何）。**要求已启动**。"""
        self._ensure_started()
        return sorted(self._packages.values(), key=lambda p: p.id)

    def models(self) -> dict[str, RobotModel]:
        """已加载的全部模型，按 id 排序（dict 保序）。"""
        self._ensure_started()
        return {rid: self._models[rid] for rid in sorted(self._models)}

    def model(self, robot_id: str) -> RobotModel:
        self._ensure_started()
        if robot_id not in self._models:
            raise RuntimeError_(
                f"未加载机器人 {robot_id!r}；可用：{sorted(self._models)}"
            )
        return self._models[robot_id]

    def has_robot(self, robot_id: str) -> bool:
        return robot_id in self._models

    def validation_report(self, robot_id: str) -> ValidationReport:
        self._ensure_started()
        if robot_id not in self._reports:
            raise RuntimeError_(f"未加载机器人 {robot_id!r}；可用：{sorted(self._reports)}")
        return self._reports[robot_id]

    def backend(self, robot_id: str) -> RobotBackend:
        """取出该机器人的 Backend（测试与诊断用；正常路径走 `step`/`get_state`）。"""
        self._ensure_started()
        return self._backend(robot_id)

    # ------------------------------------------------------------------
    # RobotCommand / RobotState（§46）
    # ------------------------------------------------------------------

    async def send_command(self, command: RobotCommand) -> None:
        """把命令交给对应 Backend。**不返回状态**（§28：命令不代表执行结果）。

        想"发完就知道它到哪了"用 `step()` —— 把两件事分开，
        是因为真实场景里它们本来就不该同步（真机要等好几个控制周期）。
        """
        await self._dispatch(command)

    async def step(self, command: RobotCommand) -> RobotState:
        """一步闭环：发命令 → 读状态。

        ## 为什么用锁串行化

        `asyncio.Lock` 保证"发命令 → 推进 → 读状态"这三步
        不会被另一个客户端的命令插在中间。没有锁时的典型 bug：
        客户端 A 发完命令、客户端 B 也发命令，A 读状态时拿到的是**B 命令**的结果
        —— 表现为"A 的指令时灵时不灵"，而且只在两个客户端同时操作时出现。

        锁是 per-Runtime 的（不是 per-robot）：v0.1 的 Backend 是单线程仿真，
        一台机器人的物理步进在同一时刻只应该有一个发起者。
        """
        async with self._lock:
            backend = self._backend_for_command(command)
            command.validate_against(self.model(command.robot).mobile_joint_ids())
            await backend.send_command(command)
            return await backend.get_state()

    async def get_state(self, robot_id: str) -> RobotState:
        self._ensure_started()
        return await self._backend(robot_id).get_state()

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _ensure_started(self) -> None:
        if not self._started:
            raise RuntimeError_(
                "RobotRuntime 尚未 start()。未启动时返回空列表会把"
                "『还没启动』伪装成『一台机器人都没有』，所以这里直接抛错。"
            )

    def _backend(self, robot_id: str) -> RobotBackend:
        if robot_id not in self._backends:
            raise RuntimeError_(
                f"没有机器人 {robot_id!r} 的 Backend；"
                f"可用：{sorted(self._backends)}"
            )
        return self._backends[robot_id]

    def _backend_for_command(self, command: RobotCommand) -> RobotBackend:
        if not isinstance(command, RobotCommand):
            raise TypeError(f"需要 RobotCommand，实际是 {type(command).__name__}。")
        return self._backend(command.robot)

    async def _dispatch(self, command: RobotCommand) -> None:
        async with self._lock:
            backend = self._backend_for_command(command)
            command.validate_against(self.model(command.robot).mobile_joint_ids())
            await backend.send_command(command)

    # ------------------------------------------------------------------
    # 诊断
    # ------------------------------------------------------------------

    def summary(self) -> dict[str, Any]:
        """一句可打印的运行时概况（CLI / 日志 / 测试断言用）。"""
        return {
            "started": self._started,
            "robots": sorted(self._models),
            "backends": {
                rid: type(b).__name__ for rid, b in sorted(self._backends.items())
            },
        }


__all__ = [
    "BackendFactory",
    "RobotRuntime",
    "RuntimeError_",
    "default_backend_factory",
]
