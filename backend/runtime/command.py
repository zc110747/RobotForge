"""RobotCommand —— 统一指令契约（提示词 §28）。

## Command ≠ State（提示词 §29 / §69 规则 8、9）

```text
RobotCommand  →  Desired   我要它去哪
RobotState    →  Actual    它现在在哪
```

这两个**必须是两个类型**，因为它们的语义不同、来源不同、可信度不同：

* Command 是**请求**，可以被拒绝、被限幅、被忽略 —— 它不代表任何已发生的事
  （§28 末句："Command 不代表真实执行结果"）。
* State 是**观测**，是 Backend 反馈的事实，只能由 Backend 产生。

把两者合成一个类型（比如都用 `dict[str, float]`）看起来省事，
代价是"发出去的目标值"和"实际的关节角"在代码里变成同一个东西，
于是"指令没生效"这种 bug 会静默通过所有测试。

## 为什么用 `dict[str, float]` 而不是位置列表

`joint_targets` 是**按关节 id 索引**的映射，不是数组。
理由与 §30 的 ID Contract 同源：数组下标不是稳定引用。
`[0.1, 0.2]` 在下一次有人调换 MJCF 里两个关节的顺序后，
含义完全变了 —— 而这**不会报错**，只会让机器人动错关节。

用 id 索引，则"关节表变了"会以 `KeyError` 的形式在第一时间暴露
（见 `validate_against`）。

## 为什么 timestamp 由**发送方**给

`timestamp` 是命令**产生**的时刻（§28 单位表：`s`），不是被执行的时刻。
由 Runtime 在收到时打时间戳会导致"命令延迟"不可测 —— 而延迟正是
Sim2Real 里最需要观测的量。所以调用方（前端）负责填，
Runtime 只做校验（非负、有限）。

⚠️ 时钟基准：v0.1 由前端 `performance.now()/1000` 与后端 `time.monotonic()`
**混用**是禁止的 —— 那会产生"负延迟"。当前实现只把 timestamp 当
**不透明的不减量**传递，不跨源做减法。跨源时间对齐留待未来版本。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

#: 关节指令值的允许类型。刻意**不含** int：用户从 JSON 里传来的是 int
#: （`{"shoulder": 1}`），而 `json` 会把它解析成 int。
#: 所以接受 int 但立即转 float —— 见 `_f()`。
Number = int | float


def _f(value: Any, where: str) -> float:
    """把 JSON 里的数字转成 float，并拒绝 bool / 字符串 / NaN / inf。

    ⚠️ **必须显式拒绝 bool**：Python 里 `isinstance(True, int)` 是 `True`，
    所以 `{"shoulder": true}` 会静默变成 `1.0` 弧度 —— 一个毫无提示的
    57 度误差。这是 JSON 边界上真实存在的陷阱。
    """
    if isinstance(value, bool):
        raise TypeError(
            f"{where} 是 bool（{value!r}）。关节值必须是数字 —— "
            f"JSON 里的 true/false 不会被当作 1/0。"
        )
    if not isinstance(value, (int, float)):
        raise TypeError(
            f"{where} 的类型是 {type(value).__name__}，需要数字（rad 或 m）。"
        )
    f = float(value)
    if f != f or f in (float("inf"), float("-inf")):
        raise ValueError(f"{where} 是 {f!r}，不允许 NaN / Infinity。")
    return f


@dataclass(frozen=True)
class RobotCommand:
    """一次关节目标设定（§28）。

    ```python
    class RobotCommand:
        robot: str
        joint_targets: dict[str, float]
        timestamp: float
    ```

    `frozen=True` 让"命令一旦发出就不可改"成为运行期事实：
    Backend 收到的是它，而不是一个还会被别人改的共享字典。
    """

    robot: str
    joint_targets: dict[str, float] = field(default_factory=dict)
    timestamp: float = 0.0

    # ---- 构造 ----

    @classmethod
    def of(
        cls,
        robot: str,
        joint_targets: Mapping[str, Number] | None = None,
        timestamp: Number = 0.0,
    ) -> "RobotCommand":
        """从"不干净"的输入构造（WS 帧 / CLI / 测试都会用这条路径）。

        `dict[str, float]` 的注解在运行期**不检查**，
        所以所有外部输入都必须过这里 —— 这是唯一的强制点。
        """
        if not isinstance(robot, str) or not robot:
            raise ValueError(f"robot 必须是非空字符串，实际是 {robot!r}。")

        raw = joint_targets or {}
        if not isinstance(raw, Mapping):
            raise TypeError(
                f"joint_targets 必须是映射（关节 id → 值），实际是 "
                f"{type(raw).__name__}。数组下标不是稳定引用（§30）。"
            )
        targets: dict[str, float] = {}
        for key, value in raw.items():
            if not isinstance(key, str) or not key:
                raise ValueError(f"关节 id 必须是非空字符串，实际是 {key!r}。")
            targets[str(key)] = _f(value, f"joint_targets[{key!r}]")

        ts = _f(timestamp, "timestamp")
        if ts < 0:
            raise ValueError(f"timestamp 必须非负（单位 s），实际是 {ts}。")
        return cls(robot=robot, joint_targets=targets, timestamp=ts)

    # ---- 校验 ----

    def validate_against(self, joint_ids: list[str]) -> None:
        """对照模型的**可动关节表**校验，不合法即抛错。

        ## 为什么未知关节 id 必须报错而不是忽略

        静默忽略一个打错的关节名（`shulder`）会产生一个"看起来成功、
        但什么都没动"的命令。用户看到的是"发了指令，机器人不动"，
        而日志里一片正常。这在 Sim2Real 上是要命的时间黑洞。

        ## 为什么不要求"必须给全所有关节"

        部分指令（只动一个关节）是合法的 —— 前端拖一个滑块就属于这类。
        未提及的关节由 Backend 保持当前位置（见 `MockBackend.send_command`）。
        """
        unknown = sorted(set(self.joint_targets) - set(joint_ids))
        if unknown:
            raise ValueError(
                f"命令里的关节 {unknown} 不在模型的可动关节表里（可用：{joint_ids}）。"
                f"静默忽略会让'打错的关节名'变成'机器人不动'，所以这里直接拒绝。"
            )

    # ---- 序列化 ----

    def to_dict(self) -> dict[str, Any]:
        """WS `robot_command` 帧的载荷（§52）。

        字段名与 §52 的 JSON 示例**逐字一致**：
        示例用 `joints`，而 §28 的 Python 类用 `joint_targets`。
        两者都对 —— 前者是**线上格式**，后者是**内部契约**。
        这个映射只在这里发生一次（避免到处写 rename）。
        """
        return {
            "robot": self.robot,
            "joints": dict(self.joint_targets),
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_ws_frame(cls, frame: Mapping[str, Any]) -> "RobotCommand":
        """从 WS 帧解析。缺字段给**明确的**错误，而不是 KeyError。

        前端是本项目自己的客户端，但一个残缺的帧不应该让连接崩掉 ——
        它应该回一个 `error` 帧（§52），所以这里抛的是可捕获的异常。
        """
        if not isinstance(frame, Mapping):
            raise TypeError(f"WS 帧必须是映射，实际是 {type(frame).__name__}。")
        if "robot" not in frame:
            raise ValueError("robot_command 帧缺少 `robot` 字段。")
        return cls.of(
            robot=frame["robot"],
            joint_targets=frame.get("joints", {}),
            timestamp=frame.get("timestamp", 0.0),
        )


__all__ = ["RobotCommand"]
