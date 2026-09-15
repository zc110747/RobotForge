"""RobotState —— 统一状态契约（提示词 §29）。

## State 是**观测**，不是**期望**

```python
class RobotState:
    robot: str
    joint_positions: dict[str, float]
    joint_velocities: dict[str, float]
    end_effector_pose: Transform | None
    status: str
    timestamp: float
```

§29 用一整段强调它必须与 `RobotCommand` 区分：

```text
Command → Desired
State   → Actual
```

并且加了一句很关键的话：**"即使 v0.1 是 Sim2Sim，也必须维持该语义。"**

这句话的实践含义是：`MockBackend` 只要*能*产生与命令不同的状态，
就必须真的产生（比如限速、限幅、或未提及关节保持原位）。
如果 MockBackend 把命令原样回显成状态，那么整条链路上
"命令失败"这个失败模式就**从来没有被演练过**，
到了真机上第一次遇到时就只能靠猜。所以 `MockBackend` 是**有状态**的，
并且会限幅（见 `backend.py`）。

## end_effector_pose 为什么可以是 None

§29 写的是 `Transform | None`。两种合法情况：

1. 模型没有声明 EndEffector（`capabilities.end_effector = False`）
2. Backend 还没算出位姿（比如 `status == "idle"` 的首帧）

**不能**用一个"零位姿"（`Transform.identity()`）去冒充"不知道" ——
`Transform.identity()` 是一个**有含义**的位姿（原点、无旋转），
把它当作哨兵会让"末端真的在原点"和"不知道末端在哪"无法区分。

## status 取值

v0.1 定义四个（`STATUSES`）：

```text
idle        已启动，未收到命令
running     正在执行命令 / 仿真推进中
reset       正在复位
error       后端出错（此时 joint_positions 可能仍是上一次的可用值）
```

刻意**不**用自由字符串：`status` 会被前端用来决定"要不要把模型画成红色"，
拼写自由会让 `"Error"` / `"err"` 静默地不被识别。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from ..model.types import Transform

#: 合法的 status 取值。新增取值必须同时更新前端与测试 ——
#: 这正是把它做成常量集合的目的（让"新增"是一次显式的三方改动）。
STATUSES = ("idle", "running", "reset", "error")


def _f(value: Any, where: str) -> float:
    """同 `command._f`：拒绝 bool / 非数字 / NaN / inf。

    这里**不能**共用 `command._f` 的 import：那会让 state 依赖 command，
    而两者按 §69 规则 9 必须是**平行**的两个契约。宁可重复 12 行。
    """
    if isinstance(value, bool):
        raise TypeError(f"{where} 是 bool（{value!r}），状态值必须是数字。")
    if not isinstance(value, (int, float)):
        raise TypeError(f"{where} 的类型是 {type(value).__name__}，需要数字。")
    f = float(value)
    if f != f or f in (float("inf"), float("-inf")):
        raise ValueError(f"{where} 是 {f!r}，不允许 NaN / Infinity。")
    return f


def _floats(raw: Mapping[str, Any] | None, field_name: str) -> dict[str, float]:
    if raw is None:
        return {}
    if not isinstance(raw, Mapping):
        raise TypeError(
            f"{field_name} 必须是映射（关节 id → 值），实际是 {type(raw).__name__}。"
        )
    out: dict[str, float] = {}
    for key, value in raw.items():
        if not isinstance(key, str) or not key:
            raise ValueError(f"{field_name} 的键必须是非空字符串，实际是 {key!r}。")
        out[str(key)] = _f(value, f"{field_name}[{key!r}]")
    return out


@dataclass(frozen=True)
class RobotState:
    """某一时刻机器人状态的**快照**（§29）。"""

    robot: str
    joint_positions: dict[str, float] = field(default_factory=dict)
    joint_velocities: dict[str, float] = field(default_factory=dict)
    end_effector_pose: Transform | None = None
    status: str = "idle"
    timestamp: float = 0.0

    # ---- 构造 ----

    @classmethod
    def of(
        cls,
        robot: str,
        joint_positions: Mapping[str, Any] | None = None,
        joint_velocities: Mapping[str, Any] | None = None,
        end_effector_pose: Transform | Sequence[float] | None = None,
        status: str = "idle",
        timestamp: Any = 0.0,
    ) -> "RobotState":
        """统一的外部输入入口（WS / Backend / 测试）。"""
        if not isinstance(robot, str) or not robot:
            raise ValueError(f"robot 必须是非空字符串，实际是 {robot!r}。")
        if status not in STATUSES:
            raise ValueError(
                f"status = {status!r} 不是合法取值；允许 {list(STATUSES)}。"
                f"（自由字符串会让前端的 `status == 'error'` 静默失效。）"
            )

        pose: Transform | None
        if end_effector_pose is None:
            pose = None
        elif isinstance(end_effector_pose, Transform):
            pose = end_effector_pose
        else:
            # 允许 `[x,y,z,qx,qy,qz,qw]` 这种扁平形式（WS 上更紧凑），
            # 但**必须在类型层归一**，否则 Transform 的构造错误会推迟到用的时候才炸。
            seq = list(end_effector_pose)
            if len(seq) != 7:
                raise ValueError(
                    f"end_effector_pose 序列必须有 7 个分量 "
                    f"[x,y,z,qx,qy,qz,qw]，实际 {len(seq)} 个。"
                )
            pose = Transform.from_parts(seq[:3], seq[3:])

        ts = _f(timestamp, "timestamp")
        if ts < 0:
            raise ValueError(f"timestamp 必须非负（单位 s），实际是 {ts}。")

        return cls(
            robot=robot,
            joint_positions=_floats(joint_positions, "joint_positions"),
            joint_velocities=_floats(joint_velocities, "joint_velocities"),
            end_effector_pose=pose,
            status=status,
            timestamp=ts,
        )

    # ---- 便捷视图 ----

    def position_vector(self, joint_ids: list[str]) -> list[float]:
        """按**模型声明的顺序**取位置向量（FK / 测试用）。

        未知关节 ⇒ `KeyError`（不补 0）：补 0 会让"状态里少了肩关节"
        表达成"肩关节在 0 弧度"，而那是一个完全合法的位姿。
        """
        return [self.joint_positions[jid] for jid in joint_ids]

    def stale_like(self, status: str = "error") -> "RobotState":
        """保留观测值但改 status —— 出错时**不要丢掉最后已知的关节角**。

        前端在 `status == "error"` 时仍需把机器人画在**最后已知位置**，
        否则一次瞬时错误会让画面跳回原点（那是"信息丢失"，不是"安全"）。
        """
        return RobotState(
            robot=self.robot,
            joint_positions=dict(self.joint_positions),
            joint_velocities=dict(self.joint_velocities),
            end_effector_pose=self.end_effector_pose,
            status=status,
            timestamp=self.timestamp,
        )

    # ---- 序列化 ----

    def to_dict(self) -> dict[str, Any]:
        """WS `robot_state` 帧的载荷（§52）。

        `joints` 是**线上字段名**（同 §52 示例），内部叫 `joint_positions`。
        `end_effector` 用 `Transform.to_dict()`（`{position:[...], orientation:[...]}`），
        姿态顺序为 `[x,y,z,w]` —— 与 §35 Quaternion Contract 一致。
        """
        return {
            "robot": self.robot,
            "joints": dict(self.joint_positions),
            "velocities": dict(self.joint_velocities),
            "end_effector": (
                self.end_effector_pose.to_dict()
                if self.end_effector_pose is not None
                else None
            ),
            "status": self.status,
            "timestamp": self.timestamp,
        }


__all__ = ["RobotState", "STATUSES"]
