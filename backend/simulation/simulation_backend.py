"""仿真后端的**引擎无关**公共部分（提示词 §50）。

## 为什么要有一层"引擎无关"

v0.1 只有 MuJoCo 一个引擎，所以抽这层看起来像过度设计。
真正的理由是**它把两件容易混淆的事分开**：

| 关注点 | 属于 | 在哪 |
|---|---|---|
| 关节顺序 / 单位 / 限位 / 状态装配 | 引擎无关 | 本文件 |
| 积分、接触、执行器动力学 | 引擎相关 | `mujoco_backend.py` |

RobotForge 的 `RobotModel` 已经规定了关节**顺序即语义**
（`mobile_joint_ids()` 的顺序就是 FK/qpos 的顺序）。把
"命令字典 → 有序向量"和"有序向量 → 状态字典"这两步写在引擎无关层，
意味着它们**只有一份实现**：

* 换引擎（未来可能加 `PyBulletBackend` / `GenesisBackend`）时，
  "关节顺序配错"这个 bug 类别不会在新引擎里重生；
* 顺序相关的契约由 `tests/test_mujoco.py::TestJointOrder` 统一钉住。

## 为什么不干脆把它变成 `RobotBackend` 的基类

`RobotBackend`（§47）是**接口**，不是"带插件的基类"。让
`RobotBackend` 承担关节映射逻辑，会把"所有 Backend 都必须是仿真
或都有关节"变成隐含假设 —— 而 §47 明确要求真机 Backend 也能实现它
（真机可能只有 2 个关节上报位置）。所以这一层是**独立的 mixin/工具**，
真机 Backend 可以选择用、也可以选择不用。

## 单位

P0 契约：关节位置 revolute→rad、prismatic→m；速度 rad/s 或 m/s。
MuJoCo 用的是同一套 SI 单位，所以这里**没有**单位换算 ——
唯一的换算点是四元数顺序，而它被限制在 `mujoco_backend.py` 单点
（§40/§35，见 `tests/test_mujoco.py::TestQuaternionBoundary`）。
"""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from ..model.robot_model import RobotModel
from ..runtime.state import RobotState

#: 仿真后端默认的物理推进步长（秒）。与 MJCF 的 `option/timestep` **无关**：
#: 前者是"一次 `step()` 推进多久"，后者是"积分器每一步多久"。
#: 比值 = 子步数。见 `mujoco_backend.py` 里对子步必要性的说明。
DEFAULT_STEP_SECONDS = 0.02


class JointOrderError(ValueError):
    """关节顺序/集合与模型不一致。

    单列一个异常是因为它是**内部一致性错误**，不是用户输入错误：
    它的成因只有"引擎里的关节表和 RobotModel 的关节表不同步"，
    而这类错误的表现（"发 shoulder 动 elbow"）最难从现象反推。
    """


def ordered_joint_ids(model: RobotModel) -> list[str]:
    """`RobotModel` 里可动关节的**规范顺序**。

    它是全项目关节向量的唯一顺序定义 —— 命令展开、状态装配、
    与引擎 `qpos` 的对账，三处都引用它。
    """
    return model.mobile_joint_ids()


def positions_to_vector(
    model: RobotModel, positions: Mapping[str, float]
) -> list[float]:
    """`{joint_id: value}` → **按模型顺序**的向量。

    缺失的关节按 `initial` 语义处理不了（我们不知道初始值），
    所以这里**要求齐全**：缺一个就报错，而不是补 0。
    补 0 会让"状态里少了 shoulder"表达成"shoulder 在 0 rad" ——
    而那是一个完全合法的位形（见 `state.py::position_vector` 的同一论证）。
    """
    ids = ordered_joint_ids(model)
    missing = [jid for jid in ids if jid not in positions]
    if missing:
        raise JointOrderError(
            f"位置字典缺少关节 {missing}；模型可动关节为 {ids}。"
            f"（补 0 会把『缺数据』伪装成『关节在零位』，故这里显式报错。）"
        )
    extra = sorted(set(positions) - set(ids))
    if extra:
        # 多余键**忽略**（与 FK 的契约一致：换机器人时上层可能还持有旧位姿字典），
        # 但要求调用方知道这一点 ⇒ 记录在返回值的语义里而非静默丢弃。
        pass
    return [float(positions[jid]) for jid in ids]


def vector_to_positions(
    model: RobotModel, vector: Sequence[float]
) -> dict[str, float]:
    """**按模型顺序**的向量 → `{joint_id: value}`。

    长度不符 ⇒ 报错。长度不符只可能来自"引擎的 dof 与模型不同步"，
    静默截断会让一个配错关节的机器人安静地跑起来。
    """
    ids = ordered_joint_ids(model)
    if len(vector) < len(ids):
        raise JointOrderError(
            f"引擎状态只有 {len(vector)} 个分量，模型有 {len(ids)} 个可动关节 {ids}。"
        )
    return {jid: float(vector[i]) for i, jid in enumerate(ids)}


def assemble_state(
    model: RobotModel,
    positions: Mapping[str, float],
    velocities: Mapping[str, float] | None = None,
    end_effector_pose: Any = None,
    status: str = "running",
    timestamp: float = 0.0,
) -> RobotState:
    """装配 `RobotState`（§29）。

    之所以不直接调 `RobotState.of(...)`：这里要保证**关节集合与模型
    完全一致**（多一个少一个都是 bug），而 `of()` 是宽进的外部入口。
    """
    ids = ordered_joint_ids(model)
    pos = {jid: float(positions.get(jid, 0.0)) for jid in ids}
    vel = (
        {jid: float((velocities or {}).get(jid, 0.0)) for jid in ids}
        if velocities is not None
        else {}
    )
    return RobotState.of(
        model.metadata.id,
        joint_positions=pos,
        joint_velocities=vel,
        end_effector_pose=end_effector_pose,
        status=status,
        timestamp=timestamp,
    )


def clamp_targets_to_limits(
    model: RobotModel, targets: Mapping[str, float]
) -> tuple[dict[str, float], dict[str, float]]:
    """把目标夹到模型声明的限位内，返回 `(clamped_targets, clamped_report)`。

    ★ 为什么限幅要在**送进引擎之前**做（而不是指望引擎自己夹）：
    位置型执行器收到超出关节 range 的目标时，MuJoCo 会在**关节限位**
    处用约束力顶住 —— 结果是"关节停在限位"，看起来**像**限幅生效了，
    但引擎内部的 `ctrl` 仍然是那个越界值。一旦关节限位的软约束参数变化，
    或换成一个没有限位的引擎，"命令越界"就会变成"关节被推着乱转"。
    所以限幅是**命令语义**的一部分，属于 RobotForge 而不是引擎。

    ## ⚠️ 用**关节**限位，不用**执行器** `ctrlrange`

    两个区间在本项目里当前数值相同，但语义不同（`mini_arm.xml` 的
    `actuator` 段注释写明了这一点）：`ctrlrange` 是"命令的取值范围"，
    将来加 Actuator Mapping 后它会是 `-1..1` 之类，
    而关节 `range` 才是物理限位。

    夹到 `ctrlrange` 会产生一个非常隐蔽的后果：**MuJoCo 会静默丢弃
    越界的 `ctrl`（保持上一个值）**。于是"命令超限"表现成
    "机器人纹丝不动"，而没有任何错误 —— 这类静默失败正是本层要消灭的。
    """
    out: dict[str, float] = {}
    clamped: dict[str, float] = {}
    for jid, target in targets.items():
        joint = model.joint(jid)
        limits = joint.limits
        if limits is None or not limits.has_position_bounds():
            # 自由旋转关节（limits 为 None 或未声明边界）⇒ 不夹。
            # 把它夹到某个"默认 ±π"是**伪造**一个模型没声明的约束。
            out[jid] = float(target)
            continue
        value, was_clamped = limits.clamps(float(target))
        out[jid] = value
        if was_clamped:
            clamped[jid] = value
    return out, clamped


__all__ = [
    "DEFAULT_STEP_SECONDS",
    "JointOrderError",
    "assemble_state",
    "clamp_targets_to_limits",
    "ordered_joint_ids",
    "positions_to_vector",
    "vector_to_positions",
]
