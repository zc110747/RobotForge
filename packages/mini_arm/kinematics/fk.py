"""mini_arm 的**正运动学**（Robot Package 内的实现，非 Core）。

## 本文件的两个函数（Phase 3 之后）

```text
forward_kinematics(model, q)          解析式闭式解  ← **本文件实现**
forward_kinematics_generic(model, q)  通用链式      ← Core 实现，此处重导出
```

## 为什么解析式仍然留在包内（而不是也搬进 Core）

Core 的通用链式 FK（`backend/kinematics/fk.py`）遍历 `RobotModel` 的 joint 链、
逐级相乘 `Transform`。本文件的 `forward_kinematics` **不**做那件事 ——
它用闭式公式直接算出末端位姿，作为通用引擎的对照基准。

这构成一种**交叉验证**（cross-check）：两条独立实现的路径若在随机位形下
逐点一致，那么"通用 FK 读对了几何"这件事就有了机器判据。
如果只依赖一条实现，FK 的 bug 会同时污染 FK 测试、IK 测试和 Sim2Sim 对比 ——
四处一起错，表现得像"全都对"。

而解析式**不能**通用化：它依赖"平面 2R + 底座偏航"这一具体机构。
所以它留在包内是正确的归属，不是技术债。

## mini_arm 的几何（真值来自 MJCF，此处是同一份数据的闭式复现）

```text
base(0,0,0)
  └─[base_yaw, 轴 +Z, 原点 (0,0,0.084)]────── shoulder_link
       └─[shoulder, 轴 +Y, 原点 (0,0,0.052)]── upper_arm
            └─[elbow, 轴 +Y, 原点 (0.103,0,0)]─ forearm_link
                 └─ fixed, 原点 (0.065,0,0)──── ee_link
                      └─ tcp site 在 (0.032,0,0)
```

关键常量（**与 MJCF 数值一致，由包内测试断言**）：

```text
BASE_HEIGHT    = 0.084   # base → base_yaw 的高度
SHOULDER_OFFSET= 0.052   # base_yaw → shoulder 的高度
L1             = 0.103   # shoulder → elbow（上臂长）
L2             = 0.065   # elbow → ee_link
L_TOOL         = 0.032   # ee_link → tcp
```

## 坐标系与轴向

P0 契约：`+X = 前`，`+Y = 左`，`+Z = 上`（右手系）。肩/肘绕 **+Y** 旋转
⇒ 连杆在 **XZ 平面**内运动。

绕 +Y 转 θ 的旋转矩阵（右手定则）：

```text
        |  cosθ   0   sinθ |
Ry(θ) = |   0     1    0   |
        | -sinθ   0   cosθ |
```

⇒ 单位 +X 向量变为 `(cosθ, 0, -sinθ)`。
即 **θ > 0 时末端向 -Z（下）方向偏转**。选择"绕 +Y"而非"绕 -Y"是刻意的，
因为它使 θ 的符号语义为"俯仰角向下为正"，与 MJCF 的 `range` 符号一致。

底座偏航绕 **+Z**：

```text
        | cosφ  -sinφ  0 |
Rz(φ) = | sinφ   cosφ  0 |
        |  0      0    1 |
```
"""

from __future__ import annotations

import math

from backend.model.robot_model import RobotModel
from backend.model.types import Quaternion, Transform, Vector3

#: 几何常量（m）。与 `model/mini_arm.xml` 逐项对应，测试断言二者一致。
BASE_HEIGHT = 0.084
SHOULDER_OFFSET = 0.052
L1 = 0.103
L2 = 0.065
L_TOOL = 0.032

#: 关节顺序（= RobotModel.mobile_joint_ids() 的期望值，也是 MuJoCo qpos 顺序）
JOINT_ORDER = ("base_yaw", "shoulder", "elbow")


def _q_revolute(axis: Vector3, angle: float) -> Quaternion:
    """绕任意 axis 转 angle。三个关节的轴都是基轴，但保留通用形式。"""
    return Quaternion.from_axis_angle(axis, angle)


def _axis_of(model: RobotModel, joint_id: str) -> Vector3:
    """从**模型**取轴，而不是硬编码 —— 这样 MJCF 改了轴，FK 自动跟上。

    这是本文件与"把公式写死"的关键区别：几何常量（长度）必须与 MJCF 同步
    （由测试保证），但**轴的方向**直接从模型读，因为轴错是最难发现的错误
    （它不会让结果变成 NaN，只会让机械臂朝错误方向运动）。
    """
    return model.joint(joint_id).axis


def _origin_of(model: RobotModel, joint_id: str) -> Vector3:
    return model.joint(joint_id).origin.position


def tcp_offset(model: RobotModel, site_id: str = "tcp") -> Vector3:
    """tcp site 相对其父 link 的偏移（从模型读，不硬编码）。"""
    return model.site(site_id).transform.position


def forward_kinematics(model: RobotModel, joint_positions: dict[str, float]) -> Transform:
    """正解：关节角 → **TCP** 位姿（世界系 / base 系，二者同向）。

    ## 输入

    `joint_positions`：`{joint_id: value}`，单位由关节类型决定
    （revolute → rad，prismatic → m，见 P0 契约 §3.1）。
    **未提供的关节按 0 处理** —— 这是刻意的：0 是"零位"，
    而"缺一个关节就报错"会让 UI 在初始化时（还没拿到全部状态）无法渲染。

    ## 输出

    `Transform`：position 为 TCP 在 base 系中的位置（m），
    orientation 为 ee_link 的姿态（四元数 `[x,y,z,w]`）。

    ⚠️ **orientation 不含工具偏移的旋转**：tcp 相对 ee_link 只有平移
    （`L_TOOL` 沿 +X），没有旋转。因此 ee_link 的姿态 = TCP 的姿态。
    如果将来 tcp 带旋转，此处必须补上 —— 包内测试
    `test_fk_matches_mujoco` 会在那时失败，从而提醒改这里。

    ## 实现方式（为什么分成两段）

    ```text
    ① 平面 2R（肩 + 肘）在 XZ 平面内算出 r, z
    ② 底座偏航 φ 把 (r, 0, z) 绕 Z 旋到 (r·cosφ, r·sinφ, z)
    ```

    这比"逐级乘 4 个矩阵"更短，也更容易核对 —— 而它的正确性由
    `test_fk_cross_check_generic_engine` 与"逐级矩阵法"互证。
    """
    angle_yaw = joint_positions.get("base_yaw", 0.0)
    theta1 = joint_positions.get("shoulder", 0.0)
    theta2 = joint_positions.get("elbow", 0.0)

    # ---- ① 平面 2R：肩 θ1、肘 θ2（都绕 +Y）----
    # 上臂：从 shoulder 原点沿 +X 走 L1，经 Ry(θ1) 后
    #   x += L1·cos(θ1),  z -= L1·sin(θ1)
    # 前臂：从肘沿 +X 走 (L2 + L_TOOL)，经 Ry(θ1+θ2) 后同理

    # 相对 shoulder 原点的平面位移（+X 为"前方"，+Z 为"上"）
    r_arm = L1 * math.cos(theta1) + (L2 + L_TOOL) * math.cos(theta1 + theta2)
    z_arm = -(L1 * math.sin(theta1) + (L2 + L_TOOL) * math.sin(theta1 + theta2))

    # ---- 抬高到 base 系：加上底座高度与肩偏移 ----
    # 注意：shoulder 原点在 base_yaw 旋转坐标系里位于 (0,0,SHOULDER_OFFSET)，
    # 而绕 +Z 旋转不改变 Z 分量 ⇒ z 可以直接相加。
    r = r_arm
    z = BASE_HEIGHT + SHOULDER_OFFSET + z_arm

    # ---- ② 底座偏航：把平面半径 r 绕 Z 转到 (x, y) ----
    x = r * math.cos(angle_yaw)
    y = r * math.sin(angle_yaw)

    # ---- 姿态 ----
    # ee_link 的朝向 = Ry(θ1) · Ry(θ2) = Ry(θ1+θ2)（同轴旋转可加）
    # 再经底座 Rz(φ)：
    #   q = Rz(φ) ∘ Ry(θ1+θ2)   （先俯仰，再随底座偏航）
    q_roll = Quaternion.identity()  # 本机构无 roll 自由度，显式写出以对齐 RPY 语义
    q_pitch = _q_revolute(_axis_of(model, "shoulder"), theta1 + theta2)
    q_yaw = _q_revolute(_axis_of(model, "base_yaw"), angle_yaw)
    orientation = (q_yaw * q_pitch * q_roll).normalized("FK orientation")

    return Transform(Vector3(x, y, z), orientation)


# =============================================================================
# 通用链式 FK —— **已提升进 Core**（Phase 3）
# -----------------------------------------------------------------------------
# 这两个函数曾经在**本文件里实现**，作为 Core 通用引擎的原型。Phase 3 把它们
# 搬进了 `backend/kinematics/fk.py`，此处只做重导出。
#
# ## 为什么要重导出而不是直接删掉
#
# ① **交叉验证是包的核心价值**。解析式与通用链式是两条**独立实现**，
#    在随机位形下逐点一致 ⇒ "通用 FK 读对了 mini_arm 的几何"这件事才有
#    机器判据。只留一条实现的话，FK 的 bug 会同时污染 FK 测试、IK 测试和
#    Sim2Sim 对比 —— 四处一起错，看起来像"全都对"。
#
# ② **保持现有调用点不动**。`ik.py` 通过 `_load_sibling_fk()` 按文件路径
#    加载本文件，包内测试也从这里取通用版；重导出让这些路径继续可用，
#    于是"提升进 Core"是一次**无行为变化**的重构 —— 这一点由
#    Phase 1/2 的全量基线零回退来证明。
#
# ## ⚠️ 一个已经实测确认过的兼容性前提
#
# `ik.py` 是**按绝对文件路径**加载本文件的（`packages/` 下刻意没有
# `__init__.py`，见 ik.py 的 `_load_sibling_fk` 文档）。按路径加载时
# 本文件不在任何包的命名空间里，因此下面这条 `from backend... import`
# 能否成功，取决于 `backend` 是否已在 `sys.modules` 里。
#
# 已实测：**可以**（因为调用方必然已经 import 过 `backend.model.*`，
# Python 的 import 系统先查 `sys.modules`）。若将来有人在"完全不 import
# backend 任何东西"的上下文里按路径加载本文件，这里会失败 ——
# 那时的修法是把解析式 FK 也一并提升，而不是在这里加 try/except 掩盖。
# =============================================================================

from backend.kinematics.fk import (  # noqa: E402  （见上方兼容性说明）
    forward_kinematics as _core_forward_kinematics,
    link_transforms as _core_link_transforms,
)

#: 通用链式 FK（Core 引擎）。与上面的解析式 `forward_kinematics` 互证。
forward_kinematics_generic = _core_forward_kinematics

#: 全部 link 的世界位姿（Core 引擎；前端渲染用）。
link_transforms = _core_link_transforms


__all__ = [
    "BASE_HEIGHT",
    "JOINT_ORDER",
    "L1",
    "L2",
    "L_TOOL",
    "SHOULDER_OFFSET",
    "forward_kinematics",
    "forward_kinematics_generic",
    "link_transforms",
    "tcp_offset",
]
