"""Core 运动学引擎。

```text
kinematics   FK（通用链式）                  ← 依赖 model
model        RobotModel / 契约对象           ← 不依赖任何上层
```

## 为什么 FK 能进 Core 而 IK 不能

这是个**数学事实**，不是工作量问题：

| | 可通用化 | 原因 |
|---|---|---|
| FK | ✅ | 沿 Link-Joint 链逐级乘 `Transform`，对**任何** Tree 型模型都成立 |
| IK | ❌ | 闭式解依赖具体机构的几何（如平面 2R 的余弦定理配对、分支判据）；<br>通用 IK 只能是**数值迭代**，而迭代会引入容差 |

因此 v0.1 的分工是：

```text
backend/kinematics/           FK 通用引擎（唯一实现）
packages/<robot>/kinematics/  该型号的解析 FK/IK（可选，用于交叉验证与精确往返）
```

`manifest.yaml` 的 `kinematics.ik.type` 字段把这个分工编码成**显式声明**
（`package` / `engine`），于是"哪台机器人有解析 IK"不需要读代码就能知道。

## 分层禁令（与 backend/__init__.py 一致）

本包**不得** import `mujoco` / `three` / `fastapi` / `runtime`。
运动学是纯数学 —— 一旦它依赖仿真器，"同样的关节角在 FK 与 MuJoCo 里
得到不同位姿"这类问题就再也无法定位。
"""

from .fk import forward_kinematics, link_transforms

__all__ = ["forward_kinematics", "link_transforms"]
