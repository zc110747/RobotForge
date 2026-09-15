"""RobotForge Backend —— 通用机器人数字孪生与仿真平台。

## 分层（严格单向依赖）

```text
api          FastAPI 路由 / WebSocket      ← 只依赖 runtime
  ↓
runtime      RobotRuntime / Command / State ← 依赖 model + loaders + simulation
  ↓
simulation   MuJoCoBackend 等               ← 依赖 model + runtime 接口
loaders      MJCFLoader                     ← 依赖 model
kinematics   FK 通用引擎（纯数学）           ← 依赖 model
model        RobotModel / 契约对象          ← **不依赖任何上层**
```

## 依赖方向的三条禁令

1. `model` 不得 import `mujoco` / `three` / `fastapi` / `numpy`（除 types 的计算辅助）。
2. `runtime` 不得 import `mujoco`（它只认 `RobotBackend` 抽象）。
3. `loaders` 不得 import `runtime`（加载是纯转换，不涉及生命周期）。

违反任一条，"换模型格式"或"换后端"就会变成改全栈 —— 而避免那个成本
正是 RobotForge 存在的理由。

## `kinematics` 的位置：与 `loaders` 同级，不是 `model` 的一部分

它有两条约束，理由与上面三条同源：

4. `kinematics` 不得 import `mujoco` / `three` / `fastapi` / `runtime`。
   运动学是**纯数学**。一旦它依赖仿真器，"同样的关节角在 FK 与 MuJoCo 里
   得到不同位姿"这类问题就再也无法定位是哪一层的责任。

5. `model` 不得 import `kinematics`。
   方向必须是 `kinematics → model`（引擎读模型），不能反过来。
   若 `RobotModel` 自己带 `forward_kinematics()` 方法，"换运动学实现"
   就变成了改数据类 —— 而那正是本平台要避免的耦合。

### 为什么 FK 在 Core，而 IK 不在（v0.1 的刻意分工）

| | 位置 | 原因 |
|---|---|---|
| FK | `backend/kinematics/` | 沿 Link-Joint 链逐级乘变换，对**任何** Tree 型模型成立 |
| IK | `packages/<robot>/kinematics/` | 闭式解依赖具体机构几何；通用 IK 只能是数值迭代，会引入容差 |

`manifest.yaml` 的 `kinematics.ik.type`（`package` / `engine`）把这个分工
编码成**显式声明**，于是"哪台机器人有解析 IK"不读代码也能知道。
"""

__version__ = "0.1.0"
