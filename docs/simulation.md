# RobotForge 仿真层 (v0.1)

> **状态：v0.1 已实现并验收（Phase 5，67/67）**
>
> 本文件描述 `backend/simulation/` 的两个文件、它们的边界、
> 以及"真物理"这条链路里几处**必须这样写**的决定。
>
> 上游契约见 `docs/runtime.md`；坐标/单位见 `docs/coordinate-system.md`。

---

## 1. 两个文件的严格分工

```text
backend/simulation/
├── simulation_backend.py   引擎无关：关节序 / 限幅 / 状态装配
└── mujoco_backend.py       引擎特定：编译 MjModel / mj_step / 四元数换算
```

| 文件 | 职责 | 依赖 |
|---|---|---|
| `simulation_backend.py` | 关节顺序、限位夹紧、状态装配 | 只依赖 `model` / `runtime.state` |
| `mujoco_backend.py` | `MjModel` 编译、`mj_step`、四元数边界换算 | 额外依赖 `mujoco` |

**为什么把"引擎无关"的那半单独放？**

为了让"**关节顺序配错**"这个 bug 类别**不会因为换引擎而重生**。
顺序定义只有一份（`model.mobile_joint_ids()`），
引擎无关的那半负责展开/装配，引擎特定的那半只负责"怎么积分"。

如果两半混在一个文件里，加第二台机器人时很容易出现
"MuJoCo 那份顺序对、Mock 那份顺序错"，而两边看起来都能跑。

---

## 2. 关节顺序：只有一个定义

```python
model.mobile_joint_ids()
```

**命令展开、状态拼装、与 MuJoCo `qposadr` 对齐 —— 全部引用它。**

`start()` 里有一条硬断言：

```python
assert qpos_addr == sorted(qpos_addr), "关节顺序与 qposadr 不一致"
```

**为什么这条断言值得单独存在？**

关节顺序错位是**最隐蔽**的一类 bug：

```text
所有测试都能跑 ✓
数值都在合法范围 ✓
只是"命令关节 A，结果关节 B 动了"
```

把顺序固定成 `qposadr` 升序并在 `start()` 断言，等于让这类错误
**在启动时就爆掉**，而不是等到某个测试恰好发现了它。

---

## 3. 限幅：夹**关节 range**，不是 actuator ctrlrange

这是整份文档里**最容易搞错**的一条。

```python
# ✅ 正确：夹关节声明的 range
joint = model.joint(jid)
limits = joint.limits
if limits.has_position_bounds():
    target, was_clamped = limits.clamps(target)
```

**为什么不能用 `ctrlrange`？**

MuJoCo 对超出 `ctrlrange` 的 `ctrl` 是**静默丢弃（保留旧值）**。
于是按 `ctrlrange` 夹会产生这样的后果：

```text
用户命令 shoulder = 1.8 rad（超出 ctrlrange）
  → 我们按 ctrlrange 把它夹到 1.5708
  → 但用户看到的"命令被接受了"
  → 而如果实现反过来（不夹、直接下发），MuJoCo 会静默忽略
  → 表现为"机器人完全不动"，且**无任何报错**
```

两种错法都很糟：前者"改小了命令但不告诉用户"，后者"命令被静默吃掉"。
**正解**是夹关节自己的 `range`（那是模型声明的物理约束），
并记录到 `.clamped` 供验收查看。

### 3.1 `limits is None` / 无界 ⇒ **不夹**

```python
if limits is None or not limits.has_position_bounds():
    pass  # 不夹
```

夹到某个默认的 ±π 是**伪造一个模型没声明的约束**。
自由旋转关节（如某些 wrist roll）不该被凭空限制。

---

## 4. `get_state()` 的末端位姿用 Core FK

```python
# ✅ 用 Core 的通用链式 FK
from ..kinematics.fk import forward_kinematics
pose = forward_kinematics(model, dict(positions))
```

**为什么不用 MuJoCo 的 `site_xpos`？**

因为 §62 的验收要求"**FK 与 Simulation 一致**"。
如果两边都读 MuJoCo 的派生量，那条验收就变成**同义反复**
（拿 MuJoCo 的输出证明 MuJoCo 的输出）。

用 Core FK 之后，两个来源互相独立：

```text
Core FK      —— 手写的链式矩阵相乘
MuJoCo       —— C++ 引擎的积分 + 自身 FK
```

两者给出相同位姿 ⇒ 说明**两边都没写错**。这是真正的交叉验证。

> ⚠️ 代价：`site_xpos` 在 `mj_step` 之后**比 `qpos` 落后一个子步**。
> 如果将来真的要用它，必须在推进后补 `mujoco.mj_forward(m, d)`
> 刷新派生量（纯运动学、不积分、不改状态）。

---

## 5. 时间步：`DEFAULT_STEP_SECONDS = 0.02` 与 MJCF `timestep` 无关

```text
DEFAULT_STEP_SECONDS = 0.02        ← RobotForge 的控制步（我们的概念）
MJCF 里的 <option timestep>        ← MuJoCo 的积分步（模型的属性）

两者相除 = 子步数（本模型 = 若干个 mj_step）
```

**永远不要改 MJCF 的 `timestep` 来"对齐"步长。**

理由：`timestep` 是**模型属性**，改了它等于改物理（接触刚度、稳定性）。
而 `DEFAULT_STEP_SECONDS` 是**接口语义**（"一次 step 推进多少秒仿真时间"）。
把两者绑在一起，会让"调整控制频率"变成"偷偷改物理"。

`DEFAULT_MAX_SUBSTEPS` 给子步数封顶，防止某个大 `dt` 请求把一帧卡死。

---

## 6. 四元数换算只在 `mujoco_backend.py` 里出现

```text
Core / RobotModel ：四元数 [x, y, z, w]   （§35 Quaternion Contract）
MuJoCo            ：四元数 [w, x, y, z]
```

换算函数：

```python
mj_quat_to_xyzw(q_wxyz) -> [x,y,z,w]
xyzw_to_mj_quat(q_xyzw) -> [w,x,y,z]
```

**两个方向都要留。** 只留单向正是当初出 bug 的原因 ——
读的时候转了一次、写的时候忘了转，于是"姿态看起来对，但仿真里机器人是镜像的"。

**验收方式**：扫描全部 `backend/` 源码，断言只有
`mujoco_backend.py` 一个文件出现这个换算。

```python
# tools/accept_phase5.py 的做法（用 tokenize 剥离注释与字符串后正则扫描）
```

> ⚠️ 用正则会误命中注释里的说明文字。必须先用 `tokenize` 剥掉
> COMMENT 与 STRING 两类 token，再扫描。
> 且**必须配扫描器元测试** —— 否则"从没匹配到任何东西"也是全绿。

---

## 7. MJCF 上两处必要的建模修正

这两处**不是数值补丁**，是**建模修正**。改回去会立刻坏。

### 7.1 `armature="0.01"`（三个关节全加）

```text
位置执行器用 kp 直接当刚度，ω = sqrt(kp / I)
近端关节在零位形下惯量极小（I → 接近 0）
⇒ ω·dt ≈ 1.55  >  2 的显式积分稳定界附近
⇒ **1 步之后 QACC / QVEL 变 NaN**
```

加 `armature` 后关节有效惯量 `I ≈ 1e-2`，`ω·dt ≈ 0.155`（约 10 倍余量）。

**为什么改 kp 不算解法**：改 kp 只是把问题**移**到另一个工作点。
`armature` 表达的是"电机转子折算到关节的惯量"—— 那是**真实存在**的物理量，
模型里本来就该有。

### 7.2 `<contact><exclude>` 8 对

```text
症状：命令 shoulder = +1.5708（正好是 range 上限）→ 只停在 0.9367
      ⇒ 关节**到不了它自己声明的限位**
根因：base_column 的视觉圆柱与上臂 capsule 自碰撞（实测 dist = -0.0013）
```

**排除自碰撞是正确做法**，因为：

1. `capabilities` 里**从未声明** `collision`；
2. 这些几何是**为好看选的**，从未做过干涉检查；
3. MuJoCo 只过滤**相邻**连杆的自碰撞，不过滤隔代（base_column 与 upper_arm 隔了一代）。

⇒ 这是"用一个未声明的约束去阻断一个已声明的约束"，属于建模缺陷。

> **将来若真要做碰撞检测**：删掉这个 `<exclude>` 块，
> 并**重新设计几何**做真正的干涉检查。不要只是删掉块就算了 ——
> 那样会把"几何本身不合法"暴露成"关节动不了"。

---

## 8. 怎么跑真物理

### 8.1 命令行

```bash
# ❌ 默认工厂 ⇒ MockBackend（无物理）
.venv/Scripts/python.exe -m uvicorn backend.api.app:create_app --factory --port 8000

# ✅ 注入 MuJoCo 工厂（§69 规则 10 的立足点）
.venv/Scripts/python.exe -m uvicorn tools.serve_mujoco:app --host 127.0.0.1 --port 8000
```

`create_app(backend_factory=mujoco_backend_factory)` ——
Runtime / WebSocket / 路由**一行不改**，只换注入的工厂。

### 8.2 怎么确认跑的是真物理

看 `simulation_state` 帧：

```json
{ "type": "simulation_state", "backends": [{ "backend": "MuJoCoBackend" }] }
```

`backend` 字段报 `MockBackend` ⇒ **这不是 §71 的 Sim2Sim 证据**。

### 8.3 真物理 vs Mock 的可观测差异（实测）

同样发 40 轮 `joints = {base_yaw: 0.4, shoulder: -0.3, elbow: 0.5}`：

| 读数 | MockBackend | MuJoCoBackend |
|---|---|---|
| 单步位移 | 0.35（硬走一步，限速值） | **0.040790**（积分出来的） |
| 误差收敛 | 0 → 一步到位 | **0.142812 → 0.002469** |
| 关节速度 | 非零但恒等于位移 | **真动力学** |
| 稳态残差 | 0（无物理） | 2.4e-3（重力下垂 + 子步残余） |

**最后一行是重点**：Mock 的稳态误差恰好为 0，
而真物理有**可解释的**残差（重力下垂）。如果 MuJoCo 版本也给出 0，
说明位置执行器的 kp 无穷大 —— 那是不符合模型的事实。

---

## 9. 独立验收

```bash
# Phase 5 全量（含上游，约 12 min；**必须后台跑**，前台会被 SIGTERM）
.venv/Scripts/python.exe -u tools/accept_phase5.py > .workbuddy/scratch/accept5.log 2>&1

# 真实 TCP 的 WS + MuJoCo 闭环（快，约 5s）
.venv/Scripts/python.exe tools/probe_ws_live.py
```

`accept_phase5.py` 的判据类型（**分类优于容差**）：

```text
不写 "|q_sim - q_fk| < TOL"
而写 "误差的**来源**能被命名：
      重力下垂 ⇒ 残差 ∈ (0, 1e-2)
      自碰撞   ⇒ 残差 > 0.5 rad（这是 bug，必须抓住）
      "
```

阈值 5e-3 能同时做到"放过物理残差"与"抓住自碰撞"
（后者超出阈值百倍）。详见 `docs/coordinate-system.md` 的判据铁律四。

---

## 10. 已知限制（v0.1）

| 项 | 状态 |
|---|---|
| 碰撞检测 | **不支持**（`capabilities.collision: false`；8 对 exclude 是有意的） |
| 接触力 / 力矩反馈 | 不支持 |
| 执行器动力学建模 | 位置执行器（kp）为主，无电机模型 |
| 传感器（IMU / 力传感器） | 不支持 |
| 抓取 / 物体交互 | 不支持 |
| 实时性保证 | 无（`step()` 是同步阻塞的） |
| 多机器人同场景 | 不支持 |
| 状态订阅推送 | 无（命令驱动 + 按需拉取） |
