# RobotModel 契约 (P0 · Canonical Internal Representation)

> **状态：已冻结 (FROZEN, v1)**
>
> `RobotModel` 是 RobotForge 的**唯一**统一内部机器人表示。
> 所有机器人描述格式（MJCF / URDF / Future）最终必须转换成它；
> 所有消费者（FK / IK / Runtime / MuJoCo Adapter / Three.js Renderer / Validator）只依赖它。

---

## 1. 它在架构中的位置

```text
MJCF ──┐
       │
URDF ──┼──→ RobotModel ──→ Runtime ──→ Backend
       │   (Canonical)      │
Future ┘                    ├── FK
                            ├── IK
                            └── Renderer
```

### 1.1 Runtime 禁止直接依赖的清单

```text
MJCF XML 元素树
URDF XML 元素树
MuJoCo MjModel / MjData
Three.js Object3D
串口 / CAN / USB 句柄
```

只要 `RobotRuntime` 里出现以上任一类型，架构即破——因为"换一个模型格式"
就会变成"改 Runtime"，而那正是 RobotForge 要消灭的成本。

### 1.2 冻结的含义（最重要的一条）

> 契约一旦冻结，**不允许** MJCF Loader / Frontend / MuJoCo Backend
> **各自私自扩展**另一套模型字段。

具体表现：

```python
# ❌ 禁止：Loader 里偷偷塞私货
model._mjcf_raw = tree
model._body_names = [...]

# ❌ 禁止：Backend 里针对某机器人特判
if model.metadata.id == "mini_arm": ...

# ❌ 禁止：Frontend 里硬编码关节名
const JOINTS = ["base_yaw", "shoulder", "elbow"]
```

正确做法：需要新信息 ⇒ **升级本契约**（加字段 + 加测试 + 记变更），
而不是在某一端打补丁。

---

## 2. 顶层结构

```python
class RobotModel:
    metadata: RobotMetadata
    coordinate: CoordinateConvention
    units: UnitConvention

    root_link: str
    base_frame: str

    links: list[Link]
    joints: list[Joint]
    actuators: list[Actuator]

    frames: list[Frame]
    sites: list[Site]
    end_effectors: list[EndEffector]

    capabilities: RobotCapabilities
```

逻辑 JSON：

```json
{
  "metadata": {},
  "coordinate": {},
  "units": {},
  "root_link": "base",
  "base_frame": "base",
  "links": [],
  "joints": [],
  "actuators": [],
  "frames": [],
  "sites": [],
  "end_effectors": [],
  "capabilities": {}
}
```

> **注意 `frames` 的设计**：`Frame` 是**显式注册表**，不是"所有坐标系的集合"。
> `links` / `joints` / `sites` 的父坐标系由各自 `parent` 字段隐式表达；
> `frames` 只承载**额外的**参考系（world / base / tool / sensor）。
> 这样"某名字是不是合法 Frame"有唯一答案，而不必遍历三张表去猜。

---

## 3. 各字段契约

### 3.1 RobotMetadata

```python
class RobotMetadata:
    id: str
    name: str
    version: str
    description: str | None
```

### 3.2 CoordinateConvention / UnitConvention

见 `coordinate-system.md`（v0.1 固定值，不接受自定义）。

### 3.3 Link

```python
class Link:
    id: str
    name: str
    parent_joint: str | None      # 根 Link ⇒ None
    child_joints: list[str]
    inertial: Inertial | None
    visual: list[GeometryRef]
    collision: list[GeometryRef]
```

### 3.4 Joint

```python
class Joint:
    id: str
    name: str
    type: str                      # fixed | revolute | prismatic
    parent_link: str
    child_link: str
    origin: Transform              # Joint Frame 相对父 Link
    axis: Vector3                  # 归一化，RobotForge 坐标系
    limits: JointLimits | None
    mimic: MimicJoint | None       # 未来机械耦合，v1 恒为 None
```

| 字段 | 必须 | 说明 |
|---|---|---|
| `id` | ✅ | Joint 唯一 ID |
| `name` | ✅ | Joint 名称（UI 显示） |
| `type` | ✅ | `fixed` / `revolute` / `prismatic` |
| `parent_link` | ✅ | 父 Link |
| `child_link` | ✅ | 子 Link |
| `origin` | ✅ | Joint Frame 相对父 Link |
| `axis` | ✅ | Joint 运动轴（`fixed` 时无意义，写 `[0,0,1]`） |
| `limits` | ❌ | 运动限制 |
| `mimic` | ❌ | 未来机械耦合 |

**`fixed` 关节的 `axis`**：契约要求字段存在（保持结构统一，避免 `None` 检查散落各处），
但值**不参与运动学**。Validator 不对其做归一化外的检查。

### 3.5 JointLimits

```python
class JointLimits:
    position_min: float | None
    position_max: float | None
    velocity_max: float | None
    effort_max: float | None
```

`None` 的语义是**未声明**（不是 0，也不是无穷）。
`IK` 在 `position_min/max` 为 `None` 时视为**无限制**，但这会让解空间可能无界 ⇒
Validator 对 `revolute` 关节**要求**声明 limits（warning 级），因为"没有限位的关节"
在真实机构里不存在，缺它通常是建模疏漏。

### 3.6 Actuator

```python
class Actuator:
    id: str
    name: str
    type: str                      # position | velocity | torque | motor
    joint: str | None
    command_min: float | None
    command_max: float | None
```

> **v0.1：`Joint ≈ Actuator`，但概念不能合并。**

理由：未来要支持

```text
Joint ──→ Actuator Mapping ──→ Servo
                 │
                 ├── offset
                 ├── direction
                 ├── scale
                 ├── coupling
                 └── mechanical linkage
```

如果 v0.1 把二者合并成一个对象，未来加映射表就必须改 `Joint` 契约 ⇒
所有已冻结的下游全部要动。保留两个类型 = 未来只需在 `Actuator` 上挂 mapping。

### 3.7 Frame / Site / EndEffector

```python
class Frame:
    id: str
    name: str
    parent: str
    transform: Transform

class Site:
    id: str
    name: str
    parent: str
    transform: Transform
    role: str | None               # 如 "end_effector"

class EndEffector:
    id: str
    name: str
    frame: str                     # 必须存在于 frames
    site: str | None               # 必须存在于 sites
```

> **EndEffector 不重复保存 Pose**（提示词 §23）。

这是刻意设计：如果 EndEffector 自己也存一份 `Transform`，那么它与
`Frame.transform` 就有了**两份真值**。改了 Frame 忘了改 EndEffector ⇒
UI 显示的位置与实际 FK 位置不一致，而这种 bug 表现为"IK 收敛但位置偏差"，
极难定位。因此 EndEffector **只做引用**。

### 3.8 GeometryRef

```python
class GeometryRef:
    type: str                      # box | sphere | capsule | cylinder | cylinder | mesh
    asset: str | None
    transform: Transform
    size: list[float]              # 语义随 type 变化
    rgba: list[float] | None       # 显示颜色 [r,g,b,a]，0~1
```

`size` 的语义（对齐 MuJoCo `geom` 的约定，避免二次解释）：

| type | size | 含义 |
|---|---|---|
| `box` | `[hx, hy, hz]` | **半长**（half-extent） |
| `sphere` | `[r]` | 半径 |
| `cylinder` | `[r, half_len]` | 半径 + 半长 |
| `capsule` | `[r, half_len]` | 半径 + 半长 |
| `mesh` | `[]` | 用 `asset` 引用，v0.1 不实现加载 |

v0.1 只支持上表前五种（MJCF 原生 geom）。
`STL / OBJ / GLTF / GLB / STEP` 属于未来 `GeometryAsset` + `AssetLoader` 扩展点。

### 3.9 Inertial

```python
class Inertial:
    mass: float                    # kg
    center_of_mass: Vector3        # m
    ixx: float
    iyy: float
    izz: float
    ixy: float
    ixz: float
    iyz: float
```

单位：`kg` / `m` / `kg*m²`。

> **如果 MJCF 没有显式数据，不要随意伪造**（提示词 §25）。
> `MJCFLoader` 对缺失 inertia 的处理是**留 `None`**，并在 Build Report 中报告；
> 不自动按几何体填充——那会让"我没写惯量"这个事实消失，
> 而 MuJoCo 自己会按 geom 估算（它的估算比我们的伪造更权威）。

### 3.10 RobotCapabilities

```python
class RobotCapabilities:
    simulation: bool
    fk: bool
    ik: bool
    actuator_control: bool
    end_effector: bool
```

> **Frontend 必须根据 capability 动态决定显示哪些功能。**

```tsx
// ❌ 禁止
{robot.id === 'mini_arm' && <IkPanel />}

// ✅ 正确
{model.capabilities.ik && <IkPanel />}
```

这样"加一台没有 IK 的机器人"只需在它的 manifest 写 `ik: false`，
前端**自动**不显示 IK 面板，不必改一行前端代码。

> **capability 与真实性的关系**：`capabilities` 是**声明**，不是**保证**。
> `RobotModelValidator` 会交叉检查（如 `ik: true` 但没有 end_effector ⇒ warning），
> 但最终"能不能解"由 FK/IK 的实际测试决定。
> 声明与实现不一致是**测试的职责**，不是靠人工维护一致性。

---

## 4. 必须分离的三件事

```text
┌─────────────────────────────┐
│         RobotModel          │  ← 机器人**是什么**（结构 / 运动学 / 几何 / 能力）
└─────────────────────────────┘
              │
   ┌──────────┴──────────┐
   ▼                     ▼
┌─────────────────┐   ┌─────────────────┐
│ RobotCommand    │   │ RobotState      │
│ 希望做什么       │   │ 实际发生什么     │
└─────────────────┘   └─────────────────┘
```

| 对象 | 语义 | 生命周期 |
|---|---|---|
| `RobotModel` | 机器人是什么 | 加载后**不变**，可跨会话缓存 |
| `RobotCommand` | 用户希望机器人做什么 | 瞬时，由用户发出 |
| `RobotState` | 机器人实际处于什么状态 | 随时间变化，由 Backend 产生 |

> **即使 v0.1 是 Sim2Sim，也必须维持该语义**：`Command → Backend → State`
> 即使 MuJoCo 一步就跟上，也**不**允许把 State 直接等于 Command。
> 因为一旦如此，将来接真机（有延迟、有误差、有丢步）就必须改协议与前端，
> 而那时改动面已经扩散到全栈。

---

## 5. RobotModel 不得包含的内容

```text
WebSocket connection          Runtime status
Current joint state           Current velocity
Current simulation time       MuJoCo MjData / viewer
Three.js Object3D             React state
Serial / CAN / TCP connection asyncio Task
```

> `RobotModel` 是**纯机器人结构、运动学、几何和能力描述**。

### 5.1 为什么这条纪律值得单独列一节

因为违反它**太方便了**：

```python
# ❌ 看起来很方便
model.runtime = MujocoRuntime(model)
```

但这样做的后果是：

1. `RobotModel` 变得**不可序列化**（发给前端时崩）；
2. 多会话时同一份模型被多个 Runtime 争用（状态互相污染）；
3. 测试里"构造一个 RobotModel"必须先起 MuJoCo（测试变慢且脆弱）。

因此 `RobotModel` 用 `@dataclass(frozen=True)` 实现——**物理上**阻止了
`model.xxx = ...` 这类写入。冻结数据类不是风格偏好，是**契约的强制执行机制**。

---

## 6. ID 契约

所有 RobotModel 对象拥有稳定 ID。

### 6.1 规则

```text
唯一     在同一类型内不重复
稳定     同一模型每次加载结果相同
ASCII    仅 [a-z0-9_]
snake_case
```

### 6.2 禁止作为长期引用的东西

```text
数组下标               # joints[2] —— 加一个关节就全错位
显示名称               # "Shoulder Joint" —— UI 一改就断
内存地址               # id(model)
运行时随机 UUID        # 每次加载都不同，无法写进测试
```

### 6.3 ID 与 Name 分离

```json
{ "id": "shoulder_joint", "name": "Shoulder Joint" }
```

`id` = 程序引用；`name` = UI 显示。
**UI 重命名不得导致内部引用失效**——这正是把二者分开的全部意义。

> **与 MuJoCo 名 `name` 的关系**：MJCF 里的 `name` 属性是机器人作者写的，
> 可能含空格/大写/中文。`MJCFLoader` 的规则是：
> ① 优先用 MJCF 的 `name`，经 `sanitize_id()` 规范化后作为 `id`；
> ② `name` 保留 MJCF 原值用于显示；
> ③ 若规范化后**冲突**（如 `Arm 1` 与 `Arm-1` 都变成 `arm_1`）⇒ **报错，不自动加后缀**。
> 自动加后缀会让"两台连杆的 id 悄悄变成 `arm_1_2`"变成静默行为，
> 而模型作者的意图是"我的模型有命名冲突"——这个事实必须被看见。

---

## 7. Link / Joint Tree 契约

v0.1 的 `RobotModel` 是 **Joint-Link Tree**：

```text
Link
 ↓
Joint
 ↓
Link
```

示例：

```text
base
 │
 └── base_yaw_joint
       │
       └── shoulder_link
             │
             └── shoulder_joint
                   │
                   └── forearm_link
                         │
                         └── elbow_joint
                               │
                               └── ee_link
```

### 7.1 必须禁止

```text
孤立 Link（不属于任何 Joint，且不是 root）
不存在的 Link ID / Joint ID
非法循环（A → B → A）
重复 ID
多父（一个 Link 被两个 Joint 当作 child）
```

### 7.2 与真实机构的关系

v0.1 **不支持**复杂闭环机构（parallelogram / 四连杆）。

> **关于 MeArm 的说明**：MeArm 的平行四连杆在 ArmPilot 里是通过
> "被动腕 + 耦合"表达的，**没有**形成 URDF 意义上的闭环。
> 因此它仍然可以在 v0.1 的 Tree 契约内表达（用 `mimic` 或被动关节），
> 只是 `mimic` 在 v0.1 不实现——见 §3.4。这也是 ArmPilot 作为
> Golden Reference 的一个重要用途：验证"Tree 契约是否足够表达真实机械臂"。

### 7.3 Root Contract

```python
root_link: str     # 必须 ∈ links
base_frame: str    # 必须 ∈ frames
```

`root_link` 是树根（其 `parent_joint is None`）。
**有且仅有一个** Link 满足 `parent_joint is None`。

`base_frame` 通常与 `root_link` 同名，但**不要求**同名——
某台机器人的基座参考系可能是"根 Link 偏移后的地面投影"。

---

## 8. 数值契约

RobotModel 内部物理量必须是：

```text
Python float
NumPy float32 / float64
```

Frontend：

```text
TypeScript number
```

**禁止**字符串物理量：

```python
"1.57"      # ❌
"100mm"     # ❌
"90deg"     # ❌
```

理由：字符串物理量必然导致两件事——① 每处使用都要 parse（且 parse 失败无声）；
② 无法与其他数值直接比较（`"1.57" > 1.5` 在 Python 里会抛错，但在 JS 里会静默转换）。

---

## 9. 校验器（RobotModelValidator）

```python
validate_robot_model(model) -> ValidationReport
```

### 9.1 检查项

**Identity**

```text
Robot ID 非空
Link / Joint / Actuator / Frame / Site / EndEffector ID 各自唯一
ID 符合 snake_case ASCII
```

**References**

```text
joint.parent_link   ∈ links
joint.child_link    ∈ links
link.parent_joint   ∈ joints
link.child_joints[] ∈ joints
actuator.joint      ∈ joints
frame.parent        ∈ (links ∪ frames)
site.parent         ∈ links
end_effector.frame  ∈ frames
end_effector.site   ∈ sites
```

**Tree**

```text
root_link ∈ links
恰好一个 Link 的 parent_joint is None
无孤立 Link
无循环
无多父（每个非根 Link 恰好被一个 Joint 作为 child）
```

**Joint**

```text
axis 已归一化（|axis| ≈ 1）
limits 合法（min <= max）
revolute 使用 rad 语义（类型检查）
prismatic 使用 m 语义（类型检查）
```

**Transform**

```text
Quaternion 长度 ≈ 1
position 全为有限数（无 NaN / inf）
```

**Coordinate（与 P0 契约交叉检查）**

```text
handedness == right
forward_axis == x / left_axis == y / up_axis == z
```

### 9.2 严重级别

| 级别 | 含义 | 处理 |
|---|---|---|
| `error` | 契约违反，模型不可用 | 加载失败 |
| `warn` | 可疑但不致命 | 记录，继续 |

**纪律**：`warn` 只用于"确实值得人看一眼、但不影响正确性"的事。
把错的告警留着比没有告警更糟——因为一旦有一条已知无害的告警，
所有人就会忽略整个告警通道。

---

## 10. 验收清单

| # | 判据 | 测试 |
|---|---|---|
| 1 | mini_arm 能构建合法 RobotModel | `test_mjcf_loader.py` |
| 2 | ID 唯一性与格式 | `test_robot_model.py::test_ids_unique_and_snake_case` |
| 3 | Tree 无孤立/循环/多父 | `test_robot_model.py::test_tree_contract` |
| 4 | root_link 唯一 | `test_robot_model.py::test_root_link_unique` |
| 5 | 引用完整性 | `test_robot_model.py::test_references_resolve` |
| 6 | axis 归一化 | `test_robot_model.py::test_joint_axis_normalized` |
| 7 | quaternion 归一化 | `test_robot_model.py::test_transform_quaternion_normalized` |
| 8 | 无 Runtime 污染（frozen 不可写） | `test_robot_model.py::test_model_is_frozen` |
| 9 | 可 JSON 序列化 | `test_robot_model.py::test_model_json_serializable` |
| 10 | capabilities 与实现一致（交叉检查） | `test_robot_model.py::test_capabilities_cross_check` |

---

## 11. 变更记录

| 版本 | 日期 | 变更 | 影响 |
|---|---|---|---|
| v1 | 2026-09-15 | 初始冻结：14 个字段组、3 对象分离、Tree 契约 | RobotForge v0.1 基线 |

> **冻结规则**：新增字段属于**兼容变更**（旧模型该字段为默认值即可）；
> 修改字段语义、删除字段、改单位属于**不兼容变更**，必须升级 `RobotModel.version`
> 并在 `MJCFLoader` 加入旧版适配。**任何情况下不允许某一端私自扩展字段。**
