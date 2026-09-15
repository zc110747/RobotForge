# RobotForge v0.1 完整统一开发提示词

你是一名资深机器人软件架构师、机器人运动学工程师、MuJoCo 专家、Python 后端工程师和 TypeScript / Three.js 前端工程师。

请从零创建一个独立项目：

# RobotForge

项目定位：

> **Robot Model + Digital Twin + Simulation Platform**

核心理念：

> **One Robot Package, One RobotModel, Multiple Runtime Backends**

RobotForge 不是针对某一台机械臂的专用程序，而是一个能够通过 Robot Package 加载不同机器人的通用机器人数字孪生与仿真平台。

---

# 1. 项目背景

已有一个独立项目：

> ArmPilot

ArmPilot 是一个完整的单机械臂 MeArm 项目，已经包含：

* MeArm 三维模型
* Web 3D
* FK
* IK
* WebSocket
* MuJoCo
* Sim2Sim
* 单机械臂控制逻辑

ArmPilot 已经完成的代码不要为了 RobotForge 进行大规模重构。

ArmPilot 保持独立，并作为：

> **Golden Reference / Golden Baseline**

未来 RobotForge 加入 MeArm 时，可以使用 ArmPilot 验证：

* FK
* IK
* Joint behavior
* Workspace
* Actuator Mapping
* MuJoCo
* Sim2Sim
* 控制结果

RobotForge 必须从零建立清晰、通用、可扩展的架构。

---

# 2. v0.1 核心目标

RobotForge v0.1：

> **建立一个基于原生 MJCF Robot Package、统一 RobotModel、统一坐标系和统一单位的 Web 机器人数字孪生与 Sim2Sim 平台。**

核心链路：

```text
packages/<robot>
      │
      ▼
manifest.yaml
      │
      ▼
MJCF Loader
      │
      ▼
RobotModel
      │
      ├──────────────┐
      ▼              ▼
     FK              IK
      │              │
      └──────┬───────┘
             ▼
       RobotRuntime
             │
             ▼
       MuJoCoBackend
             │
             ▼
           MuJoCo
             │
             ▼
        RobotState
             │
             ▼
         WebSocket
             │
             ▼
 TypeScript / Three.js
```

v0.1 只实现：

```text
Robot Package
MJCF
MJCF Loader
RobotModel
FK
IK
Python Runtime
WebSocket
MuJoCo
Sim2Sim
TypeScript
React
Three.js
mini_arm
```

---

# 3. v0.1 明确不实现

以下功能第一版本禁止实现：

```text
Real Robot
Serial Hardware
USB Hardware
CAN Hardware
TCP Hardware
Servo Driver
Physical Feedback
Sim2Real Hardware
Vision
AI
Agent
RL
Dataset
ROS
LeRobot
STEP Parser
CAD 自动解析
复杂机械臂自动建模
```

但架构必须预留：

```text
RealRobotBackend
Transport
URDFLoader
GeometryAsset
AssetLoader
Actuator Mapping
```

这些只建立接口边界，不实现具体功能。

原则：

> **为未来扩展预留稳定接口，但不要为了未来功能过度设计 v0.1。**

---

# 4. 核心架构原则

RobotForge v0.1 必须遵循：

```text
Robot Package
      ↓
    Loader
      ↓
  RobotModel
      ↓
 RobotRuntime
      ↓
   Backend
```

其中：

> **RobotModel 是 RobotForge 的 Canonical Internal Representation。**

即：

```text
MJCF ──┐
       │
URDF ──┼──→ RobotModel ──→ Runtime
       │
Future ┘
```

Runtime 不允许直接依赖：

```text
MJCF XML
URDF XML
MuJoCo MjModel
Three.js Object3D
Serial
CAN
USB
```

所有机器人描述最终必须转换成统一的：

```text
RobotModel
```

---

# 5. Robot Package

机器人通过 Package 描述。

目录统一为：

```text
packages/
```

禁止使用：

```text
robot-packages/
```

结构：

```text
packages/
├── mini_arm/
└── mearm-v1/       # Future
```

每个机器人：

```text
packages/<robot-id>/
├── manifest.yaml
├── model/
│   ├── <robot>.xml
│   └── assets/       # Future
├── kinematics/
│   ├── fk.py
│   └── ik.py
└── tests/
```

Robot Package 负责：

* Robot-specific model
* MJCF
* FK
* IK
* Joint limits
* End Effector
* Robot-specific capabilities
* Future actuator mapping
* Robot-specific tests
* Future geometry assets

核心原则：

> **增加新机器人主要通过增加 `packages/<robot>` 完成，而不是修改 Core。**

Core 不允许出现：

```python
if robot == "mini_arm":
```

等机器人专用分支。

---

# 6. mini_arm

第一版本只实现：

> `mini_arm`

mini_arm 是为了验证 RobotForge 平台架构而设计的最小机器人。

推荐：

```text
base
  │
  └── shoulder
        │
        └── elbow
              │
              └── end_effector
```

建议：

* 2~3 个 Revolute Joint
* 简单 Link
* 简单 Box / Cylinder geometry
* 明确 Joint Axis
* 明确 Joint Limits
* 一个 End Effector
* 简单 FK
* 简单 IK

不要追求真实机械臂外观。

不要因为 mini_arm 引入：

* Servo Mapping
* 复杂机械连杆
* Coupling
* Calibration
* 复杂动力学
* 真实机械结构

v0.1 的目的：

> 验证 RobotForge 的通用架构。

---

# 7. 原生 MJCF 是 v0.1 唯一机器人模型格式

v0.1 必须使用：

> **MuJoCo 原生 MJCF XML**

例如：

```text
packages/
└── mini_arm/
    └── model/
        └── mini_arm.xml
```

禁止创建：

```text
robot.json
robot.yaml
robotforge.xml
custom_robot.xml
```

作为机器人结构描述文件。

MJCF 才是 v0.1 的机器人模型源。

使用 MuJoCo 官方 API：

```python
mujoco.MjModel.from_xml_path(...)
```

不要自己重新实现 MJCF Parser。

---

# 8. MJCF Loader

Core 提供：

```text
RobotModelLoader
```

v0.1 实现：

```text
MJCFLoader
```

接口：

```python
class RobotModelLoader:
    def load(self, source):
        ...
```

流程：

```text
MJCF
 ↓
MJCFLoader
 ↓
RobotModel
```

MJCFLoader 负责：

* 定位 MJCF
* 加载 MJCF
* 解析必要模型信息
* 提取 joints
* 提取 links / bodies
* 提取 actuators
* 提取 sites
* 提取 geometry
* 提取 limits
* 提取 inertial
* 建立 RobotModel

不要自己实现完整 MJCF 标准。

---

# 9. RobotModel：核心契约

RobotModel 是：

> **RobotForge v0.1 最重要的 P0 接口。**

所有：

```text
MJCF
URDF（Future）
其他模型格式（Future）
```

最终必须转换成：

```text
RobotModel
```

以下模块必须依赖 RobotModel：

```text
FK
IK
RobotRuntime
MuJoCo Adapter
Three.js Renderer
RobotModel Validator
Future RealRobotBackend
```

RobotModel Contract 一旦冻结：

> 不允许 MJCF Loader、Frontend、MuJoCo Backend 各自私自扩展另一套模型字段。

---

# 10. RobotModel 总体结构

统一定义：

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

---

# 11. RobotMetadata

```python
class RobotMetadata:
    id: str
    name: str
    version: str
    description: str | None
```

字段：

```text
id
name
version
description
```

例如：

```json
{
  "id": "mini_arm",
  "name": "Mini Arm",
  "version": "1.0.0",
  "description": "Minimal RobotForge validation arm"
}
```

---

# 12. CoordinateConvention

RobotModel 必须明确坐标语义。

```python
class CoordinateConvention:
    convention: str
    handedness: str
    forward_axis: str
    left_axis: str
    up_axis: str
```

v0.1 固定：

```json
{
  "convention": "robotforge",
  "handedness": "right",
  "forward_axis": "x",
  "left_axis": "y",
  "up_axis": "z"
}
```

语义：

```text
+X = Forward
+Y = Left
+Z = Up
```

必须满足：

```text
X × Y = Z
```

---

# 13. UnitConvention

统一：

```python
class UnitConvention:
    length: str
    angle: str
    time: str
    linear_velocity: str
    angular_velocity: str
    force: str
    torque: str
```

v0.1：

```json
{
  "length": "m",
  "angle": "rad",
  "time": "s",
  "linear_velocity": "m/s",
  "angular_velocity": "rad/s",
  "force": "N",
  "torque": "N*m"
}
```

所有内部物理量使用 SI 单位。

禁止业务逻辑中散落：

```python
value / 1000
math.radians(value)
value * 180 / math.pi
```

单位转换必须集中在：

```text
Loader
Adapter
Converter
UI Boundary
Protocol Boundary
```

---

# 14. Link

Link 表示机器人刚体。

```python
class Link:
    id: str
    name: str

    parent_joint: str | None
    child_joints: list[str]

    inertial: Inertial | None
    visual: list[GeometryRef]
    collision: list[GeometryRef]
```

字段：

```text
id
name
parent_joint
child_joints
inertial
visual
collision
```

根 Link：

```text
parent_joint = null
```

---

# 15. Joint

Joint 是运动学核心元素。

```python
class Joint:
    id: str
    name: str

    type: str

    parent_link: str
    child_link: str

    origin: Transform
    axis: Vector3

    limits: JointLimits | None

    mimic: MimicJoint | None
```

v0.1 支持：

```text
fixed
revolute
prismatic
```

mini_arm 主要使用：

```text
revolute
```

---

# 16. Joint Contract

| 字段            | 类型               | 必须 | 说明                       |
| ------------- | ---------------- | -: | ------------------------ |
| `id`          | string           |  ✅ | Joint 唯一 ID              |
| `name`        | string           |  ✅ | Joint 名称                 |
| `type`        | enum             |  ✅ | fixed/revolute/prismatic |
| `parent_link` | string           |  ✅ | 父 Link                   |
| `child_link`  | string           |  ✅ | 子 Link                   |
| `origin`      | Transform        |  ✅ | Joint Frame 相对父 Link     |
| `axis`        | Vector3          |  ✅ | Joint 运动轴                |
| `limits`      | JointLimits/null |  ❌ | 运动限制                     |
| `mimic`       | object/null      |  ❌ | 未来机械耦合                   |

---

# 17. Joint Axis

Axis 必须使用 RobotForge 坐标系。

例如：

```json
{
  "axis": [0, 0, 1]
}
```

表示：

```text
+Z Axis
```

Axis 必须归一化：

```text
|axis| ≈ 1
```

Joint Positive Direction：

> **Right-Hand Rule**

不要在 Joint 中定义：

```text
clockwise
counter_clockwise
servo_direction
```

这些属于未来 Actuator Mapping。

---

# 18. Transform

所有位姿统一：

```python
class Transform:
    position: Vector3
    orientation: Quaternion
```

例如：

```json
{
  "position": [0.0, 0.1, 0.2],
  "orientation": [0, 0, 0, 1]
}
```

标准：

```text
Position:
    meter

Orientation:
    Quaternion [x,y,z,w]
```

Identity：

```text
[0, 0, 0, 1]
```

---

# 19. JointLimits

```python
class JointLimits:
    position_min: float | None
    position_max: float | None

    velocity_max: float | None
    effort_max: float | None
```

Revolute：

```text
position = rad
velocity = rad/s
```

Prismatic：

```text
position = m
velocity = m/s
```

禁止字段：

```text
min_degree
max_degree
min_mm
max_mm
```

字段表达物理意义，单位由 Contract 统一规定。

---

# 20. Actuator

v0.1 只做最小实现。

```python
class Actuator:
    id: str
    name: str

    type: str

    joint: str | None

    command_min: float | None
    command_max: float | None
```

例如：

```json
{
  "id": "shoulder_motor",
  "name": "Shoulder Motor",
  "type": "position",
  "joint": "shoulder_joint",
  "command_min": -1.57,
  "command_max": 1.57
}
```

v0.1：

```text
Joint ≈ Actuator
```

但概念不能合并。

未来：

```text
Joint
 ↓
Actuator Mapping
 ↓
Servo
```

可以支持：

```text
offset
direction
scale
coupling
calibration
mechanical linkage
```

---

# 21. Frame

Frame 是通用坐标参考。

```python
class Frame:
    id: str
    name: str

    parent: str
    transform: Transform
```

可以表示：

```text
World
Base
Link
Joint
Tool
Sensor
End Effector
```

---

# 22. Site

Site 是轻量语义标记点。

```python
class Site:
    id: str
    name: str

    parent: str
    transform: Transform

    role: str | None
```

例如：

```json
{
  "id": "ee_site",
  "name": "End Effector Site",
  "parent": "forearm",
  "transform": {
    "position": [0, 0, 0.08],
    "orientation": [0, 0, 0, 1]
  },
  "role": "end_effector"
}
```

---

# 23. EndEffector

End Effector 不重复保存 Pose。

使用 Frame / Site 引用：

```python
class EndEffector:
    id: str
    name: str

    frame: str
    site: str | None
```

例如：

```json
{
  "id": "main_ee",
  "name": "Main End Effector",
  "frame": "ee_frame",
  "site": "ee_site"
}
```

---

# 24. GeometryRef

RobotModel 不直接保存大型 Mesh 数据。

使用引用：

```python
class GeometryRef:
    type: str
    asset: str | None
    transform: Transform
```

v0.1 可支持：

```text
box
sphere
capsule
cylinder
```

未来：

```text
STL
OBJ
GLTF
GLB
STEP/CAD
```

通过 AssetLoader 扩展。

---

# 25. Inertial

RobotModel 允许保存基本惯性参数：

```python
class Inertial:
    mass: float
    center_of_mass: Vector3

    inertia:
        ixx: float
        iyy: float
        izz: float
        ixy: float
        ixz: float
        iyz: float
```

单位：

```text
mass        kg
center      m
inertia     kg*m²
```

如果 MJCF 没有显式数据，不要随意伪造。

---

# 26. RobotCapabilities

```python
class RobotCapabilities:
    simulation: bool
    fk: bool
    ik: bool
    actuator_control: bool
    end_effector: bool
```

v0.1：

```json
{
  "simulation": true,
  "fk": true,
  "ik": true,
  "actuator_control": true,
  "end_effector": true
}
```

Frontend 应根据 capability 动态决定显示哪些功能。

禁止：

```text
if robot == "mini_arm"
```

---

# 27. RobotModel 不保存 Runtime State

严格区分：

```text
RobotModel
RobotCommand
RobotState
```

RobotModel 表示：

> 机器人是什么。

RobotCommand 表示：

> 用户希望机器人做什么。

RobotState 表示：

> 机器人实际是什么状态。

---

# 28. RobotCommand

统一：

```python
class RobotCommand:
    robot: str
    joint_targets: dict[str, float]
    timestamp: float
```

单位：

```text
Revolute → rad
Prismatic → m
timestamp → s
```

Command 不代表真实执行结果。

---

# 29. RobotState

统一：

```python
class RobotState:
    robot: str

    joint_positions: dict[str, float]
    joint_velocities: dict[str, float]

    end_effector_pose: Transform | None

    status: str
    timestamp: float
```

必须区分：

```text
Command → Desired
State   → Actual
```

即：

```text
RobotCommand
      ↓
   Backend
      ↓
 RobotState
```

即使 v0.1 是 Sim2Sim，也必须维持该语义。

---

# 30. ID Contract

所有 RobotModel 对象拥有稳定 ID：

```text
Robot
Link
Joint
Actuator
Frame
Site
EndEffector
```

ID：

```text
唯一
稳定
ASCII
snake_case
```

推荐：

```text
base
shoulder_link
shoulder_joint
elbow_link
elbow_joint
ee_frame
ee_site
main_ee
```

不要使用：

```text
数组下标
显示名称
内存地址
运行时随机 UUID
```

作为长期内部引用。

---

# 31. ID 与 Name 分离

例如：

```json
{
  "id": "shoulder_joint",
  "name": "Shoulder Joint"
}
```

`id`：

> 程序引用。

`name`：

> UI 显示。

UI 重命名不得导致内部引用失效。

---

# 32. Link / Joint Tree Contract

v0.1 RobotModel 是：

> **Joint-Link Tree**

关系：

```text
Link
 ↓
Joint
 ↓
Link
```

例如：

```text
base
 │
 └── shoulder_joint
       │
       └── upper_arm
             │
             └── elbow_joint
                   │
                   └── forearm
```

必须禁止：

```text
孤立 Link
不存在的 Link ID
不存在的 Joint ID
非法循环
重复 ID
```

v0.1 不支持复杂闭环机构。

---

# 33. Root Contract

必须：

```python
root_link: str
base_frame: str
```

例如：

```json
{
  "root_link": "base",
  "base_frame": "base"
}
```

要求：

```text
root_link ∈ links
base_frame ∈ frames
```

---

# 34. 数值 Contract

RobotModel 内部物理量必须使用：

```text
Python float
NumPy float32
NumPy float64
```

Frontend：

```text
TypeScript number
```

禁止：

```text
"1.57"
"100mm"
"90deg"
```

等字符串物理量。

---

# 35. Quaternion Contract

RobotForge 内部统一：

```text
[x, y, z, w]
```

禁止混用：

```text
[w, x, y, z]
```

Identity：

```text
[0, 0, 0, 1]
```

Quaternion 必须：

```text
|q| ≈ 1
```

---

# 36. RobotModel 不应该包含的内容

不得放入：

```text
WebSocket connection
Runtime status
Current joint state
Current velocity
Current simulation time
MuJoCo MjData
MuJoCo viewer
Three.js Object3D
React state
Serial connection
CAN connection
TCP connection
asyncio Task
```

RobotModel 是：

> **纯机器人结构、运动学、几何和能力描述。**

---

# 37. Manifest

示例：

```yaml
id: mini_arm
name: Mini Arm
version: 1.0.0

model:
  format: mjcf
  file: model/mini_arm.xml

coordinate:
  convention: robotforge
  handedness: right
  forward_axis: x
  left_axis: y
  up_axis: z

units:
  length: m
  angle: rad
  time: s

capabilities:
  simulation: true
  fk: true
  ik: true
```

Manifest 负责：

* Package metadata
* Model entry
* Coordinate declaration
* Unit declaration
* Capabilities

不要把完整机器人结构复制到 manifest。

---

# 38. Coordinate System

RobotForge 全局：

```text
Right-Handed Coordinate System
```

定义：

```text
+X = Forward
+Y = Left
+Z = Up
```

地面：

```text
Z = 0
```

Robot Base：

```text
+X = Robot Forward
+Y = Robot Left
+Z = Robot Up
```

Joint：

```text
Right-Hand Rule
```

---

# 39. Pose Contract

Position：

```text
[x, y, z]
```

单位：

```text
m
```

Orientation：

```text
Quaternion [x,y,z,w]
```

Euler Angle 只能用于：

```text
UI
Debug
Human Input
```

不能作为 RobotModel 的标准 Orientation。

如果使用 Euler：

```text
Roll  = X
Pitch = Y
Yaw   = Z
```

转换必须集中处理。

---

# 40. MJCF 坐标适配

如果 MJCF 本身遵循 RobotForge：

```text
+X Forward
+Y Left
+Z Up
```

则：

```text
No Conversion
```

如果第三方 MJCF 坐标不同：

```text
MJCF
 ↓
MJCFLoader / Adapter
 ↓
RobotModel
```

坐标转换只能存在于：

```text
Loader
Adapter
```

禁止散落在：

```text
FK
IK
Runtime
Frontend
WebSocket
```

---

# 41. Three.js 坐标适配

Three.js 只是 Renderer。

如果需要坐标转换：

```text
RobotModel
 ↓
Three.js Renderer Adapter
 ↓
Three.js
```

不能修改 RobotModel 的语义。

不能在 FK / IK 中为了 Three.js 修改坐标。

---

# 42. 坐标和单位测试

必须测试：

```text
X × Y = Z
```

```text
Identity Quaternion = [0,0,0,1]
```

检查：

```text
X/Y/Z 无轴反转
无左手/右手混用
无 degree/radian 混用
无 mm/m 混用
无 quaternion 顺序错误
Frontend / Backend 坐标一致
Model / Simulation 坐标一致
```

---

# 43. URDF 扩展

v0.1：

```text
MJCFLoader  ✅
URDFLoader  ❌
```

未来：

```text
RobotModelLoader
      │
 ┌────┴────┐
MJCF      URDF
Loader    Loader
 │          │
 └────┬─────┘
      ↓
 RobotModel
```

Runtime 只能依赖：

```text
RobotModel
```

而不是：

```text
MJCF
```

---

# 44. Geometry Asset 扩展

v0.1：

```text
MJCF native geom
```

即可。

未来：

```text
GeometryAsset
AssetLoader
```

支持：

```text
STL
OBJ
GLTF
GLB
STEP
```

v0.1 不实现 CAD Parser。

---

# 45. Python Backend

使用：

```text
Python 3.11+
FastAPI
Pydantic
asyncio
NumPy
MuJoCo
pytest
```

Backend：

```text
backend/
├── api/
├── runtime/
├── model/
├── loaders/
├── simulation/
└── transport/
```

---

# 46. RobotRuntime

RobotRuntime 负责：

* Package discovery
* Package loading
* RobotModel management
* RobotCommand
* Backend management
* RobotState
* Runtime lifecycle

接口关系：

```text
Package
 ↓
Loader
 ↓
RobotModel
 ↓
RobotRuntime
```

RobotRuntime 不应该知道：

```text
mini_arm
mearm
serial
CAN
USB
```

等具体实现。

---

# 47. Backend 抽象

定义：

```python
class RobotBackend:
    async def start(self):
        ...

    async def stop(self):
        ...

    async def reset(self):
        ...

    async def send_command(self, command):
        ...

    async def get_state(self):
        ...
```

v0.1：

```text
MuJoCoBackend
```

未来：

```text
RealRobotBackend
```

统一：

```text
RobotCommand
RobotState
```

---

# 48. Transport 抽象

预留：

```python
class Transport:
    async def connect(self):
        ...

    async def disconnect(self):
        ...

    async def send(self, data):
        ...

    async def receive(self):
        ...
```

未来：

```text
SerialTransport
CANTransport
USBTransport
TCPTransport
```

v0.1 不实现。

Runtime 不得直接依赖硬件 Transport。

---

# 49. Sim2Sim

必须形成真实闭环：

```text
Frontend
 ↓
WebSocket
 ↓
RobotCommand
 ↓
RobotRuntime
 ↓
MuJoCoBackend
 ↓
MuJoCo
 ↓
RobotState
 ↓
WebSocket
 ↓
Frontend
```

不能：

```text
Frontend
 ↓
直接修改 Three.js
```

必须：

```text
Command
 ↓
Runtime
 ↓
Simulation
 ↓
State
 ↓
Frontend
```

---

# 50. MuJoCo 职责

MuJoCo 是：

> **Physics Backend**

MuJoCo 负责：

```text
Physics
Gravity
Collision
Contact
Joint State
Body State
Actuator
Simulation Time
Dynamics
```

RobotForge 负责：

```text
Robot Package
RobotModel
Runtime
Command
State
WebSocket
Simulation Lifecycle
```

不要把 RobotForge 设计成 MuJoCo API 的简单包装。

---

# 51. Frontend

推荐：

```text
TypeScript
Vite
React
Three.js
WebSocket
```

功能：

```text
Robot Selection
3D Visualization
Joint Control
FK
IK
End Effector Pose
Simulation Status
Robot State
Connection Status
Coordinate Frame
```

Frontend 不允许：

```javascript
if (robot === "mini_arm")
```

必须根据：

```text
RobotModel
RobotCapabilities
RobotState
```

动态运行。

---

# 52. WebSocket Protocol

Command：

```json
{
  "type": "joint_command",
  "robot": "mini_arm",
  "joints": {
    "shoulder": 0.5,
    "elbow": 0.8
  },
  "timestamp": 123.456
}
```

State：

```json
{
  "type": "robot_state",
  "robot": "mini_arm",
  "joints": {
    "shoulder": 0.5,
    "elbow": 0.8
  },
  "timestamp": 123.456
}
```

至少支持：

```text
robot_info
robot_command
robot_state
simulation_state
error
```

单位：

```text
Position       m
Angle          rad
Linear Speed   m/s
Angular Speed  rad/s
Time           s
```

所有物理数值必须是 JSON Number。

---

# 53. FK

统一：

```text
Joint
 ↓
FK
 ↓
Pose
```

输入：

```text
Joint Position
```

单位：

```text
rad / m
```

输出：

```text
Position = m
Orientation = quaternion [x,y,z,w]
```

必须遵循 RobotForge Coordinate Contract。

---

# 54. IK

统一：

```text
Target Pose
 ↓
IK
 ↓
Joint
```

输入：

```text
Position = m
Orientation = quaternion
```

输出：

```text
Joint Position = rad / m
```

IK 输出：

> Joint Space

不是：

> Servo Angle

---

# 55. FK / IK 验证

必须执行：

```text
Joint
 ↓
FK
 ↓
Pose
 ↓
IK
 ↓
Joint'
```

以及：

```text
Pose
 ↓
IK
 ↓
Joint
 ↓
FK
 ↓
Pose'
```

验证：

```text
Joint ≈ Joint'
Pose ≈ Pose'
```

必须考虑：

* 浮点误差
* IK 多解
* Joint Limits
* Reachability

---

# 56. 推荐项目结构

```text
RobotForge/
│
├── backend/
│   ├── api/
│   │   ├── routes.py
│   │   └── websocket.py
│   │
│   ├── runtime/
│   │   ├── robot_runtime.py
│   │   ├── command.py
│   │   ├── state.py
│   │   └── backend.py
│   │
│   ├── model/
│   │   ├── robot_model.py
│   │   ├── types.py
│   │   └── validator.py
│   │
│   ├── loaders/
│   │   ├── loader.py
│   │   └── mjcf_loader.py
│   │
│   ├── simulation/
│   │   ├── simulation_backend.py
│   │   └── mujoco_backend.py
│   │
│   └── transport/
│       └── transport.py
│
├── frontend/
│   └── src/
│       ├── components/
│       ├── robot/
│       ├── viewer/
│       ├── simulation/
│       ├── websocket/
│       └── state/
│
├── packages/
│   └── mini_arm/
│       ├── manifest.yaml
│       ├── model/
│       │   └── mini_arm.xml
│       ├── kinematics/
│       │   ├── fk.py
│       │   └── ik.py
│       └── tests/
│
├── tests/
│   ├── test_manifest.py
│   ├── test_mjcf_loader.py
│   ├── test_robot_model.py
│   ├── test_coordinate.py
│   ├── test_units.py
│   ├── test_fk.py
│   ├── test_ik.py
│   ├── test_runtime.py
│   ├── test_websocket.py
│   └── test_mujoco.py
│
├── docs/
│   ├── architecture.md
│   ├── robot-model.md
│   ├── robot-package.md
│   ├── coordinate-system.md
│   ├── websocket.md
│   └── simulation.md
│
├── README.md
└── pyproject.toml
```

---

# 57. Phase 0：项目初始化

实现：

```text
Python Backend
TypeScript Frontend
packages/
mini_arm/
RobotModel Contract
Documentation
Dependency Management
```

必须首先冻结：

```text
RobotModel
RobotCommand
RobotState
CoordinateConvention
UnitConvention
```

验收：

```text
Backend 可以启动
Frontend 可以启动
pytest 可以运行
```

---

# 58. Phase 1：MJCF + RobotModel

实现：

```text
packages/mini_arm
 ↓
manifest
 ↓
MJCF Loader
 ↓
RobotModel
 ↓
RobotModel Validator
```

验收：

* 自动发现 mini_arm
* 读取 manifest
* 加载 MJCF
* 获取 Links
* 获取 Joints
* 获取 Joint Axis
* 获取 Joint Limits
* 获取 Actuators
* 获取 Frames
* 获取 Sites
* 获取 End Effector
* 获取 Geometry
* 获取 Coordinate
* 获取 Units
* 构建合法 RobotModel

---

# 59. Phase 2：Three.js

实现：

```text
RobotModel
 ↓
Renderer Adapter
 ↓
Three.js
 ↓
3D mini_arm
```

验收：

* 浏览器显示机器人
* Camera Orbit
* Zoom
* Pan
* Joint hierarchy 正确
* Joint axis 正确
* End Effector 正确
* Coordinate Frame 正确
* RobotModel 与 Renderer 语义一致

---

# 60. Phase 3：FK / IK

实现：

```text
Joint → FK → Pose
Pose → IK → Joint
```

验收：

```text
Joint
 ↓
FK
 ↓
Pose
 ↓
IK
 ↓
Joint'
```

以及：

```text
Pose
 ↓
IK
 ↓
Joint
 ↓
FK
 ↓
Pose'
```

---

# 61. Phase 4：Runtime + WebSocket

实现：

```text
Frontend
 ↓
WebSocket
 ↓
RobotRuntime
 ↓
RobotCommand
 ↓
Backend
 ↓
RobotState
```

验收：

* WebSocket 连接
* Robot discovery
* RobotModel 获取
* Command 发送
* Runtime 接收
* Backend 执行
* State 返回
* Frontend 更新

---

# 62. Phase 5：MuJoCo Sim2Sim

实现：

```text
RobotCommand
 ↓
RobotRuntime
 ↓
MuJoCoBackend
 ↓
MuJoCo
 ↓
RobotState
 ↓
WebSocket
 ↓
Frontend
```

验收：

1. 启动 Simulation
2. 加载 mini_arm MJCF
3. 发送 Joint Command
4. MuJoCo 执行
5. 获取 Joint State
6. 生成 RobotState
7. WebSocket 推送
8. Frontend 同步
9. Three.js 同步
10. FK / IK 与 Simulation 状态一致

---

# 63. Phase 6：Sim2Real 架构验证

不连接真实硬件。

只验证：

```text
RobotRuntime
      │
 ┌────┴────┐
 │         │
MuJoCo   RealRobotBackend
```

切换 Backend 时：

```text
RobotCommand 不变
RobotState 不变
WebSocket 不变
Frontend 不变
RobotModel 不变
```

只替换：

```text
Backend
Transport
```

---

# 64. Phase 7：URDF 扩展点验证

不实现 URDF Parser。

只验证：

```text
RobotModelLoader
      │
 ┌────┴────┐
MJCF      URDF
Loader    Loader
 │          │
 └────┬─────┘
      ↓
 RobotModel
```

确保 Runtime：

```text
不依赖 MJCF
```

而是：

```text
依赖 RobotModel
```

---

# 65. Phase 8：Geometry Asset 扩展点验证

不实现：

```text
STL Parser
OBJ Parser
GLTF Parser
STEP Parser
```

只验证：

```text
RobotModel
 ↓
GeometryAsset
 ↓
AssetLoader
```

架构未来可以加入：

```text
STLLoader
OBJLoader
GLTFLoader
CAD/STEP Loader
```

而不影响 Runtime。

---

# 66. RobotModel Validator

必须建立：

```python
validate_robot_model(model)
```

检查：

### Identity

```text
Robot ID 唯一
Link ID 唯一
Joint ID 唯一
Actuator ID 唯一
Frame ID 唯一
Site ID 唯一
End Effector ID 唯一
```

### References

```text
parent_link 存在
child_link 存在
parent_joint 存在
actuator.joint 存在
frame.parent 存在
site.parent 存在
end_effector.frame 存在
```

### Tree

```text
root_link 存在
无孤立 Link
无非法循环
```

### Joint

```text
axis normalized
limits 合法
revolute 使用 rad
prismatic 使用 m
```

### Transform

```text
Quaternion length ≈ 1
```

### Coordinate

```text
right-handed
X forward
Y left
Z up
```

---

# 67. 自动化测试

至少：

```text
tests/
├── test_manifest.py
├── test_mjcf_loader.py
├── test_robot_model.py
├── test_coordinate.py
├── test_units.py
├── test_fk.py
├── test_ik.py
├── test_runtime.py
├── test_websocket.py
└── test_mujoco.py
```

核心测试：

```text
MJCF → RobotModel

RobotModel → Validator

Joint → FK → Pose

Pose → IK → Joint

Command → Backend → State

Command → MuJoCo → State

RobotModel → Three.js

Coordinate Conversion

Unit Conversion
```

---

# 68. P0 坐标与单位验收

必须自动验证：

```text
X × Y = Z
```

```text
Identity Quaternion = [0,0,0,1]
```

```text
Length = m
Angle = rad
Time = s
```

检查：

```text
无轴反转
无左手/右手混用
无 degree/radian 混用
无 mm/m 混用
无 quaternion 顺序错误
```

并验证：

```text
FK
MuJoCo
Three.js
WebSocket
```

对于相同机器人状态具有一致语义。

---

# 69. Architecture Constraints

以下为不可违反的 P0 规则。

### 规则 1

Core 不允许机器人专用分支：

```python
if robot == "mini_arm":
```

### 规则 2

Frontend 不允许硬编码 mini_arm 结构。

### 规则 3

v0.1 使用原生 MJCF。

### 规则 4

禁止创建自定义 Robot XML。

### 规则 5

Robot Package 保存机器人特定数据和算法。

### 规则 6

MJCF Loader 属于 Core。

### 规则 7

RobotModel 是唯一统一内部机器人模型。

### 规则 8

RobotCommand 与 RobotState 必须分离。

### 规则 9

RobotModel、RobotCommand、RobotState 三个 Contract 必须分离。

### 规则 10

Simulation 与 Real 使用统一 Backend Interface。

### 规则 11

v0.1 只实现 Sim2Sim。

### 规则 12

Sim2Real 只预留接口。

### 规则 13

URDF 只预留 Loader。

### 规则 14

STL / OBJ / GLTF / STEP 只预留 Geometry Asset 扩展。

### 规则 15

所有内部数据必须遵循统一坐标系。

### 规则 16

所有内部物理量必须使用 SI 单位。

### 规则 17

坐标转换只能存在于明确的 Loader / Adapter。

### 规则 18

单位转换不能散落在业务逻辑。

### 规则 19

Quaternion 统一 `[x,y,z,w]`。

### 规则 20

RobotModel 不得包含 Runtime State。

### 规则 21

RobotModel 不得包含 WebSocket / React / Three.js / MuJoCo Runtime 对象。

### 规则 22

RobotModel 对象必须使用稳定 ID。

### 规则 23

v0.1 RobotModel 默认是 Link-Joint Tree。

### 规则 24

不要为了未来功能过度设计。

---

# 70. 开发执行规则

不要一次性生成整个项目然后再调试。

严格执行：

```text
设计
 ↓
实现
 ↓
Build
 ↓
Run
 ↓
Test
 ↓
Validate
 ↓
Document
 ↓
下一 Phase
```

每个 Phase 完成后：

1. 运行项目
2. 执行自动化测试
3. 执行人工验收
4. 修复问题
5. 更新 README
6. 更新 architecture.md
7. 更新 robot-model.md
8. 更新 coordinate-system.md
9. 记录已知限制

当前 Phase 未通过：

> 不得进入下一 Phase。

---

# 71. 最终完整验收

RobotForge v0.1 必须能够执行：

```text
启动 RobotForge
      ↓
发现 mini_arm
      ↓
读取 manifest
      ↓
读取 Coordinate / Units
      ↓
加载 MJCF
      ↓
构建 RobotModel
      ↓
RobotModel Validator
      ↓
浏览器显示 3D mini_arm
      ↓
显示 RobotForge Coordinate Frame
      ↓
用户修改 Joint
      ↓
FK
      ↓
显示 End Effector Pose
      ↓
输入 Target XYZ
      ↓
IK
      ↓
得到 Joint
      ↓
发送 RobotCommand
      ↓
Python RobotRuntime
      ↓
MuJoCoBackend
      ↓
MuJoCo
      ↓
RobotState
      ↓
WebSocket
      ↓
Frontend
      ↓
Three.js
      ↓
3D Robot 同步
```

---

# 72. 最终架构

```text
                         Robot Package
                              │
                              │ MJCF
                              ▼
                        MJCF Loader
                              │
                              ▼
                     ┌────────────────┐
                     │   RobotModel   │
                     │ Canonical Model│
                     └────────────────┘
                       │      │      │
                      FK     IK   MuJoCo
                       │      │      │
                       └──┬───┘      │
                          ▼          ▼
                     RobotRuntime  MuJoCo
                          │
                   ┌──────┴──────┐
                   │             │
             RobotCommand   RobotState
                   │             ▲
                   └── Backend ──┘
                          │
                      WebSocket
                          │
                          ▼
                  TypeScript Frontend
                          │
                       Three.js
```

未来：

```text
                    RobotModel
                         │
          ┌──────────────┼──────────────┐
          │              │              │
         FK             IK          Backend
                                      │
                              ┌───────┴───────┐
                              │               │
                         MuJoCoBackend   RealRobotBackend
                              │               │
                           MuJoCo         Transport
```

Model Loader：

```text
                   RobotModelLoader
                          │
                 ┌────────┴────────┐
                 │                 │
             MJCFLoader        URDFLoader
              v0.1               Future
                 │                 │
                 └────────┬────────┘
                          ▼
                     RobotModel
```

Geometry：

```text
                    GeometryAsset
                         │
                    AssetLoader
                         │
             ┌───────────┼───────────┐
             │           │           │
            STL         OBJ        GLTF
             │
           Future
```

---

# 73. 最终 RobotModel Contract

RobotForge v0.1 固定：

```text
RobotModel
│
├── metadata
│   ├── id
│   ├── name
│   ├── version
│   └── description
│
├── coordinate
│   ├── convention
│   ├── handedness
│   ├── forward_axis
│   ├── left_axis
│   └── up_axis
│
├── units
│   ├── length
│   ├── angle
│   ├── time
│   ├── linear_velocity
│   ├── angular_velocity
│   ├── force
│   └── torque
│
├── root_link
├── base_frame
│
├── links[]
│   ├── id
│   ├── name
│   ├── parent_joint
│   ├── child_joints[]
│   ├── inertial
│   ├── visual[]
│   └── collision[]
│
├── joints[]
│   ├── id
│   ├── name
│   ├── type
│   ├── parent_link
│   ├── child_link
│   ├── origin
│   ├── axis
│   ├── limits
│   └── mimic
│
├── actuators[]
│   ├── id
│   ├── name
│   ├── type
│   ├── joint
│   ├── command_min
│   └── command_max
│
├── frames[]
│   ├── id
│   ├── name
│   ├── parent
│   └── transform
│
├── sites[]
│   ├── id
│   ├── name
│   ├── parent
│   ├── transform
│   └── role
│
├── end_effectors[]
│   ├── id
│   ├── name
│   ├── frame
│   └── site
│
└── capabilities
    ├── simulation
    ├── fk
    ├── ik
    ├── actuator_control
    └── end_effector
```

---

# 74. 最终三大核心 Contract

RobotForge v0.1 必须冻结三个核心对象：

```text
┌─────────────────────────────┐
│         RobotModel          │
│        机器人是什么          │
└─────────────────────────────┘
              │
              │
┌─────────────┴───────────────┐
│                             │
▼                             ▼
┌─────────────────┐   ┌─────────────────┐
│ RobotCommand    │   │ RobotState      │
│ 希望做什么       │   │ 实际发生什么     │
└─────────────────┘   └─────────────────┘
```

关系：

```text
RobotModel
    │
    ├── defines robot structure
    │
    ▼
RobotCommand
    │
    ▼
Backend
    │
    ▼
RobotState
```

这三个 Contract 是后续：

```text
Sim2Sim
Sim2Real
Data Collection
Agent
Learning
```

的基础。

---

# 75. 最终成功标准

RobotForge v0.1 不以：

> 支持多少机器人

作为成功标准。

而以：

> **能否用一个统一 RobotModel 驱动不同机器人来源和不同运行后端。**

最终必须证明：

```text
packages/mini_arm
        ↓
      MJCF
        ↓
   MJCFLoader
        ↓
    RobotModel
        ↓
 ┌──────┼───────┐
 FK     IK    MuJoCo
        │        │
        └───┬────┘
            ▼
       RobotRuntime
            │
            ▼
       RobotCommand
            │
            ▼
          State
            │
            ▼
        WebSocket
            │
            ▼
        Frontend
```

并且未来可以：

```text
packages/mearm-v1
        ↓
      MJCF
        ↓
   RobotModel
```

而不需要修改：

```text
Core
Runtime
Frontend
WebSocket
MuJoCo Backend
```

---

# 76. 最终定义

RobotForge v0.1：

> **一个基于标准 MJCF Robot Package、统一 RobotModel、统一坐标系与 SI 单位、通用 RobotRuntime 和 Backend 抽象的 Web 机器人数字孪生与 Sim2Sim 平台。**

核心：

```text
Standard MJCF
      +
Robot Package
      +
Canonical RobotModel
      +
Unified Coordinate
      +
Unified Units
      +
RobotCommand
      +
RobotState
      +
Generic Runtime
      +
Backend Abstraction
      +
WebSocket
      +
Three.js
      +
MuJoCo
```

v0.1 保持最小：

```text
MJCF
+
mini_arm
+
RobotModel
+
FK
+
IK
+
MuJoCo
+
WebSocket
+
Three.js
+
Sim2Sim
```

但核心接口必须正确。

最终原则：

> **先冻结契约，再实现功能。**

> **先保证 RobotModel 正确，再扩展 Backend。**

> **先保证坐标和单位一致，再做 FK / IK / MuJoCo / Three.js。**

> **先实现最小 mini_arm，再验证 MeArm。**

> **先验证 Sim2Sim，再进入 Sim2Real。**

> **不要为了功能数量牺牲架构边界。**

> **不要为了未来功能过度实现。**

> **RobotModel、RobotCommand、RobotState、Coordinate、Unit 是 RobotForge v0.1 的 P0 核心契约。**
