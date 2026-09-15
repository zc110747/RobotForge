"""`RobotModel` —— RobotForge 的 **Canonical Internal Representation**（P0 契约）。

本模块只定义**契约对象**，不含任何逻辑：

```text
MJCF ──┐
URDF ──┼──→ RobotModel ──→ Runtime / FK / IK / Renderer
Future ┘
```

## 三条不可违反的设计纪律

1. **全部 frozen**。`RobotModel` 及其子对象都是 `@dataclass(frozen=True)`。
   这不是风格偏好，而是**契约的强制执行机制**：它让
   `model.current_joint_state = [...]` 这类"把 Runtime 状态塞进模型"的
   写法在运行期直接抛 `FrozenInstanceError`，而不是等到多会话互相污染时才发现。

2. **不依赖任何外部类型**。本文件**不 import** mujoco / three.js / numpy（除
   `types.py` 的计算辅助外）。于是"Runtime 直接依赖 MjModel"在物理上不可能发生。

3. **可 JSON 序列化**。每个对象都有 `to_dict()`，且输出只含
   `dict / list / str / float / bool / None`。WS 的 `robot_info` 直接用它。

## 与其它两个契约的关系

```text
RobotModel  机器人**是什么**     ← 本模块（加载后不变）
RobotCommand 用户希望做什么        ← runtime/command.py
RobotState   实际处于什么状态      ← runtime/state.py
```

三者必须分离。v0.1 即便是 Sim2Sim（MuJoCo 一步就跟上），
也**不**允许把 `State` 直接等于 `Command`。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

from .types import Quaternion, Transform, Vector3

# ---------------------------------------------------------------------------
# 枚举（用 Literal 而非 Enum：JSON 序列化天然友好，前端可直接用字符串比较）
# ---------------------------------------------------------------------------

JointType = Literal["fixed", "revolute", "prismatic"]
ActuatorType = Literal["position", "velocity", "torque", "motor"]
GeometryType = Literal["box", "sphere", "cylinder", "capsule", "mesh"]

#: v0.1 支持的 joint 类型。扩充需同步 RobotModel 契约版本。
JOINT_TYPES: tuple[str, ...] = ("fixed", "revolute", "prismatic")
ACTUATOR_TYPES: tuple[str, ...] = ("position", "velocity", "torque", "motor")
GEOMETRY_TYPES: tuple[str, ...] = ("box", "sphere", "cylinder", "capsule", "mesh")

#: 有自由度的关节类型（会产生 qpos 并参与 FK）
MOBILE_JOINT_TYPES: frozenset[str] = frozenset({"revolute", "prismatic"})


# ---------------------------------------------------------------------------
# §11 RobotMetadata
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RobotMetadata:
    """机器人的身份。`id` 来自 manifest，是**包级稳定标识**。"""

    id: str
    name: str
    version: str
    description: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "version": self.version,
            "description": self.description,
        }


# ---------------------------------------------------------------------------
# §12 / §13 Coordinate & Unit Convention
# ---------------------------------------------------------------------------

#: P0 冻结值。`coordinate-system.md` §2 规定 v0.1 不接受自定义。
ROBOTFORGE_COORDINATE: dict[str, str] = {
    "convention": "robotforge",
    "handedness": "right",
    "forward_axis": "x",
    "left_axis": "y",
    "up_axis": "z",
}

#: P0 冻结值。SI 单位（`coordinate-system.md` §3）。
ROBOTFORGE_UNITS: dict[str, str] = {
    "length": "m",
    "angle": "rad",
    "time": "s",
    "linear_velocity": "m/s",
    "angular_velocity": "rad/s",
    "force": "N",
    "torque": "N*m",
}


@dataclass(frozen=True)
class CoordinateConvention:
    """坐标语义声明。v0.1 固定，但**仍然保留为对象**。

    为什么不干脆用常量：因为未来要加载第三方 MJCF（可能是 Y-up 或左手系），
    那时"模型用的是哪套坐标"必须能被**携带**到 RobotModel 里 ——
    否则"哪台机器人需要转换"这个信息就丢在 Loader 里了。
    v0.1 的所有模型都是 `robotforge`，但字段先占位。
    """

    convention: str = "robotforge"
    handedness: str = "right"
    forward_axis: str = "x"
    left_axis: str = "y"
    up_axis: str = "z"

    def to_dict(self) -> dict[str, str]:
        return {
            "convention": self.convention,
            "handedness": self.handedness,
            "forward_axis": self.forward_axis,
            "left_axis": self.left_axis,
            "up_axis": self.up_axis,
        }

    def is_robotforge(self) -> bool:
        return (
            self.convention == ROBOTFORGE_COORDINATE["convention"]
            and self.handedness == ROBOTFORGE_COORDINATE["handedness"]
            and self.forward_axis == ROBOTFORGE_COORDINATE["forward_axis"]
            and self.left_axis == ROBOTFORGE_COORDINATE["left_axis"]
            and self.up_axis == ROBOTFORGE_COORDINATE["up_axis"]
        )


@dataclass(frozen=True)
class UnitConvention:
    """单位声明。v0.1 固定 SI。"""

    length: str = "m"
    angle: str = "rad"
    time: str = "s"
    linear_velocity: str = "m/s"
    angular_velocity: str = "rad/s"
    force: str = "N"
    torque: str = "N*m"

    def to_dict(self) -> dict[str, str]:
        return {
            "length": self.length,
            "angle": self.angle,
            "time": self.time,
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "force": self.force,
            "torque": self.torque,
        }

    def is_si(self) -> bool:
        return self.to_dict() == ROBOTFORGE_UNITS


# ---------------------------------------------------------------------------
# §19 JointLimits / §3.4 MimicJoint
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointLimits:
    """关节运动限制。

    ## 单位由**类型**决定（不是由字段名）

    | Joint type | position | velocity |
    |---|---|---|
    | `revolute` | rad | rad/s |
    | `prismatic` | m | m/s |

    **禁止** `min_degree` / `max_mm` 这类在字段名里编码单位的字段：
    它允许同一个模型里混两种单位，而那正是 P0 契约要消灭的东西。

    ## `None` 的语义

    `None` = **未声明**（不是 0，也不是无穷）。
    """

    position_min: float | None = None
    position_max: float | None = None
    velocity_max: float | None = None
    effort_max: float | None = None

    def to_dict(self) -> dict[str, float | None]:
        return {
            "position_min": self.position_min,
            "position_max": self.position_max,
            "velocity_max": self.velocity_max,
            "effort_max": self.effort_max,
        }

    def has_position_bounds(self) -> bool:
        return self.position_min is not None and self.position_max is not None

    def clamps(self, value: float) -> tuple[float, bool]:
        """夹紧到区间，并返回是否发生了夹紧（供 IK / UI 反馈"已触限"）。"""
        out = value
        clamped = False
        if self.position_min is not None and out < self.position_min:
            out, clamped = self.position_min, True
        if self.position_max is not None and out > self.position_max:
            out, clamped = self.position_max, True
        return out, clamped


@dataclass(frozen=True)
class MimicJoint:
    """机械耦合声明。

    v0.1 **不实现**，字段存在是为了：

    ① 未来加平行四连杆（如 MeArm 的被动腕）时不必改 `Joint` 契约；
    ② MJCF 里**本来就有** `<joint mimic="...">` —— 如果我们不解析它，
       那它的信息就在 Loader 里被**静默丢弃**了，而"模型有耦合但 FK 不知道"
       会让 FK 与 MuJoCo 结果不一致（MuJoCo 会执行 mimic，我们不会）。
       当前的处理是：**解析并保存**，FK 遇到非 None 时**显式报错**而不是忽略。
    """

    joint: str
    multiplier: float = 1.0
    offset: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {"joint": self.joint, "multiplier": self.multiplier, "offset": self.offset}


# ---------------------------------------------------------------------------
# §24 GeometryRef
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class GeometryRef:
    """几何体**引用**（不含大型 mesh 数据）。

    ## `size` 的语义按 type 分派（对齐 MuJoCo `geom` 约定）

    | type | size | 含义 |
    |---|---|---|
    | `box` | `[hx, hy, hz]` | **半长** |
    | `sphere` | `[r]` | 半径 |
    | `cylinder` | `[r, half_len]` | 半径 + 半长 |
    | `capsule` | `[r, half_len]` | 半径 + 半长 |
    | `mesh` | `[]` | 用 `asset` 引用（v0.1 不加载） |

    ## 为什么不把 mesh 顶点放进 RobotModel

    `RobotModel` 必须能**便宜地**通过网络发送（WS `robot_info`）。
    一个 5 万顶点的 STL 会让每条 `robot_info` 变成几 MB。
    因此几何只存**引用**，真正的 asset 由前端的 AssetLoader 按需拉取
    （v0.1 只有原生 geom，`asset` 恒为 None）。

    ## `rgba` 的定位

    它**不是**物理属性，是**视觉提示**。放在这里是刻意的妥协：
    MJCF 里 geom 自带 rgba，而如果 Loader 丢掉它，前端就只能给所有几何
    上同一种颜色，"哪个 link 是哪个"在看 3D 视图时就无法分辨。
    它不参与任何物理/运动学计算。
    """

    type: str
    size: list[float] = field(default_factory=list)
    asset: str | None = None
    transform: Transform = field(default_factory=Transform.identity)
    rgba: list[float] | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": self.type,
            "size": list(self.size),
            "asset": self.asset,
            "transform": self.transform.to_dict(),
            "rgba": list(self.rgba) if self.rgba is not None else None,
        }


# ---------------------------------------------------------------------------
# §25 Inertial
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Inertial:
    """刚体惯性参数。单位：mass `kg`、center `m`、inertia `kg*m²`。

    ## 关键纪律：缺数据时**不要伪造**（契约 §3.9）

    MJCF 允许只写 `<inertial pos="..." mass="..."/>` 而不写 `diaginertia`
    （此时 MuJoCo 会**按 geom 自动估算**）。我们的处理是留 `None`，
    而不是"按 box 几何算一个"：

    - MuJoCo 的估算考虑了它自己 compilation 后的实际几何（含 geom 合并），
      比我们从一个 `GeometryRef` 猜的要准；
    - 如果我们伪造一个值并**写回** MJCF 生成链，就会让
      "我没写惯量"这个事实消失，而那是模型作者需要知道的事。
    """

    mass: float
    center_of_mass: Vector3
    ixx: float | None = None
    iyy: float | None = None
    izz: float | None = None
    ixy: float | None = None
    ixz: float | None = None
    iyz: float | None = None

    def has_full_inertia(self) -> bool:
        return all(
            v is not None for v in (self.ixx, self.iyy, self.izz, self.ixy, self.ixz, self.iyz)
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "mass": self.mass,
            "center_of_mass": self.center_of_mass.to_list(),
            "inertia": {
                "ixx": self.ixx,
                "iyy": self.iyy,
                "izz": self.izz,
                "ixy": self.ixy,
                "ixz": self.ixz,
                "iyz": self.iyz,
            },
        }


# ---------------------------------------------------------------------------
# §14 Link
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Link:
    """机器人刚体。

    ## 关于 `parent_joint is None`

    恰好**一个** Link 满足它 —— 就是 `root_link`。
    Validator 会拒绝 0 个（无根）或 2 个以上（森林，v0.1 不支持）的情况。

    ## 为什么 `child_joints` 存的是 id 而不是对象

    存对象会导致：① JSON 序列化出现循环引用（parent_link ↔ child_joints）；
    ② dataclass 的 `__eq__` 无限递归；
    ③ 复制一份 Link 就会复制整棵子树。
    存 id 则 model 是一棵"平的图"，`links` / `joints` 是两张**索引表**，
    查询走 `RobotModel.link(id)`。这与 ROS/URDF 的惯用做法一致。
    """

    id: str
    name: str
    parent_joint: str | None = None
    child_joints: list[str] = field(default_factory=list)
    inertial: Inertial | None = None
    visual: list[GeometryRef] = field(default_factory=list)
    collision: list[GeometryRef] = field(default_factory=list)

    def is_root(self) -> bool:
        return self.parent_joint is None

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "parent_joint": self.parent_joint,
            "child_joints": list(self.child_joints),
            "inertial": self.inertial.to_dict() if self.inertial else None,
            "visual": [g.to_dict() for g in self.visual],
            "collision": [g.to_dict() for g in self.collision],
        }


# ---------------------------------------------------------------------------
# §15 Joint
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Joint:
    """运动学核心元素。

    ## `axis` 对 `fixed` 关节的处理

    契约要求字段**存在**（避免 `None` 检查散落各处），值不参与运动学，
    但仍要求归一化 —— 这样"axis 一定是单位向量"这条不变量没有例外，
    消费者不必写 `if joint.type != 'fixed'` 才开始用 axis。

    ## 严禁的字段（提示词 §17）

    ```text
    clockwise / counter_clockwise / servo_direction / invert / sign
    ```

    它们是**执行器映射**概念（`Joint → Actuator Mapping → Servo`）。
    混进 Joint 会让 FK/IK 的语义依赖于具体舵机 —— 换舵机就要改运动学。
    """

    id: str
    name: str
    type: str
    parent_link: str
    child_link: str
    origin: Transform = field(default_factory=Transform.identity)
    axis: Vector3 = field(default_factory=Vector3.unit_z)
    limits: JointLimits | None = None
    mimic: MimicJoint | None = None

    def is_mobile(self) -> bool:
        """是否产生自由度（参与 FK / 产生 qpos）。"""
        return self.type in MOBILE_JOINT_TYPES

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "type": self.type,
            "parent_link": self.parent_link,
            "child_link": self.child_link,
            "origin": self.origin.to_dict(),
            "axis": self.axis.to_list(),
            "limits": self.limits.to_dict() if self.limits else None,
            "mimic": self.mimic.to_dict() if self.mimic else None,
        }


# ---------------------------------------------------------------------------
# §20 Actuator
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Actuator:
    """执行器。**v0.1：`Joint ≈ Actuator`，但概念不合并。**

    ## 为什么不合并

    未来要支持

    ```text
    Joint ──→ Actuator Mapping ──→ Servo
                     ├── offset
                     ├── direction
                     ├── scale
                     ├── coupling
                     └── mechanical linkage
    ```

    如果 v0.1 把二者合并，未来加映射表就必须改 `Joint` 契约 ⇒
    **所有已冻结的下游全部要动**（FK / IK / 前端 / 测试）。
    保留两个类型 = 未来只需在 `Actuator` 上挂 mapping 字段。

    ## `joint` 为何可空

    MJCF 允许 `<actuator>` 里出现不绑定关节的执行器（如 site 上的力）。
    可空是**忠实于 MJCF**；Validator 会把"绑定了不存在的关节"报错，
    但"没绑定关节"是合法的。
    """

    id: str
    name: str
    type: str
    joint: str | None = None
    command_min: float | None = None
    command_max: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "type": self.type,
            "joint": self.joint,
            "command_min": self.command_min,
            "command_max": self.command_max,
        }


# ---------------------------------------------------------------------------
# §21 Frame / §22 Site / §23 EndEffector
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Frame:
    """通用坐标参考系（world / base / link / joint / tool / sensor / ee）。

    ## `frames` 是**显式注册表**（契约 §2）

    `links` / `joints` / `sites` 的父坐标系由各自 `parent` 字段隐式表达；
    `frames` 只承载**额外的**参考系。于是"某名字是不是合法 Frame"有唯一答案，
    不必遍历三张表猜。

    ## `parent` 可以是 link 名或 frame 名

    这样 `world → base → tool0` 这类链能表达。
    Validator 检查 `parent ∈ (links ∪ frames)`。
    """

    id: str
    name: str
    parent: str
    transform: Transform = field(default_factory=Transform.identity)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "parent": self.parent,
            "transform": self.transform.to_dict(),
        }


@dataclass(frozen=True)
class Site:
    """轻量语义标记点（对应 MJCF `<site>`）。"""

    id: str
    name: str
    parent: str
    transform: Transform = field(default_factory=Transform.identity)
    role: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "parent": self.parent,
            "transform": self.transform.to_dict(),
            "role": self.role,
        }


@dataclass(frozen=True)
class EndEffector:
    """末端执行器。**不重复保存 Pose，只用引用**（契约 §3.7）。

    ## 为什么这是一个"必须"而非"建议"

    如果 EndEffector 自己也存一份 `Transform`，那它与 `Frame.transform`
    就有了**两份真值**。改了 Frame 忘了改 EndEffector ⇒
    UI 显示的位置与实际 FK 位置不一致。而这种 bug 的表现是
    "IK 收敛但末端位置有偏差"，排查方向会被引到 IK 数值问题上去 ——
    非常昂贵。

    ⇒ 因此 `EndEffector` 只做**引用**：`frame` 必填（必须存在于 `frames`），
      `site` 可选（若存在则用 site 的位姿做工具中心点，否则用 frame 的）。
    """

    id: str
    name: str
    frame: str
    site: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "frame": self.frame,
            "site": self.site,
        }


# ---------------------------------------------------------------------------
# §26 RobotCapabilities
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RobotCapabilities:
    """能力声明。**Frontend 必须据此动态决定显示哪些功能**（契约 §3.10）。

    ```tsx
    // ❌ 禁止
    {robot.id === 'mini_arm' && <IkPanel />}
    // ✅ 正确
    {model.capabilities.ik && <IkPanel />}
    ```

    于是"加一台没有 IK 的机器人"只需在它的 manifest 写 `ik: false`，
    前端**自动**不显示 IK 面板 —— 不必改一行前端代码。
    """

    simulation: bool = True
    fk: bool = True
    ik: bool = False
    actuator_control: bool = False
    end_effector: bool = False

    def to_dict(self) -> dict[str, bool]:
        return {
            "simulation": self.simulation,
            "fk": self.fk,
            "ik": self.ik,
            "actuator_control": self.actuator_control,
            "end_effector": self.end_effector,
        }


# ---------------------------------------------------------------------------
# §10 RobotModel
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RobotModel:
    """**RobotForge 最重要的 P0 接口。**

    ## 它是唯一表示

    所有格式（MJCF / URDF / Future）最终转成它；所有消费者只依赖它。
    Runtime 里出现 `MjModel` / `Object3D` / XML 元素 ⇒ 架构已破。

    ## 它是纯结构描述

    不含 `current joint state` / `WebSocket` / `MjData` / `asyncio.Task`。
    frozen 让这条纪律在**运行期**生效，而不只是文档里的一句话。

    ## 索引查询

    `links` / `joints` / ... 是**平表**，不是嵌套树 —— 便于 JSON 传输与
    id 查询。树结构由 `Link.parent_joint` / `Joint.parent_link` 表达。
    查询用 `link(id)` / `joint(id)` 等辅助方法（O(n)，v0.1 的模型只有个位数元素；
    若将来有上千元素的模型，再在 Loader 里建 `_index` 缓存）。
    """

    metadata: RobotMetadata
    coordinate: CoordinateConvention = field(default_factory=CoordinateConvention)
    units: UnitConvention = field(default_factory=UnitConvention)

    root_link: str = ""
    base_frame: str = ""

    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    actuators: list[Actuator] = field(default_factory=list)

    frames: list[Frame] = field(default_factory=list)
    sites: list[Site] = field(default_factory=list)
    end_effectors: list[EndEffector] = field(default_factory=list)

    capabilities: RobotCapabilities = field(default_factory=RobotCapabilities)

    # ---- 索引查询 ----

    def link(self, link_id: str) -> Link:
        """按 id 取 Link。找不到 ⇒ `KeyError`（不返回 None）。

        刻意**不**返回 `None`：`None` 会被调用方写成
        `if (l := model.link(x)) is not None:`，而漏写判断就变成 `AttributeError`。
        直接抛 `KeyError` 让"引用了不存在的 Link"在**第一现场**暴露。
        Validator 已保证引用完整性，所以运行期抛错说明模型没走 Validator。
        """
        for item in self.links:
            if item.id == link_id:
                return item
        raise KeyError(f"RobotModel 里没有 link {link_id!r}（可用: {self.link_ids()})")

    def joint(self, joint_id: str) -> Joint:
        for item in self.joints:
            if item.id == joint_id:
                return item
        raise KeyError(f"RobotModel 里没有 joint {joint_id!r}（可用: {self.joint_ids()})")

    def frame(self, frame_id: str) -> Frame:
        for item in self.frames:
            if item.id == frame_id:
                return item
        raise KeyError(f"RobotModel 里没有 frame {frame_id!r}（可用: {self.frame_ids()})")

    def site(self, site_id: str) -> Site:
        for item in self.sites:
            if item.id == site_id:
                return item
        raise KeyError(f"RobotModel 里没有 site {site_id!r}（可用: {self.site_ids()})")

    def actuator(self, actuator_id: str) -> Actuator:
        for item in self.actuators:
            if item.id == actuator_id:
                return item
        raise KeyError(f"RobotModel 里没有 actuator {actuator_id!r}")

    def end_effector(self, ee_id: str) -> EndEffector:
        for item in self.end_effectors:
            if item.id == ee_id:
                return item
        raise KeyError(f"RobotModel 里没有 end_effector {ee_id!r}")

    def has_link(self, link_id: str) -> bool:
        return any(i.id == link_id for i in self.links)

    def has_frame(self, frame_id: str) -> bool:
        return any(i.id == frame_id for i in self.frames)

    def has_joint(self, joint_id: str) -> bool:
        return any(i.id == joint_id for i in self.joints)

    # ---- id 列表 ----

    def link_ids(self) -> list[str]:
        return [i.id for i in self.links]

    def joint_ids(self) -> list[str]:
        return [i.id for i in self.joints]

    def frame_ids(self) -> list[str]:
        return [i.id for i in self.frames]

    def site_ids(self) -> list[str]:
        return [i.id for i in self.sites]

    def actuator_ids(self) -> list[str]:
        return [i.id for i in self.actuators]

    def end_effector_ids(self) -> list[str]:
        return [i.id for i in self.end_effectors]

    # ---- 运动学视图 ----

    def mobile_joint_ids(self) -> list[str]:
        """有自由度的关节（`revolute` / `prismatic`），**保持模型声明顺序**。

        顺序即 FK/IK 的向量顺序 —— 这一点必须与 MuJoCo 的 `qpos` 顺序一致，
        否则 Command 里的角度会被配错关节（表现为"发 shoulder 动 elbow"）。
        `test_mujoco.py` 会断言这个顺序与 `MjModel.jnt_qposadr` 排序一致。
        """
        return [j.id for j in self.joints if j.is_mobile()]

    def dof(self) -> int:
        return len(self.mobile_joint_ids())

    def default_end_effector(self) -> EndEffector | None:
        """默认末端执行器（第一个）。无 ⇒ `None`。"""
        return self.end_effectors[0] if self.end_effectors else None

    # ---- 序列化 ----

    def to_dict(self) -> dict[str, Any]:
        """JSON 友好形式（WS `robot_info` 的载荷）。

        只含 `dict / list / str / float / bool / None`。
        `test_robot_model.py::test_model_json_serializable` 会真的过一遍
        `json.dumps` 来验证这一点，而不是靠人工检查。
        """
        return {
            "metadata": self.metadata.to_dict(),
            "coordinate": self.coordinate.to_dict(),
            "units": self.units.to_dict(),
            "root_link": self.root_link,
            "base_frame": self.base_frame,
            "links": [i.to_dict() for i in self.links],
            "joints": [i.to_dict() for i in self.joints],
            "actuators": [i.to_dict() for i in self.actuators],
            "frames": [i.to_dict() for i in self.frames],
            "sites": [i.to_dict() for i in self.sites],
            "end_effectors": [i.to_dict() for i in self.end_effectors],
            "capabilities": self.capabilities.to_dict(),
        }

    def summary_line(self) -> str:
        """单行摘要（日志 / CLI 用）。"""
        return (
            f"{self.metadata.id} v{self.metadata.version} · "
            f"links={len(self.links)} joints={len(self.joints)}(dof={self.dof()}) "
            f"actuators={len(self.actuators)} frames={len(self.frames)} "
            f"sites={len(self.sites)} ee={len(self.end_effectors)} · "
            f"ik={'yes' if self.capabilities.ik else 'no'}"
        )
