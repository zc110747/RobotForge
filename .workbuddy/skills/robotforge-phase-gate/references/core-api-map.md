# RobotForge Core API 速查（实测，2026-09-15）

> **为什么需要这份表**：写测试时猜 API 名已经浪费了大量时间。项目至今
> 至少猜错 12 处字段名/方法性/返回值形状。动手前对照这张表，不要再猜。

## 1. `backend/model/types.py`

```python
Vector3(x, y, z)          # frozen；__post_init__ 拒绝 NaN/inf
  .x .y .z
  .dot(other) -> float
  .cross(other) -> Vector3
  .norm() -> float                    # ⚠ 不是 .length / .magnitude
  .normalized() -> Vector3
  .is_finite() -> bool
  .is_normalized(tol=...) -> bool
  .approx_eq(other, tol=...) -> bool
  .to_list() .to_numpy() .to_dict()
  Vector3.zero() .unit_x() .unit_y() .unit_z() .of(seq)

Quaternion(x, y, z, w)    # frozen；约定 **[x, y, z, w]**，identity = (0,0,0,1)
  Quaternion.identity()
  .to_numpy_wxyz()                  # 给 MuJoCo 用（w 在前）
  .normalized(where=...)
  .is_normalized(tol=...)
  .approx_eq(other, tol=...)
  q1 * q2                            # 组合：先作用 q2 再作用 q1

Transform(position: Vector3, orientation: Quaternion)
  .position                         # ⚠ 不是 .pos / .translation
  .orientation
  .compose(child)                   # **会旋转** child 的平移，不是简单相加
  .inverse()
  .from_parts(...)
  Transform.identity()
```

**NaN 由类型层拦截**（构造即 `ValueError`），所以 validator 不需要重复查
NaN —— 要触发 validator 的 `transform.nan` 必须用 `object.__setattr__`
绕过构造函数（见 `tests/test_validator.py` 的 `_all_counterexamples`）。

## 2. `backend/model/robot_model.py`

```python
JOINT_TYPES        = ("fixed", "revolute", "prismatic")
MOBILE_JOINT_TYPES = frozenset({"revolute", "prismatic"})
ACTUATOR_TYPES     = ("position", "velocity", "torque", "motor")
GEOMETRY_TYPES     = ("box", "sphere", "cylinder", "capsule", "mesh")

ROBOTFORGE_COORDINATE   # ⚠ 是 **dict**，不是 dataclass 实例
  {'convention':'robotforge','handedness':'right',
   'forward_axis':'x','left_axis':'y','up_axis':'z'}
ROBOTFORGE_UNITS        # ⚠ 同样是 dict
  {'length':'m','angle':'rad','time':'s',
   'linear_velocity':'m/s','angular_velocity':'rad/s',
   'force':'N','torque':'N*m'}

RobotMetadata       : id, name, version, description
CoordinateConvention: convention, handedness, forward_axis, left_axis, up_axis
                      .is_robotforge() -> bool
UnitConvention      : length, angle, time, linear_velocity,
                      angular_velocity, force, torque
                      .is_si() -> bool
JointLimits         : position_min, position_max, velocity_max, effort_max
                      # ⚠ 不是 .lower / .upper
                      .has_position_bounds() -> bool
                      .clamps(value) -> (value, clamped_bool)
Inertial            : mass, center_of_mass, ixx, iyy, izz, ixy, ixz, iyz
                      .has_full_inertia() -> bool
GeometryRef         : type, size, asset, transform, rgba
Link                : id, name, parent_joint, child_joints,
                      inertial, visual, collision
                      # ⚠ 没有 .geometries / .origin；几何在 visual + collision
                      .is_root() -> bool          # ⚠ 是**方法**
Joint               : id, name, type, parent_link, child_link,
                      origin, axis, limits, mimic
                      .is_mobile() -> bool        # ⚠ 是**方法**
Site                : id, name, parent, transform, role
Frame               : id, name, parent, transform
EndEffector         : id, name, frame, site
Actuator            : id, name, type, joint, command_min, command_max
MimicJoint          : joint, multiplier, offset
RobotCapabilities   : simulation, fk, ik, actuator_control, end_effector

RobotModel (frozen)
  metadata, coordinate, units, root_link, base_frame,
  links, joints, actuators, frames, sites, end_effectors, capabilities
  # ⚠ 没有 .id —— id 在 model.metadata.id
  .link(id) .joint(id) .frame(id) .site(id) .actuator(id) .end_effector(id)
     # 找不到时抛 **KeyError**（不是返回 None）
  .has_link(id) .has_frame(id) .has_joint(id) -> bool
  .link_ids() .joint_ids() .frame_ids() .site_ids()
  .actuator_ids() .end_effector_ids()
  .mobile_joint_ids() -> list[str]
  .dof() -> int
  .default_end_effector() -> EndEffector | None
  .summary_line() -> str
```

**`frozen=True` 的两层强制**：顶层 `frozen=True` **挡不住** `model.links.append(...)`
（列表原地修改）。所以契约由两层共同保证：`frozen=True` + 加载期 validator。
测试里应显式记录这一点（见 `TestFrozenContract`）。

## 3. `backend/model/validator.py`

```python
ID_PATTERN = re.compile(r"^[a-z_][a-z0-9_]*$")

Issue            : level, code, where, message ;  .line() -> str
ValidationReport : robot_id, issues
                   .ok  .errors  .warnings  .format()
                   .error(code, where, message)  .warn(code, where, message)
                   .raise_if_invalid() -> self
RobotModelError  : ValueError 子类

validate_robot_model(model) -> ValidationReport   # **绝不抛异常**
assert_valid(model) -> RobotModel                 # 失败抛 RobotModelError
summarize(report) -> dict                         # 可 JSON 序列化
```

`report.robot_id` == `model.metadata.id`。

### 规则码全表（46 error + 5 warning）

```text
metadata.empty / metadata.id_format
coordinate.mismatch / coordinate.axes_not_permutation
units.mismatch
id.duplicate / id.format
ref.joint.parent_link / ref.joint.child_link / ref.joint.mimic
ref.link.parent_joint / ref.link.child_joint
ref.actuator.joint / ref.frame.parent / ref.site.parent
ref.end_effector.frame / ref.end_effector.site
tree.root_link_empty / tree.root_link_missing / tree.root_link_not_root
tree.multiple_roots / tree.multi_parent / tree.orphan_link / tree.cycle
tree.base_frame_empty / tree.base_frame_missing
joint.type / joint.axis_not_normalized / joint.axis_zero
joint.limits_inverted / joint.limits_velocity / joint.limits_effort
actuator.type / actuator.command_range_inverted
frame.cycle
end_effector.frame_empty
transform.nan / transform.quaternion_not_normalized
geometry.type / geometry.size / geometry.size_nonpositive
geometry.mesh_no_asset / geometry.rgba
inertial.mass
capabilities.ee_mismatch / capabilities.actuator_mismatch

--- warning ---
joint.no_position_limit          （可动关节无完整位置限位）
joint.mimic_not_implemented      （v0.1 不执行 mimic 耦合，MuJoCo 会）
inertial.zero_mass
capabilities.ik_without_ee
capabilities.fk_no_dof
```

**分级理由**（不是随手定的，改动前先读）：

- `joint.no_position_limit` 是 warning：无限位在**运动学**上良定义，只在
  **物理执行**上做不到。当 error 会让研究用模型无法加载。
- `transform.nan` 是 error 但类型层已挡：留作"绕过构造函数的输入"（Loader
  出错 / JSON 反序列化漏检）的兜底。

## 4. `backend/loaders/loader.py` / `mjcf_loader.py`

```python
LoaderReport   : ...
LoaderError(RuntimeError)
RobotModelLoader(ABC)          # 抽象基类，位于 loader.py（**不是** base.py）
  @abstractmethod load(...)

MJCFLoader(...)                # __init__ 全是 keyword-only 且可选
  .load(path) -> **tuple**      # ⚠ 返回 (model, report)，不是裸 model
                                # ⚠ 也可能直接是 model —— 用
                                #   r[0] if isinstance(r, tuple) else r
sanitize_id(s, fallback=...)
  # 非 ASCII → fallback
  # [^a-z0-9]+ → "_"（**连续多个折叠成一个**：joint__x → joint_x）
  # 前导数字 → 前缀 "_"
_assign_id(...)                 # 重复 id 抛 LoaderError，**绝不自动加后缀**
```

`MJCFLoader` 的 `import mujoco` 必须是**懒加载**（在函数体内），
`tests/test_mjcf_loader.py` 有源码扫描测试钉住它。

## 5. `packages/mini_arm/kinematics/fk.py`

```python
BASE_HEIGHT = 0.084 ; SHOULDER_OFFSET = 0.052
L1 = 0.103 ; L2 = 0.065 ; L_TOOL = 0.032
JOINT_ORDER = ("base_yaw", "shoulder", "elbow")

forward_kinematics(model, joint_positions) -> Transform
    # 解析式；几何用**常量**，只从 model 取旋转轴（_axis_of）+ 算四元数
    # 未提供的关节按 0 处理；多余的键忽略；不修改输入/model
    # 返回 **TCP**（零位 x = 0.200，不是 0.168 的法兰）
    # orientation 不含工具偏移旋转（tcp 相对 ee_link 只有平移）

forward_kinematics_generic(model, joint_positions) -> Transform
    # 通用链式；几何**全部**来自 model（Phase 3 提升为 Core 引擎的原型）

link_transforms(model, joint_positions) -> dict[link_id, Transform]
    # 覆盖全部 link，供前端逐段渲染
tcp_offset(model) -> Transform
_axis_of(model, joint_id) -> Vector3     # 直接 model.joint(id).axis
_origin_of(...) ; _ee_link_id(...) ; _q_revolute(axis, angle)
```

## 6. `packages/mini_arm/kinematics/ik.py`

```python
TOL_SINGULAR = 1e-6
L2_EFF = L2 + L_TOOL          # 0.097
Z_BASE = BASE_HEIGHT + SHOULDER_OFFSET   # 0.136
YAW_LIMIT = π ; SHOULDER_LIMIT = π/2 ; ELBOW_LIMIT = 3π/4

IkSolution  : joint_positions, branch, position_error, clamped,
              clamped_joints, orientation_error ; .to_dict()
UnreachableError(IkError) : 携带 .distance / .reach_min / .reach_max
LimitViolationError(IkError)
IkError

solve(model, target, *, branch="elbow_up", prefer_limits=True,
      clamp=False, tol=1e-6) -> IkSolution
solve_all(model, target, *, clamp=False) -> list[IkSolution]   # 按误差升序
reach_limits() -> (r_min, r_max)      # 平面半径，不含底座高度
workspace_sample(n) -> ...
_side_offset(theta2) -> float         # 分支判据：肘在线哪一侧（**不是 θ2 符号**）
_orientation_error(a, b) -> float     # 用 abs(dot)，容 dual cover
```

### ⚠ v0.1 的 IK 不读 model 的几何

`'axis'` 出现 0 次、`model.<属性>` 出现 0 次。几何与限位全来自上述常量
（这些常量从 `fk.py` 复用）。改 `shoulder` 的 axis 后 `solve()` 返回**完全
相同**的关节角。`forward_kinematics(model, raw)` 的回代自检**会**用传入的
model（朝向那一路）。

## 7. 关键数值常量速查

```text
肩的世界 z            = 0.136
零位 TCP              = (0.200, 0, 0.136)
r_max（位置可达）     = 0.200      = L1 + L2_EFF
r_min（完全折叠）     = 0.006      = |L1 - L2_EFF|
r_lim（姿态可达）     = 0.076737164
   = sqrt(L1² + L2_EFF² + 2·L1·L2_EFF·cos 135°)
clamp 到 135° 的残差  ≈ 0.074241 m （solve_all 的阈值 1e-3 会拒绝它）

肘扫描（验证 MJCF 几何用）：
  sh=-90° → elbow=(0, 0, 0.239)
  sh=-45° → (0.0728, 0, 0.2088)
  sh=  0° → (0.103, 0, 0.136)
  sh=+45° → (0.0728, 0, 0.0632)
  sh=+90° → (0, 0, 0.033)        半径恒为 L1 = 0.103 的圆弧
```
