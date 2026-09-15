# RobotForge 坐标系与单位契约 (P0)

> **状态：已冻结 (FROZEN, v1)**
>
> 本文件是 RobotForge v0.1 的 P0 契约之一。任何修改必须在 `tests/test_coordinate.py`
> 与 `tests/test_units.py` 同步升级，并记录在本文档的「变更记录」中。
> 契约未冻结前不得实现 FK / IK / MuJoCo / Three.js（提示词 §76）。

---

## 1. 为什么坐标与单位是 P0

机器人项目里最贵的一类错误不是崩溃，而是**一切都能跑，但结果是错的**：

```text
角度当弧度   → 关节转 57 倍，限位形同虚设，仿真"看起来在动"
毫米当米     → 机械臂放大 1000 倍，穿模但不报错
四元数顺序错 → 姿态整体镜像，FK 自测通过（正反解用同一个错顺序）
左手/右手混  → 单轴反转，只有复合运动才暴露
```

这些错误都**不会**在类型检查或编译期暴露。因此 RobotForge 的处理方式是：

> **把坐标系与单位写成可执行断言，而不是文档里的一句话。**

对应测试：`tests/test_coordinate.py`、`tests/test_units.py`。

---

## 2. 坐标系（CoordinateConvention）

v0.1 **固定**，不接受自定义：

| 字段 | 值 | 语义 |
|---|---|---|
| `convention` | `robotforge` | 约定名（用于将来识别第三方模型的坐标系） |
| `handedness` | `right` | 右手系 |
| `forward_axis` | `x` | **+X = Forward**（前） |
| `left_axis` | `y` | **+Y = Left**（左） |
| `up_axis` | `z` | **+Z = Up**（上） |

### 2.1 必须满足的恒等式

```text
X × Y = Z          # 右手系（叉积）
Y × Z = X
Z × X = Y
```

这条恒等式是**唯一**能机器判定的"右手系"判据，因此它是 `test_coordinate.py` 的第一条断言。
`X × Y = -Z` 就是左手系，会在复合旋转时表现为镜像。

### 2.2 地面与基座

```text
世界地面：Z = 0
机器人基座：+X = 机器人前方, +Y = 机器人左方, +Z = 机器人上方
```

世界坐标系与机器人基座坐标系**同向**（不为基座引入额外旋转）。
如果某台机器人的 MJCF 基座朝向不同，转换必须发生在 `MJCFLoader`（见 §4）。

### 2.3 关节正方向

> **Right-Hand Rule（右手定则）**

绕 `axis` 向量按右手螺旋为正。

RobotForge **明令禁止**在 Joint 上定义：

```text
clockwise / counter_clockwise
servo_direction
invert
sign
```

这些是**执行器映射**层面的概念（`Joint → Actuator Mapping → Servo`），
属于未来的 `actuator mapping`，不属于运动学本体（提示词 §17）。
把它们混进 Joint 会让 FK/IK 的语义依赖于具体舵机，架构即破。

---

## 3. 单位（UnitConvention）

v0.1 **固定 SI**：

| 物理量 | 单位 | 字段 |
|---|---|---|
| 长度 | `m` | `length` |
| 角度 | `rad` | `angle` |
| 时间 | `s` | `time` |
| 线速度 | `m/s` | `linear_velocity` |
| 角速度 | `rad/s` | `angular_velocity` |
| 力 | `N` | `force` |
| 力矩 | `N*m` | `torque` |

### 3.1 关节量的单位由**类型**决定，不由字段名编码

| Joint type | position | velocity |
|---|---|---|
| `revolute` | `rad` | `rad/s` |
| `prismatic` | `m` | `m/s` |
| `fixed` | —（无自由度） | — |

**禁止**出现这样的字段名：

```text
min_degree / max_degree
min_mm / max_mm
```

> 字段表达**物理意义**（`position_min`），单位由本契约全局规定。
> 在字段名里编码单位 = 允许同一个模型里出现两种单位，这正是要消灭的东西。

### 3.2 禁止散落的单位转换

以下写法在 `backend/` 里**全部不允许**：

```python
value / 1000
value * 1000
math.radians(value)
math.degrees(value)
value * 180 / math.pi
```

单位转换**只能**存在于五个边界：

```text
Loader            外部模型 → RobotModel
Adapter           坐标系/单位差异适配
Converter         显式转换工具函数（unit_converter.py）
UI Boundary       前端显示层（rad → deg 仅供显示）
Protocol Boundary WebSocket 收发时（若对端约定非 SI）
```

`test_units.py::test_no_scattered_unit_conversion` 会对 `backend/` 做源码扫描，
发现上述模式即失败。这是**故意**的机械检查——它把"约定"变成"会失败的测试"。

---

## 4. 坐标转换只允许在 Loader / Adapter

```text
第三方 MJCF（坐标系不同）
      ↓
   MJCFLoader          ← 坐标转换只在这里发生
      ↓
   RobotModel          ← 之后全局统一为 RobotForge 坐标系
      ↓
 FK / IK / Runtime / WebSocket / Frontend
      ↑
   禁止在这里做坐标转换
```

### 4.1 三种情形

| 情形 | 处理 |
|---|---|
| MJCF 已是 RobotForge 约定（+X前/+Y左/+Z上） | **No Conversion**，`transform` 原样搬运 |
| MJCF 是 Z-up 但 Y 朝右（常见） | Loader 里左乘 Y 轴 180° 旋转，并**记录到 `metadata`** |
| MJCF 是 Y-up（Three.js/GLTF 风格） | Loader 里做 Z-up 转换 |

v0.1 的 `mini_arm` 属于第一种，因此 `MJCFLoader` 的转换矩阵是**单位矩阵**——
但这条通路必须存在且被测试，否则加第三台机器人时会在 FK 里偷偷改坐标。

### 4.2 Three.js 的 Z-up 处理（关键决策）

Three.js 世界观是 **Y-up**。RobotForge 内部是 **Z-up**。
两者相遇时，处理方式**不是**修改 RobotModel，而是：

```text
RobotModel（Z-up，永不改）
      ↓
Three.js Renderer Adapter
      ↓  scene.rotation.x = -Math.PI / 2   （整场景旋转，而非逐 body 旋转）
Three.js（Y-up 表面）
```

**为什么用整场景旋转而不是逐节点转换**：

- 整场景旋转是**一次**变换，不可能漏，也不可能与 FK 不一致；
- 逐节点转换需要在每个 body 上做矩阵乘法，漏一个就出现"某一节朝向诡异"，
  而这类 bug 的表现是"看起来还行"；
- 更重要：整场景旋转保证了 **Three.js 里读出的局部关节角与 RobotModel 完全同源**，
  因此 `test_fk_threejs_consistency` 才能做"同状态同语义"的断言。

代价是 Three.js 世界坐标 = `(x, z, -y)`，与 RobotForge 不同。
这是**允许且预期**的：它属于"UI Boundary"，`viewer/coordinateAdapter.ts` 是唯一出口。

---

## 5. 位姿契约（Pose Contract）

### 5.1 Transform

```python
class Transform:
    position: Vector3      # [x, y, z]  单位 m
    orientation: Quaternion # [x, y, z, w]  单位长度
```

### 5.2 四元数顺序

> **RobotForge 全局统一 `[x, y, z, w]`**

**禁止**混用 `[w, x, y, z]`。

| 库/框架 | 其顺序 | 交界处处理 |
|---|---|---|
| RobotForge 内部 | `[x,y,z,w]` | — |
| MuJoCo `qpos` (free joint) | `[w,x,y,z]` | `mujoco_backend.py` 边界转换 |
| Python `numpy.quaternion` | 视实现 | 不引入 |
| Three.js `Quaternion` | 内部 `x,y,z,w` | 天然一致 ✅ |

单位四元数：

```text
[0, 0, 0, 1]
```

必须满足 `|q| ≈ 1`（容差 `1e-9`）。`RobotModelValidator` 会检查它。

### 5.3 欧拉角

欧拉角**只能**用于：

```text
UI 显示
Debug 输出
人工输入框
```

**不得**作为 RobotModel 的标准姿态表示。使用时的轴序约定：

```text
Roll  = X
Pitch = Y
Yaw   = Z
```

### 5.4 MuJoCo 的交界

MuJoCo 内部是 Z-up、SI 单位、`[w,x,y,z]` 四元数。因此交界处**只有一处**差异：

```text
position:     直接对应（都是 m, Z-up）   ✅ 无转换
quaternion:   [w,x,y,z] ↔ [x,y,z,w]     ⚠️ 需转换，位置在 mujoco_backend.py
```

MuJoCo 的关节角本身就是 rad、Z-up、右手定则 ⇒ 与 RobotForge **完全同源**，
这是选择 MuJoCo 作为 v0.1 物理后端的重要理由。

---

## 6. 验收清单（自动化）

| # | 判据 | 测试 |
|---|---|---|
| 1 | `X × Y = Z`（右手系） | `test_coordinate.py::test_right_handedness` |
| 2 | 单位四元数 = `[0,0,0,1]` | `test_coordinate.py::test_identity_quaternion` |
| 3 | 四元数归一化 | `test_coordinate.py::test_quaternion_normalized` |
| 4 | `length = m / angle = rad / time = s` | `test_units.py::test_si_convention_frozen` |
| 5 | backend 无散落单位转换 | `test_units.py::test_no_scattered_unit_conversion` |
| 6 | 无轴反转（每轴独立验证） | `test_coordinate.py::test_axis_not_flipped` |
| 7 | 无左手/右手混用 | `test_coordinate.py::test_right_handedness` |
| 8 | 无 quaternion 顺序错误 | `test_coordinate.py::test_quaternion_convention_x_y_z_w` |
| 9 | FK / MuJoCo / Three.js 语义一致 | `test_mujoco.py`、`frontend` 一致性用例 |
| 10 | Loader 之后无坐标系差异 | `test_mjcf_loader.py::test_loader_normalizes_coordinate` |

---

## 7. 变更记录

| 版本 | 日期 | 变更 | 影响 |
|---|---|---|---|
| v1 | 2026-09-15 | 初始冻结：Z-up / +X前 / +Y左 / SI / `[x,y,z,w]` | RobotForge v0.1 基线 |

> **冻结规则**：本契约的字段名、单位、四元数顺序、坐标系定义**不可**在 v0.1 期间变更。
> 若必须变更，需同时：① 升级 `convention` 字符串；② 在 Loader 加入旧版→新版转换；
> ③ 升级全部相关测试；④ 在本文档记录。**不允许静默变更。**
