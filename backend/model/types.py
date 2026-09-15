"""RobotForge Core 的**基础向量 / 几何类型**（P0 契约的底层）。

## 为什么不用 numpy 数组直接当 Vector3

三个理由，都是踩过的坑：

1. **可变性**。`np.array` 是可变的，而 `RobotModel` 是 frozen dataclass。
   如果 `Joint.axis` 是一个 `np.ndarray`，那么
   `model.joints[0].axis[0] = 99` 就能**绕过** frozen 保护静默改写模型。
   本模块的 `Vector3` 是 frozen dataclass，且构造时拷贝进 `tuple`，
   于是"模型不可变"是真正成立的，而不是看起来成立。

2. **相等语义**。`np.array([1,2,3]) == np.array([1,2,3])` 返回的是**数组**
   而不是 `bool`。这会让 `assert model_a == model_b` 在 pytest 里
   报出 "truth value of an array is ambiguous" 甚至**静默通过**
   （单个元素的数组）。RobotModel 的相等性必须干净。

3. **序列化**。`json.dumps(np.float32(1.5))` 直接抛
   `TypeError: Object of type float32 is not JSON serializable`。
   而 RobotModel 必须能发给前端（WebSocket 的 `robot_info`）。
   frozen dataclass + Python `float` 天然可序列化。

因此：**边界处（Loader / Backend）用 numpy 做计算，契约对象里存纯 Python 数值。**
计算密集的 FK/IK 内部随意用 numpy，但**不把 numpy 对象泄漏进契约**。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

#: 浮点比较容差。为什么是 1e-9：
#: - 单精度 float32 的机器 epsilon ≈ 1.19e-7，所以若上游是 float32 数据，
#:   1e-9 会**过严**（真实数据永远通不过）。
#: - 但我们只对"数学上应当精确成立"的量用它：
#:   归一化后的 |axis|、|quaternion|、单位矩阵乘积。
#:   这些量即便经过 float32 往返，误差也在 1e-7 量级 —— 因此单独给
#:   "从外部数据来的量"用宽松容差 TOL_EXTERNAL。
TOL_EXACT = 1e-9
#: 用于"从外部模型文件读入后可能经过精度损失"的比较（如 MJCF 的 float 文本）
TOL_EXTERNAL = 1e-6


def _f(value: Any, where: str) -> float:
    """转成 Python float，并拒绝非有限值。

    拒绝 NaN / inf 是刻意的：一个 NaN 关节角会**静默**污染整条 FK 链，
    最终表现为 "end effector 位置是 nan"，而排查方向会被引到 IK 而不是数据源。
    在**入口**就报错，错误信息里带 `where`，能一眼看出是哪个字段。
    """
    try:
        out = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{where} 必须是数值（当前 = {value!r}）") from exc
    if not math.isfinite(out):
        raise ValueError(f"{where} 必须是有限数（当前 = {out!r}）；NaN/inf 会静默污染整条运动学链")
    return out


@dataclass(frozen=True)
class Vector3:
    """三维向量。长度单位 m（或按字段语义为无量纲方向）。"""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _f(self.x, "Vector3.x"))
        object.__setattr__(self, "y", _f(self.y, "Vector3.y"))
        object.__setattr__(self, "z", _f(self.z, "Vector3.z"))

    # ---- 构造 ----

    @classmethod
    def of(cls, value: "Vector3 | Sequence[float] | Iterable[float]", where: str = "Vector3") -> "Vector3":
        """从任意序列构造（`[x,y,z]` / `(x,y,z)` / `Vector3`）。

        长度**必须是 3**。长度不对时报错而不是补齐 ——
        补齐（如 `[0,0]` 变 `[0,0,0]`）会让"少写了一个坐标"变成静默的 z=0，
        而 z=0 恰好是很多情况下的合法值，于是错误被完美隐藏。
        """
        if isinstance(value, Vector3):
            return value
        seq = list(value)
        if len(seq) != 3:
            raise ValueError(f"{where} 必须是 3 个分量（当前 = {seq!r}，长度 {len(seq)}）")
        return cls(seq[0], seq[1], seq[2])

    @classmethod
    def zero(cls) -> "Vector3":
        return cls(0.0, 0.0, 0.0)

    @classmethod
    def unit_x(cls) -> "Vector3":
        return cls(1.0, 0.0, 0.0)

    @classmethod
    def unit_y(cls) -> "Vector3":
        return cls(0.0, 1.0, 0.0)

    @classmethod
    def unit_z(cls) -> "Vector3":
        return cls(0.0, 0.0, 1.0)

    # ---- 转换 ----

    def to_list(self) -> list[float]:
        """JSON 友好形式。**唯一**的序列化出口。"""
        return [self.x, self.y, self.z]

    def to_numpy(self):
        """给计算用（Loader / FK / IK 内部）。不进入契约对象。"""
        import numpy as np

        return np.array([self.x, self.y, self.z], dtype=np.float64)

    # ---- 运算 ----

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, s: float) -> "Vector3":
        s = _f(s, "Vector3 * scalar")
        return Vector3(self.x * s, self.y * s, self.z * s)

    __rmul__ = __mul__

    def cross(self, other: "Vector3") -> "Vector3":
        """叉积。右手系判据 `X × Y = Z` 就用它验证。"""
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def dot(self, other: "Vector3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def normalized(self, where: str = "Vector3") -> "Vector3":
        """归一化。零向量 ⇒ 报错。

        零向量**不能**归一化，而"悄悄返回零向量"的后果是
        关节轴变成 `[0,0,0]` ⇒ FK 里绕零轴旋转 = 无旋转 ⇒
        IK 永远解不到目标，而错误信息会是"IK 不收敛"。
        """
        n = self.norm()
        if n < 1e-12:
            raise ValueError(f"{where} 是零向量，无法归一化（关节轴不能为零向量）")
        return Vector3(self.x / n, self.y / n, self.z / n)

    def is_normalized(self, tol: float = TOL_EXTERNAL) -> bool:
        return abs(self.norm() - 1.0) <= tol

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.x, self.y, self.z))

    def approx_eq(self, other: "Vector3", tol: float = TOL_EXTERNAL) -> bool:
        return all(abs(a - b) <= tol for a, b in zip(self.to_list(), other.to_list()))

    def __repr__(self) -> str:
        return f"[{self.x:g}, {self.y:g}, {self.z:g}]"


@dataclass(frozen=True)
class Quaternion:
    """四元数，**RobotForge 全局统一 `[x, y, z, w]` 顺序**。

    ## 为什么顺序是 P0

    四元数顺序错误是本项目最隐蔽的一类 bug：如果 FK 输出与 IK 输入
    用了同一个（错误的）约定，那么 `FK → IK → FK` 的往返测试会**完美通过**，
    而 WebSocket / Three.js / MuJoCo 交界处却全错。

    ⇒ 因此本类型**只**接受 `[x,y,z,w]`，并且：
      ① 字段名就叫 x/y/z/w（不是 q0..q3），让读代码的人无法误解；
      ② `to_list()` 的文档里写明顺序；
      ③ MuJoCo 的 `[w,x,y,z]` 转换被限制在 `mujoco_backend.py` 单点（有测试盯）。

    ## 为什么不用 scipy / numpy.quaternion

    - `scipy` 是 v0.1 不需要的依赖（只用四元数乘法，20 行代码）；
    - `numpy-quaternion` 的顺序由库决定（`w,x,y,z` 或 `x,y,z,w` 随版本/函数而异），
      引入它就是引入一个**必须记住的例外**。自实现 20 行的顺序是**显式**的。
    """

    x: float
    y: float
    z: float
    w: float

    def __post_init__(self) -> None:
        for f in ("x", "y", "z", "w"):
            object.__setattr__(self, f, _f(getattr(self, f), f"Quaternion.{f}"))

    # ---- 构造 ----

    @classmethod
    def of(cls, value: "Quaternion | Sequence[float]", where: str = "Quaternion") -> "Quaternion":
        if isinstance(value, Quaternion):
            return value
        seq = list(value)
        if len(seq) != 4:
            raise ValueError(
                f"{where} 必须是 4 个分量 [x,y,z,w]（当前 = {seq!r}，长度 {len(seq)}）"
            )
        return cls(seq[0], seq[1], seq[2], seq[3])

    @classmethod
    def identity(cls) -> "Quaternion":
        """单位四元数 = `[0, 0, 0, 1]`。P0 验收项之一。"""
        return cls(0.0, 0.0, 0.0, 1.0)

    @classmethod
    def from_axis_angle(cls, axis: Vector3, angle_rad: float) -> "Quaternion":
        """绕**已归一化**的 axis 旋转 angle_rad（右手定则）。

        轴未归一化时**自动归一化**（此处刻意宽松）：调用方（Loader / IK）
        常常已有归一化轴，重复归一化的代价是几个 flops；
        但若这里要求严格，就会鼓励调用方在别处手工归一化 —— 那更容易漏。
        """
        a = axis.normalized("Quaternion.from_axis_angle(axis)")
        half = _f(angle_rad, "Quaternion.from_axis_angle(angle_rad)") / 2.0
        s = math.sin(half)
        return cls(a.x * s, a.y * s, a.z * s, math.cos(half))

    @classmethod
    def from_rpy(cls, roll: float, pitch: float, yaw: float) -> "Quaternion":
        """欧拉角 → 四元数，轴序 **Roll=X / Pitch=Y / Yaw=Z**（P0 契约 §5.3）。

        内旋（intrinsic）XYZ 顺序，等价于 `Rz(yaw) * Ry(pitch) * Rx(roll)`。
        仅用于 UI / Debug / 人工输入边界 —— **不得**作为 RobotModel 的标准姿态表示。
        """
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return cls(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    # ---- 转换 ----

    def to_list(self) -> list[float]:
        """JSON 友好形式，顺序 **[x, y, z, w]**。"""
        return [self.x, self.y, self.z, self.w]

    def to_numpy_wxyz(self):
        """MuJoCo 顺序 `[w,x,y,z]`。**唯一**允许产生该顺序的地方之外的辅助。"""
        import numpy as np

        return np.array([self.w, self.x, self.y, self.z], dtype=np.float64)

    # ---- 运算 ----

    def __mul__(self, other: "Quaternion") -> "Quaternion":
        """Hamilton 积（`self * other` 表示"先 other 后 self"的复合旋转）。"""
        return Quaternion(
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        )

    def conjugate(self) -> "Quaternion":
        return Quaternion(-self.x, -self.y, -self.z, self.w)

    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)

    def normalized(self, where: str = "Quaternion") -> "Quaternion":
        n = self.norm()
        if n < 1e-12:
            raise ValueError(f"{where} 是零四元数，无法归一化")
        return Quaternion(self.x / n, self.y / n, self.z / n, self.w / n)

    def is_normalized(self, tol: float = TOL_EXTERNAL) -> bool:
        return abs(self.norm() - 1.0) <= tol

    def rotate(self, v: Vector3) -> Vector3:
        """用本四元数旋转向量（`q v q*`，展开成无临时四元数的形式）。"""
        # 展开形式比 q*v*q^-1 少两次四元数乘法，且不引入中间归一化误差
        qv = Vector3(self.x, self.y, self.z)
        t = qv.cross(v) * 2.0
        return v + t * self.w + qv.cross(t)

    def to_euler_rpy(self) -> tuple[float, float, float]:
        """四元数 → `(roll, pitch, yaw)`。**仅供 UI / Debug**（契约 §5.3）。"""
        # 归一化以防累积误差导致 asin 越界
        q = self.normalized("Quaternion.to_euler_rpy")
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x**2 + q.y**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        # 数值上可能略超 1，夹紧避免 asin 抛 ValueError（万向锁附近的正常现象）
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def approx_eq(self, other: "Quaternion", tol: float = TOL_EXTERNAL) -> bool:
        """比较旋转等价性：`q` 与 `-q` 表示**同一个旋转**，故比较绝对值。

        这条很重要：`from_axis_angle` 对同一个旋转可能给出两种符号
        （取决于轴的方向约定），若按分量比较会把等价旋转判为不等。
        """
        a = self.to_list()
        b = other.to_list()
        same = all(abs(i - j) <= tol for i, j in zip(a, b))
        negated = all(abs(i + j) <= tol for i, j in zip(a, b))
        return same or negated

    def __repr__(self) -> str:
        return f"[{self.x:g}, {self.y:g}, {self.z:g}, {self.w:g}]"


@dataclass(frozen=True)
class Transform:
    """位姿：position（m）+ orientation（四元数 `[x,y,z,w]`）。"""

    position: Vector3
    orientation: Quaternion

    def __post_init__(self) -> None:
        if not isinstance(self.position, Vector3):
            object.__setattr__(self, "position", Vector3.of(self.position, "Transform.position"))
        if not isinstance(self.orientation, Quaternion):
            object.__setattr__(
                self, "orientation", Quaternion.of(self.orientation, "Transform.orientation")
            )

    @classmethod
    def identity(cls) -> "Transform":
        return cls(Vector3.zero(), Quaternion.identity())

    @classmethod
    def from_parts(
        cls,
        position: Sequence[float] | Vector3,
        orientation: Sequence[float] | Quaternion,
    ) -> "Transform":
        return cls(Vector3.of(position, "Transform.position"), Quaternion.of(orientation, "Transform.orientation"))

    def to_dict(self) -> dict[str, list[float]]:
        return {
            "position": self.position.to_list(),
            "orientation": self.orientation.to_list(),
        }

    def compose(self, other: "Transform") -> "Transform":
        """`self ∘ other`：先在 other 的坐标系里定位，再经 self 变换到 self 的父系。

        这是 FK 链式相乘的基本操作：`T_world_child = T_world_parent ∘ T_parent_child`
        """
        return Transform(
            self.position + self.orientation.rotate(other.position),
            self.orientation * other.orientation,
        )

    def inverse(self) -> "Transform":
        inv_q = self.orientation.conjugate()
        return Transform(inv_q.rotate(self.position * -1.0), inv_q)

    def __repr__(self) -> str:
        return f"Transform(pos={self.position}, quat={self.orientation})"


#: JSON 类型别名（供序列化层标注用）
JsonValue = Any
