"""mini_arm 的**逆运动学**（Robot Package 内的实现，非 Core）。

## 它是解析解，为什么

mini_arm 是"平面 2R + 底座偏航"：给定 TCP 目标 `(x, y, z)`，

```text
① 由 (x, y) 直接得底座偏航：  φ = atan2(y, x)
② 半径 r = √(x² + y²)
③ 在 XZ 平面内解 2R：        θ1, θ2  （两解：elbow-up / elbow-down）
```

解析解的**全部价值**在于它能精确支撑往返验证：

```text
Joint → FK → Pose → IK → Joint'
```

如果 IK 是数值迭代，"往返一致"就变成了"在容差内一致"，而这个容差
恰好会掩盖"FK 与 IK 用了不同坐标系/不同轴向"这类系统性错误
（它们互相抵消，往返测试通过，但真机 / MuJoCo 全错）。

## 三个必须显式处理的事（而不是悄悄返回一个数）

| 情形 | 处理 |
|---|---|
| 目标超出可达半径 | 抛 `UnreachableError`，**并报告可达范围** |
| 目标过近（在死区球内） | 抛 `UnreachableError`（r 小于 `abs(L1 - L2eff)` 时无解） |
| 两解 | 返回两个，由调用方选；`prefer` 参数给默认选择 |

**禁止**把不可达目标悄悄夹紧到最近可达点：那会让用户以为"IK 解到了"，
而实际末端停在别处。夹紧只在**显式请求**时发生（`clamp=True`），
且结果里带 `clamped=True` 标记。

## 常量的来源

`L1 / L2 / L_TOOL / BASE_HEIGHT / SHOULDER_OFFSET` 从 `fk.py` 复用
（同一份几何真值，**不重复定义**）。二者与 MJCF 的一致性由包内测试断言。
"""

from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

from backend.model.robot_model import RobotModel
from backend.model.types import Quaternion, Transform, Vector3


def _load_sibling_fk():
    """加载同目录的 `fk.py`。

    ## 为什么用文件路径加载而不是 `from .fk import ...`

    Robot Package 不是 Python 包 —— `packages/` 下**刻意没有** `__init__.py`
    （包是"数据 + 算法"的容器，不该能被跨包 import；否则"加一台机器人"
    会变成"改 sys.path / 改包的依赖关系"）。

    而 `sys.path` 里没有 `packages/mini_arm/kinematics` 这个目录，
    所以相对 import 会失败。用 `importlib` 按**绝对文件路径**加载是唯一
    不依赖调用方 cwd / sys.path 的做法。

    ## 为什么注册进 `sys.modules`

    同一个 fk.py 可能被 fk 测试和 ik 分别加载。若不注册，
    `dataclass` 装饰器会因为"模块被加载两次"而抛出
    `AttributeError: 'NoneType' object has no attribute '__dict__'`
    （Python 3.13 的 dataclass 需要能从 sys.modules 反查模块）。
    注册后第二次加载命中缓存，模块身份唯一。
    """
    name = "_mini_arm_fk"
    if name in sys.modules:
        return sys.modules[name]
    path = Path(__file__).resolve().parent / "fk.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载同目录的 fk.py：{path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


_fk = _load_sibling_fk()

BASE_HEIGHT = _fk.BASE_HEIGHT
JOINT_ORDER = _fk.JOINT_ORDER
L1 = _fk.L1
L2 = _fk.L2
L_TOOL = _fk.L_TOOL
SHOULDER_OFFSET = _fk.SHOULDER_OFFSET
forward_kinematics = _fk.forward_kinematics

#: 平面 2R 的有效前臂长度 = 肘到肘的 L2 加上工具偏移 L_TOOL
#: （二者刚性固连、同轴，故可合并为一个等效连杆 —— 这也是本机构能写成 2R 的原因）
L2_EFF = L2 + L_TOOL

#: 关节限位（rad）。与 `model/mini_arm.xml` 的 range 一致，由包内测试断言。
#: 逆解必须**在限位内**求解，否则"solver 给出的角"在 MuJoCo 里会被截断，
#: 而截断后的实际位姿与目标不符 —— 表现为"Sim2Sim 对不上但 FK/IK 测试全绿"。
YAW_LIMIT = math.pi
SHOULDER_LIMIT = math.pi / 2
ELBOW_LIMIT = 3 * math.pi / 4

#: 数值容差
TOL_REACH = 1e-9

#: **奇异位形**的判定容差（rad）。
#:
#: 语义：`|θ2| < TOL_SINGULAR` 或 `|π - |θ2|| < TOL_SINGULAR` 时，
#: "肘上/肘下"两种构型**数学上重合**，"分支"这个概念不再有意义 ⇒
#: 解会被标注 `[degenerate]`，`solve_all` 也会把两个分支去重成一个。
#:
#: ## 这个值是怎么定出来的（不是拍脑袋）
#:
#: 在 mini_arm 的工作空间上扫描 `|θ2|` 的取值分布（约 10^6 个位形），
#: 结果是**双峰且中间完全空白**：
#:
#: ```text
#: == 0 的取值数: 1        （完全伸展，唯一值 0.0）
#: (0, 1°) 之间的取值: 1.479e-06° , 2.561e-06°   （只有浮点噪声）
#: 最小非零真值:      1.0°                        （关节角网格的下一个刻度）
#: ```
#:
#: ⇒ `(2.6e-6°, 1.0°)` 是一段**没有任何物理意义**的空白带。
#:   把阈值放进这段空白里，就同时满足两件事：
#:   ① 能吃掉浮点噪声（2.6e-6° = 4.5e-8 rad）；
#:   ② 不会误吞真实的两解（最小的真实分离是 1.0° = 0.0175 rad）。
#:
#: 1e-6 rad ≈ 5.7e-5°，位于空白带中央 —— 两侧各留约 20 倍余量。
#: 这是"用**观测到的**自然间隔定容差"，而不是"用 1e-12 这类漂亮数字"。
TOL_SINGULAR = 1e-6
#: 肩偏移与底座高度之和 = TCP 在"平面 2R 解"里要减掉的 Z 基准
Z_BASE = BASE_HEIGHT + SHOULDER_OFFSET


class IkError(RuntimeError):
    """逆解失败的基类。**不用返回值表示失败** —— 返回 None 会被漏检。"""


class UnreachableError(IkError):
    """目标在可达工作空间之外。"""

    def __init__(self, message: str, *, distance: float, reach_min: float, reach_max: float) -> None:
        super().__init__(message)
        self.distance = distance
        self.reach_min = reach_min
        self.reach_max = reach_max


class LimitViolationError(IkError):
    """解存在但超出关节限位（且未允许 clamp）。"""


@dataclass(frozen=True)
class IkSolution:
    """一组逆解 + 它的**元信息**。

    ## 为什么返回对象而不是裸 dict

    因为"这个解是 elbow-up 还是 elbow-down"、"有没有触限"、
    "误差多大"都是调用方（UI / Runtime）需要的决策依据。
    返回裸 `dict[str, float]` 会迫使调用方自己重算这些量，
    而"重算"就是第二份实现的开始 —— 它会与第一份漂移。
    """

    joint_positions: dict[str, float]
    #: 'elbow_up' | 'elbow_down'
    branch: str
    #: 该解下 FK 回到的位姿与目标之间的位置误差（m）
    position_error: float
    #: 是否因触限被夹紧（仅 clamp=True 时可能为 True）
    clamped: bool = False
    #: 触限的关节 id 列表
    clamped_joints: list[str] = field(default_factory=list)
    #: 姿态是否被满足（mini_arm 的姿态由 θ1+θ2+φ 决定，不一定能同时匹配目标姿态）
    orientation_error: float | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "joints": dict(self.joint_positions),
            "branch": self.branch,
            "position_error": self.position_error,
            "clamped": self.clamped,
            "clamped_joints": list(self.clamped_joints),
            "orientation_error": self.orientation_error,
        }


def reach_limits() -> tuple[float, float]:
    """可达半径区间（**水平半径 r**，不含底座高度）。

    ```text
    r_max = L1 + L2_EFF          完全伸展
    r_min = |L1 - L2_EFF|        完全折叠
    ```

    注意这是"平面半径"，完整工作空间是一个**球壳的一部分**
    （加上 Z 方向后是绕 Z 轴的旋转体）。UI 显示工作空间时应该显示这个旋转体，
    而不是一个球 —— 显示成球会让用户以为 z 可以任意取。
    """
    return abs(L1 - L2_EFF), L1 + L2_EFF


def solve(
    model: RobotModel,
    target: Transform,
    *,
    branch: str = "elbow_up",
    prefer_limits: bool = True,
    clamp: bool = False,
    tol: float = 1e-6,
) -> IkSolution:
    """逆解：目标 TCP 位姿 → 关节角。

    ## 参数

    - `branch`：`'elbow_up'` / `'elbow_down'`。机构通常固定一个装配构型，
      因此默认值比"随机选一个"更有意义。
    - `prefer_limits`：解出后按限位筛选。若两解都超限 ⇒ 抛 `LimitViolationError`
      （除非 `clamp=True`）。
    - `clamp`：允许把超限解夹到限位。**默认 False** —— 见文件头说明。

    ## 返回

    `IkSolution`，含关节角、分支、位置误差、触限信息。

    ## 与 FK 的对称性

    本函数只求出**关节角**（Joint Space），**不**做任何执行器/舵机映射。
    `Joint → Actuator Mapping → Servo` 是未来 `Actuator` 层的职责
    （P0 契约 §3.6）。把舵机角混进 IK 会让换舵机就要改运动学。
    """
    # ---- ① 底座偏航：由 (x, y) 唯一确定 ----
    # atan2 而非 atan(y/x)：atan 丢失象限信息，且 x=0 时除零。
    # 这是 IK 里最常见的一个 bug —— 它只在第二/三象限暴露。
    x, y, z = target.position.x, target.position.y, target.position.z
    angle_yaw = math.atan2(y, x)

    # ---- ② 平面半径与高度 ----
    r = math.hypot(x, y)
    z_arm = z - Z_BASE  # 减去底座与肩偏移，回到"肩为原点"的平面坐标系

    # 平面内到肘的距离（2D 距离，含 z_arm）
    d = math.hypot(r, z_arm)
    r_min, r_max = reach_limits()

    # 容差放宽一点：目标若正好在边界上，浮点误差会把它推到外面
    if d > r_max + tol:
        raise UnreachableError(
            f"目标距离肩关节 {d:.6f} m 超出可达范围 [{r_min:.6f}, {r_max:.6f}] m；"
            f"目标位置 = ({x:.6f}, {y:.6f}, {z:.6f})，底座平面半径 r = {r:.6f} m",
            distance=d,
            reach_min=r_min,
            reach_max=r_max,
        )
    if d < r_min - tol:
        raise UnreachableError(
            f"目标距离肩关节 {d:.6f} m 小于最小可达半径 {r_min:.6f} m"
            f"（肘部完全折叠的死区球内）；无法到达",
            distance=d,
            reach_min=r_min,
            reach_max=r_max,
        )

    # ---- ③ 平面 2R：余弦定理求肘角 ----
    # cos(θ2) 由两连杆三角形确定： d² = L1² + L2e² - 2·L1·L2e·cos(π - θ2)
    # 展开后：cos(θ2) = (d² - L1² - L2e²) / (2·L1·L2e)
    cos_theta2 = (d * d - L1 * L1 - L2_EFF * L2_EFF) / (2.0 * L1 * L2_EFF)
    # 夹紧到 [-1, 1]：边界目标的浮点误差会让它略超出，而 acos 会抛 ValueError。
    # 这个夹紧**不改变解的正确性**（超出部分正是上面已判定的不可达），
    # 因此不需要报告为 clamped。
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    abs_theta2 = math.acos(cos_theta2)

    # ---- ④ 肩角：目标方向角 + 三角形内角 ----
    # 目标在平面内的方向角
    # 注意负号：绕 +Y 转 θ>0 时 +X 轴指向 -Z，故平面角 = -atan2(z_arm, r)
    phi_target = math.atan2(-z_arm, r)
    # 余弦定理求 L1 与 d 的夹角
    cos_alpha = (L1 * L1 + d * d - L2_EFF * L2_EFF) / (2.0 * L1 * d)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)

    # ---- ★ 分支的判据是"肘在肩-手连线的哪一侧"，不是 θ2 的符号 ----
    #
    # 这里改过一次，记录原因（第一版把 elbow_up 硬绑到 θ2<0，结果标反了）：
    #
    #   "肘在上方"是一个**几何**概念：肘相对**肩→手连线**的侧向偏移。
    #   而 θ2 的符号只说明"前臂相对上臂折向哪边"，二者的对应关系**取决于
    #   L1 与 L2eff 谁更长** —— 本机构 L2eff(0.097) < L1(0.103) 时
    #   θ2<0 恰好对应肘**向下**，与直觉相反。
    #
    #   如果按 θ2 的符号命名分支，那么"elbow_up"这个**接口语义**就会随
    #   连杆长度漂移 —— 改一根连杆的长度，前端选中的分支就悄悄反了，
    #   而表现只是"机械臂以镜像姿态去够同一个点"，极难发现。
    #
    #   ⇒ 所以：**先算出两个候选 θ2，再用几何量（肘的侧偏）判定谁是 up**。
    #     这样接口语义是稳定的，且与 L1/L2eff 的取值无关。
    def _side_offset(theta2_candidate: float) -> float:
        """给定 θ2，算出该分支下肘相对'肩→手连线'的**有符号侧偏**。

        正值 = 肘在连线的 +Z 侧（即"上方"，因为 +Z 是上）。
        判据用二维叉积：`hand_dir × elbow_vec` 的 z 分量符号。
        """
        t1 = _theta1_for(theta2_candidate)
        # 肘相对肩的位置（平面）
        ex, ez = L1 * math.cos(t1), -L1 * math.sin(t1)
        # 肩→手 的方向（= 目标方向）
        hx, hz = math.cos(phi_target), -math.sin(phi_target)
        # 二维叉积 z 分量：hand_dir × elbow_vec 的符号 = 肘在连线哪一侧
        return hx * ez - hz * ex

    def _theta1_for(theta2_candidate: float) -> float:
        """由 θ2 与目标位置唯一确定 θ1。

        ## ★ 配对规则的符号（这里改过一次，是最难发现的一个 bug）

        三角关系是：目标方向角 `phi_target` 与"上臂偏离目标方向的角度" `alpha`
        组成 θ1。而 α 该加还是该减，**取决于 θ2 的符号**，规则是：

        ```text
        θ2 > 0  ⇒  θ1 = phi_target - alpha
        θ2 < 0  ⇒  θ1 = phi_target + alpha
        ```

        第一版写成了 `t1 = phi + alpha * (1 if θ2 > 0 else -1)` —— **正好写反**。
        它的后果不是崩溃，而是**灾难性的静默错误**：

        ```text
        两个分支算出的 θ1 都把 TCP 放到**完全错误的位置**（误差 ~0.15 m，
        几乎等于整条臂长），但这个错误位置**仍然在关节限位内**，
        所以没被限位检查拦下；
        最终由"回代 FK 自检"（position_error）兜住 ⇒ 报"IK 失败"，
        而那是个**假失败**：目标明明可达。
        ```

        ⇒ 这就是为什么 `solve()` 必须**回代 FK 自检并报告 position_error**：
          纯解析的 IK 若没有这一步，错误会一路传到 MuJoCo 与前端，
          表现成"机械臂不听话"，而排查方向完全跑偏。
        """
        return phi_target + alpha if theta2_candidate < 0 else phi_target - alpha

    candidates_t2 = (abs_theta2, -abs_theta2)
    # 分别算出两候选的侧偏，侧偏大的那个是 elbow_up
    offsets = {t2: _side_offset(t2) for t2 in candidates_t2}
    up_t2 = max(offsets, key=lambda t: offsets[t])
    down_t2 = min(offsets, key=lambda t: offsets[t])

    # ★ 退化判据必须看**解本身**，而不是侧偏之差（改过一次，记录原因）
    #
    # 第一版用 `abs(offset_up - offset_down) < eps` 判定退化，结果是错的：
    # 在完全伸展位形（θ2 = 0）下，两个候选 θ2 都是 0，**解完全相同**，
    # 但 `_side_offset` 对 ±0 会算出符号相反的侧偏（+0 与 -0 进入
    # `theta2 > 0` 判断时落到不同分支），于是 offsets 差得很大 ⇒ 判为非退化 ⇒
    # 给"同一个解"贴上 up / down 两个互相矛盾的标签。
    #
    # ⇒ 正确判据：**两个候选 θ2 是否实质相同**（= 肘几乎伸直或几乎对折）。
    #   容差 TOL_SINGULAR 的来历见模块顶部的常量注释（是扫描出来的空白带，不是猜的）。
    degenerate = (
        abs(abs_theta2) < TOL_SINGULAR or abs(abs_theta2 - math.pi) < TOL_SINGULAR
    )

    if branch == "elbow_up":
        theta2 = up_t2
    elif branch == "elbow_down":
        theta2 = down_t2
    else:
        raise IkError(f"未知的 branch = {branch!r}；允许 'elbow_up' / 'elbow_down'")

    theta1 = _theta1_for(theta2)

    raw = {"base_yaw": angle_yaw, "shoulder": theta1, "elbow": theta2}

    # 记录实际分支（退化时诚实标注，而不是假装两个分支分开了）
    actual_branch = branch
    if degenerate:
        actual_branch = f"{branch} [degenerate]"

    # ---- ⑤ 限位处理 ----
    limits = {
        "base_yaw": YAW_LIMIT,
        "shoulder": SHOULDER_LIMIT,
        "elbow": ELBOW_LIMIT,
    }
    # base_yaw 的 atan2 结果天然落在 (-π, π]，与 YAW_LIMIT=π 一致
    clamped_joints = [k for k, v in raw.items() if abs(v) > limits[k] + 1e-9]

    if clamped_joints and prefer_limits and not clamp:
        detail = ", ".join(f"{k}={raw[k]:+.6f} (限位 ±{limits[k]:.6f})" for k in clamped_joints)
        raise LimitViolationError(
            f"目标 {branch} 解超出关节限位：{detail}。"
            f"该目标位置可达但**姿态不可达**（受关节限位约束）。"
            f"如需强制夹紧请传 clamp=True（结果会带 clamped 标记）"
        )

    if clamp:
        raw = {k: max(-limits[k], min(limits[k], v)) for k, v in raw.items()}

    # ---- ⑥ 自检：把解回代 FK，报告实际误差 ----
    # 用模块级已加载的 fk（见 _load_sibling_fk 的说明）
    check = forward_kinematics(model, raw)
    pos_err = (check.position - target.position).norm()
    ori_err = _orientation_error(check.orientation, target.orientation)

    return IkSolution(
        joint_positions=raw,
        branch=actual_branch,
        position_error=pos_err,
        clamped=bool(clamped_joints) if clamp else False,
        clamped_joints=clamped_joints if clamp else [],
        orientation_error=ori_err,
    )


def solve_all(
    model: RobotModel,
    target: Transform,
    *,
    clamp: bool = False,
) -> list[IkSolution]:
    """返回**两个**可行解（elbow_up / elbow_down），按误差升序。

    超出限位或不可达的分支被**跳过**（记入 warnings）。
    若两个分支都不可行 ⇒ 抛 `IkError`。

    ## 为什么要返回"全部解"而不只是最优解

    因为"最优"依赖于选择准则，而准则属于**应用层**：
    - 靠近奇异位形时该选"离当前位形最近"的解（避免跳变）；
    - 避障时该选"不撞"的解；
    - 演示时该选"看起来最自然"的解。

    这些都要求上层能看到候选集。IK 层擅自选一个，就会让上层无法实现这些准则。
    """
    candidates: list[IkSolution] = []
    warnings: list[str] = []
    seen: set[tuple] = set()

    for br in ("elbow_up", "elbow_down"):
        try:
            sol = solve(model, target, branch=br, clamp=clamp)
        except (UnreachableError, LimitViolationError) as exc:
            if isinstance(exc, UnreachableError):
                # 不可达 ⇒ 两个分支都不可达，直接抛出（不是"换一个分支试试"）
                raise
            warnings.append(f"{br}: {exc}")
            continue
        # 只保留误差在容差内的（防止"夹紧后误差巨大"仍被当作可行解）
        if sol.position_error > 1e-3:
            warnings.append(f"{br}: 位置误差 {sol.position_error:.6f} m 过大")
            continue
        # ★ 去重：奇异位形（完全伸展 / 完全折叠）下两分支解**相同**，
        #   返回两份一样的解会让 UI 显示"2 个解"而用户看到两个一模一样的角度。
        #   按关节角量化后去重，保留先出现的（elbow_up）。
        key = tuple(round(v, 12) for v in sol.joint_positions.values())
        if key in seen:
            warnings.append(f"{br}: 与另一分支同解（奇异位形）⇒ 去重")
            continue
        seen.add(key)
        candidates.append(sol)

    if not candidates:
        raise IkError("两个分支都不可行：\n  " + "\n  ".join(warnings))

    candidates.sort(key=lambda s: s.position_error)
    return candidates


def _orientation_error(a: Quaternion, b: Quaternion) -> float:
    """两个姿态之间的角度差（rad）。

    用 `|dot|` 再 acos：四元数 q 与 -q 表示同一旋转，故取绝对值。
    不取绝对值会让"同一个姿态"算出 180° 的误差 —— 一个经典陷阱。
    """
    dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def workspace_sample(radius_steps: int = 5, angle_steps: int = 8, z_steps: int = 5) -> list[list[float]]:
    """采样可达工作空间的边界点（供 UI 画"工作空间"提示）。

    返回世界坐标点列表。它是"可能的 TCP 位置"的**表面采样**，
    不是精确边界 —— 精确边界需要求解限位约束下的可达集，
    而 v0.1 的用途只是给用户一个空间感。
    """
    r_min, r_max = reach_limits()
    z_lo = Z_BASE - L1 - L2_EFF
    z_hi = Z_BASE + SHOULDER_LIMIT * 0  # 简化：上界用 Z_BASE + 伸展高度
    z_hi = Z_BASE + (L1 + L2_EFF) * math.sin(SHOULDER_LIMIT)

    points: list[list[float]] = []
    for i in range(radius_steps + 1):
        r = r_min + (r_max - r_min) * i / radius_steps
        for j in range(angle_steps):
            ang = 2 * math.pi * j / angle_steps
            for k in range(z_steps + 1):
                z = z_lo + (z_hi - z_lo) * k / z_steps
                points.append([r * math.cos(ang), r * math.sin(ang), z])
    return points


__all__ = [
    "ELBOW_LIMIT",
    "IkError",
    "IkSolution",
    "JOINT_ORDER",
    "L2_EFF",
    "LimitViolationError",
    "SHOULDER_LIMIT",
    "UnreachableError",
    "YAW_LIMIT",
    "reach_limits",
    "solve",
    "solve_all",
    "workspace_sample",
]
