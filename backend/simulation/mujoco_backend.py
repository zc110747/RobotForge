"""MuJoCo 仿真后端（提示词 §50 / §62）。

```text
RobotCommand ──▶ MuJoCoBackend ──▶ MjData ──▶ Forward Kinematics ──▶ RobotState
```

## §50 的原文要求

> **MuJoCo 是 Physics Backend，不是 RobotModel 的替代品。**
> MuJoCo 不参与 RobotModel 的定义，只负责"给定关节目标，算出关节实际状态"。

本文件的实现忠实于此：`MjModel` 在 `start()` 里按需编译，
**编译完只保留 `nq/nv/nu` 与关节地址映射**，绝不把 `MjModel` 泄漏到
`RobotModel` / `RobotState` / `RobotRuntime` 任何一层。
`RobotModel` 始终是唯一规范表示（§6 / §69 规则 1）。

---

## ★★ 单一的四元数换算点（§40 / §35）

MuJoCo 的四元数是 `[w, x, y, z]`；RobotForge 全局统一 `[x, y, z, w]`。
**本文件是整个后端里唯一允许出现 `[w,x,y,z]` 的地方**，
并且只出现在两个函数里：

```text
mj_quat_to_xyzw(q_wxyz) -> [x, y, z, w]     引擎 → 平台
xyzw_to_mj_quat(q_xyzw) -> [w, x, y, z]     平台 → 引擎
```

为什么必须集中：这是一个**互相抵消**的错误。如果读取侧和写入侧
用了同一个错误的顺序，`X → 平台 → X` 的往返会**完美通过**，
而真实行为（WebSocket → Three.js → 用户看到的姿态）全错。
`tests/test_mujoco.py::TestQuaternionBoundary` 用一个"不可能碰巧通过"的
**非对称四元数**（四个分量互不相等且非特殊值）钉住这两个函数的方向，
另有一项源码扫描断言全仓库除了本文件没有第二处做这个重排。

## 关节顺序

`RobotModel.mobile_joint_ids()` 的顺序 == `MjModel.jnt_qposadr` 的升序
—— 这不是巧合，是契约（`robot_model.py::mobile_joint_ids` 的注释写明）。
`start()` 会**显式断言**这一点：如果某台机器人的 MJCF 声明顺序与
关节表顺序不一致，宁可启动失败，也不要"发 shoulder 动 elbow"。
这个错误在真机上表现为"机械臂乱动"，从现象几乎不可能反推回成因。

## 推进步长与子步

一次 `send_command` **不**推进物理：它只把目标写进 `d.ctrl`。
物理在 `get_state` 之前由 `_advance()` 推进 `max_substeps` 个子步。
理由与 `MockBackend` 完全相同（见 `runtime/backend.py`）：

* 若 `send_command` 顺手推 1 步并立刻读回，那状态就近似等于命令，
  §61/§62 要求的 "Command ≠ State" 退化掉；
* 真实伺服也需要时间到位，把"要多久"变成可观测的量是对的。

⚠️ **子步数是必要的，不是优化**：MJCF 的 `timestep = 0.002 s`，
而 `armature = 0.01` 下近端关节的等效频率约 77 rad/s。
乘起来 `ω·dt ≈ 0.155`，单步稳定；但若把 `timestep` 调到 0.02 来"省步骤"，
`ω·dt ≈ 1.55` 就逼近显式积分边界 `2` —— 于是仿真会 NaN。
所以本层**绝不改 `timestep`**，而是用循环推进若干子步。
"""

from __future__ import annotations

import importlib.util
import time
from typing import Any, Mapping

from ..kinematics.fk import forward_kinematics
from ..model.robot_model import RobotModel
from ..model.types import Quaternion, Transform, Vector3
from ..runtime.backend import BackendError, RobotBackend
from ..runtime.command import RobotCommand
from ..runtime.state import RobotState
from .simulation_backend import (
    DEFAULT_STEP_SECONDS,
    JointOrderError,
    assemble_state,
    clamp_targets_to_limits,
    ordered_joint_ids,
    positions_to_vector,
)

#: 一次 `get_state()` 推到位的子步数上限。
#: 600 × 0.002 s = 1.2 s 仿真时间，是"足够收敛"与"不卡住 WS 循环"的折中。
DEFAULT_MAX_SUBSTEPS = 600


def _require_mujoco():
    """拿到 `mujoco` 模块，缺失时给**可操作**的错误。

    `backend/` 的 Core 层刻意不依赖 mujoco（`loaders/mjcf_loader.py` 是
    唯一的例外，因为 v0.1 只允许原生 MJCF）。Backend 属于执行端，
    依赖引擎是它的本职；但**没装**时要能一眼看出怎么装。
    """
    if importlib.util.find_spec("mujoco") is None:
        raise BackendError(
            "MuJoCoBackend 需要 mujoco，但当前环境里没有这个模块。\n"
            "  安装：.venv/Scripts/python.exe -m pip install mujoco"
        )
    import mujoco  # noqa: PLC0415 - 局部 import，缺失时才会触发上面的错误

    return mujoco


# ---------------------------------------------------------------------------
# ★ 唯一的四元数边界（§40 / §35）
# ---------------------------------------------------------------------------


def mj_quat_to_xyzw(q_wxyz: Any) -> list[float]:
    """MuJoCo `[w,x,y,z]` → RobotForge `[x,y,z,w]`。

    ★ 这是本文件对外的**两个**换算函数之一（另一个是 `xyzw_to_mj_quat`）。
    其余任何地方都不许再写 `q[1], q[2], q[3], q[0]` 这种重排
    （有源码扫描盯着，见 `tools/accept_phase5.py`）。

    ⚠️ 不要顺手加"归一化"。`xmat` 由 MuJoCo 的旋转矩阵转换而来，
    它已经是单位四元数；在这里归一化会**掩盖**上游的数值问题。
    需要归一化的地方是 `Quaternion` 自己（它已经做了校验）。
    """
    w, x, y, z = (float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]))
    return [x, y, z, w]


def xyzw_to_mj_quat(q_xyzw: Any) -> list[float]:
    """RobotForge `[x,y,z,w]` → MuJoCo `[w,x,y,z]`。

    v0.1 的 `MuJoCoBackend` 只驱动**铰链关节**（qpos 是标量），
    所以这个方向当前只在测试里用到。保留它的理由是：一旦将来支持
    浮动基座（free joint，qpos 里带四元数），**这一侧**就是必须的，
    而那时临时补一个"只写了一半的换算"正是当年出错的路径
    （读取侧改了、写入侧忘了）。
    """
    x, y, z, w = (float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2]), float(q_xyzw[3]))
    return [w, x, y, z]


# ---------------------------------------------------------------------------
# Backend
# ---------------------------------------------------------------------------


class MuJoCoBackend(RobotBackend):
    """把 `RobotCommand` 灌进 MuJoCo，把 `MjData` 读成 `RobotState`。

    ## 与 `MockBackend` 的关系：**接口相同，语义不同**

    | | MockBackend | MuJoCoBackend |
    |---|---|---|
    | 动力学 | 无（按 max_step 线性靠近） | 真物理（重力/惯量/阻尼/接触） |
    | 到位精度 | 精确（多步后位误差 = 0） | 收敛但有稳态误差 |
    | 重力 | 无 | 有（下垂是真实现象，不是 bug） |
    | 关节耦合 | 无 | 有（近端运动会带动远端） |

    最后一行是关键：MuJoCo 里 `shoulder` 一动，`elbow` 的**实际**角度会被
    惯性带动而变化 —— 这是 `MockBackend` 结构上**不可能**产生的现象。
    §62 的验收因此可以问一个 MockBackend 时代问不出的问题：
    "命令只发了 shoulder，elbow 真的没动吗？"

    ## `is_simulation`

    保持 `True`（继承自 `RobotBackend` 的默认值）。它不是"这是假数据"的
    标记，而是"这是仿真执行端"的标记 —— `simulation_state` 帧会把它
    转成 `type: "simulation"` 与 Backend 类名一起报给前端。
    """

    is_simulation = True

    def __init__(
        self,
        model: RobotModel,
        source: Any = None,
        step_seconds: float = DEFAULT_STEP_SECONDS,
        max_substeps: int = DEFAULT_MAX_SUBSTEPS,
        settle: bool = True,
    ) -> None:
        """
        `source`：MJCF 的来源（`Path` / XML 字符串）。默认从
        `RobotModel.metadata` 里读回加载时的路径 —— 见 `_resolve_source`。
        测试也可以直接传一个 XML 字符串来验证"换成另一台机器人也能跑"。

        `settle`：`start()` 时是否先把机器人推进到静态平衡。
        默认 True。原因是"零位形不是平衡位形"：mini_arm 的
        上臂/前臂重心在 X 方向有偏移，重力会让它们下垂几毫度。
        不做 settle 的话，第一帧状态就有一次可见的"跳一下"，
        而这与"仿真器在动"是两件不同的事，混在一起会污染诊断。
        """
        self._model = model
        self._source = source
        self._step_seconds = float(step_seconds)
        self._max_substeps = int(max_substeps)
        self._settle = bool(settle)

        self._joint_ids: list[str] = ordered_joint_ids(model)

        # 引擎对象（`start()` 时才建立）
        self._mj: Any = None
        self._data: Any = None
        self._qpos_addr: list[int] = []
        self._dof_addr: list[int] = []
        self._tcp_site_id: int = -1

        self._started = False
        self._status = "idle"
        self._t0 = time.monotonic()
        self._sim_time = 0.0

        #: 命令目标（夹紧后）。与 MockBackend 的 `clamped` 同语义，
        #: 供测试断言"限幅发生在目标计算阶段而不是引擎里"。
        self._targets: dict[str, float] = {jid: 0.0 for jid in self._joint_ids}
        #: 被限幅的目标（命令 ≠ 目标）。
        self.clamped: dict[str, float] = {}
        #: 累计收到的命令数（与 MockBackend 同名，便于同一批判据复用）。
        self.command_count = 0
        #: 累计推进的子步数（诊断用：证明状态确实是"推"出来的）。
        self.substeps = 0

    # ------------------------------------------------------------------
    # 源解析
    # ------------------------------------------------------------------

    def _resolve_source(self) -> Any:
        """找到这个 `RobotModel` 对应的 MJCF。

        优先用显式传入的 `source`。否则从 `metadata.description` 里
        解析加载器记下的路径（`Loaded from native MJCF (<path>)`）。

        ⚠️ 从 description 里反解路径确实不优雅，但它有一个重要的性质：
        **它不可能解析出"另一台机器人"**，因为那段文字是被测量的
        `RobotModel` 自己带来的。相比之下，在这里 `get_package(model.id)`
        会把 Backend 变成"需要注册表的组件"，而 §69 规则 10 要求
        Backend 只依赖 `RobotModel`（Backend 不认识 packages 目录）。

        真实路径在 `report.source` 里，但 `RobotModel` 不持有 report
        （report 是加载过程的产物，不是模型的一部分）。所以走 description。
        若将来给 `RobotMetadata` 加一个 `source` 字段，
        这个函数就是需要改的**唯一**位置。
        """
        if self._source is not None:
            return self._source
        desc = self._model.metadata.description or ""
        # 形状：Loaded from native MJCF (<path>)
        marker = "Loaded from native MJCF ("
        idx = desc.find(marker)
        if idx >= 0 and desc.endswith(")"):
            candidate = desc[idx + len(marker): -1]
            if candidate and candidate != "<memory>":
                return candidate
        raise BackendError(
            f"无法确定 RobotModel {self._model.metadata.id!r} 的 MJCF 来源。"
            f"请显式传 source=<path 或 xml 字符串>。"
            f"（metadata.description = {desc!r}）"
        )

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """编译 MjModel、建 MjData、对账关节映射、可选 settle。幂等。

        ## 对账是**启动时**的一次性检查，不是运行时的分支

        `qposadr` 与 `mobile_joint_ids()` 的顺序不一致时，本方法直接拒绝启动。
        把它做成运行时 `if` 的代价是每条命令都要付一次判断，
        而且掩盖了"这个模型不该被用"这个事实。
        """
        if self._started:
            return

        mujoco = _require_mujoco()

        source = self._resolve_source()
        try:
            # ★ 必须区分**路径**与**XML 文本**：`from_xml_path("<?xml ...>")`
            # 会把整段 XML 当成文件名，报出一个形如
            # `Error opening file '<!--\n  ====...` 的错误 —— 前几行 XML
            # 被当成路径回显，看起来像"文件损坏"而不是"调用错了 API"。
            # 判据与 `MJCFLoader._resolve_source` 一致：以 `<` 开头就是文本。
            if isinstance(source, str) and source.lstrip().startswith("<"):
                mj = mujoco.MjModel.from_xml_string(source)
            else:
                mj = mujoco.MjModel.from_xml_path(str(source))
        except Exception as exc:  # MuJoCo 抛 ValueError / RuntimeError
            raise BackendError(
                f"MuJoCo 无法编译 {self._model.metadata.id!r} 的 MJCF：{exc}"
            ) from exc

        if int(mj.nq) != len(self._joint_ids) or int(mj.nv) != len(self._joint_ids):
            raise BackendError(
                f"模型 {self._model.metadata.id!r} 有 {len(self._joint_ids)} 个可动关节，"
                f"但 MJCF 的 nq={int(mj.nq)} / nv={int(mj.nv)} 与它不符。"
                f"（v0.1 只支持全铰链/移动关节的 Tree 型机构；浮动基座会让 nq 多出四元数分量）"
            )

        qpos_addr: list[int] = []
        dof_addr: list[int] = []
        for jid in self._joint_ids:
            mj_jnt_id = mujoco.mj_name2id(
                mj, mujoco.mjtObj.mjOBJ_JOINT, jid
            )
            if mj_jnt_id < 0:
                raise BackendError(
                    f"RobotModel 的关节 {jid!r} 在 MJCF 里找不到同名 joint。"
                    f"（Loader 会把非法名字改成安全 id，若发生了改名，"
                    f"这里就会报出来而不是静默配错关节）"
                )
            qpos_addr.append(int(mj.jnt_qposadr[mj_jnt_id]))
            dof_addr.append(int(mj.jnt_dofadr[mj_jnt_id]))

        # ★ 顺序断言（见模块 docstring）
        if qpos_addr != sorted(qpos_addr):
            order = sorted(range(len(qpos_addr)), key=lambda i: qpos_addr[i])
            raise BackendError(
                f"MJCF 的关节声明顺序与 RobotModel.mobile_joint_ids() 不一致："
                f"{self._joint_ids} 对应的 qposadr = {qpos_addr}。"
                f"（按 qposadr 排序后应为 "
                f"{[self._joint_ids[i] for i in order]}）"
                f"—— 继续运行会导致『发 A 命令却动了 B 关节』。"
            )

        self._mj = mj
        self._data = mujoco.MjData(mj)
        self._qpos_addr = qpos_addr
        self._dof_addr = dof_addr
        self._tcp_site_id = self._resolve_tcp_site(mujoco, mj)

        self._started = True
        self._status = "idle"
        self._t0 = time.monotonic()
        self._sim_time = 0.0
        self.substeps = 0

        # 初始目标 = 当前关节角（不是 0）：
        # `MjData` 默认 qpos 全 0，而零位形不是平衡位形。
        for i, jid in enumerate(self._joint_ids):
            self._targets[jid] = float(self._data.qpos[self._qpos_addr[i]])

        if self._settle:
            self._settle_to_equilibrium()

    def _settle_to_equilibrium(self) -> None:
        """推进到 qvel 足够小为止（最多 `max_substeps` 步）。

        ⚠️ 这里**不**把 `qpos` 改写成 0。让重力把它推到平衡位置，
        比"强行写上零位姿"更诚实 —— 后者会造出一个**非平衡的初始状态**，
        而这个状态在下一帧就会自己跳开（表现为"机器人抖一下"）。
        """
        mujoco = _require_mujoco()
        ctrl = self._data.ctrl
        for i, jid in enumerate(self._joint_ids):
            ctrl[i] = self._targets[jid]
        for _ in range(self._max_substeps):
            mujoco.mj_step(self._mj, self._data)
            self.substeps += 1
            self._sim_time += float(self._mj.opt.timestep)
            if max(abs(float(v)) for v in self._data.qvel) < 1e-6:
                break

    async def stop(self) -> None:
        """释放 `MjModel` / `MjData`。幂等。"""
        self._mj = None
        self._data = None
        self._qpos_addr = []
        self._dof_addr = []
        self._tcp_site_id = -1
        self._started = False
        self._status = "idle"

    async def reset(self) -> None:
        """关节归零 + 仿真时间归零 + 清空限幅记录。**不**清零 `command_count`。

        与 `MockBackend.reset` 保持同一语义（理由见 `runtime/backend.py`：
        测试要靠 `command_count` 证明命令真的到达了 Backend，
        而 reset 恰好是测试里最常调用的操作）。

        ⚠️ 归零后**不**做 settle：`reset()` 的语义是"回到零位"，
        让重力再推一次会让"复位"变成"复位 + 一段时间"，
        测试里就分不清"复位没生效"与"刚复位还在沉"。
        """
        if not self._started:
            return
        mujoco = _require_mujoco()
        self._data.qpos[:] = 0.0
        self._data.qvel[:] = 0.0
        self._data.ctrl[:] = 0.0
        mujoco.mj_forward(self._mj, self._data)

        for jid in self._joint_ids:
            self._targets[jid] = 0.0
        self.clamped.clear()
        self._status = "reset"
        self._t0 = time.monotonic()
        self._sim_time = 0.0

    # ------------------------------------------------------------------
    # 命令
    # ------------------------------------------------------------------

    async def send_command(self, command: RobotCommand) -> None:
        """写目标到 `d.ctrl`。**不推进物理**（见模块 docstring）。

        严格按 `RobotBackend` 契约：命令是 Desired，是否到达由状态回答。
        """
        if not self._started:
            raise BackendError(
                "MuJoCoBackend 未启动就收到命令。Runtime 必须在 send_command 前 "
                "await start()。"
            )
        command.validate_against(self._joint_ids)
        self.command_count += 1

        targets, clamped = clamp_targets_to_limits(self._model, command.joint_targets)
        self.clamped.update(clamped)

        for jid, target in targets.items():
            self._targets[jid] = target

        # 全部关节一起下发：`d.ctrl` 是**目标向量**，
        # 只写被命令的关节会让未提及的关节回到上一次的 ctrl，
        # 而"未提及"的语义是"保持"，不是"复位"。
        for i, jid in enumerate(self._joint_ids):
            self._data.ctrl[i] = self._targets[jid]

        self._status = "running"

    # ------------------------------------------------------------------
    # 观测
    # ------------------------------------------------------------------

    async def get_state(self) -> RobotState:
        """推进物理 → 读关节 → 用 **Core FK** 算末端位姿。

        ### 为什么末端位姿用 Core FK 而不是 MuJoCo 的 `xpos`/`xmat`

        这是本后端最值得解释的一个决定。

        用 MuJoCo 直接读 `site_xpos` 更"物理正确"（它含真实的柔性/接触形变），
        但它会让 §62 的验收"FK 与 Simulation 状态一致"变成**同义反复**：
        MuJoCo 算的、FK 也算的，本来就是同一套几何。
        更糟的是，"MuJoCo 四元数顺序配错"这类错误会被掩盖 ——
        因为位置对得上，只有姿态错，而姿态恰恰是最难肉眼发现的。

        用 Core FK 则把两者变成**两个独立来源**：
        MuJoCo 负责 qpos，Core FK 负责几何。二者一致 ⇒
        既证明了 MJCF 的 `body pos` 与 RobotModel 的 `joint.origin`
        逐字段对齐，也证明了四元数边界换算正确。

        代价：`end_effector_pose` 不含 MuJoCo 的形变。
        v0.1 的连杆是刚体，没有形变可言，所以这个代价为零。

        ★ 这不是"为了 Three.js 改坐标"（§41）—— FK 的输出仍是
        RobotForge 约定，转换点仍然只有 `coordinateAdapter.ts`。
        """
        if not self._started:
            raise BackendError("MuJoCoBackend 未启动就读取状态。")

        self._advance()

        positions = {
            jid: float(self._data.qpos[self._qpos_addr[i]])
            for i, jid in enumerate(self._joint_ids)
        }
        velocities = {
            jid: float(self._data.qvel[self._dof_addr[i]])
            for i, jid in enumerate(self._joint_ids)
        }

        pose = self._end_effector_pose(positions)

        return assemble_state(
            self._model,
            positions=positions,
            velocities=velocities,
            end_effector_pose=pose,
            status=self._status,
            timestamp=self._sim_time,
        )

    def _advance(self) -> None:
        """把物理推进 `step_seconds` 秒（`max_substeps` 个子步封顶）。

        用 while 累加而不是 `int(step / timestep)`：`timestep` 是模型给的，
        两者不整除时取整会**静默地**改变实际步长，
        于是"改一下 MJCF 的 timestep"会让验收里的收敛值悄悄变化。
        """
        mujoco = _require_mujoco()
        dt = float(self._mj.opt.timestep)
        target_time = self._sim_time + self._step_seconds
        n = 0
        while self._sim_time < target_time and n < self._max_substeps:
            mujoco.mj_step(self._mj, self._data)
            self._sim_time += dt
            n += 1
        self.substeps += n
        # ★★ 必须 `mj_forward` 收尾，且这**不是**可选的优化。
        #
        # 成因（实测）：`mj_step` 是"先算 `site_xpos`，再积分 `qpos`"——
        # 于是步进结束后，`d.site_xpos` 描述的是**积分前**的位形，
        # 而 `d.qpos` 已经是**积分后**的。二者相差整整一个子步的转动量。
        # 实测：`shoulder` 速度 0.02 rad/步 时，
        # `site_xpos` 与"由当前 qpos 反算的位置"相差 5.6e-8 m
        # （约 56 纳米 —— 对刚体运动学来说是个巨大的误差）。
        #
        # 后果：`get_state()` 会返回**自相矛盾**的一帧——关节角是 t 时刻的，
        # 末端位姿是 t-dt 时刻的。这个不一致在 IK / 抓取里会表现为
        # "算出来的目标位姿对不上手"，而单看任何一半都完全正常。
        #
        # 修法：推进完后调一次 `mj_forward`，把全部派生量（`xpos` / `xmat` /
        # 传感器）重算到与 `qpos` **同一时刻**。代价是一次纯运动学前向，
        # 不做积分、不改状态，是毫秒级。
        #
        # 判据：`TestSnapshotProbe::test_tcp_gap_is_at_machine_precision`
        # 与 `test_mujoco_quaternion_matches_core_fk` 钉住这一点。
        mujoco.mj_forward(self._mj, self._data)

    def _end_effector_pose(self, positions: Mapping[str, float]) -> Transform | None:
        """末端位姿。走 Core FK；无 EndEffector 或链不完整 ⇒ `None`。

        另外做一次**独立裁判**：若 MuJoCo 有 `tcp` site，
        就用它的位姿交叉验证 Core FK 的结果（见 `snapshot()` 里的
        `tcp_position_gap`）。这不是运行时校验，是诊断量 ——
        它让"MJCF 几何与 RobotModel 几何是否对齐"变成可观测的，
        而不是只在测试里被检查一次。
        """
        if self._model.default_end_effector() is None:
            return None
        try:
            return forward_kinematics(self._model, dict(positions))
        except (KeyError, ValueError):
            return None

    # ------------------------------------------------------------------
    # 诊断
    # ------------------------------------------------------------------

    def _resolve_tcp_site(self, mujoco: Any, mj: Any) -> int:
        """找 `tcp` site 的 id；没有就 -1（诊断量缺省）。"""
        site_id = mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        return int(site_id)

    def snapshot(self) -> dict[str, Any]:
        """无副作用的内部快照（与 `MockBackend.snapshot` 同构）。

        `tcp_position_gap`：MuJoCo 的 `site_xpos` 与 Core FK 的 TCP 位置之差。
        它应当是小量（≈1e-16）。它变大只可能是两件事之一：
        ① MJCF 的几何被改了而 RobotModel 的常量没跟着改；
        ② 四元数/坐标换算出错（此时位置也会歪）。

        ⚠️ 这个量**不是**精度指标，不得用作验收阈值 ——
        它是"两套几何是否对齐"的探针，正常值应当约等于机器精度。
        """
        snap: dict[str, Any] = {
            "started": self._started,
            "status": self._status,
            "joints": list(self._joint_ids),
            "backend": type(self).__name__,
            "sim_time": self._sim_time,
            "substeps": self.substeps,
            "command_count": self.command_count,
            "targets": dict(self._targets),
            "clamped": dict(self.clamped),
        }
        if self._started:
            positions = {
                jid: float(self._data.qpos[self._qpos_addr[i]])
                for i, jid in enumerate(self._joint_ids)
            }
            snap["positions"] = positions
            snap["velocities"] = {
                jid: float(self._data.qvel[self._dof_addr[i]])
                for i, jid in enumerate(self._joint_ids)
            }
            pose = self._end_effector_pose(positions)
            if pose is not None:
                snap["core_fk_position"] = pose.position.to_list()
                snap["core_fk_quaternion_xyzw"] = pose.orientation.to_list()
            if self._tcp_site_id >= 0:
                snap["mj_tcp_position"] = [
                    float(v) for v in self._data.site_xpos[self._tcp_site_id]
                ]
                mat = self._data.site_xmat[self._tcp_site_id]
                snap["mj_tcp_quaternion_wxyz"] = [
                    float(v) for v in _mat_to_quat_wxyz(mat)
                ]
                if pose is not None:
                    snap["tcp_position_gap"] = max(
                        abs(a - b)
                        for a, b in zip(
                            snap["mj_tcp_position"], snap["core_fk_position"]
                        )
                    )
        return snap


def _mat_to_quat_wxyz(mat: Any) -> list[float]:
    """旋转矩阵（MuJoCo 的 9 分量行主序）→ 四元数 **`[w,x,y,z]`**。

    用 MuJoCo 官方 API 而不是手写 Shepperd 分支：手写版本在
    `trace ≈ -1` 附近有分支选择问题，而"偶尔错一次"的换算
    在稀疏采样的测试里几乎无法发现。

    返回值**保持 MuJoCo 顺序**，转换到 `[x,y,z,w]` 是调用方
    （`mj_quat_to_xyzw`）的事 —— 本函数不越界做第二步重排。

    ⚠️ `mju_mat2Quat` 的签名是 `(quat, mat)` **原地写 numpy 数组**，
    不接受 Python list。踩过一次：传 list 会报
    `incompatible function arguments`，而错误信息里只列出"支持 NDArray"，
    很容易被误读成"参数顺序反了"。
    """
    import numpy as np

    mujoco = _require_mujoco()
    out = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(out, np.asarray(mat, dtype=np.float64).reshape(9))
    return [float(v) for v in out]


def mujoco_backend_factory(model: RobotModel) -> RobotBackend:
    """`BackendFactory`：把 `MuJoCoBackend` 注入 `RobotRuntime` / `create_app`。

    用法（§69 规则 10 的立足点）：

    ```python
    app = create_app(backend_factory=mujoco_backend_factory)
    ```

    Runtime / WebSocket / 路由**一行不改**。
    """
    return MuJoCoBackend(model)


#: `Vector3` / `Quaternion` 在这里 import 是为了让 `Transform` 的类型
#: 在运行时可用（`_end_effector_pose` 的返回类型注解），并让
#: `__all__` 里的两个换算函数的调用方不必再 import types。
_ = (Vector3, Quaternion)


__all__ = [
    "DEFAULT_MAX_SUBSTEPS",
    "MuJoCoBackend",
    "JointOrderError",
    "mj_quat_to_xyzw",
    "mujoco_backend_factory",
    "xyzw_to_mj_quat",
]
