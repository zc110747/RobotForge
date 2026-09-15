"""仿真后端层（§50 / §62）。

```text
RobotCommand ──▶ MuJoCoBackend ──▶ MjData ──▶ Core FK ──▶ RobotState
```

## 这一层在链路里的位置

```text
Robot Package → Loader → RobotModel → RobotRuntime → Backend
                                                       ├── MockBackend   （无物理）
                                                       └── MuJoCoBackend （真物理）★
```

`backend/simulation/` 是**执行端**。它与 `backend/runtime/` 的
`MockBackend` 实现同一个 `RobotBackend` 接口（§47），
所以换引擎对 Runtime / WebSocket / 前端是**零改动**（§69 规则 10）。

## 两个文件的分工

| 文件 | 职责 | 依赖 |
|---|---|---|
| `simulation_backend.py` | 关节顺序、限位夹紧、状态装配（**引擎无关**） | 只依赖 `model` / `runtime.state` |
| `mujoco_backend.py` | 编译 MjModel、`mj_step`、四元数边界换算 | 额外依赖 `mujoco` |

把"引擎无关"的那半单独放，是为了让"关节顺序配错"这个 bug 类别
**不会因为换引擎而重生** —— 顺序定义只有一份。

## 依赖方向

```text
simulation → model / kinematics / runtime（只读其契约类型）
simulation → ✗ frontend / ✗ api / ✗ loaders
```

`runtime` **不得** import `simulation`：Backend 是由工厂注入的，
Runtime 连"有个 MuJoCoBackend 存在"都不该知道。这条由
`tests/test_runtime.py::TestArchitecture` 与 `tools/accept_phase5.py` 盯着。
"""

from .mujoco_backend import (
    DEFAULT_MAX_SUBSTEPS,
    MuJoCoBackend,
    mj_quat_to_xyzw,
    mujoco_backend_factory,
    xyzw_to_mj_quat,
)
from .simulation_backend import (
    DEFAULT_STEP_SECONDS,
    JointOrderError,
    assemble_state,
    clamp_targets_to_limits,
    ordered_joint_ids,
    positions_to_vector,
    vector_to_positions,
)

__all__ = [
    "DEFAULT_MAX_SUBSTEPS",
    "DEFAULT_STEP_SECONDS",
    "JointOrderError",
    "MuJoCoBackend",
    "assemble_state",
    "clamp_targets_to_limits",
    "mj_quat_to_xyzw",
    "mujoco_backend_factory",
    "ordered_joint_ids",
    "positions_to_vector",
    "vector_to_positions",
    "xyzw_to_mj_quat",
]
