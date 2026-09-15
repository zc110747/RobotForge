"""运行时层（提示词 §46）—— `Package → Loader → RobotModel → RobotRuntime`。

本层的三个契约按 §69 规则 8/9 **必须分离**，所以这里是三个模块：

| 模块 | 契约 | 语义 |
|---|---|---|
| `command.py` | `RobotCommand` | **Desired** —— 我要它去哪 |
| `state.py` | `RobotState` | **Actual** —— 它现在在哪 |
| `backend.py` | `RobotBackend` | 执行端抽象（v0.1: `MockBackend`） |
| `robot_runtime.py` | `RobotRuntime` | 上面三者的持有者与编排者 |

## 依赖方向（不可反转）

```text
runtime  →  model          （读写 RobotModel）
runtime  →  kinematics     （Backend 用 Core FK 算末端位姿）
runtime  →  api.registry   （发现/加载包）
runtime  →  loaders
```

反向都不允许：

* `model` / `kinematics` **不得** import `runtime`
  （`tests/test_runtime.py` 与 `accept_phase4.py` 都会扫这个）
* 特别地：`kinematics` 是纯数学，它连 `runtime` 的存在都不该知道。

## Runtime 不知道实现

§46 的硬要求：本层不得出现 `mini_arm` / `mearm` / `serial` / `CAN` / `USB`
等具体实现字面量。Backend 通过**工厂函数**注入，
所以"换成 MuJoCo"或"换成真机"在这里是换一个参数，不是改代码。
"""

from .backend import (
    DEFAULT_MAX_STEP,
    BackendError,
    MockBackend,
    RobotBackend,
)
from .command import RobotCommand
from .robot_runtime import (
    BackendFactory,
    RobotRuntime,
    RuntimeError_,
    default_backend_factory,
)
from .state import STATUSES, RobotState

__all__ = [
    "DEFAULT_MAX_STEP",
    "STATUSES",
    "BackendError",
    "BackendFactory",
    "MockBackend",
    "RobotBackend",
    "RobotCommand",
    "RobotRuntime",
    "RobotState",
    "RuntimeError_",
    "default_backend_factory",
]
