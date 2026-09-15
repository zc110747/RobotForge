"""以 **MuJoCoBackend** 启动 API 服务（§61 / §71 的端到端联调入口）。

## 为什么需要这个文件

`uvicorn backend.api.app:create_app --factory` 走的是**默认**工厂
（`create_app()` 不传 `backend_factory` ⇒ Runtime 用 `MockBackend`）。
MockBackend 是"无物理的替身"，因此用它跑出来的
"命令 → 状态"链路**不构成 §71 的 Sim2Sim 证据** ——
它的关节是"限速走一步"，不是"积分出来的"。

要验证 §71 的真实链路，必须把 `mujoco_backend_factory` 注入进去：

```text
RobotCommand → RobotRuntime → MuJoCoBackend → mj_step → MuJoCo
             → Core FK → RobotState → WebSocket → Frontend
```

`create_app(backend_factory=...)` 是 §69 规则 10 的立足点：
Runtime / WebSocket / 路由**一行不改**，只换注入的工厂。

## 用法

    .venv/Scripts/python.exe -m uvicorn tools.serve_mujoco:app --host 127.0.0.1 --port 8000
"""

from __future__ import annotations

from backend.api.app import create_app
from backend.simulation import mujoco_backend_factory

#: uvicorn 的 `--factory` **不**需要传，因为这个名字已经绑定的是 app 实例。
app = create_app(backend_factory=mujoco_backend_factory)

if __name__ == "__main__":  # pragma: no cover
    import uvicorn

    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="info")
