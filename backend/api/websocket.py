"""WebSocket 协议 —— 前后端之间**唯一**的运行期通道（提示词 §52）。

## 协议是信封，不是管道

§52 要求至少支持五种帧：

```text
robot_info          服务器 → 客户端：机器人有哪些、能力是什么
robot_command       客户端 → 服务器：设定关节目标（§52 示例用 type=joint_command）
robot_state         服务器 → 客户端：当前实际状态
simulation_state    服务器 → 客户端：仿真层面的状态（运行中/时间/步数）
error               服务器 → 客户端：这次请求失败的原因
```

## 一条铁律：帧里不放"渲染信息"

§49 画出了**唯一允许**的闭环：

```text
Frontend → WebSocket → RobotCommand → RobotRuntime → MuJoCoBackend
        → MuJoCo → RobotState → WebSocket → Frontend
```

并明确禁止：

```text
Frontend → 直接修改 Three.js
```

所以帧里**只能**有物理量（关节角、位置、速度、时间），
不能有"该怎么画"（比如 `{"rotationMatrix": ...}` 或颜色）。
前端把 `robot_state` 翻译成 Three.js 的 Object3D 是它自己的事，
反过来服务器一旦开始发渲染数据，坐标转换就会从"一个点"
（`coordinateAdapter.ts`，见 §41 与 Phase 2 的验收）
扩散到"两个点 + 一条链"，而漂移是静默的。

## 单位与数值（§52 末两段）

```text
Position m | Angle rad | Linear Speed m/s | Angular Speed rad/s | Time s
所有物理数值必须是 JSON Number
```

"必须是 JSON Number"这条排除的是字符串化数字（`"0.5"`）。
`json.dumps` 会拒绝非有限浮点，所以 `RobotState` / `RobotCommand`
在构造时就已经拒绝了 NaN / inf（见 command.py / state.py 的 `_f`）。
这里用 `json.loads(..., parse_constant=...)` 也**不**额外放行 ——
默认的 `json` 会把 `NaN` / `Infinity` 解析成 float，所以这里显式拒绝。
"""

from __future__ import annotations

import json
import logging
from typing import Any, Mapping

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from ..runtime.backend import BackendError
from ..runtime.command import RobotCommand
from ..runtime.robot_runtime import RobotRuntime, RuntimeError_
from ..runtime.state import RobotState

logger = logging.getLogger(__name__)

#: 客户端 → 服务端的帧类型。§52 的示例叫 `joint_command`，
#: 而 §52 的"至少支持"清单叫 `robot_command`。**两个都收** ——
#: 因为这两个名字都是规范里写下的，只支持其一会让另一个变成静默失败。
INBOUND_TYPES = ("joint_command", "robot_command")

#: 服务端 → 客户端的帧类型。
OUTBOUND_TYPES = ("robot_info", "robot_state", "simulation_state", "error")


def _json_loads_strict(raw: str) -> Any:
    """解析 JSON，并把 `NaN` / `Infinity` 当**错误**（而不是 float）。

    `json.loads('[NaN]')` 默认返回 `[nan]`，而 `nan` 不是合法 JSON。
    放行它等于让前端的 `JSON.parse`（它会抛错）与服务端**不一致**：
    服务端接受、前端无法解析。所以这里按 RFC 8259 收严。
    """
    def _reject(token: str) -> Any:
        raise ValueError(f"帧里出现非法的 JSON 常量 {token!r}（RFC 8259 不允许）")

    return json.loads(raw, parse_constant=_reject)


def _error(code: str, message: str, **extra: Any) -> dict[str, Any]:
    frame: dict[str, Any] = {"type": "error", "code": code, "message": message}
    frame.update(extra)
    return frame


#: 错误码 → 语义。放在一处便于前端枚举，避免前端去匹配 message 文本。
ERROR_CODES = {
    "bad_frame": "帧不是合法 JSON，或缺少 type",
    "unknown_type": "type 不在 INBOUND_TYPES 里",
    "bad_command": "命令内容非法（未知关节 / 非数字 / NaN）",
    "unknown_robot": "robot 字段指向一个未加载的机器人",
    "backend_failure": "Backend 执行失败（未启动 / 内部异常）",
}


def create_ws_router(runtime: RobotRuntime) -> APIRouter:
    """构造承载本协议的 router。**Runtime 由外部注入**。

    ## 为什么不用模块级全局 runtime

    因为测试需要造一个"只含人造机器人"的 Runtime
    （`tests/test_websocket.py` 就是这么做的）。
    模块级单例会让"加一台不存在的机器人来验证 unknown_robot 错误帧"
    变成必须去改仓库文件 —— 那就不是测试而是在改被测对象。

    这与 `create_app(packages_dir=...)` 的工厂理由完全一致。
    """
    router = APIRouter()
    path = "/ws"

    @router.websocket(path)
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        # 连接即推 robot_info：省掉一次"客户端先问、服务端再答"的往返，
        # 也让"连上了但什么都没收到"这种故障**立刻**可见。
        try:
            await ws.send_json(_robot_info_frame(runtime))
            await ws.send_json(await _simulation_state_frame(runtime))
        except (RuntimeError_, BackendError) as exc:
            await ws.send_json(_error("backend_failure", str(exc)))
            await ws.close()
            return

        try:
            while True:
                raw = await ws.receive_text()
                frame = await _handle_raw(runtime, raw)
                if frame is not None:
                    await ws.send_json(frame)
        except WebSocketDisconnect:
            # 正常断开。**不**打 error 帧（对端已经走了）。
            logger.debug("WebSocket 客户端断开")
        except Exception:  # noqa: BLE001 - 一条连接不该带崩整个服务
            logger.exception("WebSocket 处理帧时出现未预期异常")
            try:
                await ws.send_json(_error("backend_failure", "服务端内部异常"))
            except Exception:  # pragma: no cover - 对端已不可达
                pass

    return router


async def _handle_raw(runtime: RobotRuntime, raw: str) -> dict[str, Any] | None:
    """把一帧文本变成**至多一帧**响应。返回 `None` = 不回（静默帧）。

    ⚠️ 为什么可能是 `None`：`robot_command` 不产生即时响应，
    它只改变 Backend 状态；状态由随后的 `robot_state` 帧（轮询或订阅）给出。
    如果为每条命令回一帧"OK"，就创造了一种**没人在等**的响应，
    而它会在多客户端时让"这条 state 是哪条命令的结果"变得不可判定。
    """
    try:
        payload = _json_loads_strict(raw)
    except (ValueError, TypeError) as exc:
        return _error("bad_frame", f"不是合法 JSON：{exc}")

    if not isinstance(payload, Mapping):
        return _error("bad_frame", f"帧的顶层必须是对象，实际是 {type(payload).__name__}")

    kind = payload.get("type")
    if not isinstance(kind, str) or not kind:
        return _error("bad_frame", "帧缺少字符串字段 `type`")

    # ---- 客户端 → 服务端：命令 ----
    if kind in INBOUND_TYPES:
        return await _handle_command(runtime, payload)

    # ---- 客户端 → 服务端：请求全量信息 / 单次状态 ----
    if kind == "robot_info":
        try:
            return _robot_info_frame(runtime)
        except (RuntimeError_, BackendError) as exc:
            return _error("backend_failure", str(exc))

    if kind == "robot_state":
        try:
            rid = _require_robot_id(payload, runtime)
            state = await runtime.get_state(rid)
            return _state_frame(state)
        except _FrameError as exc:
            return exc.frame
        except (RuntimeError_, BackendError) as exc:
            return _error("unknown_robot", str(exc))

    if kind == "simulation_state":
        try:
            return await _simulation_state_frame(runtime)
        except (RuntimeError_, BackendError) as exc:
            return _error("backend_failure", str(exc))

    return _error(
        "unknown_type",
        f"不认识 type = {kind!r}；支持 {list(INBOUND_TYPES)} + "
        f"['robot_info', 'robot_state', 'simulation_state']",
    )


class _FrameError(Exception):
    """把一个错误帧当异常抛，便于在多层 `except` 里统一返回。"""

    def __init__(self, frame: dict[str, Any]) -> None:
        super().__init__(frame.get("message", ""))
        self.frame = frame


def _require_robot_id(payload: Mapping[str, Any], runtime: RobotRuntime) -> str:
    rid = payload.get("robot")
    if rid is None:
        # 只有一个机器人时省掉 robot 字段是常见的客户端简化，
        # 但**必须**在这里显式补全，不能让 None 一路传下去。
        available = sorted(runtime.models())
        if len(available) == 1:
            return available[0]
        raise _FrameError(
            _error(
                "unknown_robot",
                f"帧缺少 `robot` 字段，而当前有 {len(available)} 台机器人"
                f"（{available}）；必须显式指定。",
            )
        )
    if not isinstance(rid, str) or not rid:
        raise _FrameError(_error("bad_frame", f"`robot` 必须是非空字符串，实际是 {rid!r}"))
    if not runtime.has_robot(rid):
        raise _FrameError(
            _error("unknown_robot", f"没有机器人 {rid!r}；可用 {sorted(runtime.models())}")
        )
    return rid


async def _handle_command(
    runtime: RobotRuntime, payload: Mapping[str, Any]
) -> dict[str, Any] | None:
    """命令帧 → Runtime。成功返回 `None`，失败返回 `error` 帧。"""
    try:
        rid = _require_robot_id(payload, runtime)

        # §52 的示例用 `joints`；也接受内部契约名 `joint_targets`
        # （`RobotCommand.to_dict()` 输出的是前者，所以前后端对齐用的是前者）。
        raw_targets = payload.get("joints", payload.get("joint_targets", {}))

        command = RobotCommand.of(
            robot=rid,
            joint_targets=raw_targets,
            timestamp=payload.get("timestamp", 0.0),
        )

        # ★ 走 `step` 而不是 `send_command`：回显一帧实际状态，
        #   让前端**立刻**看到命令的后果（含限幅/限速后的真实值）。
        #   这不是"命令的响应"，而是"当前状态" —— 语义仍是 State = Actual。
        state = await runtime.step(command)
    except _FrameError as exc:
        return exc.frame
    except (TypeError, ValueError) as exc:
        # 未知关节 / bool / NaN / 非法 timestamp 都落这里。
        # 这些是**调用方**的错误，所以是可解释的 bad_command，
        # 而不是笼统的 500。
        return _error("bad_command", str(exc))
    except (RuntimeError_, BackendError) as exc:
        return _error("backend_failure", str(exc))

    return _state_frame(state)


# ---------------------------------------------------------------------------
# 出站帧构造
# ---------------------------------------------------------------------------


def _robot_info_frame(runtime: RobotRuntime) -> dict[str, Any]:
    """`robot_info`：**全部**机器人的 RobotModel 序列化 + 校验摘要。

    刻意把 `model` 与 `issues` 一起给（同 REST 的 `/model` 端点）：
    前端需要在画不出来时**显示为什么**，而不是去翻服务器日志。
    """
    robots: list[dict[str, Any]] = []
    for pkg in runtime.packages():
        model = runtime.model(pkg.id)
        report = runtime.validation_report(pkg.id)
        robots.append(
            {
                "package": pkg.to_dict(),
                # ★ 原样透传 RobotModel.to_dict()：一旦在这里整形，
                #   就产生第二份"前端的真值模型"（见 api/app.py 的同一论证）。
                "model": model.to_dict(),
                "validation": {
                    "ok": report.ok,
                    "issues": [
                        {
                            "level": i.level,
                            "code": i.code,
                            "where": i.where,
                            "message": i.message,
                        }
                        for i in report.issues
                    ],
                },
            }
        )
    return {"type": "robot_info", "robots": robots, "count": len(robots)}


def _state_frame(state: RobotState) -> dict[str, Any]:
    """`robot_state`：`RobotState` 的线上形式（§52）。"""
    frame = {"type": "robot_state"}
    frame.update(state.to_dict())
    return frame


async def _simulation_state_frame(runtime: RobotRuntime) -> dict[str, Any]:
    """`simulation_state`：仿真层面的概况（不是关节状态）。

    内容刻意**全部由 RobotModel / Backend 类型推导**，
    不含任何机器人型号判断 —— 这样加一台新机器人时这段代码不用改。
    """
    summary = runtime.summary()
    backends: list[dict[str, Any]] = []
    for rid in summary["robots"]:
        backend = runtime.backend(rid)
        backends.append(
            {
                "robot": rid,
                "backend": type(backend).__name__,
                "is_simulation": bool(backend.is_simulation),
                "status": (await backend.get_state()).status,
            }
        )
    return {
        "type": "simulation_state",
        "running": bool(summary["started"]),
        "robot_count": len(summary["robots"]),
        "backends": backends,
    }


__all__ = [
    "ERROR_CODES",
    "INBOUND_TYPES",
    "OUTBOUND_TYPES",
    "create_ws_router",
]
