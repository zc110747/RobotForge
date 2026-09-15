"""FastAPI 应用工厂 + REST 路由。

## 本模块的**唯一**职责

把 `RobotModel` 送出去。它**不做**几何计算、**不做**坐标转换、
**不做**字段重命名。

## 为什么响应体就是 `RobotModel.to_dict()`

§59 的最后一条验收是"RobotModel 与 Renderer 语义一致"。
达成它的**唯一可靠方式**是让前端吃到的就是 RobotModel 的序列化结果 ——
一旦 API 层做一次"为了前端方便"的整形（比如把 `joints` 拍平成 `axes` 数组），
就从"一份真值"变成了"两份真值 + 一个转换器"，
而那个转换器会漂移，且漂移是静默的。

因此这里显式断言 `payload == model.to_dict()`（见 tests/test_api.py）。
"""

from __future__ import annotations

import contextlib
from typing import Any, AsyncIterator

from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse

from ..loaders.loader import LoaderError
from ..runtime.robot_runtime import RobotRuntime, default_backend_factory
from .registry import (
    RobotPackage,
    RobotPackageError,
    discover_packages,
    get_package,
)
from .websocket import create_ws_router

#: API 版本前缀。前端只用这一处常量，避免路由字符串散落。
API_PREFIX = "/api"

#: WebSocket 端点的绝对路径（不含 API_PREFIX —— 它不是 REST 资源）。
WS_PATH = "/ws"


def _package_summary(pkg: RobotPackage) -> dict[str, Any]:
    return pkg.to_dict()


def create_app(
    packages_dir: Any = None,
    backend_factory: Any = None,
) -> FastAPI:
    """构造应用。

    `packages_dir` 可注入（测试用临时目录），默认 `packages/`。
    ★ 之所以做成工厂而不是模块级 `app = FastAPI()`：
    让测试能造一个**只含人造机器人**的环境，从而验证"发现式注册"
    真的对任意包生效 —— 否则测试里永远只有 mini_arm，
    而"加一台机器人不用改 Core"这句话就没被验证过。

    ## Runtime 绑在 lifespan 上，而不是模块级

    `RobotRuntime` 的 `start()` 是 `async`（要 await 每个 Backend 的启动），
    所以它必须在事件循环里跑 —— 这正是 FastAPI `lifespan` 的用途。

    模块级构造 Runtime 会在 **import 时**就跑 `discover_packages()`
    + 编译 MJCF，于是"某个包的模型坏了"会表现成"服务起不来"，
    而且 import 一个模块居然有副作用。放在 lifespan 里，
    失败发生在"启动服务"这一步，能被打成一条清晰的启动错误。

    ## `backend_factory` 也可注入

    Phase 5 会把默认的 `MockBackend` 换成 `MuJoCoBackend`，
    那时只需传另一个工厂 —— Runtime / WS / 路由**一行不改**
    （§69 规则 10：Simulation 与 Real 使用统一 Backend Interface）。
    """
    runtime = RobotRuntime(
        packages_dir=packages_dir,
        backend_factory=backend_factory or default_backend_factory,
    )

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        await runtime.start()
        # 把 runtime 挂到 app.state：测试/中间件能拿到**同一个**实例，
        # 而不是各自再造一个（那会让"测试里发的命令没影响被测连接"）。
        app.state.runtime = runtime
        try:
            yield
        finally:
            await runtime.stop()

    app = FastAPI(
        title="RobotForge API",
        version="0.1.0",
        description="RobotForge v0.1 —— RobotModel 服务端。",
        lifespan=lifespan,
    )
    app.state.runtime = runtime

    # ★ WebSocket 与 REST 共用**同一个** runtime 实例（§52 / §49 闭环）
    app.include_router(create_ws_router(runtime))

    @app.get(f"{API_PREFIX}/health")
    def health() -> dict[str, Any]:
        return {"status": "ok", "api_version": "0.1.0"}

    @app.get(f"{API_PREFIX}/robots")
    def list_robots() -> dict[str, Any]:
        """列出所有机器人包（不含几何，轻量）。

        返回 `{"robots": [...], "count": n}` 而不是裸数组：
        裸数组没有地方放未来的分页/版本信息，而把数组改成对象
        是前端的破坏性变更。包一层是廉价的前向兼容。
        """
        try:
            pkgs = discover_packages(packages_dir)
        except RobotPackageError as exc:
            # 包结构坏了 ⇒ 500。这不是 404：服务器**知道**有问题，
            # 而 404 的语义是"你要的那个东西不存在"。
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        return {"robots": [_package_summary(p) for p in pkgs], "count": len(pkgs)}

    @app.get(f"{API_PREFIX}/robots/{{robot_id}}/model")
    def get_robot_model(robot_id: str) -> JSONResponse:
        """返回该机器人的**完整** RobotModel 序列化（Canonical Representation）。

        `issues` 与 `model` 一起返回（而不是错了就 500）：
        前端需要在画不出正确机器人时**显示为什么**。
        一个只回 500 的接口会把这个信息丢掉，让人去翻服务器日志。
        """
        try:
            pkg = get_package(robot_id, packages_dir)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except RobotPackageError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        try:
            model, report = pkg.load_model()
        except LoaderError as exc:
            # 模型加载失败是 422（内容无法处理），不是 500 ——
            # 服务器本身没坏，是这份 MJCF 有问题。
            raise HTTPException(status_code=422, detail=str(exc)) from exc

        return JSONResponse(
            content={
                "robot_id": model.metadata.id,
                # ★ 原样透传，不整形（见模块 docstring）
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

    return app


__all__ = ["API_PREFIX", "WS_PATH", "create_app"]
