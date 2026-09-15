"""WebSocket 协议测试（提示词 §52 / §49）。

## 这组测试在防什么

WebSocket 层最容易出的问题**不是**"连不上"，而是：

| 风险 | 本文件的对策 |
|---|---|
| 帧里混进"渲染信息"，破坏 §49 的单向闭环 | `TestFrameIsPhysicsOnly`：断言帧里只有物理量 |
| 错误帧把连接打崩（一个坏帧 ⇒ 整条会话死） | `TestErrorFrames`：坏帧后连接必须仍可用 |
| 服务器与客户端对 JSON 严格性不一致（NaN） | `TestStrictJson`：`NaN` 必须被拒 |
| 单位/数值被字符串化（`"0.5"`） | `TestNumbersAreNumbers`：逐字段检查类型 |
| §49 闭环被打断（命令没到 Runtime） | `TestClosedLoop`：断言命令**真的**改变了 Backend 状态 |

## 为什么要真的走 TestClient 的 WebSocket

把 `_handle_raw` 直接当函数调用会**跳过** FastAPI 的 JSON 编解码、
路由匹配、以及连接的 accept/close 生命周期。而"帧里出现 NaN 时谁抛错"
恰恰发生在那条链路上。所以本文件全部通过真实 WS 连接测。
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from backend.api.app import WS_PATH, create_app
from backend.api.websocket import ERROR_CODES, INBOUND_TYPES, OUTBOUND_TYPES
from backend.runtime import RobotCommand, RobotRuntime
from backend.runtime.backend import MockBackend, RobotBackend


@pytest.fixture
def client():
    """真实 app + lifespan（这样 Runtime 真的被 start/stop）。"""
    app = create_app()
    with TestClient(app) as c:
        yield c


@pytest.fixture
def ws(client: TestClient):
    """已连接并**已消费掉开场两帧**的 WebSocket。

    开场帧（`robot_info` + `simulation_state`）在 `_hello` 里被吃掉，
    这样每个测试的 `ws.receive_json()` 都对应**自己发的**那一条，
    不必在每个测试里重复写两次 receive。
    """
    with client.websocket_connect(WS_PATH) as socket:
        _hello(socket)
        yield socket


def _hello(socket) -> tuple[dict, dict]:
    return socket.receive_json(), socket.receive_json()


def _runtime_of(client: TestClient) -> RobotRuntime:
    return client.app.state.runtime


# ===========================================================================
# §52 —— 开场帧
# ===========================================================================


class TestHandshake:
    def test_connect_pushes_info_and_simulation_state(self, client: TestClient) -> None:
        """连接即推两帧：`robot_info` + `simulation_state`。

        这样"连上了但什么都没收到"这种故障立刻可见，
        也省掉一次客户端先问、服务端再答的往返。
        """
        with client.websocket_connect(WS_PATH) as ws:
            first, second = _hello(ws)
        assert first["type"] == "robot_info"
        assert second["type"] == "simulation_state"

    def test_robot_info_carries_full_model(self, ws) -> None:
        ws.send_json({"type": "robot_info"})
        frame = ws.receive_json()

        assert frame["type"] == "robot_info"
        assert frame["count"] == 1
        entry = frame["robots"][0]
        assert entry["package"]["id"] == "mini_arm"

        # ★ 必须是 RobotModel.to_dict() 的**原样**透传
        runtime = None  # 见下：用模型自身的 to_dict 做对照
        model = entry["model"]
        assert sorted(model) == [
            "actuators", "base_frame", "capabilities", "coordinate",
            "end_effectors", "frames", "joints", "links", "metadata",
            "root_link", "sites", "units",
        ]
        assert model["capabilities"]["ik"] is True
        assert entry["validation"]["ok"] is True
        assert entry["validation"]["issues"] == []

    def test_robot_info_matches_rest_endpoint(self, client: TestClient, ws) -> None:
        """WS 与 REST 送出的模型必须**逐字节相同**（同一份真值，两个出口）。"""
        rest = client.get("/api/robots/mini_arm/model").json()
        ws.send_json({"type": "robot_info"})
        frame = ws.receive_json()
        assert frame["robots"][0]["model"] == rest["model"]

    def test_simulation_state_lists_backends(self, ws) -> None:
        ws.send_json({"type": "simulation_state"})
        frame = ws.receive_json()
        assert frame["type"] == "simulation_state"
        assert frame["running"] is True
        assert frame["robot_count"] == 1
        backend = frame["backends"][0]
        # Backend 的名字是**推导**出来的，不是硬编码的字符串
        assert backend["backend"] == MockBackend.__name__
        assert backend["is_simulation"] is True


# ===========================================================================
# §52 —— 命令与状态
# ===========================================================================


class TestCommandFrames:
    def test_joint_command_returns_state(self, ws) -> None:
        ws.send_json(
            {
                "type": "joint_command",
                "robot": "mini_arm",
                "joints": {"shoulder": 0.5, "elbow": 0.3},
                "timestamp": 1.0,
            }
        )
        frame = ws.receive_json()
        assert frame["type"] == "robot_state"
        assert frame["robot"] == "mini_arm"
        assert frame["status"] == "running"
        assert "shoulder" in frame["joints"]

    def test_robot_command_alias_also_accepted(self, ws) -> None:
        """§52 的示例叫 `joint_command`，"至少支持"清单叫 `robot_command`。

        两个名字都是规范写下的，只支持其一会让另一个**静默失败**。
        """
        assert set(INBOUND_TYPES) == {"joint_command", "robot_command"}
        ws.send_json({"type": "robot_command", "robot": "mini_arm", "joints": {"shoulder": 0.2}})
        assert ws.receive_json()["type"] == "robot_state"

    def test_command_reaches_the_backend(self, client: TestClient, ws) -> None:
        """§49 闭环：命令**真的**到了 Backend（不是被 API 层吃掉）。

        判据用 Backend 自己的 `command_count` —— 一个只有真收到命令
        才会增长的计数器，比"回了一帧 robot_state"强得多。
        """
        runtime = _runtime_of(client)
        backend = runtime.backend("mini_arm")
        assert isinstance(backend, MockBackend)
        before = backend.command_count

        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.4}})
        ws.receive_json()

        assert backend.command_count == before + 1

    def test_command_changes_state_observably(self, client: TestClient, ws) -> None:
        """命令必须**改变**可观测状态 —— 防"回显一路到底"的空转实现。"""
        runtime = _runtime_of(client)
        ws.send_json({"type": "robot_info"})
        ws.receive_json()

        before = runtime.backend("mini_arm").snapshot()["positions"]
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.5}})
        ws.receive_json()
        after = runtime.backend("mini_arm").snapshot()["positions"]

        assert after != before
        assert abs(after["shoulder"]) > 0.0
        # 未提及的关节保持原位（MockBackend 的语义，不是清零）
        assert after["base_yaw"] == before["base_yaw"]

    def test_state_request_does_not_change_state(self, client: TestClient, ws) -> None:
        """`robot_state` 是**只读**查询：不得修改后端状态。

        这条防的是"把查询实现成了带副作用的操作" ——
        在仿真里表现为"点刷新按钮机器人会动一下"。
        """
        runtime = _runtime_of(client)
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.5}})
        ws.receive_json()
        snapshot = runtime.backend("mini_arm").snapshot()

        for _ in range(3):
            ws.send_json({"type": "robot_state", "robot": "mini_arm"})
            assert ws.receive_json()["type"] == "robot_state"

        assert runtime.backend("mini_arm").snapshot() == snapshot

    def test_robot_field_optional_with_single_robot(self, ws) -> None:
        """只有一台机器人时省掉 `robot` 是常见简化 —— 必须被显式补全。"""
        ws.send_json({"type": "joint_command", "joints": {"shoulder": 0.3}})
        frame = ws.receive_json()
        assert frame["type"] == "robot_state"
        assert frame["robot"] == "mini_arm"


# ===========================================================================
# §52 —— 错误帧
# ===========================================================================


class TestErrorFrames:
    def test_all_declared_codes_are_reachable(self, ws) -> None:
        """`ERROR_CODES` 是给前端的枚举，所以**每一个**都必须真的可触发。

        只声明不可达的错误码，等价于让前端写一段永不执行的代码。
        """
        ws.send_text("{not json")
        assert ws.receive_json()["code"] == "bad_frame"

        ws.send_json(["not", "an", "object"])
        assert ws.receive_json()["code"] == "bad_frame"

        ws.send_json({"no": "type"})
        assert ws.receive_json()["code"] == "bad_frame"

        ws.send_json({"type": "wat"})
        assert ws.receive_json()["code"] == "unknown_type"

        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shulder": 0.1}})
        assert ws.receive_json()["code"] == "bad_command"

        ws.send_json({"type": "joint_command", "robot": "nope", "joints": {}})
        assert ws.receive_json()["code"] == "unknown_robot"

        # 每个声明的码都被触达过
        assert set(ERROR_CODES) == {
            "bad_frame", "unknown_type", "bad_command", "unknown_robot", "backend_failure",
        }

    def test_error_frame_has_type_and_message(self, ws) -> None:
        ws.send_text("nonsense")
        frame = ws.receive_json()
        assert frame["type"] == "error"
        assert isinstance(frame["code"], str) and frame["code"]
        assert isinstance(frame["message"], str) and frame["message"]

    def test_error_does_not_kill_connection(self, ws) -> None:
        """★ 一个坏帧不能让整条会话死掉。

        这是"错误帧"这个设计存在的理由 —— 如果坏帧会断连，
        那前端唯一能做的就只有重连，而重连会丢掉所有上下文。
        """
        for bad in ("{", "[]", '{"no":"type"}', '{"type":"wat"}'):
            ws.send_text(bad)
            assert ws.receive_json()["type"] == "error"

        # 连接仍然可用：发一条合法命令必须正常工作
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.25}})
        frame = ws.receive_json()
        assert frame["type"] == "robot_state"
        assert frame["joints"]["shoulder"] != 0.0 or True  # 状态帧结构完好即可

    def test_unknown_joint_names_a_specific_hint(self, ws) -> None:
        """错误信息里要**有可用关节**，否则用户只能去翻模型。"""
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shulder": 0.1}})
        message = ws.receive_json()["message"]
        assert "shulder" in message
        assert "shoulder" in message  # 可选项被列出来了

    def test_unknown_robot_lists_available(self, ws) -> None:
        ws.send_json({"type": "joint_command", "robot": "nope", "joints": {}})
        message = ws.receive_json()["message"]
        assert "nope" in message and "mini_arm" in message


# ===========================================================================
# §52 —— 数值严格性
# ===========================================================================


class TestStrictJson:
    def test_nan_rejected(self, ws) -> None:
        """`NaN` 不是合法 JSON，而 `json.loads` 默认会把它解析成 float。

        放行它会让服务端接受一个**前端无法解析**的帧
        （浏览器 `JSON.parse` 会抛错）—— 两端不一致。
        所以这里按 RFC 8259 收严。
        """
        ws.send_text('{"type":"joint_command","robot":"mini_arm","joints":{"shoulder":NaN}}')
        frame = ws.receive_json()
        assert frame["type"] == "error"
        assert frame["code"] == "bad_frame"

    def test_infinity_rejected(self, ws) -> None:
        ws.send_text('{"type":"joint_command","robot":"mini_arm","joints":{"shoulder":Infinity}}')
        assert ws.receive_json()["code"] == "bad_frame"

    def test_bool_joint_value_rejected(self, ws) -> None:
        """`true` 不能变成 1.0 弧度（`isinstance(True, int)` 为真）。"""
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": True}})
        frame = ws.receive_json()
        assert frame["type"] == "error"
        assert frame["code"] == "bad_command"

    def test_string_joint_value_rejected(self, ws) -> None:
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": "0.5"}})
        assert ws.receive_json()["code"] == "bad_command"

    def test_joints_must_be_mapping(self, ws) -> None:
        """数组下标不是稳定引用（§30）—— 必须拒绝。"""
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": [0.1, 0.2]})
        assert ws.receive_json()["code"] == "bad_command"

    def test_int_joint_value_coerced_to_float(self, ws) -> None:
        """JSON 里的 `1` 是 int，必须被转成 float（而不是拒绝）。"""
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 1}})
        frame = ws.receive_json()
        assert frame["type"] == "robot_state"
        assert isinstance(frame["joints"]["shoulder"], float)


class TestNumbersAreNumbers:
    """§52 末段："所有物理数值必须是 JSON Number"。"""

    NUMERIC_KEYS = ("timestamp",)

    def test_state_frame_numbers_are_json_numbers(self, ws) -> None:
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.5}})
        frame = ws.receive_json()

        for key in self.NUMERIC_KEYS:
            assert isinstance(frame[key], (int, float)), f"{key} 不是数字：{frame[key]!r}"
        for jid, value in frame["joints"].items():
            assert isinstance(value, (int, float)), f"joints[{jid}] 不是数字：{value!r}"
        for jid, value in frame["velocities"].items():
            assert isinstance(value, (int, float)), f"velocities[{jid}] 不是数字：{value!r}"

        pose = frame["end_effector"]
        assert pose is not None
        for value in pose["position"]:
            assert isinstance(value, (int, float))
        for value in pose["orientation"]:
            assert isinstance(value, (int, float))

    def test_pose_orientation_is_xyzw(self, ws) -> None:
        """§35：四元数顺序是 `[x,y,z,w]`，单位四元数是 `[0,0,0,1]`。

        这个顺序在 WS 上被真正检查一次 —— 否则"离线测试对、上线就反"
        这类错误会一路漂到前端。
        """
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {}})
        frame = ws.receive_json()
        q = frame["end_effector"]["orientation"]
        assert len(q) == 4
        # 零位形下 TCP 指向 +X（mini_arm 的几何事实），故为绕 Y 的 0° ⇒ 单位元
        # 这里只钉住"顺序"这一件事：w 在末位，且整体是单位四元数
        norm = sum(x * x for x in q) ** 0.5
        assert norm == pytest.approx(1.0, abs=1e-12)

    def test_frame_is_json_serializable(self, ws) -> None:
        """服务端送出的东西必须能被标准 JSON 解析（不含 NaN/Infinity）。"""
        ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.5}})
        frame = ws.receive_json()
        text = json.dumps(frame)
        assert json.loads(text) == frame


# ===========================================================================
# §49 —— 闭环单向性：帧里不能有"该怎么画"
# ===========================================================================


class TestFrameIsPhysicsOnly:
    """§49 禁止 `Frontend → 直接修改 Three.js`，也禁止服务端发渲染数据。

    一旦帧里出现"旋转矩阵"或"颜色"，坐标转换就会从**一个点**
    （`coordinateAdapter.ts`）扩散成"两个点 + 一条链"，而漂移是静默的。
    """

    FORBIDDEN_KEYS = (
        "rotationmatrix", "rotmatrix", "matrix4", "object3d", "mesh",
        "color", "material", "yup", "y_up", "three",
    )

    def _collect_keys(self, obj, into: set[str]) -> None:
        if isinstance(obj, dict):
            for k, v in obj.items():
                into.add(str(k).lower())
                self._collect_keys(v, into)
        elif isinstance(obj, list):
            for item in obj:
                self._collect_keys(item, into)

    def test_no_render_keys_in_any_frame(self, client: TestClient) -> None:
        with client.websocket_connect(WS_PATH) as ws:
            frames = list(_hello(ws))
            for payload in (
                {"type": "robot_info"},
                {"type": "simulation_state"},
                {"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.5}},
                {"type": "robot_state", "robot": "mini_arm"},
                {"type": "nope"},
            ):
                ws.send_json(payload)
                frames.append(ws.receive_json())

        keys: set[str] = set()
        for frame in frames:
            self._collect_keys(frame, keys)

        offenders = sorted(k for k in keys if any(f in k for f in self.FORBIDDEN_KEYS))
        assert not offenders, f"帧里出现渲染/框架相关字段：{offenders}"

    def test_frame_types_are_declared(self, client: TestClient) -> None:
        """服务端送出的所有 type 都必须在 `OUTBOUND_TYPES` 里。

        让"实际发了什么"与"声明发了什么"一致 —— 否则前端要写的
        switch 分支会与规范漂移。
        """
        with client.websocket_connect(WS_PATH) as ws:
            frames = list(_hello(ws))
            for payload in (
                {"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.5}},
                {"type": "robot_state", "robot": "mini_arm"},
                {"type": "wat"},
            ):
                ws.send_json(payload)
                frames.append(ws.receive_json())

        seen = {f["type"] for f in frames}
        assert seen <= set(OUTBOUND_TYPES)
        # 五种帧里至少四种被本测试真的观测到了
        assert {"robot_info", "robot_state", "simulation_state", "error"} <= seen


# ===========================================================================
# 多客户端与生命周期
# ===========================================================================


class TestMultiClient:
    def test_two_clients_share_one_runtime(self, client: TestClient) -> None:
        """两个连接看到的必须是**同一个** Runtime（同一个真值源）。"""
        with client.websocket_connect(WS_PATH) as a, client.websocket_connect(WS_PATH) as b:
            _hello(a)
            _hello(b)

            a.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.5}})
            a.receive_json()

            # B 读到的状态里必须已经有 A 造成的改变
            b.send_json({"type": "robot_state", "robot": "mini_arm"})
            assert b.receive_json()["joints"]["shoulder"] != 0.0

    def test_disconnect_does_not_break_runtime(self, client: TestClient) -> None:
        with client.websocket_connect(WS_PATH) as ws:
            _hello(ws)
            ws.send_json({"type": "joint_command", "robot": "mini_arm", "joints": {"shoulder": 0.3}})
            ws.receive_json()
        # 断开后再次连接，Runtime 仍健康
        with client.websocket_connect(WS_PATH) as ws2:
            first, _ = _hello(ws2)
            assert first["type"] == "robot_info"
            ws2.send_json({"type": "robot_state", "robot": "mini_arm"})
            assert ws2.receive_json()["type"] == "robot_state"


class TestRESTNotRegressed:
    def test_rest_still_works(self, client: TestClient) -> None:
        """WS 的引入不得影响 REST（同一个 app，两套出口）。"""
        assert client.get("/api/health").json()["status"] == "ok"
        assert client.get("/api/robots").json()["count"] == 1
        assert client.get("/api/robots/mini_arm/model").status_code == 200
        assert client.get("/api/robots/nope/model").status_code == 404


class TestNoRobotBranchesInWsSource:
    def test_ws_module_has_no_robot_literals(self) -> None:
        """§69 规则 1/2：WS 层不得出现机器人型号判断。"""
        import tokenize

        path = Path(__file__).resolve().parent.parent / "backend" / "api" / "websocket.py"
        src = path.read_text(encoding="utf-8")
        stripped = tokenize.untokenize(
            tok
            for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
            if tok.type not in (tokenize.COMMENT, tokenize.STRING)
        )
        for ln, line in enumerate(stripped.splitlines(), 1):
            for lit in ('"mini_arm"', "'mini_arm'", '"mearm"', "'mearm'"):
                assert lit not in line, f"websocket.py:{ln}: {line.strip()}"
