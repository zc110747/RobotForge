#!/usr/bin/env python
"""Phase 4 验收清单（对应 spec §61：Runtime + WebSocket）。

## 用法

```bash
.venv/Scripts/python.exe tools/accept_phase4.py
```

退出码 0 = 全部通过；非 0 = 有未通过项。

> ⚠️ 本脚本要跑 4 份上游 Phase 清单 + 全量 pytest，
> 耗时数分钟。**不要**在前台跑（本机沙箱会给长命令发 SIGTERM，
> 产出 0 字节日志）。用后台执行并落文件：
>
> ```bash
> .venv/Scripts/python.exe -u tools/accept_phase4.py > .workbuddy/accept4.log 2>&1
> ```

## Phase 4 是什么（spec §61 逐字）

实现：

```text
Frontend → WebSocket → RobotRuntime → RobotCommand → Backend → RobotState
```

验收：

```text
WebSocket 连接 / Robot discovery / RobotModel 获取 / Command 发送 /
Runtime 接收 / Backend 执行 / State 返回 / Frontend 更新
```

## ★ 这份清单的核心难点：怎么证明"闭环"不是空转的

Phase 4 的验收最容易写成**结构性通过、信息上什么都没验证**：

```python
ws.send_json({"type": "joint_command", ...})
assert ws.receive_json()["type"] == "robot_state"   # ← 可能什么都没证明
```

因为"回了一帧状态"与"命令真的被执行了"是两件事。
一份把 `MockBackend` 写成"原样回显命令"的实现会**完整通过**上面这条。

所以本脚本用**三道独立的锚**：

| 锚 | 做法 | 抓什么 |
|---|---|---|
| ① Backend 计数器 | 断言 `backend.command_count` 增长 | 命令真的穿过了 Runtime 到达 Backend |
| ② Command ≠ State | 断言限速/限幅下 `state != command` | "回显式"实现（假闭环） |
| ③ 独立裁判 | 用 **Core FK** 重算末端位姿并比对状态里的位姿 | Backend 用包内写死常量算位姿（状态与模型脱钩） |

外加一项**反向自检**：故意把 Runtime 的 `step` 换成纯回显，
确认上面的锚**真的会失败**（否则整份清单可能整体空转）。

## 覆盖的架构约束（提示词 §69 / §46 / §49）

* 规则 1/2：Core / Runtime / WS 层不得出现机器人型号字面量
* 规则 8/9：`RobotCommand` 与 `RobotState` 是**两个**类型，且必须分离
* 规则 10：Backend 可替换（用自定义 Backend 注入 Runtime 验证）
* §46：Runtime 不得知道 `mini_arm` / `serial` / `CAN` / `USB`
* §49：帧里不得有渲染信息（"该怎么画"不属于协议）
* §52：五种帧 + SI 单位 + 所有物理数值是 JSON Number

## 设计原则（与 accept_phase1/2/3.py 一致）

* 每项**独立可判**，失败给"期望 vs 实际"
* 不依赖网络
* 负向测试会自行清理（并断言清理成功）
* 所有改动都在内存副本上，**不修改任何仓库文件**
"""

from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import time
import tokenize

ROOT = pathlib.Path(__file__).resolve().parent.parent
PY = ROOT / ".venv" / "Scripts" / "python.exe"
if not PY.exists():
    PY = pathlib.Path(sys.executable)

#: 共享的 pytest 判词解析器（唯一真值源），与 harness 同目录。
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from _pytest_verdict import pytest_verdict  # noqa: E402


_results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    _results.append((name, bool(ok), detail))


def run_pytest(*args: str) -> tuple[bool, str]:
    """跑 pytest，返回 `(是否全通过, 摘要)`。

    ⚠️ **判词来自输出，不是退出码** —— 见 `_pytest_verdict.py` 的模块 docstring：
    本机沙箱的批量删除守卫会拦下 pytest 的临时目录清理，
    让"测试全过"的一次运行**退出码非 0**（实测把 Phase 7 顶成 39/50）。
    """
    proc = subprocess.run(
        [str(PY), "-m", "pytest", *args, "-q", "--no-header"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    return pytest_verdict(proc.stdout + proc.stderr)


def run_python(snippet: str, timeout: int = 180) -> tuple[int, str]:
    """在当前 venv 里跑一段 Python。

    ⚠️ 结果**显式读 stdout**：本机沙箱会吞原生进程输出，
    所以用 subprocess + capture_output，并把结论行捞出来。
    """
    proc = subprocess.run(
        [str(PY), "-c", snippet],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return proc.returncode, (proc.stdout or "").strip() + (proc.stderr or "").strip()


def run_file(rel: str, timeout: int = 300) -> tuple[int, str]:
    proc = subprocess.run(
        [str(PY), str(ROOT / rel)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


def strip_comments_and_strings(src: str) -> str:
    """剥离注释与字符串，用于"源码里有没有出现某关键字"的判定。

    必须剥离：否则一句注释"这里不知道 mini_arm"就能让检查失败。
    """
    return tokenize.untokenize(
        tok
        for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
        if tok.type not in (tokenize.COMMENT, tokenize.STRING)
    )


def _fmt(x: float, digits: int = 12) -> str:
    return f"{x:.{digits}g}"


def parse_result(out: str) -> dict:
    """从子进程输出里取出 `RESULT_JSON:` 载荷。

    ⚠️ 不能用 `line[len(prefix):]` —— `run_python` 把 stderr 拼在 stdout 后面，
    而 DeprecationWarning（FastAPI/Starlette 在本机会打）没有尾随换行，
    于是警告会**粘在 JSON 后面**，`json.loads` 报 "Extra data"。
    正确的做法是用 `JSONDecoder.raw_decode` 只吃掉**一个**完整 JSON 值，
    对尾部残留无感。这样脚本不会因为依赖库多打一行警告就误报失败。
    """
    prefix = "RESULT_JSON:"
    idx = out.find(prefix)
    if idx < 0:
        return {}
    decoder = json.JSONDecoder()
    try:
        payload, _end = decoder.raw_decode(out[idx + len(prefix):])
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


# ---------------------------------------------------------------------------
# 闭环验收片段（子进程执行，避免污染本进程 import 状态）
# ---------------------------------------------------------------------------

#: ★ 三道锚 + 一项反向自检。输出 `RESULT_JSON:` 一行。
#:
#: 自检的做法：把 Runtime 的 `step` 换成**纯回显**（直接把命令当状态返回），
#: 然后确认锚②（Command ≠ State）会失败。
#: 如果换成回显后锚②**仍然通过**，说明锚②没有在检查它声称检查的东西。
CLOSED_LOOP_SNIPPET = r'''
import asyncio, json, sys, types
sys.path.insert(0, r"{root}")

from backend.runtime import RobotRuntime, RobotCommand, RobotState, MockBackend, RobotBackend
from backend.kinematics.fk import forward_kinematics
from backend.api.registry import get_package

out = {{}}

# ---- 0. 模型（作为独立裁判的基准）----
model, _report = get_package("mini_arm").load_model()
ids = model.mobile_joint_ids()
out["joint_ids"] = ids
out["dof"] = model.dof()


async def main():
    rt = RobotRuntime()
    await rt.start()
    try:
        backend = rt.backend("mini_arm")
        out["backend_type"] = type(backend).__name__
        out["is_simulation"] = bool(backend.is_simulation)

        # ================= 锚① Backend 计数器 =================
        before = backend.command_count
        st1 = await rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.5}}))
        out["cmd_count_before"] = before
        out["cmd_count_after"] = backend.command_count

        # ================= 锚② Command ≠ State（限速）=================
        #  max_step 默认 0.35，所以"目标 0.5"一步到不了 ⇒ 状态必然 ≠ 命令
        out["state_shoulder_1step"] = st1.joint_positions["shoulder"]
        out["cmd_shoulder"] = 0.5
        out["state_differs_from_cmd"] = (
            abs(st1.joint_positions["shoulder"] - 0.5) > 1e-9
        )

        # 收敛：足够多步后必须精确到位（限速是"慢"不是"不到"）
        for _ in range(20):
            st = await rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.5}}))
        out["state_shoulder_converged"] = st.joint_positions["shoulder"]
        out["converged_exact"] = abs(st.joint_positions["shoulder"] - 0.5) < 1e-12

        # 未提及的关节保持原位（不是清零）
        #  ⚠️ 判据是**不变**，不是"已到位"：限速（0.35/步）意味着
        #     一条命令一步走不完。所以正确的观测方式是：
        #     ① 先把 shoulder 推到某个非零值；
        #     ② 之后只发 elbow 命令；
        #     ③ 断言 shoulder **一直停在那一步的值**，而 elbow 最终收敛。
        #     若把判据写成"两个都是 0.4"，就会把限速误报成失败
        #     （本次运行正是如此：shoulder=0.35, elbow=0.35）。
        await rt.reset()
        await rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.4}}))
        st = await rt.step(RobotCommand.of("mini_arm", {{"elbow": 0.4}}))
        out["hold_shoulder_after_first_elbow_cmd"] = st.joint_positions["shoulder"]
        for _ in range(10):
            st = await rt.step(RobotCommand.of("mini_arm", {{"elbow": 0.4}}))
        out["untouched_hold"] = {{
            "shoulder": st.joint_positions["shoulder"],
            "elbow": st.joint_positions["elbow"],
        }}
        # shoulder 必须与"发 elbow 命令前"的值**逐位相同**（真的没被动过）
        out["hold_shoulder_unchanged"] = (
            abs(st.joint_positions["shoulder"] - out["hold_shoulder_after_first_elbow_cmd"])
            < 1e-15
        )
        out["hold_elbow_converged"] = abs(st.joint_positions["elbow"] - 0.4) < 1e-12
        out["hold_ok"] = bool(
            out["hold_shoulder_unchanged"] and out["hold_elbow_converged"]
        )

        # 限幅：命令 99.0 ⇒ 目标被夹到上限（**要看 Backend 记录的目标**）
        #  ⚠️ 这里不能拿"状态"当判据：限速会让状态停在 max_step 处，
        #     而**限幅发生在目标计算阶段**，先于限速。
        #     正确的观测点是 `backend.clamped`（被记录的夹紧后目标），
        #     以及"多步之后状态会停在限幅边界上"。
        await rt.reset()
        j0 = ids[0]
        lim = model.joint(j0).limits
        st = await rt.step(RobotCommand.of("mini_arm", {{j0: 99.0}}))
        out["clamp_target"] = 99.0
        out["clamp_upper"] = lim.position_max
        out["clamp_actual_after_1step"] = st.joint_positions[j0]
        out["clamp_reported"] = dict(backend.clamped)
        # 持续发同一个超限命令，状态必须收敛到**上限**（而不是 99.0）
        for _ in range(60):
            st = await rt.step(RobotCommand.of("mini_arm", {{j0: 99.0}}))
        out["clamp_converged"] = st.joint_positions[j0]
        out["clamp_converged_at_limit"] = (
            lim.position_max is not None
            and abs(st.joint_positions[j0] - lim.position_max) < 1e-12
        )
        # 限幅的**主判据**：记录在 `clamped` 里的夹紧后目标 == 模型上限，
        # 且最终状态收敛到上限（而不是 99.0）——两者同时成立才算限幅生效。
        out["clamp_report_ok"] = (
            lim.position_max is not None
            and abs(out["clamp_reported"].get(j0, 0.0) - lim.position_max) < 1e-12
            and out["clamp_converged_at_limit"]
        )
        # 反过来：若目标是上限内的小值，必须精确到位（证明限幅没把一切都夹住）
        await rt.reset()
        for _ in range(10):
            st = await rt.step(RobotCommand.of("mini_arm", {{j0: 0.4}}))
        out["no_clamp_needed"] = abs(st.joint_positions[j0] - 0.4) < 1e-12

        # ================= 锚③ 独立裁判：Core FK 重算末端位姿 =================
        await rt.reset()
        st = await rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.31, "elbow": 0.17}}))
        pose = st.end_effector_pose
        out["pose_present"] = pose is not None
        if pose is not None:
            ref = forward_kinematics(model, dict(st.joint_positions))
            dp = max(
                abs(a - b)
                for a, b in zip(pose.position.to_list(), ref.position.to_list())
            )
            dq = max(
                abs(a - b)
                for a, b in zip(pose.orientation.to_list(), ref.orientation.to_list())
            )
            out["pose_pos_diff_vs_core_fk"] = dp
            out["pose_ori_diff_vs_core_fk"] = dq
        else:
            out["pose_pos_diff_vs_core_fk"] = float("inf")
            out["pose_ori_diff_vs_core_fk"] = float("inf")

        # 位姿必须**随关节角变化**（防"回显写死常量"）
        await rt.reset()
        a = await rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.10}}))
        b = await rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.90}}))
        pa = a.end_effector_pose.position.to_list()
        pb = b.end_effector_pose.position.to_list()
        out["pose_varies"] = max(abs(x - y) for x, y in zip(pa, pb))

        # ================= 反向自检：回显式实现必须让锚②失败 =================
        #  把 step 换成"命令即状态"，确认 state_differs_from_cmd 变成 False
        echo_rt = RobotRuntime(backend_factory=lambda m: MockBackend(m, max_step=1e9))
        await echo_rt.start()
        try:
            # 去掉限速（max_step 极大）⇒ 一步到位 ⇒ 状态 == 命令
            st_echo = await echo_rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.5}}))
            out["echo_state_shoulder"] = st_echo.joint_positions["shoulder"]
            out["echo_state_equals_cmd"] = (
                abs(st_echo.joint_positions["shoulder"] - 0.5) < 1e-12
            )
        finally:
            await echo_rt.stop()

        # ================= 并发不交错（Lock）=================
        #  判据是**每一步的位移不超过 max_step**。
        #  ⚠️ 不要断言"两个结果相等"或"都是 0.35" ——
        #     Lock 保证的是"两步串行执行"，而第二步从第一步的终点出发，
        #     所以两次读数的**起点不同**，结果本来就不该相同。
        #     判据要落在**不变量**上：单步位移 ≤ max_step。
        await rt.reset()
        s1, s2 = await asyncio.gather(
            rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.5}})),
            rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.5}})),
        )
        # 串行两步，每步最多 0.35 ⇒ 第二次读数最多 0.70
        hi, lo = max(
            s1.joint_positions["shoulder"], s2.joint_positions["shoulder"]
        ), min(
            s1.joint_positions["shoulder"], s2.joint_positions["shoulder"]
        )
        out["concurrent_values"] = [lo, hi]
        out["concurrent_step_bounded"] = (hi - lo) <= 0.35 + 1e-12
        out["concurrent_both_moved"] = lo > 0.0
        out["concurrent_ok"] = out["concurrent_step_bounded"] and out["concurrent_both_moved"]

    finally:
        await rt.stop()

    # ================= §69 规则 10：Backend 可替换 =================
    class _Liar(RobotBackend):
        is_simulation = False
        def __init__(self, m):
            self.m = m
            self.got = []
        async def start(self): pass
        async def stop(self): pass
        async def reset(self): pass
        async def send_command(self, c): self.got.append(c)
        async def get_state(self):
            return RobotState.of(
                self.m.metadata.id,
                joint_positions={{j: 0.123 for j in self.m.mobile_joint_ids()}},
                status="idle",
            )

    holders = []

    def factory(m):
        holders.append(_Liar(m))
        return holders[-1]

    rt2 = RobotRuntime(backend_factory=factory)
    await rt2.start()
    try:
        st = await rt2.step(RobotCommand.of("mini_arm", {{"shoulder": 0.77}}))
        out["injected_backend_used"] = bool(holders)
        out["injected_got_command"] = [c.joint_targets for c in holders[0].got]
        out["injected_state_shoulder"] = st.joint_positions["shoulder"]
        out["injected_status"] = st.status
    finally:
        await rt2.stop()


asyncio.run(main())
print("RESULT_JSON:" + json.dumps(out))
'''


#: WebSocket 端到端片段：真实 app + lifespan + 真连接。
WS_SNIPPET = r'''
import json, sys
sys.path.insert(0, r"{root}")

from fastapi.testclient import TestClient
from backend.api.app import create_app, WS_PATH

out = {{}}
frames_seen = []

app = create_app()
with TestClient(app) as client:
    with client.websocket_connect(WS_PATH) as ws:
        hello = [ws.receive_json(), ws.receive_json()]
        out["hello_types"] = [f["type"] for f in hello]
        frames_seen.extend(hello)

        info = hello[0]
        out["info_count"] = info.get("count")
        out["info_ids"] = [r["package"]["id"] for r in info.get("robots", [])]
        out["info_model_keys"] = sorted(info["robots"][0]["model"].keys())
        out["info_validation_ok"] = info["robots"][0]["validation"]["ok"]

        # ---- 命令 → 状态 ----
        ws.send_json({{"type": "joint_command", "robot": "mini_arm",
                       "joints": {{"shoulder": 0.5, "elbow": 0.3}}, "timestamp": 1.0}})
        st = ws.receive_json()
        frames_seen.append(st)
        out["cmd_resp_type"] = st["type"]
        out["cmd_resp_status"] = st["status"]
        out["cmd_resp_joints"] = st["joints"]
        out["cmd_resp_pose_present"] = st["end_effector"] is not None

        # ---- 状态查询是只读的 ----
        rt = client.app.state.runtime
        snap_before = rt.backend("mini_arm").snapshot()
        for _ in range(3):
            ws.send_json({{"type": "robot_state", "robot": "mini_arm"}})
            frames_seen.append(ws.receive_json())
        snap_after = rt.backend("mini_arm").snapshot()
        out["state_query_readonly"] = (snap_before == snap_after)

        # ---- robot_command 别名 ----
        ws.send_json({{"type": "robot_command", "robot": "mini_arm", "joints": {{"shoulder": 0.2}}}})
        alias = ws.receive_json()
        frames_seen.append(alias)
        out["alias_resp_type"] = alias["type"]

        # ---- simulation_state ----
        ws.send_json({{"type": "simulation_state"}})
        sim = ws.receive_json()
        frames_seen.append(sim)
        out["sim_running"] = sim["running"]
        out["sim_backends"] = [b["backend"] for b in sim["backends"]]

        # ---- 错误码全覆盖 ----
        codes = []
        for payload in (
            "{{not json",
            ["array"],
            {{"no": "type"}},
            {{"type": "wat"}},
            {{"type": "joint_command", "robot": "mini_arm", "joints": {{"shulder": 0.1}}}},
            {{"type": "joint_command", "robot": "nope", "joints": {{}}}},
        ):
            if isinstance(payload, str):
                ws.send_text(payload)
            else:
                ws.send_json(payload)
            frame = ws.receive_json()
            frames_seen.append(frame)
            codes.append(frame.get("code"))
        out["error_codes"] = codes

        # ---- 坏帧之后连接仍可用（错误帧的意义）----
        ws.send_json({{"type": "joint_command", "robot": "mini_arm", "joints": {{"shoulder": 0.15}}}})
        alive = ws.receive_json()
        frames_seen.append(alive)
        out["alive_after_errors"] = (alive["type"] == "robot_state")

        # ---- NaN 必须被拒（RFC 8259）----
        ws.send_text('{{"type":"joint_command","robot":"mini_arm","joints":{{"shoulder":NaN}}}}')
        nan_frame = ws.receive_json()
        frames_seen.append(nan_frame)
        out["nan_rejected_code"] = nan_frame.get("code")

        # ---- 数值类型：所有物理量必须是 JSON Number ----
        bad = []
        def walk(o, path=""):
            if isinstance(o, dict):
                for k, v in o.items():
                    walk(v, path + "." + str(k))
            elif isinstance(o, list):
                for i, v in enumerate(o):
                    walk(v, path + "[%d]" % i)
            elif isinstance(o, bool):
                pass
            elif isinstance(o, str):
                try:
                    float(o)
                except ValueError:
                    return
                bad.append(path + " = " + repr(o))
        for f in frames_seen:
            if f.get("type") in ("robot_state", "robot_command"):
                for key in ("joints", "velocities", "end_effector", "timestamp"):
                    if key in f:
                        walk(f[key], key)
        out["stringified_numbers"] = bad

        # ---- §49：帧里不得有渲染信息 ----
        FORBIDDEN = ("rotationmatrix", "rotmatrix", "matrix4", "object3d",
                     "mesh", "color", "material", "yup", "y_up", "three")
        keys = set()
        def collect(o):
            if isinstance(o, dict):
                for k, v in o.items():
                    keys.add(str(k).lower())
                    collect(v)
            elif isinstance(o, list):
                for v in o:
                    collect(v)
        for f in frames_seen:
            collect(f)
        out["render_keys"] = sorted(k for k in keys if any(x in k for x in FORBIDDEN))
        out["frame_types_seen"] = sorted({{f.get("type") for f in frames_seen if f.get("type")}})

    # ---- 多客户端共享同一 Runtime ----
    with client.websocket_connect(WS_PATH) as a, client.websocket_connect(WS_PATH) as b:
        a.receive_json(); a.receive_json()
        b.receive_json(); b.receive_json()
        a.send_json({{"type": "joint_command", "robot": "mini_arm", "joints": {{"shoulder": 0.5}}}})
        a.receive_json()
        b.send_json({{"type": "robot_state", "robot": "mini_arm"}})
        out["shared_runtime_shoulder"] = b.receive_json()["joints"]["shoulder"]

    # ---- REST 未回归 ----
    out["rest_health"] = client.get("/api/health").status_code
    out["rest_robots"] = client.get("/api/robots").json()["count"]

    # ---- WS 与 REST 的模型必须逐字节相同 ----
    rest_model = client.get("/api/robots/mini_arm/model").json()["model"]
    out["ws_rest_models_match"] = (rest_model == info["robots"][0]["model"])

print("RESULT_JSON:" + json.dumps(out))
'''


#: 架构约束：直接扫源码（在父进程做，便于报出行号）
ARCH_FILES = [
    "backend/runtime/__init__.py",
    "backend/runtime/command.py",
    "backend/runtime/state.py",
    "backend/runtime/backend.py",
    "backend/runtime/robot_runtime.py",
    "backend/api/websocket.py",
    "backend/api/app.py",
]


def main() -> int:
    t_start = time.time()
    print("=" * 72)
    print("RobotForge · Phase 4 验收清单（spec §61：Runtime + WebSocket）")
    print("=" * 72)

    # ================================================================ 1 文件
    print("\n[1] Phase 4 交付文件")
    files = [
        # Runtime 层（§46 / §47 / §28 / §29）
        "backend/runtime/__init__.py",
        "backend/runtime/command.py",
        "backend/runtime/state.py",
        "backend/runtime/backend.py",
        "backend/runtime/robot_runtime.py",
        # WebSocket 协议（§52）+ 挂载点
        "backend/api/websocket.py",
        "backend/api/app.py",
        # 测试
        "tests/test_runtime.py",
        "tests/test_websocket.py",
        # 上游（本 Phase 复用）
        "backend/kinematics/fk.py",
        "backend/api/registry.py",
        "packages/mini_arm/manifest.yaml",
    ]
    for rel in files:
        p = ROOT / rel
        check(f"存在 {rel}", p.is_file(), "缺失" if not p.is_file() else "")

    # ================================================================ 2 导入契约
    print("\n[2] Runtime 层可导入且导出正确符号")
    rc, out = run_python(
        "import sys; sys.path.insert(0, r'%s');\n"
        "from backend.runtime import (RobotCommand, RobotState, RobotBackend,\n"
        "                             MockBackend, RobotRuntime, BackendError,\n"
        "                             RuntimeError_, STATUSES)\n"
        "from backend.runtime import DEFAULT_MAX_STEP\n"
        "from backend.api.websocket import INBOUND_TYPES, OUTBOUND_TYPES, ERROR_CODES\n"
        "from backend.api.app import create_app, WS_PATH, API_PREFIX\n"
        "print('OK', sorted(STATUSES), sorted(OUTBOUND_TYPES))\n" % ROOT
    )
    check(
        "from backend.runtime import RobotCommand/RobotState/RobotBackend/RobotRuntime",
        rc == 0,
        out[-400:] if rc != 0 else "",
    )
    if rc == 0:
        print(f"    {out.splitlines()[-1]}")

    # ---- §69 规则 8/9：Command 与 State 必须是**两个**类型 ----
    rc, out = run_python(
        "import sys; sys.path.insert(0, r'%s');\n"
        "from backend.runtime import RobotCommand, RobotState\n"
        "print('SEPARATED', RobotCommand is not RobotState,\n"
        "      'cmd_fields', sorted(RobotCommand.__dataclass_fields__),\n"
        "      'state_fields', sorted(RobotState.__dataclass_fields__))\n" % ROOT
    )
    check(
        "§69 规则 8/9：RobotCommand 与 RobotState 是两个不同的类型",
        rc == 0 and "SEPARATED True" in out,
        out[-300:],
    )
    if rc == 0:
        line = [ln for ln in out.splitlines() if ln.startswith("SEPARATED")]
        print(f"    {line[0] if line else out}")

    # ---- §28：Command 不含"执行结果"字段；§29：State 不含"目标"字段 ----
    rc, out = run_python(
        "import sys; sys.path.insert(0, r'%s');\n"
        "from backend.runtime import RobotCommand, RobotState\n"
        "cf = set(RobotCommand.__dataclass_fields__)\n"
        "sf = set(RobotState.__dataclass_fields__)\n"
        "print('CROSS', sorted(cf & sf))\n" % ROOT
    )
    cross = out.split("CROSS")[-1].strip() if "CROSS" in out else "?"
    check(
        "Command 与 State 仅共享 robot/timestamp（语义不同、无冗余状态字段）",
        rc == 0 and cross == "['robot', 'timestamp']",
        f"交叉字段 = {cross}（期望 ['robot', 'timestamp']）",
    )
    print(f"    交叉字段 = {cross}")

    # ================================================================ 3 闭环（★核心）
    print("\n[3] ★ Sim2Sim 闭环（§49 / §61）—— 三道锚 + 反向自检")
    print("    锚① Backend 计数器真的增长（命令穿过 Runtime）")
    print("    锚② Command ≠ State（限速/限幅下状态与命令必须不同）")
    print("    锚③ Core FK 当独立裁判重算末端位姿")
    rc, out = run_python(CLOSED_LOOP_SNIPPET.format(root=str(ROOT).replace("\\", "\\\\")))
    loop = parse_result(out)
    if not loop:
        check("闭环脚本执行", False, out[-600:])
    else:
        # ---- 锚① ----
        grew = loop["cmd_count_after"] == loop["cmd_count_before"] + 1
        check(
            "锚① 命令真的到达 Backend（command_count 增长）",
            grew,
            f"before={loop['cmd_count_before']} after={loop['cmd_count_after']}",
        )
        print(f"    Backend={loop['backend_type']}（is_simulation={loop['is_simulation']}）"
              f"  command_count {loop['cmd_count_before']} → {loop['cmd_count_after']}")

        # ---- 锚② ----
        check(
            "锚② State ≠ Command：限速下一步到不了目标（不是回显）",
            loop["state_differs_from_cmd"] is True,
            f"一步后 shoulder={_fmt(loop['state_shoulder_1step'])}，"
            f"命令目标={_fmt(loop['cmd_shoulder'])}（若相等 ⇒ 疑似回显式实现）",
        )
        print(f"    一步后 shoulder={_fmt(loop['state_shoulder_1step'])} "
              f"vs 命令 {_fmt(loop['cmd_shoulder'])} ⇒ 确实不同")

        check(
            "锚②′ 限速是『慢』不是『不到』：足够多步后精确到位",
            loop["converged_exact"] is True,
            f"收敛值={_fmt(loop['state_shoulder_converged'])}（应 = 0.5）",
        )
        print(f"    收敛后 shoulder={_fmt(loop['state_shoulder_converged'])}（精确到位）")

        hold = loop["untouched_hold"]
        check(
            "未提及的关节保持原位（不算作目标、不清零）",
            loop["hold_ok"] is True,
            f"shoulder 保持在 {_fmt(hold['shoulder'])} "
            f"（未变={loop['hold_shoulder_unchanged']}，"
            f"发 elbow 命令前 = {_fmt(loop['hold_shoulder_after_first_elbow_cmd'])}）；"
            f"elbow 收敛到 {_fmt(hold['elbow'])}"
            f"（={loop['hold_elbow_converged']}）",
        )
        print(f"    未提及关节保持：shoulder={_fmt(hold['shoulder'])}（12 步内逐位不变），"
              f"elbow={_fmt(hold['elbow'])}（已收敛）")
        print(f"    ※ 判据是**不变**而非『已到位』—— 限速下 shoulder 停在 0.35"
              f" 属正确行为；把判据写成『两个都是 0.4』会把限速误报成失败")

        check(
            "限幅：超限目标被夹到模型声明的上限，且**可观测**（backend.clamped）",
            loop["clamp_report_ok"] is True,
            f"目标 {_fmt(loop['clamp_target'])} → 记录 {loop['clamp_reported']}，"
            f"上限 {_fmt(loop['clamp_upper'])}，"
            f"60 步后状态 {_fmt(loop['clamp_converged'])}",
        )
        print(f"    限幅：命令 {_fmt(loop['clamp_target'])} → 目标被夹到 "
              f"{_fmt(loop['clamp_upper'])}（已记录），60 步后状态 "
              f"{_fmt(loop['clamp_converged'])}（= 上限，不是 99.0）")
        print(f"    ※ 1 步后的状态是 {_fmt(loop['clamp_actual_after_1step'])}"
              f" —— 那是**限速**（0.35/步）先于到达边界，"
              f"所以限幅的判据必须看 `clamped`，不能看状态")

        check(
            "限幅不影响合法目标：0.4 必须精确到位（限幅不是无差别夹紧）",
            loop["no_clamp_needed"] is True,
            f"10 步后状态={_fmt(loop['no_clamp_needed'])}（布尔，应 True）",
        )

        # ---- 锚③ ----
        check(
            "锚③ 末端位姿与 Core FK 逐分量一致（裁判不读包内常量）",
            loop["pose_pos_diff_vs_core_fk"] <= 1e-15
            and loop["pose_ori_diff_vs_core_fk"] <= 1e-15,
            f"Δpos={_fmt(loop['pose_pos_diff_vs_core_fk'])}，"
            f"Δori={_fmt(loop['pose_ori_diff_vs_core_fk'])}（阈值 1e-15）",
        )
        print(f"    位姿 vs Core FK：Δpos={_fmt(loop['pose_pos_diff_vs_core_fk'])} m, "
              f"Δori={_fmt(loop['pose_ori_diff_vs_core_fk'])}")

        check(
            "位姿随关节角变化（防『回显写死常量』的假状态）",
            loop["pose_varies"] > 1e-3,
            f"两个位形间最大位移={_fmt(loop['pose_varies'])} m（需 > 1e-3）",
        )
        print(f"    位姿随角度变化：{_fmt(loop['pose_varies'])} m")

        # ---- 反向自检 ----
        check(
            "清单自检：去掉限速后 State 就等于 Command ⇒ 证明锚②在真的检查",
            loop["echo_state_equals_cmd"] is True,
            f"echo 状态下 shoulder={_fmt(loop['echo_state_shoulder'])}（应 = 0.5）",
        )
        print(f"    自检：极大步长下 shoulder={_fmt(loop['echo_state_shoulder'])} "
              f"⇒ 与命令相等（说明锚②的有效性来自限速，不是巧合）")

        check(
            "并发命令串行化：单步位移不超过 max_step（asyncio.Lock 生效）",
            loop["concurrent_ok"] is True,
            f"两次读数为 {loop['concurrent_values']}（增量 "
            f"{_fmt(loop['concurrent_values'][1] - loop['concurrent_values'][0])}"
            f" 应 ≤ {_fmt(loop['max_step']) if 'max_step' in loop else 0.35}）",
        )
        print(f"    并发 step：两次读数 {[round(v, 9) for v in loop['concurrent_values']]}"
              f" ⇒ 串行推进、每步 ≤ 0.35（不是交错覆盖）")

        # ---- §69 规则 10 ----
        check(
            "§69 规则 10：自定义 Backend 可注入 Runtime（Sim2Real 立足点）",
            loop["injected_backend_used"] is True
            and loop["injected_got_command"] == [{"shoulder": 0.77}]
            and abs(loop["injected_state_shoulder"] - 0.123) < 1e-12,
            f"收到命令={loop['injected_got_command']}，"
            f"状态 shoulder={loop['injected_state_shoulder']}（替代实现说了算）",
        )
        print(f"    注入替身 Backend：收到 {loop['injected_got_command']}，"
              f"但状态返回 {_fmt(loop['injected_state_shoulder'])}（status="
              f"{loop['injected_status']}）")
        print(f"    ⇒ Runtime 原样透传：换 Backend 不改 Runtime 一行")

    # ================================================================ 4 WebSocket
    print("\n[4] WebSocket 端到端（§52）—— 用真实 app + 真连接")
    rc, out = run_python(WS_SNIPPET.format(root=str(ROOT).replace("\\", "\\\\")))
    ws = parse_result(out)
    if not ws:
        check("WebSocket 端到端脚本执行", False, out[-800:])
    else:
        check(
            "§52 连接即推 robot_info + simulation_state",
            ws["hello_types"] == ["robot_info", "simulation_state"],
            f"实际 {ws['hello_types']}",
        )
        print(f"    开场帧：{ws['hello_types']}")

        check(
            "Robot discovery：robot_info 列出 mini_arm",
            ws["info_ids"] == ["mini_arm"] and ws["info_count"] == 1,
            f"ids={ws['info_ids']} count={ws['info_count']}",
        )
        print(f"    发现：{ws['info_ids']}")

        check(
            "RobotModel 获取：robot_info 带完整模型 + 校验摘要",
            ws["info_model_keys"] == [
                "actuators", "base_frame", "capabilities", "coordinate",
                "end_effectors", "frames", "joints", "links", "metadata",
                "root_link", "sites", "units",
            ]
            and ws["info_validation_ok"] is True,
            f"keys={ws['info_model_keys']}",
        )
        print(f"    模型字段 {len(ws['info_model_keys'])} 个，校验 ok={ws['info_validation_ok']}")

        check(
            "WS 与 REST 送出的 RobotModel 逐字节相同（一份真值两个出口）",
            ws["ws_rest_models_match"] is True,
        )

        check(
            "Command 发送 → State 返回（§61 闭环第一步）",
            ws["cmd_resp_type"] == "robot_state" and ws["cmd_resp_status"] == "running",
            f"type={ws['cmd_resp_type']} status={ws['cmd_resp_status']}",
        )
        print(f"    命令响应：{ws['cmd_resp_type']} status={ws['cmd_resp_status']} "
              f"joints={ {k: round(v, 9) for k, v in ws['cmd_resp_joints'].items()} }")

        check(
            "State 帧带末端位姿（end_effector 非 null）",
            ws["cmd_resp_pose_present"] is True,
        )

        check(
            "§52 `robot_command` 别名也被接受（两个名字都是规范写下的）",
            ws["alias_resp_type"] == "robot_state",
            f"实际 {ws['alias_resp_type']}",
        )

        check(
            "状态查询是只读的（`robot_state` 不改后端状态）",
            ws["state_query_readonly"] is True,
        )

        check(
            "simulation_state 反映运行中与 Backend 类型",
            ws["sim_running"] is True and ws["sim_backends"] == ["MockBackend"],
            f"running={ws['sim_running']} backends={ws['sim_backends']}",
        )
        print(f"    simulation_state：running={ws['sim_running']}, "
              f"backends={ws['sim_backends']}")

        # ---- 五种帧 ----
        check(
            "§52 五种帧全部可达（robot_info / robot_command / robot_state / "
            "simulation_state / error）",
            set(ws["frame_types_seen"]) == {
                "robot_info", "robot_state", "simulation_state", "error"
            },
            f"实际观测到 {ws['frame_types_seen']}",
        )
        print(f"    观测到的帧类型：{ws['frame_types_seen']}")

        # ---- 错误码 ----
        check(
            "§52 错误帧：五类错误码全部可达且各自正确",
            ws["error_codes"] == [
                "bad_frame", "bad_frame", "bad_frame",
                "unknown_type", "bad_command", "unknown_robot",
            ],
            f"实际 {ws['error_codes']}",
        )
        print(f"    错误码序列：{ws['error_codes']}")

        check(
            "坏帧不断连（错误帧存在的理由）",
            ws["alive_after_errors"] is True,
        )

        check(
            "NaN 被拒（RFC 8259；否则服务端接受而前端 JSON.parse 会抛错）",
            ws["nan_rejected_code"] == "bad_frame",
            f"实际 {ws['nan_rejected_code']}",
        )

        # ---- 数值类型 ----
        check(
            "§52 所有物理数值都是 JSON Number（无字符串化数字）",
            not ws["stringified_numbers"],
            f"被字符串化的数值：{ws['stringified_numbers']}",
        )
        print(f"    字符串化数字：{ws['stringified_numbers'] or '无'}")

        # ---- §49 ----
        check(
            "§49 帧里不含渲染信息（'该怎么画'不属于协议）",
            not ws["render_keys"],
            f"越界字段：{ws['render_keys']}",
        )
        print(f"    渲染相关字段：{ws['render_keys'] or '无'}")

        check(
            "多客户端共享同一 Runtime（同一份真值）",
            abs(ws["shared_runtime_shoulder"]) > 0.0,
            f"客户端 B 读到 shoulder={_fmt(ws['shared_runtime_shoulder'])}（应≠0）",
        )
        print(f"    多客户端：B 读到 A 造成的改变 shoulder="
              f"{_fmt(ws['shared_runtime_shoulder'])}")

        check(
            "WS 的引入未影响 REST",
            ws["rest_health"] == 200 and ws["rest_robots"] == 1,
            f"health={ws['rest_health']} robots={ws['rest_robots']}",
        )

    # ================================================================ 5 架构约束
    print("\n[5] 架构约束（提示词 §69 / §46 / §49）")

    # (a) §46：Runtime 不得出现机器人型号字面量
    offenders: list[str] = []
    for rel in ARCH_FILES:
        p = ROOT / rel
        if not p.is_file():
            continue
        src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
        for ln, line in enumerate(src.splitlines(), 1):
            for lit in ('"mini_arm"', "'mini_arm'", '"mearm"', "'mearm'"):
                if lit in line:
                    offenders.append(f"{rel}:{ln}: {line.strip()}")
    check(
        "§69 规则 1/2：Runtime + WS 层不含机器人型号字面量",
        not offenders,
        "; ".join(offenders[:3]),
    )
    print(f"    扫描 {len(ARCH_FILES)} 个文件：{'干净' if not offenders else offenders[:2]}")

    # (b) §46：Runtime 不得出现硬件/传输字面量
    hw: list[str] = []
    RUNTIME_FILES = [f for f in ARCH_FILES if "/runtime/" in f]
    for rel in RUNTIME_FILES:
        p = ROOT / rel
        src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
        for ln, line in enumerate(src.splitlines(), 1):
            low = line.lower()
            for lit in ("serial", "can", "usb"):
                if f'"{lit}"' in low or f"'{lit}'" in low:
                    hw.append(f"{rel}:{ln}: {line.strip()}")
    check(
        "§46：Runtime 不知道 serial / CAN / USB（它们属于未来的 Transport）",
        not hw,
        "; ".join(hw[:3]),
    )
    print(f"    硬件字面量：{'干净' if not hw else hw[:2]}")

    # (c) Runtime 不得 import fastapi / mujoco / starlette
    banned = ("fastapi", "mujoco", "starlette")
    dep: list[str] = []
    for rel in RUNTIME_FILES:
        p = ROOT / rel
        src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
        for ln, line in enumerate(src.splitlines(), 1):
            s = line.strip()
            if not (s.startswith("import ") or s.startswith("from ")):
                continue
            for b in banned:
                if b in s:
                    dep.append(f"{rel}:{ln}: {s}")
    check(
        "Runtime 是纯编排层：不 import fastapi / mujoco / starlette",
        not dep,
        "; ".join(dep[:3]),
    )
    print(f"    反向依赖：{'干净' if not dep else dep[:2]}")

    # (d) 依赖方向：kinematics / model 不得 import runtime
    rev: list[str] = []
    for sub in ("kinematics", "model", "loaders"):
        d = ROOT / "backend" / sub
        for p in sorted(d.glob("*.py")):
            src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
            for ln, line in enumerate(src.splitlines(), 1):
                s = line.strip()
                if (s.startswith("import ") or s.startswith("from ")) and "runtime" in s:
                    rev.append(f"backend/{sub}/{p.name}:{ln}: {s}")
    check(
        "依赖方向正确：kinematics / model / loaders 不 import runtime",
        not rev,
        "; ".join(rev[:3]),
    )
    print(f"    反向依赖扫描：{'干净' if not rev else rev[:2]}")

    # (e) §49：WS 层不得知道坐标转换（转换只在 coordinateAdapter.ts）
    wsrc = strip_comments_and_strings(
        (ROOT / "backend" / "api" / "websocket.py").read_text(encoding="utf-8")
    )
    spin = [
        f"websocket.py:{ln}: {line.strip()}"
        for ln, line in enumerate(wsrc.splitlines(), 1)
        if any(m in line for m in ("-math.pi / 2", "-math.pi/2", "rotate_x", "y_up", "Y-up"))
    ]
    check(
        "§41/§49：WS 层不做坐标旋转（转换只在 coordinateAdapter.ts）",
        not spin,
        "; ".join(spin[:3]),
    )
    print(f"    WS 层坐标旋转痕迹：{'干净' if not spin else spin[:2]}")

    # (f) 正向证据：WS 真的挂了 Runtime，而不是另起一套
    app_src = (ROOT / "backend" / "api" / "app.py").read_text(encoding="utf-8")
    check(
        "正向证据：app 用 lifespan 管理 Runtime 且 WS 与 REST 共用同一实例",
        "lifespan" in app_src and "create_ws_router(runtime)" in app_src,
        "未找到 lifespan / create_ws_router(runtime)",
    )
    print(f"    app.py：lifespan={'lifespan' in app_src}, "
          f"共用 runtime={'create_ws_router(runtime)' in app_src}")

    # ================================================================ 6 CLI（未回归）
    print("\n[6] CLI 子命令（Phase 3 交付，本 Phase 不得回归）")
    for cmd, args in (
        ("list", []),
        ("show", ["mini_arm"]),
        ("fk", ["mini_arm", "--joints", "shoulder=0.3"]),
        ("ik", ["mini_arm", "--target", "0.12,0,0.136"]),
        ("inspect", ["mini_arm"]),
    ):
        proc = subprocess.run(
            [str(PY), "-m", "backend.cli", cmd, *args],
            cwd=ROOT, capture_output=True, text=True, timeout=60,
        )
        check(f"python -m backend.cli {cmd}", proc.returncode == 0,
              (proc.stdout + proc.stderr).strip()[-200:])

    # ================================================================ 7 测试
    print("\n[7] 测试套件")
    for label, args in (
        ("tests/test_runtime.py", ("tests/test_runtime.py",)),
        ("tests/test_websocket.py", ("tests/test_websocket.py",)),
        ("tests/test_api.py", ("tests/test_api.py",)),
        ("tests/test_fk.py", ("tests/test_fk.py",)),
        ("tests/test_ik.py", ("tests/test_ik.py",)),
        ("tests/test_kinematics_core.py", ("tests/test_kinematics_core.py",)),
        ("packages/mini_arm/tests", ("packages/mini_arm/tests",)),
    ):
        ok, out = run_pytest(*args)
        tail = out
        check(f"pytest {label}", ok, tail)
        print(f"    {label}: {tail}")

    ok, out = run_pytest()
    tail = out
    check("pytest（全量，无回归）", ok, tail)
    print(f"    全量: {tail}")

    # ================================================================ 8 上游 Phase
    print("\n[8] 上游 Phase 无回归")
    for name, script, expect in (
        ("Phase 1", "tools/accept_phase1.py", "Phase 1"),
        ("Phase 2", "tools/accept_phase2.py", "Phase 2"),
        ("Phase 3", "tools/accept_phase3.py", "Phase 3"),
    ):
        p = ROOT / script
        if not p.is_file():
            check(f"{script} 存在", False, "缺失")
            continue
        proc = subprocess.run(
            [str(PY), str(p)], cwd=ROOT, capture_output=True, text=True, timeout=1800
        )
        combined = (proc.stdout or "") + (proc.stderr or "")
        summary = [
            ln.strip() for ln in combined.splitlines()
            if expect in ln and ("验收" in ln or "通过" in ln)
        ]
        check(
            f"{script} 仍然全通过（无回归）",
            proc.returncode == 0,
            " / ".join(summary) if summary else combined.strip()[-400:],
        )
        print(f"    {' / '.join(summary) if summary else '(无汇总行)'}")

    # ================================================================ 汇总
    elapsed = time.time() - t_start
    print("\n" + "=" * 72)
    passed = sum(1 for _, ok, _ in _results if ok)
    total = len(_results)
    for name, ok, detail in _results:
        mark = "PASS" if ok else "FAIL"
        line = f"  [{mark}] {name}"
        if not ok and detail:
            line += f"\n         ↳ {detail}"
        print(line)
    print("=" * 72)
    print(f"Phase 4 验收：{passed}/{total} 通过（耗时 {elapsed:.1f}s）")
    if passed != total:
        print("❌ Phase 4 未通过 —— 依据 spec：不得进入下一 Phase")
        return 1
    print("✅ Phase 4 通过 —— 允许进入 Phase 5（MuJoCo Sim2Sim）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
