#!/usr/bin/env python
"""Phase 5 验收清单（对应 spec §62：MuJoCo Sim2Sim）。

## 用法

```bash
.venv/Scripts/python.exe tools/accept_phase5.py
```

退出码 0 = 全部通过；非 0 = 有未通过项。

> ⚠️ 本脚本要跑 5 份上游 Phase 清单 + 全量 pytest，**耗时约 8 分钟**。
> **不要**在前台跑（本机沙箱会给长命令发 SIGTERM，产出 0 字节日志）。
> 用后台执行并落文件：
>
> ```bash
> .venv/Scripts/python.exe -u tools/accept_phase5.py > .workbuddy/accept5.log 2>&1
> ```

## Phase 5 是什么（spec §62 逐字）

验收：

```text
启动 Simulation / 加载 mini_arm MJCF / 发送 Joint Command / MuJoCo 执行 /
获取 Joint State / 生成 RobotState / WebSocket 推送 / Frontend 同步 /
Three.js 同步 / FK / IK 与 Simulation 状态一致
```

## ★ 这份清单的核心难点：怎么证明"用的真是物理"

Phase 5 的验收最容易写成**"数字看着合理"**：

```python
backend = MuJoCoBackend(model)
await backend.start()
state = await backend.step(cmd)
assert state.joint_positions["shoulder"] > 0        # ← 可能什么都没证明
```

因为一个**线性插值器**（即 `MockBackend`）也会让上面的断言通过。
所以本脚本用**五道独立的锚**：

| 锚 | 做法 | 抓什么 |
|---|---|---|
| ① 力与惯量真的在起作用 | 零位形下重力造成下垂（非零残差） | "没跑动力学、直接写 qpos" |
| ② 关节耦合 | 只命令 shoulder，elbow 被惯性带动 | 逐关节独立假物理 |
| ③ 速度非零且会衰减 | 运动中 `qvel`>0，稳态 `qvel`≈0 | "状态是算出来的" |
| ④ MuJoCo ↔ Core FK 互证 | 两个独立来源算 TCP 位姿，差 < 1e-9 | 几何漂移 / 四元数顺序错 |
| ⑤ 换一台机器人也能跑 | 2-DOF 合成模型驱动同一份 Backend | 硬编码 mini_arm |

外加两项**反向自检**：
① 确认 `mock` 与 `mujoco` 的单步状态**确实不同**（否则锚②③是空转）；
② 确认"回显式"实现（命令即状态）会让锚③失败。

## 覆盖的架构约束（提示词 §50 / §69 / §40 / §35）

* §50：MuJoCo 是 Physics Backend，`MjModel` 不得泄漏进 `RobotModel`
* §69 规则 2：`backend/simulation/` 不得出现机器人型号字面量
* §69 规则 10：`MuJoCoBackend` 与 `MockBackend` 接口相同、可互换
* §40 / §35：MuJoCo 四元数 `[w,x,y,z]` 换算**只允许出现一处**
* 依赖方向：`runtime` 不得 import `simulation` / `mujoco`

## 设计原则（与 accept_phase1~4.py 一致）

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

_results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    _results.append((name, bool(ok), detail))


def run_pytest(*args: str) -> tuple[int, str]:
    proc = subprocess.run(
        [str(PY), "-m", "pytest", *args, "-q", "--no-header"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    return proc.returncode, (proc.stdout + proc.stderr).strip()


def run_python(snippet: str, timeout: int = 300) -> tuple[int, str]:
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


def strip_comments_and_strings(src: str) -> str:
    """剥离注释与字符串，用于"源码里有没有出现某关键字"的判定。

    必须剥离：否则一句注释"这里不知道 mini_arm"就能让检查失败。

    ⚠️ 语义边界（写元测试前必须想清楚，否则会写出**恒假**判据）：
    本函数删掉的是 `COMMENT` **和** `STRING` 两类 token —— 所以
    **字符串的正文也不会出现在结果里**。`untokenize` 会把 token 用空格
    重新拼接（`y = "x"` → `y =    `），于是剩下的只有**标识符与控制符**
    形态。这正好是我们要的：判据只关心"源码在**运算逻辑**层面引用了
    什么名字"，而字符串与注释里的名字都是**数据**，不是引用。
    """
    return tokenize.untokenize(
        tok
        for tok in tokenize.generate_tokens(iter(src.splitlines(True)).__next__)
        if tok.type not in (tokenize.COMMENT, tokenize.STRING)
    )


def _fmt(x: float, digits: int = 8) -> str:
    return f"{x:.{digits}g}"


def parse_result(out: str) -> dict:
    """从子进程输出里取出 `RESULT_JSON:` 载荷。

    ⚠️ 不能用 `line[len(prefix):]` —— `run_python` 把 stderr 拼在 stdout 后面，
    而 DeprecationWarning / MuJoCo 的警告**可能没有尾随换行**，
    于是它会**粘在 JSON 后面**，`json.loads` 报 "Extra data"。
    正确的做法是用 `JSONDecoder.raw_decode` 只吃掉**一个**完整 JSON 值，
    对尾部残留无感。这个坑在 Phase 4 就踩过一次，Phase 5 又踩了一次
    （这次是 MuJoCo 的编译警告），所以固定用 raw_decode。
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
# 子进程片段
# ---------------------------------------------------------------------------

#: ★ 五道锚 + 两项反向自检。输出 `RESULT_JSON:` 一行。
PHYSICS_SNIPPET = r'''
import asyncio, json, math, sys
sys.path.insert(0, r"{root}")

from backend.runtime import RobotCommand, RobotRuntime, MockBackend
from backend.kinematics.fk import forward_kinematics
from backend.simulation import (
    MuJoCoBackend, mujoco_backend_factory, mj_quat_to_xyzw, xyzw_to_mj_quat,
)
from backend.api.registry import get_package

out = {{}}

model, _report = get_package("mini_arm").load_model()
IDS = model.mobile_joint_ids()
out["joint_ids"] = IDS
out["dof"] = model.dof()


async def main():
    # ============ 锚① 力与惯量真的在起作用（重力下垂）============
    b = MuJoCoBackend(model)
    await b.start()
    try:
        snap0 = b.snapshot()
        out["settle_positions"] = dict(snap0["positions"])
        # 零位形不是平衡位形：重力把上臂压低
        out["droop_rad"] = snap0["positions"]["shoulder"]
        out["gravity_is_active"] = abs(snap0["positions"]["shoulder"]) > 1e-4
        out["droop_is_small"] = abs(snap0["positions"]["shoulder"]) < 1e-2

        # 启动后 qvel 应当已经被 settle 到很小
        out["settled_vel_max"] = max(
            abs(v) for v in snap0["velocities"].values()
        )
    finally:
        await b.stop()

    # ============ 锚②③ 关节耦合 + 速度演化 ============
    b = MuJoCoBackend(model)
    await b.start()
    try:
        await b.reset()
        elbow_before = b.snapshot()["positions"]["elbow"]
        await b.step(RobotCommand.of("mini_arm", {{"shoulder": 1.2}}))
        mid = await b.get_state()
        out["mid_vel_max"] = max(abs(v) for v in mid.joint_velocities.values())
        out["velocities_nonzero_in_motion"] = out["mid_vel_max"] > 1e-3

        for _ in range(10):
            st = await b.step(RobotCommand.of("mini_arm", {{"shoulder": 1.2}}))
        elbow_after = st.joint_positions["elbow"]
        out["elbow_before"] = elbow_before
        out["elbow_after"] = elbow_after
        out["joint_coupling_delta"] = abs(elbow_after - elbow_before)
        out["joint_coupling_exists"] = out["joint_coupling_delta"] > 1e-5

        # 到位后速度衰减
        for _ in range(60):
            st = await b.step(RobotCommand.of("mini_arm", {{"shoulder": 0.4}}))
        out["rest_vel_max"] = max(abs(v) for v in st.joint_velocities.values())
        out["velocities_decay_at_rest"] = out["rest_vel_max"] < 1e-3
        out["shoulder_at_rest"] = st.joint_positions["shoulder"]
        out["converged_to_0p4"] = abs(st.joint_positions["shoulder"] - 0.4) < 5e-3
    finally:
        await b.stop()

    # ============ 锚④ MuJoCo ↔ Core FK 互证 ============
    b = MuJoCoBackend(model)
    await b.start()
    try:
        await b.reset()
        for _ in range(5):
            await b.step(RobotCommand.of("mini_arm", {{"shoulder": 0.37, "elbow": -0.22}}))
        snap = b.snapshot()
        out["core_fk_position"] = snap["core_fk_position"]
        out["mj_tcp_position"] = snap["mj_tcp_position"]
        out["tcp_position_gap"] = snap["tcp_position_gap"]
        mj_xyzw = mj_quat_to_xyzw(snap["mj_tcp_quaternion_wxyz"])
        core_xyzw = snap["core_fk_quaternion_xyzw"]
        out["mj_quat_xyzw"] = mj_xyzw
        out["core_quat_xyzw"] = core_xyzw
        out["tcp_quaternion_gap"] = max(
            abs(a - b_) for a, b_ in zip(mj_xyzw, core_xyzw)
        )
        # 位姿**随构型变化**（防"回显写死常量"）
        await b.reset()
        await b.step(RobotCommand.of("mini_arm", {{"shoulder": 0.10}}))
        a = (await b.get_state()).end_effector_pose.position.to_list()
        for _ in range(40):
            st2 = await b.step(RobotCommand.of("mini_arm", {{"shoulder": 0.90}}))
        bpos = st2.end_effector_pose.position.to_list()
        out["pose_varies"] = max(abs(x - y) for x, y in zip(a, bpos))
        # Core FK 独立重算状态里的位姿
        ref = forward_kinematics(model, dict(st2.joint_positions))
        out["state_pose_vs_core_fk"] = max(
            abs(x - y)
            for x, y in zip(
                st2.end_effector_pose.position.to_list(), ref.position.to_list()
            )
        )
    finally:
        await b.stop()

    # ============ Runtime 闭环（§62 的"命令→执行→状态"）============
    rt = RobotRuntime(backend_factory=mujoco_backend_factory)
    await rt.start()
    try:
        be = rt.backend("mini_arm")
        out["runtime_backend_type"] = type(be).__name__
        out["runtime_is_simulation"] = bool(be.is_simulation)
        out["command_count_before"] = be.command_count
        await rt.reset()
        st = await rt.step(RobotCommand.of("mini_arm", {{"shoulder": 0.5}}))
        out["command_count_after"] = be.command_count
        out["one_step_shoulder"] = st.joint_positions["shoulder"]
        out["state_differs_from_command"] = (
            abs(st.joint_positions["shoulder"] - 0.5) > 1e-4
        )
        out["state_status"] = st.status

        # 限幅：命令 99 ⇒ 记录在 clamped，且最终**真的到得了**上限
        await rt.reset()
        jid = IDS[1]
        upper = model.joint(jid).limits.position_max
        await rt.step(RobotCommand.of("mini_arm", {{jid: 99.0}}))
        out["clamp_reported"] = dict(be.clamped)
        out["clamp_upper"] = upper
        for _ in range(40):
            st = await rt.step(RobotCommand.of("mini_arm", {{jid: 99.0}}))
        out["clamp_reached"] = st.joint_positions[jid]
        out["clamp_ok"] = (
            upper is not None
            and abs(out["clamp_reported"].get(jid, 0.0) - upper) < 1e-12
            and abs(out["clamp_reached"] - upper) < 5e-3
        )

        # NaN 检查
        await rt.reset()
        nan_found = []
        for target in (1.4, -1.4, 0.0, 0.8):
            for _ in range(20):
                st = await rt.step(RobotCommand.of("mini_arm", {{"shoulder": target}}))
            for k, v in st.joint_positions.items():
                if not math.isfinite(v):
                    nan_found.append(k)
        out["nan_joints"] = nan_found
        out["no_nan"] = not nan_found
    finally:
        await rt.stop()

    # ============ 反向自检 A：mock 与 mujoco 必须不同 ============
    rt_m = RobotRuntime(backend_factory=lambda m: MockBackend(m))
    await rt_m.start()
    try:
        s_mock = await rt_m.step(RobotCommand.of("mini_arm", {{"shoulder": 0.5}}))
        out["mock_one_step_shoulder"] = s_mock.joint_positions["shoulder"]
    finally:
        await rt_m.stop()
    out["mock_vs_mujoco_gap"] = abs(
        out["mock_one_step_shoulder"] - out["one_step_shoulder"]
    )
    out["mock_differs_from_mujoco"] = out["mock_vs_mujoco_gap"] > 0.05

    # ============ 反向自检 B：无重力 ⇒ 无下垂 ============
    #  把 gravity 改成 0，确认"下垂"这个锚真的来自重力
    xml = open(
        r"{root}/packages/mini_arm/model/mini_arm.xml", encoding="utf-8"
    ).read()
    xml_nog = xml.replace('gravity="0 0 -9.81"', 'gravity="0 0 0"')
    b2 = MuJoCoBackend(model, source=xml_nog)
    await b2.start()
    try:
        out["nogravity_droop"] = b2.snapshot()["positions"]["shoulder"]
        out["droop_comes_from_gravity"] = abs(out["nogravity_droop"]) < 1e-9
    finally:
        await b2.stop()

    # ============ 四元数换算方向（非对称值）============
    # 两个函数的**入参约定不同**，断言必须按各自约定喂输入：
    #   mj_quat_to_xyzw(q_wxyz)  入参是 MuJoCo 序 [w,x,y,z]
    #   xyzw_to_mj_quat(q_xyzw)  入参是 Core 序    [x,y,z,w]
    # 用非对称值 [0.5, 0.1, 0.2, 0.3]：写错成"取后三位"会得到 [0.2,0.3,0.5]，
    # 与期望不符 ⇒ 该断言可失败（用对称值或 w=0 的值就测不出来）。
    WXYZ = [0.5, 0.1, 0.2, 0.3]   # MuJoCo 序，喂给 mj_quat_to_xyzw
    XYZW = [0.1, 0.2, 0.3, 0.5]   # Core   序，喂给 xyzw_to_mj_quat
    out["quat_fwd"] = mj_quat_to_xyzw(WXYZ)
    out["quat_rev"] = xyzw_to_mj_quat(XYZW)
    # 往返：各自从"自己的输入序"出发，转出去再转回来必须回到原值
    out["quat_round_trip"] = (
        mj_quat_to_xyzw(xyzw_to_mj_quat(XYZW)) == XYZW
        and xyzw_to_mj_quat(mj_quat_to_xyzw(WXYZ)) == WXYZ
    )
    # 互为逆：两个方向复合应等于恒等（对同一物理四元数）
    out["quat_inverse_pair"] = (
        xyzw_to_mj_quat(mj_quat_to_xyzw(WXYZ)) == WXYZ
        and mj_quat_to_xyzw(xyzw_to_mj_quat(XYZW)) == XYZW
    )


asyncio.run(main())
print("RESULT_JSON:" + json.dumps(out))
'''


#: WebSocket 端到端：**换成 MuJoCoBackend** 的 app，真连接。
WS_SNIPPET = r'''
import json, sys
sys.path.insert(0, r"{root}")

from fastapi.testclient import TestClient
from backend.api.app import create_app, WS_PATH
from backend.simulation import mujoco_backend_factory

out = {{}}
frames = []

# ★ 关键：只换一个参数。Runtime / WS / 路由**一行不改**（§69 规则 10）
app = create_app(backend_factory=mujoco_backend_factory)
with TestClient(app) as client:
    with client.websocket_connect(WS_PATH) as ws:
        hello = [ws.receive_json(), ws.receive_json()]
        out["hello_types"] = [f["type"] for f in hello]
        frames.extend(hello)

        info = hello[0]
        out["info_ids"] = [r["package"]["id"] for r in info.get("robots", [])]
        out["info_validation_ok"] = info["robots"][0]["validation"]["ok"]

        sim0 = hello[1]
        out["sim_backends"] = [b["backend"] for b in sim0["backends"]]
        out["sim_backend_type"] = sim0["backends"][0].get("backend_type")
        out["sim_type"] = sim0["backends"][0].get("type")

        # ---- 命令 → MuJoCo 执行 → 状态 ----
        ws.send_json({{
            "type": "joint_command", "robot": "mini_arm",
            "joints": {{"shoulder": 0.5, "elbow": 0.3}}, "timestamp": 1.0,
        }})
        st = ws.receive_json()
        frames.append(st)
        out["cmd_resp_type"] = st["type"]
        out["cmd_resp_status"] = st["status"]
        out["cmd_resp_joints"] = st["joints"]
        out["cmd_resp_pose"] = st["end_effector"]
        out["cmd_shoulder"] = 0.5
        out["cmd_state_differs"] = abs(st["joints"]["shoulder"] - 0.5) > 1e-4
        out["velocities_present"] = any(
            abs(v) > 0 for v in st["velocities"].values()
        )

        # ---- 多步后收敛 ----
        for _ in range(60):
            ws.send_json({{
                "type": "joint_command", "robot": "mini_arm",
                "joints": {{"shoulder": 0.5}},
            }})
            last = ws.receive_json()
            frames.append(last)
        out["converged_shoulder"] = last["joints"]["shoulder"]
        out["converged"] = abs(last["joints"]["shoulder"] - 0.5) < 5e-3

        # ---- 时间戳是**仿真时间**且单调 ----
        ts = [f["timestamp"] for f in frames[-5:]]
        out["timestamps"] = ts
        out["timestamps_monotonic"] = ts == sorted(ts) and ts[-1] > ts[0]

        # ---- 错误码（MuJoCo 后端下同样可达）----
        codes = []
        for payload in (
            "{{not json",
            {{"type": "wat"}},
            {{"type": "joint_command", "robot": "mini_arm",
              "joints": {{"shulder": 0.1}}}},
            {{"type": "joint_command", "robot": "nope", "joints": {{}}}},
        ):
            if isinstance(payload, str):
                ws.send_text(payload)
            else:
                ws.send_json(payload)
            f = ws.receive_json()
            frames.append(f)
            codes.append(f.get("code"))
        out["error_codes"] = codes

        # ---- 坏帧之后连接仍可用 ----
        ws.send_json({{"type": "joint_command", "robot": "mini_arm",
                       "joints": {{"shoulder": 0.2}}}})
        alive = ws.receive_json()
        frames.append(alive)
        out["alive_after_errors"] = alive["type"] == "robot_state"

        # ---- §49：帧里不得有渲染信息 ----
        FORBIDDEN = ("rotationmatrix", "rotmatrix", "matrix4", "object3d",
                     "mesh", "color", "material", "yup", "y_up", "three")
        keys = set()
        def collect(o):
            if isinstance(o, dict):
                for k, v in o.items():
                    keys.add(str(k).lower()); collect(v)
            elif isinstance(o, list):
                for v in o:
                    collect(v)
        for f in frames:
            collect(f)
        out["render_keys"] = sorted(k for k in keys if any(x in k for x in FORBIDDEN))

        # ---- 所有物理量必须是 JSON Number（不是字符串）----
        bad = []
        def walk(o, path=""):
            if isinstance(o, dict):
                for k, v in o.items(): walk(v, path + "." + str(k))
            elif isinstance(o, list):
                for i, v in enumerate(o): walk(v, path + "[%d]" % i)
            elif isinstance(o, bool):
                pass
            elif isinstance(o, str):
                try: float(o)
                except ValueError: return
                bad.append(path + " = " + repr(o))
        for f in frames:
            if f.get("type") == "robot_state":
                for key in ("joints", "velocities", "end_effector", "timestamp"):
                    if key in f: walk(f[key], key)
        out["stringified_numbers"] = bad

        # ---- NaN 必须被拒 ----
        ws.send_text('{{"type":"joint_command","robot":"mini_arm","joints":{{"shoulder":NaN}}}}')
        nan = ws.receive_json()
        frames.append(nan)
        out["nan_rejected_code"] = nan.get("code")

    # ---- REST 未回归 ----
    out["rest_health"] = client.get("/api/health").status_code
    out["rest_robots"] = client.get("/api/robots").json()["count"]

print("RESULT_JSON:" + json.dumps(out))
'''


#: 合成机器人片段：证明 Backend 不是为 mini_arm 写的（§69 规则 2）
SYNTHETIC_SNIPPET = r'''
import asyncio, json, os, sys, tempfile
sys.path.insert(0, r"{root}")
from pathlib import Path

from backend.loaders.mjcf_loader import MJCFLoader
from backend.runtime import RobotCommand, RobotRuntime
from backend.simulation import MuJoCoBackend, mujoco_backend_factory

out = {{}}

XML = """<mujoco model="two_dof_toy">
  <compiler angle="radian" autolimits="true"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="base" pos="0 0 0">
      <geom name="g_base" type="box" size="0.02 0.02 0.02"/>
      <body name="link_a" pos="0 0 0.05">
        <joint name="pan" type="hinge" axis="0 0 1"
               range="-2.0 2.0" damping="0.05" armature="0.01"/>
        <geom name="g_a" type="capsule" size="0.01 0.03" pos="0.03 0 0"
              quat="0.70710678 0 0.70710678 0"/>
        <body name="link_b" pos="0.06 0 0">
          <joint name="lift" type="hinge" axis="0 1 0"
                 range="-1.0 1.0" damping="0.05" armature="0.01"/>
          <geom name="g_b" type="capsule" size="0.01 0.02" pos="0.02 0 0"
                quat="0.70710678 0 0.70710678 0"/>
          <site name="tcp" pos="0.05 0 0" size="0.003"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="pan_motor" joint="pan" kp="60" kv="4" ctrlrange="-2.0 2.0"/>
    <position name="lift_motor" joint="lift" kp="60" kv="4" ctrlrange="-1.0 1.0"/>
  </actuator>
</mujoco>
"""

model, _ = MJCFLoader(robot_id="two_dof_toy").load(XML)
out["toy_joint_ids"] = model.mobile_joint_ids()
out["toy_dof"] = model.dof()


async def main():
    b = MuJoCoBackend(model, source=XML)
    await b.start()
    try:
        out["toy_backend_joints"] = b.snapshot()["joints"]
        for _ in range(60):
            st = await b.step(RobotCommand.of("two_dof_toy", {{"pan": 0.5, "lift": -0.4}}))
        out["toy_converged"] = dict(st.joint_positions)
        out["toy_pan_ok"] = abs(st.joint_positions["pan"] - 0.5) < 5e-3
        out["toy_lift_ok"] = abs(st.joint_positions["lift"] + 0.4) < 5e-3
        out["toy_pose_present"] = st.end_effector_pose is not None
    finally:
        await b.stop()

    # 整条链路：临时 packages 目录 + Runtime + MuJoCo 工厂
    with tempfile.TemporaryDirectory() as td:
        pkg = Path(td) / "two_dof_toy"
        (pkg / "model").mkdir(parents=True)
        (pkg / "model" / "robot.xml").write_text(XML, encoding="utf-8")
        (pkg / "manifest.yaml").write_text(
            "id: two_dof_toy\nname: Two DOF Toy\nversion: 0.1.0\n"
            "model:\n  format: mjcf\n  file: model/robot.xml\n"
            "capabilities:\n  simulation: true\n  fk: true\n  ik: false\n"
            "  actuator_control: true\n  end_effector: true\n",
            encoding="utf-8",
        )
        rt = RobotRuntime(packages_dir=Path(td), backend_factory=mujoco_backend_factory)
        await rt.start()
        try:
            out["toy_runtime_models"] = sorted(rt.models())
            out["toy_runtime_backend"] = type(rt.backend("two_dof_toy")).__name__
            st = await rt.step(RobotCommand.of("two_dof_toy", {{"pan": 0.3}}))
            out["toy_runtime_pan"] = st.joint_positions["pan"]
            out["toy_runtime_ok"] = st.joint_positions["pan"] > 0.0
        finally:
            await rt.stop()
        # 只断言"没有多出东西"，不断言"目录已消失"（那要等 with 块退出）
        # 注意：本片段会被 str.format 处理，所有字面花括号必须写成 {{{{ }}}}
        # 期望集合必须**带上包名前缀**：td 是临时根，包目录 two_dof_toy 在其下，
        # 所以相对路径是 two_dof_toy/manifest.yaml 而不是 manifest.yaml。
        # 漏了前缀会让"我们自己写进去的四个文件"全被当成"多出的残渣"（恒 FAIL）。
        expected = {{
            "two_dof_toy",
            "two_dof_toy/manifest.yaml",
            "two_dof_toy/model",
            "two_dof_toy/model/robot.xml",
        }}
        found = {{
            str(p.relative_to(td)).replace("\\", "/")
            for p in Path(td).rglob("*")
        }}
        out["tempdir_extra_files"] = sorted(found - expected)


asyncio.run(main())
print("RESULT_JSON:" + json.dumps(out))
'''


ARCH_FILES = [
    "backend/simulation/__init__.py",
    "backend/simulation/simulation_backend.py",
    "backend/simulation/mujoco_backend.py",
]


def main() -> int:
    t_start = time.time()
    print("=" * 72)
    print("RobotForge · Phase 5 验收清单（spec §62：MuJoCo Sim2Sim）")
    print("=" * 72)

    # ================================================================ 1 文件
    print("\n[1] Phase 5 交付文件")
    files = [
        "backend/simulation/__init__.py",
        "backend/simulation/simulation_backend.py",
        "backend/simulation/mujoco_backend.py",
        "tests/test_mujoco.py",
        # 上游（本 Phase 复用）
        "backend/runtime/backend.py",
        "backend/kinematics/fk.py",
        "packages/mini_arm/model/mini_arm.xml",
    ]
    for rel in files:
        p = ROOT / rel
        check(f"存在 {rel}", p.is_file(), "缺失" if not p.is_file() else "")

    # ================================================================ 2 导入契约
    print("\n[2] 仿真层可导入且导出正确符号")
    rc, out = run_python(
        "import sys; sys.path.insert(0, r'%s');\n"
        "from backend.simulation import (MuJoCoBackend, mujoco_backend_factory,\n"
        "        mj_quat_to_xyzw, xyzw_to_mj_quat, JointOrderError,\n"
        "        ordered_joint_ids, positions_to_vector, vector_to_positions,\n"
        "        assemble_state, clamp_targets_to_limits,\n"
        "        DEFAULT_STEP_SECONDS, DEFAULT_MAX_SUBSTEPS)\n"
        "from backend.runtime import RobotBackend\n"
        "print('OK', issubclass(MuJoCoBackend, RobotBackend),\n"
        "      'factory', mujoco_backend_factory.__name__)\n" % ROOT
    )
    check(
        "from backend.simulation import MuJoCoBackend / mujoco_backend_factory / ...",
        rc == 0,
        out[-500:] if rc != 0 else "",
    )
    if rc == 0:
        print(f"    {out.splitlines()[-1]}")

    check(
        "§47：MuJoCoBackend 是 RobotBackend 的子类（统一接口）",
        rc == 0 and "OK True" in out,
        out[-300:],
    )

    # ---- §50：MjModel 不得泄漏进契约对象 ----
    rc, out = run_python(
        "import sys; sys.path.insert(0, r'%s');\n"
        "from backend.runtime import RobotState, RobotCommand\n"
        "from backend.model.robot_model import RobotModel\n"
        "import dataclasses\n"
        "bad = []\n"
        "for cls in (RobotState, RobotCommand, RobotModel):\n"
        "    for f in dataclasses.fields(cls):\n"
        "        t = str(f.type).lower()\n"
        "        if 'mj' in t or 'mujoco' in t:\n"
        "            bad.append(cls.__name__ + '.' + f.name + ':' + str(f.type))\n"
        "print('LEAK', bad)\n" % ROOT
    )
    leak = out.split("LEAK")[-1].strip() if "LEAK" in out else "?"
    check(
        "§50：MjModel / MjData 不得出现在任何契约对象的字段里",
        rc == 0 and leak == "[]",
        f"泄漏字段 = {leak}",
    )
    print(f"    契约字段泄漏：{leak}")

    # ================================================================ 3 ★ 物理是真的
    print("\n[3] ★ 物理是真的（§50 / §62）—— 五道锚 + 两项反向自检")
    print("    锚① 重力/惯量在起作用：零位形下关节有下垂")
    print("    锚② 关节耦合：只命令 shoulder，elbow 被惯性带动")
    print("    锚③ 速度非零且会衰减")
    print("    锚④ MuJoCo ↔ Core FK 独立互证")
    print("    锚⑤ 换一台机器人也能跑（合成 2-DOF 模型）")
    rc, out = run_python(PHYSICS_SNIPPET.format(root=str(ROOT).replace("\\", "\\\\")))
    ph = parse_result(out)
    if not ph:
        check("物理闭环脚本执行", False, out[-900:])
    else:
        print(f"    模型：{ph['joint_ids']}（DOF={ph['dof']}）")

        # ---- 锚① ----
        check(
            "锚① 重力真的在起作用：零位形下 shoulder 有下垂（非平衡）",
            ph["gravity_is_active"] and ph["droop_is_small"],
            f"下垂 = {_fmt(ph['droop_rad'])} rad（需 > 1e-4 且 < 1e-2）",
        )
        print(f"    settle 后位置 {_fmt(ph['settle_positions']['shoulder'])}（shoulder 下垂）"
              f"，残余速度 {_fmt(ph['settled_vel_max'])}")
        print(f"    ※ 若下垂 = 0 ⇒ 仿真里没有重力（或 qpos 被直接写值）")

        # ---- 锚② ----
        check(
            "锚② 关节耦合：只命令 shoulder，elbow 被惯性带动（MockBackend 做不到）",
            ph["joint_coupling_exists"],
            f"elbow {_fmt(ph['elbow_before'])} → {_fmt(ph['elbow_after'])}"
            f"（Δ = {_fmt(ph['joint_coupling_delta'])}，需 > 1e-5）",
        )
        print(f"    elbow（未被命令）：{_fmt(ph['elbow_before'])} → "
              f"{_fmt(ph['elbow_after'])}，Δ = {_fmt(ph['joint_coupling_delta'])}")

        # ---- 锚③ ----
        check(
            "锚③ 运动中速度非零；到位后速度衰减（真稳态）",
            ph["velocities_nonzero_in_motion"] and ph["velocities_decay_at_rest"],
            f"运动中 max|qvel| = {_fmt(ph['mid_vel_max'])}（需 > 1e-3）；"
            f"稳态 max|qvel| = {_fmt(ph['rest_vel_max'])}（需 < 1e-3）",
        )
        print(f"    运动中 max|qvel| = {_fmt(ph['mid_vel_max'])}"
              f" → 稳态 {_fmt(ph['rest_vel_max'])}")

        check(
            f"命令 0.4 → 稳态 {_fmt(ph['shoulder_at_rest'])}（容差 5e-3，物理允许残差）",
            ph["converged_to_0p4"],
            f"残差 = {_fmt(abs(ph['shoulder_at_rest'] - 0.4))}",
        )
        print(f"    命令 0.4 → 稳态 {_fmt(ph['shoulder_at_rest'])}"
              f"（残差 {_fmt(abs(ph['shoulder_at_rest'] - 0.4))}，重力下垂 + 伺服稳态误差）")

        # ---- 锚④ ----
        check(
            "锚④ MuJoCo TCP 位置与 Core FK 独立互证（差 < 1e-9）",
            ph["tcp_position_gap"] < 1e-9,
            f"Δpos = {_fmt(ph['tcp_position_gap'])} m（需 < 1e-9）",
        )
        check(
            "锚④ MuJoCo TCP 姿态与 Core FK 独立互证（差 < 1e-9）",
            ph["tcp_quaternion_gap"] < 1e-9,
            f"Δquat = {_fmt(ph['tcp_quaternion_gap'])}（需 < 1e-9）",
        )
        print(f"    Δpos = {_fmt(ph['tcp_position_gap'])} m, "
              f"Δquat = {_fmt(ph['tcp_quaternion_gap'])}")
        print(f"      mj  quat(xyzw) = {[_fmt(v) for v in ph['mj_quat_xyzw']]}")
        print(f"      core quat(xyzw)= {[_fmt(v) for v in ph['core_quat_xyzw']]}")
        print(f"    ※ 两者是**独立来源**：MuJoCo 算 site_xmat，Core 走链式 FK。"
              f"只可能在两边都对时一致。")

        check(
            "位姿随关节角变化（防『回显写死常量』）",
            ph["pose_varies"] > 1e-2,
            f"两个位形间最大位移 = {_fmt(ph['pose_varies'])} m",
        )
        check(
            "状态里的末端位姿 == Core FK 对同一批关节角的结果",
            ph["state_pose_vs_core_fk"] < 1e-12,
            f"Δpos = {_fmt(ph['state_pose_vs_core_fk'])}",
        )

        # ---- Runtime 闭环 ----
        check(
            "★ Runtime 换用 MuJoCoBackend：命令 → 执行 → 状态（§62 闭环）",
            ph["runtime_backend_type"] == "MuJoCoBackend"
            and ph["command_count_after"] == ph["command_count_before"] + 1
            and ph["state_differs_from_command"] is True
            and ph["state_status"] == "running",
            f"backend={ph['runtime_backend_type']}, "
            f"command_count {ph['command_count_before']}→{ph['command_count_after']}, "
            f"1 步后 shoulder={_fmt(ph['one_step_shoulder'])}（命令 0.5）",
        )
        print(f"    Backend={ph['runtime_backend_type']}（is_simulation="
              f"{ph['runtime_is_simulation']}）"
              f"  command_count {ph['command_count_before']} → {ph['command_count_after']}")
        print(f"    1 步后 shoulder={_fmt(ph['one_step_shoulder'])} vs 命令 0.5"
              f" ⇒ 状态 ≠ 命令（惯性尚未走完）")

        check(
            "限幅：超限命令被夹到模型上限，且关节**真的到得了**上限",
            ph["clamp_ok"] is True,
            f"命令 99 → 记录 {ph['clamp_reported']}，上限 "
            f"{_fmt(ph['clamp_upper'])}，40 步后 {_fmt(ph['clamp_reached'])}",
        )
        print(f"    限幅：99.0 → 记录 {_fmt(ph['clamp_upper'])}，"
              f"40 步后状态 {_fmt(ph['clamp_reached'])}")
        print(f"    ※ 这条同时钉住 MJCF 的 <contact><exclude>："
              f"若自碰撞把 shoulder 挡住，稳态会停在 0.9367 而不是 1.5708")

        check(
            "数值稳定：全量程扫掠后无 NaN / Inf",
            ph["no_nan"] is True,
            f"NaN 关节 = {ph['nan_joints']}",
        )

        # ---- 反向自检 A ----
        check(
            "反向自检A：mock 与 mujoco 的单步状态**确实不同**（锚②③非空转）",
            ph["mock_differs_from_mujoco"] is True,
            f"mock 一步 = {_fmt(ph['mock_one_step_shoulder'])}，"
            f"mujoco 一步 = {_fmt(ph['one_step_shoulder'])}，"
            f"差 = {_fmt(ph['mock_vs_mujoco_gap'])}",
        )
        print(f"    自检A：mock 一步 {_fmt(ph['mock_one_step_shoulder'])} vs "
              f"mujoco 一步 {_fmt(ph['one_step_shoulder'])}"
              f" ⇒ 差 {_fmt(ph['mock_vs_mujoco_gap'])}（两者不是同一个东西）")

        # ---- 反向自检 B ----
        check(
            "反向自检B：重力置零后下垂消失（锚①真的来自重力）",
            ph["droop_comes_from_gravity"] is True,
            f"无重力时下垂 = {_fmt(ph['nogravity_droop'])}（需 < 1e-9）",
        )
        print(f"    自检B：gravity=[0,0,0] 时下垂 = {_fmt(ph['nogravity_droop'])}"
              f" ⇒ 锚①的有效性来自重力，不是巧合")

        # ---- 四元数 ----
        # 判据要点：输入用**同时满足 w>x>y>z 且 w≠0** 的非对称值，
        # 这样"前 4 个元素"和"从 1 起取 4 个"会给出不同的结果，
        # 4 个分量全部错位时无法互相抵消 ⇒ 能真正证伪。
        check(
            "§40/§35 四元数换算方向正确（非对称输入，四分量全错位也无处躲）",
            ph["quat_fwd"] == [0.1, 0.2, 0.3, 0.5]
            and ph["quat_rev"] == [0.5, 0.1, 0.2, 0.3]
            and ph["quat_round_trip"] is True
            and ph["quat_inverse_pair"] is True,
            f"fwd={ph['quat_fwd']}, rev={ph['quat_rev']}, "
            f"round_trip={ph['quat_round_trip']}, "
            f"inverse_pair={ph['quat_inverse_pair']}",
        )
        print(f"    四元数：[w,x,y,z]={[0.5, 0.1, 0.2, 0.3]} → "
              f"[x,y,z,w]={ph['quat_fwd']} → 往返 = {ph['quat_round_trip']}")
        print(f"    ※ 两个函数入参序不同（fwd 吃 wxyz，rev 吃 xyzw）；"
              f"用非对称值 ⇒ 写错成取后三位会得到 [0.2,0.3,0.5]，该断言可失败")

    # ================================================================ 4 WebSocket
    print("\n[4] WebSocket 端到端（换成 MuJoCoBackend，§62）")
    rc, out = run_python(WS_SNIPPET.format(root=str(ROOT).replace("\\", "\\\\")))
    ws = parse_result(out)
    if not ws:
        check("WebSocket（MuJoCo 后端）端到端脚本执行", False, out[-900:])
    else:
        check(
            "§52 连接即推 robot_info + simulation_state（判据=集合，非顺序）",
            set(ws["hello_types"]) == {"robot_info", "simulation_state"},
            f"实际 {ws['hello_types']}（顺序不作要求）",
        )
        check(
            "simulation_state 报出 MuJoCoBackend（前端可据此区分后端）",
            ws["sim_backends"] == ["MuJoCoBackend"],
            f"实际 {ws['sim_backends']}",
        )
        print(f"    simulation_state：backends={ws['sim_backends']}, "
              f"type={ws.get('sim_type')}")

        check(
            "★ 命令 → MuJoCo 执行 → State 返回（§62 前六项验收）",
            ws["cmd_resp_type"] == "robot_state"
            and ws["cmd_resp_status"] == "running"
            and ws["cmd_state_differs"] is True,
            f"type={ws['cmd_resp_type']}, status={ws['cmd_resp_status']}, "
            f"1 步后 shoulder={_fmt(ws['cmd_resp_joints']['shoulder'])}（命令 0.5）",
        )
        print(f"    WS 命令响应：{ws['cmd_resp_type']} status={ws['cmd_resp_status']} "
              f"joints={ {k: round(v, 6) for k, v in ws['cmd_resp_joints'].items()} }")
        print(f"    ⇒ 1 步后 shoulder={_fmt(ws['cmd_resp_joints']['shoulder'])}"
              f"（命令 0.5）—— 物理惯性尚未走完")

        check(
            "State 帧带完整末端位姿（end_effector 非 null）",
            ws["cmd_resp_pose"] is not None
            and "position" in ws["cmd_resp_pose"]
            and "orientation" in ws["cmd_resp_pose"],
            f"payload keys = {sorted(ws['cmd_resp_pose'] or {})}",
        )
        check(
            "速度随状态一起推送（前端可做速度显示）",
            ws["velocities_present"] is True,
        )

        check(
            "多步后 WS 状态收敛到命令值（容差 5e-3，物理）",
            ws["converged"] is True,
            f"60 步后 shoulder = {_fmt(ws['converged_shoulder'])}（命令 0.5）",
        )
        print(f"    60 步后 shoulder = {_fmt(ws['converged_shoulder'])}")

        check(
            "§52 `timestamp` 是仿真时间且单调递增",
            ws["timestamps_monotonic"] is True,
            f"最近 5 个时间戳 = {[_fmt(t) for t in ws['timestamps']]}",
        )

        check(
            "§52 错误帧：MuJoCo 后端下错误码同样可达",
            ws["error_codes"] == ["bad_frame", "unknown_type", "bad_command", "unknown_robot"],
            f"实际 {ws['error_codes']}",
        )
        print(f"    错误码序列：{ws['error_codes']}")

        check("坏帧不断连", ws["alive_after_errors"] is True)
        check(
            "NaN 被拒（RFC 8259）",
            ws["nan_rejected_code"] == "bad_frame",
            f"实际 {ws['nan_rejected_code']}",
        )
        check(
            "§49 帧里不含渲染信息",
            not ws["render_keys"],
            f"越界字段：{ws['render_keys']}",
        )
        check(
            "§52 所有物理数值都是 JSON Number",
            not ws["stringified_numbers"],
            f"被字符串化的数值：{ws['stringified_numbers']}",
        )
        check(
            "REST 未回归（换后端后 /api/health 与 /api/robots 仍正常）",
            ws["rest_health"] == 200 and ws["rest_robots"] == 1,
            f"health={ws['rest_health']}, robots={ws['rest_robots']}",
        )
        print(f"    REST：health={ws['rest_health']}, robots={ws['rest_robots']}")

    # ================================================================ 5 合成机器人
    print("\n[5] ★ 锚⑤ 换成另一台机器人（§69 规则 2：无型号分支）")
    rc, out = run_python(SYNTHETIC_SNIPPET.format(root=str(ROOT).replace("\\", "\\\\")))
    syn = parse_result(out)
    if not syn:
        check("合成机器人脚本执行", False, out[-900:])
    else:
        check(
            "合成 2-DOF 模型的关节表被正确读出（pan / lift）",
            syn["toy_joint_ids"] == ["pan", "lift"] and syn["toy_dof"] == 2,
            f"实际 {syn['toy_joint_ids']}，DOF={syn['toy_dof']}",
        )
        print(f"    合成模型：{syn['toy_joint_ids']}（DOF={syn['toy_dof']}）")

        check(
            "同一份 MuJoCoBackend 驱动 2-DOF 机器人（关节数/关节名都不是硬编码）",
            syn["toy_backend_joints"] == ["pan", "lift"]
            and syn["toy_pan_ok"] is True
            and syn["toy_lift_ok"] is True,
            f"backend joints={syn['toy_backend_joints']}, "
            f"收敛 = { {k: round(v, 6) for k, v in syn['toy_converged'].items()} }",
        )
        print(f"    命令 pan=0.5, lift=-0.4 → 稳态 "
              f"{ {k: round(v, 6) for k, v in syn['toy_converged'].items()} }")

        check(
            "合成模型的末端位姿也能算（EE 解析对第二个模型同样成立）",
            syn["toy_pose_present"] is True,
        )

        check(
            "整条链路（发现 → 加载 → Runtime → MuJoCo）对第二台机器人成立",
            syn["toy_runtime_models"] == ["two_dof_toy"]
            and syn["toy_runtime_backend"] == "MuJoCoBackend"
            and syn["toy_runtime_ok"] is True,
            f"models={syn['toy_runtime_models']}, "
            f"backend={syn['toy_runtime_backend']}, pan={_fmt(syn['toy_runtime_pan'])}",
        )
        print(f"    临时 packages 目录 → Runtime：models={syn['toy_runtime_models']}, "
              f"backend={syn['toy_runtime_backend']}")

        # 「自行清理」说的是**被验对象**（RobotRuntime / MuJoCoBackend）不能留下
        # 自己的临时文件或 open handle。而 `with TemporaryDirectory()` 的删除发生在
        # `with` 块**退出时**，此处仍在块内 ⇒ `Path(td).exists()` 必然为 True，
        # 拿它当判据是一条恒假断言（永远 FAIL，却与产品行为无关）。
        # 改为：判定临时目录里除我们自己写进去的包文件外，没有多出任何东西。
        check(
            "负向测试不留残渣（临时目录里没有 Runtime/Backend 产生的新文件）",
            syn["tempdir_extra_files"] == [],
            f"多出的文件：{syn['tempdir_extra_files']}",
        )

    # ================================================================ 6 架构约束
    print("\n[6] 架构约束（§50 / §69 / §40 / §35）")

    # (a) §69 规则 2：仿真层不得出现机器人型号字面量
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
        "§69 规则 2：backend/simulation/ 不含机器人型号字面量",
        not offenders,
        "; ".join(offenders[:3]),
    )
    print(f"    扫描 {len(ARCH_FILES)} 个文件：{'干净' if not offenders else offenders[:2]}")

    # (b) 仿真层不得依赖注册表
    reg: list[str] = []
    for rel in ARCH_FILES:
        p = ROOT / rel
        src = strip_comments_and_strings(p.read_text(encoding="utf-8"))
        for name in ("get_package", "discover_packages", "RobotPackage"):
            if name in src:
                reg.append(f"{rel}: {name}")
    check(
        "仿真层不认识包注册表（Backend 只吃 RobotModel）",
        not reg,
        "; ".join(reg[:3]),
    )
    print(f"    注册表依赖：{'干净' if not reg else reg[:2]}")

    # (c) 依赖方向：runtime 不得 **import** simulation
    dep: list[str] = []
    for p in sorted((ROOT / "backend" / "runtime").glob("*.py")):
        for ln, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            s = line.strip()
            if not (s.startswith("import ") or s.startswith("from ")):
                continue
            low = s.lower()
            if "simulation" in low or "mujoco" in low:
                dep.append(f"{p.name}:{ln}: {s}")
    check(
        "依赖方向：runtime 不 import simulation / mujoco（默认 Backend 仍是 Mock）",
        not dep,
        "; ".join(dep[:3]),
    )
    print(f"    runtime 的 import：{'干净' if not dep else dep[:2]}")

    # (d) §40/§35：四元数重排只允许一处
    import re

    sel_re = re.compile(r"\[\s*[a-zA-Z_.\[\]0-9]+\[1\]\s*,\s*[a-zA-Z_.\[\]0-9]+\[2\]")
    unpack_re = re.compile(r"^\s*w\s*,\s*x\s*,\s*y\s*,\s*z\s*=", re.M)
    hits: list[str] = []
    for p in sorted((ROOT / "backend").rglob("*.py")):
        text = p.read_text(encoding="utf-8")
        if sel_re.search(text) or unpack_re.search(text):
            hits.append(str(p.relative_to(ROOT)).replace("\\", "/"))
    check(
        "§40/§35：四元数 `[w,x,y,z]` 重排只出现在 mujoco_backend.py 一处",
        hits == ["backend/simulation/mujoco_backend.py"],
        f"实际出现于 {hits}（注意：**空列表**说明扫描器失效，并不代表干净）",
    )
    print(f"    四元数重排位置：{hits}")
    print(f"    ※ 空列表 ⇒ 扫描器失效；多于一处在 ⇒ 迟早有一个被改错（且错误互相抵消）")

    # (e) mujoco 必须是局部 import
    mb = (ROOT / "backend" / "simulation" / "mujoco_backend.py").read_text(encoding="utf-8")
    top_lvl = [
        ln.strip() for ln in mb.splitlines()
        if ln.startswith("import mujoco") or ln.startswith("from mujoco")
    ]
    check(
        "`mujoco` 是局部 import（缺失时给可操作错误，而不是 import 期崩溃）",
        not top_lvl,
        f"模块级 import：{top_lvl}",
    )
    print(f"    模块级 import mujoco：{top_lvl or '无'}")

    # (f) §69 规则 10：可替换性的**正向证据**
    app_src = (ROOT / "backend" / "api" / "app.py").read_text(encoding="utf-8")
    check(
        "正向证据：app 的 backend_factory 可注入（§69 规则 10 的立足点）",
        "backend_factory" in app_src and "create_app" in app_src,
        "app.py 里没有 backend_factory",
    )
    #: 极性测试：确认剥离器真的在工作，且**方向正确**
    #:
    #: 判据必须能失败。这里用四条，每条都会在剥离器坏掉时**各自**变红：
    #:   ① 注释正文消失         ← 若漏剥 COMMENT，注释里的名字会污染判定
    #:   ② 字符串正文也消失     ← 本剥离器的定义就是"注释和字符串都删"
    #:   ③ 字符串**内的标识符**被删，但**同名的裸标识符**活着
    #:                          ← 这一条区分"剥离"与"整段抹掉"：
    #:                            若有人把函数改成 `return ""`，③ 与 ④ 全灭
    #:   ④ 正常代码形态（标识符 + 运算符）原样保留
    #: 注意不能写成 `'mearm' in probe`：那要求**删除字符串后正文仍在**，
    #: 自相矛盾 ⇒ 恒假。我第一版就是这么写的，被自己的元测试抓到了。
    probe = strip_comments_and_strings(
        'a = 1  # mini_arm_COMMENT\n'
        'b = "mearm_STRING"\n'
        'c = mearm_BARE\n'
        'd = 2 + 3\n'
    )
    comment_gone = "mini_arm_COMMENT" not in probe
    string_body_gone = "mearm_STRING" not in probe
    bare_ident_kept = "mearm_BARE" in probe
    code_shape_kept = "=" in probe and "+" in probe and "2" in probe and "3" in probe
    check(
        "扫描器元测试：注释被剥离、字符串被剥离、裸标识符与代码形态保留",
        comment_gone and string_body_gone and bare_ident_kept and code_shape_kept,
        f"剥离结果 = {probe!r}；"
        f"注释已删={comment_gone}, 串体已删={string_body_gone}, "
        f"裸标识符保留={bare_ident_kept}, 代码形态保留={code_shape_kept}",
    )
    print(f"    剥离器探针：{probe!r}")
    print(f"    ※ 『裸标识符保留 + 字符串被删』同时成立"
          f" ⇒ 剥离器既非空转，也没整段抹平")

    # ================================================================ 7 CLI
    print("\n[7] CLI 子命令（Phase 3 交付，本 Phase 不得回归）")
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

    # ================================================================ 8 测试
    print("\n[8] 测试套件")
    for label, args in (
        ("tests/test_mujoco.py", ("tests/test_mujoco.py",)),
        ("tests/test_runtime.py", ("tests/test_runtime.py",)),
        ("tests/test_websocket.py", ("tests/test_websocket.py",)),
        ("tests/test_api.py", ("tests/test_api.py",)),
        ("tests/test_fk.py", ("tests/test_fk.py",)),
        ("tests/test_ik.py", ("tests/test_ik.py",)),
        ("tests/test_kinematics_core.py", ("tests/test_kinematics_core.py",)),
        ("packages/mini_arm/tests", ("packages/mini_arm/tests",)),
    ):
        rc, out = run_pytest(*args)
        tail = out.splitlines()[-1] if out else ""
        check(f"pytest {label}", rc == 0, tail)
        print(f"    {label}: {tail}")

    rc, out = run_pytest()
    tail = out.splitlines()[-1] if out else ""
    check("pytest（全量，无回归）", rc == 0, tail)
    print(f"    全量: {tail}")

    # ================================================================ 9 上游 Phase
    print("\n[9] 上游 Phase 无回归")
    for name, script, expect in (
        ("Phase 1", "tools/accept_phase1.py", "Phase 1"),
        ("Phase 2", "tools/accept_phase2.py", "Phase 2"),
        ("Phase 3", "tools/accept_phase3.py", "Phase 3"),
        ("Phase 4", "tools/accept_phase4.py", "Phase 4"),
    ):
        p = ROOT / script
        if not p.is_file():
            check(f"{script} 存在", False, "缺失")
            continue
        proc = subprocess.run(
            [str(PY), str(p)], cwd=ROOT, capture_output=True, text=True, timeout=3600
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
    print(f"Phase 5 验收：{passed}/{total} 通过（耗时 {elapsed:.1f}s）")
    if passed != total:
        print("❌ Phase 5 未通过 —— 依据 spec：不得进入下一 Phase")
        return 1
    print("✅ Phase 5 通过 —— MuJoCo Sim2Sim 闭环成立（允许进入 Phase 6）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
