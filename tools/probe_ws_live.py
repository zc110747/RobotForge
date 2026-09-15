"""Phase 4/5 联调探针：WebSocket 闭环 + MuJoCo 真物理。

## 为什么单独写这个脚本

WebSocket 的验收在 `tests/test_websocket.py` 与 `accept_phase4.py` 里已经做过，
但那些跑在 **TestClient**（进程内 ASGI）上。
本脚本用**真实 TCP 连接**打到已经起好的 uvicorn 上，
因此它额外证明了：

  ① uvicorn 真的把 WS 路由挂上了（不是只有 TestClient 能通）
  ② 帧的 JSON 序列化在**跨进程**时也成立（没有依赖进程内对象引用）

## 协议要点（照 `backend/api/websocket.py` 实测，**不要凭印象写**）

```text
连上即推 2 帧：  robot_info → simulation_state
robot_command   **不产生即时响应**（设计如此，见 _handle_raw 的 docstring）
                ⇒ 要拿状态必须自己发 {"type":"robot_state"} 去拉
两种命令名都收： joint_command / robot_command
```

## 判据设计

按项目铁律：**写不变量，不写期望终值**。
命令 0.4 之后不断言"状态 == 0.4"（有重力下垂 + 限速 + 限幅三重串联约束），
而是断言：

  - 状态里的关节名集合 == 命令里的关节名集合（**不变量**）
  - 每步位移 <= 声明的 max_step（**不变量**）
  - 单调向目标收敛（**不变量**，与具体终值无关）
  - 末端位姿**存在**且四元数归一（**不变量**）

另外断言"State != Command"—— §61 的核心：如果 Backend 原样回显，
下面这个断言会立刻变红，而"闭环通了"的结论就变成空的。
"""

from __future__ import annotations

import asyncio
import json
import math
import sys

import websockets

WS_URL = "ws://127.0.0.1:8000/ws"
ROBOT = "mini_arm"
JOINTS = ["base_yaw", "shoulder", "elbow"]
TARGET = {"base_yaw": 0.4, "shoulder": -0.3, "elbow": 0.5}
STEPS = 40

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, bool(ok), detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  —— {detail}" if detail else ""))


def quat_norm(q) -> float:
    return math.sqrt(sum(float(v) * float(v) for v in q))


async def recv_frame(ws, timeout: float = 10.0) -> dict:
    """读一帧并解析。"""
    return json.loads(await asyncio.wait_for(ws.recv(), timeout=timeout))


async def send_and_read(ws, payload: dict, want: str, timeout: float = 10.0) -> dict:
    """发一帧，然后**持续读**直到读到 `type == want` 的帧。

    ## 为什么不能"发一帧就读一帧"

    实测（本文件第一版就栽在这）：**`robot_command` 在 MuJoCoBackend 下会回一帧
    `robot_state`**。若按"命令静默"的假设发 2 帧只读 1 帧，就会**每轮欠一帧**，
    越积越多，最终所有断言读到的都是**错位的旧帧** ——
    而症状看起来像"协议不支持某个 type"，方向完全错。

    ⇒ 判据必须与**实测的协议约定**对齐，不能与推想的约定对齐。
    这里用"读到目标 type 为止"的稳健模式：多余的帧被跳过（并且记账），
    既不会错位，也能发现"服务端发了未预期的帧"。
    """
    await ws.send(json.dumps(payload))
    skipped: list[str] = []
    while True:
        f = await recv_frame(ws, timeout)
        if f.get("type") == want:
            if skipped:
                print(f"     [跳过 {len(skipped)} 帧: {skipped}]")
            return f
        skipped.append(f.get("type"))
        if len(skipped) > 10:  # 防死循环
            raise AssertionError(f"等 {want!r} 时跳过了太多帧：{skipped}")


async def main() -> int:
    print("=" * 74)
    print("WebSocket 闭环探针（真实 TCP → uvicorn → RobotRuntime → MuJoCo）")
    print("=" * 74)

    async with websockets.connect(WS_URL) as ws:
        # ---------- ① 连上就该收到 robot_info + simulation_state ----------
        f1 = await recv_frame(ws)
        check("首帧是 robot_info", f1.get("type") == "robot_info", f"type={f1.get('type')}")
        # ⚠️ 线格式是 robots[].package.id（不是 robots[].id）：
        #    每项是 {package, model, validation} 三件套，
        #    id 藏在 package.to_dict() 里。见 websocket.py::_robot_info_frame。
        robots = f1.get("robots") or []
        ids = [(r.get("package") or {}).get("id") for r in robots]
        check("robot_info 里含 mini_arm", ROBOT in ids, f"ids={ids}")
        # robot_info 必须带 model + validation（前端要能显示"为什么画不出来"）
        r0 = robots[0] if robots else {}
        check("robot_info 含 model", isinstance(r0.get("model"), dict),
              f"model keys={len(r0.get('model') or {})} 项")
        check("robot_info 含 validation", isinstance(r0.get("validation"), dict),
              f"ok={(r0.get('validation') or {}).get('ok')}")

        f2 = await recv_frame(ws)
        check("次帧是 simulation_state", f2.get("type") == "simulation_state",
              json.dumps(f2, ensure_ascii=False)[:110])
        # simulation_state 必须报出实际后端类型（这是"跑的是不是真物理"的唯一凭据）
        bks = f2.get("backends") or []
        bk_name = bks[0].get("backend") if bks else None
        check("simulation_state 报出后端类型", bool(bk_name), f"backend={bk_name}")

        # ---------- ② 循环「发命令 → 读状态」，观察状态真的在演化 ----------
        # ⚠️ 实测：MuJoCoBackend 下 **robot_command 会回一帧 robot_state**。
        #    所以用 send_and_read（读到目标 type 为止），不要自己数帧配对。
        # ⚠️ 状态演化发生在 **send_command** 里（Backend 每收一条命令走一步），
        #    而不是在 get_state 里。只反复拉状态会得到同一份快照 N 次 ——
        #    那样"误差在收敛"这类断言会**恒假**，且看起来像 Backend 坏了。
        print(f"\n  → 发送 {STEPS} 轮 robot_command: {json.dumps(TARGET, ensure_ascii=False)}")
        states: list[dict] = []
        for i in range(STEPS):
            f = await send_and_read(
                ws,
                {"type": "robot_command", "robot": ROBOT, "joint_targets": TARGET,
                 "timestamp": float(i)},
                "robot_state",
            )
            states.append(f)

        check("收到 robot_state 帧", len(states) >= STEPS, f"共 {len(states)} 帧")
        if len(states) < 2:
            print("  !! 状态帧太少，后续不变量无法判定")
            return _summary()

        first, last = states[0], states[-1]

        # --- 不变量 A：状态帧里的关节名集合与命令一致 ---
        got = set((first.get("joints") or {}).keys())
        check("状态关节名集合 == 命令关节名集合", got == set(JOINTS), f"got={sorted(got)}")

        # --- 不变量 B：State != Command（§61 核心）---
        fp = first.get("joints") or {}
        diff = max(abs(float(fp.get(j, 0)) - TARGET[j]) for j in JOINTS)
        check("State != Command（Backend 真的在制造状态）", diff > 1e-9,
              f"max|state-target| = {diff:.6f}")

        # --- 不变量 C：单步位移不超过声明的限速 ---
        max_step = 0.0
        for a, b in zip(states, states[1:]):
            pa, pb = a.get("joints") or {}, b.get("joints") or {}
            for j in JOINTS:
                max_step = max(max_step, abs(float(pb.get(j, 0)) - float(pa.get(j, 0))))
        check("单步位移 <= 限速上限(0.35)", max_step <= 0.35 + 1e-9, f"实测 max_step = {max_step:.6f}")

        # --- 不变量 D：向目标单调收敛（不看终值，看误差在缩小）---
        def err(fr: dict) -> float:
            p = fr.get("joints") or {}
            return max(abs(float(p.get(j, 0)) - TARGET[j]) for j in JOINTS)
        e_first, e_last = err(first), err(last)
        check("误差在收敛", e_last < e_first, f"{e_first:.6f} → {e_last:.6f}")

        # --- 不变量 E：关节速度存在且有限 ---
        fv = first.get("velocities") or {}
        check(
            "joint_velocities 存在且有限",
            set(fv.keys()) == set(JOINTS) and all(math.isfinite(float(v)) for v in fv.values()),
            f"{ {k: round(float(v), 4) for k, v in fv.items()} }",
        )

        # --- 不变量 F：末端位姿存在 + 四元数归一 ---
        pose = last.get("end_effector")
        check("末帧 end_effector_pose 非 None", pose is not None, f"pose={'有' if pose else 'None'}")
        if pose:
            quat = pose.get("orientation")
            n = quat_norm(quat)
            check("四元数 |q| ≈ 1", abs(n - 1.0) < 1e-9, f"|q| = {n:.15f}")
            pos = pose.get("position")
            check("position 是 3 维有限数", len(pos) == 3 and all(math.isfinite(float(v)) for v in pos),
                  f"{[round(float(v), 6) for v in pos]}")

        # --- 不变量 G：帧里没有渲染信息（§49）---
        banned = {"rotationMatrix", "color", "material", "mesh", "scale", "opacity"}
        all_keys = set()
        for fr in states:
            all_keys |= set(fr.keys())
        check("帧里无渲染字段（§49）", not (all_keys & banned), f"keys={sorted(all_keys)}")

        # --- 不变量 H：物理量是 JSON Number 不是字符串 ---
        bad = [j for j, v in (last.get("joints") or {}).items() if not isinstance(v, (int, float))]
        check("物理量是 JSON Number", not bad, f"非数值项={bad}")

        # ---------- ④ 主动请求 simulation_state ----------
        f = await send_and_read(ws, {"type": "simulation_state"}, "simulation_state", timeout=5)
        check("可主动拉取 simulation_state", f.get("type") == "simulation_state",
              json.dumps(f, ensure_ascii=False)[:110])

        # ---------- ⑤ 主动请求 robot_info ----------
        f = await send_and_read(ws, {"type": "robot_info"}, "robot_info", timeout=5)
        check("可主动拉取 robot_info", f.get("type") == "robot_info", f"count={f.get('count')}")

        # ---------- ⑥ NaN / Infinity 必须被拒 ----------
        # 这一条要发**非法 JSON 原文**，所以不能用 send_and_read（它把 payload 序列化）。
        await ws.send('{"type":"robot_command","robot":"mini_arm","joint_targets":{"base_yaw":NaN},"timestamp":0}')
        f = await recv_frame(ws, timeout=5)
        check("NaN 被拒绝（error/bad_frame）",
              f.get("type") == "error" and f.get("code") == "bad_frame",
              f"code={f.get('code')}")

        # ---------- ⑥b Infinity 同样必须被拒 ----------
        await ws.send('{"type":"robot_command","robot":"mini_arm","joint_targets":{"base_yaw":Infinity},"timestamp":0}')
        f = await recv_frame(ws, timeout=5)
        check("Infinity 被拒绝（error/bad_frame）",
              f.get("type") == "error" and f.get("code") == "bad_frame",
              f"code={f.get('code')}")

        # ---------- ⑦ unknown_type 必须被拒 ----------
        f = await send_and_read(ws, {"type": "no_such_frame"}, "error", timeout=5)
        check("未知 type 被拒绝（error/unknown_type）", f.get("code") == "unknown_type",
              f"code={f.get('code')}")

        # ---------- ⑧ unknown_robot 必须被拒 ----------
        f = await send_and_read(
            ws, {"type": "robot_command", "robot": "no_such_robot",
                 "joint_targets": {"x": 0.0}, "timestamp": 0.0},
            "error", timeout=5,
        )
        check("未知机器人被拒绝（error/unknown_robot）", f.get("code") == "unknown_robot",
              f"code={f.get('code')}")

        # ---------- ⑨ 未知关节必须被拒绝 ----------
        f = await send_and_read(
            ws, {"type": "robot_command", "robot": ROBOT,
                 "joint_targets": {"no_such_joint": 0.0}, "timestamp": 0.0},
            "error", timeout=5,
        )
        check("未知关节被拒绝（error/bad_command）", f.get("code") == "bad_command",
              f"code={f.get('code')}")

        # ---------- ⑩ 拒绝非法输入后链路必须仍可用 ----------
        f = await send_and_read(
            ws, {"type": "robot_command", "robot": ROBOT,
                 "joint_targets": {"base_yaw": 0.1}, "timestamp": 0.0},
            "robot_state", timeout=5,
        )
        check("拒绝非法输入后链路仍可用", f.get("type") == "robot_state", f"type={f.get('type')}")

    return _summary()


def _summary() -> int:
    print("\n" + "=" * 74)
    passed = sum(1 for _, ok, _ in results if ok)
    total = len(results)
    print(f"结果：{passed}/{total} PASS")
    print("=" * 74)
    for name, ok, detail in results:
        if not ok:
            print(f"  ✗ {name}  {detail}")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
