# RobotForge WebSocket 协议 (v0.1)

> **状态：已冻结 (FROZEN, v1)**
>
> 这是前后端之间**唯一**的运行期通道（§52）。线格式一旦冻结，
> 前端与后端可以独立演进 —— 但**帧里的字段名不能改**。
>
> 对应实现：`backend/api/websocket.py`
> 独立验收：`tools/probe_ws_live.py`（真实 TCP，25/25 PASS）

---

## 1. 协议是信封，不是管道

**§49 画出的唯一允许闭环**：

```text
Frontend → WebSocket → RobotCommand → RobotRuntime → Backend
        → 物理引擎 → RobotState → WebSocket → Frontend
```

**§49 明确禁止**：

```text
Frontend → 直接修改 Three.js
```

> **帧里只能有物理量，不能有渲染信息。**
>
> 一旦服务端开始发"该怎么画"（颜色、材质、旋转矩阵），
> 坐标转换就从"一个点"（`coordinateAdapter.ts`）扩散成
> "两个点 + 一条链"，而**漂移是静默的** —— 前后端各自转一次，
> 结果看起来"差不多"，但不再有任何一处是权威。

`tools/probe_ws_live.py` 有一条断言守着这条：扫描所有收到的帧的 key，
断言不出现 `rotationMatrix` / `color` / `material` / `mesh` / `scale` / `opacity`。

---

## 2. 连接时序

```text
客户端                    服务端
   │  ──── WS 握手 ────────▶ │
   │  ◀──── robot_info ───── │   ① 连上**立即**推
   │  ◀── simulation_state ─ │   ② 紧跟着推
   │                         │
   │  ──── robot_command ──▶ │   ③ 下发命令
   │  ◀──── robot_state ──── │   ④ ★ 命令**会**回一帧状态（见 §5）
   │                         │
   │  ──── robot_state ────▶ │   ⑤ 也可以按需拉取
   │  ◀──── robot_state ──── │
```

**为什么连上就推 `robot_info` + `simulation_state`？**
省掉一次"客户端先问、服务端再答"的往返，更重要的是让
**"连上了但什么都没收到"这种故障立刻可见** —— 否则前端会安静地挂在
loading 状态，而日志里什么都没有。

---

## 3. 五种帧类型（§52）

### 3.1 `robot_info`（服务器 → 客户端）

```json
{
  "type": "robot_info",
  "robots": [
    {
      "package":    { "id": "mini_arm", "name": "Mini Arm", ... },
      "model":      { "metadata": ..., "links": [...], "joints": [...] },
      "validation": { "ok": true, "issues": [] }
    }
  ],
  "count": 1
}
```

> ⚠️ **`id` 藏在 `robots[].package.id` 里**，不是 `robots[].id`。
> 每项是 `{package, model, validation}` 三件套。

`model` 是 `RobotModel.to_dict()` 的**原样透传**（同 REST 的 `/model`）。
一旦在这里整形，就产生了**第二份"前端的真值模型"** —— 两份真值必然漂移。

`validation` 与 `model` 一起给，是为了让前端在画不出来时**显示为什么**，
而不是让用户去翻服务器日志。

### 3.2 `robot_command`（客户端 → 服务器）

```json
{
  "type": "robot_command",
  "robot": "mini_arm",
  "joint_targets": { "base_yaw": 0.4, "shoulder": -0.3, "elbow": 0.5 },
  "timestamp": 0.0
}
```

**两个名字都收**：`robot_command` 与 `joint_command`。

§52 的"至少支持"清单写的是 `robot_command`，而 §52 的示例写的是
`joint_command`。**两个都是规范里写下的**，只支持其一会让另一个
变成**静默失败**（客户端收到 `unknown_type` 还是收不到任何东西，
取决于实现）。`tools/probe_ws_live.py` 两个都测。

### 3.3 `robot_state`（服务器 → 客户端）

```json
{
  "type": "robot_state",
  "robot": "mini_arm",
  "joints":       { "base_yaw": 0.39991541364312055, "shoulder": -0.297454 },
  "velocities":   { "base_yaw": 2.181, "shoulder": 0.0 },
  "end_effector": { "position": [0.178216, 0.075337, 0.146651],
                    "orientation": [x, y, z, w] },
  "status": "running",
  "timestamp": 0.42
}
```

> ⚠️ **线格式字段名 ≠ 领域模型字段名**。这是最容易搞错的一处：

| 领域模型（`RobotState`） | 线格式（JSON） |
|---|---|
| `joint_positions` | **`joints`** |
| `joint_velocities` | **`velocities`** |
| `end_effector_pose` | **`end_effector`** |

写前端解析代码时用**右列**。`RobotState.to_dict()` 是唯一的转换点。

`end_effector` 姿态顺序是 `[x, y, z, w]`（§35 Quaternion Contract），
`|q| ≈ 1`（实测 1.000000000000000）。

`end_effector: null` 表示**位姿未知**（模型声明了 EE 但链不完整），
**不等于**"末端在原点"。

### 3.4 `simulation_state`（服务器 → 客户端）

```json
{
  "type": "simulation_state",
  "running": true,
  "robot_count": 1,
  "backends": [
    { "robot": "mini_arm", "backend": "MuJoCoBackend", "is_simulation": true, "status": "running" }
  ]
}
```

> **`backends[].backend` 是"跑的是不是真物理"的唯一凭据。**
> 它报 `MockBackend` 还是 `MuJoCoBackend`，决定了
> "命令 → 状态"这条链路的证据强度。
>
> 如果服务端用默认工厂启动（`uvicorn backend.api.app:create_app --factory`），
> 报的是 **`MockBackend`** —— 那不是 §71 的 Sim2Sim 证据。
> 要真物理，用 `tools/serve_mujoco.py`（注入 `mujoco_backend_factory`）。

内容**全部由 RobotModel / Backend 类型推导**，不含任何机器人型号判断 ——
加一台新机器人时这段代码不用改。

### 3.5 `error`（服务器 → 客户端）

```json
{ "type": "error", "code": "bad_command", "message": "…" }
```

错误码定义在 `ERROR_CODES`（一处枚举，便于前端 match）：

| `code` | 语义 |
|---|---|
| `bad_frame` | 帧不是合法 JSON，或缺少 `type` |
| `unknown_type` | `type` 不在支持清单里 |
| `bad_command` | 命令内容非法（未知关节 / 非数字 / NaN） |
| `unknown_robot` | `robot` 指向一个未加载的机器人 |
| `backend_failure` | Backend 执行失败（未启动 / 内部异常） |

**前端应该 match `code`，不要 match `message` 文本** —— message 是给人读的，
会改；code 是给人写的，不会改。

---

## 4. 数值与单位（§52 末段）

```text
Position       m
Angle          rad
Linear Speed   m/s
Angular Speed  rad/s
Time           s
```

**所有物理数值必须是 JSON Number**，不能是字符串（`"0.5"` 非法）。
`tools/probe_ws_live.py` 有一条断言：`joints` 的每个值 `isinstance(v, (int, float))`。

---

## 5. 一个容易踩的坑：`robot_command` 会回帧

**实测行为**（本项目的 `probe_ws_live.py` 第一版就栽在这里）：

```text
发 robot_command  →  会收到一帧 robot_state
```

这**不是**错误，但会让"发 2 帧收 1 帧"的客户端**每轮欠一帧**，
越积越多，最终所有断言读到的都是**错位的旧帧**。
症状看起来像"协议不支持某个 type"，方向完全错。

**正确写法**：读到目标 `type` 为止，多余的帧跳过（并记账）。

```python
async def send_and_read(ws, payload, want, timeout=10):
    await ws.send(json.dumps(payload))
    while True:
        f = json.loads(await asyncio.wait_for(ws.recv(), timeout))
        if f.get("type") == want:
            return f
        # 不是目标帧 ⇒ 跳过（并且这里应该记账，便于发现"服务端发了未预期的帧"）
```

> 这条属于"**判据必须与实测的协议约定对齐**"——
> 不能与推想的约定对齐（见 `docs/coordinate-system.md` 的判据铁律）。

---

## 6. NaN / Infinity 必须被拒绝

`json.loads('[NaN]')` 在 Python 里**默认返回 `[nan]`**，
而 `nan` 不是合法 JSON。放行它等于让前端的 `JSON.parse`（**它会抛错**）
与服务端**不一致**：服务端接受、前端无法解析。

所以服务端按 **RFC 8259 收严**：

```python
def _reject(token):
    raise ValueError(f"帧里出现非法的 JSON 常量 {token!r}（RFC 8259 不允许）")

json.loads(raw, parse_constant=_reject)
```

实测（`tools/probe_ws_live.py`）：

```text
发 {"joints":{"base_yaw":NaN}}  →  {"type":"error","code":"bad_frame",
                                     "message":"不是合法 JSON：帧里出现非法的 JSON 常量 'NaN'（RFC 8259 不允许）"}
发 ...Infinity...               →  {"type":"error","code":"bad_frame"}
```

且**拒绝非法输入后链路仍然可用**（不是关连接）——
这也是一条断言。

---

## 7. 怎么独立验收（不依赖 pytest）

```bash
# ① 以 MuJoCoBackend 起服务（**不要**用默认工厂，那会得到 MockBackend）
.venv/Scripts/python.exe -m uvicorn tools.serve_mujoco:app \
    --host 127.0.0.1 --port 8000

# ② 跑真实 TCP 探针
.venv/Scripts/python.exe tools/probe_ws_live.py
```

探针覆盖 25 项，全部通过时的关键读数（实测）：

```text
backend                = MuJoCoBackend
单步位移               = 0.040790     （真积分；Mock 会是 0.35 硬走一步）
误差收敛               = 0.142812 → 0.002469
四元数 |q|             = 1.000000000000000
末端 position          = [0.178216, 0.075337, 0.146651]
6 类错误码             = bad_frame / unknown_type / unknown_robot / bad_command 全部正确
```

> **为什么还需要真实 TCP 探针**（pytest 里已有 `test_websocket.py`）：
> pytest 跑在 `TestClient`（进程内 ASGI）上。真实 TCP 额外证明了
> ① uvicorn 真的挂上了 WS 路由 ② 帧的 JSON 序列化**跨进程**也成立
> （没有依赖进程内对象引用）。

---

## 8. 已知限制（v0.1）

| 项 | 状态 |
|---|---|
| 认证 / 鉴权 | 无 |
| 多客户端广播 | 无（每个连接独立，各自收到自己的响应） |
| 状态订阅（服务端主动周期推送） | 无（v0.1 是"命令驱动 + 按需拉取"） |
| 二进制帧 | 不支持（纯文本 JSON） |
| 背压 / 限流 | 无 |
| 帧压缩 | 无（`permessage-deflate` 未启用） |
