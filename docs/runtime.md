# Runtime 与 WebSocket 契约 (P0 · Command / State / Backend)

> 本文档对应提示词 **§28 / §29 / §46 / §47 / §48 / §49 / §52 / §61**。
> 与 `docs/robot-model.md`（RobotModel 契约）同级：三者是 RobotForge 的三大契约。
> 改动本文件描述的任何**语义**（不只是字段名）都属于不兼容变更，见 §7。

---

## 1. 三个契约的关系

```text
┌─────────────────────────────────────────────────────────────┐
│                       RobotModel                            │  ← 机器人**是什么**
│  metadata / coordinate / units / links / joints / ...       │     加载后不变
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┴─────────────────────┐
        ▼                                           ▼
┌───────────────────────┐                 ┌───────────────────────┐
│    RobotCommand       │                 │     RobotState        │
│  **Desired** 希望做什么│  ──Backend──▶  │  **Actual** 实际是什么 │
│  robot / joint_targets│                 │  joints / velocities  │
│  timestamp            │                 │  end_effector / status│
└───────────────────────┘                 └───────────────────────┘
```

> **即使 v0.1 是 Sim2Sim，也必须维持该语义**（§29）。
> 一旦把 State 直接等于 Command，将来接真机（有延迟、有误差、有丢步）
> 就必须改协议与前端 —— 而那时改动面已扩散到全栈。

---

## 2. RobotCommand（§28）

```python
RobotCommand(
    robot: str,                      # 目标机器人的 metadata.id
    joint_targets: dict[str, float], # 关节 id → 目标值（rad 或 m）
    timestamp: float,                # 命令**产生**的时刻（s）
)
```

### 2.1 为什么 `joint_targets` 是**映射**而不是数组

数组下标不是稳定引用（§30 ID Contract）。`[0.1, 0.2]` 在有人调换
MJCF 里两个关节的顺序后含义完全变了 —— 而这**不会报错**，
只会让机器人动错关节。用 id 索引则"关节表变了"会立刻以 `KeyError` 暴露。

### 2.2 为什么 `timestamp` 由**发送方**给

它是命令**产生**的时刻，不是被执行的时刻。由 Runtime 在收到时打戳
会导致"命令延迟"不可测 —— 而延迟正是 Sim2Real 里最需要观测的量。

> ⚠️ **时钟基准**：v0.1 把 `timestamp` 当**不透明的不减量**传递，
> 不做跨源减法（前端的 `performance.now()` 与后端的 `time.monotonic()`
> 基准不同，相减会产生负延迟）。跨源时间对齐留待未来版本。

### 2.3 必须拒绝的输入

| 输入 | 处置 | 为什么 |
|---|---|---|
| `{"shoulder": true}` | **TypeError** | `isinstance(True, int)` 为真 ⇒ 不显式拒绝就变成 `1.0` rad，一个无提示的 57° 误差 |
| `{"shoulder": "0.5"}` | TypeError | 字符串数字破坏"物理量是 JSON Number"（§52） |
| `{"shoulder": NaN/Inf}` | ValueError | `json.loads` 会放行，但前端 `JSON.parse` 会抛错 ⇒ 两端不一致 |
| `{"shulder": 0.1}` | ValueError（`validate_against`） | **静默忽略打错的关节名** = "发了命令但机器人不动"，而日志一片正常 |
| 负 `timestamp` | ValueError | 语义上不存在 |

> **部分指令是合法的**（前端拖一个滑块）。未提及的关节由 Backend 保持原位 ——
> 这一点由 `MockBackend` 的 `test_untouched_joints_hold_position` 钉住。

### 2.4 线上字段名 ≠ 内部字段名

§52 的 JSON 示例用 `joints`，§28 的 Python 类用 `joint_targets`。两者都对：

```text
joint_targets  →  内部契约（Python）
joints         →  线上格式（WebSocket）
```

映射**只在 `RobotCommand.to_dict()` / `from_ws_frame()` 发生一次**，
而不是在多处写 rename。

---

## 3. RobotState（§29）

```python
RobotState(
    robot: str,
    joint_positions: dict[str, float],   # rad / m
    joint_velocities: dict[str, float],  # rad/s / m/s
    end_effector_pose: Transform | None,
    status: str,                         # idle | running | reset | error
    timestamp: float,
)
```

### 3.1 `end_effector_pose = None` ≠ `Transform.identity()`

`None` 表示**不知道**（模型没声明 EE，或 Backend 还没算出位姿）。
`Transform.identity()` 是一个**有含义**的位姿（原点、无旋转）。

用零位姿冒充"不知道"，会让"末端真的在原点"与"末端位置未知"
**再也无法区分** —— 这是一个丢信息的转换。

### 3.2 `status` 是**枚举**，不是自由字符串

```text
idle     已启动，未收到命令
running  正在执行命令 / 仿真推进中
reset    正在复位
error    后端出错（此时 joint_positions 仍是上一次的可用值）
```

自由字符串会让 `"Error"` / `"err"` 静默地不被前端的 `status == "error"` 识别。
`STATUSES` 常量是唯一的真值源。

### 3.3 出错时**不要丢**观测值

`RobotState.stale_like()` 保留关节角只改 status：

```python
stale = state.stale_like()   # status="error"，但 joint_positions 不变
```

一次瞬时错误不该让画面跳回原点 —— 那是"信息丢失"，不是"安全"。

### 3.4 禁止的构造

| 输入 | 处置 | 为什么 |
|---|---|---|
| `status="Error"` | ValueError | 拼写错误必须当场暴露 |
| `end_effector_pose=[1,2,3]` | ValueError | 扁平序列必须恰为 7 个分量 `[x,y,z,qx,qy,qz,qw]` |
| `position_vector(["missing"])` | KeyError | **不补 0**：补 0 会把"状态里缺关节"伪装成"该关节在 0 rad" |

---

## 4. RobotBackend（§47）

```python
class RobotBackend:
    async def start(self) -> None: ...
    async def stop(self) -> None: ...
    async def reset(self) -> None: ...
    async def send_command(self, command: RobotCommand) -> None: ...
    async def get_state(self) -> RobotState: ...
```

### 4.1 这个抽象存在的唯一理由

把下面这个问题在**今天**就变成可回答的：

> 把 MuJoCo 换成一台真机，需要改几行 Runtime？

答案是 **0 行**。`accept_phase4.py` 用一个"撒谎 Backend"（状态恒为 0.123，
与命令无关）注入 Runtime 来证明这一点 —— 若 Runtime 里藏着对
`MockBackend` 的 `isinstance` 或字段访问，该测试会失败。

### 4.2 为什么全是 `async`

不只是"以后要等网络"。`asyncio` 让**同一段 Runtime 代码**
在仿真（微秒级）和真机（毫秒级串口）上都成立。
一旦签名是同步的，未来加 `await` 就是**破坏性变更**。

### 4.3 `MockBackend` 不是"假数据"，是**故意的失败面**

它读的是同一份真值（`RobotModel.mobile_joint_ids()` / `JointLimits`），
并且**故意**具备三个性质：

| 性质 | 实现 | 它演练的失败模式 |
|---|---|---|
| **有状态** | 未提及的关节保持原位 | 部分指令 |
| **限速** | 单步 ≤ `max_step`（默认 0.35 rad） | 跟踪误差、多步收敛 |
| **限幅** | 超限目标夹到 `[position_min, position_max]` | 命令被拒绝/被改 |

这三条让 `State ≠ Command` 这个关键不等式在 Phase 4 就被演练。
如果 MockBackend 原样回显，那么"命令失败"这个失败模式
**从来没有被演练过**，到真机上第一次遇到时只能靠猜。

> ⚠️ **`reset()` 刻意不清零 `command_count`**：测试靠它证明
> "命令确实到达了 Backend"，而 reset 恰好是最常被调用的操作。
> 清零会让"复位后没收到命令"与"从没收到命令"无法区分。

---

## 5. RobotRuntime（§46）

### 5.1 它是一条窄腰

```text
左边：多台机器人、多种包  ──┐
                            ├──▶  RobotRuntime  ──▶  右边：多种 Backend
   它只知道三件事：              （对两边都不知情）
   ① discover_packages() → RobotPackage
   ② RobotPackage.load_model() → RobotModel
   ③ RobotBackend 收 Command 吐 State
```

### 5.2 不得知道的东西（§46 硬要求）

```text
mini_arm   mearm   serial   CAN   USB
```

`accept_phase4.py` 用 `tokenize` 剥离注释与字符串后扫描这六个字面量，
并额外禁止 Runtime import `fastapi` / `mujoco` / `starlette`。

### 5.3 为什么 Backend 由**注入的工厂**产生

如果 Runtime 里写 `MockBackend(model)`，§69 规则 10
（Simulation 与 Real 使用统一 Backend Interface）就废了。
所以构造参数是：

```python
backend_factory: Callable[[RobotModel], RobotBackend]
```

于是"换成 MuJoCo"（Phase 5）是换一个**参数**，不是改 Runtime。

### 5.4 "还没启动"必须抛错，不能返回空

```python
rt = RobotRuntime()
rt.models()   # ✗ RuntimeError_ —— 不返回 []
```

空列表会让**"还没启动"**伪装成**"一台机器人都没有"**，
而这两种情况的修复动作完全不同。

### 5.5 `asyncio.Lock` 的用途

`step()` 用锁保证"发命令 → 推进 → 读状态"三步不被另一个客户端的命令插在中间。
没有锁时的典型 bug：客户端 A 发完命令、客户端 B 也发命令，
A 读状态时拿到的是 **B 命令**的结果 —— 表现为"A 的指令时灵时不灵"，
且**只在两个客户端同时操作时出现**。

> ⚠️ 判据要落在**不变量**上（单步位移 ≤ `max_step`），
> **不要**断言"两个并发结果相等" —— 两个 step 是串行的，
> 第二步从第一步的终点出发，起点不同，结果本来就不该相同。

---

## 6. WebSocket 协议（§52）

### 6.1 五种帧

| type | 方向 | 载荷 |
|---|---|---|
| `robot_info` | S→C | 全部机器人的 `RobotModel.to_dict()` + 校验摘要 |
| `robot_command` / `joint_command` | C→S | `robot` / `joints` / `timestamp` |
| `robot_state` | 双向 | `RobotState.to_dict()`（S→C）；或 `{robot}` 查询（C→S） |
| `simulation_state` | S→C | `running` / `robot_count` / `backends[]` |
| `error` | S→C | `code` / `message` |

> 命令的**两个名字都收**：§52 的示例叫 `joint_command`，
> "至少支持"清单叫 `robot_command`。只支持其一会让另一个**静默失败**。

### 6.2 ★ 帧里不放"渲染信息"

§49 画出唯一允许的闭环，并明确禁止 `Frontend → 直接修改 Three.js`：

```text
Command → Runtime → Simulation → State → Frontend
```

所以帧里**只能**有物理量。不能有 `rotationMatrix` / `color` / `material` /
`object3D` / `yUp` 这类字段。

理由：一旦服务端开始发渲染数据，坐标转换就会从**一个点**
（`coordinateAdapter.ts`，§41）扩散成"两个点 + 一条链"，而漂移是**静默的**。

`accept_phase4.py` 会递归收集**所有帧的所有字段名**，断言不含上述关键词。

### 6.3 单位与数值（§52 末两段）

```text
Position m ｜ Angle rad ｜ Linear Speed m/s ｜ Angular Speed rad/s ｜ Time s
所有物理数值必须是 JSON Number
```

"必须是 JSON Number"排除的是字符串化数字（`"0.5"`）。
验收脚本会模拟前端做一次 `float(value)` 探测，把能被解析成数字的**字符串**报为违规。

### 6.4 JSON 必须按 RFC 8259 收严

```python
json.loads('[NaN]')          # 默认返回 [nan] —— 但 NaN 不是合法 JSON
```

放行它的后果是**服务端接受一个前端无法解析的帧**
（浏览器 `JSON.parse` 会抛错）。所以 `_json_loads_strict` 用
`parse_constant` 把 `NaN` / `Infinity` 变成错误。

### 6.5 错误码

| code | 语义 |
|---|---|
| `bad_frame` | 不是合法 JSON / 顶层非对象 / 缺 `type` / 含 NaN |
| `unknown_type` | `type` 不在支持列表里 |
| `bad_command` | 未知关节 / 非数字 / 超限时间戳 |
| `unknown_robot` | `robot` 指向未加载的机器人 |
| `backend_failure` | Backend 未启动或内部异常 |

> **错误帧不得断连**。这是错误帧这个设计存在的理由 ——
> 如果坏帧会断连，前端唯一能做的就只有重连，而重连会丢掉所有上下文。
> `accept_phase4.py` 在连发 6 个坏帧后仍发一条合法命令并断言能收到 `robot_state`。

### 6.6 为什么 "robot" 字段可以省略（仅限单机器人）

只有一台机器人时省掉 `robot` 是常见的客户端简化。
但**必须在服务端显式补全**，不能让 `None` 一路传下去。
多于一台时返回 `unknown_robot` 并列出可选项。

### 6.7 命令的响应是 `robot_state`，不是 "OK"

`robot_command` 的响应是一帧 `robot_state`（当前**实际**状态），
而不是 `{"ok": true}`。

- 它语义上仍是 **State = Actual**（含限幅/限速后的真实值）
- 不要为每条命令回一帧"成功"：那会创造一种**没人在等**的响应，
  且在多客户端时让"这条 state 是哪条命令的结果"变得不可判定

---

## 7. 变更记录

| 版本 | 日期 | 变更 | 影响 |
|---|---|---|---|
| v1 | 2026-09-15 | 初始冻结：Command/State/Backend/Runtime 四对象、五种 WS 帧、五个错误码 | RobotForge v0.1 基线 |

> **冻结规则**（与 robot-model.md 同级）：
> - **兼容变更**：新增可选字段、新增 WS 帧类型
> - **不兼容变更**：修改字段语义、改单位、改 `status` 取值、改错误码含义、
>   在帧里加入非物理量
>
> 任何不兼容变更必须同时升级前端与 `tools/accept_phase4.py`。
