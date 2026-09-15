# RobotForge 架构 (v0.1)

> **状态：v0.1 已实现并验收（Phase 0-5 + 7 + 8 全绿）**
>
> 本文件描述**已经落地**的架构，不是规划。每条约束都对应 `tools/accept_phaseN.py`
> 里可执行的断言。若某条约束无法被断言，它就不属于本文档。

---

## 1. 一句话架构

> 把**机器人是什么**（`RobotModel`）与**机器人怎么动**（`RobotRuntime` + `Backend`）
> 彻底分开，中间只用两个契约通信：

```text
RobotModel  （静态结构，加载期确定，不可变）
    ↕
RobotCommand / RobotState  （动态数据，运行期流动）
```

这条分界的价值在于：**换物理引擎、换渲染器、换机器人，都不会波及另一侧**。
v0.1 用"加一个 MuJoCoBackend 而 Runtime/WS/路由一行不改"（§61-§63）证明过它。

---

## 2. 完整数据流

```text
packages/<robot>/manifest.yaml          ← 身份 / 能力 / 指针（唯一真值源）
packages/<robot>/model/*.xml            ← 原生 MJCF（v0.1 唯一支持的格式）
        │
        │  backend/api/registry.py :: load_model()      ← 唯一加载分派点
        ▼
  LoaderRegistry ──(format_name="mjcf")──▶ MJCFLoader
        │                                      │
        │                                      │ 调 mujoco.MjModel 解析
        │                                      │ **不**把 mjModel 泄漏出去
        │                                      ▼
        │                                 RobotModel      ← 唯一规范内部表示
        │                                      │
        │            ┌─────────────────────────┼──────────────────────────┐
        │            ▼                         ▼                          ▼
        │      backend/kinematics/fk.py   packages/*/kinematics/ik.py   Validator
        │      （Core 通用链式 FK）        （包内闭式 IK）              （§66 六类检查）
        │            │                         │
        │            └────────────┬────────────┘
        │                         ▼
        │                  RobotRuntime  ←── 绑在 FastAPI lifespan 上
        │                         │           （不是模块级单例）
        │      RobotCommand ──────▶│
        │                         │  backend_factory 注入
        │                         ▼
        │                  RobotBackend（抽象）
        │                    ├── MockBackend      （Phase 4：有状态 + 限速 + 限幅）
        │                    └── MuJoCoBackend    （Phase 5：真物理）
        │                         │
        │      RobotState ◀───────┘
        │                         │
        │                         ▼
        │                  WebSocket（§52 五种帧）
        │                         │
        │                         ▼
        │            TypeScript / Three.js
        │            coordinateAdapter.ts  ← 唯一坐标转换点（§41）
```

---

## 3. 分层与依赖方向（**不可反转**）

```text
backend/
├── model/           RobotModel + 数据类 + Validator      ← 最底层，不依赖任何同层
├── kinematics/      Core FK（纯数学）                     → 只依赖 model
├── loaders/         Loader 抽象 + 注册表 + MJCF 实现      → 依赖 model、mujoco
├── runtime/         Command / State / Backend / Runtime   → 依赖 model、kinematics
├── simulation/      SimulationBackend + MuJoCoBackend     → 依赖 model、kinematics、mujoco
├── api/             registry（分派）+ app（FastAPI）+ websocket
├── transport/       （§48，v0.1 只预留 docstring）
└── cli.py           从 manifest 动态加载 entry
```

**允许的依赖**：

```text
runtime    → model / kinematics / api.registry
kinematics → model
simulation → model / kinematics / mujoco
loaders    → model / mujoco
api        → 以上全部
```

**禁止的依赖**（每条都有验收脚本断言）：

| 禁止 | 为什么 | 断言位置 |
|---|---|---|
| `model` → `kinematics` / `runtime` | model 是最底层数据契约，反向依赖会让"换 Runtime"波及模型定义 | `accept_phase4.py` |
| Core `kinematics` → `mujoco` / `three` / `fastapi` / `runtime` | 纯数学层不得知道引擎存在，否则 FK 无法独立复用 | `accept_phase3.py` |
| `runtime` → `fastapi` / `mujoco` / `starlette` | Runtime 是纯编排层；绑定 Web 框架会让"换传输层"变成重写 | `accept_phase4.py` |
| `runtime/backend.py` → `api.registry` | Backend 只吃 `RobotModel`，不得自己去加载 | `accept_phase4.py` |
| 全 `backend/` 出现机器人型号名（`mearm` / `mini_arm` 等硬编码分支） | §69 规则 2：Core 不得有型号分支 | `accept_phase2.py`（tokenize 剥注释/字符串后扫描） |

> ⚠️ 最后一条的扫描器**必须**用 `tokenize` 剥离注释与字符串：
> 正则处理不了三引号与 f-string，而注释里出现型号名是**合法**的
> （文档在解释"为什么不能硬编码"）。同时必须有**扫描器元测试**，
> 否则"从没匹配到任何东西"也会全绿。

---

## 4. 三条 P0 契约

v0.1 的稳定性全部压在这三个类型上。它们是**跨进程、跨语言、跨时间**的接口，
因此冻结级别最高。

### 4.1 `RobotModel` —— 机器人**是什么**

唯一规范内部表示。**任何模块都不得引入第二种机器人表示**
（不许把 `mujoco.MjModel`、`three.Group`、URDF 树当"另一种 model"流传）。

字段全集见 `docs/robot-model.md`。关键约束：
- 物理量一律**数值**，不得是字符串（`"1.57 rad"` 非法）。
- 不得包含 Runtime / WebSocket / Three.js / MuJoCo 对象（§36）。
- ID 唯一、稳定、ASCII、snake_case（§30）；ID 与 Name 分离（§31）。

### 4.2 `RobotCommand` —— **期望**（Desired）

```text
robot / joint_targets / timestamp
```
没有速度、没有力矩 —— v0.1 只做位置控制。

### 4.3 `RobotState` —— **实际**（Actual）

```text
robot / joint_positions / joint_velocities /
end_effector_pose / status / timestamp
```

> **§61 的验收核心**：Backend **必须真的制造出 `State ≠ Command`**。
> `MockBackend` 故意做成有状态 + 限速（0.35 rad/步）+ 限幅（记录在 `.clamped`）。
> 如果改成原样回显，验收会"结构性通过、信息上一无所有"。

`end_effector_pose = None` **不等于** `Transform.identity()`：
前者是"不知道"，后者是"位姿就是单位变换"。这两者混同会让"无末端定义的机器人"
表现为"末端在原点"。

---

## 5. 扩展点（Phase 7 / 8 的用途）

v0.1 只实现 MJCF + 零资产。但**扩展缝已经开好并验证过**：

```text
模型格式：RobotModelLoader.format_name → LoaderRegistry → RobotModel
几何资产：GeometryRef.asset → GeometryAssetRegistry → AssetLoader → GeometryAsset
```

两条设计决策值得记住（**别改回去**）：

1. **分派键只有一处真值源**：`register(loader)` 读 `loader.format_name`，
   **不**由调用方传名。传参会让"注册名"与"loader 自称的名字"可能不一致，
   而这正是最难查的一类 bug。
2. **重复格式名 ⇒ 抛异常**（不静默覆盖）。
   静默覆盖会让"注册没生效"表现成"加载出来的模型不对" ——
   错误信息指向错误的方向。

两个注册表是**两个类、两套生命周期**：
- `LoaderRegistry`：整模型 / 加载期一次 / 按**格式名**分派
- `GeometryAssetRegistry`：单个资源 / 按需多次 / 按**扩展名**分派

`load_model()` 是**唯一**加载分派点，且 `_REGISTRY = [get_loader_registry()]`
配合 `use_loader_registry()` 支持测试注入与还原。

新增格式的完整做法见 `README.md`。

---

## 6. 为什么 FK 在 Core、IK 在包内

```text
FK  ✅ backend/kinematics/fk.py            （Core，通用）
IK  ✅ packages/mini_arm/kinematics/ik.py  （包内，机构特定）
```

**FK 通用**：沿 Link-Joint 链乘 Transform，对任何 Tree 都成立，
不需要知道"这是什么机器人"。

**IK 不通用**：mini_arm 的 IK 是**闭式解**，依赖 2R 余弦定理、atan2 偏航角、
肘部上下侧判别 —— 这些全部是**机构特定**的知识。强行写"通用 IK"只有一条路：
数值迭代。而数值迭代会引入容差阈值，容差又会掩盖系统性小错
（见 `docs/coordinate-system.md` 的"分类优于容差"）。

代价是**明确的**：包内 `fk.py` 的解析解用文件内写死的几何常量
（`BASE_HEIGHT / SHOULDER_OFFSET / L1 / L2 / L_TOOL`），只从 model 取旋转轴。
⇒ **改 model 的几何时解析 FK 输出不变**。
所以做几何扰动自检必须用 Core 的 `forward_kinematics_generic`。

`manifest.yaml` 用 `kinematics.ik.type: package` 把这个边界**编码进数据**，
而不是靠文档约定。

---

## 7. 坐标与单位在哪里转换

**只有一个地方**：`frontend/src/viewer/coordinateAdapter.ts`。

```text
RobotModel（robotforge 约定，+X 前 / +Y 左 / +Z 上，SI）
        │
        │  ← 全链路保持原样，FK / IK / Runtime / Backend 一律不改坐标
        ▼
coordinateAdapter.ts  ← 唯一的转换点
        ▼
Three.js 场景
```

**§41 原文约束**：不得在 FK / IK 里为 Three.js 修改坐标。
理由：一旦 FK 里出现"顺手把 Z 轴转一下"，FK 的返回值就不再是规范量，
而 §62「FK 与 MuJoCo 一致」的验收就变成了**同义反复**
（两边都用同一个被污染的约定）。

同理，四元数的 `[x,y,z,w]` 与 MuJoCo 的 `[w,x,y,z]` 的换算
**只允许出现在 `backend/simulation/mujoco_backend.py`**，
且 `mj_quat_to_xyzw` / `xyzw_to_mj_quat` **两个方向都要留** ——
只留单向正是当初出 bug 的原因。验收脚本用正则扫描全 `backend/`
断言只有这一个文件出现该换算。

---

## 8. 为什么 Runtime 绑 lifespan 而不是模块级单例

```python
# backend/api/app.py
@asynccontextmanager
async def lifespan(app):
    await runtime.start()
    app.state.runtime = runtime
    yield
    await runtime.stop()
```

模块级单例的问题：**测试之间会互相污染**（第一个测试 start 的 backend
状态残留在第二个测试里），而且 `backend_factory` 无法按测试注入。
绑在 lifespan 上之后，REST 与 WebSocket **共用同一个实例**，
且每个 app 生命周期都是干净的。

---

## 9. 关节顺序：只有一个定义

```text
model.mobile_joint_ids()
```

命令展开、状态拼装、与 MuJoCo `qposadr` 对齐 —— **全部引用它**。
`start()` 里断言 `qpos_addr == sorted(qpos_addr)`，顺序错立刻报错。

这条约束的价值：关节顺序错位是**最隐蔽**的一类 bug ——
所有测试都能跑、数值都在合法范围，只是"命令关节 A 结果关节 B 动了"。
把顺序固定成 `qposadr` 升序并断言，等于让这类错误在 `start()` 就爆掉。

---

## 10. 验收脚本与架构约束的对应

```text
tools/accept_phase1.py   32/32   MJCF → RobotModel（字段完整性、单位、坐标）
tools/accept_phase2.py   56/56   Three.js 语义 + **Core 无型号分支扫描**
tools/accept_phase3.py   47/47   FK/IK 往返（分类判据）+ kinematics 无引擎依赖
tools/accept_phase4.py   65/65   Runtime 契约 + 依赖方向 + 四元数换算单点
tools/accept_phase5.py   67/67   Sim2Sim + 限幅语义 + 关节序断言
tools/accept_phase7.py   50/50   URDF 扩展点（注册表语义、重复名抛异常）
tools/accept_phase8.py   96/96   Geometry Asset 扩展点（缺 asset 不阻止启动）
```

> ⚠️ Phase 3 及之后的清单会跑全量 pytest + 上游 Phase，单个约 12 分钟。
> 在本机沙箱下**必须后台执行并落文件**（前台会被 SIGTERM，产出 0 字节日志）。

### 10.1 证据分级：为什么还需要浏览器级验收

三层验收**不是替代关系**，各自能覆盖的失败模式不同：

| 手段 | 能覆盖 | **测不到** |
|---|---|---|
| `pytest -q`（488 passed） | Core 语义、契约、纯函数 | 任何真实 I/O |
| `tools/probe_ws_live.py`（25/25） | 真实 TCP 上的 HTTP + WS 闭环 | 浏览器渲染 |
| `tools/e2e_browser_check.mjs`（19 PASS） | **`RobotModel → Three.js` 真的画出来了** | 后端数值精度 |

第三层是不可替代的：`RobotScene.tsx` 里曾有一个 `data-kind` 属性，
它在**单元测试全绿、REST 全部 200、WS 探针 25/25 PASS** 的情况下
让整个 3D 场景崩掉（详见 10.2）。没有任何一层测试能发现它。

```text
tools/e2e_browser_check.mjs  19 PASS / 0 FAIL / 1 SKIP
tools/probe_ws_live.py       25/25 PASS
```

### 10.2 ⚠️ Three.js fiber 元素上的属性名**不得含 `-`**

这是 v0.1 前端最贵的一个坑，症状与根因**方向完全相反**，必须记下来。

```text
RobotScene.tsx:  <group data-kind={node.kind} data-id={node.id} ...>
```

`@react-three/fiber` 的 `diffProps` 里有：

```javascript
if (key.includes("-")) entries2 = key.split("-");
```

于是 `data-kind` 被拆成 `["data", "kind"]`，`applyProps` 继续执行：

```javascript
targetProp = keys.reduce((acc, k) => acc[k], instance)
// → instance.data           →  undefined
// → undefined["kind"]       →  TypeError: Cannot read properties of
//                              undefined (reading 'kind')
```

**症状（极具误导性）**：
- 报错栈里只有 `chunk-*.js` 与 `<group>`，**看不到 `RobotScene.tsx`**
- ErrorBoundary 捕获后整棵树重挂载 ⇒ `#root` 的 innerHTML 为空、
  `canvas` / `.panel` 一个都查不到
- 于是**所有基于 DOM 查询的断言同时失败** ⇒ 看起来像"后端没起"或"前端挂了"

⇒ 规则：需要携带元数据时用 `name`（`Object3D.name`）或 `userData`（对象）。
四元数边界换算的单点约束（§41）在这里有个同类：**约定必须只有一处真值源**。

### 10.3 ⚠️ 读 WebGL 像素必须在 `requestAnimationFrame` 内

r3f 用 `WebGLRenderer` 的**默认**配置，**没有开 `preserveDrawingBuffer`**。
每帧 present 之后绘制缓冲被清空，此时 `readPixels` 只能拿到全 0
（`gl.getError()` 仍然是 `0` —— **不报错，只是没有数据**）。

实测对照（同一页面、同一时刻，唯一变量 = 读的时机）：

```text
A) 帧外 readPixels                      → nonzeroPixels =      0
B) rAF 回调内 readPixels                → nonzeroPixels = 149480
C) 帧外 canvas.toDataURL() 长度         → 6162（≈ 空白图，同样被清掉）
```

这个坑的危险之处：判据失败表现为"**页面全黑**"，
而 `Page.captureScreenshot`（走 compositor，不读 GL 缓冲）
**在同一次运行里截出来的图明明有内容** —— 两者矛盾时人容易去怀疑渲染、
相机、坐标变换，**方向全错**。

> **判据规则**：截图有内容、而像素读数为 0 ⇒ 先怀疑**读的时机**，
> 不要怀疑渲染。

### 10.4 ⚠️ 判据不得比较**机器专属**的量

`accept_phase2.py` 里原有一条：

```python
check("fixture.model 与后端实时加载的 mini_arm 一致",
      live_model.to_dict() == model, ...)
```

而 `RobotModel.metadata.description` 是

```text
Loaded from native MJCF (<仓库绝对路径>)
```

fixture 是**提交进仓库**的，于是它在
`D:\user_project\git\RobotForge\...` 上导出、在 `E:\cnb\git\RobotForge\...`
上校验 ⇒ **换一台机器必然 FAIL**。

这个失效模式的危险之处在于**它给出的下一步建议是错的**：
失败信息写着"重跑 `tools/export_view_fixture.py`"，
而重跑只是把**本机**路径写进去 —— 换台机器又不一致，
问题永远修不掉，还会让人以为"fixture 真的过期了"。

⇒ 规则：**判据要比较模型语义，不要比较环境事实**。
磁盘路径属于环境。修法是比较前归一化掉路径：

```python
def normalize(d):
    d = json.loads(json.dumps(d))          # 深拷贝，别改原对象
    desc = d.get("metadata", {}).get("description") or ""
    if "Loaded from native MJCF (" in desc and desc.endswith(")"):
        d["metadata"]["description"] = "Loaded from native MJCF (<path>)"
    return d
```

并且**两个方向都要验**（否则"归一化"可能把真差异也抹掉）：

```text
篡改 links[0].id        ⇒ 不一致（必须变红）  ✅ 实测 False
只改机器路径            ⇒ 一致  （必须仍绿）  ✅ 实测 True
```

---

## 11. 已知限制（v0.1 明确不做）

| 项 | 状态 |
|---|---|
| URDF 格式 | 只预留扩展点，**不实现 Parser**（§43） |
| 几何资产加载 | 注册表就位但**默认为空**（§44 / §65） |
| 非位置控制（速度 / 力矩） | 不支持 |
| 碰撞检测 | 不支持（MJCF 里 8 对 `<exclude>` 是**有意**排除自碰撞） |
| 多机器人同场景 | 不支持（Runtime 设计上可扩展，v0.1 不验证） |
| 认证 / 多用户 | 不涉及 |
| `transport/` 抽象 | 只有预留 docstring（§48） |
