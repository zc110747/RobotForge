# 前端自实现 sim（FK/IK）· 实现计划

> 状态：**待确认**（确认后开工）
> 基线：`d046fb7`，工作区干净，`pytest -q` 488 passed
> 目标：前端有一份**自己的**运动学实现，与 Sim2Sim 保持一致的特性

---

## 0. 三个已确认的前提（来自你的选择）

| 项 | 你的选择 |
|---|---|
| 前端 sim 的角色 | **本地预演 / 幽灵臂**，与权威态并存 |
| 「直接导入」的形式 | **包内新增 JS/TS 版 kinematics，前端直接 import** |
| State ≠ Command | **需要**：本地也做限幅 + 一阶滞后 |

---

## 1. 一个必须先说清楚的设计张力

你的选择让 `manifest.yaml` 里这句话变成**假的**：

```yaml
# entry = 包内实现文件；v0.1 只有 Python 侧（前端用通用渲染器，无包内 TS 引擎）
```

这不是要绕过的障碍，而是**这次改动的本质**：包从"Python 专属"变成
"Python + JS 双语"。所以我把它当成一次**契约变更**来做，而不是偷偷塞个文件进去。

### 1.1 为什么"三份实现"不是冗余

`docs/robot-package.md` §5.1 已经确立了这套哲学：

```text
Core   forward_kinematics          通用链式相乘
包内   fk.py::forward_kinematics   手推三角公式
                ↑
          两者互证 = "Core 读对了几何"的机器判据
```

加进 JS 版之后变成**三足**，而它的价值比"两份"更高：

```text
Core 链式    ─┐
Python 解析  ─┼─  三者两两可比
JS 解析      ─┘
```

**关键**：JS 版必须**独立移植三角公式**，**不能**把 Python 的某个输出预先烧成表。
否则它就不是裁判，是一条会漂移的缓存。

### 1.2 一条必须写进文档的警告

`fk.py` 的几何是**文件内写死的常量**（`L1/L2/L_TOOL/...`），这是既有设计。
三份实现里有**两份**（Python 解析 + JS 解析）都会共享这个弱点：

> ⚠️ **改 MJCF 的几何时，解析解的输出不变。**
> 做几何扰动自检必须用 Core 的 `forward_kinematics_generic`。

JS 版继承同一个弱点 —— 所以 JS 侧的交叉验证**只能**证明"JS 与 Python 解析解一致"，
**不能**证明"JS 读对了 MJCF"。后者由既有的包内测试 + Core 负责。
这条要写在 JS 文件的头部，防止将来有人拿它当几何正确性的证据。

---

## 2. 已实测的技术前提（不是推断）

我起了一个临时 Vite 服务实测过，**不是**照文档猜的：

| 验证项 | 结果 |
|---|---|
| Vite dev server 能否 import `packages/` 下的 `.ts` | ✅ `code=200`，自动重写为 `/@fs/E:/...`；**不需要**加 `server.fs.allow` |
| `tsc --noEmit` 是否报错 | ✅ 通过（被 import 的文件会作为依赖被拉进编译，不受 `include: ["src"]` 限制） |
| `vite build` 生产构建 | ✅ `BUILD_EXIT=0`，623 modules transformed |

> 探针文件（`__vite_fs_probe.ts` / `__probe__/`）**已全部删除**，`git status` 为 0。

**结论**：`frontend/src/` → `import { ... } from "../../packages/mini_arm/kinematics/kinematics.js"` 可用，
dev / typecheck / build 三条路径都通，**无需改 `vite.config.ts`**。

---

## 3. 文件清单

### 3.1 新增：包内 JS 引擎

```text
packages/mini_arm/kinematics/
├── fk.py             （已有，不动）
├── ik.py             （已有，不动）
└── kinematics.js     ★ 新增 —— ESM，零依赖，纯函数
```

**为什么是 `.js` 而不是 `.ts`**

包是**与语言无关的数据包**，不该假设消费方用 TypeScript。
`.js` + JSDoc 类型注释：浏览器、Node、Vite 都能直接吃，且不引入构建步骤。
前端拿到的类型由它自己的 `.d.ts` 或 JSDoc 推。

**导出的东西**（与 Python 侧一一对应）

```js
// 几何常量（与 fk.py 逐值相同，由交叉验证钉住）
export const BASE_HEIGHT = 0.084;
export const SHOULDER_OFFSET = 0.052;
export const L1 = 0.103;
export const L2 = 0.065;
export const L_TOOL = 0.032;
export const L2_EFF = L2 + L_TOOL;      // 0.097
export const Z_BASE = BASE_HEIGHT + SHOULDER_OFFSET;
export const JOINT_ORDER = ["base_yaw", "shoulder", "elbow"];

// 限位（与 MJCF range 一致，由包内测试断言）
export const YAW_LIMIT = Math.PI;
export const SHOULDER_LIMIT = Math.PI / 2;
export const ELBOW_LIMIT = 3 * Math.PI / 4;
export const TOL_SINGULAR = 1e-6;

// FK：解析闭式解 —— 移植 fk.py::forward_kinematics
export function forwardKinematics(jointPositions) -> { position:[x,y,z], orientation:[x,y,z,w] }

// IK：解析闭式解 —— 移植 ik.py::solve
export function solveIk(target, { branch, preferLimits, clamp, tol }) -> IkSolution
export function solveIkAll(target, { clamp }) -> IkSolution[]
export function reachLimits() -> [rMin, rMax]
```

**必须逐字保留的四个语义细节**（都是 Python 侧"改过一次"的坑，不能凭直觉重写）：

```text
① phi_target = atan2(-z_arm, r)          ★ 负号：绕 +Y 转 θ>0 时 +X 指向 -Z
② θ2 > 0 ⇒ θ1 = phi - alpha              ★ 配对符号，写反会静默错 0.15 m
        θ2 < 0 ⇒ θ1 = phi + alpha
③ 分支判据 = 肘侧偏（二维叉积 hx*ez - hz*ex），**不是** θ2 的符号
④ 退化判据 = 两个候选 θ2 实质相同（|θ2|<TOL 或 |π-|θ2||<TOL），
   **不是**侧偏之差（±0 会算出相反符号）
```

**姿态构造**（顺序不可反）

```js
q_pitch = fromAxisAngle(axisOf(model,"shoulder"), θ1+θ2)
q_yaw   = fromAxisAngle(axisOf(model,"base_yaw"), yaw)
orientation = quatMul(quatMul(q_yaw, q_pitch), IDENTITY).normalized()
//            注意是 Rz(φ) ∘ Ry(θ1+θ2)，先俯仰再随底座偏航
```

⚠️ **轴要从传入的 model 读**（`model.joints.find(j=>j.id===...).axis`），
不硬编码 —— 与 Python 侧"轴从模型读、长度写死"的分工保持一致。

---

### 3.2 新增：前端预演层

```text
frontend/src/sim/
├── predictor.ts        本地预演：限幅 + 一阶滞后 + 与权威态对账
└── __tests__/
    └── predictor.check.ts   自检（含反例注射）
```

**`predictor.ts` 的职责**（严格限定，不碰主渲染）

```ts
export interface Prediction {
  readonly jointPositions: Record<string, number>;  // 本地预测的关节角
  readonly tcp: Vec6;        // 本地 FK 算的 TCP（[x,y,z,x,y,z,w] → 见下）
  readonly source: "predicted" | "authoritative";
  readonly desync: number;   // 与权威态的偏差（rad，∞ 范数）
}

// 输入：上一帧预测 + 新命令 + 关节限位（来自 RobotModel）
// 输出：新预测
export function advance(pred, command, limits, dt): Prediction

// 权威帧到达时调用：决定"接受后端值"还是"保留预测"
export function reconcile(pred, authoritative, tol): Prediction
```

**限幅 + 一阶滞后（复现 State ≠ Command）**

```text
① 限幅：夹到关节 limits（position_min/max）—— 与后端 clamp_targets_to_limits 同语义
        ⚠️ 未声明边界则不夹（不伪造 ±π）
② 滞后：predicted += (clampedTarget - predicted) * alpha
        alpha 取使"单步最大位移"与后端可比的量级；
        后端 MuJoCo 一步 ≈ 600 子步 × 0.002 s = 1.2 s 物理时间 —— 
        alpha 先用 1 - exp(-dt/tau) 形式，tau 可调，默认对齐到亚秒级跟手
③ 对账：|predicted - authoritative| > tol ⇒ 丢弃预测，直接用权威值
        （防止本地模型与真实物理持续分叉还显示"我算的"）
```

**§49 合规性（这是本方案最脆弱的一点，我明确写出来）**

```text
主渲染姿态的来源 = 唯一 = 后端 robot_state        ← 不变
前端预测只画成半透明 ghost / 或仅用于面板显示"预测中"
前端 FK 的返回值**不**直接写进 Three.js 的权威节点
```

预测值如果要上屏，**只**走 `coordinateAdapter.ts`（§41 唯一转换点），
且是**另一个** `<group>`，与权威树分离。

---

### 3.3 修改：manifest.yaml

```yaml
kinematics:
  fk:
    type: package
    entry: kinematics/fk.py
  ik:
    type: package
    entry: kinematics/ik.py
  # ★ 新增：前端预演用的 JS 侧实现（与上面两份互为裁判，见 robot-package.md §5.3）
  js:
    entry: kinematics/kinematics.js
```

⚠️ 这个 `js` 段**只声明位置，不参与 Core 的加载分派** —— Core 不读它。
它是给前端（和构建脚本）的指针，避免前端硬编码
`"../../packages/mini_arm/kinematics/kinematics.js"`。

> 如果 Core 的 manifest 校验是严格的（不认识 `js` 键就报错），
> 需要同步放开校验并补 `tests/test_manifest.py` 的一条断言。**动手前先跑一遍确认。**

---

### 3.4 修改：docs/robot-package.md

- §5 增加 §5.3「JS 侧 `kinematics/` 的边界」，写明：
  - 三份实现的分工与互证关系
  - JS 版**继承**解析解"几何写死"的弱点（不能当几何正确性的证据）
  - 为什么是 `.js` 而非 `.ts`

---

## 4. 验收（三层，不可互相替代）

### 第 1 层：Python↔JS 数值一致（新脚本）

```text
tools/export_kinematics_fixture.py   Python 侧导出真值
  ├── 随机采样 N 个位形（含边界：伸直 θ2=0、折叠 θ2=π、奇异点、限位边界）
  ├── 每个位形算 forward_kinematics（解析）
  ├── 每个位形算 forward_kinematics_generic（Core，第三方裁判）
  └── 每个位形对若干目标位姿调 solve()，记 joint_positions/branch/position_error
      ↓ 写 frontend/src/sim/__tests__/fixtures/kinematics.json

frontend/src/sim/__tests__/kinematics.check.ts   TS 侧比对
  ├── 逐位形 import 包内 kinematics.js 算 FK，与 Python 解析解比 → 容差 1e-12
  ├── 与 Core 链式解比 → 容差 1e-9（Core 走 compose，误差略大是正常的）
  └── IK：比对 joint_positions / branch 字符串 / 位置误差
```

**判据写法**（遵循既有铁律，不写 `Δ < TOL` 了事）

```text
① 分类优先：IK 的分支不许"差不多"，branch 字符串必须**完全相等**
② 不变量：未提供的关节按 0、多余 joint_id 被忽略 —— 两个契约都要测
③ 反例注射：故意把 θ1 的配对符号写反 ⇒ 断言必须变红（实测记录）
④ 容差里命名残余误差："1e-12 是双精度下同一公式两种写法的舍入差"
```

### 第 2 层：前端预演自检

```text
frontend/src/sim/__tests__/predictor.check.ts
  ├── 限幅：命令超限 ⇒ 预测不超限（且未声明边界时**不夹**）
  ├── 滞后：单步位移 ≤ 设计上限（写不变量，不写终值）
  ├── 对账：权威值与预测差很大 ⇒ source 变 authoritative，预测被丢弃
  └── 反例注射：去掉限幅 ⇒ 断言变红
```

### 第 3 层：浏览器端到端（扩展 `tools/e2e_browser_check.mjs`）

把现在 §⑧ 的 `skip("页面内 WebSocket 探针", ...)` 换成真实断言：

```text
① 页面内真的建立了 WS 连接（readyState === OPEN）
② 收到 robot_info（含 model）与 robot_state
③ 拖动关节滑块 ⇒ 发出 joint_command ⇒ 收到 robot_state 且关节值**变了**
④ State ≠ Command：发一个超限命令，回显值与所发值**不等**（证明限幅真的生效）
⑤ 截图像素在拖动前后有差异（证明画面真的动了，不是只改数字）
```

---

## 5. 落地顺序（每步可独立验收）

```text
Step 1  包内 kinematics.js + manifest + 文档           ← 不改前端，可单独验
Step 2  交叉验证脚本（第 1 层验收）                     ← 证明 JS 与 Python 一致
Step 3  frontend/src/sim/predictor.ts + 自检（第 2 层）  ← 纯逻辑，Node 可跑
Step 4  前端 WebSocket 客户端 + 关节滑块面板（任务 #2/#3）
Step 5  预演 ghost 接进场景（走 coordinateAdapter）
Step 6  扩展 e2e_browser_check.mjs（第 3 层）           ← 端到端
Step 7  全量回归：pytest 488 + 各 Phase accept + e2e
```

---

## 6. 需要你拍板的两点

1. **`manifest.yaml` 的 `js` 段是否需要？**
   如果 Core 的 manifest 校验会因未知键报错，加它就要连校验一起改。
   替代方案：前端硬编码相对路径（不优雅，但零 Core 改动）。

2. **预演 ghost 是否要上屏？**
   - 要：能看到"我预测的" vs "后端实际"的分叉（教学价值高，但要多渲染一棵树）
   - 不要：预测只用于面板显示（如"预测 TCP"文字），改动面最小

---

## 7. 我不打算做的事（明确声明）

```text
✗ 不动 fk.py / ik.py 的任何一行（它们是已验证的独立裁判）
✗ 不改 Core 的 FK 引擎
✗ 不让前端预测值写进 Three.js 的权威节点（§49）
✗ 不在 JS 里为 Three.js 改坐标（§41）—— 转换只在 coordinateAdapter.ts
✗ 不为了"让数字对上"而放宽判据容差
```
