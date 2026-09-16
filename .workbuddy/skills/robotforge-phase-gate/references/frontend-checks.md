# 前端（Three.js / TS）Phase 的可执行验收

来源：RobotForge Phase 2（spec §59）实测。所有条目都是**踩过的坑**，
每条都写清"症状 → 根因 → 修法"，因为症状几乎都不指向根因。

---

## 1. 环境：Node 跑 TS/TSX 的可行组合

```text
Node 22 --experimental-strip-types
  ✅ .ts 文件（即使含类型注解、import type）
  ❌ .tsx 文件 —— 报 ERR_UNKNOWN_FILE_EXTENSION，**不支持 JSX**
```

所以检查脚本的写法有硬约束：

| 文件 | 写法 |
|---|---|
| 检查脚本本身 | 必须 `.ts`，用 `React.createElement`，**不写 JSX** |
| 被测组件（真 JSX） | 先用 **esbuild** 编译成 `.mjs`，再 import |

```ts
// 用 vite 自带的 esbuild，不必额外装
const esbuild = (await import("esbuild")) as { build: (o: Record<string, unknown>) => Promise<unknown> };
await esbuild.build({
  entryPoints: [join(SRC_DIR, "RobotScene.tsx")],
  outfile,                      // 放在检查脚本旁边
  bundle: true,                 // ★ 必须
  format: "esm",
  jsx: "automatic",
  target: "es2022",
  platform: "node",
  external: ["react", "react-dom", "react/jsx-runtime", "three", "scheduler"],
  logLevel: "silent",
});
```

**`bundle: true` 为什么必须**：产物写在 `__tests__/` 下，而被测文件里的
相对 import（`./coordinateAdapter.ts`）是按**原位置**解析的。不 bundle 就会
在 `__tests__/` 里找 `./coordinateAdapter.ts` → `ERR_MODULE_NOT_FOUND`。

**Windows 动态 import 坑**：不能 `import("D:\\...\\x.mjs")`，必须

```ts
import { pathToFileURL } from "node:url";
const mod = await import(`${pathToFileURL(outfile).href}?t=${Date.now()}`);
```

否则 `ERR_UNSUPPORTED_ESM_URL_SCHEME ... Received protocol 'd:'`。
`?t=` 用来绕开 ESM 模块缓存（同一个 outfile 反复重编译调试时必需）。

**不要用 jsdom**：Node 22 里 `navigator` 是 **getter-only 全局**
（`Cannot set property navigator of #<Object> which has only a getter`）；
且 jsdom 无 WebGL，`<Canvas>`/fiber 的 `createRoot` 会在建 renderer 时失败。
元素树方案根本不需要 DOM。

---

## 2. 三种检查各能证明什么

### 2.1 `render.check.ts` —— 元素树 ↔ 场景图

映射依据：fiber 的 reconciler 对每个 `<group>` 建一个 `THREE.Group`，
把 `name` / `position` / `quaternion` **原样赋值**，不做额外推导。
⇒ "元素树里 X 挂在 Y 下、位置是 P" ≡ "场景图里 X 是 Y 的子对象、位置是 P"。

```ts
const { renderToStaticMarkup } = await import("react-dom/server");
const el = React.createElement("group",
  { name: "threejs-adapter-root", rotation: [SCENE_ROTATION_X, 0, 0] },
  React.createElement(RobotScene, { vm }));
const markup = renderToStaticMarkup(el);       // 字符串
const tree = parseMarkup(markup);              // 自写极简 markup 解析
```

**必须真渲染，不能调组件函数**。用 `useMemo`/`useEffect` 的组件被当普通函数
调用时会因**没有 hooks dispatcher** 而抛错；若 catch 后静默返回空节点，
"展开失败"就伪装成"没有子节点" ⇒ **所有"找得到吗"断言全红**，
看起来像实现坏了，实际是检查方法坏了（本项目实测踩过）。

自写 markup 解析要点（够用即可，不要引 parser）：

```ts
const re = /<(\/?)([a-zA-Z][\w:-]*)((?:\s+[\w:-]+(?:="[^"]*")?)*)\s*(\/?)>|([^<]+)/g;
// 属性键统一小写（DOM 渲染会小写化）
// 逗号分隔的数字列表 → 数组：/^-?[\d.eE+-]+(\s*,\s*-?[\d.eE+-]+)+$/
```

### 2.2 让"值"可序列化 —— 前端检查的命门

`renderToStaticMarkup` **丢弃对象属性**。实测：

```ts
React.createElement("group", { name: "g", userData: { kind: "link" } })
// → '<group name="g" userData="[object Object]"></group>'
```

`THREE.Vector3` / `THREE.Quaternion` 更糟 —— 直接不出现。
⇒ **"关节位置对不对"这类最该验的量，用对象时完全不可验。**

修法：给 fiber 传**数字数组**（fiber 两种都接受）：

```tsx
// ✓ 序列化成 position="0.084,0,0" ⇒ 数值在产物里可核
<group name={node.key} position={[p.x, p.y, p.z]} quaternion={[q.x, q.y, q.z, q.w]}>

// 元数据同理用字符串
<group data-kind={node.kind} data-id={node.id} userData={{...}}>
```

三种属性的命运对照：

| 属性 | 真实 fiber（浏览器） | renderToStaticMarkup |
|---|---|---|
| `name` | `Object3D.name` ✅ | 原样保留（字符串） |
| `data-*` | 无意义（非 Three 属性） | 原样保留（字符串） |
| `userData` | `Object3D.userData`（可拾取）✅ | **退化成 `[object Object]`** |

⇒ 检查只依赖前两者；`userData` 留在代码里是因为它在真 fiber 里是对的。
并把"`data-kind` 必须是字符串"也写成一条断言 —— 谁改回对象，立刻变红。

### 2.3 `geometry.check.ts` —— 几何尺寸与轴向

静态渲染看不见 `<boxGeometry args={[a,b,c]}/>` 里的数值。但几何构造是
**纯 CPU** 的，可以在 Node 里直接构造并读 `boundingBox`：

```ts
const geom = makeGeometry(g);        // 直接调真实现
geom.computeBoundingBox();
const got = [bb.max.x-bb.min.x, bb.max.y-bb.min.y, bb.max.z-bb.min.z];
```

**期望值必须独立推导**（不读 `makeGeometry`）：

```text
| type     | MJCF size          | 外接盒（独立推出）        |
|----------|--------------------|--------------------------|
| box      | [hx,hy,hz] 半长    | 2hx × 2hy × 2hz          |
| sphere   | [r]                | 2r × 2r × 2r             |
| cylinder | [r, 半长] 沿 Z     | 2r × 2r × 2·L            |
| capsule  | [r, 半长] 沿 Z     | 2r × 2r × 2·L（总长）    |
```

两条互补的检查，**缺一会漏**：

```text
① 轴序规范化后比尺寸（排序后比较）  → 抓住"尺寸算错"
② 单独断言 cylinder/capsule 长轴在 Y → 抓住"轴向搞反"
```

只做 ① 的话，"把 cylinder 转 90° 塞进 box"也能蒙过。
（MJCF 的 cylinder/capsule 沿 **Z**，Three.js 的 Cylinder/CapsuleGeometry
沿 **Y** —— 这个差异必须被显式钉住。）

### 2.4 分层契约：`null` 是拒绝，兜底是上层的事

```text
makeGeometry(未知类型)  →  null        ← 明确拒绝，不瞎猜语义
GeomMesh(拿到 null)     →  品红线框占位 ← 兜底，保证"可见"
```

**在 `makeGeometry` 层断言"未知类型要有兜底几何"是错的** ——
那会把一个正确设计判成错的（本项目实测就是这么误报的）。
该层只断言"返回 `null`"，再加一条哨兵"**已知**类型不得返回 null"
（否则"拒绝"退化成"无差别拒绝"）。兜底由真渲染层断言。

同理：检索不能用 `findGroup`（只认 `<group>`）去找兜底占位，
因为它可能是 `<mesh name="geom-unknown">`。用**按 name 通用查找**：

```ts
function findByName(nodes, name) {          // 不限标签
  for (const n of nodes) {
    if (n.props["name"] === name) return n;
    const r = findByName(n.children, name);
    if (r) return r;
  }
  return null;
}
```

---

## 3. "假检查"清单（前端版）

前四条与 Python 侧同源（见主 SKILL 第 4 节），后三条是前端特有。

### 3.1 模块级可变状态污染（最隐蔽）

诊断函数若累加到**模块级**数组，多次调用会互相污染。
实测症状：跑完 4 次自检后，**同时**得到"121 项通过"与"一堆失败"两个相反结论。

```ts
// ✗ 模块级
const failures: Failure[] = [];
function runChecks(model) { /* 往里 push */ }

// ✓ 每次调用独立
function newChecker() { return { failures: [] as Failure[], count: 0 }; }
function runChecks(model, expect): CheckOutcome { const c = newChecker(); /* ... */ }
```

### 3.2 用"数量/存在性"当判据 → 把正确实现判成错的

实测：断言"可动关节下应有 1 个子 group" —— 但**关节下本来就有 2 个**
（轴指示器 + 子链路），fixed 关节有 1 个。结果正确实现被判错。

```tsx
// 修法：按语义名识别，并让名字本身成为契约
<group name={`axis:${axis.join(",")}`}>   // 检查侧按 name 前缀 "axis:" 找
```

并在注释里写明**不要用子 group 个数当判据**，否则下一个人会再犯。

### 3.3 检查脚本自身要有"注入缺陷自检"

```ts
const SELF_TESTS = [
  { name: "篡改关节轴", mutate: (m) => { m.joints[2].axis = [0,0,1]; }, mustFail: true },
  { name: "篡改父子关系", mutate: (m) => { m.joints[1].parent_link = "base"; }, mustFail: true },
  { name: "篡改末端位置", mutate: (m) => { m.frames[2].transform.position = [0.5,0,0]; }, mustFail: true },
];
// 断言：每个注入都**必须**被检出（检出数 > 0）
```

没有这一层，一个坏掉的检查看起来**比没有检查更可信**。

### 3.4 期望值不得来自被测实现（自证陷阱）

见主 SKILL 第 12 节。核心：Python 侧独立重推 → 导出 fixture 的 `expect`。
并且要有一条"`fixture.model` 与后端**实时**加载的模型一致"的检查 ——
否则"语义一致"只是与一份**可能过期**的快照一致。

---

## 4. 坐标适配（Three.js）的硬约束

RobotForge 是 Z-up，Three.js 是 Y-up。**只允许整场景旋转**一处：

```ts
// viewer/coordinateAdapter.ts —— 唯一转换点
export const SCENE_ROTATION_X = -Math.PI / 2;

// 推导（写在注释里，别背结论）：
// 绕 X 轴 θ 的矩阵 [[1,0,0],[0,cosθ,-sinθ],[0,sinθ,cosθ]]
// θ = -π/2 ⇒ (x,y,z) → (x, z, -y)
// 验证：机器人 +Z → Three.js +Y ✅
```

四条不可违反的推论：

```text
① 旋转施加在最外层 group，**不是** <Canvas>，也**不是**每个节点
   （逐节点转换 = 一次变换忘一处就静默错位）

② 四元数是**恒等映射** —— Three.js Quaternion 内部就是 x,y,z,w，
   与 RobotForge 相同。只换容器，**绝不重排分量、绝不再旋转一次**。
   （再加一次会让关节转两倍：这是"看起来只是有点怪"的错误）

③ 轴向量的转换必须保持 **LOCAL** —— axisToThree 不得调 toThreeWorld，
   父坐标系已被场景根旋转过了。搞错 = "关节会转，但方向很怪"

④ FK / IK 中**不为** Three.js 改坐标（spec §41）
```

把**公理导出成常量**，让测试直接断言公理而非重推数学：

```ts
export const AXIOMS = {
  sceneRotationX: SCENE_ROTATION_X,
  upAxisMapsTo: [0, 1, 0] as const,   // 机器人 +Z → Three.js +Y
  quaternionOrder: "xyzw" as const,
  identityQuaternion: [0, 0, 0, 1] as const,
  threeUpAxis: "y" as const,
};
```

并做**编码为检查**的两条：

```text
· fromThreeWorld(toThreeWorld(v)) == v          （往返恒等）
· axisToThree(v) ≠ toThreeWorld(v)              （两者没被合并成一个函数）
```

### 扫描式约束（"只有一个入口"怎么机器验证）

```python
# -π/2 这个常量只允许出现在 coordinateAdapter.ts
for p in frontend_sources():
    if p.name == "coordinateAdapter.ts": continue
    if "Math.PI / 2" in line: offenders.append(...)

# 只有 App.tsx 可以消费场景旋转常量（其它文件二次旋转 = 错）
if "SCENE_ROTATION_X" in code or "applySceneRotation" in code: offenders.append(...)

# backend/ 不得出现 three 相关字样
if "three" in line.lower() or "y_up" in line.lower(): offenders.append(...)
```

TS 侧需要一个**剥注释**的函数（不剥会误报注释里的教学示例）。
写个 60 行的状态机即可（比引依赖划算），要点：`//` 行注释、`/* */` 块注释、
引号串（含转义），字符**串不剥离**（关键字只出现在字符串里时也值得看一眼）。

---

## 5. 渲染树的正确拓扑（link ↔ joint 交替）

```text
world → base(link) → base_yaw(joint) → shoulder_link(link) → shoulder(joint)
      → upper_arm(link) → elbow(joint) → forearm_link(link)
      → ee_link_fixed(joint) → ee_link(link)
```

两条规则：

```ts
link 的 parentKey  = link.parent_joint === null ? "world" : jointKey(link.parent_joint)
joint 的 parentKey = linkKey(joint.parent_link)
```

**joint 的局部变换就是它的 `origin` —— 这是全树唯一"有位移"的量**
（link 相对其父 joint 是恒等变换，link 的位姿完全由父 joint 承担）。

最常见的错误：把 link 直接挂在 link 下。后果是丢掉 joint 的 origin 平移 ——
**1-DOF 关节时几乎看不出来**（只有一个原点≈0 的关节），多关节就整体错位。

### 末端执行器必须挂在**它所在的 link** 下

```tsx
// ★ 否则它就是个静止的装饰品：机器人一动、EE 标记不跟着动。
//   这是"看起来还行"的典型错误 —— 静态截图完全正常。
const eeIdsHere = node.kind === "link" ? (eeByLink.get(node.id) ?? []) : [];
```

EE 标记的 `position` 必须等于 `frame.transform.position`（TCP 原点），
**不能画在 link 原点** —— 否则在显示屏上一切正常，但它标的是错位置。

### 轴指示器只给**可动**关节

fixed 关节没有自由度，画了会误导。

---

## 6. 语义一致检查（spec §59 最后一条）

RobotModel ↔ Renderer 语义一致，靠 `tools/export_view_fixture.py`：

```python
def derive_expectations(model) -> dict:
    """★ 刻意的用最朴素的方式再推一遍 —— 不复用 TS 的任何逻辑。"""
    linkParent = {l.id: ("world" if l.parent_joint is None else "joint:"+l.parent_joint)
                  for l in model.links}
    jointParent = {j.id: "link:"+j.parent_link for j in model.joints}
    depthFirstOrder = [...]   # 显式 DFS
    axes, movableJointIds, jointOrigins, endEffectors, frames, geometry, ...
```

导出的 fixture 形如 `{robotId, model, expect}`，TS 侧只与 `expect` 比。

实测规模：5 links / 4 joints / 3 movable / dof 3，深度优先顺序
`world, link:base, joint:base_yaw, link:shoulder_link, joint:shoulder,
link:upper_arm, joint:elbow, link:forearm_link, joint:ee_link_fixed, link:ee_link`。

---

## 7. 验收清单要否证的"架构声明"

`tools/accept_phase2.py` 里几条**扫描式**声明，都要做负向测试证明它会失败：

```text
· Core (backend/) 无型号特定分支        → 注入 "mini_arm" 字面量 ⇒ 应变红
· Frontend (src/) 无型号特定分支        ← 容易漏！能力面板必须由
                                          capabilities 驱动，不看 robot.id
· RobotScene.tsx 不自己推导层级          → 注入读 parent_joint ⇒ 应变红
· viewModel.ts 零 Three.js 依赖          → 检查 import 'three'
· 场景旋转常量只出现在 coordinateAdapter.ts
· 整场景旋转只被 App.tsx 使用
· backend/ 不含 Three.js 相关坐标处理
```

**实测做法**（每次注入后还原，并 `diff` 验证逐字节一致）：

```bash
cp src/viewer/RobotScene.tsx .workbuddy/scratch/  # ★ 不能放 /tmp（Permission denied）
.venv/Scripts/python.exe - <<'PY'
# 注入违规
PY
.venv/Scripts/python.exe tools/accept_phase2.py   # 期望：FAIL + 退出码非 0
cp .workbuddy/scratch/RobotScene.tsx src/viewer/  # 还原
diff .workbuddy/scratch/RobotScene.tsx src/viewer/RobotScene.tsx  # 必须无输出
```

实测：三条注入分别使 56/56 → 52/54、53/54、52/54，退出码非 0。

---

## 8. 端到端：起服务与验证

```bash
# ★ 用 Bash 工具 run_in_background=true，不要 nohup &（后者只活到工具调用结束）
.venv/Scripts/python.exe -m uvicorn backend.api.app:create_app \
    --factory --host 127.0.0.1 --port 8000 --log-level warning

node node_modules/vite/bin/vite.js --host 127.0.0.1 --port 5173
```

验证链（`/api` 走 vite 代理，保持 URL 相对，业务代码里不出现后端地址常量）：

```bash
curl -s http://127.0.0.1:5173/api/robots                  # 应含 mini_arm, count 1
curl -s http://127.0.0.1:5173/api/robots/mini_arm/model   # validation.ok=true
# 还要确认依赖**预打包产物可取**（路径能解析 ≠ 能取到）
curl -s -o /dev/null -w "%{http_code}\n" \
  "http://127.0.0.1:5173/node_modules/.vite/deps/@react-three_fiber.js?v=..."
```

最后一项容易漏：`.tsx` 返回 200 只说明转译成功，
`import { Canvas } from "/node_modules/.vite/deps/@react-three_fiber.js?v=..."` 
是否真的可取，要单独 curl。

---

## 9. 一键启动脚本（`start.bat`）的验证方法

`start.bat` **不是**"顺序写几行命令"就完事 —— 它的失败模式是
**子进程活着、脚本却报成功**（或反之）。所以验证必须落到**端口**与**HTTP 响应**。

### 9.1 验证链路（顺序不可反）

```text
① 脚本自身的命令解析     : start.bat --help → 打印用法且不做事（默认 PAUSE 且 exit 0）
② 环境自检不误报         : 在**已装好**的环境上跑，preflight 全部 OK
③ 真启动                 : cd <repo> && start.bat --no-pause（后台，落日志文件）
④ 判据落在端口上         : netstat -ano | findstr LISTENING  看到 8000 与 5173，
                           且 PID 属于**本次启动**的进程（不是上一次残留的）
⑤ 判据落在 HTTP 上       : curl http://127.0.0.1:5173/api/robots  → mini_arm
                           curl http://127.0.0.1:5173/             → HTTP 200
                           第 ⑤ 条比 ④ 强：端口开着不等于代理通（vite 代理
                           打不通时会回 502 + 一句文本，端口照样 LISTENING）
⑥ 收尾                   : 脚本必须**只 kill 自己起的 PID**。
                           用 `taskkill /IM node.exe /F` 这类按名字杀的写法
                           会误杀用户的编辑器 / 其它 node 服务。
```

### 9.2 三条硬约束（写脚本前先立好，否则一定踩）

```text
① 脚本必须**只写英文+ASCII**。
   .bat 是 GBK（cp936）解析，UTF-8 的中文字节流会被拆成乱码并**吃掉后面的引号**，
   导致 `if` / `for` 括号配对错乱 ⇒ 报错行号与实际行号完全对不上。
   需要中文输出就用 `chcp 65001` + 纯 UTF-8 另存，但 rem 注释仍需注意；
   最省事的做法是注释与文案全用英文。

② 端口占用必须先杀**上一次的自己**再启动。
   否则第二次运行会 `strictPort` 失败（vite 的 port 冲突是**报错退出**，
   不是自动换端口 —— 见 frontend/vite.config.ts 的 strictPort: true）。

③ 暂停与退出码**必须拆开写**（cmd 的 `call :label` 是"返回"不是"终止"）：
   call :pause_exit 1    ✗ 调用处会继续往下走 ⇒ 顺序落入后面的 :fail
   call :pause_exit      ✓ 只等按键
   exit /b 1             ✓ 紧跟其后携带退出码
```

### 9.3 cmd 捕获子进程输出：**不能给 `for /f` 里的路径加引号**（实测）

写 `start.bat` 时最难定位的一个坑。目标是"跑一次 `.venv` 里的 python 拿版本号"。

```bat
rem 症状：变量恒为空，且没有任何报错
for /f "usebackq tokens=2" %%v in (`"D:\...\.venv\Scripts\python.exe" -c "import sys;print(1)" 2^>nul`) do set "V=%%v"
```

实测矩阵（Windows 10 cmd.exe，2026-09-16）：

| 形式 | 结果 |
|---|---|
| `` `python -V 2^>^&1` ``（PATH 上的裸命令） | ✅ `Python 3.13.15` |
| `` `python -c "print(1)" 2^>^&1` `` | ✅ `1` |
| `` `D:\path\python.exe -c "print(1)" 2^>^&1` `` **不加引号** | ✅ `1` |
| `` `"D:\path\python.exe" -c "print(1)"` `` **加引号 + usebackq** | ❌ **空**（静默） |
| 先 `cd` 到该目录，再用裸名字 `python.exe` | ✅ |
| **先把输出重定向到临时文件，再 `for /f ... in ("file")`** | ✅ **路径安全** |

**根因**：`usebackq` 下反引号内的内容整体交给 `cmd /c` 执行。
首字符是 `"` 时，cmd 的引号剥离规则把整串当成**一个**带引号的可执行名，
于是 `"...\python.exe" -c "..."` 被当成单个文件名 ⇒ 找不到文件。
而 `for /f` 对**跑不起来的命令零次迭代且不报错** ⇒ 变量保持空，
与"命令跑了但没输出"完全无法区分（`2^>nul` 还顺手把错误吞了）。

**正解（本工程采用）**：**不要用 `for /f` 直接捕获**，改两段式：

```bat
"%VENV_PY%" -c "import sys;print(sys.version.split()[0])" > "%TMP_OUT%" 2>&1
for /f "usebackq tokens=*" %%v in ("%TMP_OUT%") do set "V=%%v"
```

好处有三：① 路径含空格 / 中文 / `..\` 都不影响；
② "用退出码判成败"与"读 stdout 取值"两件事可以分开做；
③ 捕获到的错误文本能直接打给用户看。

> ⚠️ 附带教训：任何"用 `for /f` 取值"的写法都必须配一条
> **"取不到值时显式失败"** 的检查，否则命令没跑起来时它会静默通过 ——
> 又是"假检查"。本次就是先写成"取不到就当作没装"，
> 结果把一个**装好的环境**报成"python 不存在"，还差点把 `.venv` 删掉重建。

### 9.4 `shift` **不会**改变 `%*`（实测，非常反直觉）

想在子例程里"丢掉第一个参数（标签）再执行剩下的命令"，会自然写成：

```bat
:probe
set "LABEL=%~1"
shift
%* > out.txt 2>&1     rem ← 以为 %* 就是 shift 之后的那串
```

**实测（Windows 10 cmd.exe）**：

```text
call :dump "one" "two" three
  :dump 里  %~1        -> one
  shift 之后 %~1        -> two        （%。1 变了）
  shift 之后 %*         -> "one" "two" three   ← ★ 没变，仍是**原始**全量
```

`shift` 只影响 `%1`/`%2`…（位置参数），**不影响 `%*`**。

**后果（本工程实测症状）**：`%*` 仍以标签开头，于是实际执行的是

```text
python "D:\...\.venv\Scripts\python.exe" -c "import sys"
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 被当成**脚本路径**喂给 python
```

结果是 `SyntaxError: Non-UTF-8 code starting with '\x90' in file ...python.exe`
—— 一个把**解释器自身**当脚本读才可能出现的报错。
它极具误导性：看起来像"python 坏了 / .venv 坏了"，
实际是**参数拼错了**，而且报错内容与真正的根因（标签没被丢掉）毫无字面关联。

**正解**：不要用 `shift` + `%*`。子例程里要么用 `%~2 %~3 ...` 逐位取，
要么直接把整串作为**一个**参数传进来。本工程采用后者，因为命令行里既有
带引号的路径又有带引号的 `-c` 代码，逐位取会踩空格：

```bat
call :probe "python" "%VENV_PY%" -c "import sys"
...
:probe
set "PROBE_LABEL=%~1"
set "PROBE_CMD=%~2"        rem 整条命令作为一个参数传入
%PROBE_CMD% > "%TMP_OUT%" 2>&1
set "PROBE_RC=%errorlevel%"
exit /b %PROBE_RC%
```

> 注：把整串当**一个**参数时，调用处对引号的处理要格外小心 ——
> 见下一节。



### 9.5 诊断类开关必须**真的只读**：`--check` 曾静默杀掉运行中的实例（实测）

给启动脚本加 `--check`（"只做环境预检，不起服务"）时，很容易只把**启动**
那几行跳过，而让 `:check_ports` 原样跑完 —— 但 `:check_ports` 里含
"端口被占就 `taskkill` 掉持有者"的分支。于是：

```text
$ start.bat --check          （此时另一个实例正在 8000/5173 上跑）
  mode     : preflight check only (nothing is started)
  [WARN] port 8000 is used by another program (PID 16572) - stopping it
  [ OK ] port 8000 released
  ...
  Preflight passed. Nothing was started because --check was given.
```

**实测后果**：那次 `--check` 把正在服务的后端（PID 16572）和前端（6372）
一起杀了，两个端口全空。而它同时还打印着 "nothing is started"。
—— 一个声称"什么都不做"的命令，把用户正在用的服务端掉了。

**根因**：把"是否只读"实现成了**跳过启动步骤**，而不是**跳过所有副作用**。
判断标准应当是"这次运行会不会改变系统状态"，而不是"这次运行会不会起进程"。

**修法**：在 `:check_ports` 顶部按模式分叉，`--check` 走一条**纯报告**分支 ——
只打印谁占着端口、置一个 `PORT_BUSY` 标志，然后 `exit /b 1`，绝不 `taskkill`：

```bat
if "%CHECK_ONLY%"=="1" (
  for %%P in (%BACKEND_PORT% %FRONTEND_PORT%) do (
    set "OWNER="
    call :port_owner %%P
    if defined OWNER (
      echo   [WARN] port %%P is in use by PID !OWNER!
      echo          ^(--check does not free ports; stop that process yourself^)
      set "PORT_BUSY=1"
    )
  )
  if defined PORT_BUSY ( ... 完整结论横幅 ... & call :pause_if_needed & exit /b 1 )
  echo   ports    : %BACKEND_PORT% and %FRONTEND_PORT% are free
  exit /b 0
)
for %%P in (...) do ( ... 原有 taskkill 分支，只有真要启动时才走 ... )
```

**通用判据**（可复用）：给任何"检查/诊断/预览"类开关写实现时，
逐行问"这一行会不会改变系统状态？"，而不是"这一行会不会起服务？"。
`taskkill` / `rmdir` / 写文件 / 改配置 都属于状态改变，全部要受开关约束。

**回归测试怎么写**（这次就是这么抓到的）：先让实例跑起来，记下 PID，
**再**运行 `--check`，然后断言

```text
(a) --check 的退出码为 1（端口被占 ⇒ 预检不通过）
(b) netstat 里两个端口**仍在** LISTENING，PID 与运行前**逐字相同**
```

(b) 是真正的判据 —— 只看 (a) 的话，"杀掉实例后报错退出"同样能让 (a) 变绿。

### 9.6 从 `call` 进的子例程里用 `goto` 跳出去 —— 会把调用处的横幅一起打出来

`--check` 那条失败分支最初写成在 `:check_ports` 内部 `goto check_busy`，
把完整结论打印在另一个标签里。实测**两段横幅都打了出来**：

```text
  [FAILED] --check did not pass. Nothing was started and nothing was stopped; ...
  ===========================================================================
  [FAILED] preflight did not pass - nothing was started.
  Fix the [ERROR] or [WARN] lines above, then run start.bat again.
```

原因：`:check_ports` 是通过 `call` 进入的，函数体里的 `goto <外部标签>`
跳出去之后**再也没有回到调用点的"之后"**，于是调用处紧跟着的
`if errorlevel 1 goto failed` 照样执行，又打了一遍通用横幅。

**修法（本工程采用）**：把"自解释的完整结论"就地打印在 `:check_ports` 里，
让调用点知道这条路径不需要再补横幅：

```bat
call :check_ports
if errorlevel 1 goto ports_failed
goto ports_ok
:ports_failed
if "%CHECK_ONLY%"=="1" exit /b 1     rem 结论已在 :check_ports 里打全
goto failed                          rem 其它情况才用通用横幅
:ports_ok
```

**顺带踩到的第二个坑**：同一个位置写成嵌套括号块

```bat
if errorlevel 1 (
  if "%CHECK_ONLY%"=="1" exit /b 1
  goto failed
)
```

实测 **退出码变成了 0**（明明预检没通过）。改成上面那种
"`if ... goto` + 独立标签"的平铺写法后退出码恢复为 1。
⇒ 在 `call` 返回之后紧接着做判断并 `exit /b` 的地方，**别用括号块**。

**教训**：`pause` 与退出码要拆开（本项目铁律），**横幅与退出码也要拆开** ——
一条失败路径只应有一处负责打印、一处负责设定退出码，
否则就会出现"打两遍"或"打一遍但码是错的"。
