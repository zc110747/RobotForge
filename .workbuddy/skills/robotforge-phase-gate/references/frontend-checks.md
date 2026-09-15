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
