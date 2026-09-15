/**
 * =============================================================================
 * render.check.ts —— 渲染树结构检查（§59 "浏览器显示机器人"）
 * -----------------------------------------------------------------------------
 * 检查 `<RobotScene/>` 产出的**元素树**，断言它的层级/轴/EE 与模型一致。
 *
 * ## 执行方式
 *
 *   node --experimental-strip-types src/viewer/__tests__/render.check.ts
 *
 * 本文件先用 **esbuild**（vite 的依赖，已在 node_modules 里）把
 * `RobotScene.tsx` 就地编译成 `.mjs`（剥离 JSX），再 import。
 *
 * ## 为什么需要这一步
 *
 * Node 22 的 `--experimental-strip-types` **不支持 .tsx**：
 *   - 本文件必须写成 `.ts`（用 `React.createElement` 而非 JSX）
 *   - 但 `RobotScene.tsx` 是真 JSX，Node import 它会报
 *     `ERR_UNKNOWN_FILE_EXTENSION`
 * ⇒ 用 esbuild 把 JSX 编译掉。这是**唯一的**转译步骤，
 *   编译产物仍是 ESM，仍由 Node 直接执行。
 *
 * ## 为什么不真的起浏览器
 *
 * jsdom 无 WebGL，`<Canvas>` / fiber 的 `createRoot` 都会在创建 renderer 时
 * 失败（本机实测）；起真实 Chromium 需要全局装包 + ~500MB 下载。
 * 更关键：**截图对"层级是否正确"几乎没有判别力** ——
 * 关节轴接反、或 EE 标记没跟着 link 走的机器人，截图看起来同样"像个机械臂"。
 * 本项目已在"看起来还行"上吃过多次亏（skill `robotforge-phase-gate` 第 ④ 节）。
 *
 * ## 元素树与 Three.js 场景图一一映射
 *
 * fiber 的 reconciler 对每个 `<group>` 建一个 `THREE.Group`，把
 * `name` / `position` / `quaternion` **原样赋值**，不做任何额外推导。
 * 因此"元素树里 X 挂在 Y 下、位置是 P" ≡ "场景图里 X 是 Y 的子对象、位置是 P"。
 *
 * 唯一未覆盖的是"几何参数是否正确传给 Three.js 构造器"（需要真 GL），
 * 由 `makeGeometry` 的契约与后续目视验收覆盖。
 */

import { readFileSync, rmSync, existsSync } from "node:fs";
import type { RobotViewModel } from "../viewModel.ts";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const HERE = dirname(fileURLToPath(import.meta.url));
const FIXTURE = join(HERE, "fixtures", "model.json");
const SRC_DIR = join(HERE, "..");

interface Failure {
  check: string;
  detail: string;
}

const failures: Failure[] = [];
let checks = 0;

function ok(cond: boolean, check: string, detail: string): void {
  checks++;
  if (!cond) failures.push({ check, detail });
}

function approx(a: number, b: number, tol = 1e-9): boolean {
  return Math.abs(a - b) <= tol;
}

// ---------------------------------------------------------------------------
// JSX 转译（esbuild）
// ---------------------------------------------------------------------------

/** 用 esbuild 把 RobotScene.tsx（含其本地依赖）编译成单个 .mjs。 */
async function transpileScene(): Promise<string> {
  const esbuild = (await import("esbuild")) as {
    build: (o: Record<string, unknown>) => Promise<unknown>;
  };
  const outfile = join(HERE, "__robotscene.check.mjs");
  await esbuild.build({
    entryPoints: [join(SRC_DIR, "RobotScene.tsx")],
    outfile,
    // ★ bundle: true 是必须的：转译产物会被写到 __tests__/ 下，
    //   而 RobotScene 里的 `./coordinateAdapter.ts` 是相对它**原位置**
    //   解析的。若不 bundle，产物里的相对路径就会指向 __tests__/，
    //   找不到模块。bundle 把本地依赖内联进来，路径问题消失。
    bundle: true,
    format: "esm",
    jsx: "automatic",
    target: "es2022",
    platform: "node",
    // react / three 保持外部依赖：它们由 node_modules 解析，
    // 且不能被打包（three 很大，且是单例语义）
    external: ["react", "react-dom", "react/jsx-runtime", "three", "scheduler"],
    logLevel: "silent",
  });
  return outfile;
}

// ---------------------------------------------------------------------------
// 主流程
// ---------------------------------------------------------------------------

async function main(): Promise<number> {
  console.log("=".repeat(74));
  console.log("RobotScene 渲染树结构检查（元素树 ↔ Three.js 场景图）");
  console.log("=".repeat(74));

  const React = await import("react");
  const { buildViewModel } = await import("../viewModel.ts");
  const { SCENE_ROTATION_X } = await import("../coordinateAdapter.ts");

  const compiled = await transpileScene();
  // ⚠️ Windows 上不能直接 `import("D:\...")` —— 默认 ESM loader 只接受
  //    file/data/node 三种 scheme，收到 'd:' 会报 ERR_UNSUPPORTED_ESM_URL_SCHEME。
  //    必须转成 file:// URL。
  const compiledUrl = pathToFileURL(compiled).href;
  const mod = (await import(`${compiledUrl}?t=${Date.now()}`)) as {
    RobotScene: (p: { vm: RobotViewModel; jointPositions?: Record<string, number> }) => React.ReactNode;
    GhostArm: (p: { vm: RobotViewModel; jointPositions: Record<string, number> }) => React.ReactNode;
  };
  const { RobotScene } = mod;
  ok(typeof RobotScene === "function", "RobotScene 可被导入", `type=${typeof RobotScene}`);

  const raw = JSON.parse(readFileSync(FIXTURE, "utf-8")) as {
    model: Parameters<typeof buildViewModel>[0];
  };
  const { viewModel: vm, problems } = buildViewModel(raw.model);

  console.log(`robot=${vm.robotId} nodes=${vm.nodes.length}`);
  ok(problems.length === 0, "viewModel 构建无问题", JSON.stringify(problems));

  // ★ 用 `renderToStaticMarkup` **真的渲染一遍**，而不是把组件当普通函数调用。
  //
  // 为什么必须真渲染：本项目的渲染组件用了 `useMemo` / `useEffect`。
  // 把函数组件当普通函数调用会因为没有 hooks dispatcher 而抛错，
  // 于是"展开失败"会被静默降级成一个空节点 —— 而那恰好让所有
  // "找得到吗"的断言都失败，看起来像"实现有问题"，实际是"检查方法有问题"。
  // 用 React 自己渲染，hooks 正常，拿到的是**真实产出**。
  //
  // ★ 位置/朝向刻意用**数字数组**传给 fiber（而非 Vector3 对象），
  //   这样它们会序列化成 `position="0.084,0,0"` —— 数值可以被检查，
  //   而不是"只存在于 JS 对象里、一渲染就丢"。见 RobotScene 的注释。
  const { renderToStaticMarkup } = await import("react-dom/server");
  const adapterEl = React.createElement(
    "group",
    { name: "threejs-adapter-root", rotation: [SCENE_ROTATION_X, 0, 0] },
    React.createElement(RobotScene, { vm })
  );

  let markup: string;
  try {
    markup = renderToStaticMarkup(adapterEl);
  } catch (e) {
    console.error("渲染抛异常：", e);
    return report();
  }
  const markupTree = parseMarkup(markup);
  checkMarkupStructure(markupTree, vm, SCENE_ROTATION_X);

  // ★ 新增能力：运行期关节角（权威姿态）+ 幽灵臂（预演姿态）
  const THREE = await import("three");
  const { axisToThree } = await import("../coordinateAdapter.ts");
  checkJointPositions(
    markupTree,
    vm,
    React,
    RobotScene,
    renderToStaticMarkup,
    axisToThree,
    THREE.Quaternion as unknown as new (x: number, y: number, z: number, w: number) => {
      setFromAxisAngle: (a: { x: number; y: number; z: number }, angle: number) => {
        x: number; y: number; z: number; w: number;
      };
    }
  );
  checkGhostArm(markupTree, vm, React, mod, renderToStaticMarkup);

  // ★ 负向验证：注入一个**未知几何类型**，确认它不会静默消失。
  //
  // 为什么必须在这里做而不是在 geometry.check.ts：兜底发生在 `GeomMesh`
  // （makeGeometry 对未知类型**故意**返回 null，兜底是上层职责）。
  // 只有真渲染一遍，才能证明"未知类型 → 有东西画出来"。
  await checkUnknownGeometryFallback(
    React,
    mod,
    raw.model,
    renderToStaticMarkup,
    buildViewModel,
    SCENE_ROTATION_X
  );

  try {
    if (existsSync(compiled)) rmSync(compiled);
  } catch {
    /* 清理失败不影响结论 */
  }

  return report();
}

// ---------------------------------------------------------------------------
// 元素树展开
// ---------------------------------------------------------------------------

interface ElemNode {
  name: string;
  props: Record<string, unknown>;
  children: ElemNode[];
}

/**
 * 解析 `renderToStaticMarkup` 的产物成元素树。
 *
 * 只处理自闭合与普通标签（React 的静态标记里不会有 script/style 特例）。
 * 属性值统一按 **小写键** 存（DOM 渲染会小写化属性名）。
 */
function parseMarkup(markup: string): ElemNode {
  const root: ElemNode = { name: "(root)", props: {}, children: [] };
  const stack: ElemNode[] = [root];
  // 标签 或 文本；用非贪婪匹配
  const re = /<(\/?)([a-zA-Z][\w:-]*)((?:\s+[\w:-]+(?:="[^"]*")?)*)\s*(\/?)>|([^<]+)/g;

  let m: RegExpExecArray | null;
  while ((m = re.exec(markup)) !== null) {
    const [, closing, tag, attrStr, selfClose, text] = m;
    if (text !== undefined) continue; // 纯文本不参与结构检查
    if (closing === "/") {
      if (stack.length > 1) stack.pop();
      continue;
    }
    const props = parseAttrs(attrStr ?? "");
    const node: ElemNode = { name: tag ?? "?", props, children: [] };
    stack[stack.length - 1]!.children.push(node);
    if (selfClose !== "/") stack.push(node);
  }
  return root;
}

/** 解析 `name="value"` 串；`rotation="0.1,0,0"` 这类会尝试还原成数组。 */
function parseAttrs(s: string): Record<string, unknown> {
  const props: Record<string, unknown> = {};
  const re = /([\w:-]+)(?:="([^"]*)")?/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(s)) !== null) {
    const key = (m[1] ?? "").toLowerCase();
    const raw = m[2];
    if (raw === undefined) {
      props[key] = true;
      continue;
    }
    props[key] = coerce(raw);
  }
  return props;
}

/** 把属性字符串还原成有意义的 JS 值（数字 / 数组 / 字符串）。 */
function coerce(raw: string): unknown {
  // 逗号分隔的数字列表 ⇒ 数组（React 渲染 Vec3/数组时就是这个形式）
  if (/^-?[\d.eE+-]+(\s*,\s*-?[\d.eE+-]+)+$/.test(raw)) {
    return raw.split(",").map((x) => Number(x.trim()));
  }
  const n = Number(raw);
  if (raw.trim() !== "" && !Number.isNaN(n)) return n;
  return raw;
}

/** 在元素树里找 `<group name="X">`。 */
function findGroup(nodes: ElemNode[], name: string): ElemNode | null {
  return findByAttr(nodes, "group", "name", name);
}

/**
 * 在元素树里找任意标签上 `name="X"` 的元素。
 *
 * ⚠️ 为什么需要它：`findGroup` 只认 `<group>`，而兜底占位是
 *    `<mesh name="geom-unknown">` —— 用 `findGroup` 找它**永远找不到**，
 *    于是"兜底没生效"会被误报（本检查刚踩过）。
 *    按"标签 + 属性"通用查找，才不会被下游换了元素类型骗到。
 */
function findByName(nodes: ElemNode[], name: string): ElemNode | null {
  for (const n of nodes) {
    if (n.props["name"] === name) return n;
    const r = findByName(n.children, name);
    if (r) return r;
  }
  return null;
}

function findByAttr(
  nodes: ElemNode[],
  tag: string,
  attr: string,
  value: string
): ElemNode | null {
  for (const n of nodes) {
    if (n.name === tag && n.props[attr] === value) return n;
    const r = findByAttr(n.children, tag, attr, value);
    if (r) return r;
  }
  return null;
}

function flatten(nodes: ElemNode[], depth = 0): string[] {
  const out: string[] = [];
  for (const n of nodes) {
    const label =
      n.name === "group" ? `group:${String(n.props["name"] ?? "?")}` : n.name;
    out.push(`${"  ".repeat(depth)}${label}`);
    out.push(...flatten(n.children, depth + 1));
  }
  return out;
}

/**
 * 收集子树里的**全部节点**（含自身后代，前序）。
 *
 * 与 `flatten` 的区别：后者返回给**人看**的字符串，本函数返回节点本身
 * —— 需要对 `props` 做判断时必须用这个（`flatten` 的字符串里
 * `group:` 前缀会被当成名字的一部分）。
 *
 * ⚠️ `stopAtJoint = true` 时**不下钻**到别的关节 group 里。
 *
 * 为什么必须能停：每个关节都有**自己的**轴指示器。若一路下钻，
 * 父关节的子树会把所有子孙关节的轴一起数进来 ——
 * 实测 `base_yaw` 会数出 3 个轴（自己 + shoulder + elbow），
 * 于是"恰好 1 个"这条判据变成"叶子关节才通过"。而那是**错的语义**：
 * 我们要问的是"这个关节有没有它自己的轴指示器"，
 * 不是"它这条链上一共有几个轴"。
 */
function collect(nodes: ElemNode[], stopAtJoint = false): ElemNode[] {
  const out: ElemNode[] = [];
  for (const n of nodes) {
    out.push(n);
    const isNestedJoint =
      n.name === "group" && String(n.props["name"] ?? "").startsWith("joint:");
    if (stopAtJoint && isNestedJoint) continue;
    out.push(...collect(n.children, stopAtJoint));
  }
  return out;
}

/** 把 position/rotation prop 归一化成 [x,y,z]（数组、对象或 Vector3 皆可）。 */
function asXYZ(v: unknown): [number, number, number] | null {
  if (Array.isArray(v) && v.length >= 3) {
    return [Number(v[0]), Number(v[1]), Number(v[2])];
  }
  if (v !== null && typeof v === "object") {
    const o = v as Record<string, unknown>;
    if ("x" in o && "y" in o && "z" in o) {
      return [Number(o["x"]), Number(o["y"]), Number(o["z"])];
    }
  }
  return null;
}

// 注：本文件第一版曾有一套"渲染前展开元素树"的机制（`expandElement`），
// 用来读取 `THREE.Vector3` 这类**对象属性** —— 因为对象不会被序列化成
// HTML 属性。那套机制现已删除，理由是它本身不可靠：
//   * 把函数组件当普通函数调用会因缺 hooks dispatcher 而抛错，
//     于是"展开失败"会伪装成"没有子节点"，让断言假通过或假失败；
//   * 根治办法不是绕过渲染，而是让**值本身可序列化** ——
//     见 RobotScene：position / quaternion 一律以数字数组交给 fiber，
//     于是 `position="0.084,0,0"` 出现在真实渲染产物里，直接可核。
// 检查只依赖真实渲染产物，是唯一可信的基础。

/**
 * 未知几何类型的兜底：**必须画出一个可见的占位**，而不是静默消失。
 *
 * 做法：把模型里第一个 link 的第一个几何换成 `hypercube`（一个不存在的类型），
 * 重新渲染，然后在渲染产物里找 `geom-unknown`。
 *
 * 这条为什么值得单独做：新增几何类型时，"什么都没画"和"画了但位置错"
 * 在屏幕上都是"这块空了"，而日志里一声不响。把它变成会失败的检查，
 * 未来加 mesh / hfield / sdf 时立刻会有人看见。
 */
async function checkUnknownGeometryFallback(
  React: typeof import("react"),
  mod: { RobotScene: (p: { vm: RobotViewModel }) => React.ReactNode },
  rawModel: unknown,
  renderToStaticMarkup: (el: React.ReactElement) => string,
  buildViewModel: (m: never) => ReturnType<typeof import("../viewModel.ts").buildViewModel>,
  sceneRotationX: number
): Promise<void> {
  const m = JSON.parse(JSON.stringify(rawModel)) as {
    links: Array<{ collision?: Array<{ type: string }> }>;
  };
  const host = m.links.find((l) => (l.collision?.length ?? 0) > 0);
  if (!host || !host.collision) {
    ok(false, "有 link 带几何（兜底检查的前提）", "没有 link 带几何 ⇒ 无法注入");
    return;
  }
  host.collision[0]!.type = "hypercube";

  const { viewModel: vm2, problems: p2 } = buildViewModel(m as never);
  ok(p2.length === 0, "注入未知类型后 viewModel 仍能构建（不崩）", JSON.stringify(p2));

  const el = React.createElement(
    "group",
    { name: "threejs-adapter-root", rotation: [sceneRotationX, 0, 0] },
    React.createElement(mod.RobotScene, { vm: vm2 })
  );
  let mk: string;
  try {
    mk = renderToStaticMarkup(el);
  } catch (e) {
    ok(false, "未知类型不导致渲染抛异常", String(e).slice(0, 200));
    return;
  }
  const tree2 = parseMarkup(mk);
  ok(
    findByName(tree2.children, "geom-unknown") !== null,
    "未知几何类型渲染出可见占位（geom-unknown），而不是静默消失",
    "渲染产物里找不到 geom-unknown ⇒ 新增几何类型会无声丢失"
  );
  console.log("  ✓ 未知类型 'hypercube' ⇒ 渲染产物含 geom-unknown 占位（可被发现）");
}



/** 结构检查：用真实渲染产物（markup）。 */
function checkMarkupStructure(
  tree: ElemNode,
  vm: RobotViewModel,
  sceneRotationX: number
): void {
  console.log("\n── 真实渲染产物（renderToStaticMarkup）──────────────────");
  console.log(flatten([tree]).join("\n"));

  // ★ 整场景旋转施加在最外层 group 上，且恰好是 -π/2
  const adapter = findGroup([tree], "threejs-adapter-root");
  ok(adapter !== null, "存在 threejs-adapter-root 组", "找不到该 group");
  if (adapter) {
    const rot = asXYZ(adapter.props["rotation"]);
    ok(rot !== null, "adapter 组有 rotation", "props 里没有合法 rotation");
    if (rot) {
      ok(approx(rot[0], sceneRotationX), "adapter 组 rotation.x = -π/2", `实际 ${rot[0]}`);
      ok(
        approx(rot[1], 0) && approx(rot[2], 0),
        "adapter 组只有 X 轴旋转（不引入额外 Y/Z 旋转）",
        `实际 y=${rot[1]} z=${rot[2]}`
      );
    }
  }

  // 每个非 world 节点都要有 group
  for (const n of vm.nodes) {
    if (n.kind === "world") continue;
    ok(findGroup([tree], n.key) !== null, `渲染产物里有 group name="${n.key}"`, "找不到");
  }

  // ★★ 节点标识只能落在 `name` 上；**禁止 `data-*`**。
  //
  // ## 为什么这条断言必须写（血泪教训）
  //
  // 本文件曾经反过来：断言每个 group 带 `data-kind` / `data-id`，
  // 理由是"对象属性在静态渲染里会退化成 `[object Object]`，
  // 而字符串属性原样保留、可被核对"。
  //
  // **那个理由只在 DOM 渲染器下成立，在真实浏览器里是致命的。**
  //
  // `@react-three/fiber` 的 `diffProps` 里有：
  //
  //     if (key.includes("-")) entries2 = key.split("-");
  //
  // 于是 `data-kind` → `["data", "kind"]`，`applyProps` 接着执行
  // `instance.data.kind`；而 `instance.data` 是 `undefined`
  // ⇒ `TypeError: Cannot read properties of undefined (reading 'kind')`
  // ⇒ ErrorBoundary 捕获 ⇒ 整棵树重挂载 ⇒ **页面全黑**。
  //
  // 实测后果：`pytest` 488 全绿、REST 全部 200、WS 探针 25/25 PASS，
  // **而浏览器里什么都不显示**。这一层检查（静态渲染树）当时也全绿 ——
  // 因为 DOM 渲染器不认识 fiber 的语义，把 `data-*` 当普通 HTML 属性放行了。
  //
  // ⇒ 正确的判据不是"标识能被序列化"，而是
  //   **"标识落在 `name` 上，且没有任何属性名含 `-`"**。
  //   后者才是与渲染器无关的真实约束。
  for (const n of vm.nodes) {
    if (n.kind === "world") continue;
    const grp = findGroup([tree], n.key);
    if (!grp) continue;
    ok(
      grp.props.name === n.key,
      `${n.key} 的 group name 正确`,
      `实际 name=${JSON.stringify(grp.props.name)}`
    );
  }

  // 全局扫描：**任何**元素的属性名都不得含 `-`。
  //
  // 这条比逐节点断言强 —— 它覆盖"以后新增了一个节点/子元素忘了改"。
  //
  // ⚠️ 注意扫的是 `parseMarkup` 产出的元素树（`{name, props, children}`），
  //    **不是** React 元素。一开始我按 React 元素写（找 `props.children`），
  //    结果什么都没扫到 ⇒ 反例注入了却依然全绿 ⇒ 这是一条**恒真的假断言**。
  //    教训与 `parseMarkup` 的存在本身一样：先用"已知答案的探针"验证
  //    检查器能看见目标，再据此下结论。
  const dashedProps: string[] = [];
  const walk = (nodes: ElemNode[], path: string): void => {
    for (const n of nodes) {
      for (const k of Object.keys(n.props)) {
        if (k.includes("-")) dashedProps.push(`${path}/${n.name}@${k}`);
      }
      walk(n.children, `${path}/${n.name}`);
    }
  };
  walk(tree.children, "");
  ok(
    dashedProps.length === 0,
    "渲染树里没有任何含 `-` 的属性名（fiber 按 `-` 拆路径 ⇒ 整个场景崩）",
    `发现 ${dashedProps.length} 处：${dashedProps.slice(0, 3).join(", ")}`
  );

  // 层级：子 group 必须出现在父 group 的子树里
  for (const n of vm.nodes) {
    if (n.parentKey === null || n.kind === "world") continue;
    const parent = findGroup([tree], n.parentKey);
    ok(parent !== null, `父 group ${n.parentKey} 存在`, "");
    if (!parent) continue;
    ok(
      findGroup(parent.children, n.key) !== null,
      `${n.key} 出现在 ${n.parentKey} 的子树里`,
      `${n.parentKey} 的子：${flatten(parent.children).slice(0, 6).join(" | ")}`
    );
  }

  // ★ 可动关节必须有轴指示器；fixed 关节**不得**有。
  //
  // 判据是"关节 group 的**子树**里有没有 name 以 `axis:` 开头的 group"。
  //
  // ⚠️ 不要用"子 group 个数"当判据 —— 关节下**同时**有一个子 group 承载
  //    子 link，所以可动关节本来就有 2 个子 group（轴 + 子链路），
  //    fixed 关节有 1 个。按个数判断会**把正确的实现判成错的**
  //    （本文件第一版就踩了这个）。
  //
  // ⚠️ 为什么要用 `flatten(node.children)` 而不是 `node.children`：
  //    运行期自由度被渲染在一个**内层** `dof:<jointId>` group 里
  //    （见 RobotScene.tsx 的说明：外层承载模型静态安装位姿，
  //     这样 `quaternion == adapter(localTransform)` 那条 §41 断言
  //     仍然是独立可验的）。于是轴指示器现在是**孙节点**。
  //     若要它仍是直接子节点，就得把自由度并进外层 group ——
  //     那会让上面那条 §41 断言变成自证。**宁可放宽这里的层数假设。**
  //
  //     注意放宽的只是"深度"，**没有**放宽"个数"：仍然要求恰好 1 个。
  for (const j of vm.joints) {
    const node = findGroup([tree], `joint:${j.id}`);
    if (!node) continue;
    const all = collect(node.children, /* stopAtJoint */ true);
    const axisGroups = all.filter((n) => String(n.props["name"] ?? "").startsWith("axis:"));
    if (j.isMovable) {
      ok(
        axisGroups.length === 1,
        `可动关节 ${j.id} 有轴指示器`,
        `找到 ${axisGroups.length} 个 axis:* 组；子树：${all.map((n) => n.name).slice(0, 8).join(" | ")}`
      );
    } else {
      ok(
        axisGroups.length === 0,
        `fixed 关节 ${j.id} **没有**轴指示器（无自由度，画了会误导）`,
        `找到 ${axisGroups.length} 个 axis:* 组`
      );
    }
  }

  // ★ 末端执行器标记必须挂在**它所在的 link** 下
  ok(vm.endEffectors.length > 0, "至少有一个末端执行器（否则此项无从验证）", "");
  for (const ee of vm.endEffectors) {
    const linkGroup = findGroup([tree], `link:${ee.linkId}`);
    ok(linkGroup !== null, `EE ${ee.id} 所在 link:${ee.linkId} 在渲染产物里`, "");
    if (!linkGroup) continue;
    const marker = findGroup(linkGroup.children, `ee:${ee.id}`);
    ok(
      marker !== null,
      `EE ${ee.id} 的标记挂在 link:${ee.linkId} 下（而不是场景根）`,
      `link 下的 group：${flatten(linkGroup.children).slice(0, 8).join(" | ")}`
    );
  }

  // link 的几何数量与模型一致（不得漏画）
  for (const n of vm.nodes) {
    if (n.kind !== "link") continue;
    const grp = findGroup([tree], n.key);
    if (!grp) continue;
    // 几何渲染成 <mesh>（函数组件 GeomMesh 已展开成它输出的 mesh）
    const meshCount = grp.children.filter((c) => c.name === "mesh").length;
    ok(
      meshCount === n.geometries.length,
      `link ${n.id} 的几何数量`,
      `渲染产物=${meshCount} 模型=${n.geometries.length}`
    );
  }

  // 几何总数 > 0（防"渲染了一个空机器人"）
  const totalGeom = vm.nodes
    .filter((n) => n.kind === "link")
    .reduce((a, n) => a + n.geometries.length, 0);
  ok(totalGeom > 0, "模型有可渲染的几何", `几何总数=${totalGeom}`);

  // ---- 数值检查 ----------------------------------------------------------
  // ★ 前提：RobotScene 把 position / rotation 以**数字数组**传给 fiber，
  //   于是它们序列化成 `position="0.084,0,0"` 出现在 markup 里，
  //   数值**可以在渲染产物上直接核对**。
  //   如果哪天有人改回传 `THREE.Vector3` 对象，这些断言会立刻变红 ——
  //   这正是它们存在的意义（把"值一渲染就丢"变成会失败的检查）。

  ok(approx(sceneRotationX, -Math.PI / 2), "场景旋转常量 = -π/2", `${sceneRotationX}`);

  // 关节 group 的 position 必须逐分量等于模型 origin。
  // ★ 这是整棵树里**唯一**"有位移"的量（见 viewModel.ts 的注释）：
  //   link 相对父 joint 是恒等变换，因此关节位置画错 = 机器人整体错位。
  for (const j of vm.joints) {
    const node = findGroup([tree], `joint:${j.id}`);
    if (!node) {
      ok(false, `关节 ${j.id} 的 group 在渲染产物里`, "找不到");
      continue;
    }
    const pos = asXYZ(node.props["position"]);
    ok(pos !== null, `关节 ${j.id} 的 group 有 position`, "props 里没有合法 position");
    if (pos) {
      const p = j.origin.position;
      ok(
        approx(pos[0], p[0]) && approx(pos[1], p[1]) && approx(pos[2], p[2]),
        `关节 ${j.id} 的位置 = 模型 origin`,
        `渲染产物=[${pos.join(",")}] 模型=[${p.join(",")}]`
      );
    }
    // 轴都是单位向量（否则 Three.js 的 setFromUnitVectors 会给出错的旋转）
    const norm = Math.hypot(j.axis[0], j.axis[1], j.axis[2]);
    ok(approx(norm, 1, 1e-12), `关节 ${j.id} 的轴是单位向量`, `|axis|=${norm}`);
  }

  // EE 标记的 position 必须等于 frame.transform.position
  for (const ee of vm.endEffectors) {
    const linkGroup = findGroup([tree], `link:${ee.linkId}`);
    if (!linkGroup) continue;
    const marker = findGroup(linkGroup.children, `ee:${ee.id}`);
    if (!marker) continue;
    const pos = asXYZ(marker.props["position"]);
    ok(pos !== null, `EE ${ee.id} 的标记有 position`, "");
    if (!pos) continue;
    const p = ee.transform.position;
    ok(
      approx(pos[0], p[0]) && approx(pos[1], p[1]) && approx(pos[2], p[2]),
      `EE ${ee.id} 的位置 = frame.transform.position（TCP 不能画在 link 原点）`,
      `渲染产物=[${pos.join(",")}] 模型=[${p.join(",")}]`
    );
  }

  // link 几何的位置必须逐 link 逐几何等于模型 geom.transform.position
  for (const n of vm.nodes) {
    if (n.kind !== "link") continue;
    const grp = findGroup([tree], n.key);
    if (!grp) continue;
    const meshes = grp.children.filter((c) => c.name === "mesh");
    n.geometries.forEach((want, i) => {
      const m = meshes[i];
      if (!m) return;
      const pos = asXYZ(m.props["position"]);
      ok(pos !== null, `link ${n.id} 几何[${i}] 有 position`, "");
      if (!pos) return;
      const p = want.transform.position;
      ok(
        approx(pos[0], p[0]) && approx(pos[1], p[1]) && approx(pos[2], p[2]),
        `link ${n.id} 几何[${i}] 的位置 = 模型 geom.transform.position`,
        `渲染产物=[${pos.join(",")}] 模型=[${p.join(",")}]`
      );
    });
  }
}

// ---------------------------------------------------------------------------
// 运行期关节角（权威姿态）
// ---------------------------------------------------------------------------

/**
 * 传入 `jointPositions` 后，每个可动关节的 `dof:<id>` group 必须真的转起来，
 * 且**转的是绕模型声明的那个轴、那个角**。
 *
 * ## 判据为什么落在四元数上而不是"看起来转了"
 *
 * 因为"轴接反"和"角度符号反"都会得到一根**同样像机械臂**的胳膊。
 * 唯一能区分的是四元数：
 * ```text
 * 期望 q = setFromAxisAngle(adapter(axis), θ)
 * ```
 * 其中 `adapter` 是 `axisToThree` —— 与被测代码**同一个**转换函数。
 * ⚠️ 这看起来像"用被验对象证明被验对象"，但它不是：
 *    这里验的是**施加方式**（绕哪个轴、转多少、乘在哪一层），
 *    而 `axisToThree` 本身的正确性由 `geometry.check.ts` 与
 *    `semantics.check.ts` 独立覆盖。两者合起来才完整。
 */
function checkJointPositions(
  tree: ElemNode,
  vm: RobotViewModel,
  React: typeof import("react"),
  RobotScene: (p: { vm: RobotViewModel; jointPositions?: Record<string, number> }) => React.ReactNode,
  renderToStaticMarkup: (el: React.ReactElement) => string,
  axisToThree: (a: readonly [number, number, number]) => { x: number; y: number; z: number },
  QuaternionCtor: new (x: number, y: number, z: number, w: number) => {
    setFromAxisAngle: (a: { x: number; y: number; z: number }, angle: number) => {
      x: number; y: number; z: number; w: number;
    };
  }
): void {
  void tree; // 这个检查要的是"另一个位形下的产物"，不是上面那棵树

  const movable = vm.joints.filter((j) => j.isMovable);
  ok(movable.length > 0, "至少有一个可动关节（否则关节角无从验证）", "");
  if (movable.length === 0) return;

  // 非零测试位形：每个可动关节给不同角度，避免"全一样"掩盖轴/符号错误
  const pose: Record<string, number> = {};
  movable.forEach((j, i) => {
    pose[j.id] = 0.3 + i * 0.25;
  });

  const el = React.createElement(RobotScene, { vm, jointPositions: pose });
  let markup: string;
  try {
    markup = renderToStaticMarkup(el);
  } catch (e) {
    ok(false, "带 jointPositions 的场景可渲染", String(e));
    return;
  }
  const posed = parseMarkup(markup);

  for (const j of movable) {
    const theta = pose[j.id] ?? 0;
    const grp = findGroup([posed], `dof:${j.id}`);
    ok(grp !== null, `关节 ${j.id} 有 dof:<id> 层承载自由度`, "找不到 dof group");
    if (!grp) continue;

    const q = grp.props["quaternion"];
    ok(Array.isArray(q) && q.length === 4, `关节 ${j.id} 的 dof 层有 quaternion`, JSON.stringify(q));
    if (!Array.isArray(q) || q.length !== 4) continue;

    // ⚠️ `setFromAxisAngle` 是**实例方法**（不是静态的）。
    //    上一版直接调 `THREE.Quaternion.setFromAxisAngle(...)` 会抛
    //    "is not a function" —— 所以这里 new 一个再调。
    const want = new QuaternionCtor(0, 0, 0, 1).setFromAxisAngle(axisToThree(j.axis), theta);
    const got = q.map((x) => Number(x));
    const diff = Math.max(
      Math.abs(got[0]! - want.x),
      Math.abs(got[1]! - want.y),
      Math.abs(got[2]! - want.z),
      Math.abs(got[3]! - want.w)
    );
    ok(
      diff <= 1e-12,
      `关节 ${j.id} 的转动 = setFromAxisAngle(axis, ${theta.toFixed(3)})`,
      `渲染产物=[${got.join(",")}] 期望=[${[want.x, want.y, want.z, want.w].join(",")}] 最大差=${diff.toExponential(3)}`
    );
  }

  // ★ 零位形必须**不产生** dof 层的转动（identity）。
  //   反向断言：若零位形也有非恒等旋转，说明"角度没读进来、用了别的默认值"。
  const zeroEl = React.createElement(RobotScene, {
    vm,
    jointPositions: Object.fromEntries(movable.map((j) => [j.id, 0])),
  });
  const zeroTree = parseMarkup(renderToStaticMarkup(zeroEl));
  for (const j of movable) {
    const grp = findGroup([zeroTree], `dof:${j.id}`);
    if (!grp) continue;
    const q = grp.props["quaternion"];
    if (!Array.isArray(q)) continue;
    const isIdentity =
      approx(Number(q[0]), 0) && approx(Number(q[1]), 0) && approx(Number(q[2]), 0) && approx(Number(q[3]), 1);
    ok(isIdentity, `零位形下关节 ${j.id} 是恒等旋转`, `实得 [${q.join(",")}]`);
  }

  // ★ 反例注射：把期望的四元数**故意写错**（用相邻关节的轴），
  //   判据必须变红 —— 否则它没在量"轴"这件事。
  //
  // ⚠️ 必须比较**完整四元数**，不能只比 `w`：
  //    `w = cos(θ/2)` **只与角度有关、与轴无关**。所以"换了个轴"
  //    在 w 上完全看不出来（实测：base_yaw 的 w=0.9887710779 与
  //    换成 shoulder 轴后算出的 w 逐位相同）。第一版就是这么写的，
  //    于是这条"反例注射"永远为红 —— 那不是判据在发现问题，
  //    是**判据自己写错了**。四元数的轴信息在 x/y/z 三个分量上。
  {
    const j0 = movable[0]!;
    const theta = pose[j0.id] ?? 0;
    const wrongAxis: readonly [number, number, number] =
      movable.length > 1 ? movable[1]!.axis : [-j0.axis[0], -j0.axis[1], -j0.axis[2]];
    const wrong = new QuaternionCtor(0, 0, 0, 1).setFromAxisAngle(axisToThree(wrongAxis), theta);
    const grp = findGroup([posed], `dof:${j0.id}`);
    const q = grp?.props["quaternion"];
    const wrongArr = [wrong.x, wrong.y, wrong.z, wrong.w];
    const maxDiff =
      Array.isArray(q) && q.length === 4
        ? Math.max(...wrongArr.map((v, i) => Math.abs(Number(q[i]) - v)))
        : 0;
    ok(
      maxDiff > 1e-6,
      `反例注射：关节 ${j0.id} 用错轴算出的四元数与实际不符（证明判据在量轴）`,
      `与错轴的四元数最大差=${maxDiff.toExponential(3)}（应当明显非零）`
    );
  }

  // ★ 反例注射：故意把角度乘 2，判据必须变红。
  //   这条防的是"判据只验了'有没有转'，没验'转了多少'"。
  {
    const j0 = movable[0]!;
    const theta = pose[j0.id] ?? 0;
    const wrong = new QuaternionCtor(0, 0, 0, 1).setFromAxisAngle(axisToThree(j0.axis), theta * 2);
    const grp = findGroup([posed], `dof:${j0.id}`);
    const q = grp?.props["quaternion"];
    const maxDiff =
      Array.isArray(q) && q.length === 4
        ? Math.max(
            Math.abs(Number(q[0]) - wrong.x),
            Math.abs(Number(q[1]) - wrong.y),
            Math.abs(Number(q[2]) - wrong.z),
            Math.abs(Number(q[3]) - wrong.w)
          )
        : 0;
    ok(
      maxDiff > 1e-6,
      `反例注射：角度差 2 倍时判据能发现（证明判据在量角度）`,
      `与 2θ 的四元数最大差=${maxDiff.toExponential(3)}`
    );
  }

  // ★ §41 未被破坏：外层 group 的 quaternion 仍等于 localTransform 的换算值，
  //   **不**受 jointPositions 影响。这是"自由度没有污染模型静态位姿"的判据。
  for (const j of vm.joints) {
    const outer = findGroup([posed], `joint:${j.id}`);
    if (!outer) continue;
    const q = outer.props["quaternion"];
    if (!Array.isArray(q)) continue;
    // 与零位形（上面那棵树）对比：外层必须**逐位相同**
    const outer0 = findGroup([zeroTree], `joint:${j.id}`);
    const q0 = outer0?.props["quaternion"];
    if (!Array.isArray(q0)) continue;
    const sameOuter = q.every((v, i) => Number(v) === Number(q0[i]));
    ok(
      sameOuter,
      `关节 ${j.id} 外层 group 的朝向与关节角无关（≤ 自由度只在 dof 层）`,
      `有位形=[${q.join(",")}] 零位形=[${q0.join(",")}]`
    );
  }
}

// ---------------------------------------------------------------------------
// 幽灵臂（预演姿态）
// ---------------------------------------------------------------------------

/**
 * `<GhostArm/>` 必须产出**独立的**一棵树，且与权威树同构但姿态不同。
 *
 * ## 为什么"独立"是这里最重要的判据
 *
 * §49 禁止"前端直接改 Three.js"。若幽灵臂与权威臂共用同一批节点，
 * 那么"屏幕上看到的是预测还是真值"就无从区分 —— 预测**事实上**成了
 * 权威姿态的来源，而这正是被禁止的状态。
 *
 * 判据写的是**结构性**的（存在 `ghost-arm` 根、其 `dof:` 层角度不同），
 * 而不是"颜色是不是蓝的" —— 后者是外观，改样式就失效。
 */
function checkGhostArm(
  tree: ElemNode,
  vm: RobotViewModel,
  React: typeof import("react"),
  mod: { GhostArm?: unknown },
  renderToStaticMarkup: (el: React.ReactElement) => string
): void {
  const GhostArm = mod.GhostArm as
    | ((p: { vm: RobotViewModel; jointPositions: Record<string, number> }) => React.ReactNode)
    | undefined;
  ok(typeof GhostArm === "function", "GhostArm 可被导入", `type=${typeof GhostArm}`);
  if (typeof GhostArm !== "function") return;

  const movable = vm.joints.filter((j) => j.isMovable);
  const pose: Record<string, number> = {};
  movable.forEach((j, i) => {
    pose[j.id] = 0.5 + i * 0.2;
  });

  let ghostTree: ElemNode;
  try {
    const el = React.createElement(GhostArm, { vm, jointPositions: pose });
    ghostTree = parseMarkup(renderToStaticMarkup(el));
  } catch (e) {
    ok(false, "GhostArm 可渲染", String(e));
    return;
  }

  // ① 有独立的根 —— 不与权威树共用节点
  const ghostRoot = findGroup([ghostTree], "ghost-arm");
  ok(ghostRoot !== null, "幽灵臂有独立的 `ghost-arm` 根节点", "找不到 ghost-arm group");
  if (!ghostRoot) return;

  // ② 与权威树同构：关节 group 都在（否则幽灵臂画的不是同一台机器人）
  for (const j of vm.joints) {
    ok(
      findGroup([ghostTree], `joint:${j.id}`) !== null,
      `幽灵臂含关节 ${j.id}（与权威树同构）`,
      ""
    );
  }

  // ③ 姿态真的不同：至少一个 dof 层的 quaternion 与零位形不同
  let differs = 0;
  for (const j of movable) {
    const grp = findGroup([ghostTree], `dof:${j.id}`);
    if (!grp) continue;
    const q = grp.props["quaternion"];
    if (!Array.isArray(q)) continue;
    if (!(approx(Number(q[0]), 0) && approx(Number(q[1]), 0) && approx(Number(q[2]), 0) && approx(Number(q[3]), 1))) {
      differs += 1;
    }
  }
  ok(
    differs === movable.length,
    "幽灵臂每个可动关节都按传入位形转动了",
    `转动了 ${differs}/${movable.length} 个`
  );

  // ④ 幽灵臂**不画**轴指示器与 EE 标记 —— 它是"预测"的视觉提示，
  //    带上权威标记会让人误以为那是后端确认过的位姿。
  const ghostAxisCount = collect(ghostRoot.children, true).filter((n) =>
    String(n.props["name"] ?? "").startsWith("axis:")
  ).length;
  ok(ghostAxisCount === 0, "幽灵臂不画关节轴指示器（避免与权威臂混淆）", `找到 ${ghostAxisCount} 个`);
  const ghostEeCount = collect(ghostRoot.children, true).filter((n) =>
    String(n.props["name"] ?? "").startsWith("ee:")
  ).length;
  ok(ghostEeCount === 0, "幽灵臂不画末端执行器标记（它是预测，不是确认过的 TCP）", `找到 ${ghostEeCount} 个`);

  // ⑤ 权威树不受影响：幽灵臂的存在不改变上面那棵树
  //    （两者是分别渲染的，这里断言权威树里**没有** ghost-arm）
  ok(
    findGroup([tree], "ghost-arm") === null,
    "权威树里不含 ghost-arm（两棵树必须分离，§49）",
    "权威树里发现了 ghost-arm —— 预测混进了权威树"
  );
}

// ---------------------------------------------------------------------------

function report(): number {
  console.log("\n" + "=".repeat(74));
  if (failures.length === 0) {
    console.log(`✅ 渲染树结构正确：全部 ${checks} 项检查通过`);
    console.log("=".repeat(74));
    return 0;
  }
  console.log(`❌ 渲染树结构有问题：${failures.length} 项失败（共 ${checks} 项）`);
  for (const f of failures) {
    console.log(`\n  ✗ ${f.check}`);
    if (f.detail) console.log(`      ${f.detail}`);
  }
  console.log("=".repeat(74));
  return 1;
}

main().then(
  (code) => process.exit(code),
  (e: unknown) => {
    console.error("检查过程抛异常：", e);
    process.exit(2);
  }
);
