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
    RobotScene: (p: { vm: RobotViewModel }) => React.ReactNode;
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

  // ★ 元数据属性必须落在**字符串**字段上（`data-kind` / `data-id`）。
  //
  // 为什么单独查这个：本文件曾依赖 `userData={{kind, id}}`，但对象属性在
  // 静态渲染里退化成 `[object Object]` ⇒ 拿不到 kind/id。改用字符串属性后
  // 它们原样出现在产物里，**可以被核对**。这条断言就是在钉住这点：
  // 谁把 data-kind 改回对象，这里立刻变红。
  for (const n of vm.nodes) {
    if (n.kind === "world") continue;
    const grp = findGroup([tree], n.key);
    if (!grp) continue;
    ok(
      grp.props["data-kind"] === n.kind,
      `${n.key} 带 data-kind=${n.kind}（可序列化，非 [object Object]）`,
      `实际 ${JSON.stringify(grp.props["data-kind"])}`
    );
    ok(
      grp.props["data-id"] === n.id,
      `${n.key} 带 data-id=${n.id}`,
      `实际 ${JSON.stringify(grp.props["data-id"])}`
    );
  }

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
  // 判据是"关节 group 下有没有 name 以 `axis:` 开头的 group"。
  // ⚠️ 不要用"子 group 个数"当判据 —— 关节下**同时**有一个子 group 承载
  //    子 link，所以可动关节本来就有 2 个子 group（轴 + 子链路），
  //    fixed 关节有 1 个。按个数判断会**把正确的实现判成错的**
  //    （本文件第一版就踩了这个）。
  for (const j of vm.joints) {
    const node = findGroup([tree], `joint:${j.id}`);
    if (!node) continue;
    const axisGroups = node.children.filter(
      (c) => c.name === "group" && String(c.props["name"] ?? "").startsWith("axis:")
    );
    if (j.isMovable) {
      ok(
        axisGroups.length === 1,
        `可动关节 ${j.id} 有轴指示器`,
        `找到 ${axisGroups.length} 个 axis:* 组；子：${node.children.map((c) => c.name).join(", ")}`
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
