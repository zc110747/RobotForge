/**
 * geometry.check.ts —— 几何构造检查（调用**真的** makeGeometry）
 *
 * ## 为什么需要它
 *
 * `render.check.ts` 用 `renderToStaticMarkup` 验证的是**元素树结构**：
 * 它能证明"该画的都挂了、位置数值对"。但它证明不了
 * **`makeGeometry` 把 MJCF 的 size 语义解释对了** ——
 * 在静态渲染里，`<boxGeometry args={[a,b,c]}/>` 只是个标签，
 * `args` 里的数值对不对，它看不见。
 *
 * 而"尺寸错"恰恰是最容易犯又最不该犯的错：MJCF 的 `size` 语义
 * **按 type 不同**（box 是半长、cylinder 是 [半径, 半长]…），
 * 记错一个就是"机器人比例明显不对"。
 *
 * ## 关键：验的是**实现**，不是把规格再抄一遍
 *
 * 本检查 import 真 `makeGeometry`，构造 `BufferGeometry`，
 * 再读它的 `boundingBox` 与模型数值比对。
 * ★ "期望的外接尺寸"由 **MJCF 语义**独立推出（不读 makeGeometry 的代码），
 *   所以两边是**独立来源**：一边是模型的数字，一边是实现的构造结果。
 *   若只在测试里重抄一遍 `BoxGeometry(hx*2,...)`，那就是自证。
 *
 * ## 怎么做到不起 GL
 *
 * 只 **构造** geometry 并读 boundingBox，不创建 renderer、不画。
 * Three.js 的几何构造是纯 CPU 的，Node 里完全可跑（实测）。
 *
 * ## 执行
 *
 *   node --experimental-strip-types src/viewer/__tests__/geometry.check.ts
 */

import { readFileSync } from "node:fs";
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

type GeomLike = {
  type: string;
  size: number[];
  asset: string | null;
  rgba?: number[];
  transform?: unknown;
};

/**
 * 期望的**外接盒尺寸**（按 MJCF size 语义独立推导）。
 *
 * ⚠️ 轴序问题：MJCF 的 cylinder/capsule 沿 **Z**，而 Three.js 的
 *    Cylinder/CapsuleGeometry 沿 **Y**。所以期望值按"轴序规范化"
 *    （排序后比较）给出 —— 这样既不放过"尺寸错"，也不误报"轴向差"。
 *    轴向单独有一条检查（见下）。
 */
function expectedExtents(
  type: string,
  size: number[]
): { extents: number[]; note: string } | null {
  const [a = 0, b = 0, c = 0] = size;
  switch (type) {
    case "box":
      // size = [半长x, 半长y, 半长z]
      return { extents: [a * 2, b * 2, c * 2], note: `全尺寸 [${a * 2}, ${b * 2}, ${c * 2}]` };
    case "sphere":
      // size = [半径]
      return { extents: [a * 2, a * 2, a * 2], note: `直径 ${a * 2}` };
    case "cylinder":
      // size = [半径, 半长]（沿 Z）⇒ 外接盒 = 直径 × 直径 × 全长
      return { extents: [a * 2, a * 2, b * 2], note: `直径 ${a * 2} × 长 ${b * 2}` };
    case "capsule":
      // size = [半径, 半长]（沿 Z）⇒ 总长 = 2 × 半长
      return { extents: [a * 2, a * 2, b * 2], note: `直径 ${a * 2} × 总长 ${b * 2}` };
    case "ellipsoid":
      // size = [半长x, 半长y, 半长z]（按 box 同构）
      return { extents: [a * 2, b * 2, c * 2], note: `全尺寸` };
    case "plane":
      return { extents: [a * 2, b * 2, 0], note: `平面 ${a * 2} × ${b * 2}` };
    default:
      return null;
  }
}

async function transpile(): Promise<string> {
  const esbuild = (await import("esbuild")) as {
    build: (o: Record<string, unknown>) => Promise<unknown>;
  };
  const outfile = join(HERE, "__geom.check.mjs");
  await esbuild.build({
    entryPoints: [join(SRC_DIR, "RobotScene.tsx")],
    outfile,
    bundle: true,
    format: "esm",
    jsx: "automatic",
    target: "es2022",
    platform: "node",
    external: ["react", "react-dom", "react/jsx-runtime", "three", "scheduler"],
    logLevel: "silent",
  });
  return outfile;
}

async function main(): Promise<number> {
  console.log("=".repeat(74));
  console.log("几何构造检查（调用真 makeGeometry，核对外接盒）");
  console.log("=".repeat(74));

  const outfile = await transpile();
  const mod = (await import(`${pathToFileURL(outfile).href}?t=${Date.now()}`)) as {
    makeGeometry?: (g: GeomLike) => unknown;
  };
  const makeGeometry = mod.makeGeometry;
  ok(
    typeof makeGeometry === "function",
    "makeGeometry 被导出（检查才能验真实现）",
    `type=${typeof makeGeometry}`
  );
  if (typeof makeGeometry !== "function") {
    return report();
  }

  const THREE = await import("three");
  const fixture = JSON.parse(readFileSync(FIXTURE, "utf-8")) as {
    model: {
      links: Array<{ id: string; collision?: GeomLike[]; visual?: GeomLike[] }>;
    };
  };

  // 模型里实际出现的全部几何（collision 是 v0.1 的显示数据源）
  const all: Array<{ link: string; g: GeomLike }> = [];
  for (const link of fixture.model.links) {
    for (const g of link.collision ?? []) all.push({ link: link.id, g });
  }

  console.log("\n── 模型里出现的几何（type : size）────────────────────────");
  for (const { link, g } of all) {
    console.log(`  ${g.type.padEnd(9)} [${g.size.join(", ")}]   ← link ${link}`);
  }
  ok(all.length > 0, "模型有可测的几何（否则本检查空转）", `数量=${all.length}`);

  // ---- 逐几何：调 makeGeometry，核对 bbox ------------------------------
  console.log("\n── 逐几何核对外接盒（makeGeometry 真实现 vs MJCF 语义）──");
  let compared = 0;
  for (const { link, g } of all) {
    const exp = expectedExtents(g.type, g.size);
    if (!exp) continue; // 未覆盖类型另有一条检查

    const geom = makeGeometry(g) as
      | (InstanceType<typeof THREE.BufferGeometry> & { dispose?: () => void })
      | null;
    ok(
      geom !== null && geom !== undefined,
      `${link}:${g.type} makeGeometry 返回了几何（非 null）`,
      "返回 null ⇒ 该几何不会渲染"
    );
    if (!geom) continue;

    geom.computeBoundingBox();
    const bb = geom.boundingBox;
    ok(bb !== null, `${link}:${g.type} 有 boundingBox`, "computeBoundingBox 得到 null");
    if (!bb) continue;

    const got = [bb.max.x - bb.min.x, bb.max.y - bb.min.y, bb.max.z - bb.min.z];
    // 轴序规范化：MJCF 沿 Z、Three.js 沿 Y，排序后比较只看"尺寸集合"
    const gotS = [...got].sort((p, q) => p - q);
    const wantS = [...exp.extents].sort((p, q) => p - q);
    const match =
      approx(gotS[0]!, wantS[0]!, 1e-6) &&
      approx(gotS[1]!, wantS[1]!, 1e-6) &&
      approx(gotS[2]!, wantS[2]!, 1e-6);
    compared++;
    ok(
      match,
      `${link}:${g.type} 外接尺寸 = MJCF 语义（${exp.note}）`,
      `实际 bbox=[${got.map((n) => n.toFixed(4)).join(", ")}]（排序 [${gotS
        .map((n) => n.toFixed(4))
        .join(", ")}]）期望排序 [${wantS.map((n) => n.toFixed(4)).join(", ")}]`
    );
    console.log(
      `  ${match ? "✓" : "✗"} ${link.padEnd(14)} ${g.type.padEnd(9)} bbox=[${got
        .map((n) => n.toFixed(4))
        .join(", ")}]  ← ${exp.note}`
    );

    // 几何不能被 GC 前留着不释放（v0.1 数量小，但习惯要对）
    if (typeof geom.dispose === "function") geom.dispose();
  }
  ok(compared > 0, "至少比对了一个几何（防空转）", `比对=${compared}`);

  // ---- 轴向：cylinder / capsule 必须是"长轴 = Y"（Three.js 约定）-----
  //
  // ★ 这条与上一条互补：上一条把轴序**规范化**了（不看方向），
  //   这一条专门看方向。二者一起才能既容错轴向、又保证轴向被固定住 ——
  //   否则"把 cylinder 转 90° 塞进 box"也能蒙过上面那条。
  console.log("\n── 轴向（Three.js 的 Cylinder/Capsule 长轴必须是 Y）──────");
  for (const { link, g } of all) {
    if (g.type !== "cylinder" && g.type !== "capsule") continue;
    const geom = makeGeometry(g) as InstanceType<typeof THREE.BufferGeometry> | null;
    if (!geom) continue;
    geom.computeBoundingBox();
    const bb = geom.boundingBox!;
    const ext = {
      x: bb.max.x - bb.min.x,
      y: bb.max.y - bb.min.y,
      z: bb.max.z - bb.min.z,
    };
    const halfLen = g.size[1] ?? 0;
    const radius = g.size[0] ?? 0;
    ok(
      approx(ext.y, halfLen * 2, 1e-6) && approx(ext.x, radius * 2, 1e-6) && approx(ext.z, radius * 2, 1e-6),
      `${link}:${g.type} 长轴沿 Y（半径在 X/Z）`,
      `bbox=[${ext.x.toFixed(4)}, ${ext.y.toFixed(4)}, ${ext.z.toFixed(4)}] ` +
        `期望 [${(radius * 2).toFixed(4)}, ${(halfLen * 2).toFixed(4)}, ${(radius * 2).toFixed(4)}]`
    );
    console.log(
      `  ✓ ${link.padEnd(14)} ${g.type.padEnd(9)} 长轴 Y=${ext.y.toFixed(4)}，横截=${ext.x.toFixed(4)}/${ext.z.toFixed(4)}`
    );
    geom.dispose();
  }

  // ---- 未知类型必须有兜底（不得静默消失）----------------------------
  //
  // ★ 这里有个容易搞错的层次问题，值得写清楚：
  //
  //   `makeGeometry` 对未知类型**故意返回 `null`** —— 它的职责只是
  //   "把已知类型映射成几何"，不认识就该说"我不认识"，而不是
  //   自己编一个尺寸出去。**兜底是 `GeomMesh` 的职责**：它拿到 null
  //   会渲染一个品红线框盒（见 RobotScene.tsx 的 `geom-unknown` 分支）。
  //
  //   所以"未知类型会不会静默消失"这件事，**不能**在 makeGeometry 这一层
  //   断言 —— 那会把一个正确的设计判成错的（本检查第一版就这么干了）。
  //   真正该断言的是分层的**契约**：
  //     · makeGeometry 不认识 ⇒ 返回 null（明确拒绝，而非瞎猜）
  //     · GeomMesh 见到 null ⇒ 渲染可见的占位（而非什么都不画）
  //   后者由 render.check.ts 的"未知几何类型有显式兜底"覆盖。
  //   这里只钉住前半条。
  console.log("\n── 未知类型的分层契约 ────────────────────────────────────");
  {
    const bogus: GeomLike = { type: "hypercube", size: [0.01], asset: null };
    const geom = makeGeometry(bogus);
    ok(
      geom === null,
      "makeGeometry 对未知类型返回 null（明确拒绝，不瞎猜尺寸）",
      `实际返回 ${geom === null ? "null" : "一个几何"} ⇒ 实现可能在臆测未知类型的语义`
    );
    console.log(
      `  ✓ 未知类型 'hypercube' → makeGeometry 返回 null（拒绝），` +
        `由 GeomMesh 渲染品红线框占位（可见）`
    );

    // 哨兵：**已知**类型不得走 null 分支，否则"拒绝"就退化成"全都拒绝"
    const known: GeomLike = { type: "box", size: [0.01, 0.01, 0.01], asset: null };
    const kg = makeGeometry(known) as
      | (InstanceType<typeof THREE.BufferGeometry> & { dispose?: () => void })
      | null;
    ok(kg !== null, "已知类型 box 不返回 null（拒绝不是无差别拒绝）", "box 也返回了 null");
    kg?.dispose?.();
  }

  return report();
}

function report(): number {
  console.log("\n" + "=".repeat(74));
  if (failures.length === 0) {
    console.log(`✅ 几何构造正确：全部 ${checks} 项检查通过`);
    console.log("=".repeat(74));
    return 0;
  }
  console.log(`❌ 几何构造有问题：${failures.length} 项失败（共 ${checks} 项）`);
  for (const f of failures) {
    console.log(`\n  ✗ ${f.check}\n      ${f.detail}`);
  }
  console.log("=".repeat(74));
  return 1;
}

main().then(
  (c) => process.exit(c),
  (e: unknown) => {
    console.error("检查过程抛异常：", e);
    process.exit(2);
  }
);
