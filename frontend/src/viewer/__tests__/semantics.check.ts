/**
 * =============================================================================
 * semantics.check.ts —— RobotModel ↔ Renderer 语义一致性检查（§59 最后一条）
 * -----------------------------------------------------------------------------
 * 运行方式（Node 原生 TypeScript 剥离，无构建步骤）：
 *
 *   node --experimental-strip-types src/viewer/__tests__/semantics.check.ts
 *
 * 或由 `tools/accept_phase2.py` 调用。
 *
 * ## 它到底在验证什么
 *
 * 输入是 `fixtures/model.json`，里面有两部分：
 *   - `model`  ：RobotModel 的**真实**序列化（由后端 pytest 导出，不是手抄）
 *   - `expect` ：**Python 侧独立推导**的渲染事实
 *
 * 检查把 `buildViewModel(model)` 的结果与 `expect` 逐字段比对。
 *
 * ★ 关键：`expect` **不是**由本文件的逻辑生成的，也不是由 `viewModel.ts`
 *   生成的 —— 它来自 `tools/export_view_fixture.py` 里另写一遍的推导。
 *   两套独立实现得出同一结论，才叫"语义一致"。
 *   如果这里只是把 TS 的输出打印出来，测试永远绿而什么都没验证。
 *
 * ## 自检（防"永远绿"）
 *
 * 见文件末尾的 `runSelfTests()`：它会故意篡改输入，
 * 断言"检查**会**失败"。没有这一步，一个写错的检查会显得比没有检查更可信。
 */

import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

import { buildViewModel, depthFirstOrder, jointChainTo, linkKey } from "../viewModel.ts";
import type { RobotModelDTO } from "../model.ts";
import {
  AXIOMS,
  SCENE_ROTATION_X,
  toThreeWorld,
  fromThreeWorld,
  quaternionToThree,
  axisToThree,
  positionToThree,
} from "../coordinateAdapter.ts";

// ---------------------------------------------------------------------------
// 极小的断言框架（避免引入测试依赖）
// ---------------------------------------------------------------------------
//
// ★ 为什么 failures 是**每次调用新建**的，而不是模块级全局
//
// 最初把它写成模块级 `const failures: Failure[] = []`，结果 `runChecks()`
// 被自检反复调用时，**各次调用的失败项混进了同一个数组**：
// 主检查显示"10 项失败"，而单独调用同一函数却 0 失败。
// 更糟的是：自检注入的缺陷会被算进主检查的失败数，
// 于是"自检通过了"与"主检查失败了"两个相反的结论会同时出现。
//
// 教训：一个**返回诊断结果**的函数，绝不能让诊断结果活在共享可变状态里。
// 这与 backend 的 validator 契约是同一条道理（见 skill 第 ② 节）。

interface Failure {
  readonly check: string;
  readonly detail: string;
}

interface Checker {
  readonly failures: Failure[];
  count: number;
  ok(cond: boolean, check: string, detail: string): void;
}

function newChecker(): Checker {
  const c: Checker = {
    failures: [],
    count: 0,
    ok(cond, check, detail) {
      c.count++;
      if (!cond) c.failures.push({ check, detail });
    },
  };
  return c;
}

function deepEqual(a: unknown, b: unknown): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}

function numEq(a: number, b: number, tol = 1e-12): boolean {
  return Math.abs(a - b) <= tol;
}

// ---------------------------------------------------------------------------
// 载入 fixture
// ---------------------------------------------------------------------------

const HERE = dirname(fileURLToPath(import.meta.url));
const FIXTURE = join(HERE, "fixtures", "model.json");

interface Expect {
  linkCount: number;
  jointCount: number;
  movableJointCount: number;
  movableJointIds: string[];
  linkParent: Record<string, string>;
  jointParent: Record<string, string>;
  depthFirstOrder: string[];
  axes: Record<string, number[]>;
  jointOrigins: Record<string, number[]>;
  endEffectors: { id: string; link: string; position: number[]; site: string }[];
  frames: { id: string; parent: string }[];
  geometry: Record<string, { type: string; size: number[]; position: number[] }[]>;
  rootLink: string | null;
  baseFrame: string | null;
  dof: number;
}

function loadFixture(): { model: RobotModelDTO; expect: Expect } {
  const raw = JSON.parse(readFileSync(FIXTURE, "utf-8")) as {
    robotId: string;
    model: RobotModelDTO;
    expect: Expect;
  };
  return { model: raw.model, expect: raw.expect };
}

// ---------------------------------------------------------------------------
// 主检查
// ---------------------------------------------------------------------------

export interface CheckOutcome {
  readonly failures: readonly Failure[];
  /** 执行的检查项数 */
  readonly checks: number;
}

export function runChecks(model: RobotModelDTO, expect: Expect): CheckOutcome {
  const c = newChecker();
  const { ok } = c;

  const { viewModel: vm, problems } = buildViewModel(model);

  // --- 0. 构建过程本身不得报问题 ------------------------------------------
  ok(
    problems.length === 0,
    "buildViewModel 无 problem",
    `构建视图模型时报了 ${problems.length} 个问题：\n    ${problems.join("\n    ")}`
  );

  // --- 1. 数量 -------------------------------------------------------------
  const linkNodes = vm.nodes.filter((n) => n.kind === "link");
  const jointNodes = vm.nodes.filter((n) => n.kind === "joint");
  ok(
    linkNodes.length === expect.linkCount,
    "link 节点数",
    `TS=${linkNodes.length} Python=${expect.linkCount}`
  );
  ok(
    jointNodes.length === expect.jointCount,
    "joint 节点数",
    `TS=${jointNodes.length} Python=${expect.jointCount}`
  );

  // ★ 节点总数 = 1(world) + links + joints
  ok(
    vm.nodes.length === 1 + expect.linkCount + expect.jointCount,
    "节点总数 = 1 + links + joints",
    `TS=${vm.nodes.length} 期望=${1 + expect.linkCount + expect.jointCount}`
  );

  // --- 2. 层级：link 的父 --------------------------------------------------
  for (const [linkId, wantParent] of Object.entries(expect.linkParent)) {
    const node = vm.nodes.find((n) => n.kind === "link" && n.id === linkId);
    ok(node !== undefined, `link ${linkId} 存在`, "TS 里找不到该 link 节点");
    if (node) {
      ok(
        node.parentKey === wantParent,
        `link ${linkId} 的父`,
        `TS=${node.parentKey!} Python=${wantParent}`
      );
    }
  }

  // --- 3. 层级：joint 的父 -------------------------------------------------
  for (const [jointId, wantParent] of Object.entries(expect.jointParent)) {
    const node = vm.nodes.find((n) => n.kind === "joint" && n.id === jointId);
    ok(node !== undefined, `joint ${jointId} 存在`, "TS 里找不到该 joint 节点");
    if (node) {
      ok(
        node.parentKey === wantParent,
        `joint ${jointId} 的父`,
        `TS=${node.parentKey!} Python=${wantParent}`
      );
    }
  }

  // --- 4. 深度优先顺序 -----------------------------------------------------
  const tsOrder = depthFirstOrder(vm);
  ok(
    deepEqual(tsOrder, expect.depthFirstOrder),
    "深度优先顺序",
    `\n    TS    =${JSON.stringify(tsOrder)}\n    Python=${JSON.stringify(expect.depthFirstOrder)}`
  );

  // --- 5. 关节轴（逐分量）--------------------------------------------------
  for (const [jointId, wantAxis] of Object.entries(expect.axes)) {
    const jv = vm.joints.find((j) => j.id === jointId);
    ok(jv !== undefined, `关节 ${jointId} 在 joints[] 里`, "找不到");
    if (jv) {
      const same =
        jv.axis.length === wantAxis.length &&
        jv.axis.every((c, i) => numEq(c, wantAxis[i]!));
      ok(
        same,
        `关节 ${jointId} 的轴`,
        `TS=${JSON.stringify(jv.axis)} Python=${JSON.stringify(wantAxis)}`
      );
    }
    // 同时检查节点里的 axis（渲染器实际读的是这个）
    const node = vm.nodes.find((n) => n.kind === "joint" && n.id === jointId);
    ok(node?.axis !== null && node?.axis !== undefined, `关节节点 ${jointId} 有轴`, "为 null");
    if (node?.axis) {
      const same =
        node.axis.length === wantAxis.length &&
        node.axis.every((c, i) => numEq(c, wantAxis[i]!));
      ok(
        same,
        `关节节点 ${jointId} 的轴`,
        `TS=${JSON.stringify(node.axis)} Python=${JSON.stringify(wantAxis)}`
      );
    }
  }

  // --- 6. 关节 origin（唯一的位移来源）------------------------------------
  for (const [jointId, wantPos] of Object.entries(expect.jointOrigins)) {
    const node = vm.nodes.find((n) => n.kind === "joint" && n.id === jointId);
    if (!node) continue;
    const same =
      node.localTransform.position.length === wantPos.length &&
      node.localTransform.position.every((c, i) => numEq(c, wantPos[i]!));
    ok(
      same,
      `关节 ${jointId} 的 origin`,
      `TS=${JSON.stringify(node.localTransform.position)} Python=${JSON.stringify(wantPos)}`
    );
  }

  // --- 7. 可动关节 ---------------------------------------------------------
  ok(
    deepEqual([...vm.movableJointIds], expect.movableJointIds),
    "可动关节集合",
    `TS=${JSON.stringify(vm.movableJointIds)} Python=${JSON.stringify(expect.movableJointIds)}`
  );
  ok(
    vm.movableJointIds.length === expect.movableJointCount,
    "可动关节数",
    `TS=${vm.movableJointIds.length} Python=${expect.movableJointCount}`
  );

  // --- 8. 末端执行器 -------------------------------------------------------
  ok(
    vm.endEffectors.length === expect.endEffectors.length,
    "末端执行器数量",
    `TS=${vm.endEffectors.length} Python=${expect.endEffectors.length}`
  );
  for (const want of expect.endEffectors) {
    const got = vm.endEffectors.find((e) => e.id === want.id);
    ok(got !== undefined, `末端执行器 ${want.id} 存在`, "TS 里找不到");
    if (!got) continue;
    ok(got.linkId === want.link, `EE ${want.id} 所在 link`,
      `TS=${got.linkId} Python=${want.link}`);
    const same =
      got.transform.position.length === want.position.length &&
      got.transform.position.every((c, i) => numEq(c, want.position[i]!));
    ok(same, `EE ${want.id} 位置`,
      `TS=${JSON.stringify(got.transform.position)} Python=${JSON.stringify(want.position)}`);
  }

  // --- 9. 坐标架 -----------------------------------------------------------
  ok(
    vm.frames.length === expect.frames.length,
    "坐标架数量",
    `TS=${vm.frames.length} Python=${expect.frames.length}`
  );
  for (const want of expect.frames) {
    const got = vm.frames.find((f) => f.id === want.id);
    ok(got !== undefined, `坐标架 ${want.id} 存在`, "TS 里找不到");
    if (got) {
      ok(got.parentId === want.parent, `坐标架 ${want.id} 的父`,
        `TS=${got.parentId} Python=${want.parent}`);
    }
  }
  // world 坐标架必须存在
  ok(
    vm.frames.some((f) => f.isWorld && f.id === "world"),
    "world 坐标架存在",
    `frames=${JSON.stringify(vm.frames.map((f) => f.id))}`
  );

  // --- 10. 几何（每个 link 的显示体）--------------------------------------
  for (const [linkId, wantGeoms] of Object.entries(expect.geometry)) {
    const node = vm.nodes.find((n) => n.kind === "link" && n.id === linkId);
    if (!node) {
      ok(false, `link ${linkId} 的几何`, "TS 里找不到该 link 节点");
      continue;
    }
    ok(
      node.geometries.length === wantGeoms.length,
      `link ${linkId} 的几何数量`,
      `TS=${node.geometries.length} Python=${wantGeoms.length}`
    );
    wantGeoms.forEach((wg, i) => {
      const g = node.geometries[i];
      if (!g) return;
      ok(g.type === wg.type, `link ${linkId} 几何[${i}] 类型`,
        `TS=${g.type} Python=${wg.type}`);
      const sameSize =
        g.size.length === wg.size.length &&
        g.size.every((c, k) => numEq(c, wg.size[k]!));
      ok(sameSize, `link ${linkId} 几何[${i}] 尺寸`,
        `TS=${JSON.stringify(g.size)} Python=${JSON.stringify(wg.size)}`);
      const samePos =
        g.transform.position.length === wg.position.length &&
        g.transform.position.every((c, k) => numEq(c, wg.position[k]!));
      ok(samePos, `link ${linkId} 几何[${i}] 位置`,
        `TS=${JSON.stringify(g.transform.position)} Python=${JSON.stringify(wg.position)}`);
    });
  }

  // --- 11. 关节链（从 world 到 ee_link）----------------------------------
  const chain = jointChainTo(vm, linkKey("ee_link"));
  ok(
    deepEqual([...chain], ["base_yaw", "shoulder", "elbow", "ee_link_fixed"]),
    "world→ee_link 的关节链",
    `TS=${JSON.stringify(chain)}`
  );
  // 链上的每个节点都必须是 joint
  for (const jid of chain) {
    const node = jointNodeOf(vm, jid);
    ok(node?.kind === "joint", `链上 ${jid} 是 joint 节点`, `kind=${node?.kind}`);
  }

  // --- 12. 坐标适配层公理 -------------------------------------------------
  runAdapterAxioms(c, expect);

  return { failures: c.failures, checks: c.count };
}

function jointNodeOf(vm: ReturnType<typeof buildViewModel>["viewModel"], id: string) {
  return vm.nodes.find((n) => n.kind === "joint" && n.id === id);
}

// ---------------------------------------------------------------------------
// 坐标适配层公理（§42 必测项）
// ---------------------------------------------------------------------------

function runAdapterAxioms(c: Checker, _expect: Expect): void {
  const { ok } = c;
  // ① 场景旋转必须恰好是 -π/2
  ok(
    numEq(SCENE_ROTATION_X, -Math.PI / 2),
    "场景旋转 = -π/2",
    `实际 ${SCENE_ROTATION_X}`
  );
  ok(
    numEq(AXIOMS.sceneRotationX, -Math.PI / 2),
    "AXIOMS.sceneRotationX = -π/2",
    `实际 ${AXIOMS.sceneRotationX}`
  );

  // ② Z-up → Three.js Y-up：机器人的 +Z 必须落到 Three.js 的 +Y
  {
    const up = toThreeWorld({ x: 0, y: 0, z: 1 });
    ok(
      numEq(up.x, AXIOMS.upAxisMapsTo.x) &&
        numEq(up.y, AXIOMS.upAxisMapsTo.y) &&
        numEq(up.z, AXIOMS.upAxisMapsTo.z),
      "RobotForge +Z → Three.js +Y",
      `实际 ${JSON.stringify(up)}`
    );
  }

  // ③ 完整映射 (x,y,z) → (x, z, -y)，逐轴验证（不是只看一个特例）
  {
    const cases: [number, number, number][] = [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
      [0.103, 0.0, 0.084],
      [-0.5, 0.25, -1.75],
    ];
    for (const [x, y, z] of cases) {
      const got = toThreeWorld({ x, y, z });
      const want = { x, y: z, z: -y };
      ok(
        numEq(got.x, want.x) && numEq(got.y, want.y) && numEq(got.z, want.z),
        `toThreeWorld(${x},${y},${z})`,
        `TS=${JSON.stringify(got)} 期望=${JSON.stringify(want)}`
      );
      // 逆变换必须还原
      const back = fromThreeWorld(got);
      ok(
        numEq(back.x, x) && numEq(back.y, y) && numEq(back.z, z),
        `fromThreeWorld∘toThreeWorld(${x},${y},${z}) = 恒等`,
        `实际 ${JSON.stringify(back)}`
      );
    }
  }

  // ④ 四元数恒等映射 —— 顺序 [x,y,z,w] 与 Three.js 一致
  {
    const q = quaternionToThree([0, 0, 0, 1]);
    ok(
      numEq(q.x, 0) && numEq(q.y, 0) && numEq(q.z, 0) && numEq(q.w, 1),
      "恒等四元数 [0,0,0,1] 直通",
      `实际 x=${q.x} y=${q.y} z=${q.z} w=${q.w}`
    );
    // 非恒等：分量必须**原位**映射，不得重排
    const q2 = quaternionToThree([0.1, 0.2, 0.3, 0.9273618495495703]);
    ok(
      numEq(q2.x, 0.1) && numEq(q2.y, 0.2) && numEq(q2.z, 0.3) &&
        numEq(q2.w, 0.9273618495495703),
      "非恒等四元数分量原位映射（无重排）",
      `实际 x=${q2.x} y=${q2.y} z=${q2.z} w=${q2.w}`
    );
    ok(
      numEq(AXIOMS.identityQuaternion[3]!, 1),
      "AXIOMS.identityQuaternion 第 4 位是 w",
      `实际 ${JSON.stringify(AXIOMS.identityQuaternion)}`
    );
  }

  // ⑤ 轴是**局部**的：不得把 axisToThree 与 toThreeWorld 混用
  {
    const a = axisToThree([0, 0, 1]);
    ok(
      numEq(a.x, 0) && numEq(a.y, 0) && numEq(a.z, 1),
      "axisToThree 保持局部（+Z 仍是 +Z）",
      `实际 ${a.x},${a.y},${a.z}`
    );
    // 与 toThreeWorld 对比：两者**必须**不同，否则说明有人把轴也做了世界转换
    const worldConverted = toThreeWorld({ x: 0, y: 0, z: 1 });
    ok(
      !(numEq(worldConverted.x, 0) && numEq(worldConverted.y, 0) && numEq(worldConverted.z, 1)),
      "轴**不应**与世界位置转换同构",
      `toThreeWorld([0,0,1]) = ${JSON.stringify(worldConverted)} —— 若它也返回 (0,0,1)，说明两个函数被合并了，关节会转错方向`
    );
  }

  // ⑥ 位置转 Vector3 与 toThreeWorld 必须一致（防两份实现漂移）
  {
    const cases: [number, number, number][] = [
      [0, 0, 0],
      [0.103, 0, 0],
      [0, 0, 0.084],
    ];
    for (const p of cases) {
      const v = positionToThree(p);
      const w = toThreeWorld({ x: p[0], y: p[1], z: p[2] });
      ok(
        numEq(v.x, w.x) && numEq(v.y, w.y) && numEq(v.z, w.z),
        `positionToThree 与 toThreeWorld 一致 @ ${JSON.stringify(p)}`,
        `positionToThree=${v.x},${v.y},${v.z}  toThreeWorld=${JSON.stringify(w)}`
      );
    }
  }
}

// ---------------------------------------------------------------------------
// 自检：证明这个检查**会**失败（防"永远绿"）
// ---------------------------------------------------------------------------

interface SelfTest {
  readonly name: string;
  readonly run: () => { failed: number; detail: string };
}

const SELF_TESTS: SelfTest[] = [
  {
    name: "篡改关节轴 ⇒ 检查必须报错",
    run: () => {
      const { model, expect } = loadFixture();
      // 把 elbow 的轴从 (0,1,0) 改成 (0,0,1) —— 这正是"轴的语义被写错"的典型
      const mutated = {
        ...model,
        joints: model.joints.map((j) =>
          j.id === "elbow" ? { ...j, axis: [0, 0, 1] as const } : j
        ),
      };
      return { failed: runChecks(mutated as RobotModelDTO, expect).failures.length, detail: "elbow 轴被改" };
    },
  },
  {
    name: "篡改父子关系（把 upper_arm 挂到 base 下）⇒ 检查必须报错",
    run: () => {
      const { model, expect } = loadFixture();
      const mutated = {
        ...model,
        joints: model.joints.map((j) =>
          j.id === "shoulder" ? { ...j, parent_link: "base" } : j
        ),
      };
      return { failed: runChecks(mutated as RobotModelDTO, expect).failures.length, detail: "shoulder 的父被改" };
    },
  },
  {
    name: "篡改末端执行器位置 ⇒ 检查必须报错",
    run: () => {
      const { model, expect } = loadFixture();
      const mutated = {
        ...model,
        frames: model.frames.map((f) =>
          f.id === "tcp_frame"
            ? { ...f, transform: { ...f.transform, position: [0.5, 0, 0] as const } }
            : f
        ),
      };
      return { failed: runChecks(mutated as RobotModelDTO, expect).failures.length, detail: "tcp_frame 位置被改" };
    },
  },
  {
    name: "篡改 depth-first 顺序前提（删掉一个 link）⇒ 检查必须报错",
    run: () => {
      const { model, expect } = loadFixture();
      const mutated = {
        ...model,
        links: model.links.filter((l) => l.id !== "upper_arm"),
        joints: model.joints.filter((j) => j.id !== "shoulder" && j.id !== "elbow"),
      };
      return { failed: runChecks(mutated as RobotModelDTO, expect).failures.length, detail: "删掉了 upper_arm" };
    },
  },
];

function runSelfTests(): Failure[] {
  const out: Failure[] = [];
  console.log("\n── 自检：证明检查会失败 ──────────────────────────────────");
  for (const st of SELF_TESTS) {
    let failed = 0;
    let err: string | null = null;
    try {
      failed = st.run().failed;
    } catch (e) {
      err = e instanceof Error ? e.message : String(e);
    }
    if (err !== null) {
      // 抛异常也算"发现了问题"，但要区分开
      console.log(`  ✓ ${st.name}：检出（抛错：${err.slice(0, 80)}）`);
      continue;
    }
    if (failed > 0) {
      console.log(`  ✓ ${st.name}：检出 ${failed} 处不一致`);
    } else {
      console.log(`  ✗ ${st.name}：**没检出** —— 这是假检查！`);
      out.push({
        check: `自检 ${st.name}`,
        detail: "注入的缺陷没有被检出，说明该维度的检查是空的",
      });
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// 入口
// ---------------------------------------------------------------------------

function main(): number {
  const { model, expect } = loadFixture();
  console.log("=".repeat(74));
  console.log("RobotModel ↔ Renderer 语义一致性检查");
  console.log("=".repeat(74));
  console.log(`robot: ${model.metadata.id}`);
  console.log(`links=${expect.linkCount} joints=${expect.jointCount} ` +
    `movable=${expect.movableJointCount} dof=${expect.dof}`);

  const result = runChecks(model, expect);
  const selfFailures = runSelfTests();
  const all = [...result.failures, ...selfFailures];

  console.log("\n" + "=".repeat(74));
  if (all.length === 0) {
    console.log(
      `✅ 语义一致：全部 ${result.checks} 项检查通过，自检 ${SELF_TESTS.length} 项全部检出`
    );
    console.log("=".repeat(74));
    return 0;
  }
  console.log(`❌ 语义不一致：${all.length} 项失败（共 ${result.checks} 项检查）`);
  for (const f of all) {
    console.log(`\n  ✗ ${f.check}`);
    console.log(`      ${f.detail}`);
  }
  console.log("=".repeat(74));
  return 1;
}

// 只有在直接执行时才跑 main（被 import 时只暴露函数）
const isDirect =
  process.argv[1] !== undefined &&
  fileURLToPath(import.meta.url).replace(/\\/g, "/").endsWith(
    process.argv[1].replace(/\\/g, "/")
  );

if (isDirect) {
  process.exit(main());
}

export { runSelfTests, SELF_TESTS };
