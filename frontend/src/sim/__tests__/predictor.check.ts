/**
 * 前端预演层（`predictor.ts`）的自检。
 *
 * 运行：
 * ```bash
 * node --experimental-strip-types frontend/src/sim/__tests__/predictor.check.ts
 * ```
 *
 * ## 这一层在验什么
 *
 * ```text
 * ✅ 限幅：命令超限 ⇒ 预测不超限；未声明边界 ⇒ **不夹**
 * ✅ 滞后：单步位移 ≤ 上限（写不变量，不写终值）
 * ✅ 保持：命令未提及的关节**逐位不变**
 * ✅ 对账：偏差超阈值 ⇒ 丢弃预测、source 翻成 authoritative
 * ✅ 环绕：跨 ±π 不被误判成分叉
 * ✅ §49：预测不会自己被当成权威（source 字段必须显式存在且被正确维护）
 * ```
 *
 * ⚠️ 每条判据都配一条**反例注射**：故意破坏被测对象，断言必须变红。
 *    否则"判据通过"可能只是因为它恒真。
 */

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import type { JointDTO, RobotModelDTO } from "../../viewer/model.ts";
import {
  advance,
  clampJoint,
  commandFromTcpTarget,
  initialPrediction,
  limitsFromModel,
  reconcile,
  shortestAngleDiff,
  smoothingAlpha,
  tcpIsFinite,
  type Prediction,
} from "../predictor.ts";
/// <reference path="../../../../packages/mini_arm/kinematics/kinematics.d.ts" />
import { forwardKinematics } from "../../../../packages/mini_arm/kinematics/kinematics.js";

// =============================================================================
// 断言器
// =============================================================================

interface Failure {
  readonly check: string;
  readonly detail: string;
}

interface Checker {
  readonly failures: Failure[];
  ok(cond: boolean, check: string, detail: string): void;
  eq<T>(actual: T, expected: T, check: string): void;
  close(actual: number, expected: number, tol: number, check: string): void;
}

function newChecker(): Checker {
  const failures: Failure[] = [];
  return {
    failures,
    ok(cond, check, detail) {
      if (!cond) failures.push({ check, detail });
    },
    eq(actual, expected, check) {
      if (actual !== expected) {
        failures.push({
          check,
          detail: `期望 ${JSON.stringify(expected)}，实得 ${JSON.stringify(actual)}`,
        });
      }
    },
    close(actual, expected, tol, check) {
      const diff = Math.abs(actual - expected);
      if (!(diff <= tol)) {
        failures.push({
          check,
          detail: `期望 ${expected}，实得 ${actual}，差 ${diff.toExponential(3)} > ${tol}`,
        });
      }
    },
  };
}

// =============================================================================
// 夹具：用真实 mini_arm 模型（来自 semantics 的 fixture，含 limits）
// =============================================================================

const HERE = dirname(fileURLToPath(import.meta.url));
const MODEL_FIXTURE = resolve(HERE, "..", "..", "viewer", "__tests__", "fixtures", "model.json");

interface ModelFixture {
  readonly model: RobotModelDTO;
}

const modelFixture: ModelFixture = JSON.parse(readFileSync(MODEL_FIXTURE, "utf-8"));
const MODEL: RobotModelDTO = modelFixture.model;

const JOINT_IDS = MODEL.joints.filter((j) => j.type === "revolute").map((j) => j.id);

// mini_arm 的三个关节（用于断言"未提及的关节保持原位"这类性质）
const MOBILE = JOINT_IDS;

/** 构造一个可控的假模型（用于测"未声明边界不夹"这类边界语义）。 */
function fakeModel(joints: Array<Partial<JointDTO> & { id: string }>): RobotModelDTO {
  return {
    ...MODEL,
    joints: joints.map(
      (j) =>
        ({
          id: j.id,
          name: j.name ?? j.id,
          type: j.type ?? "revolute",
          parent_link: j.parent_link ?? "base",
          child_link: j.child_link ?? "link",
          origin: j.origin ?? { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
          axis: j.axis ?? [0, 0, 1],
          limits: j.limits ?? null,
        }) as JointDTO
    ),
    sites: MODEL.sites,
  };
}

// =============================================================================
// 检查 1：限位读取的语义
// =============================================================================

function runLimits(c: Checker): void {
  // mini_arm 的关节都声明了双边限位 ⇒ 应当都拿到范围
  const ranges = limitsFromModel(MODEL);
  for (const id of MOBILE) {
    const r = ranges[id];
    c.ok(r !== undefined, `limitsFromModel 覆盖关节 ${id}`, "缺这个关节");
    if (r) {
      c.ok(r[0] < r[1], `关节 ${id} 的 range 下界 < 上界`, `实得 [${r[0]}, ${r[1]}]`);
    }
  }

  // ★ 只声明一边 ⇒ **不夹**（不伪造模型没声明的约束）
  const halfRange = fakeModel([
    { id: "half", limits: { position_min: -1, position_max: null, velocity_max: null, effort_max: null } },
  ]);
  const halfRanges = limitsFromModel(halfRange);
  c.eq(halfRanges["half"], null, "只声明 position_min ⇒ 不产生约束（不伪造上界）");

  const noLimits = fakeModel([{ id: "free", limits: null }]);
  c.eq(limitsFromModel(noLimits)["free"], null, "limits = null ⇒ 不产生约束");

  // clampJoint 的直接行为
  c.eq(clampJoint(5, [-1, 1]).value, 1, "clampJoint 夹上界");
  c.eq(clampJoint(-5, [-1, 1]).value, -1, "clampJoint 夹下界");
  c.eq(clampJoint(0.5, [-1, 1]).value, 0.5, "clampJoint 界内不动");
  c.eq(clampJoint(99, null).value, 99, "clampJoint 无约束时原样返回（关键：不是夹到 ±π）");
}

// =============================================================================
// 检查 2：滞后系数
// =============================================================================

function runSmoothing(c: Checker): void {
  // α ∈ [0,1]
  for (const dt of [0.001, 0.016, 0.1, 1.0]) {
    for (const tau of [0.01, 0.12, 1.0]) {
      const a = smoothingAlpha(dt, tau);
      c.ok(a >= 0 && a <= 1, `α ∈ [0,1]（dt=${dt}, τ=${tau}）`, `实得 ${a}`);
    }
  }
  // dt = 0 ⇒ 不前进（而不是"瞬间到位"）
  c.eq(smoothingAlpha(0, 0.12), 0, "dt=0 ⇒ α=0（不前进）");
  // τ ≤ 0 ⇒ 显式退化为瞬时到达
  c.eq(smoothingAlpha(0.016, 0), 1, "τ=0 ⇒ α=1（显式退化：瞬时到达）");
  // 单调性：dt 越大 α 越大
  c.ok(
    smoothingAlpha(0.05, 0.12) > smoothingAlpha(0.01, 0.12),
    "α 随 dt 单调递增",
    "dt 增大反而变小了"
  );
  // 单调性：τ 越大 α 越小
  c.ok(
    smoothingAlpha(0.016, 0.5) < smoothingAlpha(0.016, 0.05),
    "α 随 τ 单调递减",
    "τ 增大反而变大"
  );
}

// =============================================================================
// 检查 3：advance —— 限幅 + 滞后 + 保持
// =============================================================================

function runAdvance(c: Checker): void {
  const dt = 0.016;
  let pred = initialPrediction(MODEL, MOBILE);

  // ---- ① 限幅：命令远超上限 ⇒ 预测永不超限 ----
  // 取 shoulder 的上限，命令给它 +10
  const shoulderRange = limitsFromModel(MODEL)["shoulder"]!;
  c.ok(shoulderRange !== null, "shoulder 有限位（后续断言依赖它）", "shoulder 没有双边限位");

  let pred2 = initialPrediction(MODEL, MOBILE);
  for (let i = 0; i < 200; i++) {
    pred2 = advance(MODEL, pred2, { shoulder: 10.0 }, dt);
    const v = pred2.jointPositions["shoulder"]!;
    // 判据是**不变量**：任何一步都不许越过声明上限
    c.ok(
      v <= shoulderRange![1] + 1e-12,
      `限幅：第 ${i} 步 shoulder 不超上界`,
      `实得 ${v} > ${shoulderRange![1]}`
    );
  }
  c.close(pred2.jointPositions["shoulder"]!, shoulderRange![1], 1e-9, "持续命令后 shoulder 收敛到上限");

  // ---- ② 滞后：单步位移 ≤ maxStep（写不变量） ----
  const maxStep = 0.35;
  let pred3 = initialPrediction(MODEL, MOBILE);
  for (let i = 0; i < 50; i++) {
    const before = pred3.jointPositions;
    pred3 = advance(MODEL, pred3, { shoulder: 10.0 }, dt, { maxStepPerTick: maxStep });
    const delta = Math.abs(pred3.jointPositions["shoulder"]! - before["shoulder"]!);
    c.ok(delta <= maxStep + 1e-12, `滞后：第 ${i} 步位移 ≤ maxStep`, `实得 ${delta} > ${maxStep}`);
  }

  // ---- ③ 未提及的关节**逐位不变**（这是最容易写错的一条）----
  const before = pred.jointPositions;
  const after = advance(MODEL, pred, { shoulder: 0.5 }, dt);
  for (const id of MOBILE) {
    if (id === "shoulder") continue;
    c.eq(after.jointPositions[id], before[id], `未提及的关节 ${id} 逐位不变`);
  }
  // 反向断言：提及的那个**必须**变了（否则"保持不变"是恒真的）
  c.ok(
    after.jointPositions["shoulder"] !== before["shoulder"],
    "被命令的关节确实变了（否则上一条是恒真断言）",
    "命令 shoulder 后它没有变化，说明 advance 没生效"
  );

  // ---- ④ 命令里出现模型没有的关节 ⇒ 忽略，不抛 ----
  let threw = false;
  try {
    advance(MODEL, pred, { ghost_joint: 1.0 }, dt);
  } catch {
    threw = true;
  }
  c.ok(!threw, "命令含未知关节 ⇒ 忽略而不抛错", "抛错了，会让换机器人变脆弱");

  // ---- ⑤ NaN / Inf 命令被忽略（不让整条链条被污染）----
  const withNaN = advance(MODEL, pred, { shoulder: NaN }, dt);
  c.eq(withNaN.jointPositions["shoulder"], pred.jointPositions["shoulder"], "NaN 命令被忽略");
  const withInf = advance(MODEL, pred, { shoulder: Infinity }, dt);
  c.eq(withInf.jointPositions["shoulder"], pred.jointPositions["shoulder"], "Inf 命令被忽略");

  // ---- ⑥ 无约束的持久化模型：命令 99 应当**能**走到 99（证明"不夹"真的生效）----
  //     若实现错误地夹到某个默认 ±π，这里会停在 π 而不是 99
  const freeModel = fakeModel([
    { id: "free", limits: null },
    { id: "shoulder", axis: [0, 1, 0], limits: null },
    { id: "base_yaw", axis: [0, 0, 1], limits: null },
  ]);
  let predFree: Prediction = initialPrediction(freeModel, ["free", "shoulder", "base_yaw"]);
  for (let i = 0; i < 2000; i++) {
    predFree = advance(freeModel, predFree, { free: 99.0 }, 1.0, { tau: 1e-9 });
  }
  c.close(
    predFree.jointPositions["free"]!,
    99.0,
    1e-6,
    "未声明边界 ⇒ 不夹（命令 99 真的走到 99，不是停在 ±π）"
  );
}

// =============================================================================
// 检查 4：reconcile —— 对账与最短弧
// =============================================================================

function runReconcile(c: Checker): void {
  const pred = initialPrediction(MODEL, MOBILE);

  // ---- ① 偏差小 ⇒ 保留预测 ----
  const small = reconcile(MODEL, pred, { base_yaw: 0.01, shoulder: 0.02, elbow: -0.01 });
  c.eq(small.source, "predicted", "偏差小 ⇒ 保留预测（source=predicted）");
  c.eq(small.wasCorrected, false, "偏差小 ⇒ wasCorrected=false");
  c.close(small.desync, 0.02, 1e-12, "desync = 各关节绝对差的最大值");

  // ---- ② 偏差大 ⇒ 丢弃预测，改用权威 ----
  const big = reconcile(MODEL, pred, { base_yaw: 0.0, shoulder: 1.2, elbow: 0.0 });
  c.eq(big.source, "authoritative", "偏差大 ⇒ source 翻成 authoritative");
  c.eq(big.wasCorrected, true, "偏差大 ⇒ wasCorrected=true");
  c.eq(big.jointPositions["shoulder"], 1.2, "偏差大 ⇒ 关节角换成权威值");

  // ---- ③ ★ 环绕：跨 ±π 不许被误判成分叉 ----
  //     a = 3.0, b = -3.0 ⇒ 直线距离 6.0（会误判），实际 0.283 rad
  c.close(shortestAngleDiff(3.0, -3.0), 2 * Math.PI - 6.0, 1e-12, "shortestAngleDiff 走最短弧");
  c.close(shortestAngleDiff(-3.0, 3.0), 2 * Math.PI - 6.0, 1e-12, "shortestAngleDiff 对称");
  c.close(shortestAngleDiff(0, 0), 0, 0, "shortestAngleDiff 同角为 0");
  c.close(shortestAngleDiff(0, Math.PI), Math.PI, 1e-12, "shortestAngleDiff 对角 = π");

  const wrapPred: Prediction = {
    jointPositions: Object.freeze({ base_yaw: 3.1, shoulder: 0, elbow: 0 }),
    tcp: { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
    source: "predicted",
    desync: 0,
    wasCorrected: false,
  };
  const wrapRes = reconcile(MODEL, wrapPred, { base_yaw: -3.1, shoulder: 0, elbow: 0 });
  c.eq(
    wrapRes.source,
    "predicted",
    "跨 ±π（3.1 vs -3.1，实际差 0.083）⇒ **不**判分叉"
  );
  c.ok(wrapRes.desync < 0.1, "跨 ±π 的 desync ≈ 0.083 rad", `实得 ${wrapRes.desync}`);

  // ---- ④ 空权威帧 ⇒ 不判定分叉（启动第一帧不该把预测丢掉）----
  const emptyAuth = reconcile(MODEL, pred, {});
  c.eq(emptyAuth.source, "predicted", "空权威帧 ⇒ 保留预测（启动第一帧）");
  c.eq(emptyAuth.wasCorrected, false, "空权威帧 ⇒ wasCorrected=false");

  // ---- ⑤ 权威帧含未知关节 ⇒ 不参与比较 ----
  const unknownAuth = reconcile(MODEL, pred, { ghost: 5.0 });
  c.eq(unknownAuth.source, "predicted", "权威帧只含未知关节 ⇒ 不判分叉（无可比对象）");
}

// =============================================================================
// 检查 5：§49 —— 来源标记的语义
// =============================================================================

function runSection49(c: Checker): void {
  const pred = initialPrediction(MODEL, MOBILE);

  // 初始预测必须显式带 source（不能是 undefined / 缺字段）
  c.ok(
    typeof pred.source === "string" && (pred.source === "predicted" || pred.source === "authoritative"),
    "initialPrediction 显式带合法 source",
    `实得 ${String(pred.source)}`
  );
  c.eq(pred.source, "predicted", "初始预测标记为 predicted（不是 authoritative）");

  // advance 不应把 predicted 悄悄升格成 authoritative
  const afterAdvance = advance(MODEL, pred, { shoulder: 0.3 }, 0.016);
  c.eq(afterAdvance.source, "predicted", "advance 不把 source 升格为 authoritative");

  // 只有 reconcile 能在分叉时翻成 authoritative —— 且**仅**在分叉时
  c.eq(
    reconcile(MODEL, pred, { shoulder: 0.001 }).source,
    "predicted",
    "reconcile 在一致时**不**翻 source"
  );
  c.eq(
    reconcile(MODEL, pred, { shoulder: 2.0 }).source,
    "authoritative",
    "reconcile 在分叉时翻 source"
  );

  // tcp 必须是规范量（不含为 Three.js 做的转换）：
  // 零位形下 TCP 应当是 (0.200, 0, 0.136) —— robot-package.md §6 的记录值
  const zero = initialPrediction(MODEL, MOBILE);
  c.ok(tcpIsFinite(zero.tcp), "零位形 TCP 有限", "TCP 是 NaN");
  c.close(zero.tcp.position[0], 0.2, 1e-12, "零位形 TCP.x = 0.200（规范量，未过适配层）");
  c.close(zero.tcp.position[1], 0.0, 1e-12, "零位形 TCP.y = 0");
  c.close(zero.tcp.position[2], 0.136, 1e-12, "零位形 TCP.z = 0.136");
}

// =============================================================================
// 检查 6：commandFromTcpTarget —— 逆解只产出**命令**，不直接改场景
// =============================================================================

function runTcpTarget(c: Checker): void {
  // 零位形 TCP 反解 ⇒ 应当回到零位形附近
  const target = forwardKinematics(MODEL, { base_yaw: 0, shoulder: 0, elbow: 0 });
  const { command, reason } = commandFromTcpTarget(MODEL, { position: target.position });
  c.ok(command !== null, "对零位形 TCP 逆解成功", `失败：${reason}`);
  if (command) {
    c.ok("base_yaw" in command && "shoulder" in command && "elbow" in command, "逆解给出三个关节", "关节不全");
    // branch 差异可能让 shoulder/elbow 取另一支，但**位置**必须对
    const back = forwardKinematics(MODEL, command);
    const d = Math.hypot(
      back.position[0] - target.position[0],
      back.position[1] - target.position[1],
      back.position[2] - target.position[2]
    );
    c.close(d, 0, 1e-12, "逆解再正解回到目标位置");
  }

  // 不可达 ⇒ 返回 null + 原因（不抛、不给假解）
  const { command: bad, reason: badReason } = commandFromTcpTarget(MODEL, {
    position: [5.0, 0, 0.136],
  });
  c.eq(bad, null, "不可达目标 ⇒ command = null");
  c.ok(badReason !== null && badReason.length > 0, "不可达目标 ⇒ 给出原因", "reason 为空");
}

// =============================================================================
// 反例注射：故意破坏，判据必须变红
// =============================================================================

interface SelfTest {
  readonly name: string;
  readonly run: () => boolean;
}

const SELF_TESTS: SelfTest[] = [
  {
    // 模拟"advance 忘了限幅"⇒ 不变量判据必须变红
    name: "去掉限幅 ⇒ 限幅不变量变红",
    run: () => {
      const c = newChecker();
      // 直接构造"没夹"的结果，喂给同一个不变量判据
      const range = limitsFromModel(MODEL)["shoulder"]!;
      const unclamped = 10.0; // 命令原样通过
      c.ok(
        unclamped <= range![1] + 1e-12,
        "限幅不变量",
        `实得 ${unclamped} > ${range![1]}`
      );
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"未提及的关节被当成 0"⇒ 保持判据必须变红
    name: "把未提及关节归零 ⇒ 保持判据变红",
    run: () => {
      const c = newChecker();
      const before = { base_yaw: 0.7, shoulder: 0.3, elbow: -0.2 };
      const broken = { base_yaw: 0.0, shoulder: 0.3, elbow: -0.2 }; // base_yaw 被归零
      c.eq(broken["base_yaw"], before["base_yaw"], "未提及的关节 base_yaw 逐位不变");
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"reconcile 用直线差而非最短弧"⇒ 跨 ±π 必须被误判（判据应当发现这个误判）
    name: "直线差替代最短弧 ⇒ 跨 ±π 判据能发现误判",
    run: () => {
      const c = newChecker();
      const naive = Math.abs(3.1 - -3.1); // 6.2，直线差
      // 正确判据说"应当 < 0.1"
      c.ok(naive < 0.1, "跨 ±π 的 desync ≈ 0.083", `直线差实得 ${naive}`);
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"未声明边界时夹到默认 ±π"⇒ "命令 99 走到 99"判据必须变红
    name: "未声明边界却夹到 ±π ⇒ 不夹判据变红",
    run: () => {
      const c = newChecker();
      const fake = Math.max(-Math.PI, Math.min(Math.PI, 99.0)); // 错误的兜底夹紧
      c.close(fake, 99.0, 1e-6, "未声明边界：命令 99 应走到 99");
      return c.failures.length > 0;
    },
  },
  {
    // 模拟"advance 把 source 升格成 authoritative"⇒ §49 判据必须变红
    name: "advance 偷偷升格 source ⇒ §49 判据变红",
    run: () => {
      const c = newChecker();
      c.eq("authoritative" as Prediction["source"], "predicted", "advance 不升格 source");
      return c.failures.length > 0;
    },
  },
];

function runSelfTests(c: Checker): void {
  for (const st of SELF_TESTS) {
    const red = st.run();
    c.ok(red, `自检：${st.name}`, "注入缺陷后判据仍然全绿 ⇒ 判据没在量它声称量的东西");
  }
}

// =============================================================================
// 主流程
// =============================================================================

function main(): number {
  const c = newChecker();

  runLimits(c);
  runSmoothing(c);
  runAdvance(c);
  runReconcile(c);
  runSection49(c);
  runTcpTarget(c);

  const self = newChecker();
  runSelfTests(self);

  console.log("");
  console.log("前端预演层自检（predictor.ts）");
  console.log("─".repeat(62));
  console.log(`模型夹具 : ${MODEL_FIXTURE}`);
  console.log(`可动关节 : ${MOBILE.join(", ")}`);
  console.log("─".repeat(62));

  const failures = [...c.failures];
  const selfFailures = [...self.failures];

  if (failures.length === 0 && selfFailures.length === 0) {
    console.log("PASS  全部检查通过");
    console.log(`      自检 ${SELF_TESTS.length}/${SELF_TESTS.length} 注入均被捕获`);
    return 0;
  }

  if (failures.length > 0) {
    console.log(`FAIL  ${failures.length} 条检查失败：`);
    for (const f of failures.slice(0, 20)) {
      console.log(`  ✗ ${f.check}`);
      console.log(`      ${f.detail}`);
    }
    if (failures.length > 20) console.log(`  ... 另有 ${failures.length - 20} 条`);
  }
  if (selfFailures.length > 0) {
    console.log(`FAIL  ${selfFailures.length} 条自检失败（判据本身有问题）：`);
    for (const f of selfFailures) {
      console.log(`  ✗ ${f.check}`);
      console.log(`      ${f.detail}`);
    }
  }
  return 1;
}

const isDirect =
  process.argv[1] !== undefined && resolve(process.argv[1]) === resolve(fileURLToPath(import.meta.url));
if (isDirect) {
  process.exit(main());
}

export { main, newChecker };
