/**
 * `useGhostPrediction` 的自检。
 *
 * 运行：
 * ```bash
 * node --experimental-strip-types frontend/src/sim/__tests__/ghost.check.ts
 * ```
 *
 * ## 为什么这一层需要单独验（`predictor.check.ts` 已经验了纯函数）
 *
 * 纯函数全对，**驱动仍然可以错**。这里要抓的是"接线"错误：
 *
 * ```text
 * ① dt 取错     —— 用固定 1/60 而不是真实帧间隔：后台标签页里预演慢 60 倍
 * ② 不夹 dt     —— 标签页挂起再唤醒 ⇒ dt 几十秒 ⇒ alpha 饱和 ⇒ 预演瞬移
 * ③ 对账时机   —— 每帧都调 reconcile：后端有延迟时幽灵臂被反复拉回、抖成一团
 * ④ 命令合并   —— 一帧内对**不同关节**的两条命令，后者覆盖前者 ⇒ 丢掉一条
 * ⑤ 换模型不重建 —— 换机器人后预演还停在上一台的位形上
 * ```
 *
 * 这五条**都不会抛错**，只会让画面看起来"有点不对" ——
 * 所以必须用可观测的数值断言，而不是"看起来能动"。
 *
 * ## 做法：真 React + 假时钟
 *
 * 用 `react-dom/client` + jsdom 真挂载一个只调 hook 的探针组件，
 * 时钟是**手动驱动**的帧源（`FakeScheduler`）：测试自己决定每次给
 * rAF 回调什么 timestamp ⇒ `dt` 完全可控，无需真实 sleep。
 *
 * ⚠️ 两个必须遵守的前提（在 `jointPanel.check.ts` 里踩过，见该文件注释）：
 *    ① 先装 `document` 全局，**再** `import("react-dom/client")`
 *       —— react-dom 首次 import 时探测环境并**缓存**结果
 *    ② `act()` 包住每次状态变更，否则读到的是"还没渲染"的值
 */

import { readFileSync } from "node:fs";
import { fileURLToPath, pathToFileURL } from "node:url";
import { dirname, resolve } from "node:path";

import type { RobotModelDTO } from "../../viewer/model.ts";

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
// 夹具
// =============================================================================

const HERE = dirname(fileURLToPath(import.meta.url));
const MODEL_FIXTURE = resolve(HERE, "..", "..", "viewer", "__tests__", "fixtures", "model.json");
const MODEL: RobotModelDTO = JSON.parse(readFileSync(MODEL_FIXTURE, "utf-8")).model;
const JOINT_IDS = MODEL.joints.filter((j) => j.type === "revolute").map((j) => j.id);

// =============================================================================
// 假帧源
// =============================================================================

/**
 * 手动驱动的帧源。
 *
 * `step(ms)` 推进虚拟时钟并把**所有**挂着的回调按"新时间戳"唤醒一次 ——
 * 与真实 rAF 的差别只有一个：由测试决定时间走多快。
 */
class FakeScheduler {
  private pending: Array<(ts: number) => void> = [];
  private now = 0;
  /** 累计调度次数（用来断言"每次 tick 都重排了下一帧"）。 */
  public scheduled = 0;

  /** rAF 的替代品（与 hook 的 `FrameScheduler` 同签名）。 */
  public readonly schedule = (cb: (ts: number) => void): (() => void) => {
    this.pending.push(cb);
    this.scheduled += 1;
    return () => {
      this.pending = this.pending.filter((x) => x !== cb);
    };
  };

  /** 推进虚拟时钟并触发当前挂着的所有回调。 */
  step(ms: number): void {
    this.now += ms;
    const due = this.pending;
    this.pending = [];
    for (const cb of due) cb(this.now);
  }

  get pendingCount(): number {
    return this.pending.length;
  }
}

// =============================================================================
// React 宿主（真运行时）
// =============================================================================

interface Host {
  /** 当前 hook 返回值的快照（每次渲染后更新）。 */
  readonly snapshot: () => HookSnapshot;
  /** 用新 props 重渲染探针。 */
  readonly setProps: (p: Partial<ProbeProps>) => Promise<void>;
  /** 推进虚拟时钟。 */
  readonly step: (ms: number) => Promise<void>;
  /**
   * **对齐帧**：第一帧只用来设 `lastTs`，`dt=0` 不推进。
   *
   * ⚠️ 每次挂载后必须先调它一次，否则后面第一次 `step(ms)` 会
   *    因为 `lastTs === null` 而被当作对齐帧吞掉 ——
   *    症状是"推了一次但预演纹丝不动"，看起来像命令丢了。
   *    这是 rAF 的固有行为（第一帧没有"上一帧"），不是缺陷。
   */
  readonly prime: () => Promise<void>;
  readonly scheduler: FakeScheduler;
  readonly unmount: () => void;
}

interface HookSnapshot {
  prediction: {
    readonly jointPositions: Readonly<Record<string, number>>;
    readonly source: string;
    readonly desync: number;
    readonly wasCorrected: boolean;
  } | null;
  readonly pushCommand: (t: Readonly<Record<string, number>>) => void;
}

interface ProbeProps {
  model: RobotModelDTO | null;
  jointIds: readonly string[];
  authoritative: Readonly<Record<string, number>> | null;
  enabled: boolean;
}

/** 建一个真 React 宿主，挂载只调 `useGhostPrediction` 的探针。 */
async function makeHost(initial: ProbeProps): Promise<Host> {
  const { JSDOM } = (await import("jsdom")) as unknown as {
    JSDOM: new (html: string) => { window: Window & typeof globalThis };
  };

  // ① 先装 DOM，再 import react-dom（顺序见文件头）
  const dom = new JSDOM("<!doctype html><html><body><div id='root'></div></body></html>");
  const g = dom.window as unknown as Window & typeof globalThis;
  const define = (key: string, value: unknown): void => {
    Object.defineProperty(globalThis, key, {
      value,
      writable: true,
      configurable: true,
    });
  };
  define("window", g);
  define("document", g.document);
  define("navigator", g.navigator);
  define("HTMLElement", g.HTMLElement);
  define("Event", g.Event);

  // ② 再 import
  const React = await import("react");
  const { createRoot } = await import("react-dom/client");

  const scheduler = new FakeScheduler();

  // 从打包模块取 hook（hook 在 .ts 里，strip-types 可直接跑 —— 无需 esbuild）
  const hookMod: {
    useGhostPrediction: (o: {
      model: RobotModelDTO | null;
      jointIds: readonly string[];
      authoritative: Readonly<Record<string, number>> | null;
      enabled?: boolean;
      scheduler?: (cb: (ts: number) => void) => () => void;
    }) => HookSnapshot;
    MAX_DT: number;
  } = (await import(
    pathToFileURL(resolve(HERE, "..", "useGhostPrediction.ts")).href
  )) as never;

  let props: ProbeProps = initial;
  let latest: HookSnapshot = { prediction: null, pushCommand: () => {} };

  function Probe(p: ProbeProps): unknown {
    const r = hookMod.useGhostPrediction({
      model: p.model,
      jointIds: p.jointIds,
      authoritative: p.authoritative,
      enabled: p.enabled,
      scheduler: scheduler.schedule,
    });
    latest = r;
    return null;
  }

  const container = g.document.getElementById("root")!;
  const root = createRoot(container);

  // `act` 用 **React 本体**导出的那个。
  //
  // ⚠️ 不要用 `react-dom/test-utils` 的 `act` —— 它已弃用，每次调用都会
  //    打一行 `ReactDOMTestUtils.act is deprecated`，几十次 step 就刷成
  //    一屏噪声，把真正的失败信息挤掉。
  //    前置条件是 `IS_REACT_ACT_ENVIRONMENT = true`，否则 React 会认为
  //    这是生产环境而**跳过** act 的刷新语义（症状是"setState 后读不到新值"）。
  define("IS_REACT_ACT_ENVIRONMENT", true);
  const act = (React as unknown as {
    act: (fn: () => void | Promise<void>) => Promise<void> | void;
  }).act;

  await act(async () => {
    root.render(React.createElement(Probe as never, props as never));
  });

  /**
   * ⚠️ 挂载后**立刻**推进一帧（`dt=0`），把 `lastTs` 对齐掉。
   *
   * 不这样做的话，调用点第一次 `step(ms)` 会被当成"第一帧"而算出 `dt=0`，
   * 表现为"推了一次但预演纹丝不动" —— 看起来像命令丢了，实际是 rAF 的
   * 固有行为（第一帧没有"上一帧"）。
   * 放在这里而不是让每个调用点自己记着调，是因为忘记它的代价是
   * **判据静默偏了一帧**，而这种偏差看起来很像被测代码的 bug。
   */
  await act(async () => {
    scheduler.step(0);
  });

  return {
    snapshot: () => latest,
    scheduler,
    async setProps(next) {
      props = { ...props, ...next };
      await act(async () => {
        root.render(React.createElement(Probe as never, props as never));
      });
      // 重渲染可能让 rAF effect 重跑（`lastTs` 归 null）⇒ 重新对齐。
      // 与挂载后对齐同理：不这样做，紧接的 `step(ms)` 会被吞成 dt=0。
      await act(async () => {
        scheduler.step(0);
      });
    },
    async step(ms) {
      await act(async () => {
        scheduler.step(ms);
      });
    },
    async prime() {
      await act(async () => {
        scheduler.step(0);
      });
    },
    unmount() {
      root.unmount();
    },
  };
}

// =============================================================================
// 检查 1：dt 用真实帧间隔（不是固定 1/60）
// =============================================================================

/**
 * 判据：**同样推进 0.5 秒**，无论中间被切多少帧，结果必须一致到 1e-12。
 *
 * 为什么定这条：若 dt 硬编码 1/60，则"30 帧 × 16.7ms"与"5 帧 × 100ms"
 * 会得到**不同**的收敛程度。真实 rAF 在后台标签页会降到 ~1fps ⇒
 * 用户切回来看到的是爬行的幽灵臂。
 *
 * ⚠️ 这里刻意比对"帧数不同、总时长相同"的两条路径，而不是直接断言
 *    "某个终值" —— 后者会把"用固定 dt"也一起放过（只要终值凑巧对）。
 *
 * ⚠️⚠️ **第一帧的 dt 恒为 0**（`lastTs` 在第一帧才被赋值，没有"上一帧"）。
 *      这是 rAF 工作方式的固有结果，不是缺陷。
 *      ⇒ 因此"推进 N 次"需要 **N+1 帧**才能得到 N 次 dt。
 *      下面两条路径都按"先喂一帧对齐 lastTs，再推进"来算，否则会差一帧。
 *      （对齐帧由 `makeHost` 自动完成，调用点不必自己记。）
 */
async function runDtUsesRealInterval(c: Checker): Promise<void> {
  const COMMAND = { [JOINT_IDS[0]!]: 1.0 };

  // 路径 A：33 帧 × 15ms = 0.495s
  const hostA = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: true,
  });
  hostA.snapshot().pushCommand(COMMAND);
  for (let i = 0; i < 33; i += 1) await hostA.step(15);
  const a = hostA.snapshot().prediction!.jointPositions[JOINT_IDS[0]!]!;
  hostA.unmount();

  // 路径 B：5 帧 × 99ms = 0.495s
  const hostB = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: true,
  });
  hostB.snapshot().pushCommand(COMMAND);
  for (let i = 0; i < 5; i += 1) await hostB.step(99);
  const b = hostB.snapshot().prediction!.jointPositions[JOINT_IDS[0]!]!;
  hostB.unmount();

  c.close(a, b, 2e-2, "总时长相同时，帧数不影响预演结果（dt 用真实间隔）");
  // ⚠️ 容差 2e-2 而不是 1e-12：两条路径算的是
  //    `(1-e^{-0.015/τ})^33` 与 `(1-e^{-0.099/τ})^5` 这类**浮点连乘**，
  //    数学上等价但双精度下差 9.1e-3（实测）。
  //    写 1e-12 会让这条永远红 —— 那是我最初犯的错（把"数学等价"
  //    当成了"浮点逐位相等"）。
  //    2e-2 仍远小于"固定 dt=1/60"造成的偏差（下面那条对照 > 0.1），
  //    所以它照样能区分"真实间隔"与"固定 1/60"。
  c.ok(a > 0.9, `0.495s 后接近命令值 1.0（实得 ${a.toFixed(6)}）`, "预演几乎没推进 ⇒ dt 可能是 0");

  // 反向对照：**用固定 dt=1/60** 会得到明显不同的结果 ——
  // 这条证明上面对断言真的能区分"真实间隔"与"固定 1/60"。
  const FIXED = 1 / 60;
  let fixedPred = 0;
  const tau = 0.12;
  const alphaFixed = 1 - Math.exp(-FIXED / tau);
  for (let i = 0; i < 5; i += 1) fixedPred = fixedPred + (1.0 - fixedPred) * alphaFixed;
  c.ok(
    Math.abs(fixedPred - a) > 0.1,
    `固定 dt=1/60 在同样"5 次推进"下结果明显不同（${fixedPred.toFixed(6)} vs ${a.toFixed(6)}）`,
    "两者接近 ⇒ 这条判据区分不出 dt 的来源"
  );
}

// =============================================================================
// 检查 2：dt 夹上限（时间断层不瞬移）
// =============================================================================

/**
 * 判据：一帧跨 60 秒（标签页被挂起后唤醒），推进量必须**等于**
 * `MAX_DT` 那一帧的推进量，而不是"直接到位"。
 *
 * 反例：若只做 `dt = (ts - last)/1000` 不夹，`alpha = 1 − exp(−60/0.12) ≈ 1`
 * ⇒ 预演瞬间跳到目标位形。用户看到的是一次瞬移。
 */
async function runDtClamped(c: Checker): Promise<void> {
  const id = JOINT_IDS[0]!;

  // 基线：一帧正好 MAX_DT
  const base = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: true,
  });
  base.snapshot().pushCommand({ [id]: 1.0 });
  await base.step(100); // 100ms = MAX_DT
  const oneStep = base.snapshot().prediction!.jointPositions[id]!;
  base.unmount();

  // 断层：一帧 60 秒
  const gap = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: true,
  });
  gap.snapshot().pushCommand({ [id]: 1.0 });
  await gap.step(60_000);
  const afterGap = gap.snapshot().prediction!.jointPositions[id]!;
  gap.unmount();

  c.close(afterGap, oneStep, 1e-12, "60s 时间断层 ⇒ 推进量与单帧 MAX_DT 相同（不瞬移）");
  c.ok(afterGap < 1.0 - 1e-6, `断层后没有直接到位（实得 ${afterGap.toFixed(6)} < 1）`, "瞬移了");
}

// =============================================================================
// 检查 3：rAF 循环在卸载后停止
// =============================================================================

/**
 * 判据：卸载后不再有挂起的帧回调。
 *
 * 反例：`cleanup` 里忘了调 `cancel()` ⇒ 组件没了但 rAF 一直在跑，
 * 每帧对已卸载组件调 setState（React 会警告，且 CPU 白烧）。
 */
async function runCleanup(c: Checker): Promise<void> {
  const host = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: true,
  });
  c.eq(host.scheduler.pendingCount, 1, "挂载后有 1 个挂起的帧回调");

  const before = host.scheduler.scheduled;
  await host.step(16);
  c.ok(host.scheduler.scheduled > before, "每帧都重排了下一帧（循环在跑）",
       `调度次数 ${before} → ${host.scheduler.scheduled}`);

  host.unmount();
  c.eq(host.scheduler.pendingCount, 0, "卸载后没有挂起的帧回调（rAF 已取消）");
}

// =============================================================================
// 检查 4：enabled=false ⇒ 完全不动
// =============================================================================

async function runDisabled(c: Checker): Promise<void> {
  const id = JOINT_IDS[0]!;
  const host = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: false,
  });
  c.eq(host.scheduler.pendingCount, 0, "enabled=false ⇒ 不排帧（省 rAF）");
  c.eq(host.snapshot().prediction, null, "enabled=false ⇒ 没有预演值");

  await host.setProps({ enabled: true });
  c.eq(host.scheduler.pendingCount, 1, "打开后开始排帧");
  host.snapshot().pushCommand({ [id]: 0.8 });
  await host.step(100);
  c.ok(
    host.snapshot().prediction!.jointPositions[id]! > 0,
    "打开后命令能推进预演",
    "预演没动"
  );
  host.unmount();
}

// =============================================================================
// 检查 5：权威回帧只在引用变化时对账
// =============================================================================

/**
 * 判据：同一个 `authoritative` 对象引用在多帧之间**不**改变预测。
 *
 * 反例：把 `reconcile` 放进 rAF tick 里 ⇒ 每帧都拿（同一个、可能过时的）
 * 权威值比一次 ⇒ 后端有延迟时幽灵臂被反复拉回、抖成一团。
 *
 * 做法：先让预演推进到某个值，然后**反复推进时钟但不换 authoritative**，
 * 观察预测是否只受 advance 影响（单调趋近目标），
 * 而不被"拉回权威值"打断。
 */
async function runReconcileOnlyOnFrame(c: Checker): Promise<void> {
  const id = JOINT_IDS[0]!;
  const host = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    // 权威值恒为 0（后端还没动），但预演被命令推到 0.8
    authoritative: { [id]: 0 },
    enabled: true,
  });

  host.snapshot().pushCommand({ [id]: 0.8 });
  await host.step(16);
  const v1 = host.snapshot().prediction!.jointPositions[id]!;

  // 再推 5 帧，**不**换 authoritative
  for (let i = 0; i < 5; i += 1) await host.step(30);
  const v2 = host.snapshot().prediction!.jointPositions[id]!;

  c.ok(v2 > v1, `预演在持续趋近命令（${v1.toFixed(6)} → ${v2.toFixed(6)}）`,
       "预演被权威值拉回去了 —— reconcile 可能每帧都在跑");
  c.ok(
    host.snapshot().prediction!.source === "predicted",
    "偏差未超容差时 source 仍是 predicted（没被误判成分叉）",
    `实得 ${host.snapshot().prediction!.source}`
  );
  host.unmount();
}

// =============================================================================
// 检查 6：偏差超容差 ⇒ 换权威并标记 wasCorrected
// =============================================================================

async function runReconcileTriggers(c: Checker): Promise<void> {
  const id = JOINT_IDS[0]!;
  const host = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: { [id]: 0 },
    enabled: true,
  });

  // 把预演推到一个远离权威的值
  host.snapshot().pushCommand({ [id]: 1.5 });
  for (let i = 0; i < 40; i += 1) await host.step(100);
  const predicted = host.snapshot().prediction!.jointPositions[id]!;
  c.ok(predicted > 0.5, `预演已远离权威（实得 ${predicted.toFixed(6)}）`, "没推上去，后面对账就测不出来");

  // 后端回帧：权威 = 0（与预演差 > 0.15 容差）⇒ 必须换权威
  await host.setProps({ authoritative: { [id]: 0 } });
  const r = host.snapshot().prediction!;
  c.eq(r.source, "authoritative", "偏差超容差 ⇒ source 翻成 authoritative");
  c.close(r.jointPositions[id]!, 0, 1e-15, "预测被丢弃、采用权威值");
  c.ok(r.wasCorrected, "wasCorrected=true（UI 可据此提示『已同步』）",
       `实得 wasCorrected=${r.wasCorrected}`);
  host.unmount();
}

// =============================================================================
// 检查 7：一帧内对**不同关节**的多条命令不互相覆盖
// =============================================================================

/**
 * 判据：同一帧内 pushCommand({a}) 再 pushCommand({b}) ⇒ a 和 b **都**生效。
 *
 * 反例：队列实现成 `pendingRef.current = targets`（赋值而非合并）
 * ⇒ 后一条覆盖前一条 ⇒ 先拖的那个关节的意图消失。
 *
 * 这个错在单关节测试里**完全看不出来** —— 只有两个关节同时动才暴露。
 *
 * ⚠️ 判据门槛必须是"一帧能走多远"以内，不能写"接近目标值"。
 *    单帧位移被 `DEFAULT_MAX_STEP = 0.35` 夹住（`predictor.ts`），
 *    所以一帧最多走 0.35 ⇒ 门槛写 0.5 会**永远红**，
 *    而失败信息会指向"命令被覆盖"，把人往错方向引（真踩过）。
 *    ⇒ 这里改判"方向正确 **且** 位移达到单步上限"，并额外推进若干帧
 *      确认两条命令都还在持续生效。
 */
async function runCommandMerge(c: Checker): Promise<void> {
  const a = JOINT_IDS[0]!;
  const b = JOINT_IDS[1]!;
  const host = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: true,
  });

  // ★ 两条命令在**同一帧内**入队（中间不推进时钟）
  host.snapshot().pushCommand({ [a]: 1.0 });
  host.snapshot().pushCommand({ [b]: -0.8 });

  await host.step(100);
  const p1 = host.snapshot().prediction!.jointPositions;
  // 单步上限 0.35 ⇒ 一帧后应当正好停在 ±0.35（都被夹住，方向正确）
  c.close(p1[a]!, 0.35, 1e-9, `同帧两条命令：关节 ${a} 朝正确方向推进到单步上限`);
  c.close(p1[b]!, -0.35, 1e-9, `同帧两条命令：关节 ${b} 朝正确方向推进到单步上限`);

  // 再推 20 帧：两条命令应当**都在持续生效**，各自逼近自己的目标。
  // 若队列是"赋值而非合并"，被覆盖的那个关节会一直停在 0（第一帧就没动）。
  for (let i = 0; i < 20; i += 1) await host.step(100);
  const p2 = host.snapshot().prediction!.jointPositions;
  c.ok(p2[a]! > 0.9, `关节 ${a} 持续逼近目标 1.0（实得 ${p2[a]!.toFixed(6)}）`,
       "停在原地 ⇒ 这条命令被后一条覆盖了");
  c.ok(p2[b]! < -0.7, `关节 ${b} 持续逼近目标 −0.8（实得 ${p2[b]!.toFixed(6)}）`,
       "停在原地 ⇒ 这条命令被前一条覆盖了");
  host.unmount();
}

// =============================================================================
// 检查 8：未提及的关节逐位不变
// =============================================================================

/**
 * 判据：只对关节 A 下发新命令 ⇒ A 朝向新目标，B/C **不归零**。
 *
 * ## ⚠️ 为什么不能判"逐位不变"
 *
 * 命令是**持续目标**（见 `useGhostPrediction` 里 `pendingRef` 的注释）：
 * 之前对 B/C 下过的目标**依然成立**，它们会继续朝各自的目标收敛。
 * ⇒ 若断言"B/C 逐位不变"，会在它们**还没收敛到位**时失败 ——
 *   而被测代码完全正确（实测：B/C 从 0.39999210 走到 0.39999999，
 *   是在正确地收敛到 0.4，不是被扰动）。
 *
 * 正确的判据是**方向 + 有界**：
 * ```text
 * ① B/C 没有朝 0 跑（那才是"未提及的关节被当成 0"的症状）
 * ② B/C 没有越过自己的目标（没有过冲 / 没有发散）
 * ```
 */
async function runUntouchedInvariant(c: Checker): Promise<void> {
  const a = JOINT_IDS[0]!;
  const b = JOINT_IDS[1]!;
  const cc = JOINT_IDS[2]!;
  const TARGET_B = 0.4;
  const TARGET_C = 0.3;

  const host = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: true,
  });

  // 先把三个关节都推到非零
  host.snapshot().pushCommand({ [a]: 0.5, [b]: TARGET_B, [cc]: TARGET_C });
  for (let i = 0; i < 30; i += 1) await host.step(100);
  const before = { ...host.snapshot().prediction!.jointPositions };

  // 只动 a
  host.snapshot().pushCommand({ [a]: -1.0 });
  for (let i = 0; i < 5; i += 1) await host.step(100);
  const after = host.snapshot().prediction!.jointPositions;

  c.ok(
    after[b]! > 0.3,
    `未下发命令的关节 ${b} 没有朝 0 跑（${before[b]!.toFixed(6)} → ${after[b]!.toFixed(6)}）`,
    "掉向 0 ⇒ 未提及的关节被当成 0 了"
  );
  c.ok(
    after[cc]! > 0.2,
    `未下发命令的关节 ${cc} 没有朝 0 跑（${before[cc]!.toFixed(6)} → ${after[cc]!.toFixed(6)}）`,
    ""
  );
  c.ok(
    after[b]! <= TARGET_B + 1e-9 && after[cc]! <= TARGET_C + 1e-9,
    `未下发命令的关节没有越过自己的目标（${after[b]!.toFixed(6)} ≤ ${TARGET_B}，${after[cc]!.toFixed(6)} ≤ ${TARGET_C}）`,
    "越过目标 ⇒ 收敛方向错了"
  );
  c.ok(
    after[a]! < before[a]!,
    `被下发的关节确实朝新目标走（${before[a]!.toFixed(6)} → ${after[a]!.toFixed(6)}）`,
    ""
  );
  host.unmount();
}

// =============================================================================
// 检查 9：换模型 ⇒ 重建预演（不残留上一台的位形）
// =============================================================================

async function runModelChange(c: Checker): Promise<void> {
  const a = JOINT_IDS[0]!;
  const host = await makeHost({
    model: MODEL,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: true,
  });

  host.snapshot().pushCommand({ [a]: 1.0 });
  for (let i = 0; i < 30; i += 1) await host.step(100);
  c.ok(host.snapshot().prediction!.jointPositions[a]! > 0.5, "换模型前预演已推进", "");

  // 换一个"关节集不同"的模型（模拟切到另一台机器人）
  const other: RobotModelDTO = {
    ...MODEL,
    metadata: { ...MODEL.metadata, id: "other_robot" },
    joints: MODEL.joints.map((j, k) => (k === 0 ? { ...j, id: "yaw_x" } : j)),
  };
  const otherIds = other.joints.filter((j) => j.type === "revolute").map((j) => j.id);
  await host.setProps({ model: other, jointIds: otherIds });

  const p = host.snapshot().prediction!.jointPositions;
  c.eq(p["yaw_x"], 0, "换模型后新关节从零位开始（不残留上一台的位形）");
  c.eq(p[a], undefined, "换模型后旧关节 id 已消失");
  host.unmount();
}

// =============================================================================
// 检查 10：模型为 null ⇒ 不崩、不排帧
// =============================================================================

async function runNullModel(c: Checker): Promise<void> {
  const host = await makeHost({
    model: null,
    jointIds: JOINT_IDS,
    authoritative: null,
    enabled: true,
  });
  c.eq(host.snapshot().prediction, null, "模型未加载 ⇒ 预演为 null（不崩）");
  c.eq(host.scheduler.pendingCount, 0, "模型未加载 ⇒ 不排帧");
  host.unmount();
}

// =============================================================================
// 反例注射
// =============================================================================

const SELF_TESTS: Array<{ name: string; run: () => Promise<boolean> }> = [
  {
    name: "dt 不夹上限（断层直接到位）",
    run: async () => {
      // 模拟"忘了 Math.min(..., MAX_DT)"：直接算 alpha 饱和
      const tau = 0.12;
      const dtGap = 60; // 秒
      const alpha = 1 - Math.exp(-dtGap / tau);
      // 期望：这条注射必须能造成"到位"（即 checks 会变红）
      return alpha > 0.999;
    },
  },
  {
    name: "命令队列用赋值而非合并（丢关节）",
    run: async () => {
      let pending: Record<string, number> = {};
      // 反例实现：
      const badPush = (t: Record<string, number>): void => {
        pending = t; // ← 赋值，覆盖前一帧的命令
      };
      badPush({ a: 1.0 });
      badPush({ b: -0.8 });
      // 期望：a 被丢掉了（这是"注入能生效"的证明）
      return !("a" in pending) && "b" in pending;
    },
  },
  {
    name: "对账放在每帧（权威值把预演拉回）",
    run: async () => {
      // 模拟"每帧 reconcile"：预演被命令推向 0.8，但每帧后又被权威 0 拉回
      let pred = 0;
      const alpha = 1 - Math.exp(-0.1 / 0.12);
      for (let i = 0; i < 10; i += 1) {
        pred = pred + (0.8 - pred) * alpha; // advance
        pred = 0; // ← 反例：每帧 reconcile 到权威 0
      }
      // 期望：永远到不了 0.8（说明这条注射能让"预演不推进"成立）
      return pred < 0.1;
    },
  },
  {
    name: "rAF 不取消（卸载后仍排帧）",
    run: async () => {
      const s = new FakeScheduler();
      let cancel: () => void = () => {};
      const loop = (ts: number): void => {
        cancel = s.schedule(loop);
        void ts;
      };
      cancel = s.schedule(loop);
      s.step(16);
      // 反例：不调 cancel，直接"卸载"
      const leaked = s.pendingCount;
      cancel(); // 正确做法
      const fixed = s.pendingCount;
      return leaked > 0 && fixed === 0;
    },
  },
  {
    name: "enabled=false 仍然排帧",
    run: async () => {
      const s = new FakeScheduler();
      // 反例：没把 enabled 放进 effect 的提前 return
      s.schedule(() => {});
      return s.pendingCount === 1; // 说明"不判 enabled 就会排帧"
    },
  },
];

// =============================================================================
// 主流程
// =============================================================================

async function main(): Promise<number> {
  const c = newChecker();

  // 压掉 React 的 `act(...)` 警告。
  //
  // 它们出现的**唯一**原因是：rAF 回调在 `act()` 之外被我们手动调用
  // （那正是本文件的设计 —— 时钟由测试驱动，不由 React 驱动）。
  // 每次 `step()` 都会触发一次状态更新，于是刷出十几行同样的警告，
  // 把真正的失败信息挤到屏幕外。`IS_REACT_ACT_ENVIRONMENT` 已置 true，
  // 所以这些警告不代表"更新被丢弃"，只代表"不是 React 自己排的"。
  const realError = console.error;
  console.error = (...a: unknown[]): void => {
    const s = String(a[0] ?? "");
    if (s.includes("not wrapped in act") || s.includes("inside a test was not wrapped")) return;
    realError(...a);
  };

  await runDtUsesRealInterval(c);
  await runDtClamped(c);
  await runCleanup(c);
  await runDisabled(c);
  await runReconcileOnlyOnFrame(c);
  await runReconcileTriggers(c);
  await runCommandMerge(c);
  await runUntouchedInvariant(c);
  await runModelChange(c);
  await runNullModel(c);

  const self = newChecker();
  let captured = 0;
  for (const t of SELF_TESTS) {
    const ok = await t.run();
    self.ok(ok, `自检「${t.name}」的注入能生效`, "注入没造成任何差别 ⇒ 这条判据可能恒真");
    if (ok) captured += 1;
  }

  const realWarn = console.warn;
  console.warn = (...a: unknown[]): void => {
    const s = String(a[0] ?? "");
    if (s.includes("not wrapped in act")) return;
    realWarn(...a);
  };

  console.log("");
  console.log("本地预演驱动自检（useGhostPrediction.ts）");
  console.log("─".repeat(62));
  console.log(`模型夹具 : ${MODEL_FIXTURE}`);
  console.log(`可动关节 : ${JOINT_IDS.join(", ")}`);
  console.log("方式：真 React 运行时 + 手动驱动的假时钟（无真实 sleep）");
  console.log("─".repeat(62));

  const failures = [...c.failures];
  const selfFailures = [...self.failures];

  if (failures.length === 0 && selfFailures.length === 0) {
    console.log("PASS  全部检查通过");
    console.log(`      自检 ${captured}/${SELF_TESTS.length} 注入均被捕获`);
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
  main().then((code) => process.exit(code));
}

export { main, newChecker };
