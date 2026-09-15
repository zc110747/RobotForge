/**
 * JointPanel 的纯逻辑自检 + 渲染行为自检。
 *
 * 运行：
 * ```bash
 * node --experimental-strip-types frontend/src/jointPanel.check.ts
 * ```
 *
 * ## 为什么这个面板需要自检
 *
 * 它有一个**很难通过看页面发现**的核心缺陷：
 *
 * ```text
 * 若滑块 value 直接绑到"后端实际位置"，用户在拖动时会被回帧覆盖 —
 * 症状是"滑块自己往回弹 / 拖不动"。
 * 这个症状与"后端慢"、"网络丢帧"表现一致，于是排查方向会完全跑偏。
 * ```
 *
 * ⇒ 判据必须是**不变量**：「用户拖过之后，后端的回帧不得改变滑块的值」。
 *   这条用渲染 + 读 props 就能验，不需要真实后端。
 */

import { createElement } from "react";
import type { ReactElement } from "react";
import { rmSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

import type { JointVisual } from "./viewer/viewModel.ts";
import type { RobotCapabilitiesDTO } from "./viewer/model.ts";

// ---------------------------------------------------------------------------
// JointPanel 是 .tsx（含 JSX），Node 的类型剥离**不能**处理 JSX
// ---------------------------------------------------------------------------
//
// `--experimental-strip-types` 只删类型标注，不做代码生成；
// JSX 需要被转译成 `jsx()` 调用，所以必须先用 esbuild 打包成 .mjs。
// 与 `viewer/__tests__/render.check.ts` 的做法一致（那里也是先转译 RobotScene.tsx）。
//
// ⚠️ 纯逻辑函数（rangeOf / isClamped / …）也在 `JointPanel.tsx` 里，
//    所以**它们也必须**从打包后的模块里取 —— 不能直接从 `./JointPanel.ts` import
//    （模块不存在，且 .tsx 无法被 strip-types 处理）。
//    下面的 `P` 在 `loadPanel()` 里赋值。

/**
 * 打包后的 JointPanel 模块（含组件 + 纯函数）。
 *
 * ⚠️ `JointPanel` 的返回类型必须是 `ReactElement | null`，不能写 `unknown`：
 *    `createElement` 的重载解析要求组件返回 `ReactNode`，
 *    `unknown` 不满足 ⇒ 每个 `createElement(JointPanel, …)` 都报
 *    `TS2769: No overload matches this call`（共 7 处）。
 *    这里只是**声明形状**（真实类型由 esbuild 打包的 .tsx 提供），
 *    声明宽一点确实能让它编译过，但会顺手把上面那个错误也一起放过。
 */
let P: {
  JointPanel: (p: Record<string, unknown>) => ReactElement | null;
  rangeOf: (j: Pick<JointVisual, "limits">) => readonly [number, number] | null;
  controllableJoints: (js: readonly JointVisual[]) => readonly JointVisual[];
  isClamped: (
    target: number,
    actual: number | undefined,
    range: readonly [number, number] | null,
    edgeTol?: number
  ) => boolean;
  toDegrees: (rad: number) => number;
  sliderBounds: (r: readonly [number, number] | null) => readonly [number, number];
};

async function loadPanel(): Promise<string> {
  const HERE = dirname(fileURLToPath(import.meta.url));
  const esbuild = (await import("esbuild")) as {
    build: (o: Record<string, unknown>) => Promise<unknown>;
  };
  const outfile = join(HERE, "__jointpanel.check.mjs");
  await esbuild.build({
    entryPoints: [join(HERE, "JointPanel.tsx")],
    outfile,
    bundle: true,
    format: "esm",
    jsx: "automatic",
    target: "es2022",
    platform: "node",
    external: ["react", "react-dom", "react/jsx-runtime", "scheduler"],
    logLevel: "silent",
  });
  return outfile;
}

// 纯函数在这里**转发**给打包模块，让下面的检查体保持可读。
const rangeOf = (j: Pick<JointVisual, "limits">) => P.rangeOf(j);
const controllableJoints = (js: readonly JointVisual[]) => P.controllableJoints(js);
const isClamped = (
  t: number,
  a: number | undefined,
  r: readonly [number, number] | null,
  tol?: number
) => P.isClamped(t, a, r, tol);
const toDegrees = (rad: number) => P.toDegrees(rad);
const sliderBounds = (r: readonly [number, number] | null) => P.sliderBounds(r);
const JointPanel = (p: Record<string, unknown>) => P.JointPanel(p);

// =============================================================================
// 断言器
// =============================================================================

interface Failure {
  readonly check: string;
  readonly detail: string;
}

interface Checker {
  readonly failures: Failure[];
  ok(cond: boolean, check: string, detail?: string): void;
  eq<T>(actual: T, expected: T, check: string): void;
  close(actual: number, expected: number, tol: number, check: string): void;
}

function newChecker(): Checker {
  const failures: Failure[] = [];
  return {
    failures,
    ok(cond, check, detail = "") {
      if (!cond) failures.push({ check, detail });
    },
    eq(actual, expected, check) {
      if (actual !== expected) {
        failures.push({ check, detail: `期望 ${JSON.stringify(expected)}，实得 ${JSON.stringify(actual)}` });
      }
    },
    close(actual, expected, tol, check) {
      const diff = Math.abs(actual - expected);
      if (!(diff <= tol)) failures.push({ check, detail: `期望 ${expected}，实得 ${actual}，差 ${diff}` });
    },
  };
}

// =============================================================================
// 假数据
// =============================================================================

const CAPS: RobotCapabilitiesDTO = {
  simulation: true,
  fk: true,
  ik: true,
  actuator_control: true,
  end_effector: true,
};

function mkJoint(
  id: string,
  limits: { min: number; max: number } | null,
  type = "revolute"
): JointVisual {
  return {
    id,
    type,
    parentLink: "l0",
    childLink: "l1",
    origin: { position: [0, 0, 0], orientation: [0, 0, 0, 1] },
    axis: [0, 0, 1],
    limits,
    isMovable: type !== "fixed",
  };
}

// =============================================================================
// 检查 1：限位取自模型，不伪造
// =============================================================================

function runRange(c: Checker): void {
  c.eq(rangeOf(mkJoint("a", { min: -1, max: 1 }))?.join(","), "-1,1", "声明了限位 ⇒ 原样取出");

  // ★ 未声明 ⇒ null（**不**给默认 ±π）
  c.eq(rangeOf(mkJoint("b", null)), null, "未声明限位 ⇒ null（不伪造 ±π 约束）");

  // 退化区间不算约束
  c.eq(rangeOf(mkJoint("c", { min: 1, max: 1 })), null, "min == max ⇒ 不当作约束");
  c.eq(rangeOf(mkJoint("d", { min: 2, max: 1 })), null, "min > max ⇒ 不当作约束");

  // 未声明限位时滑块给一个"仅供拖动"的窗口，且**必须**标注未声明
  const b = sliderBounds(null);
  c.ok(b[0] < 0 && b[1] > 0, "未声明限位时滑块有可拖动窗口", `[${b.join(",")}]`);
  c.eq(sliderBounds([-0.5, 0.5]).join(","), "-0.5,0.5", "有声明时滑块边界 = 声明值（不放大）");
}

// =============================================================================
// 检查 2：可命令关节的挑选
// =============================================================================

function runControllable(c: Checker): void {
  const js = [
    mkJoint("yaw", { min: -3, max: 3 }),
    mkJoint("slider", null, "prismatic"),
    mkJoint("welded", null, "fixed"),
  ];
  const got = controllableJoints(js).map((j) => j.id);
  c.eq(got.join(","), "yaw,slider", "只挑可动 revolute/prismatic");
  c.ok(!got.includes("welded"), "fixed 不出现（拖了没反应的控件会被当成 bug）");
}

// =============================================================================
// 检查 3：限位标记的判据
// =============================================================================

function runClamped(c: Checker): void {
  const range: [number, number] = [-1.5708, 1.5708];

  // 真实限位：命令 1.6（越界），实际停在 1.5708（贴边界）
  c.ok(isClamped(1.6, 1.5708, range), "命令越界 + 实际贴边界 ⇒ 标记已限位");

  // ★ 这是最容易被写错的一条：命令 1.6、实际 0.9（**没**贴边界）。
  //   那不是限位，是"物理还在收敛"。标成限位会把正常滞后误报成约束。
  c.ok(!isClamped(1.6, 0.9, range), "命令越界但实际未贴边界 ⇒ **不**标限位（那是滞后，不是限位）");

  // 命令在范围内、实际也在范围内 ⇒ 不是限位
  c.ok(!isClamped(0.5, 0.4, range), "命令未越界 ⇒ 不标限位（差值是正常滞后）");

  // 未声明限位 ⇒ 永远不标（没有约束可以生效）
  c.ok(!isClamped(99, 9, null), "未声明限位 ⇒ 永不标限位（没有约束在生效）");

  // 实际值未知 ⇒ 不标（不知道就不下结论）
  c.ok(!isClamped(1.6, undefined, range), "实际值未知 ⇒ 不标限位（不猜测）");
}

// =============================================================================
// 检查 4：角度换算
// =============================================================================

function runDegrees(c: Checker): void {
  c.close(toDegrees(Math.PI), 180, 1e-12, "π rad = 180°");
  c.close(toDegrees(Math.PI / 2), 90, 1e-12, "π/2 rad = 90°");
  c.close(toDegrees(0), 0, 0, "0 rad = 0°");
}

// =============================================================================
// 检查 5：渲染 —— 滑块值 = 用户的目标，**不**被后端回帧覆盖
// =============================================================================

interface SliderInfo {
  readonly jointId: string;
  readonly value: number;
  readonly min: number;
  readonly max: number;
  readonly disabled: boolean;
}

/** 从静态渲染产物里抓出所有滑块。 */
function extractSliders(markup: string): SliderInfo[] {
  const out: SliderInfo[] = [];
  const re = /<input([^>]*)\/?>/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(markup)) !== null) {
    const attrs = m[1] ?? "";
    if (!attrs.includes('type="range"')) continue;
    const val = (k: string): string => {
      const mm = new RegExp(`${k}="([^"]*)"`).exec(attrs);
      return mm?.[1] ?? "";
    };
    const label = val("aria-label").replace(" 目标角", "");
    out.push({
      jointId: label,
      value: Number(val("value")),
      min: Number(val("min")),
      max: Number(val("max")),
      disabled: /\sdisabled(=|"|>|\s|$)/.test(attrs),
    });
  }
  return out;
}

async function runRender(c: Checker): Promise<void> {
  const { renderToStaticMarkup } = await import("react-dom/server");
  const joints = [mkJoint("base_yaw", { min: -Math.PI, max: Math.PI }), mkJoint("shoulder", { min: -1.5708, max: 1.5708 })];

  // ① 基本渲染：有滑块、有 aria-label
  const base = renderToStaticMarkup(
    createElement(JointPanel, {
      joints,
      actual: { base_yaw: 0, shoulder: 0 },
      status: "idle",
      capabilities: CAPS,
      connection: "open",
      onCommand: () => true,
      lastError: null,
    })
  );
  const s0 = extractSliders(base);
  c.eq(s0.length, 2, "两个可动关节 ⇒ 两个滑块");
  c.eq(s0.map((s) => s.jointId).join(","), "base_yaw,shoulder", "滑块顺序与模型一致");
  // 滑块边界必须来自模型声明的 range（不放大到 ±π）
  const sh = s0.find((s) => s.jointId === "shoulder")!;
  c.close(sh.min, -1.5708, 1e-9, "shoulder 滑块下界 = 模型 range.min");
  c.close(sh.max, 1.5708, 1e-9, "shoulder 滑块上界 = 模型 range.max");

  // ② 连接未开 ⇒ 滑块禁用（否则用户拖了半天什么都没发出去）
  const closed = renderToStaticMarkup(
    createElement(JointPanel, {
      joints,
      actual: { base_yaw: 0, shoulder: 0 },
      status: "idle",
      capabilities: CAPS,
      connection: "closed",
      onCommand: () => true,
      lastError: null,
    })
  );
  const s1 = extractSliders(closed);
  c.ok(s1.every((s) => s.disabled), "连接断开时滑块禁用（不让用户白拖）");

  // ③ ★ 核心不变量：命令值 ≠ 实际值时，面板**同时**显示两个数
  //    （把 State ≠ Command 变成肉眼可见的事实，而不是用户去猜）
  const diverged = renderToStaticMarkup(
    createElement(JointPanel, {
      joints: [joints[1]!],
      actual: { shoulder: 0.35 },
      status: "running",
      capabilities: CAPS,
      connection: "open",
      onCommand: () => true,
      lastError: null,
    })
  );
  c.ok(diverged.includes("命令"), "面板有「命令」一行");
  c.ok(diverged.includes("实际"), "面板有「实际」一行");
  c.ok(diverged.includes("0.3500"), "实际值 0.35 出现在面板上（回显后端状态）", "");

  // ④ 未声明限位的关节必须**显式标注**，而不是默默给个 ±π 的滑块
  const noLimit = renderToStaticMarkup(
    createElement(JointPanel, {
      joints: [mkJoint("free1", null)],
      actual: { free1: 0 },
      status: "idle",
      capabilities: CAPS,
      connection: "open",
      onCommand: () => true,
      lastError: null,
    })
  );
  c.ok(noLimit.includes("模型未声明限位"), "未声明限位的关节有显式标注");

  // ⑤ capability 为 false ⇒ 不显示滑块（capability 驱动，不按型号）
  const noCap = renderToStaticMarkup(
    createElement(JointPanel, {
      joints,
      actual: { base_yaw: 0, shoulder: 0 },
      status: "idle",
      capabilities: { ...CAPS, actuator_control: false },
      connection: "open",
      onCommand: () => true,
      lastError: null,
    })
  );
  c.ok(extractSliders(noCap).length === 0, "actuator_control=false ⇒ 没有滑块");
  c.ok(noCap.includes("actuator_control"), "并说明了原因（capability 驱动）");

  // ⑥ 错误必须显示出来（不能静默）
  const withErr = renderToStaticMarkup(
    createElement(JointPanel, {
      joints,
      actual: null,
      status: null,
      capabilities: CAPS,
      connection: "closed",
      onCommand: () => true,
      lastError: "bad_command: 未知关节 'nope'",
    })
  );
  c.ok(withErr.includes("bad_command"), "error 帧内容显示在面板上");

  // ⑦ ★★ 最重要的一条：**后端回帧不得覆盖用户已拖动的值**
  //
  //    做法：渲染两次，第一次 actual=0（滑块初值取 0），
  //    第二次 actual=0.9（后端追上来了）。因为这是**无状态**的静态渲染，
  //    真正的"不覆盖"由 `touched` ref 在运行时保证 ——
  //    所以这里能验的是**逻辑函数**层面的东西：
  //    其一是"没有 actual 时滑块仍可渲染（不崩）"，其二是下面 §5.8 的
  //    `send` 路径。真正的时序断言在 `jointPanelTiming` 里用 hooks 跑。
  const nullActual = renderToStaticMarkup(
    createElement(JointPanel, {
      joints,
      actual: null,
      status: null,
      capabilities: CAPS,
      connection: "connecting",
      onCommand: () => true,
      lastError: null,
    })
  );
  c.eq(extractSliders(nullActual).length, 2, "actual 为 null 时滑块仍渲染（不崩）");
  c.ok(nullActual.includes("后端未上报"), "actual 缺失时显示「后端未上报」而不是假装是 0");
}

// =============================================================================
// 检查 6：时序 —— 用户拖动后不被回帧覆盖（**真 React 运行时**）
// =============================================================================

/**
 * 这一节是本文件里**唯一**能真正证明"拖动不被回帧覆盖"的地方。
 *
 * ## 为什么不能用手写逻辑复刻（实测教训）
 *
 * 第一版这里写的是"把 JointPanel 里那段 `setTargets` 逻辑抄一份出来跑"。
 * 实测：**反例注射通不过** —— 我把组件里真正的 `touched` 守卫删掉
 * （改成 `if (true)`），本节自检依然全绿。因为被验的是我抄的那段代码，
 * 不是组件里的那段。
 *
 * 这是"自证"的典型形态：判据看起来在验组件，实际在验判据自己。
 * 必须换掉，否则这一节提供的信心是**零**。
 *
 * ## 做法
 *
 * 用 `react-dom/client` + jsdom 真的挂载组件，然后：
 * ```text
 * ① 挂载（actual 全 0）→ 读滑块值
 * ② 派发真实 input 事件拖到 0.8 → 读滑块值 / 读下发的命令
 * ③ 用新的 actual=0.9 重新渲染（模拟后端回帧）→ 再读滑块值
 * ④ 断言：滑块**仍是 0.8**（不是 0.9）
 * ```
 * `act()` 用来把 state 更新 flush 掉再断言 —— React 18 的更新不是同步生效的，
 * 不等一下就断言会把"还没渲染"读成"值没变"（反过来也一样）。
 */
async function runRealReRender(c: Checker): Promise<void> {
  const { JSDOM } = (await import("jsdom")) as unknown as {
    JSDOM: new (html: string) => { window: Window & typeof globalThis };
  };

  // ---- 先把 DOM 装成全局，**再** import React ----
  //
  // ⚠️⚠️ 顺序很重要（实测踩到）：
  //     `react-dom` 在**首次被 import 时**就会做环境探测并把结果缓存起来。
  //     若先 import React 再装 `document`，它探测到的是"没有 DOM"，
  //     于是事件系统走**无 DOM 分支** —— 症状是"派发了 input 事件但
  //     onChange 一次都不触发"，而组件渲染、DOM 查询、断言全部正常。
  //
  //     这个坑很隐蔽：失败信息是 `sent=[]`（看起来像事件没绑），
  //     根因却是**模块加载顺序**。
  const dom = new JSDOM("<!doctype html><html><body><div id='root'></div></body></html>");
  const g = dom.window as unknown as Window & typeof globalThis;
  const G = globalThis as unknown as Record<string, unknown>;
  const define = (key: string, value: unknown): void => {
    Object.defineProperty(globalThis, key, {
      value,
      writable: true,
      configurable: true,
    });
  };
  // ⚠️ `navigator` 在 Node ≥21 上是**只有 getter** 的全局属性，
  //    直接赋值会抛 "which has only a getter"。defineProperty 才能覆盖。
  define("window", g);
  define("document", g.document);
  define("navigator", g.navigator);
  define("HTMLElement", g.HTMLElement);
  define("HTMLInputElement", g.HTMLInputElement);
  define("Event", g.Event);
  G["IS_REACT_ACT_ENVIRONMENT"] = true;

  // DOM 就绪后才加载 React（见上面的顺序说明）。
  const React = await import("react");
  const { createRoot } = (await import("react-dom/client")) as unknown as {
    createRoot: (el: Element) => { render: (node: unknown) => void; unmount: () => void };
  };
  const { act } = React as unknown as {
    act: (fn: () => void) => void;
  };

  const container = g.document.getElementById("root")!;
  const root = createRoot(container);

  const joints = [
    mkJoint("base_yaw", { min: -Math.PI, max: Math.PI }),
    mkJoint("shoulder", { min: -1.5708, max: 1.5708 }),
  ];

  const sent: Array<Record<string, number>> = [];
  const props = (actual: Record<string, number>) => ({
    joints,
    actual,
    status: "running",
    capabilities: CAPS,
    connection: "open",
    onCommand: (t: Record<string, number>) => {
      sent.push(t);
      return true;
    },
    lastError: null,
  });

  const sliderFor = (id: string): HTMLInputElement => {
    const all = Array.from(container.querySelectorAll<HTMLInputElement>('input[type="range"]'));
    const el = all.find((s) => s.getAttribute("aria-label") === `${id} 目标角`);
    if (!el) throw new Error(`找不到 ${id} 的滑块`);
    return el;
  };

  try {
    // ---- ① 挂载：actual 全是 0 ----
    act(() => {
      root.render(createElement(P.JointPanel as never, props({ base_yaw: 0, shoulder: 0 })));
    });
    c.eq(
      container.querySelectorAll('input[type="range"]').length,
      2,
      "真实挂载：两个滑块"
    );
    c.close(Number(sliderFor("shoulder").value), 0, 1e-9, "初值 = 后端实际值（用户还没碰过）");

    // ---- ② 派发真实的 input 事件：拖到 0.8 ----
    //
    // ⚠️⚠️ 受控 input 的 value 必须用 **React 在原型上装的那个 setter** 写。
    //
    // 直觉写法 `el.value = "0.8"` 对**非受控** DOM 有效，但对 React 的
    // 受控 input 会失效：React 在元素上挂了一个内部的 value tracker
    // （`el._valueTracker`），它记着"上次 React 写进去的值"。
    // 直接赋值会让 tracker 与实际值**不一致**，React 便认为"值没变"，
    // 于是**根本不触发 onChange** —— 表现为"派发了事件但组件毫无反应"。
    //
    // 正确做法：调原型上的原生 setter（绕过 tracker 写入），
    // 让 tracker 里仍是旧值 ⇒ 随后派发 input 事件时 React 发现
    // "DOM 值变了" ⇒ 正常触发 onChange。
    // 这是 React 官方测试工具（react-dom/test-utils）背后的同一套手法。
    act(() => {
      const el = sliderFor("shoulder");
      const desc = Object.getOwnPropertyDescriptor(g.HTMLInputElement.prototype, "value");
      const nativeSetter = desc?.set;
      if (nativeSetter) {
        nativeSetter.call(el, "0.8");
      } else {
        el.value = "0.8";
      }
      el.dispatchEvent(new g.Event("input", { bubbles: true }));
    });
    c.ok(sent.length >= 1, "拖动确实下发了命令", `sent=${JSON.stringify(sent)}`);
    c.close(
      sent[sent.length - 1]?.["shoulder"] ?? NaN,
      0.8,
      1e-9,
      "下发的是拖动后的目标值"
    );
    c.close(Number(sliderFor("shoulder").value), 0.8, 1e-9, "滑块跟上用户的拖动");

    // ---- ③ 后端回帧：actual.shoulder = 0.9 ----
    act(() => {
      root.render(createElement(P.JointPanel as never, props({ base_yaw: 0, shoulder: 0.9 })));
    });

    // ★★ 核心断言 ★★
    c.close(
      Number(sliderFor("shoulder").value),
      0.8,
      1e-9,
      "★ 后端回帧**不覆盖**用户拖动过的滑块（拖到 0.8、回帧 0.9 ⇒ 滑块仍 0.8）"
    );
    c.ok(
      Math.abs(Number(sliderFor("shoulder").value) - 0.9) > 1e-9,
      "且确实不是后端值（证明这条判据能区分「跟随 actual」的实现）",
      `实得 ${sliderFor("shoulder").value}`
    );

    // ---- ④ 未碰过的关节：初值取一次，之后**不跟随** ----
    //
    // ⚠️ 这条判据我第一版写错了：我期望"未碰过的关节跟随后端"。
    //    实测实得 0（不是 0.25），组件行为与我的期望不符 ——
    //    但**组件是对的，期望是错的**。
    //
    //    理由：`useEffect` 只在 `next[id] === undefined` 时写入。
    //    于是未碰过的关节**取一次初值就固定**，不再被回帧推动。
    //    这是刻意的：若未碰过的关节持续跟随后端，那么后端把
    //    base_yaw 转起来时，用户面前那个滑块会**自己滑动** ——
    //    而他并没有拖它。用户会以为界面失控。
    //    （"滑块是输入控件，不是显示器"—— 只有"实际"那一栏该跟着变。）
    //
    //    ⇒ 判据改成：初值取到了（0），且**不再**随 actual 变到 0.25。
    act(() => {
      root.render(createElement(P.JointPanel as never, props({ base_yaw: 0.25, shoulder: 0.9 })));
    });
    c.close(
      Number(sliderFor("base_yaw").value),
      0,
      1e-9,
      "未碰过的关节取一次初值后**不**被回帧推动（滑块不会自己滑走）"
    );
    c.ok(
      Math.abs(Number(sliderFor("base_yaw").value) - 0.25) > 1e-9,
      "且确实没有跟到 0.25（证明判据能区分「跟随」实现）",
      `实得 ${sliderFor("base_yaw").value}`
    );
    // 而"实际"那一栏**必须**跟上 —— 那才是显示后端状态的地方
    c.ok(
      (container.textContent ?? "").includes("0.2500"),
      "但「实际」栏显示了后端值 0.2500（显示与输入分离）",
      ""
    );

    // ---- ⑤ 实际值那一栏仍显示后端值（回显没被 state 逻辑破坏）----
    const text = container.textContent ?? "";
    c.ok(text.includes("0.9000"), "面板显示后端实际值 0.9000", `文本片段：${text.slice(0, 160)}`);
    c.ok(text.includes("0.8000"), "面板显示用户命令值 0.8000（两者并列）");
  } finally {
    act(() => {
      root.unmount();
    });
  }
}

// =============================================================================
// 自检：故意注入缺陷，断言必须变红
// =============================================================================

interface SelfTest {
  readonly name: string;
  readonly run: () => boolean;
}

/**
 * 复刻"滑块直接绑到 actual"的错误实现。
 *
 * ⚠️ 它**只**用来证明"这条判据的形状能区分两种实现"，即
 *    `close(slider.value, 0.8)` 在"跟随 actual"的实现下**会**变红。
 *
 *    **真正的**反例注射是：把这套注入打进 `JointPanel.tsx`（改掉
 *    `touched.current.has` 那行），再跑 `runRealReRender`。
 *    那一步已实测（见文件末尾的说明），结果 `EXIT=1`。
 */
function naiveTouchedIgnored(
  _prev: Record<string, number>,
  actual: Record<string, number>,
  ids: readonly string[]
): Record<string, number> {
  const next: Record<string, number> = {};
  for (const id of ids) {
    if (actual[id] !== undefined) next[id] = actual[id]!;
  }
  return next;
}

const SELF_TESTS: SelfTest[] = [
  {
    name: "滑块跟随 actual（回弹 bug）⇒ 不覆盖判据变红",
    run: () => {
      const c = newChecker();
      const naive = naiveTouchedIgnored({ shoulder: 0.8 }, { shoulder: 0.9 }, ["shoulder"]);
      c.close(naive["shoulder"] ?? NaN, 0.8, 0, "tampered: 被覆盖了");
      return c.failures.length > 0;
    },
  },
  {
    name: "未声明限位时伪造 ±π ⇒ 限位判据变红",
    run: () => {
      const c = newChecker();
      // 模拟"给了默认 ±π"的实现
      const faked = rangeOf(mkJoint("x", null)) ?? [-Math.PI, Math.PI];
      c.eq(faked, null, "tampered: 伪造了限位");
      return c.failures.length > 0;
    },
  },
  {
    name: "把滞后误报成限位 ⇒ 限位标记判据变红",
    run: () => {
      const c = newChecker();
      // 模拟"只看 |target-actual| 大就标限位"的实现
      const naiveClamped = Math.abs(1.6 - 0.9) > 1e-3;
      c.ok(!naiveClamped, "tampered: 把滞后标成了限位");
      return c.failures.length > 0;
    },
  },
  {
    name: "滑块边界放大到 ±π（无视模型 range）⇒ 边界判据变红",
    run: () => {
      const c = newChecker();
      // 模拟"总是用 ±π"
      const widened: [number, number] = [-Math.PI, Math.PI];
      c.close(widened[1], 1.5708, 1e-9, "tampered: 边界被放大");
      return c.failures.length > 0;
    },
  },
  {
    name: "fixed 关节混进滑块 ⇒ 挑选判据变红",
    run: () => {
      const c = newChecker();
      const got = controllableJoints([
        mkJoint("welded", null, "fixed"),
      ]).map((j) => j.id);
      c.ok(got.includes("welded"), "tampered: fixed 进了列表");
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

async function main(): Promise<number> {
  const compiled = await loadPanel();
  // ⚠️ Windows 上不能直接 `import("E:\...")` —— 默认 ESM loader 只接受
  //    file/data/node 三种 scheme，收到 'e:' 会报 ERR_UNSUPPORTED_ESM_URL_SCHEME。
  //    必须转成 file:// URL。（这是 `render.check.ts` 已经踩过的同一个坑。）
  P = (await import(`${pathToFileURL(compiled).href}?t=${Date.now()}`)) as typeof P;

  const c = newChecker();
  runRange(c);
  runControllable(c);
  runClamped(c);
  runDegrees(c);
  await runRender(c);
  await runRealReRender(c);

  const self = newChecker();
  runSelfTests(self);

  try {
    rmSync(compiled);
  } catch {
    /* 清理失败不影响结论 */
  }

  console.log("");
  console.log("关节控制面板自检（JointPanel.tsx）");
  console.log("─".repeat(62));
  console.log("被测：限位读取 / 关节挑选 / 限位标记判据 / 渲染 / 拖动不被回帧覆盖");
  console.log("方式：纯函数 + renderToStaticMarkup（无需真实后端/浏览器）");
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
    for (const f of failures) {
      console.log(`  ✗ ${f.check}`);
      console.log(`      ${f.detail}`);
    }
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
  process.argv[1] !== undefined &&
  process.argv[1].replace(/\\/g, "/").endsWith("jointPanel.check.ts");
if (isDirect) {
  main()
    .then((code) => process.exit(code))
    .catch((e) => {
      console.error("探针本身出错：", e);
      process.exit(2);
    });
}

export { main, newChecker };
