/**
 * =============================================================================
 * useGhostPrediction.ts —— 本地预演（ghost）的**驱动**
 * -----------------------------------------------------------------------------
 * `predictor.ts` 是纯函数；本文件负责把"时间"这一维接上去。它做三件事：
 *
 * ```text
 * ① 每帧（rAF）按真实 dt 调 advance()，把用户命令推进成预测位形
 * ② robot_state 回帧到达时调 reconcile()，判断"预测是否已经跑飞"
 * ③ 把"该画哪个位形"作为结果交给调用方
 * ```
 *
 * ## ★ 为什么不全放进 App.tsx
 *
 * `App.tsx` 里已经有 4 个 effect 在管"拉列表 / 拉模型"，再塞进"每帧推进 +
 * 回帧对账"会让"哪条路径改了哪个 state"完全无法追踪。更要紧的是
 * **它需要被测试** —— 而装在 `App.tsx` 里就只能靠端到端猜。
 *
 * ⇒ 抽出成 hook，用**注入的时钟**在 Node 里跑（见 `ghost.check.ts`）。
 *   `App.tsx` 只剩"调用它、把结果传给 <RobotScene/>"。
 *
 * ## ★ rAF 与 dt
 *
 * `dt` 用**真实经过时间**（两帧 timestamp 之差），不用固定的 1/60：
 * 后台标签页里 rAF 会降到 ~1fps，若假设 dt=1/60，预演会"走得比现实慢 60 倍"
 * —— 而用户切回来看到的是一台爬行的幽灵臂，且没有任何报错。
 *
 * `dt` 同时**要夹上限**（`MAX_DT`）：标签页被挂起再唤醒时，两帧时间差可能是
 * 几十秒 ⇒ `alpha = 1 − exp(−dt/τ)` 直接饱和到 1 ⇒ 预演瞬移。
 * 上限取 0.1 s：超过它就不是"渲染节流"，而是"时间断层"。
 *
 * ## ★ 为什么要"等回帧"而不是每帧都对账
 *
 * `reconcile` 的语义是"与**权威**比"。权威只在回帧时更新。
 * 若每帧都拿同一个（可能已过时的）权威值去比，会在后端延迟期间
 * **反复**触发重同步 —— 表现为幽灵臂每帧被拉回后端位置、抖成一团。
 * ⇒ 只在 `authoritative` 的**引用**变化时调用。
 *
 * ## ★ 为什么必须传真 `model`
 *
 * `advance` / `reconcile` 内部用 `forwardKinematics(model, q)` 算 TCP。
 * Core 的通用 FK 需要**完整 link tree**（沿 Link-Joint 链相乘）。
 * 摆一个"只有 joints 没有 links"的假模型不会得到一个可用的 TCP，
 * 只会抛异常 → `safeTcp` 兜成 `NaN` → 幽灵臂的 TCP 标记位置无意义。
 * ⇒ 接真 `RobotModelDTO`，并把它当**引用稳定**的参数传进来（`useMemo`）。
 */

import { useCallback, useEffect, useRef, useState } from "react";

import type { RobotModelDTO } from "../viewer/model.ts";
import { advance, initialPrediction, reconcile } from "./predictor.ts";
import type { Prediction, PredictorOptions } from "./predictor.ts";

/**
 * 单帧最大步进时长（秒）。
 *
 * 超过它就是"时间断层"（标签页被挂起 / 断点暂停 / 系统休眠），
 * 不是"渲染变慢"。见文件头。
 */
export const MAX_DT = 0.1;

/**
 * "已收敛"阈值（rad）。小于它的变化不推送 React 状态。
 *
 * 1e-5 rad ≈ 0.0006° —— 远低于任何可见角度（SVG/WebGL 的一个像素
 * 在 0.3m 距离上约对应 0.001 rad），但足以让"已经到位"停止触发渲染。
 *
 * ⚠️ 它**只**影响"要不要 setState"，不影响 `predRef` 的推进 ——
 *    见 tick 里的注释。
 */
export const SETTLE_EPS = 1e-5;

/** 帧调度器。`cb` 收到的时间戳单位是**毫秒**（与 rAF 一致）。 */
export type FrameScheduler = (cb: (timestamp: number) => void) => () => void;

const defaultScheduler: FrameScheduler = (cb) => {
  if (typeof requestAnimationFrame !== "function") {
    // 无 rAF（Node / SSR）⇒ 不推进，退化成静态预演。
    // 刻意**不**用 setInterval 兜底：那会在测试里制造不可控的定时器。
    return () => {};
  }
  const handle = requestAnimationFrame(cb);
  return () => cancelAnimationFrame(handle);
};

export interface GhostPredictionOptions {
  /** 用于算 TCP 与读限位的模型。**引用必须稳定**（用 `useMemo`）。 */
  readonly model: RobotModelDTO | null;
  /** 要跟踪的关节（`vm.movableJointIds`）。 */
  readonly jointIds: readonly string[];
  /** 权威关节角（`robot_state.joints`）。**null = 后端还没上报**。 */
  readonly authoritative: Readonly<Record<string, number>> | null;
  /** 关掉它可以彻底停掉预演（省 rAF，也让"幽灵臂"能真的消失）。 */
  readonly enabled?: boolean;
  /** 传给 `advance` / `reconcile` 的参数（τ / 容差 / 单步上限）。 */
  readonly options?: PredictorOptions;
  /** 供测试注入。默认 `requestAnimationFrame`。 */
  readonly scheduler?: FrameScheduler;
}

export interface GhostPrediction {
  /**
   * 当前该画的位形。`source === "authoritative"` 表示它刚刚被后端
   * 纠正过（UI 可以据此提示"已同步"）。
   */
  readonly prediction: Prediction | null;
  /** 下发一条命令（**只**改预演，不发 WebSocket —— 发不发由调用方决定）。 */
  readonly pushCommand: (targets: Readonly<Record<string, number>>) => void;
}

export function useGhostPrediction({
  model,
  jointIds,
  authoritative,
  enabled = true,
  options,
  scheduler,
}: GhostPredictionOptions): GhostPrediction {
  const [prediction, setPrediction] = useState<Prediction | null>(null);
  const predRef = useRef<Prediction | null>(null);

  // 目标队列：滑块拖动可能在一帧内来好几次。
  // 用**合并**而不是"后一条覆盖前一条"：两条命令若针对**不同关节**，
  // 覆盖会丢掉其中一个（先拖 A 再拖 B，A 的意图消失）。
  //
  // ★ 它是**持续的目标**，不是一次性脉冲。
  //
  //   语义上"命令"= "让这个关节去某处"，这个意图在下一帧被兑现后
  //   **依然成立**，直到有新命令覆盖它。
  //   若每帧清空（把命令当脉冲），预演只会走一帧就停住 ——
  //   症状是"拖一点点，幽灵臂只动一丝"，而且看起来像"滞后系数算错了"，
  //   极难定位。后端的语义也是持续的：
  //   `mujoco_backend.send_command` 把目标存进 actuator 直到下次被改。
  const pendingRef = useRef<Record<string, number>>({});

  // 这些放进 ref，避免把它们塞进 effect 依赖导致每帧重建 rAF 循环。
  const modelRef = useRef(model);
  modelRef.current = model;
  const idsRef = useRef(jointIds);
  idsRef.current = jointIds;
  const optRef = useRef(options);
  optRef.current = options;

  // `jointIds` 用内容做 key —— 数组每次都是新引用。
  const idsKey = jointIds.join(",");
  // 模型用身份做 key（同一模型内容变了也应该重置预演）。
  const modelKey = model === null ? "" : String(modelHash(model));

  // 建 / 重建初值：换模型或换关节集时重置。
  useEffect(() => {
    if (!enabled) return;
    const m = modelRef.current;
    if (m === null || idsRef.current.length === 0) {
      predRef.current = null;
      setPrediction(null);
      return;
    }
    const init = initialPrediction(m, idsRef.current);
    predRef.current = init;
    setPrediction(init);
    // ⚠️ **刻意不清空 `pendingRef`**。
    //
    //    早先这里写了 `pendingRef.current = {}`，后果是一个真实的丢命令窗口：
    //    调用方"先 pushCommand 再触发一次重渲染（换了 model 引用 / enabled）"
    //    就会把刚入队的命令抹掉 —— 表现成"拖了滑块但幽灵臂不动"，
    //    而且只在特定时序下复现。
    //
    //    保留是**安全**的：换模型后旧命令里的关节在新模型里不存在，
    //    `advance` 对"命令里出现但模型没有的关节"是**忽略**（不抛）——
    //    这正是它设计好的语义（见 predictor.ts 的注释）。
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [modelKey, idsKey, enabled]);

  // rAF 推进
  useEffect(() => {
    if (!enabled) return;
    if (model === null || jointIds.length === 0) return;

    let lastTs: number | null = null;
    let cancelled = false;
    let cancel: () => void = () => {};

    const tick = (ts: number): void => {
      if (cancelled) return;
      if (lastTs === null) lastTs = ts;
      const rawDt = (ts - lastTs) / 1000;
      lastTs = ts;
      // 下限 0：注入的假时钟可能给同一个 ts（一帧内多次调用）。
      // 上限 MAX_DT：见文件头"时间断层"。
      const dt = Math.min(Math.max(rawDt, 0), MAX_DT);

      const cur = predRef.current;
      const m = modelRef.current;
      if (cur !== null && m !== null && dt > 0) {
        // ⚠️ **不**清空：命令是持续目标（见 `pendingRef` 的注释）。
        //    没有目标时跳过 advance —— 不空转 `forwardKinematics`
        //    （每次都要走完整条 Link-Joint 链，是每帧几毫秒的纯浪费）。
        const cmds = pendingRef.current;
        if (Object.keys(cmds).length > 0) {
          const next = advance(m, cur, cmds, dt, optRef.current);
          // ★ 先无条件推进 ref，再**按可见性**决定要不要 setState。
          //
          //   顺序不能反：若"变化太小就不更新 ref"，预演会永远停在
          //   离目标 SETTLE_EPS 的地方，而下一帧仍从那个旧值算 ——
          //   值会在阈值边缘抖动，且 `advance` 每帧都被调用。
          predRef.current = next;
          // 一阶滞后是渐近逼近（`q → target` 永远差一点），
          // `next !== cur` 恒为真 ⇒ 收敛后仍每帧重渲染整个 scene graph。
          // 只在变化超过 1e-5 rad（≈0.0006°，远低于可见）时才推送。
          if (changedBeyond(next, cur, SETTLE_EPS)) {
            setPrediction(next);
          }
        }
      }
      cancel = (scheduler ?? defaultScheduler)(tick);
    };

    cancel = (scheduler ?? defaultScheduler)(tick);
    return () => {
      cancelled = true;
      cancel();
    };
  }, [model, modelKey, idsKey, enabled, scheduler, jointIds.length]);

  // 回帧对账。
  //
  // ⚠️ 依赖是 `authoritative` 的**引用**：后端每回一帧就是新对象 ⇒ 触发一次。
  //    若写成"每帧都调"，后端延迟期间会反复重同步（见文件头）。
  useEffect(() => {
    if (!enabled) return;
    if (authoritative === null) return;
    const cur = predRef.current;
    const m = modelRef.current;
    if (cur === null || m === null) return;
    const next = reconcile(m, cur, authoritative, optRef.current);
    predRef.current = next;
    setPrediction(next);
  }, [authoritative, enabled]);

  const pushCommand = useCallback(
    (targets: Readonly<Record<string, number>>): void => {
      Object.assign(pendingRef.current, targets);
    },
    []
  );

  return { prediction, pushCommand };
}

/**
 * 两个预测之间是否有**超过阈值**的关节角变化。
 *
 * 只比关节角，不比 TCP —— TCP 是由关节角算出来的派生量，
 * 比它就等于比两次同一个东西（且 FK 的数值噪声会让它更容易误报"变了"）。
 */
function changedBeyond(a: Prediction, b: Prediction, eps: number): boolean {
  const ak = Object.keys(a.jointPositions);
  const bk = Object.keys(b.jointPositions);
  if (ak.length !== bk.length) return true;
  for (const id of ak) {
    const av = a.jointPositions[id];
    const bv = b.jointPositions[id];
    if (av === undefined || bv === undefined) return true;
    if (Math.abs(av - bv) > eps) return true;
  }
  return false;
}

/**
 * 一个稳定的模型身份标识。
 *
 * 不用 `JSON.stringify(model)` —— 模型有几十个节点，每帧序列化一次
 * 是纯浪费。这里只拼"机器人 id + 关节数 + 首尾关节 id"，
 * 足以区分"同一个模型"与"换了模型"。
 *
 * ⚠️ 机器人 id 在 `metadata.id`，**不是**顶层 `robot` 字段
 *    （`RobotModelDTO` 有 metadata / links / joints / …，没有 `robot`）。
 */
function modelHash(model: RobotModelDTO): string {
  const js = model.joints ?? [];
  const n = js.length;
  const first = n > 0 ? js[0]!.id : "";
  const last = n > 0 ? js[n - 1]!.id : "";
  const rid = model.metadata?.id ?? "?";
  return `${rid}#${n}#${first}#${last}`;
}
