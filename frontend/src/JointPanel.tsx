/**
 * =============================================================================
 * JointPanel.tsx —— 关节控制面板（命令下发 + 状态回显）
 * -----------------------------------------------------------------------------
 * §59 的 "命令面板" 落点。本组件负责三件事，**只有**三件：
 *
 * ```text
 * ① 显示每个可动关节的**权威**位置（来自 robot_state，不来自本地预演）
 * ② 拖动滑块 ⇒ 经 ws.sendCommand() 发出一条 robot_command
 * ③ 显示 State ≠ Command 的证据：命令值 与 实际值 并列，且标出被夹紧的关节
 * ```
 *
 * ## ★ 滑块的值不是"当前状态"
 *
 * 这是一个**容易做错且很难发现**的地方。
 *
 * 直觉做法是"滑块 value = 关节当前位置"，于是一边拖一边被后端回帧覆盖。
 * 后果：后端有延迟时，滑块会**自己往回弹**，用户的手感和看到的数字打架，
 * 表现为"拖不动"或"抽搐"。而根因不在后端，在于把"期望值"与"实际值"
 * 混用了一个变量。
 *
 * 所以本组件持有 **两份** 值：
 * ```text
 * target[id]  —— 用户拖出来的**期望**（本组件的本地状态，只由用户输入改变）
 * actual[id]  —— 后端 robot_state 里的**实际**（只读，来自 props）
 * ```
 * 两条都显示，才能看出 §29 的 State ≠ Command 是真的在发生。
 *
 * ## ★ 被夹紧的关节必须显式标出
 *
 * 后端会对超限命令做 clamp（`clamp_targets_to_limits`）。若面板只显示
 * 实际值，用户会看到"我拖到 1.6，它停在 1.5708"，然后合理地怀疑是 bug。
 * ⇒ 当 |target − actual| 明显且 target 超出模型声明的 range 时，
 *   画一个"已限位"标记。这条信息**只有前端知道**
 *   （它同时知道用户拖到哪、模型声明什么、后端回什么）。
 *
 * ## ★ 面板按 capability 显示，不按型号（§69 规则 2）
 *
 * 是否显示由 `capabilities.actuator_control` / `ik` 决定。
 * 本文件**不允许**出现机器人 id 字面量比较。
 *
 * ## 零 Three.js 依赖
 *
 * 本文件只吃 `RobotViewModel` 与两个 plain object，可以在 Node 里直接跑
 * （`jointPanel.check.ts` 需要）。它**不** import three / r3f。
 */

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { ReactNode } from "react";

import type { JointDTO, RobotCapabilitiesDTO } from "./viewer/model.ts";
import type { JointVisual } from "./viewer/viewModel.ts";

// ---------------------------------------------------------------------------
// 纯逻辑（可测）
// ---------------------------------------------------------------------------

/** 一个关节的限位：`[min, max]`。**只有两边都声明**才算有约束。 */
export type JointRange = readonly [number, number];

/**
 * 从 `JointVisual.limits` 取限位。
 *
 * ⚠️ `limits === null` ⇒ **不夹、也不给滑块设边界**。
 *
 * 给滑块设一个默认的 ±π 是"伪造一个模型没声明的约束"：
 * 用户会以为"这台机器人只能转到 π"，而模型其实没说。
 * 这与后端 `clamp_targets_to_limits` 的 `has_position_bounds()`
 * 是同一条判据（见 `backend/runtime/backend.py`），两边必须一致 ——
 * 否则前端会显示一个后端根本不存在的限位。
 */
export function rangeOf(joint: Pick<JointVisual, "limits">): JointRange | null {
  const l = joint.limits;
  if (!l) return null;
  if (typeof l.min !== "number" || typeof l.max !== "number") return null;
  if (!Number.isFinite(l.min) || !Number.isFinite(l.max)) return null;
  if (!(l.max > l.min)) return null; // 退化区间不当作约束
  return [l.min, l.max];
}

/** 可命令的关节（与 `ws.ts::commandableJoints` 同一条判据，这里按 viewModel 收）。 */
export function controllableJoints(joints: readonly JointVisual[]): readonly JointVisual[] {
  return joints.filter((j) => j.isMovable && (j.type === "revolute" || j.type === "prismatic"));
}

/**
 * 某个关节当前是否"被限位挡住了"。
 *
 * 判据：**期望值**超出了模型声明的 range，且实际值贴在边界上。
 *
 * ⚠️ 不能只写"|target − actual| > 阈值" —— 那会把"物理还在收敛"（正常的
 *    滞后）也标成限位。必须同时满足"target 真的越界"。
 *    这是"判据要描述什么不该变"的具体应用：限位标记描述的是
 *    "模型声明的约束正在生效"，而不是"两个数不一样"。
 */
export function isClamped(
  target: number,
  actual: number | undefined,
  range: JointRange | null,
  edgeTol = 1e-3
): boolean {
  if (range === null || actual === undefined) return false;
  const [lo, hi] = range;
  const outOfRange = target < lo - edgeTol || target > hi + edgeTol;
  if (!outOfRange) return false;
  const atEdge = Math.abs(actual - lo) <= edgeTol || Math.abs(actual - hi) <= edgeTol;
  return atEdge;
}

/** 关节角 → 显示用度数（面板同时给人看 rad 与 deg，避免心算）。 */
export function toDegrees(rad: number): number {
  return (rad * 180) / Math.PI;
}

/** 滑块步长：0.5° 足够细，又不会让拖动产生一堆几乎相同的命令。 */
export const SLIDER_STEP_RAD = (0.5 * Math.PI) / 180;

/**
 * 计算滑块的 min/max。
 *
 * 有声明限位 ⇒ 用它（这就是滑块**真实的**可动范围）。
 * 没声明 ⇒ 给一个"仅供拖动"的宽松窗口，并在 UI 上标注"模型未声明限位"。
 */
export function sliderBounds(range: JointRange | null): JointRange {
  if (range !== null) return range;
  // ±180°：够宽，足以表达"模型没限位"，又不会让滑块精度差到不能用
  return [-Math.PI, Math.PI];
}

// ---------------------------------------------------------------------------
// 组件
// ---------------------------------------------------------------------------

export interface JointPanelProps {
  readonly joints: readonly JointVisual[];
  /** `robot_state.joints` —— **权威**位置。 */
  readonly actual: Readonly<Record<string, number>> | null;
  /** `robot_state.status`（idle/running/reset/error）。 */
  readonly status: string | null;
  readonly capabilities: RobotCapabilitiesDTO;
  readonly connection: "connecting" | "open" | "closed";
  /** 下发一条命令。返回是否成功（socket 未开时为 false）。 */
  readonly onCommand: (targets: Readonly<Record<string, number>>) => boolean;
  /** 最近一次错误（error 帧 / 连接失败）。 */
  readonly lastError: string | null;
  /**
   * 本地预演值（ghost）。给了就显示第三列 ——
   * 让"预演与实际差多少"可见，而不是让人猜。
   */
  readonly predicted?: Readonly<Record<string, number>> | null;
}

export function JointPanel({
  joints,
  actual,
  status,
  capabilities,
  connection,
  onCommand,
  lastError,
  predicted = null,
}: JointPanelProps): ReactNode {
  const controllable = useMemo(() => controllableJoints(joints), [joints]);

  // ★ 期望值：只由用户输入改变。初始为"当前实际"，之后**不**跟着 actual 走。
  //
  //   刻意不用 `useEffect` 把 actual 同步进来 —— 那会在每次回帧时
  //   覆盖用户的拖动（症状：滑块往回弹）。
  //   唯一的例外是"用户还没碰过这个关节"：那时用 actual 作为初值更直观。
  const [targets, setTargets] = useState<Record<string, number>>({});
  const touched = useRef<Set<string>>(new Set());

  useEffect(() => {
    if (actual === null) return;
    setTargets((prev) => {
      let changed = false;
      const next = { ...prev };
      for (const j of controllable) {
        const a = actual[j.id];
        if (a === undefined) continue;
        if (next[j.id] === undefined && !touched.current.has(j.id)) {
          next[j.id] = a;
          changed = true;
        }
      }
      return changed ? next : prev;
    });
  }, [actual, controllable]);

  const send = useCallback(
    (id: string, value: number) => {
      touched.current.add(id);
      setTargets((prev) => {
        const next = { ...prev, [id]: value };
        // ★ 每次拖动都发**一条只含该关节**的命令。
        //   后端对未提及的关节保持原位（MockBackend / MuJoCoBackend 实测），
        //   所以不需要把其余关节一起发 —— 一起发会在多客户端时互相踩。
        onCommand({ [id]: value });
        return next;
      });
    },
    [onCommand]
  );

  if (!capabilities.actuator_control) {
    return (
      <div className="panel">
        <h3>关节控制</h3>
        <p className="hint">
          该机器人的 <code>capabilities.actuator_control</code> 为 false，
          因此不显示命令面板。这是 capability 驱动的（不是按型号判断）。
        </p>
      </div>
    );
  }

  if (controllable.length === 0) {
    return (
      <div className="panel">
        <h3>关节控制</h3>
        <p className="hint">模型没有可命令的关节（revolute / prismatic）。</p>
      </div>
    );
  }

  return (
    <div className="panel" data-testid="joint-panel">
      <h3>关节控制（命令 → 后端）</h3>

      <p className="hint">
        拖动即经 WebSocket 下发；下行的数字是**后端实际状态**。
        两者不一致是正常的 —— <strong>State ≠ Command</strong>。
      </p>

      <div className="joint-rows">
        {controllable.map((j) => {
          const range = rangeOf(j);
          const [lo, hi] = sliderBounds(range);
          const t = targets[j.id] ?? actual?.[j.id] ?? 0;
          const a = actual?.[j.id];
          const p = predicted?.[j.id];
          const clamped = isClamped(t, a, range);

          return (
            <div key={j.id} className="joint-row">
              <div className="joint-head">
                <code>{j.id}</code>
                {range === null && <span className="tag warn">模型未声明限位</span>}
                {clamped && <span className="tag bad">已限位</span>}
              </div>

              <input
                type="range"
                min={lo}
                max={hi}
                step={SLIDER_STEP_RAD}
                value={t}
                aria-label={`${j.id} 目标角`}
                disabled={connection !== "open"}
                onChange={(e) => send(j.id, Number(e.target.value))}
              />

              <table className="joint-nums">
                <tbody>
                  <tr>
                    <td>命令</td>
                    <td>
                      <code>{t.toFixed(4)}</code> rad
                      <span className="dim"> ({toDegrees(t).toFixed(2)}°)</span>
                    </td>
                  </tr>
                  <tr>
                    <td>实际</td>
                    <td>
                      {a === undefined ? (
                        <span className="dim">—（后端未上报）</span>
                      ) : (
                        <>
                          <code>{a.toFixed(4)}</code> rad
                          <span className="dim"> ({toDegrees(a).toFixed(2)}°)</span>
                        </>
                      )}
                    </td>
                  </tr>
                  {predicted !== null && (
                    <tr>
                      <td>预演</td>
                      <td>
                        {p === undefined ? (
                          <span className="dim">—</span>
                        ) : (
                          <>
                            <code>{p.toFixed(4)}</code> rad
                            <span className="dim"> ({toDegrees(p).toFixed(2)}°)</span>
                          </>
                        )}
                      </td>
                    </tr>
                  )}
                  {range !== null && (
                    <tr>
                      <td>限位</td>
                      <td>
                        <span className="dim">
                          {range[0].toFixed(4)} … {range[1].toFixed(4)} rad
                        </span>
                      </td>
                    </tr>
                  )}
                </tbody>
              </table>
            </div>
          );
        })}
      </div>

      <div className="joint-footer">
        <span>
          连接：<strong>{connection}</strong>
        </span>
        {status !== null && (
          <span>
            后端 status：<strong>{status}</strong>
          </span>
        )}
      </div>

      {lastError !== null && (
        <p className="bad" data-testid="joint-panel-error">
          {lastError}
        </p>
      )}
    </div>
  );
}

export { type JointDTO };
