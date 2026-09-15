/**
 * =============================================================================
 * predictor.ts —— 前端**本地预演**（ghost）层
 * -----------------------------------------------------------------------------
 * 拖动关节滑块时，本地立刻预测"机器人大概会变成什么样"，不等后端往返。
 * 这让交互跟手，而**权威姿态始终来自后端的 `robot_state`**。
 *
 * ## ⚠️ §49 硬边界（本文件存在的最大风险就是越过它）
 *
 * ```text
 * ✅ 权威姿态 = 唯一来自 WS 的 robot_state
 * ✅ 预演姿态 = ghost / 面板显示，与权威树分离
 * ❌ 禁止：把本文件算出的位姿直接写进 Three.js 的权威节点
 * ```
 *
 * 越过这条线的后果不是"画面不准"，而是**整个 Sim2Sim 验收失去意义**：
 * `State ≡ Command` ⇒ §62「FK 与 MuJoCo 一致」变成同义反复，
 * 而 MuJoCo 的限幅与积分被绕过 —— 屏幕上很听话，真机 / 仿真里不是那个位形。
 *
 * ⇒ 因此 `Prediction.source` 是一个**显式字段**：调用方必须据此决定画在哪。
 *   `reconcile()` 会在预测与权威分叉时把 source 翻成 `"authoritative"`，
 *   防止本地模型持续漂移却仍显示"我算的"。
 *
 * ## 复现 State ≠ Command
 *
 * 后端（MuJoCoBackend / MockBackend）返回的 State **不等于**发出的 Command：
 * ```text
 * ① 限幅：命令超关节 range ⇒ 被夹紧（clamp_targets_to_limits）
 * ② 滞后：物理积分需要时间 ⇒ 一步到位不了（MuJoCo 600 子步 × 0.002s）
 * ```
 * 预演不复现这两点的话，画面会"瞬间跳到位"，而后端回帧时又跳回去 ——
 * 表现为**肉眼可见的抖动**，且让人误以为后端有延迟 bug。
 *
 * ## 限位从哪来
 *
 * 从 `RobotModel.joints[].limits` **读**，不在本文件硬编码。
 * 硬编码会让"给机器人改限位"变成"改两处"，而漏改的那处是静默的。
 * `limits === null` 或只声明了一边 ⇒ **不夹**（不伪造模型没声明的约束）。
 *
 * ## 零 Three.js 依赖
 *
 * 本文件是纯逻辑，可以在 Node 里直接跑（自检脚本需要）。
 * 坐标转换只在 `coordinateAdapter.ts` 里发生（§41）。
 */

import type { JointDTO, JointLimitsDTO, RobotModelDTO } from "../viewer/model";
// `include: ["src"]` 覆盖不到包目录 ⇒ 用三重斜杠引用把包内的类型声明拉进来。
// 这是**编译期**引用，不进运行时、不影响 Vite 打包（详见 kinematics.d.ts 文件头）。
/// <reference path="../../../packages/mini_arm/kinematics/kinematics.d.ts" />
import {
  forwardKinematics,
  solveIk,
  type Pose,
} from "../../../packages/mini_arm/kinematics/kinematics.js";

/**
 * 预演的来源标记。
 *
 * `"predicted"` —— 本地算的，可以当 ghost 画。
 * `"authoritative"` —— 后端值 / 预测被判定不可信，**必须**用它画权威姿态。
 */
export type PredictionSource = "predicted" | "authoritative";

export interface Prediction {
  /** 预测 / 采用的关节角（rad）。 */
  readonly jointPositions: Readonly<Record<string, number>>;
  /** 由上面的关节角算出的 TCP 位姿（RobotForge 约定，未做 Three.js 转换）。 */
  readonly tcp: Pose;
  /** 这个位姿能不能当权威用 —— 调用方必须据此决定画在哪。 */
  readonly source: PredictionSource;
  /** 与权威态的偏差（rad，各关节绝对差的最大值）。无权威值时 = 0。 */
  readonly desync: number;
  /** 上一次 reconcile 是否丢弃了预测（供 UI 显示"已同步到后端"）。 */
  readonly wasCorrected: boolean;
}

export interface PredictorOptions {
  /**
   * 一阶滞后的时间常数（秒）。
   *
   * ## 这个值是怎么定的（不是拍脑袋）
   *
   * 太小（→0）⇒ 预演瞬间到位，与后端的差异全落在"后端慢"上，
   * 拖滑块时界面自己先跳、松手后不回弹（因为预测已经到位），
   * 用户看不到任何"正在跟随"的反馈。
   *
   * 太大 ⇒ 预演肉眼看不出跟手，失去预演的意义。
   *
   * 取 0.12 s：约 3-4 帧（60fps 下 0.2 s ≈ 12 帧）内走完 95%，
   * 既能看出"在跟"，又不会拖沓。
   */
  readonly tau?: number;
  /**
   * 预测与权威的偏差超过它 ⇒ 丢弃预测、改用权威（rad）。
   *
   * 定 0.15 rad ≈ 8.6°：小于它的差异属于"物理还在收敛"的正常范围；
   * 大于它说明本地模型与真实物理**结构性不一致**（改了参数没同步、
   * 或后端换了引擎），此时继续显示预测值就是在骗人。
   */
  readonly resyncTolerance?: number;
  /**
   * 单步最大位移（rad）。
   *
   * 与 `backend/runtime/backend.py` 的 `MockBackend.DEFAULT_MAX_STEP = 0.35`
   * 取同一个量级。它**不是**在复现 MuJoCo 的积分（那需要完整动力学），
   * 而是保证"预演不会比后端更激进" —— 两者数量级一致时，
   * 回帧的视觉落差最小。
   */
  readonly maxStepPerTick?: number;
}

const DEFAULT_TAU = 0.12;
const DEFAULT_RESYNC_TOL = 0.15;
const DEFAULT_MAX_STEP = 0.35;

/** 一阶滞后的平滑系数：`alpha = 1 − exp(−dt/τ)`。 */
export function smoothingAlpha(dt: number, tau: number): number {
  if (!(tau > 0)) return 1; // tau ≤ 0 ⇒ 无滞后（显式退化为瞬时到达）
  if (!(dt > 0)) return 0; // dt ≤ 0 ⇒ 不动（不前进也不倒退）
  return 1 - Math.exp(-dt / tau);
}

/**
 * 从模型的关节限位构造 `{jointId: [min, max] | null}`。
 *
 * **只有两边都声明了**才产生约束。理由：
 * ```text
 * position_min = -1.0, position_max = null  ⇒ 这是一个单向限位
 * ```
 * 那种情况下"夹到某个默认的 ±π"是在**伪造一个模型没声明的约束**，
 * 而伪造的约束会让预演与后端分叉（后端按真实语义处理）。
 * 与 `backend/simulation/simulation_backend.py::clamp_targets_to_limits`
 * 的 `has_position_bounds()` 判断保持一致。
 */
export function limitsFromModel(
  model: Pick<RobotModelDTO, "joints">
): Record<string, readonly [number, number] | null> {
  const out: Record<string, readonly [number, number] | null> = {};
  for (const joint of model.joints) {
    out[joint.id] = declaredRange(joint);
  }
  return out;
}

function declaredRange(joint: JointDTO): readonly [number, number] | null {
  const limits: JointLimitsDTO | null = joint.limits;
  if (limits === null) return null;
  const { position_min: lo, position_max: hi } = limits;
  if (lo === null || hi === null) return null;
  return [lo, hi];
}

/** 夹紧一个关节角到声明范围（未声明 ⇒ 原样返回）。 */
export function clampJoint(
  value: number,
  range: readonly [number, number] | null
): { value: number; clamped: boolean } {
  if (range === null) return { value, clamped: false };
  const [lo, hi] = range;
  if (value < lo) return { value: lo, clamped: true };
  if (value > hi) return { value: hi, clamped: true };
  return { value, clamped: false };
}

/**
 * 创建初始预测（零位形）。
 *
 * 为什么不是"等后端第一帧"：Canvas 要在拿到状态前就能画。
 * 而且零位形是模型定义的合法位形，不是"猜的"。
 */
export function initialPrediction(
  model: RobotModelDTO,
  jointIds: readonly string[]
): Prediction {
  const positions: Record<string, number> = {};
  for (const id of jointIds) positions[id] = 0;
  return {
    jointPositions: Object.freeze(positions),
    tcp: safeTcp(model, positions),
    source: "predicted",
    desync: 0,
    wasCorrected: false,
  };
}

/**
 * 推进一步：把新命令经**限幅 + 一阶滞后**转成新的预测。
 *
 * ```text
 * target  = clamp(command[cmdJoint], range)         ← 复现后端限幅
 * next[j] = current[j] + (target − current[j]) · α  ← 复现"到不了位"
 * next[j] = clamp(next[j], range)                   ← 滞后过程也不许出界
 * ```
 *
 * ⚠️ **未在命令里出现的关节保持原位**（不当作 0）。
 *    后端的语义是"只改被下发的关节"（见 `mujoco_backend.send_command`），
 *    若这里当成 0，拖一个滑块会让其它关节全部归零 —— 一个很显眼的错。
 *
 * @param model      用于算 TCP（取轴）与读限位
 * @param current    上一次的预测
 * @param command    `{jointId: 目标角}`，**只含被下发的关节**
 * @param dt         步进时长（秒）
 */
export function advance(
  model: RobotModelDTO,
  current: Prediction,
  command: Readonly<Record<string, number>>,
  dt: number,
  options: PredictorOptions = {}
): Prediction {
  const tau = options.tau ?? DEFAULT_TAU;
  const maxStep = options.maxStepPerTick ?? DEFAULT_MAX_STEP;
  const alpha = smoothingAlpha(dt, tau);

  const ranges = limitsFromModel(model);
  const next: Record<string, number> = { ...current.jointPositions };

  for (const [jointId, rawTarget] of Object.entries(command)) {
    // 命令里出现但模型没有的关节 ⇒ 忽略（不抛）。
    // 理由与 Core FK 的"多余 joint_id 被忽略"同源：
    // 换机器人时上层可能持有上一个模型的命令，报错会让换机器人变脆弱。
    if (!(jointId in next)) continue;
    if (!Number.isFinite(rawTarget)) continue; // NaN/Inf 会让整条链条静默污染

    const range = ranges[jointId] ?? null;
    const clampedTarget = clampJoint(rawTarget, range).value;

    const cur = next[jointId]!;
    const step = (clampedTarget - cur) * alpha;
    // 单步位移上限：与后端 MockBackend 的量级对齐
    const limited = Math.max(-maxStep, Math.min(maxStep, step));
    // 滞后过程也不许越界（限幅后仍可能因 α<1 停在界内，这里是防御性一致）
    //
    // ⚠️ 本函数有**两处**夹紧（一处夹目标、一处夹结果）。
    //    这是刻意的纵深防御，但有一个副作用值得知道：
    //    **只去掉一处，行为不会变** —— 反例注射必须两处都去掉才能变红。
    //    （实测：只去掉上面那处时全套断言仍然 PASS，说明单点缺陷会被另一处吸收。
    //     这意味着"限幅被误删"这类回归不会立刻暴露，判据本身是对的，
    //     但注射脚本必须知道这件事，否则会误判成"判据失效"。）
    next[jointId] = clampJoint(cur + limited, range).value;
  }

  return {
    jointPositions: Object.freeze(next),
    tcp: safeTcp(model, next),
    source: current.source,
    desync: current.desync,
    wasCorrected: false,
  };
}

/**
 * 与权威态对账：决定"采用后端值"还是"保留预测"。
 *
 * ## 判据
 *
 * ```text
 * desync = max_j |predicted[j] − authoritative[j]|   （只看两者都有的关节）
 * desync > tolerance ⇒ 丢弃预测，jointPositions 换成权威值，source = "authoritative"
 * 否则               ⇒ 保留预测，source = "predicted"，只记录 desync
 * ```
 *
 * ## ⚠️ 角度差必须走最短弧
 *
 * 关节角是周期量：`−π` 与 `+π` 是**同一个位形**，但直接相减得 2π。
 * 若不做环绕处理，关节跨过 ±π 时会被判成"偏差 6.28 rad ⇒ 严重分叉"，
 * 于是每次过界都触发一次重同步 —— 表现为**在 π 附近持续抖动**。
 *
 * 这是本文件里最容易漏的一条，也是"看起来像后端抖动"的最可能来源。
 */
export function reconcile(
  model: RobotModelDTO,
  predicted: Prediction,
  authoritative: Readonly<Record<string, number>>,
  options: PredictorOptions = {}
): Prediction {
  const tolerance = options.resyncTolerance ?? DEFAULT_RESYNC_TOL;

  let worst = 0;
  let compared = 0;
  for (const [jointId, authValue] of Object.entries(authoritative)) {
    const predValue = predicted.jointPositions[jointId];
    if (predValue === undefined) continue; // 模型里没有这个关节 ⇒ 不参与
    if (!Number.isFinite(authValue)) continue;
    worst = Math.max(worst, shortestAngleDiff(predValue, authValue));
    compared += 1;
  }

  // 没有任何可比关节（例如权威帧是空的）⇒ 不判定分叉，
  // 保留预测。否则会在启动第一帧就把预测丢掉。
  if (compared === 0) {
    return { ...predicted, wasCorrected: false };
  }

  if (worst > tolerance) {
    return {
      jointPositions: Object.freeze({ ...authoritative }),
      tcp: safeTcp(model, authoritative),
      source: "authoritative",
      desync: worst,
      wasCorrected: true,
    };
  }

  return { ...predicted, desync: worst, wasCorrected: false };
}

/**
 * 两个角度之间的**最短弧**距离（rad），结果落在 `[0, π]`。
 *
 * ```text
 * a =  3.0, b = −3.0  ⇒  直接相减得 6.0，但实际只差 0.283 rad
 * ```
 * 不处理会让关节跨 ±π 时被误判为"严重分叉"（见 `reconcile` 的说明）。
 */
export function shortestAngleDiff(a: number, b: number): number {
  const twoPi = 2 * Math.PI;
  let d = (a - b) % twoPi;
  if (d > Math.PI) d -= twoPi;
  if (d < -Math.PI) d += twoPi;
  return Math.abs(d);
}

/**
 * 算 TCP，但**不抛异常**。
 *
 * 预演是"尽力而为"的显示层：模型里缺 axis / 缺 site 时应该退化成
 * "TCP 不可用"，而不是让整个界面崩掉。与 `viewModel.ts::buildViewModel`
 * "问题进 problems 而不是抛"的设计一致。
 */
function safeTcp(model: RobotModelDTO, positions: Readonly<Record<string, number>>): Pose {
  try {
    return forwardKinematics(model, positions);
  } catch {
    // TCP 不可用：给一个 NaN 位姿，调用方据 source / 有限性判断是否显示
    return { position: [NaN, NaN, NaN], orientation: [0, 0, 0, 1] };
  }
}

/** TCP 是否可用（`safeTcp` 失败时是 NaN）。 */
export function tcpIsFinite(tcp: Pose): boolean {
  return (
    Number.isFinite(tcp.position[0]) &&
    Number.isFinite(tcp.position[1]) &&
    Number.isFinite(tcp.position[2])
  );
}

/**
 * 逆解：把"目标 TCP 位置"换成关节命令。
 *
 * ★ 返回值**仍然是命令**，要经 WS 发给后端 —— 本函数**不**直接改场景。
 *   这是 §49 的落地方式："前端算出的东西"只有经过
 *   `joint_command → Runtime → Backend → robot_state` 才能影响权威姿态。
 *
 * 失败时返回 `null`，并把原因写进 `reason`（供 UI 提示），
 * 而不是抛异常或返回一个假的解。
 */
export function commandFromTcpTarget(
  model: RobotModelDTO,
  target: { position: readonly [number, number, number] },
  options: { branch?: "elbow_up" | "elbow_down"; clamp?: boolean } = {}
): { command: Record<string, number> | null; reason: string | null } {
  const branch = options.branch ?? "elbow_up";
  const clamp = options.clamp ?? false;
  try {
    const sol = solveIk(
      model,
      {
        position: target.position,
        orientation: [0, 0, 0, 1],
      },
      { branch, clamp }
    );
    if (!Number.isFinite(sol.positionError) || sol.positionError > 1e-3) {
      return { command: null, reason: `逆解位置误差过大（${sol.positionError.toFixed(6)} m）` };
    }
    return { command: { ...sol.jointPositions }, reason: null };
  } catch (exc) {
    const message = exc instanceof Error ? exc.message : String(exc);
    return { command: null, reason: message };
  }
}
